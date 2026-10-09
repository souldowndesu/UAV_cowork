"""Select a physical black-drone preset and run causal pursuit in AirSim."""
from __future__ import annotations
import argparse
from collections import deque
from dataclasses import asdict,replace
import json
import hashlib
from pathlib import Path
import sys
import time
import traceback
import numpy as np

if __package__ in (None,''):sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from navigation.config import Config,parse_overrides
from navigation.profiles import profile_overrides
from navigation.paths import PROJECT_ROOT,RECORDS,SHARED_MAP,project_path
from navigation.chase import ChaseOptions,TargetTracker
from navigation.airsim_io import AirSimIO
from navigation.safe_navigation import SafeNavigationPipeline
from experiments.chase_target import WorldVehicle,TargetFlight,vehicle_offsets
from experiments.recorder import Recorder,atomic_json
from experiments.experiment import file_hash,run_metadata
from experiments.web_server import VizServer

PRESETS=PROJECT_ROOT/'experiments'/'scenarios'/'chase'


def presets():
    return {p.stem:json.loads(p.read_text(encoding='utf-8')) for p in sorted(PRESETS.glob('*.json'))}


def prepare_vehicles(cfg,preset,args,metadata,out):
    import airsim
    p0=np.asarray(preset['target_path'][0],dtype=float)
    client=airsim.MultirotorClient(ip=cfg.ip,port=cfg.port,timeout_value=4)
    if not client.ping():raise RuntimeError('Simulator RPC unavailable')
    vehicles=client.listVehicles()
    if args.vehicle not in vehicles or args.target_vehicle not in vehicles:
        raise RuntimeError('Need '+args.vehicle+' and '+args.target_vehicle+'; start with environment/settings_chase.json')
    metadata['actual_settings_sha256']=hashlib.sha256(client.getSettingsString().encode('utf-8')).hexdigest()
    client.simPause(False)
    for name in (args.vehicle,args.target_vehicle):client.cancelLastTask(vehicle_name=name)
    client.reset();client.simPause(True);client.simFlushPersistentMarkers()
    offset=vehicle_offsets(client,args.vehicle,args.target_vehicle)
    pursuer=AirSimIO(cfg)
    target_io=WorldVehicle(AirSimIO(replace(cfg,vehicle_name=args.target_vehicle)),offset)
    p0=np.asarray(preset['target_path'][0],dtype=float)
    # Model-only teleport leaves stale multirotor kinematics in this build.
    # Initialize the two physics states once, then fly using physical controls.
    client.simPause(True)
    for name,position in ((args.vehicle,[preset['start'][0],preset['start'][1],-cfg.flight_altitude]),
                          (args.target_vehicle,p0-offset)):
        client.enableApiControl(True,vehicle_name=name)
        if not client.armDisarm(True,vehicle_name=name):raise RuntimeError('Cannot arm '+name)
        k=airsim.KinematicsState();k.position=airsim.Vector3r(*map(float,position))
        yaw=np.deg2rad(preset.get('start_yaw',0.)) if name==args.vehicle else 0.
        k.orientation=airsim.to_quaternion(0.,0.,float(yaw))
        for field in ('linear_velocity','angular_velocity','linear_acceleration','angular_acceleration'):
            setattr(k,field,airsim.Vector3r(0.,0.,0.))
        client.simSetKinematics(k,True,vehicle_name=name)
        client.simSetVehiclePose(airsim.Pose(k.position,k.orientation),True,vehicle_name=name)
        client.simGetVehiclePose(vehicle_name=name)
    client.simPause(False)
    for name in (args.vehicle,args.target_vehicle):client.moveByVelocityAsync(0.,0.,0.,2.,vehicle_name=name)
    time.sleep(1.)
    pstart=pursuer.get_state();tstart=target_io.get_state()
    start_error=float(np.linalg.norm(tstart.position-p0))
    pursuer_error=float(np.linalg.norm(pstart.position-np.array([preset['start'][0],preset['start'][1],-cfg.flight_altitude])))
    print('[ChaseStart]',json.dumps(dict(offset=offset.tolist(),requested=p0.tolist(),actual=tstart.position.tolist(),error=start_error)),flush=True)
    metadata.update(preflight_pursuer=pstart.position.tolist(),preflight_target=tstart.position.tolist(),target_offset=offset.tolist(),
                    target_start_error=start_error,pursuer_start_error=pursuer_error,benchmark_valid=max(start_error,pursuer_error)<1.5,
                    initialization='paused physics-state initialization, followed by physical velocity flight')
    atomic_json(out/'run_meta.json',metadata)
    if max(start_error,pursuer_error)>=1.5 or tstart.collision or pstart.collision:
        pursuer.land();target_io.land()
        raise RuntimeError('Invalid actual preflight position / collision; benchmark refused')
    return pursuer,target_io


def run_chase(preset,args,base_config):
    import airsim
    options=ChaseOptions(strategy=args.strategy,lead_max=args.lead_max,stand_off=args.stand_off,
                         vertical_offset=args.vertical_offset,capture_radius=args.capture_radius,
                         capture_hold=args.capture_hold,replan_interval=args.replan_interval,
                         stale_after=max(1.5,args.delay+.5),stand_off_mode=args.stand_off_mode)
    cfg=replace(base_config,vehicle_name=args.vehicle,navigation_mode='safe_route',
                use_persistent_map=True,
                allow_safe_approach=False,goal_occupied_confirmed=False,goal_in_building_confirmed=False,
                load_persistent_map=args.load_map,persistent_seed_path='',persistent_accumulate=True,
                persistent_map_path=str(SHARED_MAP),flight_altitude=preset.get('flight_altitude',10.))
    batch=time.strftime('%Y%m%d_%H%M%S')+f'_{time.time_ns()%1000000000:09d}'
    out=project_path(args.output_dir)/preset['name']/args.strategy/batch;out.mkdir(parents=True)
    context=dict(mission_type='chase',preset=preset['name'],strategy=args.strategy,target_vehicle=args.target_vehicle,
                 target_speed=args.target_speed or preset['target_speed'],delay=args.delay,
                 chase_options=asdict(options),shared_map=str(SHARED_MAP),load_map=args.load_map)
    context['capture_alignment']='actual target history interpolation; extrapolation limited to 0.15 sim seconds'
    metadata=run_metadata(cfg,preset['start'],preset['target_path'][0],preset.get('start_yaw',0.),
                          dict(path=str(SHARED_MAP),initial_sha256=file_hash(SHARED_MAP),loads_history=args.load_map),preset)
    metadata.update(context,outcome='preparing',initial_state_note='Reset and initialize both physics states while paused; start from verified airborne poses, then use physical velocity controls.')
    atomic_json(out/'run_meta.json',metadata)
    try:
        pursuer,target_io=prepare_vehicles(cfg,preset,args,metadata,out)
    except (Exception,KeyboardInterrupt) as exc:
        metadata.update(outcome='interrupted' if isinstance(exc,KeyboardInterrupt) else 'error',captured=False,benchmark_valid=False,error=str(exc),run_dir=str(out))
        atomic_json(out/'run_meta.json',metadata)
        atomic_json(out/'chase_summary.json',dict(context,outcome=metadata['outcome'],captured=False,benchmark_valid=False,error=str(exc),run_dir=str(out)))
        try:airsim.MultirotorClient(ip=cfg.ip,port=cfg.port,timeout_value=2).simPause(False)
        except Exception:pass
        print('[Chase] preparation failed:',exc,flush=True)
        return metadata
    p0=np.asarray(preset['target_path'][0],dtype=float)
    rec=Recorder(map_size=cfg.map_size);rec.set_goal(p0)
    rec.start(out/'recording.json',dict(cfg.to_dict(),**context))
    pipe=SafeNavigationPipeline(pursuer,cfg);pipe.recorder=rec;pipe.set_goal(p0)
    pipe.update_moving_target(None,options,time.time())
    tracker=TargetTracker(options);target=TargetFlight(target_io,preset,args.target_speed)
    viz=VizServer(recorder=rec,port=args.viz_port) if args.visualize else None
    queue=deque();samples=[];capture_since=None;first_close=None;last_stamp=None
    outcome='error';started=time.monotonic();origin_ns=target_io.get_state().timestamp_ns
    duration_limit=args.max_duration or preset.get('max_duration',100.)
    try:
        target.start();pipe.start()
        if viz:viz.start();print('[Chase] Live',viz.url,flush=True)
        while True:
            elapsed=time.monotonic()-started
            if elapsed>=duration_limit:outcome='timed_out';break
            truth=target.state;pstate=pipe._latest_state
            if target.error:
                outcome='collided' if pipe.collided or (pstate is not None and pstate.collision) else 'target_invalid'
                break
            if truth is None or pstate is None:time.sleep(.05);continue
            sim=(pstate.timestamp_ns-origin_ns)*1e-9;wall=time.time()
            if truth.timestamp_ns!=last_stamp:
                queue.append(truth);last_stamp=truth.timestamp_ns
            observation=None
            while queue and (pstate.timestamp_ns-queue[0].timestamp_ns)*1e-9>=args.delay:
                observation=queue.popleft()
            if observation is not None:
                tracker.observe(observation.position,observation.velocity,(observation.timestamp_ns-origin_ns)*1e-9)
            prediction=tracker.predict(pstate.position,sim,cfg.v_mission)
            pipe.update_moving_target(prediction,options,wall)
            skew=(pstate.timestamp_ns-truth.timestamp_ns)*1e-9
            target_position=target.position_at(pstate.timestamp_ns)
            time_valid=target_position is not None
            if not time_valid:target_position=truth.position
            distance=float(np.linalg.norm(pstate.position-target_position))
            body=dict(seconds=sim,wall_seconds=elapsed,distance=distance,
                      pursuer=pstate.position.tolist(),target=target_position.tolist(),raw_target=truth.position.tolist(),
                      time_skew_sim_s=skew,capture_time_valid=time_valid,
                      target_speed=float(np.linalg.norm(truth.velocity)),
                      target_distance=target.distance,lead=prediction['lead'] if prediction else None,
                      age=prediction['age'] if prediction else None,
                      predicted_goal=prediction['goal'].tolist() if prediction else None,
                      pursuer_collision=pstate.collision,target_collision=truth.collision)
            samples.append(body)
            rec.record_target(wall,truth,lead=body['lead'],age=body['age'],predicted_goal=body['predicted_goal'],
                              observed_p=tracker.position.tolist() if tracker.position is not None else None)
            if pstate.collision or pipe.collided:outcome='collided';break
            if truth.collision:outcome='target_invalid';break
            if time_valid and distance<options.capture_radius and target.distance>15.:
                if first_close is None:first_close=sim
                if capture_since is None:capture_since=sim
                if sim-capture_since>=options.capture_hold:outcome='captured';break
            else:capture_since=None
            if len(samples)%20==1:
                print(f'[Chase] {preset["name"]} {args.strategy} t={sim:.1f}s d={distance:.1f}m target_v={body["target_speed"]:.1f} lead={body["lead"]} plans={len(rec.plans)}',flush=True)
            time.sleep(.1)
    except KeyboardInterrupt:outcome='interrupted'
    except Exception as exc:
        context['error']=str(exc);outcome='error';traceback.print_exc()
    finally:
        cleanup_errors=[]
        for label,stop in (('pursuer',pipe.stop),('target',target.stop)):
            try:stop()
            except Exception as exc:cleanup_errors.append(label+': '+str(exc))
        telemetry=pipe.telemetry()
        atomic_json(out/'telemetry.json',dict(records=telemetry,goal=pipe.goal.tolist(),outcome=outcome))
        try:
            if pipe.persistent is not None:rec.set_persistent(pipe.persistent.occupied_points(stride=2))
            rec.record_event('outcome',dict(t=time.time(),outcome=outcome));rec.close()
        except Exception as exc:
            context['recording_error']=str(exc);outcome='error'
        for label,land in (('pursuer land',pursuer.land),('target land',target_io.land)):
            try:land()
            except Exception as exc:cleanup_errors.append(label+': '+str(exc))
        if viz:
            try:viz.stop()
            except Exception as exc:cleanup_errors.append('live view: '+str(exc))
        atomic_json(out/'chase_telemetry.json',dict(context=context,samples=samples))
        stats=dict(outcome=outcome,captured=outcome=='captured',samples=len(samples),
                   duration_wall_s=time.monotonic()-started,capture_sim_seconds=samples[-1]['seconds'] if outcome=='captured' else None,
                   first_close_seconds=first_close,min_distance=min((s['distance'] for s in samples),default=None),
                   mean_distance=float(np.mean([s['distance'] for s in samples])) if samples else None,
                   target_distance=target.distance,target_legs=target.legs,target_error=target.error,
                   benchmark_valid=outcome not in ('error','target_invalid','interrupted'),
                   plans=rec.meta()['n_plans'],cleanup_errors=cleanup_errors,
                   map_saved=pipe._persist_save_error is None,persistent_save_error=pipe._persist_save_error,
                   pursuer_distance=float(np.linalg.norm(np.diff([s['pursuer'] for s in samples],axis=0),axis=1).sum()) if len(samples)>1 else 0.,
                   pursuer_collision=bool(pipe.collided),target_collision=bool(target.state and target.state.collision),
                   run_dir=str(out),**context)
        metadata.update(stats);atomic_json(out/'run_meta.json',metadata);atomic_json(out/'chase_summary.json',stats)
        print('[Chase] RESULT',json.dumps(stats,ensure_ascii=False),flush=True)
    return stats


def main(argv=None):
    available=presets()
    parser=argparse.ArgumentParser(description='真实双机追击：预设目标轨迹、预测追击与统一地图')
    parser.add_argument('--preset',choices=tuple(available)+('all',))
    parser.add_argument('--list',action='store_true')
    parser.add_argument('--strategy',choices=('pure','latency','predict','all'),default='predict')
    parser.add_argument('--profile',choices=('safe5','safe8'),default='safe5')
    parser.add_argument('--runs',type=int,default=1)
    parser.add_argument('--delay',type=float,default=.35)
    parser.add_argument('--lead-max',type=float,default=3.)
    parser.add_argument('--replan-interval',type=float,default=1.2)
    parser.add_argument('--stand-off',type=float,default=3.5)
    parser.add_argument('--stand-off-mode',choices=('trailing','intercept'),default='intercept')
    parser.add_argument('--vertical-offset',type=float,default=2.)
    parser.add_argument('--capture-radius',type=float,default=5.)
    parser.add_argument('--capture-hold',type=float,default=2.)
    parser.add_argument('--target-speed',type=float)
    parser.add_argument('--max-duration',type=float)
    parser.add_argument('--vehicle',default='Drone1');parser.add_argument('--target-vehicle',default='BlackDrone')
    parser.add_argument('--load-map',action=argparse.BooleanOptionalAction,default=False)
    parser.add_argument('--output-dir',default=str(RECORDS/'chase_runs'))
    parser.add_argument('--visualize',action='store_true');parser.add_argument('--viz-port',type=int,default=8767)
    args,rest=parser.parse_known_args(argv)
    for field in ('delay','lead_max','stand_off','vertical_offset','capture_radius','capture_hold','replan_interval'):
        if not np.isfinite(getattr(args,field)) or getattr(args,field)<0:parser.error('Invalid '+field)
    if args.runs<1 or args.capture_radius<=0 or args.capture_hold<=0:parser.error('Invalid run / capture settings')
    for field in ('target_speed','max_duration'):
        value=getattr(args,field)
        if value is not None and (not np.isfinite(value) or value<=0):parser.error('Invalid '+field)
    try:
        overrides=profile_overrides(args.profile);overrides.update(parse_overrides(rest))
        cfg=Config.from_overrides(overrides)
    except (ValueError,TypeError,ZeroDivisionError,OverflowError) as exc:parser.error(str(exc))
    if not np.isfinite([cfg.v_mission,cfg.v_max]).all() or not 0<cfg.v_mission<=cfg.v_max:
        parser.error('Cruise speed must be positive and no greater than v_max')
    if args.vehicle==args.target_vehicle:parser.error('Pursuer and target must be different vehicles')
    for field in ('a_max','brake_accel','res','plan_res','sensor_hz','mapping_hz','planning_hz','control_hz','map_stale_timeout'):
        if not np.isfinite(getattr(cfg,field)) or getattr(cfg,field)<=0:parser.error('Must be positive and finite: '+field)
    if args.replan_interval<=0:parser.error('replan-interval must be positive')
    if np.hypot(args.stand_off,args.vertical_offset)>=args.capture_radius:
        parser.error('capture-radius must exceed the configured horizontal and vertical standoff')
    if args.list:
        for i,(name,case) in enumerate(available.items(),1):print(f'{i}. {name}: {case["label"]} ({case["target_speed"]} m/s)')
        return 0
    choice=args.preset
    if choice is None:
        for i,(name,case) in enumerate(available.items(),1):print(f'{i}. {name}: {case["label"]}')
        value=input('选择目标案例编号或名称（q 退出）：').strip()
        if value.lower()=='q':return 0
        choice=list(available)[int(value)-1] if value.isdigit() and 1<=int(value)<=len(available) else value
        if choice not in available:parser.error('Unknown preset')
    results=[]
    strategies=('pure','latency','predict') if args.strategy=='all' else (args.strategy,)
    batch_dir=project_path(args.output_dir)/'batches';batch_dir.mkdir(parents=True,exist_ok=True)
    report_path=batch_dir/(time.strftime('%Y%m%d_%H%M%S')+'.json')
    for name in available if choice=='all' else (choice,):
        for strategy in strategies:
            case_args=argparse.Namespace(**dict(vars(args),strategy=strategy))
            for _ in range(args.runs):
                results.append(run_chase(available[name],case_args,cfg))
                atomic_json(report_path,dict(arguments=vars(args),results=results))
                if results[-1]['outcome']=='interrupted':return 130
                time.sleep(1.)
    print('[Chase] Batch report:',report_path,flush=True)
    return 0 if all(r['captured'] for r in results) else 1

if __name__=='__main__':raise SystemExit(main())
