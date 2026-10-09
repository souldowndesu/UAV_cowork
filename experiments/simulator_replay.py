"""Render recorded motion and contemporaneous predictions in an AirSim world.

RPC calls belong to one worker thread. HTTP/UI updates contain a recording ID,
relative time and camera offsets; motion/path data come from the recording.
"""
from __future__ import annotations

from bisect import bisect_right
import json
import math
from pathlib import Path
import threading
import time

import numpy as np


def points(value):
    result = np.asarray(value, dtype=float).reshape(-1, 3)
    if not np.isfinite(result).all(): raise ValueError('Non-finite replay coordinates')
    return result.tolist()


def angle(a, b, fraction):
    return a + math.atan2(math.sin(b-a), math.cos(b-a))*fraction


def camera_controls(value=None):
    """Bound orbit controls; clients cannot supply arbitrary vehicle poses."""
    if value is None:value={}
    if not isinstance(value,dict) or set(value)-{'azimuth','elevation','zoom','pan'}:
        raise ValueError('Invalid camera controls')
    def number(v,low,high):
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not low<=v<=high:
            raise ValueError('Camera control is outside its finite range')
        return float(v)
    pan=value.get('pan',[0.,0.])
    if not isinstance(pan,(list,tuple)) or len(pan)!=2:raise ValueError('Invalid camera pan')
    return dict(azimuth=number(value.get('azimuth',0.),-math.pi,math.pi),
                elevation=number(value.get('elevation',0.),-math.pi,math.pi),
                zoom=number(value.get('zoom',1.),.15,20.),pan=[number(v,-10.,10.) for v in pan])


class ReplayTimeline:
    def __init__(self, data, telemetry=None):
        self.states = sorted((dict(s) for s in data.get('states', [])), key=lambda s:s['t'])
        if not self.states: raise ValueError('录像没有可播放的状态')
        self.wall = np.array([s['t'] for s in self.states], dtype=float)
        self.positions = points([s['p'] for s in self.states])
        if not np.isfinite(self.wall).all() or np.any(np.diff(self.wall)<=0):
            raise ValueError('录像状态时间必须严格递增')
        stamps = [s.get('sim_timestamp_ns') for s in self.states]
        self.sim_clock = (len(stamps)>1 and all(isinstance(s,(int,float)) and math.isfinite(s) for s in stamps)
                          and all(b>=a for a,b in zip(stamps,stamps[1:])) and stamps[-1]>stamps[0])
        self.times = np.array([(s-stamps[0])*1e-9 for s in stamps]) if self.sim_clock else self.wall-self.wall[0]
        self.duration = float(self.times[-1])
        # Older v2 states contain yaw only, while their full telemetry preserves
        # all three angles. Fill only missing fields from the matching timestamp.
        telemetry = sorted((telemetry or {}).get('records', []),key=lambda r:r['t'])
        telemetry_times = [r['t'] for r in telemetry]
        for s in self.states:
            i = bisect_right(telemetry_times,s['t'])
            candidates = telemetry[max(0,i-1):i+1]
            nearby = min(candidates,key=lambda r:abs(r['t']-s['t'])) if candidates else {}
            for field in ('pitch','roll','yaw'):
                if field not in s and nearby and abs(nearby['t']-s['t'])<.5:
                    s[field] = nearby.get(field,0.)
                s[field] = float(s.get(field,0.))
                if not math.isfinite(s[field]): raise ValueError('Non-finite replay attitude')
        self.plans = sorted(data.get('plans',[]),key=lambda p:p['t'])
        self.plan_times = [self.convert(p['t']) for p in self.plans]
        self.goal = points([data['goal']])[0] if data.get('goal') is not None else None
        all_positions = np.asarray(self.positions + ([self.goal] if self.goal else []))
        self.center = ((all_positions.min(axis=0)+all_positions.max(axis=0))*.5).tolist()
        self.span = float(np.max(np.ptp(all_positions,axis=0)))

    def convert(self, wall_time):
        if len(self.wall)==1:return float(wall_time-self.wall[0])
        i = min(max(bisect_right(self.wall,wall_time)-1,0),len(self.wall)-2)
        fraction = (wall_time-self.wall[i])/(self.wall[i+1]-self.wall[i])
        return float(self.times[i]+(self.times[i+1]-self.times[i])*fraction)

    def sample(self, seconds):
        seconds = float(np.clip(seconds,0.,self.duration))
        i = min(max(bisect_right(self.times,seconds)-1,0),len(self.states)-1)
        j = min(i+1,len(self.states)-1)
        fraction = (seconds-self.times[i])/max(1e-9,self.times[j]-self.times[i]) if j!=i else 0.
        a,b = self.states[i],self.states[j]
        position = ((1-fraction)*np.asarray(self.positions[i])+fraction*np.asarray(self.positions[j])).tolist()
        attitude = {k:angle(a[k],b[k],fraction) for k in ('pitch','roll','yaw')}
        # Only the already flown prefix is blue. Never expose a future plan.
        history = self.positions[:i+1] + [position]
        if len(history)>1500: history = [history[k] for k in np.linspace(0,len(history)-1,1500,dtype=int)]
        plan_index = bisect_right(self.plan_times,seconds)-1
        plan = self.plans[plan_index] if plan_index>=0 else {}
        return dict(seconds=seconds,position=position,attitude=attitude,history=history,
                    path=points(plan.get('path',[])),trajectory=points(plan.get('traj',[])),
                    plan_index=plan_index,goal=plan.get('goal',self.goal))

    def camera(self, sample, view, controls=None):
        p = np.asarray(sample['position'])
        if view=='overview':
            target = np.asarray(self.center)
            distance = max(40.,self.span*.8)
            camera = target + [-distance,-distance,-distance]
        elif view=='top':
            target = p
            camera = p + [-.1,0.,-35.]
        else:
            target = p
            yaw = sample['attitude']['yaw']
            camera = p + [-12.*math.cos(yaw)-5.*math.sin(yaw),-12.*math.sin(yaw)+5.*math.cos(yaw),-7.]
        controls=camera_controls(controls)
        offset=camera-target
        distance=float(np.linalg.norm(offset))*controls['zoom']
        azimuth=math.atan2(offset[1],offset[0])+controls['azimuth']
        elevation=float(np.clip(math.atan2(-offset[2],math.hypot(offset[0],offset[1]))+
                                controls['elevation'],math.radians(-85),math.radians(89.9)))
        ca,sa,ce,se=math.cos(azimuth),math.sin(azimuth),math.cos(elevation),math.sin(elevation)
        # Pan along the camera's screen axes, with NED +Z pointing down.
        right=np.array([sa,-ca,0.]);up=np.array([-se*ca,-se*sa,-ce])
        target=target+distance*(controls['pan'][0]*right+controls['pan'][1]*up)
        camera=target+distance*np.array([ce*ca,ce*sa,-se])
        direction = target-camera
        yaw = math.atan2(direction[1],direction[0])
        pitch = -math.atan2(direction[2],math.hypot(direction[0],direction[1]))
        return camera.tolist(),pitch,yaw


class SimulatorReplay:
    def __init__(self, library, ip='127.0.0.1', port=41451, vehicle='', camera='ReplayCamera', fps=12.):
        self.library=library;self.ip=ip;self.port=port;self.vehicle=vehicle;self.camera_name=camera
        self.interval=1./fps
        self._condition=threading.Condition();self._pending=None;self._closed=False
        self._client=None;self._original_pose=None;self._original_pause=False
        self._original_actor_pose=None;self._world_offset=np.zeros(3)
        self._timeline=None;self._recording_id=None;self._last_request=0.;self._clients={}
        self._image=None;self._image_seq=0;self._last_sample=None
        self._status=dict(enabled=False,connected=False,frame_available=False,frame_sequence=0,
                          recording_id=None,seconds=0.,camera_error=None,error=None)
        self._thread=threading.Thread(target=self._run,name='simulator-replay',daemon=True)
        self._thread.start()

    def status(self):
        with self._condition:return dict(self._status)

    def image(self):
        with self._condition:return self._image

    def submit(self, request):
        if not isinstance(request,dict) or type(request.get('enabled')) is not bool:
            raise ValueError('enabled must be a boolean')
        if request['enabled']:
            if not isinstance(request.get('recording_id'),str):raise ValueError('Missing recording ID')
            seconds=request.get('seconds')
            if isinstance(seconds,bool) or not isinstance(seconds,(int,float)) or not math.isfinite(seconds):
                raise ValueError('seconds must be finite')
            if request.get('view','follow') not in ('follow','top','overview'):raise ValueError('Unknown camera view')
            controls=camera_controls(request.get('camera'))
        client=request.get('client','default');seq=request.get('sequence',0)
        if not isinstance(client,str) or len(client)>128 or type(seq) is not int or seq<0:
            raise ValueError('Invalid playback sequence')
        with self._condition:
            if self._closed:raise ValueError('Replay worker is closed')
            if seq>0 and seq<=self._clients.get(client,-1):return dict(self._status)
            if len(self._clients)>32:self._clients.clear()
            self._clients[client]=seq
            self._pending=dict(request)
            if request['enabled']:self._pending['camera']=controls
            self._last_request=time.monotonic()
            self._condition.notify()
            return dict(self._status)

    def _update(self, **fields):
        with self._condition:self._status.update(fields)

    def _connect(self):
        import airsim
        self._airsim=airsim
        client=airsim.MultirotorClient(ip=self.ip,port=self.port,timeout_value=3)
        if not client.ping():raise RuntimeError('AirSim RPC 未响应，请先启动模拟器')
        self._original_pause=client.simIsPause()
        self._original_pose=client.simGetVehiclePose(vehicle_name=self.vehicle)
        vehicles=client.listVehicles()
        if not self.vehicle and len(vehicles)!=1:raise ValueError('多载具场景请指定 --sim-vehicle')
        self._actor_name=self.vehicle or vehicles[0]
        self._original_actor_pose=client.simGetObjectPose(self._actor_name)
        self._world_offset=np.array([getattr(self._original_actor_pose.position,k)-getattr(self._original_pose.position,k)
                                     for k in ('x_val','y_val','z_val')])
        if not np.isfinite(self._world_offset).all():raise ValueError('无法定位无人机模型')
        client.cancelLastTask(vehicle_name=self.vehicle)
        client.simPause(True)
        self._client=client
        self._camera_ready=False
        try:
            client.simGetCameraInfo(self.camera_name,vehicle_name=self.vehicle,external=True)
            self._camera_ready=True
            self._update(camera_error=None)
        except Exception:
            self._update(camera_error='网页相机不可用：请用 replay_scene.cmd 启动回放用模拟器；仍可在模拟器窗口查看轨迹')
        self._update(connected=True,error=None)

    def _restore(self):
        client=self._client
        if client:
            try:
                client.simFlushPersistentMarkers()
                if self._original_pose:client.simSetVehiclePose(self._original_pose,True,vehicle_name=self.vehicle)
                if self._original_actor_pose:client.simSetObjectPose(self._actor_name,self._original_actor_pose,teleport=True)
            finally:
                client.simPause(self._original_pause)
        self._client=None;self._original_pose=None;self._last_sample=None
        self._timeline=None;self._recording_id=None
        with self._condition:self._image=None
        self._update(enabled=False,connected=False,frame_available=False,recording_id=None)

    def _load(self, identity):
        entry=next((e for e in self.library.list() if e['id']==identity),None)
        if entry is None:raise ValueError('Unknown recording ID')
        data=self.library.get(identity)
        telemetry_path=Path(entry['path']).parent/'telemetry.json'
        telemetry=None
        if telemetry_path.is_file():
            try:telemetry=json.loads(telemetry_path.read_text(encoding='utf-8'))
            except (OSError,ValueError):pass
        self._timeline=ReplayTimeline(data,telemetry)
        self._recording_id=identity;self._last_sample=None
        with self._condition:self._image=None
        self._update(frame_available=False,recording_id=identity)

    def _render(self, request):
        if request['recording_id']!=self._recording_id:self._load(request['recording_id'])
        if self._client is None:self._connect()
        sample=self._timeline.sample(request['seconds'])
        view=request.get('view','follow')
        controls=camera_controls(request.get('camera'))
        key=(sample['seconds'],view,self._recording_id,controls['azimuth'],controls['elevation'],
             controls['zoom'],*controls['pan'])
        if key==self._last_sample:return
        api=self._airsim;client=self._client
        def vec(p):return api.Vector3r(*map(float,p))
        attitude=sample['attitude']
        pose=api.Pose(vec(sample['position']),api.to_quaternion(attitude['pitch'],attitude['roll'],attitude['yaw']))
        client.simSetVehiclePose(pose,True,vehicle_name=self.vehicle)
        # Multirotor's setVehiclePose defers its actor transform until a physics
        # tick. While paused, update the actual Unreal pawn as well. Object pose
        # and markers use world NED, unlike the vehicle's spawn-relative pose.
        world=lambda p:(np.asarray(p)+self._world_offset).tolist()
        actor_pose=api.Pose(vec(world(sample['position'])),pose.orientation)
        if not client.simSetObjectPose(self._actor_name,actor_pose,teleport=True):
            # UE also returns false for an unchanged transform, e.g. switching
            # cameras while paused. Distinguish that from a missing/wrong actor.
            current=client.simGetObjectPose(self._actor_name)
            xyz=lambda p:np.array([getattr(p,k) for k in ('x_val','y_val','z_val')])
            quat=lambda q:np.array([getattr(q,k) for k in ('x_val','y_val','z_val','w_val')])
            a,b=quat(current.orientation),quat(actor_pose.orientation)
            position=xyz(current.position)
            if not np.isfinite(np.r_[position,a,b]).all() or np.linalg.norm(position-xyz(actor_pose.position))>1e-3 or min(np.linalg.norm(a-b),np.linalg.norm(a+b))>1e-4:
                raise RuntimeError('无法更新模拟器中的无人机模型')
        # Infinite markers survive physics pause; explicitly clear on every seek
        # so rewinding also removes the old future portion of the flown track.
        client.simFlushPersistentMarkers()
        for path,color,width in ((sample['history'],[.02,.25,1.,1.],5.),
                                 (sample['path'],[1.,.25,.01,1.],7.),
                                 (sample['trajectory'],[0.,.95,.03,1.],3.)):
            if len(path)>1:client.simPlotLineStrip([vec(world(p)) for p in path],color,thickness=width,is_persistent=True)
        if sample['goal'] is not None:
            client.simPlotPoints([vec(world(sample['goal']))],[1.,.482,.447,1.],size=18.,is_persistent=True)
        if self._camera_ready:
            camera,pitch,yaw=self._timeline.camera(sample,view,controls)
            client.simSetCameraPose(self.camera_name,api.Pose(vec(world(camera)),api.to_quaternion(pitch,0.,yaw)),
                                    vehicle_name=self.vehicle,external=True)
            payload=client.simGetImage(self.camera_name,api.ImageType.Scene,vehicle_name=self.vehicle,external=True)
            if not payload:raise RuntimeError('模拟器未返回相机画面')
            import cv2
            frame=cv2.imdecode(np.frombuffer(payload,dtype=np.uint8),cv2.IMREAD_COLOR)
            if frame is None:raise RuntimeError('无法解码模拟器相机画面')
            ok,encoded=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,85])
            if not ok:raise RuntimeError('无法编码模拟器相机画面')
            with self._condition:
                self._image=encoded.tobytes();self._image_seq+=1
            self._update(frame_available=True,frame_sequence=self._image_seq)
        self._last_sample=key
        self._update(enabled=True,seconds=sample['seconds'],duration=self._timeline.duration,
                     clock='simulation' if self._timeline.sim_clock else 'wall',plan_index=sample['plan_index'],
                     view=view,camera=controls,error=None)

    def _run(self):
        retry_after=0.
        while True:
            with self._condition:
                self._condition.wait_for(lambda:self._closed or self._pending is not None,timeout=1.)
                if self._closed:break
                request=self._pending;self._pending=None
            try:
                if request is None:
                    if self._client and time.monotonic()-self._last_request>8.:self._restore()
                elif not request['enabled']:self._restore()
                elif time.monotonic()>=retry_after:
                    started=time.monotonic();self._render(request)
                    time.sleep(max(0.,self.interval-(time.monotonic()-started)))
            except Exception as exc:
                self._update(error=str(exc),frame_available=False)
                retry_after=time.monotonic()+2.
        try:self._restore()
        except Exception as exc:self._update(error=str(exc))

    def stop(self):
        with self._condition:self._closed=True;self._condition.notify()
        self._thread.join(timeout=10.)
