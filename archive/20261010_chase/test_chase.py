"""Focused checks for causal prediction, shared deltas, and dual-drone replay."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import numpy as np
import pytest
from navigation.chase import ChaseOptions,TargetTracker
from navigation.persistent_map import PersistentMap
from navigation.shared_map import merge_observations
from navigation.occupancy import FREE,OCCUPIED,UNKNOWN
from experiments.simulator_replay import ReplayTimeline


def test_delay_compensation_and_bounded_intercept():
    tracker=TargetTracker(ChaseOptions())
    tracker.observe([20,0,-10],[3,0,0],1.)
    pred=tracker.predict([0,0,-10],1.4,5.)
    assert np.allclose(pred['position'],[21.2,0,-10])
    assert 0<pred['lead']<=3.
    assert pred['goal'][0]>pred['follow_goal'][0]
    assert tracker.predict([0,0,-10],3.,5.) is None


def test_pure_uses_received_position_without_future_lead():
    tracker=TargetTracker(ChaseOptions(strategy='pure'))
    tracker.observe([20,0,-10],[3,0,0],1.)
    pred=tracker.predict([0,0,-10],1.4,5.)
    assert np.allclose(pred['position'],[20,0,-10])
    assert pred['lead']==0


def test_observations_reject_old_timestamps_and_nonfinite_values():
    tracker=TargetTracker(ChaseOptions())
    tracker.observe([20,0,-10],[3,0,0],1.)
    assert not tracker.observe([999,0,0],[0,0,0],1.)
    assert tracker.position[0]==20
    with pytest.raises(ValueError):tracker.observe([np.nan,0,0],[0,0,0],2.)


def test_stopped_target_keeps_horizontal_standoff_within_capture_radius():
    tracker=TargetTracker(ChaseOptions())
    tracker.observe([20,0,-10],[3,0,0],1.)
    tracker.predict([10,0,-12],1.,5.)
    tracker.observe([20,0,-10],[0,0,0],10.)
    pred=tracker.predict([20,0,-12],10.,5.)
    assert np.allclose(pred['follow_goal'],[16.5,0,-12],atol=1e-6)
    assert np.linalg.norm(pred['follow_goal']-pred['position'])<5.


def test_headon_approach_stays_on_pursuer_side_without_nearby_flips():
    tracker=TargetTracker(ChaseOptions())
    tracker.observe([20,0,-10],[-3,0,0],1.)
    pred=tracker.predict([0,0,-12],1.,5.)
    assert pred['follow_goal'][0]==16.5
    # Tiny projection changes near overlap must not jump the anchor by seven metres.
    pred=tracker.predict([21,0,-12],1.,5.)
    assert pred['follow_goal'][0]==16.5
    trailing=TargetTracker(ChaseOptions(stand_off_mode='trailing'))
    trailing.observe([20,0,-10],[-3,0,0],1.)
    assert trailing.predict([0,0,-12],1.,5.)['follow_goal'][0]==23.5


def test_close_control_brakes_for_relative_speed_and_rejects_a_blocked_chord():
    from types import SimpleNamespace
    from navigation.airsim_io import VehicleState
    from navigation.config import Config
    from navigation.profiles import profile_overrides
    from navigation.safe_navigation import RouteCoordinator
    cfg=Config.from_overrides(profile_overrides('safe5'));control=RouteCoordinator(None,cfg)
    state=VehicleState(np.array([0.,0.,-10.]),np.array([5.,0.,0.]),np.zeros(3),0.)
    snap=SimpleNamespace(ts=10.,dist=np.full((100,100,100),6.,dtype='float32'),origin=np.array([-20.,-20.,-30.]))
    control.moving_target=dict(follow_goal=np.array([12.,0.,-10.]),velocity=np.array([-3.,0.,0.]),wall_t=10.)
    velocity=control.terminal_velocity(state,snap,10.)
    assert velocity is not None and 0<velocity[0]<5.
    snap.dist[74:77,:,:]=0.
    assert control.terminal_velocity(state,snap,10.) is None


def test_example_reflight_reads_only_the_common_map():
    from types import SimpleNamespace
    from experiments.fly_examples import read_catalog,prepare_scenario
    from navigation.paths import SHARED_MAP
    args=SimpleNamespace(output_dir='data/records/example_runs',ip=None,port=None,load_map=None)
    entry=next(e for e in read_catalog() if e['flight']==2)
    scenario=prepare_scenario(entry,args)
    assert scenario['config']['persistent_map_path']==str(SHARED_MAP)
    assert scenario['config']['persistent_seed_path']==''
    assert scenario['map_policy']=='snapshot'
    args.load_map=False
    assert prepare_scenario(entry,args)['map_policy']=='cold'


def test_neighbor_seed_matches_reference_for_shifted_negative_grids():
    from navigation.safe_navigation import seed_history
    rng=np.random.default_rng(10);history=PersistentMap(.8,4)
    for key in [(0,0,0),(-1,0,0),(0,-1,-1),(50,50,50)]:history.blocks[key]=rng.integers(0,3,(4,4,4),dtype='int8')
    for origin in ([-2.,-.8,-1.6],[.4,.4,-.4]):
        a=rng.integers(0,3,(12,10,8),dtype='int8');b=a.copy()
        expected=history.seed_local(a,np.asarray(origin),.4)
        assert seed_history(history,b,np.asarray(origin),.4)==expected
        assert np.array_equal(a,b)


def test_cold_observations_preserve_history_and_clear_observed_cells(tmp_path):
    path=tmp_path/'common.npz'
    old=PersistentMap(.8,2);old.blocks[(0,0,0)]=np.full((2,2,2),UNKNOWN,dtype='int8')
    old.blocks[(0,0,0)][0,0,0]=OCCUPIED;old.blocks[(0,0,0)][1,0,0]=OCCUPIED;old.save(path,strict=True)
    fresh=PersistentMap(.8,2);fresh.blocks[(0,0,0)]=np.full((2,2,2),UNKNOWN,dtype='int8')
    fresh.blocks[(0,0,0)][0,0,0]=FREE;fresh.blocks[(1,0,0)]=np.full((2,2,2),OCCUPIED,dtype='int8')
    merge_observations(fresh,path)
    result=PersistentMap.load(path,strict=True)
    assert result.query_voxel(0,0,0)==FREE
    assert result.query_voxel(1,0,0)==OCCUPIED
    assert result.query_voxel(2,0,0)==OCCUPIED


def test_corrupt_shared_map_is_not_silently_overwritten(tmp_path):
    path=tmp_path/'common.npz';path.write_bytes(b'broken map')
    with pytest.raises(Exception):merge_observations(PersistentMap(.8),path)
    assert path.read_bytes()==b'broken map'


def test_failed_save_is_reported(tmp_path):
    path=tmp_path/'directory.npz';path.mkdir()
    with pytest.raises(OSError):PersistentMap(.8).save(path,strict=True)


def test_dual_replay_interpolates_both_and_has_no_future_target_trail():
    data={'states':[{'t':10.,'p':[0,0,-10],'sim_timestamp_ns':1000000000},
                    {'t':12.,'p':[10,0,-10],'sim_timestamp_ns':3000000000}],
          'targets':[{'t':10.,'p':[20,0,-10],'sim_timestamp_ns':1000000000},
                     {'t':12.,'p':[26,0,-10],'sim_timestamp_ns':3000000000}]}
    timeline=ReplayTimeline(data);sample=timeline.sample(1.)
    assert sample['position']==[5,0,-10]
    assert sample['target']['position']==[23,0,-10]
    assert sample['target_history']==[[20,0,-10],[23,0,-10]]
    camera,pitch,yaw=timeline.camera(sample,'pair_top')
    assert camera[2]<-40 and abs(camera[0]-14)<.2


def test_invalid_target_clock_is_rejected():
    data={'states':[{'t':1.,'p':[0,0,0]},{'t':2.,'p':[1,0,0]}],
          'targets':[{'t':float('nan'),'p':[0,0,0]}]}
    with pytest.raises(ValueError):ReplayTimeline(data)


def test_target_corrects_endpoint_overshoot_instead_of_stopping_outside(tmp_path):
    from navigation.airsim_io import VehicleState
    from experiments.chase_target import TargetFlight
    class IO:
        def get_state(self):return VehicleState(np.array([12.,0.,-10.]),np.zeros(3),np.zeros(3),0.,timestamp_ns=1000000000)
        def move_by_velocity(self,*command):
            self.command=command;flight._stop.set()
    io=IO();flight=TargetFlight(io,dict(target_speed=3.,target_path=[[0,0,-10],[10,0,-10]]))
    flight._run()
    assert io.command[0]<0
    assert flight.legs==0 and flight.error is None


def test_capture_uses_coeval_actual_history_and_rejects_large_gaps():
    from navigation.airsim_io import VehicleState
    from experiments.chase_target import aligned_target_position
    def state(seconds,x):
        return VehicleState(np.array([x,0.,-10.]),np.array([4.,0.,0.]),np.zeros(3),0.,timestamp_ns=int(seconds*1e9))
    history=[state(1.,0.),state(1.1,.4),state(1.2,.8)]
    assert np.allclose(aligned_target_position(history,int(1.025e9)),[.1,0,-10])
    assert np.allclose(aligned_target_position(history,int(1.25e9)),[1.,0,-10])
    assert aligned_target_position(history,int(1.5e9)) is None
    assert aligned_target_position([state(1.,0.),state(2.,4.)],int(1.5e9)) is None
    assert aligned_target_position([],int(1e9)) is None


def test_startup_propagates_rpc_error_instead_of_silent_join(monkeypatch):
    from types import SimpleNamespace
    from navigation.airsim_io import AirSimIO
    from navigation.config import Config
    closed=[]
    class Future:
        def get(self):raise TimeoutError('slow RPC')
    client=SimpleNamespace(takeoffAsync=lambda **kwargs:Future(),client=SimpleNamespace(close=lambda:closed.append(True)))
    io=AirSimIO(Config());monkeypatch.setattr(io,'_client_locked',lambda:client)
    io._airsim=SimpleNamespace(MultirotorClient=lambda **kwargs:client)
    with pytest.raises(TimeoutError,match='slow RPC'):io.takeoff()
    assert closed==[True]


@pytest.mark.parametrize('actual_z,completed',[(-9.9,True),(-6.5,False)])
def test_climb_checks_actual_height_even_when_simulator_returns_false(monkeypatch,actual_z,completed):
    from types import SimpleNamespace
    from navigation.airsim_io import AirSimIO
    from navigation.config import Config
    class Future:
        def get(self):return False
    client=SimpleNamespace(moveToZAsync=lambda *args,**kwargs:Future(),client=SimpleNamespace(close=lambda:None))
    io=AirSimIO(Config());monkeypatch.setattr(io,'_client_locked',lambda:client)
    io._airsim=SimpleNamespace(MultirotorClient=lambda **kwargs:client)
    monkeypatch.setattr(io,'get_state',lambda:SimpleNamespace(position=np.array([0.,0.,actual_z])))
    if completed:io.move_to_z(-10.)
    else:
        with pytest.raises(RuntimeError,match='Climb ended'):io.move_to_z(-10.)


def test_snapshot_reuses_memory_seed_without_overwriting_shared_updates(tmp_path,monkeypatch):
    import experiments.experiment as experiment
    from navigation.config import Config
    from navigation.pipeline import NavigationPipeline
    path=tmp_path/'shared.npz';monkeypatch.setattr(experiment,'SHARED_MAP',path)
    pm=PersistentMap(.8,2);pm.blocks[(0,0,0)]=np.full((2,2,2),OCCUPIED,dtype='int8');pm.save(path,strict=True)
    cfg=Config(persistent_map_path=str(path),persistent_res=.8,persistent_block=2,res=.4)
    baseline=experiment.MapBaseline(cfg,'snapshot',tmp_path)
    baseline.prepare(cfg,tmp_path/'one');first=NavigationPipeline(None,cfg)
    first._persistent_delta.blocks[(0,0,0)]=np.full((2,2,2),FREE,dtype='int8')
    first._save_persistent_map()
    baseline.prepare(cfg,tmp_path/'two');second=NavigationPipeline(None,cfg)
    assert second.persistent.query_voxel(0,0,0)==OCCUPIED
    assert baseline.seed.query_voxel(0,0,0)==OCCUPIED
    assert not first._persistent_delta.blocks
    second._save_persistent_map()
    assert PersistentMap.load(path,strict=True).query_voxel(0,0,0)==FREE
    assert not list(tmp_path.rglob('run_map*'))


def test_async_save_detaches_delta_and_retains_newer_cells_on_failure(tmp_path,monkeypatch):
    import threading
    import navigation.shared_map as shared
    from navigation.config import Config
    from navigation.pipeline import NavigationPipeline
    pipe=NavigationPipeline(None,Config(persistent_map_path=str(tmp_path/'map.npz'),load_persistent_map=False))
    original=np.full((16,16,16),UNKNOWN,dtype='int8');original[0,0,0]=OCCUPIED;original[1,0,0]=OCCUPIED
    pipe._persistent_delta.blocks[(0,0,0)]=original
    started=threading.Event();release=threading.Event()
    def fail(*args):started.set();release.wait(2);raise OSError('write unavailable')
    monkeypatch.setattr(shared,'merge_observations',fail)
    pipe._request_persistent_save();assert started.wait(1)
    assert not pipe._persistent_delta.blocks
    new=np.full_like(original,UNKNOWN);new[0,0,0]=FREE
    pipe._persistent_delta.blocks[(0,0,0)]=new
    release.set();pipe._persist_worker.join(2)
    assert pipe._persistent_delta.query_voxel(0,0,0)==FREE
    assert pipe._persistent_delta.query_voxel(1,0,0)==OCCUPIED
    assert pipe._persist_save_error=='write unavailable'
