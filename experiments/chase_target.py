"""Physical target flight along preset corridors; observations expose actual state only."""
from dataclasses import replace
from collections import deque
import threading
import time
import numpy as np

from navigation.safe_navigation import SpatialRoute


def aligned_target_position(history,timestamp_ns,max_gap=.3,max_extrapolation=.15):
    """Evaluate coeval truth; never supplied to the delayed navigation observer."""
    if not history:return None
    for left,right in zip(history,history[1:]):
        if left.timestamp_ns<=timestamp_ns<=right.timestamp_ns:
            gap=(right.timestamp_ns-left.timestamp_ns)*1e-9
            if gap<=0 or gap>max_gap:return None
            alpha=(timestamp_ns-left.timestamp_ns)/(right.timestamp_ns-left.timestamp_ns)
            return left.position+(right.position-left.position)*alpha
    endpoint=history[0] if timestamp_ns<history[0].timestamp_ns else history[-1]
    offset=(timestamp_ns-endpoint.timestamp_ns)*1e-9
    if abs(offset)<=max_extrapolation:return endpoint.position+endpoint.velocity*offset
    return None


class WorldVehicle:
    """Translate a vehicle's spawn-relative coordinates into the pursuer's frame."""
    def __init__(self,io,offset):self.io=io;self.offset=np.asarray(offset,dtype=float)
    def get_state(self):
        state=self.io.get_state()
        return replace(state,position=state.position+self.offset)
    def teleport(self,position,yaw=0.):self.io.teleport(np.asarray(position)-self.offset,yaw)
    def arm(self):self.io.arm()
    def takeoff(self):self.io.takeoff()
    def move_to_z(self,z,speed):self.io.move_to_z(z-self.offset[2],speed)
    def move_by_velocity(self,*args):self.io.move_by_velocity(*args)
    def land(self):self.io.land()


class TargetFlight:
    def __init__(self,io,preset,speed=None):
        self.io=io;self.preset=preset;self.speed=float(speed or preset['target_speed'])
        self.points=np.asarray(preset['target_path'],dtype=float)
        self.route=SpatialRoute(self.points,self.speed)
        self._stop=threading.Event();self.state=None;self.error=None
        self.distance=0.;self.legs=0;self._u=np.zeros(3);self.samples=0
        self._history=deque(maxlen=256);self._history_lock=threading.Lock()

    def position_at(self,timestamp_ns):
        with self._history_lock:history=list(self._history)
        return aligned_target_position(history,timestamp_ns)

    def start(self):
        self._thread=threading.Thread(target=self._run,name='black-drone',daemon=True);self._thread.start()

    def _run(self):
        last=None;started_sim=None;last_sim=None
        while not self._stop.is_set():
            began=time.monotonic()
            try:
                state=self.io.get_state()
                with self._history_lock:
                    if not self._history or state.timestamp_ns>self._history[-1].timestamp_ns:self._history.append(state)
                self.state=state;self.samples+=1
                now=state.timestamp_ns*1e-9
                if started_sim is None:started_sim=now
                dt=.1 if last_sim is None else np.clip(now-last_sim,.0,.2)
                last_sim=now
                if last is not None:self.distance+=float(np.linalg.norm(state.position-last))
                last=state.position.copy()
                index=self.route.locate(state.position);arc=self.route.arc[index]
                remaining=self.route.arc[-1]-arc
                end_distance=np.linalg.norm(state.position-self.route.points[-1])
                if end_distance<1.2 and np.linalg.norm(state.velocity)<1.:
                    self.legs+=1
                    self.route=SpatialRoute(self.route.points[::-1].copy(),self.speed)
                    arc=0.;remaining=self.route.arc[-1];end_distance=remaining
                speed=self.speed
                cycle=self.preset.get('speed_cycle')
                if cycle:
                    phase=(now-started_sim)%sum(item[0] for item in cycle)
                    for length,value in cycle:
                        if phase<length:speed=value*self.speed/self.preset['target_speed'];break
                        phase-=length
                # Projection can reach the last sample before the aircraft does.
                # Brake by physical endpoint distance, then correct any overshoot.
                speed=min(speed,np.sqrt(2.*1.1*max(0.,end_distance-.6)))
                carrot=self.route.at(min(arc+max(1.,speed*.45),self.route.arc[-1]))
                delta=carrot-state.position
                requested=delta/max(np.linalg.norm(delta),1e-6)*speed
                change=requested-self._u
                self._u+=change*min(1.,1.6*dt/max(np.linalg.norm(change),1e-6))
                self.io.move_by_velocity(*self._u.tolist(),.5)
                if state.collision:raise RuntimeError('Target collided with the scene; benchmark invalid')
            except Exception as exc:
                self.error=str(exc);self._stop.set()
            self._stop.wait(max(.01,.1-(time.monotonic()-began)))

    def stop(self):
        self._stop.set()
        if hasattr(self,'_thread'):self._thread.join(timeout=4.)
        self.io.move_by_velocity(0.,0.,0.,.5)


def vehicle_offsets(client,pursuer,target):
    def xyz(value):return np.array([value.x_val,value.y_val,value.z_val])
    offsets={}
    for name in (pursuer,target):
        offsets[name]=xyz(client.simGetObjectPose(name).position)-xyz(client.simGetVehiclePose(vehicle_name=name).position)
        if not np.isfinite(offsets[name]).all():raise ValueError('Cannot locate vehicle actor '+name)
    return offsets[target]-offsets[pursuer]
