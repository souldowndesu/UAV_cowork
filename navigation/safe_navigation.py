"""Validated spatial splines, persistent route commitment, and receding control."""
from __future__ import annotations
import threading
import time
import traceback
import itertools
from dataclasses import replace
import numpy as np

from .pipeline import NavigationPipeline, PlanSnapshot
from .occupancy import OccupancyMap
from . import raycast, distance_field, bspline
from .geometry import distance_query
from scipy import ndimage


def clearances(points, snapshot, res):
    values, _, valid = distance_query(points, snapshot.dist, snapshot.origin, res)
    return np.where(valid, values, 0.)


def route_clearance(cfg):
    return cfg.d_min+cfg.route_buffer+(cfg.brake_margin if cfg.velocity_safety_guard else 0.)


def corridor_speed_limit(position, direction, snapshot, cfg, reaction):
    """Limit speed by the measured stopping corridor, including map latency."""
    length=np.linalg.norm(direction)
    if length<1e-6:return cfg.v_max,cfg.d_max
    maximum=cfg.v_max*reaction+cfg.v_max**2/(2*cfg.brake_accel)+1.
    distances=np.linspace(0.,maximum,max(10,int(maximum/.2)+1))
    values=clearances(position+distances[:,None]*direction/length,snapshot,cfg.res)
    bad=np.flatnonzero(values<cfg.d_min+cfg.route_buffer+cfg.brake_margin)
    available=float(distances[bad[0]]) if len(bad) else maximum
    reserve=cfg.brake_accel*reaction
    limit=max(0.,-reserve+np.sqrt(reserve**2+2*cfg.brake_accel*max(0.,available-.6)))
    return min(cfg.v_max,limit),float(values.min())


def nearest_observed_approach(snapshot,position,goal,cfg):
    minimum=route_clearance(cfg)
    traversable=(snapshot.occ!=2)&(snapshot.dist>=minimum)
    labels,_=ndimage.label(traversable)
    start=np.floor((position-snapshot.origin)/cfg.res).astype(int)
    if (start<0).any() or (start>=snapshot.occ.shape).any():return None
    component=labels[tuple(start)]
    if not component:return None
    heights=snapshot.origin[2]+(np.arange(snapshot.occ.shape[2])+.5)*cfg.res
    mask=(snapshot.occ==1)&(snapshot.dist>=max(cfg.d_safe,minimum+1.))&(labels==component)
    mask[:,:,np.abs(heights-goal[2])>1.2]=False
    indices=np.argwhere(mask)
    if not len(indices):return None
    candidates=snapshot.origin+(indices+.5)*cfg.res
    coarse=NavigationPipeline._coarsen(snapshot.occ,snapshot.origin,cfg.res,cfg.plan_res)
    coarse_dist=distance_field.compute(coarse,cfg)
    ds,_,valid=distance_query(candidates,coarse_dist,coarse.origin,coarse.res)
    candidates=candidates[valid&(ds>=minimum+.3)]
    if cfg.approach_max_distance>0:
        candidates=candidates[np.linalg.norm(candidates-goal,axis=1)<=cfg.approach_max_distance]
    if not len(candidates):return None
    return candidates[np.argmin(np.linalg.norm(candidates-goal,axis=1))]


def seed_history(history, data, origin, res):
    """Same centre-query semantics as seed_local, without sorting every voxel."""
    axes=[np.floor((origin[a]+(np.arange(data.shape[a])+.5)*res-history.origin[a])/history.res).astype(int) for a in range(3)]
    block_axes=[np.floor_divide(a,history.block) for a in axes]
    filled=0
    for key, block in list(history.blocks.items()):
        indices=[np.flatnonzero(block_axes[a]==key[a]) for a in range(3)]
        if any(len(i)==0 for i in indices):continue
        local_index=np.ix_(*indices)
        values=block[np.ix_(*[axes[a][indices[a]]%history.block for a in range(3)])]
        local=data[local_index]
        mask=(local==0)&(values!=0)
        filled+=int(mask.sum());local[mask]=values[mask];data[local_index]=local
    return filled


def merge_history(history,data,origin,res):
    axes=[np.floor((origin[a]+(np.arange(data.shape[a])+.5)*res-history.origin[a])/history.res).astype(int) for a in range(3)]
    block_axes=[np.floor_divide(a,history.block) for a in axes]
    count=0
    for key in itertools.product(*[np.unique(a) for a in block_axes]):
        indices=[np.flatnonzero(block_axes[a]==key[a]) for a in range(3)]
        local=data[np.ix_(*indices)];known=np.argwhere(local!=0)
        if not len(known):continue
        count+=len(known)
        batch=np.zeros((history.block,)*3,dtype=np.int8)
        offsets=[axes[a][indices[a]][known[:,a]]%history.block for a in range(3)]
        np.maximum.at(batch,tuple(offsets),local[tuple(known.T)])
        block=history.blocks.setdefault(tuple(int(a) for a in key),np.zeros_like(batch))
        touched=batch!=0;block[touched]=batch[touched]
    return count


def integrate_sensor_history(history,origin,points,cfg):
    """Persist measured rays in the complete sensor sphere, beyond local z bounds."""
    res=history.res;half=cfg.max_range
    low=history.origin+np.floor((origin-half-history.origin)/res)*res
    side=int(np.ceil(2*half/res))+2
    observed=OccupancyMap(side,side,side,res,tuple(low))
    rays=raycast.integrate(observed,origin,points,replace(cfg,res=res))
    # Native DDA works relative to buffer origin; exact-boundary floating
    # rounding can choose a neighbouring endpoint. Canonical world indices
    # own occupied hits, independent of the temporary buffer's translation.
    points=np.asarray(points,dtype=float)
    lengths=np.linalg.norm(points-origin,axis=1)
    hits=points[np.isfinite(points).all(axis=1)&(lengths>=cfg.self_echo_dist)&(lengths<=half)]
    base=np.rint((low-history.origin)/res).astype(int)
    cells=np.floor((hits-history.origin)/res).astype(int)-base
    observed.data[observed.data==2]=0
    if len(cells):observed.data[tuple(cells.T)]=2
    merge_history(history,observed.data,observed.origin,res)
    return rays


def ray_goal(start, goal, origin, res, shape):
    delta=np.asarray(goal)-start
    lo=origin+res;hi=origin+(np.asarray(shape)-1)*res
    fraction=1.
    for a in range(3):
        if delta[a]>1e-6:fraction=min(fraction,(hi[a]-start[a])/delta[a])
        elif delta[a]<-1e-6:fraction=min(fraction,(lo[a]-start[a])/delta[a])
    return np.clip(start+max(0.,fraction)*delta,lo,hi)


def connected_local_goal(world,dist,position,requested,minimum,guide=None):
    traversable=(world.data!=2)&(dist>=minimum)
    labels,_=ndimage.label(traversable)
    start=world.world_to_voxel(position)
    if not world.in_bounds(*start) or not labels[start]:return np.asarray(position).copy()
    reachable=labels==labels[start]
    goal=world.world_to_voxel(requested)
    if world.in_bounds(*goal) and reachable[goal]:return requested
    # Follow the reachable prefix of the committed global detour. A point
    # beyond the next wall must not be projected into unrelated old FREE cells.
    if guide is not None and guide.path is not None:
        nearest=guide.locate(position)
        indices=np.flatnonzero((guide.arc>=guide.arc[nearest])&(guide.arc<=guide.arc[nearest]+guide.cfg.guide_lookahead))
        for i in reversed(indices):
            point=guide.path[i];voxel=world.world_to_voxel(point)
            if np.linalg.norm(point-position)>3. and world.in_bounds(*voxel) and reachable[voxel]:return point
        guide.path=None;guide.last_update=-np.inf
    indices=np.argwhere(reachable)
    points=world.origin+(indices+.5)*world.res
    return points[np.argmin(np.linalg.norm(points-requested,axis=1))]


def shorten_path(points, dist, origin, res, minimum):
    """Remove voxel stair steps only when the entire shortcut has clearance."""
    output=[points[0]];i=0
    while i<len(points)-1:
        next_i=i+1
        for j in range(len(points)-1,i+1,-1):
            chord=np.linspace(points[i],points[j],max(3,int(np.linalg.norm(points[j]-points[i])/.15)+1))
            ds,_,valid=distance_query(chord,dist,origin,res)
            if valid.all() and (ds>=minimum).all():next_i=j;break
        output.append(points[next_i]);i=next_i
    return np.asarray(output)


class SpatialRoute:
    def __init__(self, points, speed, ctrl=None):
        self.points = np.asarray(points, dtype=float)
        keep = np.r_[True, np.linalg.norm(np.diff(self.points, axis=0), axis=1) > 1e-5]
        self.points = self.points[keep]
        self.arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.points, axis=0), axis=1))]
        self.speed = speed
        self.final=False
        self.ctrl = np.asarray(ctrl if ctrl is not None else self.points)
        self.total_time = max(.1, self.arc[-1] / max(speed, .1))
        self.dt = self.total_time / max(1, len(self.ctrl)-1)

    def locate(self, position):
        previous=getattr(self,'_progress',0)
        candidates=np.flatnonzero((self.arc>=self.arc[previous]-1.5)&(self.arc<=self.arc[previous]+8.))
        nearest=int(candidates[np.argmin(np.linalg.norm(self.points[candidates]-position,axis=1))])
        self._progress=max(previous,nearest)
        return nearest

    def at(self, arc):
        return np.array([np.interp(arc, self.arc, self.points[:, a]) for a in range(3)])

    def eval(self, t):
        s = min(max(t*self.speed, 0), self.arc[-1])
        p = self.at(s)
        delta = self.at(min(s+.1, self.arc[-1])) - p
        v = delta / max(np.linalg.norm(delta), 1e-6) * self.speed
        return p, v, np.zeros(3)


def build_validated_route(path, start, coarse_map, dist, cfg):
    points = np.vstack([start, path])
    keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-5]
    points = points[keep]
    if len(points) < 2:
        return None
    if cfg.safe_shortcuts:
        points=shorten_path(points,dist,coarse_map.origin,coarse_map.res,route_clearance(cfg)+.6)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    # No mixing of unrelated old controls. Increase density if corner rounding is unsafe.
    for spacing in ((4.,2.,1.2,.8,.4,.2) if cfg.safe_shortcuts else (.8,.4,.2)):
        positions = np.linspace(0., arc[-1], max(4, int(np.ceil(arc[-1]/spacing))+1))
        controls = np.column_stack([np.interp(positions, arc, points[:, a]) for a in range(3)])
        controls = np.vstack([start, start, controls])
        spline = bspline.BSplineTrajectory(controls, max(.03, spacing / cfg.v_mission))
        times = np.linspace(0, spline.total_time, max(100, int(arc[-1]/.12)))
        sample = np.array([spline.eval(t)[0] for t in times])
        clearance, _, valid = distance_query(sample, dist, coarse_map.origin, coarse_map.res)
        if valid.all() and (clearance >= route_clearance(cfg) + .1).all():
            return SpatialRoute(sample, cfg.v_mission, controls)
    # Dense linear reference remains collision-checked; controller slows turns.
    positions = np.linspace(0., arc[-1], max(2, int(arc[-1]/.1)+1))
    sample = np.column_stack([np.interp(positions, arc, points[:, a]) for a in range(3)])
    clearance, _, valid = distance_query(sample, dist, coarse_map.origin, coarse_map.res)
    if valid.all() and (clearance >= route_clearance(cfg)).all():
        return SpatialRoute(sample, cfg.v_mission)
    return None


class RouteCoordinator:
    def __init__(self, io, cfg):
        self.io, self.cfg = io, cfg
        self.traj = None; self.t0 = 0.; self.snapshot = None
        self.raw_snapshot = None
        self._u = np.zeros(3); self._acc = np.zeros(3); self._last_ts = None
        self.min_clearance = cfg.d_max; self.emergency = False
        self._route_lock = threading.Lock()

    def set_trajectory(self, route, now):
        with self._route_lock:
            self.traj, self.t0 = route, now

    def tick(self, state, now):
        cfg = self.cfg
        dt = cfg.control_dt if self._last_ts is None else (state.timestamp_ns-self._last_ts)*1e-9
        self._last_ts = state.timestamp_ns
        dt = np.clip(dt, 0., .1)
        route, snap = self.traj, self.snapshot
        target = np.zeros(3); braking = False; speed_limit = 0.
        if route is not None and snap is not None and now-snap.ts < cfg.map_stale_timeout:
            idx = route.locate(state.position); s = route.arc[idx]
            remaining = route.arc[-1]-s
            actual_speed = np.linalg.norm(state.velocity)
            horizon = min(remaining, max(8., actual_speed**2/(2*cfg.a_max)+actual_speed*.7+3.))
            arc_samples = np.arange(s, min(s+horizon, route.arc[-1])+.05, .25)
            samples = np.array([route.at(a) for a in arc_samples])
            ds = clearances(samples, snap, cfg.res)
            bad = np.where(ds < cfg.d_min + cfg.route_buffer)[0]
            safe_length = float(arc_samples[bad[0]]-s) if len(bad) else horizon
            # Space for perception reaction and braking, independent of speculative suffix.
            speed_limit = max(0., -cfg.a_max*.55 + np.sqrt((cfg.a_max*.55)**2 + 2*cfg.a_max*max(0., safe_length-2.)))
            if cfg.safe_shortcuts and not len(bad):speed_limit=cfg.v_max
            speed = min(cfg.v_mission, cfg.v_max, speed_limit, np.sqrt(2*cfg.a_max*max(0., remaining-.2)))
            if cfg.safe_shortcuts and remaining<3.:
                speed=min(speed_limit,2.,1.3*np.linalg.norm(route.points[-1]-state.position))
            if route.final:
                reserve=cfg.a_max*cfg.goal_brake_reaction
                goal_distance=float(np.linalg.norm(route.points[-1]-state.position))
                # Once projection reaches the endpoint, remaining arc is zero
                # even after overshoot. Recovery must use physical distance.
                braking_distance=min(remaining,goal_distance) if remaining>1. else goal_distance
                goal_limit=max(.25,-reserve+np.sqrt(reserve**2+2*cfg.a_max*max(0.,braking_distance-.6)))
                speed=min(speed,goal_limit)
            if cfg.curve_accel_limit>0:
                # Brake before bends according to lateral acceleration v²*kappa.
                for offset in np.arange(0.,horizon,.5):
                    center=s+offset
                    if center<.5 or center+.5>route.arc[-1]:continue
                    before=route.at(center)-route.at(center-.5)
                    after=route.at(center+.5)-route.at(center)
                    curvature=np.linalg.norm(after/max(np.linalg.norm(after),1e-6)-before/max(np.linalg.norm(before),1e-6))/.5
                    if curvature>.02:
                        turn_speed=np.sqrt(cfg.curve_accel_limit/curvature)
                        if cfg.velocity_safety_guard:
                            reserve=cfg.brake_accel*cfg.goal_brake_reaction
                            turn_limit=-reserve+np.sqrt(reserve**2+turn_speed**2+2*cfg.brake_accel*max(0.,offset-1.))
                            speed=min(speed,max(.3,turn_limit))
                        else:speed=min(speed,np.sqrt(turn_speed**2+2*cfg.a_max*max(0.,offset-1.)))
            lookahead = min(remaining, max(1.2, actual_speed*.5+1.))
            carrot = route.at(s+lookahead)
            # Reduce speed for upcoming changes of tangent and cross-track recovery.
            direction = carrot-state.position
            norm = np.linalg.norm(direction)
            if norm > .05:
                tangent1 = route.at(min(s+1., route.arc[-1]))-route.at(s)
                tangent2 = route.at(min(s+lookahead+1., route.arc[-1]))-route.at(min(s+lookahead, route.arc[-1]))
                cosine = float(tangent1@tangent2 / max(np.linalg.norm(tangent1)*np.linalg.norm(tangent2),1e-6))
                speed *= max(.3, np.sqrt(max(0., (1+cosine)/2)))
                if not cfg.safe_shortcuts: speed = min(speed, max(.3, 1.4*remaining))
                # The carrot chord must itself be safe; decrease lookahead at tight bends.
                for _ in range(5):
                    chord = np.linspace(state.position, carrot, max(3,int(norm/.15)+1))
                    if (clearances(chord, snap, cfg.res) >= route_clearance(cfg)).all(): break
                    lookahead *= .5; carrot=route.at(s+lookahead); direction=carrot-state.position; norm=np.linalg.norm(direction)
                else:
                    speed = 0.
                target = direction/max(norm,1e-6)*speed
            braking = speed_limit < .2
            self.min_clearance = float(clearances([state.position],snap,cfg.res)[0])
        else:
            braking = True
        # A newly observed wall can invalidate a stationary start in the coarser
        # grid. Move slowly along an increasing-clearance chord, never snap the
        # search start to another cell or relax the physical clearance check.
        if snap is not None and now-snap.ts < cfg.map_stale_timeout:
            values, gradients, valid = distance_query([state.position],snap.dist,snap.origin,cfg.res)
            self.min_clearance = float(values[0])
            if valid[0] and values[0] < cfg.d_min+cfg.route_buffer+1.5 and np.linalg.norm(target)<.2:
                gradient=gradients[0]; length=np.linalg.norm(gradient)
                if length>.1:
                    escape=gradient/length
                    chord=state.position+np.linspace(0.,1.5,20)[:,None]*escape
                    ds=clearances(chord,snap,cfg.res)
                    if ds.min()>=cfg.d_min and ds[-1]>values[0]+.3:
                        target=escape*.75; braking=False
        if cfg.safe_shortcuts and self.min_clearance<cfg.d_min+cfg.route_buffer+1.5 and np.linalg.norm(target)<.2 and self.raw_snapshot is not None:
            raw=self.raw_snapshot
            if now-raw.ts<cfg.map_stale_timeout:
                best=None;score=-1.
                for direction in itertools.product((-1.,0.,1.),repeat=3):
                    direction=np.asarray(direction);length=np.linalg.norm(direction)
                    if length<.1:continue
                    direction/=length
                    chord=state.position+np.linspace(0.,2.5,30)[:,None]*direction
                    ds=clearances(chord,snap,cfg.res);physical=clearances(chord,raw,cfg.res)
                    if physical.min()<cfg.drone_radius+.25 or ds[-1]<cfg.d_min+1.:
                        continue
                    if (np.diff(ds)<-.15).any():continue
                    if ds[-1]>score:best=direction;score=ds[-1]
                if best is not None:target=best*.6;braking=False
        if self.emergency: target[:]=0.; braking=True
        predicted_min = cfg.d_max
        hard_brake=False
        if cfg.velocity_safety_guard and snap is not None and now-snap.ts<cfg.map_stale_timeout:
            actual_speed=float(np.linalg.norm(state.velocity))
            target_speed=float(np.linalg.norm(target))
            reaction=cfg.brake_reaction+min(max(0.,now-snap.ts),cfg.map_stale_timeout)*.5
            if target_speed>.75:
                limit,minimum=corridor_speed_limit(state.position,target,snap,cfg,reaction)
                predicted_min=min(predicted_min,minimum)
                target*=min(1.,limit/max(target_speed,1e-6))
                speed_limit=min(speed_limit,limit)
            if actual_speed>.75:
                actual_limit,minimum=corridor_speed_limit(state.position,state.velocity,snap,cfg,reaction)
                predicted_min=min(predicted_min,minimum)
                if actual_speed>actual_limit+.15:
                    target[:]=0.;braking=True;hard_brake=True
        if cfg.predictive_braking and snap is not None and now-snap.ts<cfg.map_stale_timeout:
            speed_actual=float(np.linalg.norm(state.velocity))
            if speed_actual>.5:
                stop_distance=speed_actual*cfg.brake_reaction+speed_actual**2/(2*cfg.brake_accel)+.8
                coast=state.position+np.linspace(0.,stop_distance,max(5,int(stop_distance/.2)+1))[:,None]*state.velocity/speed_actual
                predicted_min=float(clearances(coast,snap,cfg.res).min())
                if predicted_min<cfg.d_min+cfg.route_buffer:
                    target[:]=0.;braking=True
        if cfg.flight_altitude_max>0:
            room=max(0.,cfg.flight_altitude_max+state.position[2]-.8)
            upward_limit=np.sqrt(2*cfg.a_max*room)
            target[2]=max(target[2],-upward_limit)
            if state.position[2]<-cfg.flight_altitude_max:target[2]=max(target[2],.8)
        if cfg.flight_altitude_min>0:
            room=max(0.,-cfg.flight_altitude_min-state.position[2]-.8)
            target[2]=min(target[2],np.sqrt(2*cfg.a_max*room))
            if state.position[2]>-cfg.flight_altitude_min:target[2]=min(target[2],-.8)
        if hard_brake:
            # Do not add a second software ramp to AirSim's braking response.
            self._u[:]=0.;self._acc[:]=0.
        elif dt > 0:
            desired_a = (target-self._u)/max(dt,1e-3)
            desired_a *= min(1., cfg.a_max/max(np.linalg.norm(desired_a),1e-6))
            if cfg.safe_shortcuts and braking:
                self._acc=desired_a.copy()  # Safety braking takes priority over comfort jerk.
            da = desired_a-self._acc
            da *= min(1., cfg.jerk_max*dt/max(np.linalg.norm(da),1e-6))
            self._acc += da
            new_u = self._u+self._acc*dt
            if (target-self._u)@(target-new_u) < 0: new_u=target; self._acc[:]=0
            self._u = new_u
            self._u *= min(1., cfg.v_max/max(np.linalg.norm(self._u),1e-6))
        self.io.move_by_velocity(*self._u.tolist(), cfg.velocity_duration)
        return dict(u=self._u.tolist(), braking=bool(braking), clearance=self.min_clearance,
                    speed_limit=float(speed_limit),predicted_braking_clearance=predicted_min)


class SafeNavigationPipeline(NavigationPipeline):
    def __init__(self, io, cfg):
        super().__init__(io,cfg)
        self.planner.d_min=route_clearance(cfg)
        self.coordinator = RouteCoordinator(io,cfg)
        self._frame = None; self._map_version = 0
        self.approach_goal=None;self.reached_safe=False
        self._goal_lock=threading.Lock()
        self._approach_checked=0.
        from .world_guide import WorldGuide
        self.world_guide=WorldGuide(cfg) if cfg.use_world_guide else None
        from .progress_explorer import ProgressExplorer
        self.explorer=ProgressExplorer(cfg) if cfg.progress_exploration and not (cfg.goal_occupied_confirmed or cfg.goal_in_building_confirmed) else None
        self._guide_recorded=0;self._exploration_recorded=0
        self._historical_checked=not bool(self.persistent is not None and self.persistent.blocks)

    def _select_approach(self, proxy, started):
        # Arrival and endpoint updates share a lock. An accepted terminal goal
        # stays fixed while the mission monitor and recorder finish.
        with self._goal_lock:
            if self.reached:return False
            self.approach_goal=np.asarray(proxy,dtype=float).copy()
            self.coordinator.set_trajectory(None,started)
            return True

    def _check_arrival(self, state):
        with self._goal_lock:
            if state.collision or self.collided:return False
            if self.reached:return True
            active=self.approach_goal if self.approach_goal is not None else self.goal
            safe=self.approach_goal is None or (self._snapshot is not None and clearances([active],self._snapshot,self.cfg.res)[0]>=max(self.cfg.d_safe-.2,self.cfg.d_min+self.cfg.route_buffer+.8))
            if safe and np.linalg.norm(state.position-active)<self.cfg.goal_tol and np.linalg.norm(state.velocity)<1.:
                self.reached=True
                self.reached_safe=self.approach_goal is not None
                if self.reached_safe and self.persistent is not None and not self.collided:
                    self.persistent.remember_approach(self.goal,self.approach_goal)
                return True
            return False

    def stop(self):
        self._stop.set()
        try:self.io.move_by_velocity(0.,0.,0.,self.cfg.velocity_duration)
        finally:
            super().stop()
            # A control RPC already in flight could supersede the first stop.
            self.io.move_by_velocity(0.,0.,0.,self.cfg.velocity_duration)

    def _sensor_loop(self):
        period=1./self.cfg.sensor_hz
        while not self._stop.is_set():
            started=time.time()
            try:
                points, origin, stamp = self.io.get_lidar_frame()
                if len(points): self._frame=(points,origin,stamp,time.time())
                if self.recorder is not None: self.recorder.record_points(started,points)
            except Exception as exc:
                print('[Sensor]',repr(exc),flush=True)
            self._stop.wait(max(0.,period-(time.time()-started)))

    def _mapping_loop(self):
        cfg=self.cfg; last_stamp=None; last_seed=0.; last_merge=0.; last_save=time.time(); last_full=0.
        while not self._stop.is_set():
            started=time.time(); frame=self._frame; state=self._latest_state
            if frame is None or state is None or frame[2]==last_stamp:
                self._stop.wait(.03);continue
            try:
                pts,origin,stamp,received=frame; last_stamp=stamp
                self.map.recenter(state.position)
                if self.persistent is not None and started-last_seed>2.:
                    if cfg.fast_history:seed_history(self.persistent,self.map.data,self.map.origin,self.map.res)
                    else:self.persistent.seed_local(self.map.data,self.map.origin,self.map.res)
                    last_seed=started
                raycast.integrate(self.map,origin,pts,cfg)
                collision=self.map.derive_collision(state.position,np.radians(cfg.lidar_dtheta_h_deg),np.radians(cfg.lidar_dtheta_v_deg),cfg.drone_radius)
                dist=distance_field.compute(collision,cfg)
                if cfg.safe_shortcuts:
                    raw_dist=distance_field.compute(self.map,cfg)
                    self.coordinator.raw_snapshot=PlanSnapshot(self.map.data.copy(),self.map.origin.copy(),raw_dist,state.position.copy(),received)
                self._snapshot=PlanSnapshot(collision.data.copy(),collision.origin.copy(),dist,state.position.copy(),received if cfg.fast_history else time.time())
                self._map_version+=1
                if self.recorder is not None:
                    self._record_map_snapshot(state.position,collision)
                if self.persistent is not None and started-last_merge>2.:
                    if cfg.fast_history:merge_history(self.persistent,self.map.data,self.map.origin,self.map.res)
                    else:self.persistent.merge_from_local(self.map.data,self.map.origin,self.map.res)
                    last_merge=started
                if self.persistent is not None and cfg.persistent_full_sensor and started-last_full>=cfg.persistent_integrate_interval:
                    integrate_sensor_history(self.persistent,origin,pts,cfg)
                    last_full=started
                if self.persistent is not None and started-last_save>=cfg.persistent_save_interval:
                    self.persistent.save(self._persistent_path)
                    last_save=started
            except Exception:
                traceback.print_exc()
            self._stop.wait(max(.01,1./cfg.mapping_hz-(time.time()-started)))

    def _planning_loop(self):
        cfg=self.cfg
        while not self._stop.is_set():
            started=time.time(); snap=self._snapshot; state=self._latest_state
            if snap is None or state is None:
                self._stop.wait(.05);continue
            if self.reached:
                self._stop.wait(.05);continue
            if not self._historical_checked and cfg.allow_safe_approach and (cfg.goal_occupied_confirmed or cfg.goal_in_building_confirmed):
                self._historical_checked=True
                try:
                    from .world_guide import historical_approach
                    prior=historical_approach(self.persistent,state.position,self.goal,cfg)
                    if prior is not None and self._select_approach(prior,started):
                        if self.recorder is not None:self.recorder.record_event('safe_approach_selected',dict(t=started,original_goal=self.goal.tolist(),approach_goal=prior.tolist(),reason='validated first-flight endpoint / observed historical map'))
                        print('[SafeGoal] historical approach',prior,flush=True)
                except Exception:traceback.print_exc()
            if cfg.allow_safe_approach and (cfg.goal_occupied_confirmed or cfg.goal_in_building_confirmed) and np.linalg.norm(state.position-self.goal)<cfg.max_range+5. and started-self._approach_checked>1.5:
                self._approach_checked=started
                proxy=nearest_observed_approach(snap,state.position,self.goal,cfg)
                invalid_proxy=False
                if self.approach_goal is not None:
                    clearance,_,valid=distance_query([self.approach_goal],snap.dist,snap.origin,cfg.res)
                    invalid_proxy=bool(valid[0] and clearance[0]<max(cfg.d_safe-.2,cfg.d_min+cfg.route_buffer+.8))
                    if invalid_proxy:self.coordinator.set_trajectory(None,started)
                if proxy is not None and (self.approach_goal is None or invalid_proxy or np.linalg.norm(self.approach_goal-self.goal)-np.linalg.norm(proxy-self.goal)>1.5):
                    if self._select_approach(proxy,started):
                        if self.recorder is not None:self.recorder.record_event('safe_approach_selected',dict(t=time.time(),original_goal=self.goal.tolist(),approach_goal=proxy.tolist(),reason=('confirmed building interior' if cfg.goal_in_building_confirmed else 'confirmed occupied goal')+('; unsafe previous proxy replaced' if invalid_proxy else '; closer observed safe point')))
                        print('[SafeGoal] original',self.goal,'observed safe approach',proxy,flush=True)
            old=self.coordinator.traj
            if old is not None:
                idx=old.locate(state.position); s=old.arc[idx]; remaining=old.arc[-1]-s
                preview=np.array([old.at(a) for a in np.arange(s,min(s+18.,old.arc[-1])+.1,.4)])
                active=self.approach_goal if self.approach_goal is not None else self.goal
                committed=remaining>cfg.route_replan_distance or (cfg.safe_shortcuts and np.linalg.norm(old.points[-1]-active)<1.)
                tracking_error=np.linalg.norm(old.at(s)-state.position)
                if committed and tracking_error<2. and (clearances(preview,snap,cfg.res)>=route_clearance(cfg)+.1).all():
                    self._stop.wait(.15);continue
            try:
                coarse=self._coarsen(snap.occ,snap.origin,cfg.res,cfg.plan_res)
                pad=int(round(cfg.plan_pad/coarse.res));padz=int(round(cfg.plan_pad_z/coarse.res))
                if pad or padz: coarse=self._pad_unknown(coarse,pad,padz)
                if self.persistent is not None:
                    if cfg.fast_history:seed_history(self.persistent,coarse.data,coarse.origin,coarse.res)
                    else:self.persistent.seed_local(coarse.data,coarse.origin,coarse.res)
                heights=-(coarse.origin[2]+(np.arange(coarse.nz)+.5)*coarse.res)
                outside=np.zeros(coarse.nz,dtype=bool)
                if cfg.flight_altitude_min>0:outside|=heights<cfg.flight_altitude_min
                if cfg.flight_altitude_max>0:outside|=heights>cfg.flight_altitude_max
                coarse.data[:,:,outside]=2
                dist=distance_field.compute(coarse,cfg)
                active=self.approach_goal if self.approach_goal is not None else self.goal
                guidance=active
                if self.explorer is not None:
                    guidance=self.explorer.target(state.position,active,started)
                    if self.recorder is not None and self.explorer.events!=self._exploration_recorded:
                        self.recorder.record_event('exploration',dict(t=started,phase=self.explorer.phase,
                            waypoint=guidance.tolist(),original_goal=active.tolist(),reason='no progress during repeated detours'))
                        self._exploration_recorded=self.explorer.events
                        print('[Explore]',self.explorer.phase,'vantage',np.round(guidance,1),flush=True)
                if self.world_guide is not None:
                    guidance=self.world_guide.waypoint(self.persistent,self.coordinator.raw_snapshot,state.position,guidance,started,snap)
                    if self.recorder is not None and self.world_guide.plan_count!=self._guide_recorded:
                        self.recorder.record_event('world_guide',dict(t=started,path=self.world_guide.path[::2],
                            waypoint=guidance,replans=self.world_guide.plan_count,failures=self.world_guide.failure_count))
                        self._guide_recorded=self.world_guide.plan_count
                goal=self._local_goal(state.position,active,coarse.origin,coarse.res,coarse.nx,coarse.ny,coarse.nz)
                if cfg.safe_shortcuts:goal=ray_goal(state.position,guidance,coarse.origin,coarse.res,coarse.data.shape)
                goal=connected_local_goal(coarse,dist,state.position,goal,route_clearance(cfg)+.1,self.world_guide)
                path=self.planner.plan(coarse,dist,state.position,goal,v_dir=state.velocity,w_dir=cfg.w_dir)
                route=build_validated_route(path,state.position,coarse,dist,cfg) if path and len(path)>1 else None
                if route is not None:
                    route.final=np.linalg.norm(route.points[-1]-active)<1.
                    # Reject a stale activation start; retry from fresh physical state.
                    current=self._latest_state
                    if np.linalg.norm(current.position-state.position)<2.:
                        self.coordinator.set_trajectory(route,time.time())
                        if self.recorder is not None:
                            self.recorder.record_plan(time.time(),goal,path,route.points[::max(1,len(route.points)//300)],route.ctrl)
                        print(f'[SafePlan] route={route.arc[-1]:.1f}m start={np.round(state.position,1)} goal={np.round(goal,1)} compute={time.time()-started:.2f}s',flush=True)
                else:
                    dc,_,_=distance_query([state.position],dist,coarse.origin,coarse.res)
                    print('[SafePlan] no validated route',np.round(state.position,1),'coarse_clearance',np.round(dc,2),'astar_points',len(path) if path else 0,flush=True)
            except Exception:
                traceback.print_exc()
            self._stop.wait(max(.1,1./cfg.planning_hz-(time.time()-started)))

    def _control_loop(self):
        cfg=self.cfg; count=0
        while not self._stop.is_set():
            started=time.time()
            try:
                state=self.io.get_state(); self._latest_state=state
                self.coordinator.snapshot=self._snapshot
                if self._check_arrival(state):
                    self.io.move_by_velocity(0.,0.,0.,cfg.velocity_duration)
                    result=dict(u=[0.,0.,0.],braking=True,clearance=self.coordinator.min_clearance,speed_limit=0.,predicted_braking_clearance=None)
                else:result=self.coordinator.tick(state,time.time())
                self._record(state,result)
                with self._telemetry_lock:
                    self._telemetry[-1].update(sim_timestamp_ns=state.timestamp_ns,pitch=state.pitch,roll=state.roll,yaw=state.yaw,
                                               speed_limit=result['speed_limit'],map_age=time.time()-self._snapshot.ts if self._snapshot else None,
                                               raw_clearance=float(clearances([state.position],self.coordinator.raw_snapshot,cfg.res)[0]) if self.coordinator.raw_snapshot is not None else None,
                                               predicted_braking_clearance=result['predicted_braking_clearance'],
                                               exploration_phase=self.explorer.phase if self.explorer is not None else 'direct',
                                               approach_goal=self.approach_goal.tolist() if self.approach_goal is not None else None,
                                               approach_dist=float(np.linalg.norm(state.position-self.approach_goal)) if self.approach_goal is not None else None)
                if self.recorder is not None:
                    self.recorder.record_state(time.time(),state.position,state.velocity,state.acceleration,state.yaw,
                        command=result['u'],reference_speed=cfg.v_mission,speed_limit=result['speed_limit'],
                        approach_goal=self.approach_goal,sim_timestamp_ns=state.timestamp_ns,pitch=state.pitch,roll=state.roll)
                if state.collision:self.collided=True
                count+=1
                if count%60==1:
                    print(f'[SafeControl] p={np.round(state.position,2)} v={np.linalg.norm(state.velocity):.2f} dist={np.linalg.norm(state.position-self.goal):.1f} clearance={result["clearance"]:.1f} brake={result["braking"]}',flush=True)
            except Exception:
                traceback.print_exc()
            self._stop.wait(max(.001,1./cfg.control_hz-(time.time()-started)))
