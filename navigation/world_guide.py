"""Persistent observations guide local planning; execution still uses the live map."""
from dataclasses import replace
import numpy as np
from .occupancy import OccupancyMap
from .astar import AStarPlanner
from . import distance_field
from .geometry import distance_query


def historical_approach(history, position, goal, cfg):
    """Reuse a reached endpoint, or improve it using observed map candidates."""
    if history is None or not history.blocks:return None
    from .safe_navigation import nearest_observed_approach
    from .pipeline import PlanSnapshot
    from scipy import ndimage
    world,dist=planning_world(history,None,position,goal,cfg)
    snap=PlanSnapshot(world.data,world.origin,dist,np.asarray(position),0.)
    candidate=nearest_observed_approach(snap,np.asarray(position),np.asarray(goal),replace(cfg,res=world.res,plan_res=world.res))
    anchor=history.approach_for(goal)
    if anchor is not None:
        ds,_,valid=distance_query([anchor],dist,world.origin,world.res)
        labels,_=ndimage.label((world.data!=2)&(dist>=cfg.guide_clearance))
        start=world.world_to_voxel(position);end=world.world_to_voxel(anchor)
        reachable=world.in_bounds(*start) and world.in_bounds(*end) and labels[start]!=0 and labels[start]==labels[end]
        observed=history.query_voxel(*history.world_to_voxel(anchor))==1
        if valid[0] and ds[0]>=cfg.guide_clearance and reachable and observed:
            if candidate is None or np.linalg.norm(candidate-goal)>=np.linalg.norm(anchor-goal)-1.5:
                candidate=anchor
    radius=cfg.approach_max_distance if cfg.approach_max_distance>0 else cfg.max_range+8.
    if candidate is not None and np.linalg.norm(candidate-goal)<=radius:
        return candidate
    return None


def planning_world(history, raw, position, goal, cfg, collision=None):
    res=cfg.guide_res
    low=np.minimum(position,goal)-np.array([cfg.guide_margin,cfg.guide_margin,12.])
    high=np.maximum(position,goal)+np.array([cfg.guide_margin,cfg.guide_margin,12.])
    # Never fabricate observed free space outside the sensor window.
    low[2]=min(low[2],-cfg.guide_altitude_max)
    high[2]=max(high[2],-cfg.guide_altitude_min)
    low=np.floor(low/res)*res
    shape=np.ceil((high-low)/res).astype(int)
    world=OccupancyMap(*shape,res,tuple(low))
    axes=[np.floor((low[a]+(np.arange(shape[a])+.5)*res-history.origin[a])/history.res).astype(int) for a in range(3)] if history else None
    if history:
        # Query sparse blocks; occupied samples are also splatted below so a
        # coarse cell cannot miss a thin wall between its query centres.
        block_axes=[np.floor_divide(a,history.block) for a in axes]
        for key,block in list(history.blocks.items()):
            ix=[np.flatnonzero(block_axes[a]==key[a]) for a in range(3)]
            if any(len(i)==0 for i in ix):continue
            world.data[np.ix_(*ix)]=block[np.ix_(*[axes[a][ix[a]]%history.block for a in range(3)])]
        hits=history.occupied_points()
        indices=np.floor((hits-low)/res).astype(int)
        inside=((indices>=0)&(indices<shape)).all(axis=1)
        if inside.any():world.data[tuple(indices[inside].T)]=2
    if raw is not None:
        known=np.argwhere(raw.occ!=0)
        points=raw.origin+(known+.5)*cfg.res
        indices=np.floor((points-low)/res).astype(int)
        inside=((indices>=0)&(indices<shape)).all(axis=1)
        indices=indices[inside];values=raw.occ[tuple(known[inside].T)]
        batch=np.zeros_like(world.data)
        np.maximum.at(batch,tuple(indices.T),values)
        # Latest measured cells override old observations, occupied wins
        # within a coarse cell. Unknown remains implicit, never overwrites.
        touched=batch!=0;world.data[touched]=batch[touched]
    world.inflate_fixed(cfg.drone_radius)
    if collision is not None:
        occupied=np.argwhere(collision.occ==2)
        cells=np.floor((collision.origin+(occupied+.5)*cfg.res-low)/res).astype(int)
        inside=((cells>=0)&(cells<shape)).all(axis=1)
        if inside.any():world.data[tuple(cells[inside].T)]=2
    heights=-(low[2]+(np.arange(shape[2])+.5)*res)
    world.data[:,:,(heights<cfg.guide_altitude_min)|(heights>cfg.guide_altitude_max)]=2
    return world,distance_field.compute(world,cfg)


class WorldGuide:
    def __init__(self,cfg):
        self.cfg=cfg;self.path=None;self.arc=None;self.last_update=-np.inf
        self.target=None;self.plan_count=0;self.failure_count=0
        self._progress=0;self._position=None
        self.planner=AStarPlanner(replace(cfg,d_min=cfg.guide_clearance,h_weight=1.1))

    def locate(self,position):
        moved=0. if self._position is None else float(np.linalg.norm(position-self._position))
        previous=self._progress
        candidates=np.flatnonzero((self.arc>=self.arc[previous]-2.)&(self.arc<=self.arc[previous]+max(6.,moved+4.)))
        nearest=int(candidates[np.argmin(np.linalg.norm(self.path[candidates]-position,axis=1))])
        self._progress=max(previous,nearest);self._position=np.asarray(position).copy()
        return nearest

    def waypoint(self,history,raw,position,goal,now,collision=None):
        cfg=self.cfg
        changed=self.target is None or np.linalg.norm(goal-self.target)>.5
        if changed:self.path=None;self.target=np.asarray(goal).copy()
        if now-self.last_update>=cfg.guide_interval or changed:
            self.last_update=now
            world,dist=planning_world(history,raw,position,goal,cfg,collision)
            reusable=False
            if self.path is not None:
                nearest=self.locate(position)
                suffix=self.path[nearest:]
                ds,_,valid=distance_query(suffix,dist,world.origin,world.res)
                reusable=valid.all() and (ds>=cfg.guide_clearance).all() and np.linalg.norm(self.path[nearest]-position)<5.
            if not reusable:
                cost_dist=dist
                if cfg.guide_altitude_cost>0:
                    heights=-(world.origin[2]+(np.arange(world.nz)+.5)*world.res)
                    preferred=max(cfg.flight_altitude,-goal[2])+2.
                    penalty=cfg.guide_altitude_cost*np.maximum(0.,heights-preferred)
                    # A* uses exponential clearance cost. Encode a height
                    # preference in that soft cost, keeping the hard cutoff
                    # above guide_clearance. Physical validation uses dist.
                    soft=np.maximum(cfg.guide_clearance+.1,cfg.d_safe-np.log1p(penalty))
                    cost_dist=np.minimum(dist,soft[None,None,:]).astype(np.float32)
                path=self.planner.plan(world,cost_dist,position,goal)
                if path:
                    self.path=np.vstack([position,path])
                    self.arc=np.r_[0.,np.linalg.norm(np.diff(self.path,axis=0),axis=1).cumsum()]
                    self._progress=0;self._position=np.asarray(position).copy()
                    self.plan_count+=1
                else:
                    self.path=None;self.failure_count+=1
        if self.path is None:return np.asarray(goal)
        nearest=self.locate(position)
        remaining=self.arc[-1]-self.arc[nearest]
        # Refine the exact goal in the fine live map only after traversing the
        # remaining detour; distance-to-goal alone can abandon a wall detour.
        if remaining<cfg.guide_lookahead and np.linalg.norm(self.path[-1]-goal)<8.:
            return np.asarray(goal)
        at=min(self.arc[-1],self.arc[nearest]+cfg.guide_lookahead)
        return np.array([np.interp(at,self.arc,self.path[:,a]) for a in range(3)])
