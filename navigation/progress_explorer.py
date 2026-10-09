"""Commit to an observation vantage when repeated local detours make no progress."""
import numpy as np


class ProgressExplorer:
    def __init__(self,cfg):
        self.cfg=cfg;self.phase='direct';self.waypoint=None
        self.best=float('inf');self.last_progress=None;self.active=None;self.events=0

    def target(self,position,goal,now):
        cfg=self.cfg;position=np.asarray(position);goal=np.asarray(goal)
        if self.active is None or np.linalg.norm(goal-self.active)>1.:
            self.active=goal.copy();self.phase='direct';self.waypoint=None
            self.best=float('inf');self.last_progress=now
        target=goal if self.waypoint is None else self.waypoint
        distance=float(np.linalg.norm(position-target))
        if distance<self.best-2.:
            self.best=distance;self.last_progress=now
        if self.phase=='ascend' and distance<2.5:
            self.phase='cross';self.waypoint=np.r_[goal[:2],self.waypoint[2]]
            self.best=float('inf');self.last_progress=now
        elif self.phase=='cross' and np.linalg.norm(position[:2]-goal[:2])<6.:
            self.phase='descend';self.waypoint=None
            self.best=float('inf');self.last_progress=now
        if now-self.last_progress>cfg.explore_timeout and np.linalg.norm(position-goal)>5.:
            height=min(cfg.guide_altitude_max-2.,-position[2]+cfg.explore_height_step)
            if height>-position[2]+3.:
                self.phase='ascend';self.waypoint=np.r_[position[:2],-height]
                self.best=float('inf');self.last_progress=now;self.events+=1
        return goal if self.waypoint is None else self.waypoint
