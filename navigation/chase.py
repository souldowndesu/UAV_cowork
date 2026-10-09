"""Causal moving-target prediction; no access to a preset's future points."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class ChaseOptions:
    strategy: str = 'predict'
    lead_max: float = 3.0
    lead_min: float = .3
    stand_off: float = 3.5
    vertical_offset: float = 2.0
    stale_after: float = 1.5
    velocity_tau: float = .35
    replan_shift: float = 4.0
    replan_interval: float = 1.2
    capture_radius: float = 5.0
    capture_hold: float = 2.0
    stand_off_mode: str = 'intercept'


class TargetTracker:
    def __init__(self, options):
        self.options=options;self.position=None;self.velocity=np.zeros(3);self.stamp=None
        self.turn_rate=0.;self.updates=0
        self.follow_direction=None
        self.follow_side=None

    def observe(self, position, velocity, stamp):
        p=np.asarray(position,dtype=float);v=np.asarray(velocity,dtype=float)
        if p.shape!=(3,) or v.shape!=(3,) or not np.isfinite(np.r_[p,v,stamp]).all():
            raise ValueError('Target observation must be finite')
        if self.stamp is not None and stamp<=self.stamp:return False
        speed=np.linalg.norm(v)
        if speed>12:v=v*12/speed
        if self.stamp is None:self.velocity=v.copy()
        else:
            dt=stamp-self.stamp;old=self.velocity.copy()
            alpha=1-math.exp(-dt/self.options.velocity_tau)
            self.velocity+=alpha*(v-self.velocity)
            if np.linalg.norm(old)>.6 and np.linalg.norm(self.velocity)>.6:
                cosine=np.clip(old@self.velocity/(np.linalg.norm(old)*np.linalg.norm(self.velocity)),-1.,1.)
                self.turn_rate=.7*self.turn_rate+.3*math.acos(cosine)/max(dt,.01)
            else:self.turn_rate*=.7
        self.position=p.copy();self.stamp=float(stamp);self.updates+=1
        return True

    def predict(self, pursuer, now, cruise):
        if self.position is None:return None
        age=max(0.,now-self.stamp)
        if age>self.options.stale_after:return None
        velocity=self.velocity.copy()
        current=self.position+velocity*age if self.options.strategy!='pure' else self.position.copy()
        offset=current-np.asarray(pursuer)
        lead=0.
        if self.options.strategy=='predict':
            a=velocity@velocity-cruise**2;b=2*(offset@velocity);c=offset@offset
            discriminant=b*b-4*a*c
            roots=[]
            if abs(a)<1e-6:
                if abs(b)>1e-6:roots=[-c/b]
            elif discriminant>=0:
                roots=[(-b-math.sqrt(discriminant))/(2*a),(-b+math.sqrt(discriminant))/(2*a)]
            positive=[r for r in roots if r>0]
            intercept=min(positive) if positive else np.linalg.norm(offset)/max(1.,cruise)
            lead=float(np.clip(intercept,self.options.lead_min,self.options.lead_max))
            lead/=1.+2.5*self.turn_rate
            # Little lookahead when already following, rather than orbiting ahead.
            lead*=min(1.,max(0.,np.linalg.norm(offset)-self.options.stand_off)/12.)
        predicted=current+velocity*lead
        horizontal=velocity.copy();horizontal[2]=0.
        speed=np.linalg.norm(horizontal)
        if speed>.5:
            heading=horizontal/speed
            projection=-float(offset@heading)  # pursuer ahead (+) or behind (-)
            if self.options.stand_off_mode=='trailing':self.follow_side=1.
            elif self.follow_side is None:self.follow_side=-1. if projection>0 else 1.
            elif abs(projection)>self.options.stand_off+2.5:
                self.follow_side=-1. if projection>0 else 1.
            self.follow_direction=heading*self.follow_side
        elif self.follow_direction is None:
            direction=offset.copy();direction[2]=0.
            self.follow_direction=direction/max(np.linalg.norm(direction),1e-6)
        direction=self.follow_direction
        goal=predicted-direction*self.options.stand_off+np.array([0.,0.,-self.options.vertical_offset])
        follow_goal=current-direction*self.options.stand_off+np.array([0.,0.,-self.options.vertical_offset])
        return dict(position=current,velocity=velocity,goal=goal,follow_goal=follow_goal,lead=lead,age=age,stamp=now)
