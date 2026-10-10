"""只做评估：按时间插值 Gazebo 真值，不提供任何控制／定位输出。"""
import bisect
import math
from collections import deque
from .core import wrap

class TruthSeries:
    def __init__(self,max_gap=.1):self.samples=deque(maxlen=500);self.max_gap=max_gap
    def add(self,stamp,pose):
        if not math.isfinite(stamp) or stamp<=0 or not all(math.isfinite(x) for x in pose):return False
        if self.samples and stamp<=self.samples[-1][0]:return False
        self.samples.append((stamp,tuple(pose)));return True
    def at(self,stamp):
        if not self.samples:return None
        samples=list(self.samples);times=[s[0] for s in samples];index=bisect.bisect_left(times,stamp)
        if index<len(samples) and abs(times[index]-stamp)<1e-8:return samples[index][1]
        if index==0 or index==len(samples):return None
        t0,p0=samples[index-1];t1,p1=samples[index]
        if t1-t0>self.max_gap+1e-9:return None
        f=(stamp-t0)/(t1-t0)
        return (p0[0]+f*(p1[0]-p0[0]),p0[1]+f*(p1[1]-p0[1]),wrap(p0[2]+f*wrap(p1[2]-p0[2])))

class ErrorStats:
    def __init__(self):self.count=0;self.xy2=0.;self.yaw2=0.;self.xymax=0.;self.yawmax=0.
    def add(self,estimate,truth):
        xy=math.dist(estimate[:2],truth[:2]);angle=abs(wrap(estimate[2]-truth[2]))
        self.count+=1;self.xy2+=xy*xy;self.yaw2+=angle*angle
        self.xymax=max(self.xymax,xy);self.yawmax=max(self.yawmax,angle)
        return xy,angle
    def report(self):
        return {'samples':self.count,'position_rmse_m':math.sqrt(self.xy2/self.count) if self.count else None,
                'yaw_rmse_rad':math.sqrt(self.yaw2/self.count) if self.count else None,
                'position_max_m':self.xymax if self.count else None,'yaw_max_rad':self.yawmax if self.count else None}
