"""2D scan-to-fixed-wall localization. No ROS, commands, IMU or pose truth.

The known rectangular field and configured initial pose fix the coordinate gauge.
Only scan returns close to fixed boundary surfaces participate in the fit.
"""
import math
import numpy as np


def wrap(a):
    return math.atan2(math.sin(a),math.cos(a))


def scan_points(ranges,angle_min,angle_increment,range_min,range_max):
    if (not all(math.isfinite(v) for v in (angle_min,angle_increment,range_min,range_max)) or
            angle_increment==0 or abs(angle_increment)>2*math.pi or
            range_min<0 or range_max<=range_min):
        raise ValueError('invalid scan geometry/range limits')
    ranges=np.asarray(ranges,dtype=float)
    if ranges.ndim!=1:raise ValueError('scan ranges must be one dimensional')
    angles=angle_min+np.arange(len(ranges))*angle_increment
    valid=np.isfinite(ranges)&(ranges>range_min)&(ranges<range_max)
    return np.column_stack((ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid])))


class WallLocalizer:
    def __init__(self,field,initial_pose):
        self.field=tuple(field)
        self.pose=np.asarray(initial_pose,dtype=float)
        self.last_stamp=None
        self.history=[]
        self.velocity=np.zeros(3)

    def correspondences(self,points,pose,gate=.10):
        c,s=math.cos(pose[2]),math.sin(pose[2])
        rotated=points@np.array([[c,s],[-s,c]])
        world=rotated+pose[:2]
        width,height=self.field
        distances=np.column_stack((world[:,0],world[:,0]-width,world[:,1],world[:,1]-height))
        wall=np.argmin(np.abs(distances),axis=1)
        residual=distances[np.arange(len(points)),wall]
        along=np.where(wall<2,world[:,1],world[:,0])
        extent=np.where(wall<2,height,width)
        valid=(np.abs(residual)<gate)&(along>=-.01)&(along<=extent+.01)
        rotated=rotated[valid];wall=wall[valid];residual=residual[valid]
        jacobian=np.zeros((len(wall),3))
        vertical=wall<2
        jacobian[vertical,0]=1.;jacobian[~vertical,1]=1.
        jacobian[:,2]=np.where(vertical,-rotated[:,1],rotated[:,0])
        return residual,jacobian,wall

    def update(self,points,stamp):
        if not math.isfinite(stamp):raise ValueError('invalid scan stamp')
        if self.last_stamp is not None and stamp<=self.last_stamp:
            raise ValueError('duplicate or out-of-order scan')
        # 本算法要求 points 已在 base_link 平面；仿真雷达 x/y/yaw 为零。
        # 实机雷达有偏移/偏航时由定位入口先应用外参，单发 TF 不会自动变换此数组。
        points=np.asarray(points,dtype=float)
        points=points[np.all(np.isfinite(points),axis=1)]
        if len(points)<60:raise ValueError('too few scan points')
        dt=0. if self.last_stamp is None else stamp-self.last_stamp
        guess=self.pose.copy()
        # Prediction is obtained only from previously accepted scan estimates.
        if 0<dt<.5:guess+=self.velocity*dt
        for _ in range(15):
            residual,jac,wall=self.correspondences(points,guess)
            if len(residual)<60:raise ValueError('too few fixed-wall returns')
            weights=1./(1.+(residual/.015)**2)
            weighted=jac*np.sqrt(weights)[:,None]
            if np.linalg.matrix_rank(weighted)<3:raise ValueError('unobservable wall geometry')
            delta=np.linalg.lstsq(weighted,-residual*np.sqrt(weights),rcond=None)[0]
            if np.linalg.norm(delta[:2])>.12 or abs(delta[2])>.12:
                raise ValueError('scan correction outside capture range')
            guess+=delta;guess[2]=wrap(guess[2])
            if np.linalg.norm(delta)<1e-7:break
        residual,jac,wall=self.correspondences(points,guess,gate=.025)
        vertical=wall<2
        if len(residual)<60 or np.sum(vertical)<15 or np.sum(~vertical)<15:
            raise ValueError('insufficient independent fixed walls')
        information=jac.T@jac
        eigen=np.linalg.eigvalsh(information)
        if eigen[0]/eigen[-1]<1e-4:raise ValueError('degenerate scan geometry')
        rms=float(np.sqrt(np.mean(residual**2)))
        if rms>.012:raise ValueError('wall residual too large')
        if not (0<guess[0]<self.field[0] and 0<guess[1]<self.field[1]):
            raise ValueError('estimated pose outside field')
        change=guess-self.pose;change[2]=wrap(change[2])
        if self.last_stamp is not None:
            if dt>.6:raise ValueError('scan gap requires stationary relocalization')
            if np.linalg.norm(change[:2])>.45*dt+.015 or abs(change[2])>.6*dt+.02:
                raise ValueError('scan pose jump rejected')
        self.pose=guess;self.last_stamp=stamp
        self.history.append((stamp,guess.copy()))
        self.history=[item for item in self.history if stamp-item[0]<=.4]
        # Regression suppresses scan noise in stop detection without command input.
        self.velocity=np.zeros(3)
        if len(self.history)>=3:
            times=np.array([item[0] for item in self.history]);times-=times.mean()
            values=np.array([item[1] for item in self.history])
            values[:,2]=np.unwrap(values[:,2])
            self.velocity=np.sum(times[:,None]*values,axis=0)/np.sum(times**2)
        covariance=max(rms**2,1e-6)*np.linalg.inv(information)
        return dict(pose=guess.copy(),velocity=self.velocity.copy(),covariance=covariance,
                    inliers=len(residual),rms=rms)

    def recover_after_gap(self):
        # Retain last scan pose, never reset to simulator truth or commanded pose.
        self.last_stamp=None;self.history=[];self.velocity=np.zeros(3)
