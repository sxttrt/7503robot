from pathlib import Path
import sys, math
import numpy as np
from types import SimpleNamespace as NS
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'navigation/scripts'))
import scene
from grid_planner import GridMap, Planner

def replay(poses):
    res=.025;ox=-.5;oy=-2.;w=h=240
    data=np.full((h,w),255,dtype=np.uint8)
    obstacles=[r for name in scene.RACKS for r in scene.rack_leg_rects(name)]
    obstacles += [(-.06,0,0,2),(3.02,3.08,0,2),(-.06,3.08,-.06,0),(-.06,3.08,2,2.06)]
    for x,y,yaw in poses:
        for a in np.linspace(-math.pi,math.pi,720)+yaw:
            dx,dy=math.cos(a),math.sin(a);dist=8.
            for x0,x1,y0,y1 in obstacles:
                tx=sorted(((x0-x)/dx,(x1-x)/dx)) if abs(dx)>1e-12 else [-math.inf,math.inf] if x0<=x<=x1 else [math.inf,math.inf]
                ty=sorted(((y0-y)/dy,(y1-y)/dy)) if abs(dy)>1e-12 else [-math.inf,math.inf] if y0<=y<=y1 else [math.inf,math.inf]
                lo=max(tx[0],ty[0],0);hi=min(tx[1],ty[1])
                if hi>=lo:dist=min(dist,lo)
            # Bresenham costmap clearing, then mark the measured endpoint.
            i,j=int((x-ox)/res),int((y-oy)/res)
            ei,ej=int((x+dist*dx-ox)/res),int((y+dist*dy-oy)/res)
            di,dj=abs(ei-i),abs(ej-j);si=1 if i<ei else -1;sj=1 if j<ej else -1;err=di-dj
            while True:
                if 0<=i<w and 0<=j<h:data[j,i]=0
                if i==ei and j==ej:break
                e=2*err
                if e>-dj:err-=dj;i+=si
                if e<di:err+=di;j+=sj
            if 0<=ei<w and 0<=ej<h:data[ej,ei]=254
    meta=NS(resolution=res,size_x=w,size_y=h,origin=NS(position=NS(x=ox,y=oy),orientation=NS(x=0,y=0,z=0)))
    return meta,data

if __name__=='__main__':
    poses=[(2.72-i*.008,1.,math.pi) for i in range(21)]
    poses += [(2.559,1.,a) for a in np.linspace(math.pi,math.pi+math.pi/6,80)]
    meta,data=replay(poses)
    planner=Planner(GridMap(meta,data.ravel()),(.1232,.0848),(3.02,2),margin=.05)
    start=(2.559,1);goal=scene.scan('A')
    print('replayed goal:',goal,'blocked:',planner.segment_blocked_reason(goal,goal))
    x,y=goal
    crop=data[int((y-.1348-meta.origin.position.y)/.025):int((y+.1348-meta.origin.position.y)/.025)+1,int((x-.1732-meta.origin.position.x)/.025):int((x+.1732-meta.origin.position.x)/.025)+1]
    print('goal footprint: lethal=',int((crop==254).sum()),'unknown=',int((crop==255).sum()))
    print('full route:',planner.plan(start,goal))
    for y in (.8,.75,.7,.65):print('observed staging point:',(goal[0],y),planner.plan(start,(goal[0],y)))
