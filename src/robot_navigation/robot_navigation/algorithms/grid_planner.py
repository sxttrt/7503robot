"""Pure rectilinear planner with exact endpoints and swept footprint checks."""
import bisect
import heapq
import math
import numpy as np


def overlap(a, b):
    return a[0] < b[1] and b[0] < a[1] and a[2] < b[3] and b[2] < a[3]


def polygon_hits_rect(poly, rect):
    other = [(rect[0], rect[2]), (rect[1], rect[2]), (rect[1], rect[3]), (rect[0], rect[3])]
    axes = [(1., 0.), (0., 1.)]
    for a, b in zip(poly, poly[1:] + poly[:1]):
        axes.append((a[1]-b[1], b[0]-a[0]))
    for ax, ay in axes:
        first = [x*ax+y*ay for x,y in poly]
        second = [x*ax+y*ay for x,y in other]
        if max(first) < min(second) or max(second) < min(first):
            return False
    return True


class GridMap:
    def __init__(self, meta, data, map_from_world=(0., 0., 0.), *, unknown_is_free=False):
        self.res = float(meta.resolution)
        self.ox, self.oy = meta.origin.position.x, meta.origin.position.y
        self.w, self.h = int(meta.size_x), int(meta.size_y)
        if (not all(math.isfinite(v) for v in (self.res,self.ox,self.oy)) or
                self.res <= 0 or self.w<=0 or self.h<=0 or len(data) != self.w*self.h):
            raise ValueError("invalid costmap dimensions")
        q = meta.origin.orientation
        if not all(math.isfinite(v) for v in (q.x,q.y,q.z)) or abs(q.x)+abs(q.y)+abs(q.z) > 1e-6:
            raise ValueError("rotated costmap origins are unsupported")
        self.tx, self.ty, yaw = map_from_world
        self.c, self.s = math.cos(yaw), math.sin(yaw)
        self.costs = np.asarray(data, dtype=np.uint8).reshape(self.h,self.w).copy()
        if type(unknown_is_free) is not bool:raise ValueError('unknown_is_free must be boolean')
        self.unknown_is_free=unknown_is_free
        # Preserve raw 255 for diagnostics; never turn observed 254 into free.
        # Planner's field and swept-footprint bounds apply to every segment.
        self.blocked = (self.costs == 254) | ((self.costs == 255) & (not unknown_is_free))
        self.prefix = np.pad(self.blocked.astype(np.int32).cumsum(0).cumsum(1), ((1,0),(1,0)))

    def blocking_world_rects(self, rect):
        """Occupied cells hitting a world sweep, transformed back to world.

        Used to retain witnessed blockers across map changes during one goal.
        Rotation uses conservative cell bounding boxes; no occupied cell is erased.
        """
        x0,x1,y0,y1=rect
        poly=[(self.tx+self.c*x-self.s*y,self.ty+self.s*x+self.c*y)
              for x,y in ((x0,y0),(x1,y0),(x1,y1),(x0,y1))]
        i0=max(0,math.floor((min(x for x,y in poly)-self.ox)/self.res))
        i1=min(self.w,math.floor((max(x for x,y in poly)-self.ox)/self.res)+1)
        j0=max(0,math.floor((min(y for x,y in poly)-self.oy)/self.res))
        j1=min(self.h,math.floor((max(y for x,y in poly)-self.oy)/self.res)+1)
        result=[]
        for jj,ii in np.argwhere(self.blocked[j0:j1,i0:i1]):
            i,j=i0+int(ii),j0+int(jj)
            cell=(self.ox+i*self.res,self.ox+(i+1)*self.res,
                  self.oy+j*self.res,self.oy+(j+1)*self.res)
            if not polygon_hits_rect(poly,cell):continue
            corners=[(self.c*(x-self.tx)+self.s*(y-self.ty),
                      -self.s*(x-self.tx)+self.c*(y-self.ty))
                     for x,y in ((cell[0],cell[2]),(cell[1],cell[2]),
                                 (cell[1],cell[3]),(cell[0],cell[3]))]
            result.append(tuple(round(v,8) for v in
                          (min(x for x,y in corners),max(x for x,y in corners),
                           min(y for x,y in corners),max(y for x,y in corners))))
        return result

    def box_free(self, rect):
        x0,x1,y0,y1 = rect
        poly = [(self.tx+self.c*x-self.s*y, self.ty+self.s*x+self.c*y)
                for x,y in ((x0,y0),(x1,y0),(x1,y1),(x0,y1))]
        left, right = min(x for x,y in poly), max(x for x,y in poly)
        low, high = min(y for x,y in poly), max(y for x,y in poly)
        if left < self.ox or low < self.oy or right > self.ox+self.w*self.res or high > self.oy+self.h*self.res:
            return False
        i0=max(0,math.floor((left-self.ox)/self.res))
        i1=min(self.w,math.floor((right-self.ox)/self.res)+1)
        j0=max(0,math.floor((low-self.oy)/self.res))
        j1=min(self.h,math.floor((high-self.oy)/self.res)+1)
        p=self.prefix
        if p[j1,i1]-p[j0,i1]-p[j1,i0]+p[j0,i0] == 0:
            return True
        for jj,ii in np.argwhere(self.blocked[j0:j1,i0:i1]):
            i,j=i0+int(ii),j0+int(jj)
            cell=(self.ox+i*self.res,self.ox+(i+1)*self.res,
                  self.oy+j*self.res,self.oy+(j+1)*self.res)
            if polygon_hits_rect(poly,cell):
                return False
        return True


class Planner:
    def __init__(self, grid, half_size, field, obstacles=(), margin=.005, step=.025):
        self.grid=grid
        self.hx,self.hy=half_size
        self.field=field
        self.obstacles=list(obstacles)
        self.margin=margin
        self.step=step

    def segment_free(self, a, b):
        return self.segment_blocked_reason(a,b) is None

    def segment_blocked_reason(self, a, b):
        if abs(a[0]-b[0])>1e-8 and abs(a[1]-b[1])>1e-8:
            return 'non-axis-aligned segment'
        mx,my=self.hx+self.margin,self.hy+self.margin
        box=(min(a[0],b[0])-mx,max(a[0],b[0])+mx,
             min(a[1],b[1])-my,max(a[1],b[1])+my)
        if box[0]<0 or box[2]<0 or box[1]>self.field[0] or box[3]>self.field[1]:
            return f'field boundary box={box}'
        for rect in self.obstacles:
            if overlap(box,rect):return f'scene obstacle rect={rect} box={box}'
        if not self.grid.box_free(box):return f'radar costmap lethal/unknown/outside box={box}'
        return None

    def plan(self,start,goal):
        start,goal=tuple(start),tuple(goal)
        if not self.segment_free(start,start) or not self.segment_free(goal,goal):
            return None
        if math.dist(start,goal)<1e-9:
            return []
        direct=[]
        for middle in ((goal[0],start[1]),(start[0],goal[1])):
            if self.segment_free(start,middle) and self.segment_free(middle,goal):
                pts=[p for p in (middle,goal) if p!=start]
                if len(pts)==2 and pts[0]==pts[1]:pts.pop()
                direct.append(pts)
        if direct:return min(direct,key=len)
        xs=sorted(set([start[0],goal[0]]+[round(i*self.step,8) for i in range(1,math.ceil(self.field[0]/self.step))]))
        ys=sorted(set([start[1],goal[1]]+[round(i*self.step,8) for i in range(1,math.ceil(self.field[1]/self.step))]))
        s=(bisect.bisect_left(xs,start[0]),bisect.bisect_left(ys,start[1]),-1)
        g=(bisect.bisect_left(xs,goal[0]),bisect.bisect_left(ys,goal[1]))
        def point(k):return xs[k[0]],ys[k[1]]
        def heuristic(k):return abs(xs[k[0]]-goal[0])+abs(ys[k[1]]-goal[1])
        best={s:(0.,0)}
        queue=[(heuristic(s),0,0.,s)]
        came={};edges={};end=None
        while queue:
            _f,turns,length,k=heapq.heappop(queue)
            if best.get(k)!=(length,turns):continue
            if k[:2]==g:end=k;break
            for direction,(dx,dy) in enumerate(((1,0),(-1,0),(0,1),(0,-1))):
                n=(k[0]+dx,k[1]+dy,direction)
                if not (0<=n[0]<len(xs) and 0<=n[1]<len(ys)):continue
                edge=tuple(sorted((k[:2],n[:2])))
                if edge not in edges:edges[edge]=self.segment_free(point(k),point(n))
                if not edges[edge]:continue
                distance=round(length+math.dist(point(k),point(n)),8)
                nt=turns+int(k[2]!=-1 and k[2]!=direction)
                if (distance,nt)<best.get(n,(math.inf,math.inf)):
                    best[n]=(distance,nt);came[n]=k
                    heapq.heappush(queue,(round(distance+heuristic(n),8),nt,distance,n))
        if end is None:return None
        nodes=[end]
        while nodes[-1]!=s:nodes.append(came[nodes[-1]])
        nodes.reverse();result=[]
        for i in range(1,len(nodes)):
            if i==len(nodes)-1 or nodes[i][2]!=nodes[i+1][2]:result.append(point(nodes[i]))
        return result
