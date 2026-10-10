"""遮挡地图的分段规划；候选拓扑不下发，执行段必须全部通过原地图检查。"""
import copy
import math
import numpy as np


def observed_route(planner, start, goal, reserve=.05):
    """返回 (已观测安全路径, 是否到最终目标)。无安全进展返回 (None, False)。

    先尝试正常全程规划。未知格仅在内部候选拓扑中视为可探索；254 障碍、
    场地边界和完整载货/空车包络始终保留。只截取原地图确认畅通的前缀，
    在未知边界前留停车余量；停稳并收到新的扫描地图后才能再次调用。
    """
    route=planner.plan(start,goal)
    if route is not None:return route,True
    # Permissive in-field planning has already tested unknown-as-free. Failure
    # now means known obstacles/bounds; frontier staging cannot repair it.
    if getattr(planner.grid,'unknown_is_free',False):return None,False
    if not planner.segment_free(start,start):return None,False
    # 没有未知格时不能用探索掩盖真实碰撞或几何不通。
    if not np.any(planner.grid.costs==255):return None,False
    topology=copy.copy(planner);topology.grid=copy.copy(planner.grid)
    topology.grid.blocked=topology.grid.costs==254
    topology.grid.prefix=np.pad(topology.grid.blocked.astype(np.int32).cumsum(0).cumsum(1),((1,0),(1,0)))
    candidate=topology.plan(start,goal)
    if candidate is None:return None,False
    step=min(planner.step,planner.grid.res)/2
    points=[tuple(start)];previous=tuple(start)
    for endpoint in candidate:
        distance=math.dist(previous,endpoint)
        n=max(1,math.ceil(distance/step));last=previous
        for i in range(1,n+1):
            p=tuple(a+(b-a)*i/n for a,b in zip(previous,endpoint))
            if not planner.segment_free(previous,p):
                if last!=points[-1]:points.append(last)
                # 从安全前缀尾端回退，以容纳到位容差/雷达噪声，不贴未知格停车。
                remaining=reserve
                while len(points)>1 and remaining>0:
                    length=math.dist(points[-2],points[-1])
                    if length<=remaining:
                        points.pop();remaining-=length
                    else:
                        a,b=points[-2:];fraction=(length-remaining)/length
                        points[-1]=tuple(x+(y-x)*fraction for x,y in zip(a,b));remaining=0
                if len(points)<2 or sum(math.dist(a,b) for a,b in zip(points,points[1:]))<reserve:
                    return None,False
                assert all(planner.segment_free(a,b) for a,b in zip(points,points[1:]))
                return points[1:],False
            last=p
        if endpoint!=points[-1]:points.append(tuple(endpoint))
        previous=tuple(endpoint)
    # 候选全程若通过严格检查，则正常返回；从不返回未经检查的候选段。
    return points[1:],True


def endpoint_summary(planner,point):
    x,y=point;grid=planner.grid;mx=planner.hx+planner.margin;my=planner.hy+planner.margin
    # 当前实机 map/odom 同轴，记录周边格的分类帮助定位未知/障碍/边界。
    i0=max(0,math.floor((x-mx-grid.ox)/grid.res));i1=min(grid.w,math.floor((x+mx-grid.ox)/grid.res)+1)
    j0=max(0,math.floor((y-my-grid.oy)/grid.res));j1=min(grid.h,math.floor((y+my-grid.oy)/grid.res)+1)
    cells=grid.costs[j0:j1,i0:i1]
    return dict(point=list(point),reason=planner.segment_blocked_reason(point,point),
                lethal=int(np.count_nonzero(cells==254)),unknown=int(np.count_nonzero(cells==255)))
