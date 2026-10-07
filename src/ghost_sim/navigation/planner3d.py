"""Six-neighbour voxel A* and continuous swept-sphere segment validation."""
import heapq
import numpy as np
from ghost_sim.mapping.voxel_map import build,index_of,FREE

class Planner:
    def __init__(self):
        self.states,self.centres,self.meta=build()
        self.radius=self.meta['ghost_radius']+self.meta['margin']

    def free(self,p):
        p=np.asarray(p,dtype=float)
        return (p.shape==(3,) and np.isfinite(p).all() and np.all(p>=self.meta['origin'])
                and np.all(p<self.meta['upper_exclusive']) and self.states[tuple(index_of(p,self.meta))]==FREE)

    def segment_clear(self,a,b):
        a,b=np.asarray(a,dtype=float),np.asarray(b,dtype=float)
        lower=np.array(self.meta['origin'])+self.radius
        upper=np.array(self.meta['upper_exclusive'])-self.radius
        if not np.isfinite([a,b]).all() or np.any(np.minimum(a,b)<lower) or np.any(np.maximum(a,b)>upper):return False
        d=b-a
        for box in self.meta['boxes']:
            lo=np.array(box['centre'])-box['half_size'];hi=np.array(box['centre'])+box['half_size']
            breaks=[0.,1.]
            for axis in range(3):
                if abs(d[axis])>1e-12:
                    breaks.extend(t for edge in [lo[axis],hi[axis]] if 0<(t:=(edge-a[axis])/d[axis])<1)
            breaks=sorted(set(breaks)); candidates=list(breaks)
            for left,right in zip(breaks,breaks[1:]):
                p=a+d*(left+right)/2
                mask=(p<lo)|(p>hi)
                offset=a-np.where(p<lo,lo,hi)
                denominator=np.dot(d[mask],d[mask])
                if denominator>1e-15:
                    candidates.append(float(np.clip(-np.dot(d[mask],offset[mask])/denominator,left,right)))
            points=a+np.asarray(candidates)[:,None]*d
            dist=np.linalg.norm(np.maximum(np.maximum(lo-points,points-hi),0),axis=1)
            if dist.min()<=self.radius+1e-9:return False
        return True

    def plan(self,start,goal):
        if not self.free(start):
            # A canceled smoothed path can stop in a conservative blocked voxel,
            # while the actual sphere remains clear of all physical obstacles.
            if not self.segment_clear(start,start):
                raise ValueError('当前位置的球体间距不足，请先手动移到空地。')
            centres=self.centres[self.states==FREE]
            distances=np.linalg.norm(centres-np.asarray(start),axis=1)
            for candidate in centres[np.argsort(distances)[:32]]:
                if np.linalg.norm(candidate-start)>.5:break
                if self.segment_clear(start,candidate):
                    try:return [np.asarray(start).tolist()]+self.plan(candidate,goal)
                    except ValueError:continue
            raise ValueError('当前位置无法安全连接到附近自由体素。')
        if not self.free(goal):raise ValueError('目标在障碍、间距禁区或地图之外。')
        start,goal=np.array(start),np.array(goal)
        if self.segment_clear(start,goal):return [start.tolist(),goal.tolist()]
        s=tuple(index_of(start,self.meta));g=tuple(index_of(goal,self.meta))
        queue=[(0,0,s)];cost={s:0};parent={};closed=set()
        directions=[(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)]
        while queue:
            _,distance,u=heapq.heappop(queue)
            if u in closed:continue
            if u==g:break
            closed.add(u)
            for direction in directions:
                v=tuple(u[i]+direction[i] for i in range(3))
                if any(v[i]<0 or v[i]>=self.states.shape[i] for i in range(3)) or self.states[v]!=FREE:continue
                new=distance+1
                if new<cost.get(v,float('inf')):
                    cost[v]=new;parent[v]=u
                    heapq.heappush(queue,(new+sum(abs(v[i]-g[i]) for i in range(3)),new,v))
        else:raise ValueError('三维地图中找不到连通路线。')
        cells=[g]
        while cells[-1]!=s:cells.append(parent[cells[-1]])
        raw=[start]+[self.centres[c] for c in reversed(cells)]+[goal]
        result=[raw[0]];i=0
        while i<len(raw)-1:
            for j in range(len(raw)-1,i,-1):
                if self.segment_clear(raw[i],raw[j]):break
            else:raise ValueError('路线连续碰撞检查失败。')
            result.append(raw[j]);i=j
        return [p.tolist() for p in result]
