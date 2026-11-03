"""CPU-only online-side prototype. No oracle, simulator, semantic IDs or GT imports."""
from dataclasses import dataclass
from enum import Enum
import math

class Status(str, Enum):
    HIT='HIT'; NO_HIT='NO_HIT'; INVALID='INVALID'; AMBIGUOUS='AMBIGUOUS'

@dataclass(frozen=True)
class Measurement:
    distance: float
    status: Status
    timestamp: float
    provenance: str

def classify(distance, timestamp, *, rich=False, query_hit=None, healthy=None, far=12.):
    if not math.isfinite(distance) or distance < 0:
        return Measurement(distance,Status.INVALID,timestamp,'nonfinite_or_negative')
    if not rich:
        # No extra health assertion smuggled into the legacy-distance contract.
        return Measurement(distance,Status.AMBIGUOUS,timestamp,'distance_only')
    if healthy is False:
        return Measurement(distance,Status.INVALID,timestamp,'explicit_invalid')
    if healthy is not True or query_hit is None:
        return Measurement(distance,Status.AMBIGUOUS,timestamp,'missing_health_or_query')
    if query_hit and .2 <= distance <= far:
        return Measurement(distance,Status.HIT,timestamp,'synthetic_exact_valid_query')
    if query_hit is False and distance == 1000.:
        return Measurement(far,Status.NO_HIT,timestamp,'synthetic_exact_valid_query')
    return Measurement(distance,Status.AMBIGUOUS,timestamp,'inconsistent_or_clipped')

class Ledger:
    """Registered single-beam interval ledger. No full cells cleared from partial rays."""
    width=.05; freshness=.05
    def __init__(self): self.cells={}
    def cell(self,i):
        return self.cells.setdefault(i,{'state':'UNKNOWN','occupied':False,'hit_times':[], 'candidate':False})
    def update(self,m,now,*,candidate=False,camera_age=0.,camera_visible=True):
        if m.status not in (Status.HIT,Status.NO_HIT): return 'NO_VALID_FREE_EVIDENCE'
        if not 0 <= now-m.timestamp <= self.freshness: return 'STALE_REJECTED'
        end=m.distance
        if m.status == Status.HIT:
            # A previously observed empty interval is not a CURRENT visibility claim
            # behind a new first-hit. Keep occupied evidence, withdraw old free status.
            for i,c in self.cells.items():
                if i*self.width>end and not c['occupied']: c['state']='UNKNOWN'
        for i in range(int(end/self.width)):
            if (i+1)*self.width >= end-1e-7: continue
            c=self.cell(i)
            if c['occupied'] or c['candidate']: c['state']='CONFLICT'
            else: c['state']='MEASURED_FREE'
        if m.status == Status.NO_HIT: return 'UPDATED'
        c=self.cell(int(end/self.width))
        supported=candidate and camera_visible and 0<=camera_age<=self.freshness
        if supported:
            c['candidate']=True
            c['state']='CONFLICT' if c['occupied'] else 'TARGET_CANDIDATE'
        else:
            c['occupied']=True
            if m.timestamp not in c['hit_times']: c['hit_times'].append(m.timestamp)
            c['state']='CONFIRMED_STATIC_OCCUPIED' if len(c['hit_times'])>=2 and not c['candidate'] else 'CONFLICT'
        return 'UPDATED'
    def at(self,r): return self.cell(int(r/self.width)).copy()
    def approach(self,lo,hi):
        cells=[self.cell(i) for i in range(int(lo/self.width),int(hi/self.width)+1)]
        if any(c['occupied'] or c['state']=='CONFLICT' for c in cells): return 'OCCUPANCY_OR_CONFLICT'
        if any(c['state']!='MEASURED_FREE' for c in cells): return 'INFORMATION_LIMITED'
        return 'MEASURED_FREE_ON_THIS_BEAM_ONLY'

def point_segment(p,a,b):
    dx=b[0]-a[0];dy=b[1]-a[1];den=dx*dx+dy*dy
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den)) if den else 0.
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)

def crosses(a,b,box,strict=False):
    xmin,ymin,xmax,ymax=box
    lo,hi=0.,1.
    for x,d,mn,mx in [(a[0],b[0]-a[0],xmin,xmax),(a[1],b[1]-a[1],ymin,ymax)]:
        if abs(d)<1e-15:
            if x<mn or x>mx: return False
        else:
            t0,t1=sorted(((mn-x)/d,(mx-x)/d));lo=max(lo,t0);hi=min(hi,t1)
            if lo>hi: return False
    if not strict: return True
    t=(lo+hi)/2;x=a[0]+t*(b[0]-a[0]);y=a[1]+t*(b[1]-a[1])
    return xmin+1e-9<x<xmax-1e-9 and ymin+1e-9<y<ymax-1e-9

def segment_box_distance(a,b,box):
    if crosses(a,b,box): return 0.
    x0,y0,x1,y1=box
    vals=[point_segment(p,a,b) for p in [(x0,y0),(x0,y1),(x1,y0),(x1,y1)]]
    vals += [math.hypot(max(x0-p[0],0,p[0]-x1),max(y0-p[1],0,p[1]-y1)) for p in (a,b)]
    return min(vals)

def evidence(rays,now=0.):
    segments=[];surfaces=[]
    for r in rays:
        if r['status'] not in ('HIT','NO_HIT') or not r['valid_exact']: continue
        o=tuple(r['origin']);d=r['direction'];length=r['range']
        if not all(math.isfinite(x) for x in [*o,*d,length]) or length<=0: continue
        stamp=r.get('timestamp',math.nan)
        if not math.isfinite(stamp) or not 0<=now-stamp<=.05: continue
        if abs(math.hypot(*d)-1.)>1e-9 or not .2<=length<=12.: continue
        e=(o[0]+d[0]*(length-1e-7),o[1]+d[1]*(length-1e-7))
        segments.append((o,e))
        if r['status']=='HIT' and not r['candidate']:
            surfaces.append((o[0]+d[0]*length,o[1]+d[1]*length))
    return segments,surfaces

def certify(path,rays,shapes,method,*,radius=.45,resolution=.002,max_nodes=100000,axis_aligned_prior=False,now=0.):
    """Universal continuous-center coverage, not occupancy at sampled centers."""
    if method not in ('A','B'): raise ValueError('unknown certificate')
    if method=='B' and not axis_aligned_prior:
        return {'certified':False,'reason':'ORIENTATION_PRIOR_NOT_QUALIFIED','nodes':0}
    free,surfaces=evidence(rays,now);a,b=path
    if any(point_segment(p,a,b)<=radius+1e-9 for p in surfaces):
        return {'certified':False,'reason':'OBSERVED_SURFACE_INTERSECTION','nodes':0}
    if not free: return {'certified':False,'reason':'NO_VALID_RAYS','nodes':0}
    total=0
    for hx,hy in ([(0.,0.)] if method=='A' else shapes):
        reach=1.01 if method=='A' else radius
        stack=[(min(a[0],b[0])-reach-hx,min(a[1],b[1])-reach-hy,max(a[0],b[0])+reach+hx,max(a[1],b[1])+reach+hy,0)]
        nodes=0
        while stack:
            x0,y0,x1,y1,dep=stack.pop();nodes+=1;total+=1
            if nodes>max_nodes: return {'certified':False,'reason':'NODE_BUDGET_UNKNOWN','nodes':total}
            if segment_box_distance(a,b,(x0-hx,y0-hy,x1+hx,y1+hy))>reach+1e-9: continue
            cx=(x0+x1)/2;cy=(y0+y1)/2;ex=(x1-x0)/2;ey=(y1-y0)/2
            if method=='A':
                covered=any(point_segment((cx,cy),s,e)+math.hypot(ex,ey)<.2-1e-9 for s,e in free)
            else:
                covered=hx>ex and hy>ey and any(crosses(s,e,(cx-hx+ex,cy-hy+ey,cx+hx-ex,cy+hy-ey),True) for s,e in free)
            if covered: continue
            if max(x1-x0,y1-y0)<=resolution or dep>=24:
                return {'certified':False,'reason':'UNRESOLVED_CENTER_CELL','nodes':total,'shape_halfsize':[hx,hy],'cell':[x0,y0,x1,y1]}
            if ex>=ey:
                stack += [(x0,y0,cx,y1,dep+1),(cx,y0,x1,y1,dep+1)]
            else:
                stack += [(x0,y0,x1,cy,dep+1),(x0,cy,x1,y1,dep+1)]
    return {'certified':True,'reason':'ALL_COLLIDING_CENTER_CELLS_EXCLUDED','nodes':total}

def qualify_straight_with_history(path,rays,shapes,ledger):
    """Minimal connection for a registered +x straight path, not a general map adapter."""
    a,b=path
    if a!=(0.,0.) and list(a)!=[0.,0.]:
        return {'certified':False,'reason':'UNSUPPORTED_LEDGER_FRAME'}
    if abs(b[1])>1e-12 or b[0]<0:
        return {'certified':False,'reason':'UNSUPPORTED_LEDGER_PATH'}
    state=ledger.approach(0.,b[0]+.45)
    if state!='MEASURED_FREE_ON_THIS_BEAM_ONLY':
        return {'certified':False,'reason':'HISTORY_'+state}
    return certify(path,rays,shapes,'B',axis_aligned_prior=True)
