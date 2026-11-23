"""Faithful Python port of frontier.c mapping + frontier selection, replayed on scanlog records."""
import sys, math, collections
import numpy as np
sys.path.insert(0, sys.argv[0].rsplit('/',1)[0])
from scanlog import load
CELL=0.15; W=H=1024; RAY_STEP=0.075; OCC_MARGIN=0.05; OCC_MAX=3.5; TOF_MAX=4.0
UNK,FREE,OCC=-1,0,1
DIRBASE=[0,90,180,-90]

def robust_col(grid, col):
    vals=[]
    for row in range(8):
        mm=grid[row,col]
        if mm in (0,0xFFFF): continue
        m=mm*0.001
        if m<=0.02: continue
        vals.append(min(m,TOF_MAX))
    vals.sort()
    return vals[1] if len(vals)>1 else (vals[0] if vals else math.nan)

class Map:
    def __init__(s,cx,cy):
        s.g=np.full((H,W),UNK,np.int8); s.ox=cx-0.5*W*CELL; s.oy=cy-0.5*H*CELL
    def w2c(s,x,y):
        ix=math.floor((x-s.ox)/CELL); iy=math.floor((y-s.oy)/CELL)
        return (ix,iy) if 0<=ix<W and 0<=iy<H else None
    def c2w(s,ix,iy): return s.ox+(ix+.5)*CELL, s.oy+(iy+.5)*CELL
    def val(s,ix,iy): return OCC if not(0<=ix<W and 0<=iy<H) else s.g[iy,ix]
    def setw(s,x,y,v):
        c=s.w2c(x,y)
        if c: s.g[c[1],c[0]]=v
    def ray(s,x0,y0,yaw,d):
        if d<=0.02: return
        lim=max(0,d-OCC_MARGIN); st=0.0; c,sn=math.cos(math.radians(yaw)),math.sin(math.radians(yaw))
        while st<=lim:
            s.setw(x0+c*st,y0+sn*st,FREE); st+=RAY_STEP
        if d<=OCC_MAX: s.setw(x0+c*d,y0+sn*d,OCC)
    def add_scan(s,r,fov):
        for d in range(4):
            base=r['yaw']+DIRBASE[d]
            for col in range(8):
                dist=robust_col(r['grid'][d],col)
                if not math.isfinite(dist): continue
                u=(col+.5)/8; s.ray(r['x'],r['y'],base+(u-.5)*fov,dist)
    def is_frontier(s,ix,iy):
        if s.val(ix,iy)!=FREE: return False
        return any(s.val(ix+dx,iy+dy)==UNK for dx in(-1,0,1) for dy in(-1,0,1) if dx or dy)
    def clearance(s,ix,iy):
        r=math.ceil(1.5/CELL); sub=s.g[max(0,iy-r):iy+r+1, max(0,ix-r):ix+r+1]
        ys,xs=np.nonzero(sub==OCC)
        if len(ys)==0: return 10.0
        ys=ys+max(0,iy-r)-iy; xs=xs+max(0,ix-r)-ix
        return float(np.min(np.hypot(xs,ys))*CELL)
    def line_clear(s,x0,y0,x1,y1,mc):
        d=math.hypot(x1-x0,y1-y0); n=max(1,math.ceil(d/RAY_STEP))
        for i in range(n+1):
            u=i/n; c=s.w2c(x0+(x1-x0)*u,y0+(y1-y0)*u)
            if not c: return False,'oob'
            if s.val(*c)==OCC: return False,'occ'
            if s.clearance(*c)<mc: return False,'clr'
        return True,''

def select(m,cx,cy,yaw_deg,clr=0.14,cone=120.0,step_max=0.25,arr=0.20,min_cluster=2):
    ys,xs=np.nonzero(m.g==FREE)
    fr=set((x,y) for x,y in zip(xs,ys) if m.is_frontier(x,y))
    seen=set(); clusters=[]
    for c in sorted(fr):
        if c in seen: continue
        q=collections.deque([c]); seen.add(c); cl=[]
        while q:
            a=q.popleft(); cl.append(a)
            for dx in(-1,0,1):
                for dy in(-1,0,1):
                    n=(a[0]+dx,a[1]+dy)
                    if n in fr and n not in seen: seen.add(n); q.append(n)
        clusters.append(cl)
    rej=collections.Counter(); best=None; out=[]
    for cl in clusters:
        if len(cl)<min_cluster: rej['small_cluster']+=1; continue
        cen=np.mean([m.c2w(*c) for c in cl],axis=0); cand=None
        for c in cl:
            wx,wy=m.c2w(*c); cr=m.clearance(*c)
            if cr<clr: rej['raw_clearance']+=1; continue
            ok,why=m.line_clear(cx,cy,wx,wy,clr)
            if not ok: rej['raw_path_'+why]+=1; continue
            dist=math.hypot(wx-cx,wy-cy)
            off=abs((math.degrees(math.atan2(wy-cy,wx-cx))-yaw_deg+180)%360-180)
            if off>cone: rej['raw_cone']+=1; continue
            sc=dist+0.35*math.hypot(wx-cen[0],wy-cen[1])-0.25*min(cr,1)+0.4*off/cone
            if cand is None or sc<cand[0]: cand=(sc,wx,wy)
        if cand is None: out.append((cl,cen,'no_raw')); continue
        _,rx,ry=cand; d=math.hypot(rx-cx,ry-cy)
        if d<arr: rej['step_too_close']+=1; out.append((cl,cen,'too_close')); continue
        st=min(d,1.2,step_max); ok=False
        while st>=arr:
            tx,ty=cx+(rx-cx)*st/d, cy+(ry-cy)*st/d; c=m.w2c(tx,ty)
            if c and m.val(*c)==FREE and m.clearance(*c)>=clr and m.line_clear(cx,cy,tx,ty,clr)[0]: ok=True; break
            st-=CELL
        if not ok: rej['step_fail']+=1; out.append((cl,cen,'step_fail')); continue
        out.append((cl,cen,'ok')); 
        if best is None or st<best[0]: best=(st,tx,ty,len(cl))
    return fr,clusters,rej,best,out
