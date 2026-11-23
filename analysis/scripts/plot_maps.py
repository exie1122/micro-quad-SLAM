import sys, math
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
import numpy as np
from scanlog import load
from frontier_replay import Map, select, FREE, OCC, UNK, CELL
import os
FLIGHT_DATA = os.environ.get('FLIGHT_DATA', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'flight-data'))
def run(scan, t0, ox, oy, ts, fov):
    R=[r for r in load(scan) if t0<=r['host_ms']/1000<=ts]
    m=Map(ox,oy)
    for r in R: m.add_scan(r,fov)
    last=R[-1]; fr,cl,rej,best,out=select(m,last['x'],last['y'],last['yaw'])
    return m,R,fr,cl,best,rej
def draw(ax, m, R, fr, cl, best, title):
    ys,xs=np.nonzero(m.g!=UNK); pad=8
    x0,x1,y0,y1=xs.min()-pad,xs.max()+pad,ys.min()-pad,ys.max()+pad
    sub=m.g[y0:y1,x0:x1].astype(float)
    img=np.zeros(sub.shape+(3,)); img[sub==UNK]=(0.75,0.75,0.75); img[sub==FREE]=(1,1,1); img[sub==OCC]=(0,0,0)
    # NED: x north (up), y east (right) -> plot east on horizontal, north vertical
    ext=[m.oy+y0*CELL, m.oy+y1*CELL, m.ox+x0*CELL, m.ox+x1*CELL]
    ax.imshow(np.transpose(img,(1,0,2)), origin='lower', extent=ext, interpolation='nearest')
    cmap=plt.get_cmap('tab10')
    for k,c in enumerate(cl):
        pts=np.array([m.c2w(*p) for p in c]); ax.scatter(pts[:,1],pts[:,0],s=14,color=cmap(k%10),marker='s',label=f'frontier cluster {len(c)} cells' if k<4 else None)
    tr=np.array([(r['y'],r['x']) for r in R]); ax.plot(tr[:,0],tr[:,1],'-',color='tab:blue',lw=1.5,label='EKF path (scan poses)')
    p=R[-1]; ax.plot(p['y'],p['x'],'o',color='tab:blue',ms=7)
    h=math.radians(p['yaw']); ax.arrow(p['y'],p['x'],0.3*math.sin(h),0.3*math.cos(h),head_width=0.06,color='tab:blue')
    if best: ax.plot(best[2],best[1],'*',color='red',ms=14,label='chosen 0.25 m step target')
    ax.set_title(title,fontsize=9); ax.set_xlabel('East (m)'); ax.set_ylabel('North (m)'); ax.set_aspect('equal')
    ax.legend(fontsize=6,loc='lower left')
cases=[('4_frontierTest_scanlog.bin',38.875,0.11,-0.16,42.231,'Flight 4 (frontierTest) at 1st SELECT'),
       ('4_frontierTest_scanlog.bin',38.875,0.11,-0.16,48.874,'Flight 4 at 2nd SELECT (logged: no valid frontier)'),
       ('3_hallWay_scanlog.bin',129.823,0.54,0.12,152.893,'3_hallWay at last SELECT (after 5 aborted moves)')]
fig,axes=plt.subplots(2,3,figsize=(16,10))
import os; os.chdir((FLIGHT_DATA + '/onboard/scanlogs'))
for j,(sc,t0,ox,oy,ts,title) in enumerate(cases):
    for i,fov in enumerate((70,45)):
        m,R,fr,cl,best,rej=run(sc,t0,ox,oy,ts,fov)
        draw(axes[i,j],m,R,fr,cl,best,f"{title}\nFOV={fov}° {'(as coded)' if fov==70 else '(true VL53L5CX)'}: free={int((m.g==FREE).sum())} occ={int((m.g==OCC).sum())} frontier={len(fr)}")
        print(title, fov, dict(rej), best)
plt.tight_layout(); plt.savefig(__import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)),'..','figures','frontier_maps.png'),dpi=110)
