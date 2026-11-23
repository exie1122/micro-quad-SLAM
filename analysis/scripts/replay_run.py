import sys, math
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from scanlog import load
from frontier_replay import Map, select, FREE, OCC, UNK
import numpy as np
scan, t_start, ox, oy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
t_sel=[float(x) for x in sys.argv[5].split(',')]; fov=float(sys.argv[6]) if len(sys.argv)>6 else 70
R=[r for r in load(scan) if r['host_ms']/1000>=t_start]
print('scan host range %.2f..%.2f'%(R[0]['host_ms']/1000,R[-1]['host_ms']/1000) if R else 'none')
m=Map(ox,oy); i=0
for ts in t_sel:
    while i<len(R) and R[i]['host_ms']/1000<=ts:
        m.add_scan(R[i],fov); i+=1
    last=R[i-1]
    fr,cl,rej,best,out=select(m,last['x'],last['y'],last['yaw'])
    n_free=(m.g==FREE).sum(); n_occ=(m.g==OCC).sum()
    print(f"t={ts:.2f} FOV={fov:.0f} scans={i} pose=({last['x']:.2f},{last['y']:.2f}) yaw={last['yaw']:.1f} free={n_free} occ={n_occ} frontier_cells={len(fr)} clusters={len(cl)} sizes={sorted(len(c) for c in cl)[::-1][:12]}")
    print('   rejections:',dict(rej),' best:',best)
    print('   cluster outcomes:',[(len(c),tuple(np.round(cen,2)),o) for c,cen,o in out][:12])
    np.save(f'map_{ts:.2f}_{fov:.0f}.npy', m.g)
