import sys, glob, os
from pymavlink import mavutil
import numpy as np
import os
FLIGHT_DATA = os.environ.get('FLIGHT_DATA', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'flight-data'))
print("QuadX: M1=front-right(CCW) M2=rear-left(CCW) M3=front-left(CW) M4=rear-right(CW)")
for f in sorted(glob.glob((FLIGHT_DATA + '/fc_blackbox/QUADROTOR/1/*.bin'))):
    m = mavutil.mavlink_connection(f, dialect='ardupilotmega')
    d={'ATT':[], 'RCOU':[], 'PIDR':[], 'PIDP':[], 'PIDY':[], 'XKF1':[], 'CTUN':[]}
    arm=None; dis=None; land=None
    while True:
        x=m.recv_match(type=list(d)+['EV','MODE'])
        if x is None: break
        t=x.TimeUS/1e6; ty=x.get_type()
        if ty=='EV':
            if x.Id==28: arm=t
            if x.Id==11 and arm and t>arm and dis is None: dis=t
        elif ty=='MODE':
            if x.Mode==9 and arm: land=t
        elif ty=='ATT': d[ty].append((t,x.DesRoll,x.Roll,x.DesPitch,x.Pitch))
        elif ty=='RCOU': d[ty].append((t,x.C1,x.C2,x.C3,x.C4))
        elif ty in('PIDR','PIDP','PIDY'): d[ty].append((t,x.I,x.Flags))
        elif ty=='XKF1' and x.C==0: d[ty].append((t,x.VN,x.VE,-x.PD))
        elif ty=='CTUN': d[ty].append((t,x.ThO,x.ThH))
    D={k:np.array(v) for k,v in d.items()}
    end = min(x for x in (dis,land) if x) - 0.3
    def win(a, t0, t1): return a[(a[:,0]>t0)&(a[:,0]<t1)]
    # stable hover: airborne > 1.5s after liftoff, before LAND/disarm, alt>0.3, small tilt
    k=win(D['XKF1'],arm+1.5,end); alt_ok = k[k[:,3]>0.3]
    t0=alt_ok[0,0] if len(alt_ok) else arm+1.5
    A=win(D['ATT'],t0,end); A=A[(abs(A[:,2])<8)&(abs(A[:,4])<8)]
    R=win(D['RCOU'],t0,end); Pr=win(D['PIDR'],t0,end); Pp=win(D['PIDP'],t0,end); C=win(D['CTUN'],t0,end)
    mot=R[:,1:].mean(0)
    # first 2s after liftoff velocity drift direction
    k2=win(D['XKF1'],arm,arm+2.5)
    print(f"{os.path.basename(f)[:20]:21s} hover {end-t0:4.1f}s | DesR {A[:,1].mean():+.2f} R {A[:,2].mean():+.2f} DesP {A[:,3].mean():+.2f} P {A[:,4].mean():+.2f} deg | motors {mot.astype(int)} (RR-mean {mot[3]-mot.mean():+.0f}) | rateI roll {Pr[:,1].mean():+.3f} pitch {Pp[:,1].mean():+.3f} | ThO {C[:,1].mean():.3f} ThH {C[:,2].mean():.3f} | takeoff vN {k2[:,1].mean():+.2f} vE {k2[:,2].mean():+.2f}")
