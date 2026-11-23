import sys, glob, os
from pymavlink import mavutil
import numpy as np
import os
FLIGHT_DATA = os.environ.get('FLIGHT_DATA', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'flight-data'))
for f in sorted(glob.glob((FLIGHT_DATA + '/fc_blackbox/QUADROTOR/1/*.bin'))):
    m = mavutil.mavlink_connection(f, dialect='ardupilotmega')
    of=[]; arm=None; dis=None; rf=[]
    while True:
        x=m.recv_match(type=['OF','EV','RFND'])
        if x is None: break
        t=x.TimeUS/1e6
        if x.get_type()=='EV':
            if x.Id==28: arm=t
            if x.Id==11 and arm is not None and t>arm: dis=t
        elif x.get_type()=='RFND': rf.append((t,x.Dist))
        else: of.append((t,x.Qual,x.flowX,x.flowY,x.bodyX,x.bodyY))
    of=np.array(of); rf=np.array(rf)
    w=(of[:,0]>arm+2)&(of[:,0]<(dis or 1e9)-1)&(of[:,1]>50)
    o=of[w]
    # high-pass (remove translation) by differencing
    res=[]
    for i,j in ((2,4),(3,5)):
        a=o[:,i]; b=o[:,j]
        c=np.corrcoef(a,b)[0,1]
        slope=np.polyfit(b,a,1)[0]
        res.append((c,slope))
    print(f"{os.path.basename(f)[:20]:22s} n={len(o):4d}  corr(flowX,bodyX)={res[0][0]:+.2f} slope={res[0][1]:+.2f} | corr(flowY,bodyY)={res[1][0]:+.2f} slope={res[1][1]:+.2f} | OF qual median={np.median(o[:,1]):.0f}  OF rate={len(of)/(of[-1,0]-of[0,0]):.0f}Hz")
