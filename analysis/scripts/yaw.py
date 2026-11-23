import glob, os
from pymavlink import mavutil
import numpy as np
import os
FLIGHT_DATA = os.environ.get('FLIGHT_DATA', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'flight-data'))
for f in sorted(glob.glob((FLIGHT_DATA + '/fc_blackbox/QUADROTOR/1/*.bin'))):
    m=mavutil.mavlink_connection(f,dialect='ardupilotmega'); yaw=[]; sm=[]; arm=None; dis=None
    while True:
        x=m.recv_match(type=['ATT','XKF4','EV'])
        if x is None: break
        t=x.TimeUS/1e6
        if x.get_type()=='EV':
            if x.Id==28: arm=t; dis=None
            if x.Id==11 and arm and t>arm and dis is None: dis=t
        elif x.get_type()=='ATT': yaw.append((t,x.Yaw))
        elif x.C==0: sm.append((t,x.SM))
    yaw=np.array(yaw); sm=np.array(sm)
    w=(yaw[:,0]>arm+1)&(yaw[:,0]<dis-0.5); y=np.unwrap(np.radians(yaw[w,1])); 
    ws=(sm[:,0]>arm)&(sm[:,0]<dis)
    print(f"{os.path.basename(f)[:20]:21s} yaw span in flight {np.degrees(y.max()-y.min()):5.1f} deg | mag test ratio SM median {np.median(sm[ws,1]):.2f} max {sm[ws,1].max():.2f}")
