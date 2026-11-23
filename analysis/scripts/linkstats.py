import sys, glob, os
from pymavlink import mavutil
import numpy as np
import os
FLIGHT_DATA = os.environ.get('FLIGHT_DATA', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'flight-data'))
for f in sorted(glob.glob((FLIGHT_DATA + '/fc_blackbox/QUADROTOR/1/*.bin'))):
    m = mavutil.mavlink_connection(f, dialect='ardupilotmega')
    u5=[]; mav3=[]; arm=None; disarm=None
    while True:
        x = m.recv_match(type=['UART','MAV','EV'])
        if x is None: break
        t=x.TimeUS/1e6
        if x.get_type()=='EV':
            if x.Id==28 and arm is None: arm=t
            if x.Id==11: disarm=t
        elif x.get_type()=='UART' and x.I==5: u5.append((t,x.Rx,x.Tx))
        elif x.get_type()=='MAV' and x.chan==3: mav3.append((t,x.rxp,x.txp,x.tf))
    u5=np.array(u5); mav3=np.array(mav3)
    a = arm or u5[0,0]; d = disarm or u5[-1,0]
    w = (u5[:,0]>a+3)&(u5[:,0]<d)
    mw = (mav3[:,0]>a+3)&(mav3[:,0]<d)
    mm = mav3[mw]
    rxps = np.diff(mm[:,1])/np.diff(mm[:,0]) if len(mm)>1 else [np.nan]
    print(f"{os.path.basename(f)[:22]:24s} airborne {a:6.1f}-{d:6.1f}  SERIAL5 Rx bytes/s: median={np.median(u5[w,1]):5.0f} max={u5[w,1].max():5.0f} (cap 5760) | valid pkts parsed/s: median={np.median(rxps):4.1f}  | FC->comp Tx B/s={np.median(u5[w,2]):5.0f}")
