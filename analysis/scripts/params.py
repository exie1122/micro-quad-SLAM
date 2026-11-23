import sys, glob, os, re
from pymavlink import mavutil
import os
FLIGHT_DATA = os.environ.get('FLIGHT_DATA', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'flight-data'))
logs = sorted(glob.glob((FLIGHT_DATA + '/fc_blackbox/QUADROTOR/1/*.bin')))
allp=[]
for f in logs:
    m = mavutil.mavlink_connection(f, dialect='ardupilotmega'); p={}
    while True:
        x=m.recv_match(type='PARM')
        if x is None: break
        p[x.Name]=x.Value
    allp.append(p)
pat = re.compile(sys.argv[1])
keys = sorted(set(k for p in allp for k in p if pat.match(k)))
print('%-18s'%'PARAM', ' '.join('%9s'%os.path.basename(f)[:9] for f in logs))
for k in keys:
    vals=[p.get(k) for p in allp]
    print('%-18s'%k, ' '.join('%9.4g'%v if v is not None else '%9s'%'-' for v in vals))
