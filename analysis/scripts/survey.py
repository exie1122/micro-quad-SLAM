import sys, glob, os, collections
from pymavlink import mavutil
import os
FLIGHT_DATA = os.environ.get('FLIGHT_DATA', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'flight-data'))
for f in sorted(glob.glob((FLIGHT_DATA + '/fc_blackbox/QUADROTOR/1/*.bin'))):
    m = mavutil.mavlink_connection(f, dialect='ardupilotmega')
    counts = collections.Counter(); msgs=[]; modes=[]; t0=None; t1=None; arm=[]
    while True:
        x = m.recv_match()
        if x is None: break
        ty = x.get_type(); counts[ty]+=1
        if ty=='MSG': msgs.append((getattr(x,'TimeUS',0)/1e6, x.Message))
        if ty=='MODE': modes.append((x.TimeUS/1e6, x.Mode, getattr(x,'Rsn',None)))
        if ty=='EV': arm.append((x.TimeUS/1e6, x.Id))
        if hasattr(x,'TimeUS') and ty not in ('FMT','FMTU','UNIT','MULT','PARM'):
            t0 = x.TimeUS if t0 is None else t0; t1 = x.TimeUS
    print('=====', os.path.basename(f), 'dur=%.1fs'%((t1-t0)/1e6 if t0 else 0))
    for t,s in msgs[:14]: print('  MSG %.1f %s'%(t,s))
    print('  MODES', [(round(a,1),b,c) for a,b,c in modes][:20])
    print('  EV', [(round(a,1),b) for a,b in arm][:30])
    print('  types:', ' '.join(f'{k}:{v}' for k,v in sorted(counts.items()) if v>0)[:900])
