import sys
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1], dialect='ardupilotmega')
while True:
    x = m.recv_match(type=['MAVC','ARM','MODE','EV','CMD'])
    if x is None: break
    d=x.to_dict(); t=x.TimeUS/1e6
    if x.get_type()=='MAVC' and d['Cmd']==400 and d['P2']==21196: continue
    print('%.3f'%t, x.get_type(), {k:v for k,v in d.items() if k not in ('mavpackettype','TimeUS')})
