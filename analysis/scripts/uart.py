import sys, collections
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1], dialect='ardupilotmega')
params={}
first=True
while True:
    x = m.recv_match(type=['UART','MAV','PARM'])
    if x is None: break
    if x.get_type()=='PARM':
        params[x.Name]=x.Value; continue
    if first and x.get_type()=='UART': print('UART fields', x.get_fieldnames()); first=False
    d=x.to_dict(); t=x.TimeUS/1e6
    print('%.1f'%t, x.get_type(), {k:(round(v,1) if isinstance(v,float) else v) for k,v in d.items() if k not in ('mavpackettype','TimeUS')})
print({k:v for k,v in params.items() if k.startswith('SERIAL') or k.startswith('SR') or k.startswith('MAV_')})
