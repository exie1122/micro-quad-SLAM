import sys, os
from pymavlink import mavutil
f = sys.argv[1]
m = mavutil.mavlink_connection(f, dialect='ardupilotmega')
fmt_printed=set()
rows=[]
while True:
    x = m.recv_match(type=['GUIP','PSCN','PSCE','MODE','EV','XKF1','RCOU','ATT','MAVC','CMD'])
    if x is None: break
    ty=x.get_type()
    if ty not in fmt_printed:
        fmt_printed.add(ty); print('FIELDS', ty, x.get_fieldnames())
    rows.append(x)
# print GUIP all
last={}
for x in rows:
    ty=x.get_type(); t=x.TimeUS/1e6
    if ty=='GUIP': print('%.2f GUIP type=%d p=(%.2f,%.2f,%.2f) v=(%.2f,%.2f,%.2f)'%(t,x.Type,x.pX,x.pY,x.pZ,x.vX,x.vY,x.vZ))
    elif ty in ('MODE','EV'): print('%.2f %s %s'%(t,ty,x.to_dict()))
    elif ty=='MAVC': print('%.2f MAVC %s'%(t,{k:v for k,v in x.to_dict().items() if k in ('Cmd','P1','P2','P7','Res','SS','SC','TS','TC','Fr')}))
