import sys
from pymavlink import mavutil
import numpy as np
f=sys.argv[1]; t0=float(sys.argv[2]); t1=float(sys.argv[3]); step=float(sys.argv[4]) if len(sys.argv)>4 else 0.25
m = mavutil.mavlink_connection(f, dialect='ardupilotmega')
last={}; out=[]; nxt=t0; pf={}
types=['ATT','RCOU','CTUN','RFND','XKF1','PSCN','PSCE','OF','PIDR','PIDP','MOTB','GUIP','MODE','EV','RATE']
while True:
    x=m.recv_match(type=types)
    if x is None: break
    t=x.TimeUS/1e6; ty=x.get_type()
    if ty in ('PIDR','PIDP'):
        pf.setdefault(ty,[]).append((t,x.Flags, x.I, x.P, x.D))
    if ty=='XKF1' and x.C!=0: continue
    if ty=='RFND' and getattr(x,'Instance',0)!=0: continue
    last[ty]=x
    if ty in ('MODE','EV','GUIP') and t0<=t<=t1: print('   %.2f %s %s'%(t,ty,{k:(round(v,2) if isinstance(v,float) else v) for k,v in x.to_dict().items() if k not in('mavpackettype','TimeUS')}))
    if t>=nxt and t<=t1 and all(k in last for k in ('ATT','RCOU','CTUN','XKF1','PSCN','PSCE')):
        nxt=t+step
        a=last['ATT']; r=last['RCOU']; c=last['CTUN']; k=last['XKF1']; pn=last['PSCN']; pe=last['PSCE']
        rf=last.get('RFND'); of=last.get('OF')
        print('%.2f R%6.1f(d%6.1f) P%6.1f(d%6.1f) Y%6.1f | M %4d %4d %4d %4d | ThO %.3f | alt %.2f rf %.2f | PN %5.2f TPN %5.2f PE %5.2f TPE %5.2f | VN %5.2f VE %5.2f TVN %5.2f TVE %5.2f | OFq %s'%(
          t,a.Roll,a.DesRoll,a.Pitch,a.DesPitch,a.Yaw,r.C1,r.C2,r.C3,r.C4,c.ThO,-k.PD, rf.Dist if rf else -1,
          pn.PN,pn.TPN,pe.PE,pe.TPE,pn.VN,pe.VE,pn.TVN,pe.TVE, of.Qual if of else '-'))
