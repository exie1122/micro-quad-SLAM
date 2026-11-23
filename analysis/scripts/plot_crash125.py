import os
from pymavlink import mavutil
import numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
import os
FLIGHT_DATA = os.environ.get('FLIGHT_DATA', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'flight-data'))
BLUE,ORANGE,INK,INK2,SURF='#2a78d6','#eb6834','#0b0b0b','#52514e','#fcfcfb'
f=(FLIGHT_DATA + '/fc_blackbox/QUADROTOR/1/125 1969-12-31 4-00-00 PM.bin')
m=mavutil.mavlink_connection(f,dialect='ardupilotmega'); A=[];P=[];M=[]
while True:
    x=m.recv_match(type=['ATT','PSCE','MAV'])
    if x is None: break
    t=x.TimeUS/1e6
    if x.get_type()=='ATT': A.append((t,x.DesRoll,x.Roll))
    elif x.get_type()=='PSCE': P.append((t,x.TVE,x.VE))
    elif x.chan==3: M.append((t,x.rxp))
A,P,M=map(np.array,(A,P,M)); t0,t1=33.0,49.6
OFF=1.0  # onboard -> FC time (approx; queue delay varies 0.3-1.8 s)
rejected=[(38.119,39.383),(43.158,43.668),(45.202,48.878)]
plt.rcParams.update({'font.size':9,'axes.edgecolor':'#c9c8c3','axes.labelcolor':INK2,'xtick.color':INK2,'ytick.color':INK2,'axes.facecolor':SURF,'figure.facecolor':SURF})
fig,ax=plt.subplots(3,1,figsize=(11,8),sharex=True,gridspec_kw={'height_ratios':[3,2,1.4]})
for a in ax:
    for s,e in rejected: a.axvspan(s-OFF,e-OFF,color='#d9d8d3',alpha=0.7,lw=0)
    a.axvline(48.55,color=INK2,lw=1,ls=':'); a.grid(axis='y',color='#ecebe7',lw=0.8); a.spines[['top','right']].set_visible(False)
w=(P[:,0]>t0)&(P[:,0]<t1)
ax[0].plot(P[w,0],P[w,2],color=BLUE,lw=2,label='actual east velocity (EKF)')
ax[0].plot(P[w,0],P[w,1],color=ORANGE,lw=2,label='FC target east velocity')
ax[0].axhline(0,color='#c9c8c3',lw=1)
ax[0].set_ylabel('East velocity (m/s)\n(negative = toward left wall)')
ax[0].annotate('companion "airwall" sends +0.30 m/s (push right)\n-> rejected -> FC re-seeds target from actual motion',xy=(47.7,-0.45),xytext=(40.3,-0.62),fontsize=8,color=INK,arrowprops=dict(arrowstyle='->',color=INK2))
ax[0].text(48.6,0.25,'wall impact\n(roll -60°)',fontsize=8,color=INK)
ax[0].legend(loc='upper left',frameon=False,fontsize=8)
ax[0].set_title('Flight 4_frontierTest (FC log 125): velocity target follows the drift instead of opposing it',loc='left',fontsize=10,color=INK)
w=(A[:,0]>t0)&(A[:,0]<t1)
ax[1].plot(A[w,0],A[w,2],color=BLUE,lw=2,label='actual roll'); ax[1].plot(A[w,0],A[w,1],color=ORANGE,lw=2,label='desired roll')
ax[1].set_ylim(-25,15); ax[1].set_ylabel('Roll (deg)\n(clipped at -25)'); ax[1].legend(loc='lower left',frameon=False,fontsize=8)
w=(M[:,0]>t0-1)&(M[:,0]<t1)
rate=np.diff(M[w,1])/np.diff(M[w,0]); tm=M[w,0][1:]
ax[2].step(tm,rate,where='pre',color=BLUE,lw=2)
ax[2].set_ylabel('valid companion\npackets / s'); ax[2].set_ylim(0,max(30,rate.max()+5))
ax[2].text(34,rate.max()*0.55 if rate.max()>10 else 20,'link at 5760 B/s (100% of 57600 baud) but only 1-4 packets/s parse',fontsize=8,color=INK)
ax[2].set_xlabel('FC time (s).  Gray = companion sending setpoints ArduPilot rejects (brake / prelock / airwall); onboard times shifted -1.0 s (approx.)')
plt.tight_layout(); plt.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','figures','crash_flight125.png'),dpi=120)
