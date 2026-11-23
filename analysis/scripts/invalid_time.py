import re, sys, os, glob
import os
FLIGHT_DATA = os.environ.get('FLIGHT_DATA', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'flight-data'))
os.chdir((FLIGHT_DATA + '/onboard/logs'))
pat=re.compile(rb'^\[(\d+\.\d+)\] (.*)$')
files=['hallWay_log.txt','2_hallWay_log.txt','3_hallWay_log.txt','frontierTest_log.txt','3_frontierTest_log.txt','4_frontierTest_log.txt','6_frontierTest_log.txt','7_frontierTest_log.txt','2026-04-06_frontier_log.txt']
hover={'HOVER','HOVER_SCAN','SELECT_FRONTIER','SETTLE'}
for f in files:
    ev=[]
    for line in open(f,'rb'):
        m=pat.match(line.strip())
        if not m: continue
        t=float(m.group(1)); s=m.group(2).decode('latin1')
        ev.append((t,s))
    flights=[]; st=None; locked=False; brake=False; seg=None; acc=None; aw=set()
    for t,s in ev:
        if s.startswith('AIRWALL'): aw.add(int(t))
        m=re.match(r'STATE: (\S+) -> (\S+)',s)
        if m:
            a,b=m.groups()
            if b=='TAKEOFF' and a!='LIFTOFF_ASSIST': acc={'t0':t,'valid':0,'invalid':0,'other':0,'last':t,'mode':'other'}; brake=False
            if acc:
                acc[acc['mode']]+=t-acc['last']; acc['last']=t
            if b in hover and a not in hover and a!='MOVE_TO_FRONTIER': locked=False
            if b=='MOVE_TO_FRONTIER': locked=False
            if acc:
                if b in ('LANDING','DISARMING','IDLE') and a not in ('LANDING','DISARMING','IDLE'):
                    acc['t1']=t; acc['aw']=len([x for x in aw if acc['t0']<=x<=t]); flights.append(acc); acc=None
                else:
                    acc['mode']='valid' if (b=='MOVE_TO_FRONTIER' or (b in hover and locked)) else ('invalid' if b in hover else 'other')
            st=b; continue
        if not acc: continue
        if 'TAKEOFF_BRAKE: entering' in s: acc[acc['mode']]+=t-acc['last']; acc['last']=t; acc['mode']='invalid'
        if 'XY_LOCK_SUCCESS' in s: acc[acc['mode']]+=t-acc['last']; acc['last']=t; locked=True; acc['mode']='valid'
        if 'XY_LOCK_BREAK' in s or 'XY lock dropped' in s: acc[acc['mode']]+=t-acc['last']; acc['last']=t; locked=False; acc['mode']='invalid'
        if 'FRONTIER: move success' in s or 'No Progress Retry' in s or 'Move Retry' in s: locked=False
    for a in flights:
        T=a['t1']-a['t0']; inv=a['invalid']; val=a['valid']
        # airwall seconds during 'valid' time convert valid->invalid (upper bound: airwall logs at most 1/s)
        aw_s=min(a['aw'],val)
        print(f"{f[:24]:25s} {a['t0']:7.1f}-{a['t1']:7.1f} ({T:4.1f}s): takeoff-climb(NAV_TAKEOFF)={a['other']:4.1f}s | REJECTED-mask (brake/prelock)={inv:4.1f}s | valid-pos={val:4.1f}s of which airwall-active≈{aw_s}s  -> rejected share of controlled time ≈ {100*(inv+aw_s)/max(1e-6,inv+val):3.0f}%")
