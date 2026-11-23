import sys, struct, os, glob
import numpy as np
REC = struct.Struct('<IQII' + 'f'*19 + 'III' + 'H'*6 + 'B'*8 + 'H' + '512s')
assert REC.size == 642, REC.size
def load(path):
    b=open(path,'rb').read(); recs=[]; i=0
    while True:
        j=b.find(b'SCN3', i)
        if j<0 or j+642>len(b): break
        v=REC.unpack_from(b,j); i=j+642
        g=np.frombuffer(v[-1],dtype='<u2').reshape(4,8,8).astype(float)  # dir,row,col
        f=v[4:23]
        recs.append(dict(host_ms=v[1],scan_ms=v[2],x=f[0],y=f[1],yaw=f[2],alt=f[3],roll=f[5],pitch=f[6],vx=f[7],vy=f[8],rf=f[12],armed=v[-6+0] if False else None,grid=g))
    return recs
if __name__=='__main__':
    for p in sys.argv[1:]:
        r=load(p); print('==',os.path.basename(p),'records',len(r))
        if not r: continue
        G=np.stack([x['grid'] for x in r]); alt=np.array([x['alt'] for x in r])
        sm=np.array([x['scan_ms'] for x in r]); hm=np.array([x['host_ms'] for x in r])
        # stale: identical grid per sensor vs previous record
        same=np.all(G[1:]==G[:-1],axis=(2,3))  # [n-1,4]
        print('  dt host median %.0fms; identical-to-previous frame per sensor (F,R,B,L): %s'%(np.median(np.diff(hm)), np.round(same.mean(0)*100,1)))
        bad=(G==0)|(G==0xFFFF)|(G<20)
        print('  zones 0/0xFFFF/<20mm per sensor %%:', np.round(bad.mean(axis=(0,2,3))*100,1))
        air = alt>0.3
        print('  airborne records', air.sum(), 'alt median %.2f'%np.median(alt[air]) if air.any() else '')
        if air.any():
            names='FRBL'
            for d in range(4):
                med=[np.median(np.where(bad[air][:,d,row,:],np.nan,G[air][:,d,row,:]))/1000 for row in range(8)]
                # correlation of row distance with altitude
                cors=[]
                for row in range(8):
                    v=np.nanmedian(np.where(bad[air][:,d,row,:],np.nan,G[air][:,d,row,:]),axis=1)/1000
                    ok=np.isfinite(v)
                    cors.append(np.corrcoef(v[ok],alt[air][ok])[0,1] if ok.sum()>20 else np.nan)
                print('   %s row medians (m) r0..r7: %s | corr w/ alt: %s'%(names[d],' '.join('%.2f'%m for m in med),' '.join('%+.2f'%c for c in cors)))
