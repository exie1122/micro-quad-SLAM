"""Does the VL53L5CX report RADIAL distance (along each zone's ray) or PERPENDICULAR depth
(along the sensor axis)? plot_scan.py assumes perpendicular and divides by cos(az)*cos(el).

On a flat surface the correct model puts a sensor's 8 column points on a straight line. For
every frame/sensor where 7-8 columns see something within 0.2-2.5 m, fit a line under both
models and compare the RMS residual. The model with consistently lower residual is right.
Also reports the classic head-on signature: facing a wall square-on, radial readings are ~6 %
longer at the edge columns (1/cos 19.7 deg) than at the centre; perpendicular readings are flat.
"""
import os, sys, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from scanlog import load
from verify_sensor_mounting import sessions

BASE = os.path.join(os.environ.get('FLIGHT_DATA', os.path.join(HERE, '..', '..', 'flight-data')), 'onboard', 'scanlogs')
AZ = np.radians((np.arange(8) - 3.5) * (45.0 / 8))
EL = math.radians(2.8)                        # rows 3-4 are +/-2.8 deg

def fit_rms(x, y):
    P = np.column_stack([x, y]); P = P - P.mean(0)
    return math.sqrt(max(np.linalg.eigvalsh(P.T @ P / len(P))[0], 0.0))

seen = set(); better_radial = []; rms_r = []; rms_p = []; edge_ratio = []
for f in sorted(os.listdir(BASE)):
    if not f.endswith('_scanlog.bin'): continue
    for s in sessions(load(os.path.join(BASE, f))):
        key = (len(s), s[0]['host_ms'], s[-1]['host_ms'])
        if key in seen or len(s) < 40: continue
        seen.add(key)
        for r in s:
            if r['alt'] < 0.25: continue
            for d in range(4):
                v = r['grid'][d, 3:5, :]
                ok = (v > 200) & (v < 2500)
                if ok.sum(0).min() < 2: continue                 # need all 8 columns
                rng = np.median(np.where(ok, v, np.nan), axis=0) / 1000.0
                if np.ptp(rng) > 0.6: continue                   # one surface, no big jumps
                # radial: point = rng * dir ; perpendicular: point = rng/(cos az cos el) * dir
                xr, yr = rng * np.cos(AZ), rng * np.sin(AZ)
                k = 1.0 / (np.cos(AZ) * math.cos(EL))
                xp, yp = xr * k, yr * k
                a, b = fit_rms(xr, yr), fit_rms(xp, yp)
                if max(a, b) > 0.05: continue                     # not a flat surface under either model
                rms_r.append(a); rms_p.append(b); better_radial.append(a < b)
                # head-on frames: profile symmetric (left half vs right half within 2 %)
                if abs(rng[:4].mean() - rng[4:].mean()) < 0.02 * rng.mean():
                    edge_ratio.append(((rng[0] + rng[7]) / 2) / ((rng[3] + rng[4]) / 2))
rms_r, rms_p = np.array(rms_r), np.array(rms_p)
print(f"flat-surface views: {len(rms_r)}")
print(f"  median line-fit RMS: radial model {np.median(rms_r)*1000:.1f} mm | perpendicular model (plot_scan.py) {np.median(rms_p)*1000:.1f} mm")
print(f"  radial model straighter in {np.mean(better_radial)*100:.0f}% of views")
if edge_ratio:
    e = np.array(edge_ratio)
    print(f"  head-on views: n={len(e)}, edge/centre range ratio median {np.median(e):.3f} "
          f"(radial predicts {1/math.cos(AZ[0]):.3f}, perpendicular predicts 1.000)")
