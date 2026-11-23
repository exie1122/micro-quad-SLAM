"""Two independent checks of each ToF sensor's mounting, across every scanlog session.

Test 1 (which way up): rows 0 and 7 are ~19.7 deg below/above horizontal. A floor return in an
outer row reads ~ alt / sin(19.7 deg). Count, per sensor, how often row 0 vs row 7 matches that.

Test 2 (mirroring): fit a line to each sensor's 8 column points (middle rows 3-4) on a flat
surface. When the drone yaws by dpsi, a fixed wall's angle in the sensor's frame changes by
-dpsi. A correctly ordered sensor therefore measures d(angle)/d(psi) = -1; a column-mirrored
sensor measures +1. Pooled over frame pairs 0.2-1.0 s apart with |dpsi| >= 2 deg that see the
same surface (similar range, both fits clean). Uses only the gyro yaw and raw zone data.

Data are interpreted exactly as the firmware sent them (columns already flipped x = 7 - col for
every sensor), so "mirrored" here means "the firmware's flip is wrong for this sensor".
"""
import os, sys, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from scanlog import load

BASE = os.path.join(os.environ.get('FLIGHT_DATA', os.path.join(HERE, '..', '..', 'flight-data')), 'onboard', 'scanlogs')
NAMES = ['Front', 'Right', 'Back', 'Left']
DIR = [0.0, 90.0, 180.0, -90.0]
COL_AZ = (np.arange(8) - 3.5) * (45.0 / 8)

def sessions(recs):
    out, cur = [], [recs[0]]
    for a, b in zip(recs[:-1], recs[1:]):
        if b['host_ms'] < a['host_ms'] or b['host_ms'] - a['host_ms'] > 3000:
            out.append(cur); cur = []
        cur.append(b)
    out.append(cur)
    return out

def sensor_line(r, d):
    """body-frame line angle (deg, mod 180) through the 8 column points, plus mean range."""
    pts = []
    for c in range(8):
        v = r['grid'][d, 3:5, c]; v = v[(v > 150) & (v < 2500)]
        if len(v) < 2: continue
        rng = float(np.median(v)) / 1000.0
        az = math.radians(DIR[d] + COL_AZ[c])
        pts.append((rng * math.cos(az), rng * math.sin(az), rng))
    if len(pts) < 6: return None
    P = np.array(pts); X = P[:, :2] - P[:, :2].mean(0)
    w, V = np.linalg.eigh(X.T @ X / len(X))
    if math.sqrt(w[0]) > 0.02 or math.sqrt(w[1]) < 0.06: return None     # must be a clean line
    ang = math.degrees(math.atan2(V[1, 1], V[0, 1])) % 180.0
    return ang, float(P[:, 2].mean())

def wrap90(a):
    return (a + 90.0) % 180.0 - 90.0

def main():
    files = sorted(f for f in os.listdir(BASE) if f.endswith('_scanlog.bin'))
    seen = set()
    t1 = {d: [0, 0] for d in range(4)}         # pooled floor hits: [row0, row7]
    t2 = {d: [] for d in range(4)}             # pooled (dpsi, dangle)
    print(f"{'session':28s} | " + " | ".join(f"{n:^23s}" for n in NAMES))
    print(f"{'':28s} | " + " | ".join(f"{'floor r0/r7  slope(n)':^23s}" for _ in NAMES))
    for f in files:
        recs = load(os.path.join(BASE, f))
        if not recs: continue
        for k, s in enumerate(sessions(recs)):
            key = (len(s), s[0]['host_ms'], s[-1]['host_ms'])
            if key in seen or len(s) < 40: continue          # logs are appended: skip duplicate sessions
            seen.add(key)
            air = [r for r in s if r['alt'] > 0.3 and abs(math.degrees(r['roll'])) < 6 and abs(math.degrees(r['pitch'])) < 6]
            if len(air) < 20: continue
            cells = []
            for d in range(4):
                h = [0, 0]
                for r in air:
                    exp = r['alt'] / math.sin(math.radians(19.7))
                    for i, row in enumerate((0, 7)):
                        v = r['grid'][d, row, 2:6]; v = v[(v > 20) & (v < 4000)] / 1000.0
                        if len(v) >= 3 and abs(np.median(v) - exp) < 0.2 * exp: h[i] += 1
                t1[d][0] += h[0]; t1[d][1] += h[1]
                lines = [(r['host_ms'] / 1000.0, r['yaw'], sensor_line(r, d)) for r in air]
                lines = [(t, y, l) for t, y, l in lines if l]
                pairs = []
                for i in range(len(lines)):
                    for j in range(i + 1, len(lines)):
                        dt = lines[j][0] - lines[i][0]
                        if dt > 1.0: break
                        if dt < 0.2: continue
                        dpsi = wrap90(lines[j][1] - lines[i][1])
                        if abs(dpsi) < 2.0 or abs(dpsi) > 30: continue
                        if abs(lines[j][2][1] - lines[i][2][1]) > 0.25: continue     # same surface
                        dang = wrap90(lines[j][2][0] - lines[i][2][0])
                        pairs.append((dpsi, dang))
                t2[d] += pairs
                slope = (np.sum([p * a for p, a in pairs]) / np.sum([p * p for p, _ in pairs])) if len(pairs) >= 5 else float('nan')
                cells.append(f"{h[0]:3d}/{h[1]:<3d}  {slope:+5.2f}({len(pairs):3d})")
            print(f"{f.replace('_scanlog.bin','')+' s'+str(k+1):28s} | " + " | ".join(f"{c:^23s}" for c in cells))
    print("\nPOOLED over all unique sessions:")
    rng = np.random.default_rng(0)
    for d in range(4):
        r0, r7 = t1[d]
        up = "row 0 (same as firmware assumes for Front)" if r0 > r7 else "row 7 (opposite)"
        P = np.array(t2[d])
        if len(P) >= 10:
            sl = np.sum(P[:, 0] * P[:, 1]) / np.sum(P[:, 0] ** 2)
            bs = []
            for _ in range(2000):
                q = P[rng.integers(0, len(P), len(P))]
                bs.append(np.sum(q[:, 0] * q[:, 1]) / np.sum(q[:, 0] ** 2))
            lo, hi = np.percentile(bs, [2.5, 97.5])
            verdict = "MIRRORED (+1)" if lo > 0 else ("correct (-1)" if hi < 0 else "inconclusive")
            s2 = f"rotation slope {sl:+.2f} [95% CI {lo:+.2f}, {hi:+.2f}], n={len(P)} pairs -> {verdict}"
        else:
            s2 = f"rotation slope: too few pairs (n={len(P)})"
        print(f"  {NAMES[d]:5s}: floor in row0 {r0:4d} vs row7 {r7:4d} frames -> floor {up}; {s2}")

if __name__ == '__main__':
    main()
