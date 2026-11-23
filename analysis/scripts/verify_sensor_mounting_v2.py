"""Stricter mounting checks (supersedes test 1 of verify_sensor_mounting.py).
Test 1b: a real floor return must TRACK ALTITUDE (corr(range, alt)); fixed-range walls cannot.
Test 2b: rotation test with relaxed pairing for more samples: slope -1 = columns correct, +1 = mirrored.
Result (Nov 2026 logs): no sensor shows a floor row that tracks altitude; Right columns correct
(-0.69 [-0.82, -0.54], 611 pairs), Front control -0.53. No firmware change justified."""
import os, sys, math
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scanlog import load
from verify_sensor_mounting import sessions, sensor_line, wrap90, NAMES
BASE = os.path.join(os.environ.get('FLIGHT_DATA', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'flight-data')), 'onboard', 'scanlogs')
seen = set(); edge = {d: {k: [] for k in ('row0', 'row7', 'col0', 'col7')} for d in range(4)}
pairs = {d: [] for d in range(4)}
for f in sorted(os.listdir(BASE)):
    if not f.endswith('_scanlog.bin'): continue
    recs = load(os.path.join(BASE, f))
    for s in sessions(recs):
        key = (len(s), s[0]['host_ms'], s[-1]['host_ms'])
        if key in seen or len(s) < 40: continue
        seen.add(key)
        air = [r for r in s if r['alt'] > 0.25 and abs(math.degrees(r['roll'])) < 6 and abs(math.degrees(r['pitch'])) < 6]
        for r in air:
            for d in range(4):
                g = r['grid'][d]
                for k, z in (('row0', g[0, 2:6]), ('row7', g[7, 2:6]), ('col0', g[2:6, 0]), ('col7', g[2:6, 7])):
                    v = z[(z > 20) & (z < 4000)]
                    if len(v) >= 3: edge[d][k].append((r['alt'], np.median(v) / 1000.0))
        L = [(r['host_ms'] / 1000.0, r['yaw'], [sensor_line(r, d) for d in range(4)]) for r in air]
        for i in range(len(L)):
            for j in range(i + 1, len(L)):
                dt = L[j][0] - L[i][0]
                if dt > 2.0: break
                dpsi = wrap90(L[j][1] - L[i][1])
                if abs(dpsi) < 1.0 or abs(dpsi) > 30: continue
                for d in range(4):
                    a, b = L[i][2][d], L[j][2][d]
                    if a and b and abs(a[1] - b[1]) < 0.3:
                        pairs[d].append((dpsi, wrap90(b[0] - a[0])))
rng = np.random.default_rng(0)
print("Test 1b: does the edge row/column range TRACK ALTITUDE like a floor return (range ~ 2.96 x alt)?")
for d in range(4):
    out = []
    for k in ('row0', 'row7', 'col0', 'col7'):
        A = np.array(edge[d][k])
        if len(A) < 30: out.append(f"{k}: n/a"); continue
        ratio = A[:, 1] / A[:, 0]
        floorlike = np.abs(ratio - 2.96) < 0.3 * 2.96
        c = np.corrcoef(A[floorlike, 0], A[floorlike, 1])[0, 1] if floorlike.sum() > 20 else float('nan')
        allc = np.corrcoef(A[:, 0], A[:, 1])[0, 1]
        out.append(f"{k}: floor-like {floorlike.mean()*100:3.0f}%, corr(range,alt) {allc:+.2f}")
    print(f"  {NAMES[d]:5s} " + " | ".join(out))
print("\nTest 2b: rotation slope d(angle)/d(yaw): -1 = columns correct, +1 = mirrored")
for d in range(4):
    P = np.array(pairs[d])
    if len(P) < 10: print(f"  {NAMES[d]:5s}: n={len(P)} too few"); continue
    sl = np.sum(P[:, 0] * P[:, 1]) / np.sum(P[:, 0] ** 2)
    bs = [np.sum(q[:, 0] * q[:, 1]) / np.sum(q[:, 0] ** 2) for q in (P[rng.integers(0, len(P), len(P))] for _ in range(2000))]
    lo, hi = np.percentile(bs, [2.5, 97.5])
    med = np.median(P[:, 1] / P[:, 0])
    print(f"  {NAMES[d]:5s}: n={len(P):4d} pairs, slope {sl:+.2f} [95% CI {lo:+.2f}, {hi:+.2f}], median ratio {med:+.2f}, "
          f"sign agreement with -1: {np.mean(np.sign(P[:,1]) == -np.sign(P[:,0]))*100:.0f}%")
