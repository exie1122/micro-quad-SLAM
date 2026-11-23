"""Offline map clean-up prototype for the micro-quad ToF scan logs (floor1 by default).

Maps produced from the same scanlog:
  A. As flown:  the onboard model (70 deg FOV, 2nd-smallest of all 8 rows, no tilt) + EKF poses.
  B. Corrected sensor model (45 deg FOV, middle rows 2-5, tilt-compensated rays, floor hits
     removed) + EKF odometry rescaled by a flow-scale factor MEASURED FROM THIS FLIGHT:
     when the drone moves a distance D along a sensor's centre ray toward a fixed surface, that
     range must drop by exactly D. The median of (range change / odometry displacement) gives
     true/odometry scale per body axis, independent of map-quality metrics.
  C. B + gated scan matching: 1 s chunks (5 scans, ~115 pts) matched point-to-line against the
     previous ~6 s of map with an odometry prior; a correction is accepted only if it is small,
     well supported and the residual matches sensor noise; then a pose graph (odometry +
     accepted matches + gated loop closures) is solved jointly.

Lessons from earlier versions (kept here so they are not repeated):
  - v1 matched single 23-point scans to the submap with loose gating: correspondences in new
    territory snapped to old walls and the trajectory COLLAPSED (2.95 m "correction").
  - Median local wall thickness is dominated by the ~20 cm zone footprint: it changed < 0.3 cm
    across a +/-40 % scale range, so it cannot validate trajectories.
  - Revisit consistency (distance between copies of a surface seen >= 5 s apart) is sensitive
    but rewards shrinking the trajectory, so it is only compared at a fixed scale (B vs C),
    together with collapse guards (path-length ratio, max shift).

Usage: python scanmatch_floor1.py [scanlog.bin] [out.png]
"""
import sys, os, math
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from scanlog import load  # noqa: E402

DIR_DEG = [0.0, 90.0, 180.0, -90.0]        # F, R, B, L relative to heading (NED, clockwise +)
MAX_RANGE = 3.0                              # m; VL53L5CX 8x8 gets noisy beyond this indoors
SENSOR_NOISE = 0.05                          # m; observed ICP residual (zone footprint ~5.6 deg)

# ------------------------------------------------------------------ SE(2) helpers
def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def R2(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s], [s, c]])

def to_world(pose, pts):
    return pts @ R2(pose[2]).T + pose[:2]

def rel(a, b):
    d = R2(-a[2]) @ (b[:2] - a[:2])
    return np.array([d[0], d[1], wrap(b[2] - a[2])])

def compose(a, d):
    p = a[:2] + R2(a[2]) @ d[:2]
    return np.array([p[0], p[1], wrap(a[2] + d[2])])

# ------------------------------------------------------------------ sensor models
def rot_body_to_level(roll, pitch):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return Ry @ Rx                                       # FRD body -> levelled (yaw-only) frame

def scan_points_corrected(r):
    """Levelled body-frame points (x fwd, y right): middle rows, 45 deg FOV, tilt-compensated,
    floor/ceiling hits rejected."""
    g = r['grid']; R = rot_body_to_level(r['roll'], r['pitch'])
    alt = r['alt'] if np.isfinite(r['alt']) else 0.5
    pts = []
    for d in range(4):
        for c in range(8):
            v = g[d, 2:6, c]
            v = v[(v > 20) & (v < 4000)]
            if len(v) < 2:
                continue
            dist = float(np.median(v)) / 1000.0
            if dist > MAX_RANGE:
                continue
            off = math.radians(((c + 0.5) / 8.0 - 0.5) * 45.0)
            h = math.radians(DIR_DEG[d]) + off
            dist = dist / math.cos(off)        # sensor reports perpendicular depth (verify_range_model.py)
            p = (R @ np.array([math.cos(h), math.sin(h), 0.0])) * dist
            z_above_ground = alt - p[2]                    # FRD: +z is down
            if z_above_ground < 0.08 or z_above_ground > alt + 1.2:
                continue
            pts.append((p[0], p[1]))
    return np.array(pts).reshape(-1, 2)

def scan_points_asflown(r):
    """The onboard frontier.c model the drone flew with."""
    g = r['grid']; pts = []
    for d in range(4):
        for c in range(8):
            col = g[d, :, c]
            v = np.sort(col[(col > 20) & (col != 0xFFFF)])
            if len(v) == 0:
                continue
            dist = min((v[1] if len(v) > 1 else v[0]) / 1000.0, 4.0)
            if dist > 3.5:
                continue
            h = math.radians(DIR_DEG[d] + ((c + 0.5) / 8.0 - 0.5) * 70.0)
            pts.append((dist * math.cos(h), dist * math.sin(h)))
    return np.array(pts).reshape(-1, 2)

# ------------------------------------------------------------------ flow-scale estimate from ToF
def center_range(r, d):
    v = r['grid'][d, 2:6, 3:5].ravel()
    v = v[(v > 20) & (v < 3500)]
    return np.median(v) / 1000.0 if len(v) >= 4 and np.ptp(v) < 120 else np.nan  # same surface

def estimate_flow_scale(recs, t, ekf, rng):
    """true/odometry scale per body axis from ToF range changes along each sensor's centre ray."""
    ratios = {0: [], 1: []}
    for m in (1, 2, 3):
        for k in range(len(recs) - m):
            if t[k + m] - t[k] > 0.9 or abs(math.degrees(wrap(ekf[k + m, 2] - ekf[k, 2]))) > 1.5:
                continue
            b = R2(-ekf[k, 2]) @ (ekf[k + m, :2] - ekf[k, :2])          # body-frame displacement
            for sens, axis, sign in ((0, 0, 1), (2, 0, -1), (1, 1, 1), (3, 1, -1)):
                along = sign * b[axis]; lateral = b[1 - axis]
                if abs(along) < 0.06 or abs(lateral) > 0.4 * abs(along):
                    continue
                r0, r1 = center_range(recs[k], sens), center_range(recs[k + m], sens)
                if not (np.isfinite(r0) and np.isfinite(r1)):
                    continue
                dr = r1 - r0
                if abs(dr) > 0.6 or dr * along > 0:
                    continue
                ratios[axis].append(-dr / along)
    est = {}
    for axis in (0, 1):
        a = np.array(ratios[axis])
        if len(a) >= 20:
            boot = [np.median(rng.choice(a, len(a))) for _ in range(2000)]
            lo, hi = np.percentile(boot, [2.5, 97.5])
            est[axis] = (float(np.median(a)), float(lo), float(hi), len(a))
        else:
            est[axis] = (np.nan, np.nan, np.nan, len(a))
    # Use an axis only if its interval is tight; otherwise borrow the other axis.
    usable = {ax: v[0] for ax, v in est.items() if np.isfinite(v[0]) and (v[2] - v[1]) <= 0.15}
    if not usable:
        return 1.0, 1.0, est
    fallback = float(np.mean(list(usable.values())))
    return usable.get(0, fallback), usable.get(1, fallback), est

def rescale_odometry(ekf, sx, sy):
    P = ekf.copy()
    for k in range(1, len(ekf)):
        b = R2(-ekf[k - 1, 2]) @ (ekf[k, :2] - ekf[k - 1, :2])
        P[k, :2] = P[k - 1, :2] + R2(ekf[k - 1, 2]) @ np.array([b[0] * sx, b[1] * sy])
    return P

# ------------------------------------------------------------------ ICP
def normals(tree, ref, radius=0.15, min_nb=4):
    n = np.full(ref.shape, np.nan); lin = np.zeros(len(ref))
    for i, nb in enumerate(tree.query_ball_point(ref, radius)):
        if len(nb) < min_nb:
            continue
        P = ref[nb] - ref[nb].mean(0)
        w, V = np.linalg.eigh(P.T @ P / len(nb))
        n[i] = V[:, 0]
        lin[i] = 1.0 - w[0] / max(w[1], 1e-9)
    return n, lin

def icp_point_to_line(src, ref, init, prior_sigma, iters=20, max_d=0.2, huber=0.05):
    """Returns pose, inliers, rms, min eigenvalue of the data Hessian (translation block)."""
    if len(ref) < 20 or len(src) < 6:
        return init.copy(), 0, np.inf, 0.0
    tree = cKDTree(ref)
    nrm, lin = normals(tree, ref)
    good_ref = np.isfinite(nrm[:, 0]) & (lin > 0.6)
    pose = init.copy(); W_prior = np.diag(1.0 / np.square(prior_sigma))
    n_inl, rms, cond = 0, np.inf, 0.0
    for _ in range(iters):
        w = to_world(pose, src)
        dist, idx = tree.query(w)
        m = (dist < max_d) & good_ref[idx]
        n_inl = int(m.sum())
        if n_inl < 6:
            break
        q, nn, p_loc = ref[idx[m]], nrm[idx[m]], src[m]
        e = np.einsum('ij,ij->i', w[m] - q, nn)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        dRp = np.stack([-s * p_loc[:, 0] - c * p_loc[:, 1], c * p_loc[:, 0] - s * p_loc[:, 1]], 1)
        J = np.stack([nn[:, 0], nn[:, 1], np.einsum('ij,ij->i', dRp, nn)], 1)
        wts = np.where(np.abs(e) < huber, 1.0, huber / np.abs(e)) / SENSOR_NOISE ** 2
        Hd = J.T @ (J * wts[:, None])
        dp = pose - init; dp[2] = wrap(dp[2])
        step = -np.linalg.solve(Hd + W_prior, J.T @ (wts * e) + W_prior @ dp)
        pose = pose + step; pose[2] = wrap(pose[2])
        rms = float(np.sqrt(np.mean(e ** 2)))
        cond = float(np.linalg.eigvalsh(Hd[:2, :2])[0])
        if np.linalg.norm(step[:2]) < 1e-4 and abs(step[2]) < 1e-4:
            break
    return pose, n_inl, rms, cond

def voxel(pts, v=0.03):
    if len(pts) == 0:
        return pts
    k = np.floor(pts / v).astype(np.int64)
    _, i = np.unique(k, axis=0, return_index=True)
    return pts[np.sort(i)]

# ------------------------------------------------------------------ metrics
def build_map(poses, scans, t):
    pts, tt = [], []
    for p, s, ti in zip(poses, scans, t):
        if len(s):
            w = to_world(p, s); pts.append(w); tt.append(np.full(len(w), ti))
    return np.vstack(pts), np.concatenate(tt)

def revisit_consistency(pts, tt, gap=5.0, r=0.3):
    """median distance (cm) from each point to the nearest point seen >= gap s apart"""
    tree = cKDTree(pts); d = []
    for i, nb in enumerate(tree.query_ball_point(pts, r)):
        nb = [j for j in nb if abs(tt[j] - tt[i]) >= gap]
        if nb:
            d.append(np.min(np.hypot(*(pts[nb] - pts[i]).T)))
    return float(np.median(d)) * 100.0, len(d)

def path_len(P):
    return float(np.sum(np.hypot(*np.diff(P[:, :2], axis=0).T)))

# ------------------------------------------------------------------ main
def main():
    scan_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        HERE, '..', '..', 'flight-data', 'onboard', 'scanlogs', 'floor1_scanlog.bin')
    out_png = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '..', 'figures', 'floor1_scanmatch.png')
    rng = np.random.default_rng(0)

    recs = [r for r in load(scan_path) if np.isfinite(r['alt']) and r['alt'] > 0.25]
    t = np.array([r['host_ms'] for r in recs]) / 1000.0
    ekf = np.array([[r['x'], r['y'], math.radians(r['yaw'])] for r in recs])
    scans_c = [scan_points_corrected(r) for r in recs]
    scans_a = [scan_points_asflown(r) for r in recs]
    print(f"{len(recs)} airborne scans, {t[-1]-t[0]:.1f} s, mean pts/scan (corrected) {np.mean([len(s) for s in scans_c]):.1f}")

    # ---- B: flow scale measured from ToF
    sx, sy, est = estimate_flow_scale(recs, t, ekf, rng)
    for ax, name in ((0, 'body X'), (1, 'body Y')):
        m, lo, hi, n = est[ax]
        print(f"flow scale {name}: n={n} median={m:.3f} 95% CI [{lo:.3f}, {hi:.3f}]")
    print(f"applied scale: sx={sx:.3f} sy={sy:.3f}")
    odoB = rescale_odometry(ekf, sx, sy)

    # ---- C: gated chunk scan matching on top of B
    CH = 5
    chunks = [list(range(i, min(i + CH, len(recs)))) for i in range(0, len(recs), CH)]
    anchor = [c[len(c) // 2] for c in chunks]

    def chunk_cloud(ci, poses):
        a0 = poses[anchor[ci]]; out = []
        for j in chunks[ci]:
            if len(scans_c[j]):
                out.append((to_world(poses[j], scans_c[j]) - a0[:2]) @ R2(a0[2]))
        return np.vstack(out) if out else np.zeros((0, 2))

    local = [chunk_cloud(ci, odoB) for ci in range(len(chunks))]
    odo_anchor = odoB[anchor]
    odo_ch = [rel(odo_anchor[i - 1], odo_anchor[i]) for i in range(1, len(chunks))]

    def gated(src, ref, init, prior, max_dxy, max_dth, min_inl):
        pose, n_inl, rms, cond = icp_point_to_line(src, ref, init, prior)
        d = pose - init; d[2] = wrap(d[2])
        ok = (n_inl >= min_inl and rms <= 1.4 * SENSOR_NOISE and cond > 2000.0 and
              math.hypot(d[0], d[1]) <= max_dxy and abs(d[2]) <= max_dth)
        return pose, ok, d

    seq = [odo_anchor[0].copy()]; seq_ok = []; acc = []
    for i in range(1, len(chunks)):
        init = compose(seq[-1], odo_ch[i - 1])
        ref_parts = [to_world(seq[j], local[j]) for j in range(max(0, i - 6), i) if len(local[j])]
        ref = voxel(np.vstack(ref_parts)) if ref_parts else np.zeros((0, 2))
        pose, ok, d = gated(local[i], ref, init, np.array([0.04, 0.04, math.radians(2.0)]),
                            0.08, math.radians(3.0), 40)
        seq.append(pose if ok else init); seq_ok.append(ok)
        if ok:
            acc.append(math.hypot(d[0], d[1]))
    seq = np.array(seq)
    print(f"chunks={len(chunks)}: sequential matches accepted {np.mean(seq_ok)*100:.0f}% "
          f"(median accepted correction {np.median(acc)*100 if acc else 0:.1f} cm)")

    loops = []
    tc = t[anchor]
    for i in range(len(chunks)):
        best = None
        for j in range(i):
            if tc[i] - tc[j] < 10.0 or np.hypot(*(seq[i, :2] - seq[j, :2])) > 1.0:
                continue
            parts = [to_world(seq[q], local[q]) for q in range(max(0, j - 1), min(len(chunks), j + 2)) if len(local[q])]
            if not parts:
                continue
            pose, ok, d = gated(local[i], voxel(np.vstack(parts)), seq[i].copy(),
                                np.array([0.2, 0.2, math.radians(5.0)]), 0.30, math.radians(5.0), 60)
            if ok and (best is None or math.hypot(d[0], d[1]) < best[3]):
                best = (j, i, rel(seq[j], pose), math.hypot(d[0], d[1]))
        if best:
            loops.append(best[:3])
    print(f"loop closures accepted: {len(loops)}")

    edges = []
    for i in range(1, len(chunks)):
        dist = math.hypot(odo_ch[i - 1][0], odo_ch[i - 1][1])
        edges.append((i - 1, i, odo_ch[i - 1],
                      np.array([0.02 + 0.06 * dist, 0.02 + 0.06 * dist, math.radians(1.0 + 2.0 * dist)])))
        if seq_ok[i - 1]:
            edges.append((i - 1, i, rel(seq[i - 1], seq[i]), np.array([0.025, 0.025, math.radians(0.8)])))
    for j, i, z in loops:
        edges.append((j, i, z, np.array([0.03, 0.03, math.radians(1.0)])))
    I = np.array([e[0] for e in edges]); J = np.array([e[1] for e in edges])
    Z = np.array([e[2] for e in edges]); SIG = np.array([e[3] for e in edges])

    def residuals(x):
        P = x.reshape(-1, 3); a, b = P[I], P[J]
        c, s = np.cos(a[:, 2]), np.sin(a[:, 2])
        dx, dy = b[:, 0] - a[:, 0], b[:, 1] - a[:, 1]
        r = np.stack([c * dx + s * dy - Z[:, 0], -s * dx + c * dy - Z[:, 1],
                      np.angle(np.exp(1j * (b[:, 2] - a[:, 2] - Z[:, 2])))], 1) / SIG
        return np.concatenate([r.ravel(), (P[0] - odo_anchor[0]) / 1e-3])
    opt_ch = least_squares(residuals, seq.ravel(), loss='huber', f_scale=2.0, max_nfev=300).x.reshape(-1, 3)
    optC = np.zeros_like(ekf)
    for ci, c in enumerate(chunks):
        for j in c:
            optC[j] = compose(opt_ch[ci], rel(odoB[anchor[ci]], odoB[j]))

    # ---- metrics
    mA, tA = build_map(ekf, scans_a, t)
    mB, tB = build_map(odoB, scans_c, t)
    mC, tC = build_map(optC, scans_c, t)
    m0, t0 = build_map(ekf, scans_c, t)
    rA, _ = revisit_consistency(mA, tA); r0, _ = revisit_consistency(m0, t0)
    rB, nB = revisit_consistency(mB, tB); rC, nC = revisit_consistency(mC, tC)
    pr = path_len(optC) / path_len(odoB)
    shiftC = float(np.max(np.hypot(*(optC[:, :2] - odoB[:, :2]).T)))
    print(f"revisit consistency (median cm, lower = fewer double walls): "
          f"A {rA:.2f} | corrected sensor @EKF scale {r0:.2f} | B (scale-corrected) {rB:.2f} | C {rC:.2f}")
    print(f"C vs B collapse guards: path length ratio {pr:.3f}, max shift {shiftC:.2f} m")

    # ---- figure
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    INK, INK2, SURF, BLUE, ORANGE = '#0b0b0b', '#52514e', '#fcfcfb', '#2a78d6', '#eb6834'
    plt.rcParams.update({'font.size': 9, 'figure.facecolor': SURF, 'axes.facecolor': SURF,
                         'axes.edgecolor': '#c9c8c3', 'xtick.color': INK2, 'ytick.color': INK2})
    fig, ax = plt.subplots(1, 3, figsize=(18, 7.2), sharex=True, sharey=True)
    panels = [
        (mA, ekf, None, f"A. As flown: 70° FOV, all rows, no tilt, EKF poses\nrevisit consistency {rA:.1f} cm"),
        (mB, odoB, ekf, f"B. Corrected sensor model + measured flow scale (x{sx:.2f}, x{sy:.2f})\n"
                        f"revisit consistency {rB:.1f} cm (sensor fix alone: {r0:.1f} cm)"),
        (mC, optC, odoB, f"C. B + gated scan matching ({np.mean(seq_ok)*100:.0f}% chunks matched, {len(loops)} loop closures)\n"
                         f"revisit consistency {rC:.1f} cm; path ratio {pr:.2f}, max shift {shiftC:.2f} m"),
    ]
    for a, (m, P, ref, title) in zip(ax, panels):
        a.scatter(m[:, 1], m[:, 0], s=1.2, color=INK, alpha=0.55, linewidths=0, label='ToF points')
        if ref is not None:
            a.plot(ref[:, 1], ref[:, 0], color=ORANGE, lw=1.1, alpha=0.8,
                   label='EKF trajectory' if ref is ekf else 'trajectory from B')
        a.plot(P[:, 1], P[:, 0], color=BLUE, lw=1.6, label='trajectory used for this map')
        a.set_title(title, loc='left', fontsize=9.5, color=INK)
        a.set_aspect('equal'); a.grid(color='#ecebe7', lw=0.6); a.set_xlabel('East (m)')
        a.legend(loc='lower left', fontsize=7, frameon=False, markerscale=6)
    ax[0].set_ylabel('North (m)')
    ex, ey = est[0], est[1]
    y_ok = np.isfinite(ey[0]) and (ey[2] - ey[1]) <= 0.15
    ytxt = (f"body Y {ey[0]:.2f} [{ey[1]:.2f}-{ey[2]:.2f}]" if y_ok
            else "body Y unusable on this flight (too few clean side-wall approaches), so X value used")
    fig.suptitle(f"floor1: {len(recs)} scans over {t[-1]-t[0]:.0f} s.  Optical-flow scale measured from ToF range changes "
                 f"(true / EKF distance): body X {ex[0]:.2f} [95% CI {ex[1]:.2f}-{ex[2]:.2f}]; {ytxt}.\n"
                 "Revisit consistency = median distance between copies of the same surface seen >= 5 s apart "
                 "(lower = fewer double walls). Compare B vs C directly; A uses a different sensor model.",
                 x=0.01, ha='left', fontsize=9, color=INK2)
    plt.tight_layout(rect=(0, 0, 1, 0.93))
    os.makedirs(os.path.dirname(os.path.abspath(out_png)), exist_ok=True)
    plt.savefig(out_png, dpi=130)
    print('wrote', os.path.abspath(out_png))

if __name__ == '__main__':
    main()
