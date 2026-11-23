"""Map clean-up on top of the project's own plot_scan.py pipeline, for any scanlog.

The scanlog is first split into SESSIONS (the onboard logger appends: e.g. 4_hallWay_scanlog.bin
holds the 3_hallWay flight followed by the 4_hallWay flight, each starting at EKF (0,0)).
Each session gets one row of four panels:

A. plot_scan.py defaults exactly as used for the GitHub figures: filter aggressiveness 2.0,
   startup filter on, middle rows {3,4}, 3D rays, per-frame isolated-point rejection,
   Manhattan off.
B. A + END filter (drop everything after the last stable hover, and from 1 s before the first
   crash-like tilt > 25 deg) + optical-flow scale correction measured from the session's own
   ToF data (moving D toward a fixed surface along a sensor's centre ray must shorten that
   range by exactly D; no correction if neither axis gives a tight estimate).
C. (removed) An earlier version mirrored the right/back sensors' columns. A direct test
   (verify_sensor_mounting.py: how a sensor's measured wall angle changes as the drone yaws)
   showed the right sensor's columns are CORRECT (slope -0.69, 95% CI -0.82..-0.54, 611 pairs;
   front control -0.53), and the 'floor row' evidence for upside-down mounting did not survive
   an altitude-tracking check. The map-metric gain from mirroring was not a real correction.
   Panel C is therefore identical to B, and D is B + the display filter.
D. C + multi-frame support filter (DISPLAY ONLY): keep a point only if >= SUPPORT_MIN points
   from other moments (>= SUPPORT_DT_S apart) lie within SUPPORT_R_M. Removes one-off returns.

Tested and NOT applied (inconsistent across flights): Manhattan yaw correction (made floor1
worse; walls are not all orthogonal), per-sensor yaw trims, scan-to-pose latency.

plot_scan.py is the project's own viewer, copied unmodified from the GitHub repo.

Usage: python cleanup_scanlog.py <scanlog.bin> [out.png]
"""
import os, sys, math, io, contextlib
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.append(os.path.join(HERE, '..', '..'))   # repo root plot_scan.py when no local copy
import plot_scan as ps                                   # noqa: E402
from scanmatch_floor1 import estimate_flow_scale, rescale_odometry, revisit_consistency  # noqa: E402

CRASH_TILT_DEG = 25.0
CRASH_PRE_CUT_S = 1.0
MIN_SESSION_FRAMES = 30
MIRRORED_SENSORS = ()              # none: column order verified correct (see docstring)
SUPPORT_R_M = 0.10
SUPPORT_DT_S = 0.5
SUPPORT_MIN = 3

def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)

def split_sessions(recs):
    t = [r['t_ms'] for r in recs]
    br = [0] + [i + 1 for i in range(len(t) - 1) if t[i + 1] < t[i] or t[i + 1] - t[i] > 3000] + [len(t)]
    return [recs[a:b] for a, b in zip(br[:-1], br[1:])]

def end_cut_t_ms(recs, startup_t_ms):
    """min(last stable-hover frame, 1 s before first crash-like tilt after startup)."""
    run, last_end = 0, None
    for i, r in enumerate(recs):
        ok = r['alt'] >= ps.MIN_STABLE_ALT_M and abs(r['vz']) < ps.STABLE_HOVER_VZ_THRESH
        run = run + 1 if ok else 0
        if run >= ps.STABLE_HOVER_CONSEC_FRAMES:
            last_end = i
    cut = recs[last_end]['t_ms'] if last_end is not None else None
    reason = 'last stable hover'
    for r in recs:
        if r['t_ms'] <= startup_t_ms:
            continue
        if max(abs(math.degrees(r['roll'])), abs(math.degrees(r['pitch']))) > CRASH_TILT_DEG:
            crash_cut = r['t_ms'] - int(CRASH_PRE_CUT_S * 1000)
            if cut is None or crash_cut < cut:
                cut, reason = crash_cut, f"crash at {r['t_ms']/1000:.1f}s"
            break
    return cut, reason

def mirror_offsets(sensors):
    col = (np.arange(ps.COLS) - (ps.COLS - 1) / 2.0) * (ps.FOV_H / ps.COLS)
    off = np.zeros((ps.NUM_SENSORS, ps.COLS), dtype=np.float32)
    for s in sensors:
        off[s] = -2.0 * col                       # azimuth -> -azimuth for that sensor
    return off

def points_with_time(recs, profile, col_offsets=None):
    px, py, sid, frame_idx, _ = quiet(ps.compute_points, recs, column_az_offsets_deg=col_offsets,
                                      filter_profile=profile, row_filter_indices=set(ps.MIDDLE_ROW_INDICES))
    tt = np.zeros(len(px))
    for k in range(len(recs)):
        tt[frame_idx[k]:frame_idx[k + 1]] = recs[k]['t_ms'] / 1000.0
    return px, py, sid, tt

def support_filter(P):
    px, py, sid, tt = P
    if len(px) == 0:
        return P, 0
    pts = np.column_stack([px, py]); tree = cKDTree(pts)
    keep = np.zeros(len(px), dtype=bool)
    for i, nb in enumerate(tree.query_ball_point(pts, SUPPORT_R_M)):
        keep[i] = sum(1 for j in nb if abs(tt[j] - tt[i]) >= SUPPORT_DT_S) >= SUPPORT_MIN
    return (px[keep], py[keep], sid[keep], tt[keep]), int((~keep).sum())

def process_session(recs):
    prof = ps.build_filter_profile(ps.DEFAULT_FILTER_AGGRESSIVENESS)
    prof['stable_hover_t_ms'] = ps.compute_stable_hover_t_ms(recs)
    A = points_with_time(recs, prof)

    cut, why = end_cut_t_ms(recs, prof['stable_hover_t_ms'])
    recsB = [dict(r) for r in recs]
    dropped = 0
    for r in recsB:
        if cut is not None and r['t_ms'] > cut:
            dropped += int(ps.frame_pose_is_usable(r, prof))
            r['pose_flags'] = 0                          # fails REQUIRED_POSE_FLAGS -> skipped
    air = [i for i, r in enumerate(recsB) if ps.frame_pose_is_usable(r, prof)]
    sub = [{'grid': np.frombuffer(recsB[i]['grid_bytes'], dtype='<u2').reshape(4, 8, 8).astype(float),
            'alt': recsB[i]['alt']} for i in air]
    t = np.array([recsB[i]['t_ms'] for i in air]) / 1000.0
    ekf_sub = np.array([[recsB[i]['x'], recsB[i]['y'], math.radians(recsB[i]['yaw'])] for i in air])
    sx, sy, est = estimate_flow_scale(sub, t, ekf_sub, np.random.default_rng(0))
    scaled = rescale_odometry(np.array([[r['x'], r['y'], math.radians(r['yaw'])] for r in recsB]), sx, sy)
    for r, p in zip(recsB, scaled):
        r['x'], r['y'] = float(p[0]), float(p[1])
    B = points_with_time(recsB, prof)
    C = points_with_time(recsB, prof, mirror_offsets(MIRRORED_SENSORS))
    D, removed = support_filter(C)

    def rc(P):
        return revisit_consistency(np.column_stack([P[0], P[1]]), P[3])[0] if len(P[0]) > 20 else float('nan')
    def traj(rs):
        return np.array([[r['x'], r['y']] for r in rs if ps.frame_pose_is_usable(r, prof)]).reshape(-1, 2)
    trA, trB = traj(recs), traj(recsB)
    info = dict(n=len(recs), t0=recs[0]['t_ms'] / 1000, t1=recs[-1]['t_ms'] / 1000,
                path=float(np.sum(np.hypot(*np.diff(trB, axis=0).T))) if len(trB) > 1 else 0.0,
                startup=prof['stable_hover_t_ms'] / 1000, cut=(cut / 1000 if cut else None), why=why,
                dropped=dropped, sx=sx, sy=sy, est=est, removed=removed)
    return [(A, trA, rc(A)), (B, trB, rc(B)), (D, trB, None)], info

def scale_text(info):
    def ax_txt(ax_name, e):
        m, lo, hi, n = e
        if not np.isfinite(m):
            return f"{ax_name}: too few samples (n={n})"
        tag = "" if (hi - lo) <= 0.15 else " (too uncertain, not used)"
        return f"{ax_name} {m:.2f} [95% CI {lo:.2f}-{hi:.2f}]{tag}"
    applied = ("no correction applied" if info['sx'] == 1.0 and info['sy'] == 1.0
               else f"applied x{info['sx']:.2f}, x{info['sy']:.2f}")
    return f"flow scale {ax_txt('X', info['est'][0])}; {ax_txt('Y', info['est'][1])}; {applied}"

def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    scan = sys.argv[1]
    name = os.path.basename(scan).replace('_scanlog.bin', '').replace('.bin', '')
    out_png = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '..', 'figures', f'{name}_cleanup.png')

    sessions = [s for s in split_sessions(quiet(ps.parse_log, scan)) if len(s) >= MIN_SESSION_FRAMES]
    results = []
    for k, s in enumerate(sessions):
        panels, info = process_session(s)
        print(f"session {k+1}: t {info['t0']:.1f}-{info['t1']:.1f}s, path {info['path']:.1f} m, end cut ({info['why']}) "
              f"dropped {info['dropped']} frames; {scale_text(info)}; revisit cm A {panels[0][2]:.2f} "
              f"B {panels[1][2]:.2f}; support filter removed {info['removed']} pts")
        results.append((panels, info))

    rows = len(results)
    fig = plt.figure(figsize=(19, 7.2 * rows + 0.9), layout='constrained')
    fig.suptitle(f"{name}: {rows} session(s), each plotted separately (no common origin). Pipeline and colours of plot_scan.py "
                 "(filter 2.0, startup filter, rows 3-4, 3D rays).\n"
                 "Revisit consistency = median distance between copies of a surface seen >= 5 s apart "
                 "(lower = fewer double walls; compare within a row).",
                 x=0.01, ha='left', fontsize=10)
    subfigs = np.atleast_1d(fig.subfigures(rows, 1, hspace=0.04))
    for row, ((panels, info), sf) in enumerate(zip(results, subfigs)):
        sf.suptitle(f"Session {row+1}: t {info['t0']:.0f}-{info['t1']:.0f} s, {info['path']:.1f} m path.\n{scale_text(info)}\n",
                    x=0.01, ha='left', fontsize=10, color='#52514e')
        axs = sf.subplots(1, 3, sharex=True, sharey=True)
        titles = ["A. plot_scan.py defaults (as on GitHub)",
                  f"B. + end filter ({info['dropped']} frames after {info['why']})\n   + flow scale "
                  + ("(not measurable)" if info['sx'] == 1.0 and info['sy'] == 1.0 else f"x{info['sx']:.2f}"),
                  f"C. B + multi-frame support filter (display only)\n   {info['removed']:,} one-off points hidden"]
        allpts = np.concatenate([np.column_stack([p[0][0], p[0][1]]) for p in panels if len(p[0][0])] or [np.zeros((1, 2))])
        lo, hi = np.percentile(allpts, 0.5, axis=0) - 0.4, np.percentile(allpts, 99.5, axis=0) + 0.4
        for (P, tr, r), title, a in zip(panels, titles, axs):
            for s_ in range(ps.NUM_SENSORS):
                m = P[2] == s_
                a.scatter(P[0][m], P[1][m], s=4, alpha=0.6, color=ps.SENSOR_COLORS[s_], label=ps.SENSOR_NAMES[s_])
            if len(tr):
                a.plot(tr[:, 0], tr[:, 1], 'r-', lw=0.9, label='Trajectory')
            sub_t = f"{len(P[0]):,} pts" + (f", revisit consistency {r:.1f} cm" if r is not None else "")
            a.set_title(f"{title}\n{sub_t}", fontsize=10, loc='left')
            a.set_xlim(lo[0], hi[0]); a.set_ylim(lo[1], hi[1])
            a.set_aspect('equal'); a.grid(True); a.set_xlabel('X (m)')
        axs[0].set_ylabel('Y (m)')
        axs[0].legend(loc='upper right', fontsize=7, markerscale=2)
    fig.get_layout_engine().set(rect=(0, 0.03, 1, 0.97))
    plt.savefig(out_png, dpi=110)
    print('wrote', os.path.abspath(out_png))

if __name__ == '__main__':
    main()
