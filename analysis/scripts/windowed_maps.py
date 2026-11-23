"""Plot a session as consecutive ~10 s windows, each on its own, plus the full stitched map.
If the windows are clean and only the stitched map is messy, the mess is pose drift between
windows rather than the sensor model. Same pipeline as cleanup_scanlog.py panel B
(plot_scan.py defaults + end filter + measured flow scale).

Usage: python windowed_maps.py <scanlog.bin> [window_s] [out.png]
"""
import os, sys
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
sys.path.append(os.path.join(HERE, '..', '..'))   # repo root plot_scan.py when no local copy
import plot_scan as ps
from cleanup_scanlog import quiet, split_sessions, process_session

def main():
    scan = sys.argv[1]
    win = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    name = os.path.basename(scan).replace('_scanlog.bin', '')
    out = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, '..', 'figures', f'{name}_windows.png')
    sess = max(split_sessions(quiet(ps.parse_log, scan)), key=len)
    panels, info = process_session(sess)
    (px, py, sid, tt), traj, _ = panels[1]                      # panel B
    t0, t1 = tt.min(), tt.max()
    edges = np.arange(t0, t1 + 1e-6, win)
    if t1 - edges[-1] > 0.4 * win: edges = np.append(edges, t1 + 1e-6)
    else: edges[-1] = t1 + 1e-6
    n = len(edges) - 1
    fig, axs = plt.subplots(1, n + 1, figsize=(4.6 * (n + 1), 5.6), sharex=True, sharey=True, layout='constrained')
    cmap = plt.get_cmap('viridis')
    for i in range(n):
        m = (tt >= edges[i]) & (tt < edges[i + 1])
        a = axs[i]
        for s_ in range(ps.NUM_SENSORS):
            k = m & (sid == s_)
            a.scatter(px[k], py[k], s=5, alpha=0.7, color=ps.SENSOR_COLORS[s_], label=ps.SENSOR_NAMES[s_])
        a.set_title(f"window {i+1}: {edges[i]-t0:.0f}-{min(edges[i+1], t1)-t0:.0f} s\n{m.sum()} pts", fontsize=10, loc='left')
        a.set_aspect('equal'); a.grid(True); a.set_xlabel('X (m)')
    a = axs[-1]
    for i in range(n):
        m = (tt >= edges[i]) & (tt < edges[i + 1])
        a.scatter(px[m], py[m], s=4, alpha=0.6, color=cmap(i / max(n - 1, 1)), label=f"window {i+1}")
    a.set_title("all windows stitched\n(colour = window)", fontsize=10, loc='left')
    a.set_aspect('equal'); a.grid(True); a.set_xlabel('X (m)')
    a.legend(fontsize=7, loc='upper right', markerscale=2)
    axs[0].set_ylabel('Y (m)'); axs[0].legend(fontsize=7, loc='upper right', markerscale=2)
    fig.suptitle(f"{name}: each {win:.0f} s window on its own vs all stitched (plot_scan.py pipeline + end filter + flow scale x{info['sx']:.2f}). "
                 "Clean windows + messy stitch = position drift between windows, not the sensor model.",
                 x=0.01, ha='left', fontsize=10)
    plt.savefig(out, dpi=105)
    print('wrote', os.path.abspath(out))

if __name__ == '__main__':
    main()
