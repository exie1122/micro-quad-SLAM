#!/usr/bin/env python3
"""
ORB-on-CPU benchmark for the LicheeRV Nano (SG2002, single C906 @ ~1 GHz).

Measures, per resolution, averaged over many frames:
  - ORB detect+compute time / frame
  - descriptor matching time (BFMatcher / Hamming) between consecutive frames
  - total ms/frame and FPS
  - peak process RAM (RSS)
  - which CPU core the work landed on

Frames are synthetic: a textured base image is warped (pan + tiny rotation) each
frame to simulate smooth camera motion, so consecutive frames share many
matchable ORB features. No external image files needed.

Self-contained: only needs numpy + OpenCV (both present on the board).
"""
import os
import time
import argparse
import resource
import numpy as np
import cv2


def make_base_texture(h, w, seed=0):
    """A feature-rich grayscale texture: noise + random shapes (lots of corners)."""
    rng = np.random.default_rng(seed)
    img = (rng.integers(0, 60, size=(h, w), dtype=np.uint8))  # dim noise floor
    n = max(40, (h * w) // 1500)
    for _ in range(n):
        x1, y1 = rng.integers(0, w), rng.integers(0, h)
        x2, y2 = rng.integers(0, w), rng.integers(0, h)
        val = int(rng.integers(80, 255))
        kind = rng.integers(0, 3)
        if kind == 0:
            cv2.rectangle(img, (x1, y1), (x2, y2), val, thickness=int(rng.integers(1, 4)))
        elif kind == 1:
            r = int(rng.integers(3, 25))
            cv2.circle(img, (x1, y1), r, val, thickness=int(rng.integers(1, 3)))
        else:
            cv2.line(img, (x1, y1), (x2, y2), val, thickness=int(rng.integers(1, 3)))
    return img


def warped_frame(base, i):
    """Frame i: translate + tiny rotation to simulate motion. Borders reflected."""
    h, w = base.shape
    angle = i * 0.15            # degrees
    dx, dy = i * 1.0, i * 0.5   # pixels
    M = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), angle, 1.0)
    M[0, 2] += dx
    M[1, 2] += dy
    return cv2.warpAffine(base, M, (w, h), borderMode=cv2.BORDER_REFLECT)


def current_cpu():
    """Last CPU this thread executed on (field 39 of /proc/self/stat)."""
    try:
        with open("/proc/self/stat") as f:
            return int(f.read().split()[38])
    except Exception:
        return -1


def bench(w, h, n_frames, n_features, warmup=5):
    base = make_base_texture(h, w, seed=42)
    orb = cv2.ORB_create(nfeatures=n_features)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # warmup (JIT caches, allocator, etc.)
    prev_des = None
    for i in range(warmup):
        f = warped_frame(base, i)
        _, prev_des = orb.detectAndCompute(f, None)

    det_times, match_times, kp_counts, match_counts = [], [], [], []
    prev_des = None
    cpu_seen = set()
    for i in range(n_frames):
        frame = warped_frame(base, i + warmup)

        t0 = time.perf_counter()
        kp, des = orb.detectAndCompute(frame, None)
        t1 = time.perf_counter()
        det_times.append((t1 - t0) * 1000.0)
        kp_counts.append(len(kp))

        if prev_des is not None and des is not None and len(des) and len(prev_des):
            t2 = time.perf_counter()
            matches = bf.match(prev_des, des)
            t3 = time.perf_counter()
            match_times.append((t3 - t2) * 1000.0)
            match_counts.append(len(matches))
        prev_des = des
        cpu_seen.add(current_cpu())

    det = float(np.mean(det_times))
    match = float(np.mean(match_times)) if match_times else 0.0
    total = det + match
    peak_rss_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0  # KB->MB on Linux
    return {
        "res": f"{w}x{h}",
        "frames": n_frames,
        "features_req": n_features,
        "kp_avg": float(np.mean(kp_counts)),
        "matches_avg": float(np.mean(match_counts)) if match_counts else 0.0,
        "detect_compute_ms": det,
        "match_ms": match,
        "total_ms": total,
        "fps": 1000.0 / total if total > 0 else 0.0,
        "peak_rss_mb": peak_rss_mb,
        "cpus_used": sorted(cpu_seen),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=120)
    ap.add_argument("--features", type=int, default=500)
    args = ap.parse_args()

    print(f"OpenCV {cv2.__version__} | nproc={os.cpu_count()} | "
          f"affinity={sorted(os.sched_getaffinity(0))}")
    print(f"Frames/res={args.frames}  features={args.features}\n")

    results = []
    for (w, h) in [(320, 240), (640, 480)]:
        r = bench(w, h, args.frames, args.features)
        results.append(r)
        print(f"=== {r['res']} ===")
        print(f"  kp avg            : {r['kp_avg']:.0f}")
        print(f"  matches avg       : {r['matches_avg']:.0f}")
        print(f"  detect+compute ms : {r['detect_compute_ms']:.3f}")
        print(f"  match ms          : {r['match_ms']:.3f}")
        print(f"  total ms/frame    : {r['total_ms']:.3f}")
        print(f"  FPS               : {r['fps']:.2f}")
        print(f"  CPU core(s) used  : {r['cpus_used']}")
        print()
    print(f"peak process RSS (whole run): {results[-1]['peak_rss_mb']:.1f} MB")


if __name__ == "__main__":
    main()
