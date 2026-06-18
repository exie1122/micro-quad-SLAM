"""
Stage 5 - comparison harness (the scientific core).

Over the held-out TEST sequence, compute ACCUMULATED yaw over time for three
methods and compare each to ground truth:

  - gyro-only : integrate gyro yaw-rate (SIMULATED here: GT rate + bias + noise)
  - ORB (CPU) : inter-frame rotation from ORB matches (homography K R K^-1) -> integrate
  - learned   : INT8/NPU model's inter-frame yaw -> integrate

Also reports the INT8-vs-FP32 accuracy delta. Produces yaw(t) and error-vs-time
plots and a summary table; writes everything needed for RESULTS.md.
"""
import os
import json
import argparse
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from config_util import load_config, ensure_workdir
from geometry import matrix_to_rotvec, VERTICAL_AXIS


def orb_relative_yaw(img_a_path, img_b_path, orb, bf, K, min_matches):
    """Inter-frame yaw (rad) from ORB matches via homography. None if too few matches."""
    a = cv2.imread(img_a_path, cv2.IMREAD_GRAYSCALE)
    b = cv2.imread(img_b_path, cv2.IMREAD_GRAYSCALE)
    ka, da = orb.detectAndCompute(a, None)
    kb, db = orb.detectAndCompute(b, None)
    if da is None or db is None or len(ka) < min_matches or len(kb) < min_matches:
        return None
    matches = bf.match(da, db)
    if len(matches) < min_matches:
        return None
    pa = np.float32([ka[m.queryIdx].pt for m in matches])
    pb = np.float32([kb[m.trainIdx].pt for m in matches])
    H, mask = cv2.findHomography(pa, pb, cv2.RANSAC, 3.0)
    if H is None:
        return None
    # pure-rotation model: H ~ K R K^-1  ->  R ~ K^-1 H K, re-orthonormalized
    M = np.linalg.inv(K) @ H @ K
    U, S, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        R = U @ np.diag([1, 1, -1]) @ Vt
    return float(matrix_to_rotvec(R)[VERTICAL_AXIS])


def accumulate(per_pair, dt=None, rate=False):
    """Cumulative yaw(t); if rate=True, per_pair is a rate to multiply by dt."""
    step = per_pair * dt if rate else per_pair
    return np.cumsum(step)


def metrics(acc, gt_acc):
    err = acc - gt_acc
    return {
        "final_drift_deg": float(np.degrees(err[-1])),
        "rms_error_deg": float(np.degrees(np.sqrt(np.mean(err ** 2)))),
        "max_abs_error_deg": float(np.degrees(np.max(np.abs(err)))),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    args = ap.parse_args()
    cfg = load_config(args.config)
    workdir = ensure_workdir(cfg)
    manifest = json.load(open(os.path.join(workdir, "manifest.json")))
    cc = cfg["compare"]

    test_seq = manifest["splits"]["test"][0]
    pairs = manifest["sequences"][test_seq]

    # align length to NPU eval (it may be capped)
    int8 = np.load(os.path.join(workdir, f"int8_pred_{test_seq}.npz"))["angle"]
    fp32 = np.load(os.path.join(workdir, f"fp32_pred_{test_seq}.npz"))["angle"]
    n = min(len(pairs), len(int8), len(fp32))
    pairs, int8, fp32 = pairs[:n], int8[:n], fp32[:n]

    # IMPORTANT: data_prep emits a pair at EVERY frame index (gap = stride), so
    # consecutive pairs OVERLAP. For temporal integration we must step through
    # NON-OVERLAPPING pairs (step = stride) or each interval is counted `stride`
    # times. (Overlapping pairs are still fine as independent training samples.)
    stride = manifest["frame_stride"]
    sel = np.arange(0, n, stride)
    pairs = [pairs[i] for i in sel]
    int8, fp32 = int8[sel], fp32[sel]
    n = len(sel)

    dt = np.array([p["t_b"] - p["t_a"] for p in pairs])
    t = np.cumsum(dt)
    gt = np.array([p["rel_yaw"] for p in pairs])
    gyro_rate = np.array([p["gyro_yaw_rate"] for p in pairs])

    # ORB arm (full-res frames, real intrinsics)
    fx, fy, cx, cy = cc["intrinsics"]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    orb = cv2.ORB_create(nfeatures=cc["orb_nfeatures"])
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    orb_yaw = np.zeros(n)
    orb_fail = 0
    for i, p in enumerate(pairs):
        y = orb_relative_yaw(p["img_a"], p["img_b"], orb, bf, K, cc["min_matches"])
        if y is None:
            orb_yaw[i] = 0.0
            orb_fail += 1
        else:
            orb_yaw[i] = y
    # Coordinate handedness is configured once; never infer it from test GT.
    orb_sign = float(cc.get("orb_yaw_sign", 1.0))
    orb_yaw *= orb_sign

    # accumulate
    acc_gt = accumulate(gt)
    acc_gyro = accumulate(gyro_rate, dt, rate=True)
    acc_orb = accumulate(orb_yaw)
    acc_learn = accumulate(int8)
    acc_fp32 = accumulate(fp32)

    res = {
        "gyro (simulated)": metrics(acc_gyro, acc_gt),
        "ORB (CPU)": metrics(acc_orb, acc_gt),
        "learned INT8 (NPU)": metrics(acc_learn, acc_gt),
    }
    # INT8 vs FP32 accuracy delta
    dang = np.degrees(np.arctan2(np.sin(int8 - fp32), np.cos(int8 - fp32)))
    int8_fp32 = {
        "per_pair_mae_deg": float(np.mean(np.abs(dang))),
        "fp32_final_drift_deg": float(np.degrees(acc_fp32[-1] - acc_gt[-1])),
        "int8_final_drift_deg": float(np.degrees(acc_learn[-1] - acc_gt[-1])),
    }
    int8_fp32["delta_final_drift_deg"] = abs(
        int8_fp32["int8_final_drift_deg"] - int8_fp32["fp32_final_drift_deg"])

    # ---- plots ----
    td = t
    plt.figure(figsize=(9, 5))
    plt.plot(td, np.degrees(acc_gt), "k-", lw=2.5, label="ground truth")
    plt.plot(td, np.degrees(acc_gyro), label="gyro-only (simulated)")
    plt.plot(td, np.degrees(acc_orb), label="ORB (CPU)")
    plt.plot(td, np.degrees(acc_learn), label="learned INT8 (NPU)")
    plt.xlabel("time (s)"); plt.ylabel("accumulated yaw (deg)")
    plt.title(f"Accumulated yaw vs time - {test_seq}\n[DRY RUN: TUM public data, gyro SIMULATED]")
    plt.legend(); plt.grid(alpha=0.3); plt.tight_layout()
    p1 = os.path.join(workdir, "yaw_accumulated.png"); plt.savefig(p1, dpi=130); plt.close()

    plt.figure(figsize=(9, 5))
    plt.axhline(0, color="k", lw=1)
    plt.plot(td, np.degrees(acc_gyro - acc_gt), label="gyro-only (simulated)")
    plt.plot(td, np.degrees(acc_orb - acc_gt), label="ORB (CPU)")
    plt.plot(td, np.degrees(acc_learn - acc_gt), label="learned INT8 (NPU)")
    plt.xlabel("time (s)"); plt.ylabel("yaw error vs GT (deg)")
    plt.title(f"Yaw drift/error vs time - {test_seq}\n[DRY RUN: TUM public data, gyro SIMULATED]")
    plt.legend(); plt.grid(alpha=0.3); plt.tight_layout()
    p2 = os.path.join(workdir, "yaw_error.png"); plt.savefig(p2, dpi=130); plt.close()

    summary = {
        "test_sequence": test_seq,
        "n_pairs": int(n),
        "orb_failed_pairs": int(orb_fail),
        "orb_yaw_sign": orb_sign,
        "gt_total_yaw_deg": float(np.degrees(acc_gt[-1])),
        "methods": res,
        "int8_vs_fp32": int8_fp32,
        "plots": [os.path.basename(p1), os.path.basename(p2)],
    }
    json.dump(summary, open(os.path.join(workdir, "compare_summary.json"), "w"), indent=2)

    print(f"[compare] test={test_seq}  n={n}  GT total yaw={summary['gt_total_yaw_deg']:.1f} deg  "
          f"(ORB failed {orb_fail} pairs)")
    print(f"{'method':22s} {'final drift':>12s} {'RMS err':>10s} {'max err':>10s}  (deg)")
    for k, m in res.items():
        print(f"{k:22s} {m['final_drift_deg']:12.1f} {m['rms_error_deg']:10.1f} {m['max_abs_error_deg']:10.1f}")
    print(f"[compare] INT8 vs FP32: per-pair MAE={int8_fp32['per_pair_mae_deg']:.3f} deg, "
          f"final-drift delta={int8_fp32['delta_final_drift_deg']:.2f} deg")
    print(f"[compare] plots -> {p1}, {p2}")


if __name__ == "__main__":
    main()
