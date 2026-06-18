#!/usr/bin/env python3
"""Summarize a CSV produced by the native board live-yaw harness."""
import argparse
import csv
import json
import os

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="yaw_live.csv from the board")
    ap.add_argument("--stationary", action="store_true",
                    help="label this recording as a stationary-camera bias test")
    args = ap.parse_args()

    with open(args.csv, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit("no predictions in CSV")
    yaw = np.array([float(r["relative_yaw_deg"]) for r in rows])
    time = np.array([float(r["timestamp_s"]) for r in rows])
    npu = np.array([float(r["npu_ms"]) for r in rows])
    duration = float(time[-1] - time[0]) if len(time) > 1 else 0.0
    summary = {
        "stationary_test": args.stationary,
        "pairs": len(rows),
        "duration_s": duration,
        "mean_step_deg": float(yaw.mean()),
        "median_step_deg": float(np.median(yaw)),
        "step_std_deg": float(yaw.std()),
        "accumulated_yaw_deg": float(yaw.sum()),
        "apparent_drift_deg_s": float(yaw.sum() / duration) if duration else None,
        "npu_ms_mean": float(npu.mean()),
        "npu_ms_p95": float(np.percentile(npu, 95)),
        "npu_ms_max": float(npu.max()),
    }
    if "stationary_gate" in rows[0]:
        gate = np.array([int(r["stationary_gate"]) for r in rows])
        pixel = np.array([float(r["pixel_mae"]) for r in rows])
        summary["stationary_gate_fraction"] = float(gate.mean())
        summary["pixel_mae_mean"] = float(pixel.mean())
        summary["pixel_mae_p95"] = float(np.percentile(pixel, 95))
    out = os.path.splitext(args.csv)[0] + ".summary.json"
    json.dump(summary, open(out, "w"), indent=2)
    label = "stationary " if args.stationary else ""
    print(f"{label}pairs={len(yaw)} mean={yaw.mean():+.3f}deg std={yaw.std():.3f}deg "
          f"accumulated={yaw.sum():+.1f}deg NPU mean={npu.mean():.3f}ms p95={np.percentile(npu,95):.3f}ms" +
          (f" still={100*summary['stationary_gate_fraction']:.1f}%" if "stationary_gate_fraction" in summary else ""))
    print(f"[analyze_live] wrote {out}")


if __name__ == "__main__":
    main()
