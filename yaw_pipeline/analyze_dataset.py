#!/usr/bin/env python3
"""Report yaw-label coverage before spending time retraining."""
import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from config_util import ensure_workdir, load_config


def sequence_stats(pairs):
    deg = np.degrees([p["rel_yaw"] for p in pairs])
    dt = np.array([p["t_b"] - p["t_a"] for p in pairs])
    absolute = np.abs(deg)
    return {
        "pairs": len(pairs),
        "yaw_deg_min": float(deg.min()),
        "yaw_deg_max": float(deg.max()),
        "yaw_deg_mean": float(deg.mean()),
        "abs_yaw_deg_percentiles": {
            str(p): float(v) for p, v in zip(
                [25, 50, 75, 90, 95, 99], np.percentile(absolute, [25, 50, 75, 90, 95, 99]))
        },
        "positive_fraction": float(np.mean(deg > 0)),
        "near_zero_fraction_lt_0_25_deg": float(np.mean(absolute < 0.25)),
        "dt_ms_mean": float(1000.0 * dt.mean()),
        "dt_ms_std": float(1000.0 * dt.std()),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    args = ap.parse_args()
    cfg = load_config(args.config)
    workdir = ensure_workdir(cfg)
    manifest = json.load(open(os.path.join(workdir, "manifest.json")))

    stats = {
        "img_size": manifest["img_size"],
        "frame_stride": manifest["frame_stride"],
        "resize_mode": cfg["dataset"].get("resize_mode", "stretch"),
        "sequences": {name: sequence_stats(pairs)
                      for name, pairs in manifest["sequences"].items()},
    }

    plt.figure(figsize=(9, 5))
    for name, pairs in manifest["sequences"].items():
        deg = np.degrees([p["rel_yaw"] for p in pairs])
        plt.hist(deg, bins=np.linspace(-20, 20, 81), histtype="step", linewidth=1.5,
                 label=name.replace("rgbd_dataset_", ""))
    plt.xlabel("ground-truth relative yaw per pair (deg)")
    plt.ylabel("count")
    plt.title("Yaw-label coverage")
    plt.grid(alpha=0.2)
    plt.legend()
    plt.tight_layout()
    plot_path = os.path.join(workdir, "yaw_distribution.png")
    plt.savefig(plot_path, dpi=130)
    plt.close()
    stats["plot"] = os.path.basename(plot_path)

    out = os.path.join(workdir, "dataset_diagnostics.json")
    json.dump(stats, open(out, "w"), indent=2)
    for name, s in stats["sequences"].items():
        p = s["abs_yaw_deg_percentiles"]
        print(f"{name}: n={s['pairs']} median|yaw|={p['50']:.2f}deg "
              f"p95={p['95']:.2f}deg positive={100*s['positive_fraction']:.1f}%")
    print(f"[analyze] wrote {out} and {plot_path}")


if __name__ == "__main__":
    main()
