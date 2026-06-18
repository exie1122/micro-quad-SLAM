"""
Stage 1 - data_prep.

Builds the dataset adapter, derives per-frame-pair ground-truth relative yaw,
splits BY SEQUENCE (whole sequences held out for test -> no frame-level leakage),
and writes a clean labeled manifest that downstream stages consume.

Output: <workdir>/manifest.json
"""
import os
import json
import argparse

from config_util import load_config, ensure_workdir
from datasets import TumYawDataset, RealRigDataset


def build_dataset(cfg):
    d = cfg["dataset"]
    if d["adapter"] == "tum":
        seq_names = d["sequences"]["train"] + d["sequences"]["test"]
        seq_dirs = {n: os.path.join(d["cache_dir"], n) for n in seq_names}
        missing = [p for p in seq_dirs.values() if not os.path.isdir(p)]
        if missing:
            raise FileNotFoundError(
                "Missing TUM sequence dirs (run the download step first):\n  "
                + "\n  ".join(missing))
        return TumYawDataset(
            seq_dirs=seq_dirs, frame_stride=d["frame_stride"], img_size=d["img_size"],
            gyro_bias_deg_s=d["gyro_bias_deg_s"], gyro_noise_deg_s=d["gyro_noise_deg_s"],
            seed=cfg["train"]["seed"])
    elif d["adapter"] == "real_rig":
        return RealRigDataset(root=d.get("cache_dir", ""))
    raise ValueError(f"unknown adapter {d['adapter']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    args = ap.parse_args()
    cfg = load_config(args.config)
    workdir = ensure_workdir(cfg)

    ds = build_dataset(cfg)
    seqs = {s.name: s for s in ds.sequences()}

    train_names = cfg["dataset"]["sequences"]["train"]
    test_names = cfg["dataset"]["sequences"]["test"]

    manifest = {
        "dataset": ds.name,
        "gyro_is_simulated": ds.gyro_is_simulated,
        "img_size": cfg["dataset"]["img_size"],
        "frame_stride": cfg["dataset"]["frame_stride"],
        "run_label": cfg["run_label"],
        "splits": {"train": train_names, "test": test_names},
        "sequences": {},
    }
    for name, seq in seqs.items():
        manifest["sequences"][name] = [{
            "idx": p.idx, "t_a": p.t_a, "t_b": p.t_b,
            "rel_yaw": p.rel_yaw, "gyro_yaw_rate": p.gyro_yaw_rate,
            "img_a": p.img_a, "img_b": p.img_b,
        } for p in seq]

    out = os.path.join(workdir, "manifest.json")
    with open(out, "w") as f:
        json.dump(manifest, f)

    print(f"[data_prep] adapter={ds.name}  gyro_simulated={ds.gyro_is_simulated}")
    for name in train_names + test_names:
        split = "train" if name in train_names else "TEST(held-out)"
        print(f"  {split:16s} {name}: {len(seqs[name])} pairs")
    print(f"[data_prep] wrote {out}")


if __name__ == "__main__":
    main()
