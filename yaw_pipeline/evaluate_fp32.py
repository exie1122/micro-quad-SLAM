#!/usr/bin/env python3
"""Evaluate the exported FP32 model without requiring the SG2002 board."""
import argparse
import json
import os

import numpy as np
import onnxruntime as ort

from config_util import ensure_workdir, load_config
from preprocess import load_pair_float


def wrapped_error(pred, gt):
    return np.arctan2(np.sin(pred - gt), np.cos(pred - gt))


def metrics(pred, gt):
    err = np.degrees(wrapped_error(pred, gt))
    if np.std(pred) > 1e-12 and np.std(gt) > 1e-12:
        corr = float(np.corrcoef(pred, gt)[0, 1])
        slope = float(np.polyfit(gt, pred, 1)[0])
    else:
        corr, slope = None, None
    return {
        "pairs": len(pred),
        "mae_deg": float(np.mean(np.abs(err))),
        "rmse_deg": float(np.sqrt(np.mean(err ** 2))),
        "bias_deg": float(np.mean(err)),
        "corr": corr,
        "slope": slope,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    args = ap.parse_args()
    cfg = load_config(args.config)
    workdir = ensure_workdir(cfg)
    manifest = json.load(open(os.path.join(workdir, "manifest.json")))
    size = manifest["img_size"]
    mode = cfg["dataset"].get("resize_mode", "stretch")
    session = ort.InferenceSession(os.path.join(workdir, "yaw_net.onnx"),
                                   providers=["CPUExecutionProvider"])

    report = {"img_size": size, "resize_mode": mode, "sequences": {}}
    for name, pairs in manifest["sequences"].items():
        pred = []
        for p in pairs:
            out = session.run(None, {"input": load_pair_float(p, size, mode)})[0].reshape(-1)
            pred.append(float(np.arctan2(out[0], out[1])))
        pred = np.asarray(pred)
        gt = np.asarray([p["rel_yaw"] for p in pairs])
        result = metrics(pred, gt)

        if name in manifest["splits"]["test"]:
            sel = np.arange(0, len(pairs), manifest["frame_stride"])
            result["nonoverlap_pairs"] = int(len(sel))
            result["gt_total_yaw_deg"] = float(np.degrees(gt[sel].sum()))
            result["final_drift_deg"] = float(np.degrees(pred[sel].sum() - gt[sel].sum()))
        report["sequences"][name] = result
        print(f"{name}: MAE={result['mae_deg']:.3f}deg corr={result['corr']:.3f} "
              f"slope={result['slope']:.3f}" +
              (f" drift={result['final_drift_deg']:.1f}deg" if "final_drift_deg" in result else ""))

    out = os.path.join(workdir, "fp32_eval.json")
    json.dump(report, open(out, "w"), indent=2)
    print(f"[evaluate_fp32] wrote {out}")


if __name__ == "__main__":
    main()
