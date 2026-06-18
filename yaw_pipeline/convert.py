"""
Stage 3 - convert.

ONNX -> INT8 .cvimodel via TPU-MLIR (cv181x), calibrating on REAL held-out frames
(not synthetic). Also runs the FP32 ONNX over the test sequence(s) so we can later
report the INT8-vs-FP32 accuracy delta in degrees.

Outputs in <workdir>:
  yaw_net.mlir, yaw_net_cali_table, yaw_net_int8.cvimodel
  fp32_pred_<seq>.npz   (FP32 predicted angles per pair, for the delta)
"""
import os
import json
import argparse
import subprocess
import numpy as np
from config_util import load_config, ensure_workdir
from preprocess import load_pair_float


def _load_pair(p, size, resize_mode):
    return load_pair_float(p, size, resize_mode)


def run(cmd, cwd):
    print("  $", " ".join(cmd))
    r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-2000:]); print(r.stderr[-2000:])
        raise SystemExit(f"[convert] command failed (rc={r.returncode}): {' '.join(cmd)}")
    return r


def make_calib(manifest, workdir, n, size, resize_mode, split):
    """Write calibration tensors from a named split (train by default)."""
    seq_names = manifest["splits"][split]
    pairs = [p for seq in seq_names for p in manifest["sequences"][seq]]
    sel = np.linspace(0, len(pairs) - 1, n).astype(int)
    cdir = os.path.join(workdir, "calib")
    os.makedirs(cdir, exist_ok=True)
    lines = []
    for i, j in enumerate(sel):
        x = _load_pair(pairs[j], size, resize_mode).astype(np.float32)
        fp = os.path.join(cdir, f"calib_{i:03d}.npz")
        np.savez(fp, input=x)
        lines.append(fp)
    listf = os.path.join(workdir, "calib_list.txt")
    open(listf, "w").write("\n".join(lines) + "\n")
    print(f"[convert] {n} real calibration tensors from {split} sequences: {seq_names}")
    return listf


def fp32_predict(manifest, workdir, size, resize_mode):
    import onnxruntime as ort
    sess = ort.InferenceSession(os.path.join(workdir, "yaw_net.onnx"),
                                providers=["CPUExecutionProvider"])
    for seq in manifest["splits"]["test"]:
        pairs = manifest["sequences"][seq]
        angles = []
        for p in pairs:
            out = sess.run(None, {"input": _load_pair(p, size, resize_mode)})[0][0]
            out = out / (np.linalg.norm(out) + 1e-9)
            angles.append(float(np.arctan2(out[0], out[1])))
        np.savez(os.path.join(workdir, f"fp32_pred_{seq}.npz"), angle=np.array(angles))
        print(f"[convert] FP32 predictions for {seq}: {len(angles)} pairs")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    args = ap.parse_args()
    cfg = load_config(args.config)
    workdir = ensure_workdir(cfg)
    manifest = json.load(open(os.path.join(workdir, "manifest.json")))
    size = manifest["img_size"]
    resize_mode = cfg["dataset"].get("resize_mode", "stretch")
    chip = cfg["convert"]["chip"]
    calib_split = cfg["convert"].get("calibration_split", "train")

    listf = make_calib(manifest, workdir, cfg["convert"]["calib_frames"], size,
                       resize_mode, calib_split)

    print("[convert] model_transform ...")
    run(["model_transform.py", "--model_name", "yaw_net",
         "--model_def", "yaw_net.onnx",
         "--input_shapes", f"[[1,2,{size},{size}]]",
         "--mlir", "yaw_net.mlir"], cwd=workdir)

    print("[convert] run_calibration ...")
    run(["run_calibration.py", "yaw_net.mlir",
         "--data_list", os.path.basename(listf),
         "--input_num", str(cfg["convert"]["calib_frames"]),
         "-o", "yaw_net_cali_table"], cwd=workdir)

    print(f"[convert] model_deploy (INT8, {chip}) ...")
    run(["model_deploy.py", "--mlir", "yaw_net.mlir",
         "--quantize", "INT8", "--calibration_table", "yaw_net_cali_table",
         "--chip", chip, "--model", "yaw_net_int8.cvimodel"], cwd=workdir)

    fp32_predict(manifest, workdir, size, resize_mode)
    print(f"[convert] DONE -> {os.path.join(workdir, 'yaw_net_int8.cvimodel')}")


if __name__ == "__main__":
    main()
