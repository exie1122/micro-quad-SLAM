"""
Stage 4 - deploy + on-board NPU eval.

Pushes the INT8 .cvimodel to the board, runs it over the held-out test sequence on
the NPU via model_runner (real INT8 inference), captures predictions + latency, and
pulls results back.

Outputs in <workdir>:
  int8_pred_<seq>.npz   (INT8/NPU predicted angles per pair)
  npu_latency.json      (pure NPU inference latency + FPS)
"""
import os
import json
import argparse
import subprocess
import tarfile
import numpy as np
import cv2

from config_util import load_config, ensure_workdir

SSH_OPTS = ["-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=no",
           "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=12"]

# Board-side script: reconstructs the fp32 input from uint8, loops model_runner
# over each input, collects 2-float outputs, and times pure NPU latency.
BOARD_SCRIPT = r'''
import os, sys, glob, subprocess, numpy as np
d = sys.argv[1]
ins = sorted(glob.glob(os.path.join(d, "inputs", "*.npz")))
tmp = os.path.join(d, "in.npz")
out_vals = []
for f in ins:
    u8 = np.load(f)["u8"]                       # [2,128,128] uint8
    x = (u8.astype(np.float32) / 255.0)[None]   # [1,2,128,128]
    np.savez(tmp, input=x)
    of = os.path.join(d, "out.npz")
    subprocess.run(["model_runner", "--model", os.path.join(d, "yaw_net_int8.cvimodel"),
                    "--input", tmp, "--output", of], cwd=d,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
    z = np.load(of)
    v = z[z.files[0]].reshape(-1)[:2]
    out_vals.append([float(v[0]), float(v[1])])
np.savez(os.path.join(d, "int8_out.npz"), sincos=np.array(out_vals))
# pure NPU latency on a single input
np.savez(tmp, input=(np.load(ins[0])["u8"].astype(np.float32) / 255.0)[None])
r = subprocess.run(["model_runner", "--model", os.path.join(d, "yaw_net_int8.cvimodel"),
                    "--input", tmp, "--count", "2000", "--enable-timer"],
                   cwd=d, capture_output=True, text=True)
open(os.path.join(d, "latency.txt"), "w").write(r.stdout + r.stderr)
print("BOARD_EVAL_DONE", len(out_vals))
'''


def _load_pair_u8(p, size):
    def g(path):
        im = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if im.shape != (size, size):
            im = cv2.resize(im, (size, size), interpolation=cv2.INTER_AREA)
        return im
    return np.stack([g(p["img_a"]), g(p["img_b"])], 0).astype(np.uint8)  # [2,H,W]


def sh(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-1500:]); print(r.stderr[-1500:])
        raise SystemExit(f"[deploy] failed: {' '.join(cmd)}")
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    args = ap.parse_args()
    cfg = load_config(args.config)
    workdir = ensure_workdir(cfg)
    manifest = json.load(open(os.path.join(workdir, "manifest.json")))
    size = manifest["img_size"]
    host = cfg["board"]["host"]
    rdir = cfg["board"]["remote_dir"]
    cap = cfg["board"]["eval_max_pairs"]

    test_seq = manifest["splits"]["test"][0]
    pairs = manifest["sequences"][test_seq][:cap]
    print(f"[deploy] preparing {len(pairs)} NPU inputs for held-out '{test_seq}'")

    indir = os.path.join(workdir, "eval_inputs")
    os.makedirs(indir, exist_ok=True)
    for f in os.listdir(indir):
        os.remove(os.path.join(indir, f))
    for i, p in enumerate(pairs):
        np.savez_compressed(os.path.join(indir, f"in_{i:04d}.npz"), u8=_load_pair_u8(p, size))

    # bundle inputs + model + board script (gzip for the slow USB link)
    tarpath = os.path.join(workdir, "eval_bundle.tar")
    with tarfile.open(tarpath, "w") as t:
        t.add(indir, arcname="inputs")
        t.add(os.path.join(workdir, "yaw_net_int8.cvimodel"), arcname="yaw_net_int8.cvimodel")
    bscript = os.path.join(workdir, "board_eval.py")
    open(bscript, "w").write(BOARD_SCRIPT)

    print("[deploy] copying to board ...")
    sh(["ssh", *SSH_OPTS, host, f"rm -rf {rdir}; mkdir -p {rdir}"])
    sh(["scp", *SSH_OPTS, tarpath, f"{host}:{rdir}/eval_bundle.tar"])
    sh(["scp", *SSH_OPTS, bscript, f"{host}:{rdir}/board_eval.py"])

    print("[deploy] running NPU eval on board (this loops model_runner) ...")
    r = sh(["ssh", *SSH_OPTS, host,
            f"cd {rdir} && tar -xf eval_bundle.tar && python3 board_eval.py {rdir}"])
    print("  " + r.stdout.strip().splitlines()[-1])

    print("[deploy] pulling results ...")
    sh(["scp", *SSH_OPTS, f"{host}:{rdir}/int8_out.npz", os.path.join(workdir, "int8_out.npz")])
    sh(["scp", *SSH_OPTS, f"{host}:{rdir}/latency.txt", os.path.join(workdir, "latency.txt")])

    sincos = np.load(os.path.join(workdir, "int8_out.npz"))["sincos"]
    sincos = sincos / (np.linalg.norm(sincos, axis=1, keepdims=True) + 1e-9)
    angles = np.arctan2(sincos[:, 0], sincos[:, 1])
    np.savez(os.path.join(workdir, f"int8_pred_{test_seq}.npz"), angle=angles)

    import re
    lat = open(os.path.join(workdir, "latency.txt")).read()
    ms = fps = None
    m = re.search(r"each run takes ([\d.]+) ms.*?fps ([\d.]+)", lat)
    if m:
        ms, fps = float(m.group(1)), float(m.group(2))
    json.dump({"npu_ms_per_inf": ms, "npu_fps": fps, "eval_pairs": len(pairs)},
              open(os.path.join(workdir, "npu_latency.json"), "w"))
    print(f"[deploy] NPU latency: {ms} ms/inf  ({fps} FPS) over {len(pairs)} real eval pairs")
    print(f"[deploy] DONE -> int8_pred_{test_seq}.npz")


if __name__ == "__main__":
    main()
