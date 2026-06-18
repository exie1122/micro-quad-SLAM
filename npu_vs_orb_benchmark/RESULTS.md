# NPU vs. ORB-on-CPU — Feasibility Benchmark

**Board:** Sipeed LicheeRV Nano · SOPHGO SG2002 (cv181x family)
**Date:** 2026-06-15
**Question:** Can the SG2002 NPU run a small vision CNN meaningfully faster than classical
ORB runs on the C906 CPU? **→ Yes, by ~2 orders of magnitude. GO.**

---

## Environment (probed on-device)

| | |
|---|---|
| OS / kernel | Buildroot 2023.11.2, Linux 5.10.4, riscv64 |
| CPU | **Single** C906 core @ ~1 GHz (`rv64imafdvc`, vector ext) |
| RAM | **128 MB visible to Linux** (~87 MB free; remainder reserved for NPU/VPU). Not the 256 MB on the box. |
| OpenCV | 4.9.0 (Python) |
| NPU | `/dev/cvi-tpu0`, CVITEK runtime 1.4.0, `model_runner` |
| Toolchain | TPU-MLIR v1.14 (run on x86 host via `pip install tpu_mlir`; **no Docker needed**) |

---

## Headline numbers

| Workload | Resolution | ms / frame | **FPS** | Peak RAM | Core |
|---|---|---|---|---|---|
| **ORB (CPU)** — detect+compute+match | 320×240 | 331.7 | **3.01** | 48 MB | C906 (core 0) |
| **ORB (CPU)** — detect+compute+match | 640×480 | 740.7 | **1.35** | 48 MB | C906 (core 0) |
| **Tiny CNN (NPU)** — INT8, 1×1×128×128 | 128×128 | 1.53 | **654** | ~2.7 MB¹ | NPU (`/dev/cvi-tpu0`) |

¹ Process RSS only. Model weights/tensors live in the NPU's separate reserved memory pool,
so they don't show up in Linux RSS.

**Speedup (NPU CNN vs. ORB full pipeline):** ~217× over 320×240, ~485× over 640×480.

---

## ORB on CPU — per-stage detail

Averaged over 150 frames, ~500 features, grayscale, `BFMatcher(NORM_HAMMING, crossCheck=True)`
between consecutive frames. Synthetic warped frames (pan + small rotation).

| Stage | 320×240 | 640×480 |
|---|---|---|
| keypoints (avg) | 468 | 500 |
| matches (avg) | 337 | 348 |
| detect + compute | 105.0 ms | 482.9 ms |
| descriptor match (Hamming) | 226.7 ms | 257.8 ms |
| **total / frame** | **331.7 ms** | **740.7 ms** |
| **FPS** | **3.01** | **1.35** |

Single-core bound (`nproc = 1`, affinity `[0]`). Matching is expensive here because the C906
lacks a vectorized popcount, so brute-force Hamming over ~470×470 descriptors is costly.

## Tiny CNN on NPU — detail

Architecture (stand-in for a yaw/rotation regressor): 4× `Conv(3×3, stride 2)+ReLU`
(1→8→16→32→32 ch) → `GlobalAveragePool` → `FC(32→3)`. **15,235 params.** Input `1×1×128×128`
grayscale, output 3 floats. Exported to ONNX (opset 13), quantized to INT8 with 60 calibration
frames via TPU-MLIR, deployed as a 21 KB `.cvimodel`.

| Metric | Value |
|---|---|
| Latency / inference (8000-run sustained) | **1.53 ms** |
| Latency / inference (200-run) | 1.41 ms |
| **FPS** | **654** (200-run: 710) |
| Peak process RAM | ~2.7 MB |
| Executes on NPU? | **Yes** — `/dev/cvi-tpu0` held open by the process; 654 FPS for 4 conv layers over 128×128 is physically impossible on the 1 GHz core (ORB detect alone is 105 ms). |

---

## Recommendation — **GO**

The NPU path is overwhelmingly faster than ORB-on-CPU and uses far less RAM (~2.7 MB vs 48 MB),
which matters on this 128 MB-visible board. For a real-time yaw/rotation estimator the NPU is
clearly the right engine.

**Honest caveats (for the writeup):**
- These are **different workloads**, not the same algorithm on two engines: ORB is a full
  feature-detector + descriptor + matcher at 320/640; the CNN is a 128×128 regressor. The
  comparison answers the *feasibility* question ("is a CNN-on-NPU approach viable and fast?"),
  not "NPU is N× faster at the identical task."
- The CNN weights are **untrained** (random init). This is a **latency/feasibility** benchmark;
  the architecture and I/O shapes are representative, but accuracy was not evaluated.
- The CNN runs at 128×128; ORB at 320/480. A like-for-like comparison would feed both the same
  resolution, but even the most favorable ORB case (320×240, 3 FPS) is ~200× slower than the NPU.
- Even if a beta unit caps at ~0.5 TOPS (half the 1 TOPS assumed), the NPU would still be
  ~100×+ faster than ORB — the conclusion is robust.

## Reproduce

```bash
# 1. ORB CPU benchmark (on the board)
scp orb_cpu_benchmark.py root@10.43.61.1:/root/
ssh root@10.43.61.1 'python3 /root/orb_cpu_benchmark.py --frames 150 --features 500'

# 2. Build + export the tiny CNN (x86 host)
python3 tiny_cnn_export.py

# 3. Convert ONNX -> INT8 .cvimodel (x86 host; needs `pip install tpu_mlir`)
./convert_tpu_mlir.sh

# 4. Deploy + benchmark on the NPU (drives the board over SSH)
./run_npu_benchmark.sh root@10.43.61.1 8000
```
