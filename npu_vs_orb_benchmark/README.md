# NPU vs ORB-on-CPU feasibility benchmark — LicheeRV Nano (SG2002)

Two benchmarks to decide whether a small vision CNN on the SG2002 NPU beats classical ORB on
the C906 CPU. **Result + go/no-go: [RESULTS.md](RESULTS.md). (Spoiler: GO — NPU ~200× faster.)**

## Files
| File | Role | Runs on |
|---|---|---|
| `orb_cpu_benchmark.py` | ORB detect+compute+match timing, 320×240 & 640×480 | board (RISC-V) |
| `tiny_cnn_export.py` | builds tiny CNN, exports ONNX, makes calibration frames | x86 host |
| `convert_tpu_mlir.sh` | ONNX → INT8 `.cvimodel` (TPU-MLIR, chip cv181x) | x86 host |
| `run_npu_benchmark.sh` | deploy + NPU timing + RAM + NPU-execution check | x86 host → board |
| `tiny_yaw_cnn.onnx`, `tiny_yaw_cnn_int8.cvimodel` | the model artifacts | — |
| `calib_images/` | 60 synthetic calibration frames | — |

## Host setup (x86, for the CNN half)
```bash
pip install torch onnx onnxruntime opencv-python   # model export
pip install tpu_mlir                                # SOPHGO toolchain (no Docker needed)
```
The board side needs nothing extra — OpenCV 4.9, Python 3.11 and the CVITEK `model_runner`
are already installed.

## Board access
No onboard WiFi. SSH over the USB/Ethernet gadget: `root@10.43.61.1` (pw `root`).
The scripts assume key-based or passwordless SSH; install your pubkey first if needed.

See [RESULTS.md](RESULTS.md) for the reproduce commands and full numbers.
