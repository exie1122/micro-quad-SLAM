# Learned Visual Yaw-Correction Pipeline (dry run)

End-to-end pipeline for a learned relative-yaw estimator on the LicheeRV Nano NPU,
compared against a gyro baseline and classical ORB. **This repo state is a DRY RUN
on public data (TUM RGB-D) with a SIMULATED gyro** — it validates the full chain so
that plugging in real captured data is the only remaining step. See
[RESULTS.md](RESULTS.md).

## The point: data-source abstraction
Everything downstream consumes one interface (`datasets.py`):

    YawDataset.sequences() -> [YawSequence] -> [FramePair(frames, GT rel-yaw, gyro)]

- `TumYawDataset` — working public-dataset adapter (gyro **simulated** from GT).
- `RealRigDataset` — **documented stub**; implement it with your captured frames +
  GT pose + **real** gyro, flip `dataset.adapter` to `real_rig`, and the whole
  pipeline runs unchanged.

## Modules
| File | Stage | Runs on |
|---|---|---|
| `datasets.py` | data-source interface + adapters | host |
| `geometry.py` | quaternion/rotation → yaw helpers | host |
| `data_prep.py` | sync + GT relative yaw + split-by-sequence → manifest | host |
| `model.py`, `train.py` | YawNet train (val, MAE°) → FP32 ONNX | host (GPU) |
| `convert.py` | ONNX → INT8 `.cvimodel` (TPU-MLIR, cv181x) + FP32 preds | host |
| `deploy_eval.py` | push to board, NPU inference over test seq + latency | host→board |
| `compare.py` | gyro vs ORB vs learned: accumulated yaw, drift, plots | host |
| `run_pipeline.py` | one entry point for the whole chain | host |
| `config.yaml` | all knobs (sequences, train, board, intrinsics) | — |

## Run
```bash
pip install torch onnx onnxruntime opencv-python pyyaml matplotlib tpu_mlir
python3 run_pipeline.py                  # prep -> train -> convert -> deploy -> compare
python3 run_pipeline.py --start convert  # resume from a stage
```
Board access (USB/Ethernet gadget, no WiFi): `root@10.43.61.1`, key-based SSH.
Outputs: `results/` (committed plots + metrics), `work/` (heavy intermediates, gitignored).
