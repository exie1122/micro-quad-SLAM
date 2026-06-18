# Accuracy experiments

The committed baseline remains `config.yaml`. Two isolated variants were trained
and evaluated to avoid replacing a known deployment with a model that merely has
a better validation score.

| Configuration | Validation MAE | Held-out `fr1_360` MAE | Correlation | FP32 final drift |
|---|---:|---:|---:|---:|
| Original baseline | 1.76° | 2.35° | 0.183 | −85.6° |
| 160px center-crop + balance + augmentation | 1.38° | 2.52° | 0.212 | −368.3° |
| 128px + zero-motion/photometric augmentation | 1.21° | 2.07° | 0.303 | −253.5° |

Neither experimental model is a deployment candidate: both improve local
validation or per-pair metrics while severely worsening accumulated drift. This
shows that per-pair MAE alone is not an adequate selection metric.

The native CSI test also found a large real-camera stationary offset in the
original model: +3.37° per pair over the first 30-pair run. Startup visual-offset
calibration plus a pixel-motion gate reduced a later 50-pair nominally stationary
run to −3.39° total.

## First handheld moving-camera test

After 20 stationary calibration pairs, the camera was turned approximately 90°
by hand and returned to its starting orientation. The model reached −50.36° at
the direction reversal and finished at −26.56° rather than approximately 0°.
It therefore detected the reversal, but underestimated motion and retained large
round-trip drift. Mean direct NPU latency was 4.17 ms (8.09 ms p95). The runtime
target is met; real-camera accuracy is not deployment-ready.
