# Native SG2002 live yaw harness

`yaw_live` captures the GC4653 CSI camera through the CVI middleware, samples
non-overlapping frame pairs, runs YawNet directly through `cviruntime`, and logs
relative plus accumulated yaw to CSV. It does not require V4L2 or Python.

Build in the existing WSL SDK environment:

```bash
export COMPILER=$HOME/host-tools/gcc/riscv64-linux-musl-x86_64/bin
export SDK_PATH=$HOME/cvitek-tdl-sdk-sg200x
cmake -S /mnt/c/Users/Ethan/yaw_pipeline/board -B /tmp/yaw_live_build
cmake --build /tmp/yaw_live_build -j$(nproc)
```

Copy `yaw_live` and the matching `.cvimodel` to the board, then run:

```bash
./yaw_live yaw_net_int8.cvimodel --size 128 --stride 4 --resize stretch \
  --calibrate 20 --stationary-pixel 4.0 --pairs 100 --log yaw_live.csv \
  --save-dir yaw_samples --save-every 10
```

Use `--resize stretch` with the original model and `center_crop` with the
improved configuration. Press Ctrl-C to stop a continuous run. Accumulated yaw
is meaningful because the harness predicts only non-overlapping pairs. Keep the
camera stationary during the startup calibration; this estimates model offset,
not gyro bias, and requires no IMU. The stationary gate reports `STILL` and
suppresses accumulated drift while the mean pixel change is below its threshold.
Lower `--stationary-pixel` if genuine slow motion is being suppressed.
