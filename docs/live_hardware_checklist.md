# Live hardware checklist

## DO NOT PROCEED TO REAL FLIGHT IF:

- the deployment validator fails or the exact serial devices/permissions differ.
- the ESP32 protocol cannot be confirmed as the exact `legacy-a5-v0` A5 frame.
- Any test in `make test`, `make sanitize`, or `make riscv` fails.
- A target startup service still references `clean_uav_fc_tof_nav`, `frontier`,
  `uav_local_nav`, or another unguarded executable.
- Props are fitted for the checks below.

## Props-off integration order

1. Record current Git commit, compiler version, executable/config hashes, target
   hostname, FC firmware/parameters, wiring, UART device, and baud rates.
2. Build `make riscv`; copy only the new static binary and a reviewed config. Verify
   SHA-256 after transfer. Do not deploy a legacy source binary.
3. Audit all systemd/init/cron/RC scripts on target. The repository contained none,
   so target state is unknown. Configure exploration disabled.
4. With FC power but props removed, prove that default launch and replay open no
   serial device and emit no MAVLink command.
5. Enable live serial explicitly and verify expected H743 sysid/compid, heartbeat
   age, GUIDED availability, pre-arm status, observed mode, local NED frame, range,
   optical-flow quality, battery, and landed state. Do not arm.
6. Connect the live ToF mapper; inject unplug/stale/malformed/version/dimension/
   resolution/frame faults and prove horizontal movement remains prohibited.
7. Exercise mode request/ACK/observation without arming. Exercise rejected ACK,
   timeout, wrong-source ACK, link loss, and manual stop.
8. Only after an independent operator and hard kill method are present, perform an
   ArduPilot-approved props-off arm/disarm test. Confirm disarm is never emitted
   while landed state is unknown or in-air.
9. Use SITL for takeoff/landing sequencing. Do not send a physical takeoff command
   during this checklist pass.
10. Review the run manifest and decision log for every gate, clamp, transition,
    ACK, observed state, and failure response.

## Deployment commands

```text
/opt/micro-quad-autonomy/bin/autonomy_controller --backend mavlink \
  --allow-live-serial --serial /dev/ttyS2 --baud 57600 \
  --expected-system 1 --expected-component 1 \
  --tof-serial /dev/ttyS1 --tof-baud 115200 \
  --tof-protocol legacy-a5-v0 --map-source live
```

This exact command is monitor-only because it omits `--start-mission`; startup can
never arm. Only an attended props-off procedure may add `--start-mission`, and only
after that may `--explore` be considered. The installed systemd service never adds
`--start-mission` under any configuration.
