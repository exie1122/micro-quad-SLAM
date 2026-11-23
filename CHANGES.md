# Code fixes after the year-1 post-mortem

The as-flown `frontier.c` is the previous commit in this repository's history. The reasoning and flight-log evidence behind each fix
are in `analysis/REPORT.md`.

## frontier.c

### Link / loop (crashes, drift)
- **Fixed-rate control loop.** `control_tick()` now runs every 40 ms (25 Hz). Before, it ran after
  every `poll()` wakeup, i.e. hundreds of times per second.
- **One setpoint per tick, never written as a fragment.** Setpoint senders call `queue_setpoint()`;
  `flush_setpoint()` writes the last one at the end of the tick. If the kernel TX queue holds more
  than 256 B, the setpoint is dropped whole. Commands (`mav_send`) are always written whole, with a
  200 ms hard cap so a stuck UART can't freeze the loop. A `LINK:` log line reports drops.
- `--fc-baud N` (57600 … 921600). An unsupported baud is now an error instead of silently using
  57600. A warning is printed below 230400.
- Streams: duplicate OPTICAL_FLOW_RAD and RANGEFINDER requests removed (the logs show they were never
  the data source), the other rates trimmed, and streams re-requested if any essential one goes stale
  (EKF_STATUS_REPORT never arrived before).
- DISARM was sent on every tick while armed; it's now sent immediately, then every 250 ms. LAND is
  re-requested until the heartbeat confirms it; before, the 800 ms mode rate-limit could silently
  swallow it for 2 s. No more GUIDED setpoints once in LAND mode.

### Setpoint type_masks (the controller-reset bug)
- Only the four mask shapes ArduCopter accepts are used (`PT_MASK_*`, documented in the code).
  - `send_prelock_hold`: position+velocity, with x,y = current position and z = target.
  - `send_z_yaw_ned`: velocity + a companion altitude P loop.
  - Takeoff brake: uses `send_prelock_hold`.
  - Locked ceiling hold: position+velocity.
- Airwall: when locked, it slides the hold anchor away from the wall (pure position). When not locked
  it uses a valid position+velocity command. It is now also active during `MOVE_TO_FRONTIER`,
  where it slides the target.
- Prelock and locked hover use the same altitude target (0.55 m). Before, every lock change stepped
  the target by 5 cm.

### ToF
- Only rows 2–5 are used for ranging (the outer rows see the floor). Invalid zones (0xFFFF) are skipped.
- The filter updates once per new frame, not once per loop tick.
- The parser resyncs on the next 0xA5 after a bad checksum, and drains the UART fully.
- ARM needs two matching frames within 500 ms (the M5Stack already sends two). Control-frame debug
  spam is rate-limited, and the AIRWALL log line is no longer duplicated.

### Frontier
- **Range model fix:** the VL53L5CX reports *perpendicular depth*, so the horizontal range along a
  column ray is `depth / cos(column azimuth)`. The onboard map treated depth as range, bending walls
  by up to 6% at the edge columns. Verified on 1,284 flat-wall views (`analysis/scripts/verify_range_model.py`).
- FOV 45° (true VL53L5CX), not 70°.
- Log-odds occupancy (int8 per cell, 2 net hits to become occupied), with each cell updated at most
  once per ray. Map updates are skipped when tilt exceeds 12°.
- `HOVER_SCAN` yaw sweep: rotate 45° in ≤12° steps (below the 20° yaw guardian), alternating
  direction each cycle. This fills the blind wedges between the four sensors.
- Target selection only scans the observed bounding box. Frontier cells closer than 0.6 m are
  ignored. Steps are up to 0.8 m (was 0.25). Clusters are ranked by score plus a size bonus, and the
  minimum cluster size is 3.
- Progress gate: abort if there's no 5 cm improvement in 3 s, or if the error diverges by 20 cm
  (was 2 cm per 2 s).
- Comments corrected: `SET_ATTITUDE_TARGET.thrust` is a climb-rate request with `GUID_OPTIONS=0`;
  the yaw guardian is not about the magnetometer (it was never fused).

## tof_esp32.ino
- Zones with `target_status` other than 5/9 are sent as 0xFFFF.
- A frame is sent when all sensors have new data (or after 150 ms if one is slow). Sensors without
  new data are sent as 0xFFFF, and nothing is sent if no sensor has new data. The `delay(80)` and the
  duplicate status pass are gone.
- ESP-NOW commands are queued, so two frames 8 ms apart are both forwarded.
- **Not compiled here** (no Arduino toolchain on this machine). Build it in the Arduino IDE before
  flashing.

## Verification
- `analysis/tests/test_frontier.c` compiles the real `frontier.c` and checks:
  - every setpoint mask against ArduCopter 4.6.3's acceptance rule;
  - the writer on an undrained pseudo-terminal (whole frames only, 0 parse errors);
  - mapping/selection on the flight-4 and 3_hallWay scan logs;
  - the yaw-sweep state machine.

  Build and run:
  ```
  gcc -std=gnu11 -O1 -w -I<c_library_v2> -I. -o test_frontier analysis/tests/test_frontier.c -lm -lutil
  SCANDIR=<path to your flight-data/onboard/scanlogs> ./test_frontier
  ```
- Builds natively with `-Wall -Wextra` and no new warnings. The RISC-V cross-build wasn't run here.
  Use the README command, **without `-w`**.
- **Not flight-tested.** First flights should be tethered or in a large open space, using
  `--hover-only` before frontier mode.
