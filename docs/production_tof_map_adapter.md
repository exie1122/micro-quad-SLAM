# Production ToF and map adapter

Repository evidence confirms the ESP32 frame emitted by `arduino/tof_esp32.ino`:
518 bytes, A5 sync, little-endian ESP32 `millis()`, F/R/B/L 8×8 unsigned-millimetre
matrices, and XOR over bytes 0–516 in byte 517. A separate 7-byte A6 control frame
exists and is checksum-validated/ignored; sensor input can never arm the vehicle.

There is no on-wire version field. The live command therefore requires the explicit
operator compatibility selection `--tof-protocol legacy-a5-v0`. Other/missing names
fail before opening serial. The bounded parser rejects bad checksums, partial frames,
repeated/out-of-order uptime, all-invalid scans, and disconnects. Values outside
100–4000 mm become invalid zones, never free space.

Accepted scans receive a companion monotonic timestamp and local sequence. Mapping
requires a finite, fresh local-NED position and attitude; a stale/missing yaw cannot
rotate rays into the grid. Live mapping rejects `grid.synthetic`, preserves UNKNOWN,
does not clear existing occupied evidence with a free ray, and records scan time as
map revision time. A stopped stream reaches HOLD and then LAND.
