# LicheeRV Nano deployment

1. Run `scripts/validate_build_environment.sh`, then cross-build with `make riscv`
   and verify the resulting RISC-V static ELF. Generated MAVLink 2 ArduPilotMega C
   headers are required; set `MAVLINK_INCLUDE_DIR=/path/to/c_library_v2` when they
   are not under `third_party/`, `$HOME/c_library_v2`, or the standard include paths.
2. On the target, run `deploy/install.sh /path/to/autonomy_controller-riscv64-static`
   as root. Installation does not enable or start the service.
3. Edit `/etc/micro-quad-autonomy/autonomy.conf`; confirm `/dev/ttyS2` is MAVLink at
   57600 and `/dev/ttyS1` is the ESP32 A5 stream at 115200. Update the service's
   `DeviceAllow` entries if target names differ.
4. Set `ALLOW_LIVE_SERIAL=yes` only after props are removed, then run
   `/opt/micro-quad-autonomy/bin/validate_deployment.sh /etc/micro-quad-autonomy/autonomy.conf`.
5. `systemctl enable --now micro-quad-autonomy.service` starts monitor-only operation.
   It does not pass `--start-mission`, even if exploration is configured.

Each run receives a new UTC directory under `/var/log/micro-quad-autonomy` with
commit, executable/config hashes, arguments, decision/frontier/telemetry CSVs, and
stdout. Logrotate retains 14 compressed daily rotations. `deploy/uninstall.sh`
stops/removes program/service files but intentionally retains configuration and logs.

For rollback, stop the service, restore a previously hashed binary under
`/opt/micro-quad-autonomy/bin/autonomy_controller`, validate, and restart monitor-only.
Never rollback to either legacy controller.
