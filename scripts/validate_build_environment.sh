#!/bin/sh
set -eu
mavlink_dir=${MAVLINK_INCLUDE_DIR:-}
if [ -z "$mavlink_dir" ]; then
  for candidate in "$PWD/third_party/c_library_v2" "$HOME/c_library_v2" /usr/local/include/mavlink/v2.0 /usr/include/mavlink/v2.0; do
    if [ -f "$candidate/ardupilotmega/mavlink.h" ]; then mavlink_dir=$candidate; break; fi
  done
fi
if [ -z "$mavlink_dir" ] || [ ! -f "$mavlink_dir/ardupilotmega/mavlink.h" ]; then
  printf 'ERROR: generated MAVLink 2 ArduPilotMega C headers not found; set MAVLINK_INCLUDE_DIR\n' >&2; exit 2
fi
printf 'MAVLink headers: %s\n' "$mavlink_dir"
cc --version | sed -n '1p'
if command -v riscv64-linux-gnu-gcc >/dev/null 2>&1; then riscv64-linux-gnu-gcc --version | sed -n '1p'; else printf 'RISC-V cross compiler: MISSING\n'; fi
