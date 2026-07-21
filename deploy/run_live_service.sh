#!/bin/sh
set -eu
config=${1:-/etc/micro-quad-autonomy/autonomy.conf}
# shellcheck disable=SC1090
. "$config"
run_id=$(date -u +%Y%m%dT%H%M%SZ)-$$
run_dir="$LOG_ROOT/$run_id"
mkdir -p "$run_dir"
commit=unknown
if [ -r "$SOURCE_COMMIT_FILE" ]; then commit=$(sed -n '1p' "$SOURCE_COMMIT_FILE"); fi
exe_hash=$(sha256sum "$AUTONOMY_BIN" | awk '{print $1}')
config_hash=$(sha256sum "$config" | awk '{print $1}')
args="--backend mavlink --allow-live-serial --serial $MAVLINK_DEVICE --baud $MAVLINK_BAUD --expected-system $MAVLINK_TARGET_SYSTEM --expected-component $MAVLINK_TARGET_COMPONENT --tof-serial $TOF_DEVICE --tof-baud $TOF_BAUD --tof-protocol $TOF_PROTOCOL --map-source live --run-dir $run_dir"
if [ "$EXPLORATION_ENABLED" = yes ]; then args="$args --explore"; fi
{
  printf 'run_id=%s\nsource_commit=%s\nexecutable_sha256=%s\nconfig_sha256=%s\nbackend=mavlink\n' "$run_id" "$commit" "$exe_hash" "$config_hash"
  printf 'startup_mode=monitor-only\nstart_mission_argument=absent\narguments=%s\nstarted_utc=%s\n' "$args" "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf 'outputs=manifest.txt,decisions.csv,frontiers.csv,telemetry.csv,stdout.log\n'
} > "$run_dir/manifest.txt"
set +e
# Intentional word splitting: every validated value is restricted to a shell-safe token.
# shellcheck disable=SC2086
"$AUTONOMY_BIN" $args >"$run_dir/stdout.log" 2>&1
status=$?
set -e
printf 'ended_utc=%s\nexit_status=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$status" >> "$run_dir/manifest.txt"
exit "$status"
