#!/bin/sh
set -eu
config=${1:-/etc/micro-quad-autonomy/autonomy.conf}
offline=false
if [ "${2:-}" = "--offline" ]; then offline=true; fi
if [ ! -r "$config" ]; then printf 'ERROR: unreadable configuration: %s\n' "$config" >&2; exit 2; fi
# Configuration is installed root-owned and must contain simple KEY=value assignments.
if grep -Ev '^[A-Z][A-Z0-9_]*=[A-Za-z0-9_./:-]+$|^[[:space:]]*(#.*)?$' "$config" | grep . >/dev/null; then
  printf 'ERROR: configuration contains unsupported shell syntax\n' >&2; exit 2
fi
# shellcheck disable=SC1090
. "$config"
required='ALLOW_LIVE_SERIAL MAVLINK_DEVICE MAVLINK_BAUD MAVLINK_TARGET_SYSTEM MAVLINK_TARGET_COMPONENT TOF_DEVICE TOF_BAUD TOF_PROTOCOL MAP_SOURCE EXPLORATION_ENABLED LOG_ROOT AUTONOMY_BIN SOURCE_COMMIT_FILE'
for key in $required; do eval "value=\${$key-}"; if [ -z "$value" ]; then printf 'ERROR: missing %s\n' "$key" >&2; exit 2; fi; done
if [ "$ALLOW_LIVE_SERIAL" != yes ]; then printf 'ERROR: ALLOW_LIVE_SERIAL must be explicitly set to yes after props-off review\n' >&2; exit 3; fi
if [ "$TOF_PROTOCOL" != legacy-a5-v0 ]; then printf 'ERROR: unsupported/unconfirmed ToF protocol: %s\n' "$TOF_PROTOCOL" >&2; exit 3; fi
if [ "$MAP_SOURCE" != live ]; then printf 'ERROR: production service requires MAP_SOURCE=live\n' >&2; exit 3; fi
case "$EXPLORATION_ENABLED" in yes|no) ;; *) printf 'ERROR: EXPLORATION_ENABLED must be yes or no\n' >&2; exit 2;; esac
case "$MAVLINK_BAUD:$TOF_BAUD:$MAVLINK_TARGET_SYSTEM:$MAVLINK_TARGET_COMPONENT" in *[!0-9:]*|'') printf 'ERROR: baud rates and MAVLink IDs must be numeric\n' >&2; exit 2;; esac
if [ ! -x "$AUTONOMY_BIN" ] && [ "$offline" = false ]; then printf 'ERROR: executable missing: %s\n' "$AUTONOMY_BIN" >&2; exit 3; fi
if [ "$offline" = false ]; then
  for device in "$MAVLINK_DEVICE" "$TOF_DEVICE"; do
    if [ ! -c "$device" ]; then printf 'ERROR: serial device missing/not character device: %s\n' "$device" >&2; exit 3; fi
    if [ ! -r "$device" ] || [ ! -w "$device" ]; then printf 'ERROR: insufficient serial permissions: %s\n' "$device" >&2; exit 3; fi
  done
  if [ ! -d "$LOG_ROOT" ] || [ ! -w "$LOG_ROOT" ]; then printf 'ERROR: log root missing/not writable: %s\n' "$LOG_ROOT" >&2; exit 3; fi
fi
printf 'deployment configuration: VALID (%s mode; startup remains monitor-only)\n' "$(if [ "$offline" = true ]; then printf offline; else printf target; fi)"
