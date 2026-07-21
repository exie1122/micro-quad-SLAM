#!/bin/sh
set -eu
if [ "$(id -u)" -ne 0 ]; then printf 'run as root\n' >&2; exit 2; fi
repo=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
binary=${1:-$repo/build/autonomy_controller-riscv64-static}
if [ ! -x "$binary" ]; then printf 'missing target executable: %s\n' "$binary" >&2; exit 2; fi
id uav-autonomy >/dev/null 2>&1 || useradd --system --home /nonexistent --shell /usr/sbin/nologin --groups dialout uav-autonomy
install -d -m 0755 /opt/micro-quad-autonomy/bin /etc/micro-quad-autonomy
install -d -o uav-autonomy -g uav-autonomy -m 0750 /var/log/micro-quad-autonomy
install -m 0755 "$binary" /opt/micro-quad-autonomy/bin/autonomy_controller
install -m 0755 "$repo/deploy/validate_deployment.sh" "$repo/deploy/run_live_service.sh" /opt/micro-quad-autonomy/bin/
if [ ! -e /etc/micro-quad-autonomy/autonomy.conf ]; then install -m 0640 -o root -g uav-autonomy "$repo/config/autonomy-live.conf" /etc/micro-quad-autonomy/autonomy.conf; fi
git -C "$repo" rev-parse HEAD > /opt/micro-quad-autonomy/SOURCE_COMMIT
install -m 0644 "$repo/deploy/micro-quad-autonomy.service" /etc/systemd/system/
install -m 0644 "$repo/deploy/micro-quad-autonomy.logrotate" /etc/logrotate.d/micro-quad-autonomy
systemctl daemon-reload
printf 'Installed but NOT enabled or started. Edit /etc/micro-quad-autonomy/autonomy.conf, update service DeviceAllow paths if needed, validate, then explicitly enable.\n'
