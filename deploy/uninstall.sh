#!/bin/sh
set -eu
if [ "$(id -u)" -ne 0 ]; then printf 'run as root\n' >&2; exit 2; fi
systemctl disable --now micro-quad-autonomy.service 2>/dev/null || true
rm -f /etc/systemd/system/micro-quad-autonomy.service /etc/logrotate.d/micro-quad-autonomy
rm -rf /opt/micro-quad-autonomy
systemctl daemon-reload
printf 'Program files removed. Configuration and run logs retained in /etc/micro-quad-autonomy and /var/log/micro-quad-autonomy.\n'
