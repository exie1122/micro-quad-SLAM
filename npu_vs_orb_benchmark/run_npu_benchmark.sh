#!/usr/bin/env bash
# Deploy the INT8 .cvimodel to the LicheeRV Nano and benchmark it on the NPU.
# Confirms execution on /dev/cvi-tpu0 and captures peak process RAM.
#
# Usage: ./run_npu_benchmark.sh [host] [count]
set -euo pipefail
cd "$(dirname "$0")"

HOST="${1:-root@10.43.61.1}"
COUNT="${2:-8000}"
MODEL=tiny_yaw_cnn_int8.cvimodel
SSH="ssh -o BatchMode=yes -o StrictHostKeyChecking=no $HOST"

echo "== deploying $MODEL to $HOST =="
scp -o BatchMode=yes -o StrictHostKeyChecking=no "$MODEL" "$HOST:/root/"

echo "== running $COUNT inferences on the NPU =="
$SSH "cd /root
model_runner --model $MODEL --count $COUNT --enable-timer > /tmp/npu_run.log 2>&1 &
PID=\$!
PEAK=0; TPU_OPEN=no
while kill -0 \$PID 2>/dev/null; do
  HWM=\$(grep VmHWM /proc/\$PID/status 2>/dev/null | awk '{print \$2}')
  [ -n \"\$HWM\" ] && [ \"\$HWM\" -gt \"\$PEAK\" ] && PEAK=\$HWM
  ls -l /proc/\$PID/fd 2>/dev/null | grep -q cvi-tpu && TPU_OPEN=yes
  sleep 0.2
done
wait \$PID
echo
cat /tmp/npu_run.log | grep -E 'Inputs|Outputs|input|yaw|Performance'
echo \"peak process RSS : \${PEAK} kB (\$((PEAK/1024)) MB)\"
echo \"NPU /dev/cvi-tpu0 held open : \$TPU_OPEN\""
