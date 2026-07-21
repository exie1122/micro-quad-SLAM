#!/bin/sh
set -eu
repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$repo_dir"
make all >/dev/null
run_id=$(date -u +%Y%m%dT%H%M%SZ)-$$
run_dir="runs/$run_id"
mkdir -p "$run_dir"
commit=$(git rev-parse HEAD 2>/dev/null || printf unknown)
if git diff --quiet --ignore-submodules HEAD 2>/dev/null && [ -z "$(git ls-files --others --exclude-standard 2>/dev/null)" ]; then dirty=false; else dirty=true; fi
exe_hash=$(sha256sum build/autonomy_controller | awk '{print $1}')
config_hash=$(sha256sum c/autonomy/autonomy_config.c c/autonomy/autonomy_config.h | sha256sum | awk '{print $1}')
backend=fake
prev=
for arg in "$@"; do
  if [ "$prev" = backend ]; then backend=$arg; fi
  if [ "$arg" = --backend ]; then prev=backend; else prev=; fi
done
{
  printf 'run_id=%s\n' "$run_id"
  printf 'commit=%s\n' "$commit"
  printf 'working_tree_dirty=%s\n' "$dirty"
  printf 'executable_sha256=%s\n' "$exe_hash"
  printf 'config_sha256=%s\n' "$config_hash"
  printf 'backend=%s\n' "$backend"
  printf 'started_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf 'arguments='; printf '%s ' "$@"; printf '\n'
  for arg in "$@"; do
    if [ -f "$arg" ]; then printf 'input_sha256=%s  %s\n' "$(sha256sum "$arg" | awk '{print $1}')" "$arg"; fi
  done
  printf 'outputs=manifest.txt,decisions.csv,frontiers.csv,telemetry.csv,stdout.log\n'
} > "$run_dir/manifest.txt"
set +e
./build/autonomy_controller --run-dir "$run_dir" "$@" > "$run_dir/stdout.log" 2>&1
rc=$?
set -e
cat "$run_dir/stdout.log"
printf 'ended_utc=%s\nexit_status=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$rc" >> "$run_dir/manifest.txt"
exit "$rc"
