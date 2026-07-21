#!/bin/sh
set -eu
if [ "$#" -lt 1 ]; then echo "usage: $0 path/to/SCLOG3.bin [--speed N]" >&2; exit 2; fi
input=$1
shift
exec "$(dirname "$0")/run_autonomy.sh" --backend replay --input "$input" "$@"
