# Recorded-log replay

Build and replay a committed session:

```bash
make all
./scripts/replay_autonomy.sh log/3_frontierTest_scanlog.bin
```

The wrapper creates a new `runs/<UTC>-<pid>/` with `manifest.txt`, `stdout.log`,
and `decisions.csv`. The manifest records commit, executable/config/input SHA-256,
arguments, backend, outputs, timestamps, and exit status. Existing runs and raw
logs are never overwritten.

The reader requires the exact seven-byte `SCLOG3\n` header and 642-byte `SCN3`
layout. SCLOG1 and SCLOG2 are rejected; a short final record produces a warning.
Unexpected bytes cause bounded resynchronization at the next SCN3 magic without
interpreting skipped data. Backward timestamps are counted as concatenated-session
resets. Original timestamps drive freshness decisions. Replay is deterministic;
speed zero (default) runs as fast as possible, `--speed 1` preserves original
wall-clock intervals, and values above one provide accelerated timed replay.

Each accepted record supplies pose, velocity, yaw, flow quality/ages, range/age,
heartbeat age/mode/armed fields, ToF matrices, and map revision to the actual C
mapper, inflation, frontier detector, path planner, scoring, and safety gate. The
decision log records whether a safe path existed and the new frontier score. A
nearest-Euclidean legacy comparator is not claimed as flight equivalence; historical
legacy choices remain in the paired human logs.

Replay instantiates no serial descriptor, refuses backend actuation, never arms,
and never sends movement. It validates software behavior only—not vehicle dynamics,
sensor timing under load, MAVLink correctness, or flight safety.

Known structural warnings are listed in `LOG_AND_REPO_DISCOVERY.md`. In particular,
`changedEnvironment` contains a corrupt gap and `mixed_scanlog` has a truncated
tail. Those warnings are expected and must remain visible.
