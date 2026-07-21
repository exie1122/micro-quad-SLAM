# Autonomy integration report

## Runtime ownership

| Concern | Supported path | Status |
|---|---|---|
| Core mission, safety, planning | `c/autonomy/` | native and RISC-V build target |
| Default launch | `scripts/run_autonomy.sh` | fake backend, exploration disabled |
| Replay | `scripts/replay_autonomy.sh` | strict SCLOG3; no serial/commands |
| Live MAVLink | `vehicle_backend.c` | fail-closed; integration blocked |
| Live map/ToF | `mapping_adapter.c` | replay implemented; live blocked |
| Legacy hover/frontier | `c/*.c` | retained reference, not default |

There was no service, init script, or deployed executable in the repository to
update. Any target-side service that exists outside source control must be audited
before props-off use.

## Old behavior and failure mechanism

The legacy frontier controller derived free/occupied rays from the newest ToF
scan, clustered free cells adjacent to unknown, chose a short 0.25 m target that
passed straight-line clearance, and streamed an absolute local-NED position target.
It did include hover, scan, settle, freshness, flow, emergency obstacle, move-count,
and progress checks. However, frontier mode was its source default, takeoff could
hand directly to `HOVER_SCAN`, and there was no bounded route search through known
free cells. A line test could not prove reachability around obstacles or reject
topologically narrow/twisty paths. ACK and observed state were not separated behind
a testable backend, and thrust/RC liftoff fallback enlarged the failure surface.

## New integration behavior

- Mode, arm, takeoff, and disarm are distinct nonblocking states. Both ACK-like and
  observed state inputs are required for mode/arm; takeoff requires ACK, measured
  altitude increase, target altitude, then stable horizontal/vertical dwell.
- Horizontal motion is legal only in `EXPLORE`. Every other airborne planning or
  recovery state requests hover.
- The grid carries dimensions, resolution, origin, frame, timestamp, sequence,
  validity, and synthetic-source flag. Live mode will reject synthetic maps.
- Obstacles are inflated by vehicle radius plus prop/guard, pose, mapping, and
  safety margins, never less than one cell.
- Frontiers are connected free-cell clusters adjacent to unknown. The representative
  is a validated free cluster member nearest the centroid, never a raw centroid.
- Each candidate requires a bounded path before scoring. Scoring logs gain,
  distance, turn, risk, revisit, uncertainty, complexity, and total components.
- Only the next route point within 0.25 m is exposed as a local subgoal. It must be
  revalidated after hover/scan cycles.
- Stale/uncertain perception causes HOLD first; persistence causes LAND. Heartbeat
  loss, critical battery, ceiling violation, manual stop, emergency obstacle, or
  backend failure land/abort immediately.
- Signals initiate land if armed and close if disarmed. Touchdown must be observed
  before the disarm action is emitted.

## Backend command sequence contract

`connect → fresh expected heartbeat → preflight sources → safe-mode request and
ACK/observation → arm request and ACK/observation → conservative takeoff ACK →
measured rise → target altitude → stable hover dwell → scan → plan → bounded stream
→ hover/replan → land → touchdown observation → disarm observation → close`.

The fake backend implements this contract without external I/O. Replay suppresses
all actuating calls. MAVLink/SITL intentionally return unavailable.

## Test evidence

Final command results are recorded at the end of `AUTONOMY_UPGRADE_LOG.md`. The
strict host suite passes 107 C checks plus Python integration, ASan/UBSan, cppcheck,
all 3,726 committed replay records, and static RISC-V cross-compilation. Unit coverage includes grid conversion/bounds/staleness,
unknown blocking, inflation, frontier clustering/rejection, scoring components,
bounded planning, narrow-gap rejection, safety escalation, smoothing, transition
ordering, timeout/interrupt landing, strict replay versions, deterministic replay,
live refusal, and production-launch ownership.

## Remaining integration blockers

1. Implement the real nonblocking MAVLink backend using the existing generated
   headers, with command-ID-correlated ACKs and independently observed mode/arm,
   altitude, landed, and disarmed states.
2. Connect the real `/dev/ttyS1` ToF/map producer to `MappingAdapter`, including
   pose-time alignment and an explicit feed version; reject fake/synthetic sources.
3. Add target-side service/deployment files and prove that no external startup unit
   still launches an old binary.
4. Run ArduPilot SITL through the same backend and inject stale map/pose/link faults.
5. Conduct props-off UART, sensor age, mode/ACK, takeoff-command suppression, land,
   and touchdown/disarm tests on the actual LicheeRV/H743 wiring.

Until all five are evidenced, the stack is for host replay/dry-run only.
