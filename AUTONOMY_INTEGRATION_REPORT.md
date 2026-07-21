# Autonomy integration report

## Runtime ownership

| Concern | Supported path | Status |
|---|---|---|
| Core mission, safety, planning | `c/autonomy/` | native and RISC-V build target |
| Default launch | `scripts/run_autonomy.sh` | fake backend, exploration disabled |
| Replay | `scripts/replay_autonomy.sh` | strict SCLOG3; no serial/commands |
| Live MAVLink | `mavlink_backend.c` behind `vehicle_backend.c` | nonblocking UART/UDP, bounded queues, ACK + observed state |
| Live map/ToF | `live_tof_adapter.c` → `mapping_adapter.c` | explicit `legacy-a5-v0`, checksum/order/staleness validation |
| Target startup | `deploy/`, `config/autonomy-live.conf` | systemd monitor-only; never passes `--start-mission` |
| Legacy hover/frontier | `c/*.c` | retained reference, not default |

No prior service existed. The new service is not enabled by installation and fails
until serial permission/configuration is explicitly validated. Target-side units
outside source control still require a props-off audit.

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

The fake backend implements this contract without external I/O and replay suppresses
all actuating calls. Live MAVLink uses one outstanding `COMMAND_LONG` transaction,
an internal correlation ID, exact command/source/target matching, bounded retries,
and independently observed state. ArduPilot's wire command has no request transaction
ID, so serialization is the correlation mechanism; duplicate, late, and wrong ACKs
are counted and ignored.

## Test evidence

Final command results are recorded at the end of `AUTONOMY_UPGRADE_LOG.md`. The
strict host suite passes 110 core C checks and 161 live-interface C checks plus
Python integration, ASan/UBSan, cppcheck, all 3,726 committed replay records, and
static RISC-V cross-compilation. Real-backend ArduCopter SITL passes both a normal
mission and an injected stale-ToF HOLD/LAND mission. Unit coverage includes grid conversion/bounds/staleness,
unknown blocking, inflation, frontier clustering/rejection, scoring components,
bounded planning, narrow-gap rejection, safety escalation, smoothing, transition
ordering, timeout/interrupt landing, strict replay versions, deterministic replay,
live refusal, and production-launch ownership.

## Remaining integration blockers

1. Audit the actual LicheeRV filesystem for older external systemd/init/cron units.
2. Confirm the deployed ESP32 firmware is exactly the unversioned A5 format; live
   launch requires the operator-assigned compatibility ID `legacy-a5-v0` because
   the wire frame itself has no version byte.
3. Conduct props-off UART, sensor age, mode/ACK, takeoff-command suppression, land,
   and touchdown/disarm tests on the actual LicheeRV/H743 wiring.

The software is suitable for the props-off integration sequence, not propeller-on flight.
