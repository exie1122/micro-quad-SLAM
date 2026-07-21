# Autonomy upgrade log

Current live runtime: No systemd service or startup script was present. The README
compiled and operators manually launched the monolithic C hover controller.

Current deployable source: Before this pass, `c/clean_uav_fc_tof_nav.c`; frontier
flight experiments used the separate `c/frontier.c`. The supported source is now
`c/autonomy/*.c`, built as `build/autonomy_controller`.

Current build/deployment path: Previously one direct
`riscv64-linux-gnu-gcc ... c/clean_uav_fc_tof_nav.c` command. Now `make`,
`make test`, `make riscv`, and `scripts/run_autonomy.sh` are authoritative.

Current mapping input: Legacy frontier code built a 1024×1024, 0.15 m grid from
four 8×8 ToF matrices received on `/dev/ttyS1`; SCLOG3 preserves the same F/R/B/L
raw matrices and synchronized pose. The hardened replay adapter reconstructs a
bounded local grid with the same ray semantics. The hardened live feed is not yet
connected and fails closed.

Current vehicle backend: Legacy direct MAVLink 2 over `/dev/ttyS2` at 57,600 baud,
ToF UART `/dev/ttyS1` at 115,200. The new narrow backend has fake/replay support;
MAVLink/SITL return `BACKEND_UNAVAILABLE` before I/O.

Current mission sequence: `INIT → PREFLIGHT_CHECK → WAIT_HEARTBEAT → SET_MODE →
ARM_REQUESTED → TAKEOFF → HOVER_STABILIZE → HOVER_SCAN → SELECT_FRONTIER →
PLAN_PATH → EXPLORE → REPLAN/HOLD → LAND → DISARM → MISSION_COMPLETE`.

Known crash/failure risks: legacy liftoff assistance sent attitude/thrust and RC
override fallbacks; `frontier.c` defaulted to frontier mode; it transitioned
`TAKEOFF → HOVER_SCAN`, selected a short line-of-sight target without a bounded
route planner, repeatedly streamed absolute position targets, force-disarmed in
some abnormal states, and treated ACK/state handling inside a monolith. Logs show
takeoff ramp failures, no-progress retries, prelock timeouts, and recovery landings.

Existing safety gates: Legacy heartbeat, battery, range, optical-flow, XY-lock,
ceiling, air-wall, progress, and landing checks existed and are retained as
evidence. Their coupling and unsafe launch defaults prevented independent proof.

Existing tests: No tests were committed in the nested Git repository.

Upgrade plan: preserve legacy evidence; make bounded C modules authoritative;
default to non-actuating fake/hover-only; add explicit occupancy/frontier/path,
safety, smoothing, state-machine, SCLOG3 replay, provenance, unit/integration
tests, cross-build, and live fail-closed documentation.

## Implemented changes

- Added deterministic fixed-capacity C modules under `c/autonomy/`; no allocation
  occurs in the control/planning loop.
- Added explicit map metadata and cell states. Unknown is never plannable.
- Added configurable composite clearance and separate obstacle inflation.
- Added connected frontier clustering, safe representative selection, component
  scoring, deterministic tie-breaks, bounded four-neighbor BFS, and local subgoals.
- Added central safety decisions (`OK`, `HOVER`, `LAND`, `ABORT`) with transient
  hold timing and immediate heartbeat/battery/altitude/manual/backend responses.
- Added speed, vertical, yaw, acceleration, timestep, stale-command, and subgoal
  limiting. HOLD/LAND remove horizontal motion.
- Added fake/replay backend protections and strict live CLI guards.
- Hardened the Python SCLOG3 plot reader against SCLOG1 and corruption.
- Marked every old C controller as legacy. `c/frontier.c` additionally refuses to
  start without `--allow-legacy-unsafe-frontier`.

## Deliberately blocked work

The existing monolithic MAVLink functions are not copied into the new backend.
Doing so would preserve blocking/coupled behavior without proving ACK plus observed
state. Live MAVLink and SITL therefore remain unavailable. Likewise, the real ToF
UART mapper is described and replayed, but is not yet connected to the new live
adapter. These are real-flight blockers, not simulated completions.

## Verification record (2026-07-21)

| Command | Result |
|---|---|
| `make clean test` | PASS: strict GNU11 `-Wall -Wextra -Wpedantic -Werror`; 107 C checks, integration suite PASS |
| `make sanitize` | PASS: AddressSanitizer + UndefinedBehaviorSanitizer; same 107 checks and integration suite |
| `cppcheck --enable=warning,performance,portability ...` | PASS: 16 files, no findings at error threshold |
| `python3 -m py_compile ...` plus parser checks | PASS: corrupt-gap SCLOG3 yielded 234 records and warning; SCLOG1 rejected |
| replay every committed `*scanlog*.bin` | PASS: 3,726 records/decisions; expected one warning each for `changedEnvironment` and `mixed`; no serial/commands |
| external `2_frontierTest_scanlog.bin` replay | PASS: resynchronized after prefix; 102 records, one warning |
| SCLOG1 external replay | PASS (negative test): rejected before record decoding |
| default `scripts/run_autonomy.sh` | PASS: fake, exploration disabled, `MISSION_COMPLETE` |
| `scripts/run_autonomy.sh --explore` | PASS: fake synthetic exploration, `MISSION_COMPLETE` |
| fully opted-in dummy MAVLink request | PASS (negative test): refused before serial open because production map/backend unavailable |
| `make riscv` | PASS: GCC 13.3, static RISC-V/RVC double-float ELF, SHA-256 `ee069fa4ac45089021c2b21b7f602229fa2ccd504f7d193d656a3b35d370fab4` |

SITL: **not run / unavailable**. No SITL connector existed and the new backend
correctly returns unavailable rather than substituting the fake backend. Physical
hardware, UART, arming, takeoff, and props-off tests were not performed.
