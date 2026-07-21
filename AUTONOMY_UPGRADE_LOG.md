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
raw matrices and synchronized pose. The hardened live adapter now validates the
exact A5 length/XOR/uptime ordering, aligns it with fresh local-NED pose and attitude,
and updates a bounded grid without changing UNKNOWN to FREE except along measured rays.

Current vehicle backend: Legacy direct MAVLink 2 over `/dev/ttyS2` at 57,600 baud,
ToF UART `/dev/ttyS1` at 115,200. The new narrow backend supports fake, replay,
live UART, and numeric-IPv4 UDP SITL; fake remains the default.

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

## Integration-completion pass

- Added bounded nonblocking MAVLink UART/UDP transport, explicit connection/loss and
  restart latches, exact target and advertised pre-arm-health verification, message interval requests, telemetry
  validation, command retries, ACK classification, and no force-disarm path.
- Added the exact ESP32 A5 parser. Because the wire format has no version byte, the
  compatibility version must be configured as `legacy-a5-v0`; unknown identifiers
  fail before opening the device.
- Added production live mapping with fresh attitude/pose alignment and synthetic-map
  rejection, plus target deployment/service validation.
- Built official ArduCopter SITL and passed a real-backend normal mission and a
  stale-ToF HOLD/LAND/touchdown/disarm fault run. This does not prove hardware safety.

## Verification record (2026-07-21)

| Command | Result |
|---|---|
| `make clean test` | PASS: strict GNU11 `-Wall -Wextra -Wpedantic -Werror`; 110 core C checks, 161 live-interface C checks, Python integration suite PASS |
| `make sanitize` | PASS: AddressSanitizer + UndefinedBehaviorSanitizer; same 271 C checks and Python integration suite |
| `cppcheck --enable=warning,performance,portability ...` | PASS on project sources; generated MAVLink headers excluded from project finding count |
| `python3 -m py_compile ...` plus parser checks | PASS: corrupt-gap SCLOG3 yielded 234 records and warning; SCLOG1 rejected |
| replay every committed `*scanlog*.bin` | PASS: 3,726 records/decisions; expected one warning each for `changedEnvironment` and `mixed`; no serial/commands |
| external `2_frontierTest_scanlog.bin` replay | PASS: resynchronized after prefix; 102 records, one warning |
| SCLOG1 external replay | PASS (negative test): rejected before record decoding |
| default `scripts/run_autonomy.sh` | PASS: fake, exploration disabled, `MISSION_COMPLETE` |
| `scripts/run_autonomy.sh --explore` | PASS: fake synthetic exploration, `MISSION_COMPLETE` |
| fully opted-in dummy MAVLink request | PASS (negative test): explicit production interface accepted, nonexistent MAVLink device failed closed before ToF open or mission start |
| `make riscv` | PASS: GCC 13.3, static RISC-V/RVC double-float ELF, SHA-256 `7594de9df5df95c1a58b73e15283bf139a13d487e1472f8d0c5405404c47c172` |
| deployment validation | PASS: checked-in safe template fails closed; explicit opt-in test configuration validates without opening hardware |
| real-backend ArduCopter SITL normal mission | PASS: heartbeat, mode/arm ACK plus observation, rise, target altitude, stable hover, exploration command stream, land, touchdown, disarm |
| real-backend ArduCopter SITL stale-ToF fault | PASS: injected stream loss caused HOLD/LAND, confirmed touchdown, then disarm |

SITL used the real C MAVLink backend, the production mission state machine, and a
PTY carrying the repository-confirmed A5 ToF wire format; it did not substitute
the fake vehicle backend. Physical hardware, UART, arming, takeoff, and props-off
tests were not performed.
