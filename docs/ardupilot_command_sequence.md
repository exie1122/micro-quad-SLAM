# ArduPilot command sequence contract

The generated MAVLink library already used by legacy code should be reused behind
`vehicle_backend.h`. Do not copy protocol definitions or command raw motors.

The required nonblocking sequence is:

1. Open the explicitly named UART only after `--backend mavlink`,
   `--allow-live-serial`, and `--serial` are present.
2. Receive a fresh heartbeat from configured/expected system and component IDs.
3. Verify pre-arm health, local pose, range altitude, optical-flow health, live map
   and ToF freshness, battery, and landed state.
4. Request GUIDED/safe mode. Correlate its response and independently observe the
   heartbeat custom mode before advancing.
5. Request arm after all gates pass. Require command ACK and observed armed bit.
6. Issue conservative `MAV_CMD_NAV_TAKEOFF`. Require the matching ACK, measured
   altitude increase, target-with-tolerance, and stable velocity/altitude dwell.
7. Hover and scan. Stream only clamped local-NED velocity or short position
   subgoals at `command_rate_hz`; preserve altitude and limit yaw while translating.
8. On uncertainty, immediately stream hover/zero horizontal motion and replan. On
   persistence or immediate hazards, request LAND and continue monitoring.
9. Require `EXTENDED_SYS_STATE` landed/touchdown agreement before requesting
   disarm. Require observed disarmed state before closing.

Each request needs a monotonic deadline and retry budget. Reading MAVLink, updating
state, planning, and command streaming must continue while an ACK is outstanding;
no sleep-based wait is permitted. An ACK alone is never proof of mode, arm, takeoff,
landing, or disarm success. Repeated send failures are an irrecoverable backend
failure. SIGINT/SIGTERM enters the same land/touchdown/disarm sequence when armed.

## Current implementation boundary

The fake backend proves the interface and mission sequencing. Replay never calls
actuating functions. The MAVLink and SITL backend returns unavailable before opening
any device because the ACK/state machinery above is not implemented in the new
stack. Legacy direct-UART code remains evidence only.
