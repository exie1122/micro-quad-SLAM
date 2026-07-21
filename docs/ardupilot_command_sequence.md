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

## Implemented correlation boundary

`mavlink_backend.c` implements this sequence with nonblocking `O_NONBLOCK` I/O,
a 16-frame transmit ring, one outstanding command, a 1.2 s deadline, and two retries.
`COMMAND_LONG` has no caller transaction number, so the backend assigns an internal
correlation ID and serializes commands; an ACK must match command, FC source IDs,
companion target extensions (when populated), and the active time window. ACK alone
never sets mode, armed, altitude, or landed state. An accepted arm ACK and an accepted
disarm ACK are distinguished by stored parameter 1. Reconnect/FC restart latches an
operator-reset requirement and never re-arms.

Explicit message intervals request heartbeat, attitude, local-NED pose, range,
battery/status, landed state, and optical flow. Invalid floating-point telemetry is
discarded. When ArduPilot advertises `MAV_SYS_STATUS_PREARM_CHECK`, preflight waits
for that bit to be healthy in a fresh `SYS_STATUS`; an arm rejection still aborts.
Only `SET_POSITION_TARGET_LOCAL_NED` and standard mode/arm/takeoff/land
commands are emitted; there is no motor or force-disarm command.
