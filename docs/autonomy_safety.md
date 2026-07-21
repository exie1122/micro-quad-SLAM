# Autonomy safety model

The hardened controller is proof-oriented: movement is refused unless every
required source and route is current and valid. Fake + exploration-disabled is
the default. Unknown cells are blocked, not optimistic free space.

## State transition table

| State | Horizontal motion | Success transition | Timeout/failure |
|---|---:|---|---|
| INIT | no | PREFLIGHT_CHECK | FAILSAFE |
| PREFLIGHT_CHECK | no | WAIT_HEARTBEAT | FAILSAFE after 5 s |
| WAIT_HEARTBEAT | no | SET_MODE | FAILSAFE after 5 s |
| SET_MODE | no | ARM_REQUESTED after ACK + observed mode | FAILSAFE after 4 s |
| ARM_REQUESTED | no | TAKEOFF after ACK + observed armed | FAILSAFE after 5 s |
| TAKEOFF | no | HOVER_STABILIZE after ACK, rise, and target | LAND on ACK/rise/target timeout |
| HOVER_STABILIZE | no | HOVER_SCAN after stable dwell | LAND after 8 s |
| HOVER_SCAN | no | SELECT_FRONTIER or HOLD | remain hover |
| SELECT_FRONTIER | no | PLAN_PATH | RETURN_OR_LAND if none |
| PLAN_PATH | no | EXPLORE | HOLD, then LAND after repeats |
| EXPLORE | yes | REPLAN at subgoal | HOLD/LAND on safety/no progress |
| REPLAN | no | HOVER_SCAN | HOLD/LAND |
| HOLD | no | REPLAN after recovery | LAND on persistence |
| RETURN_OR_LAND | no | LAND | LAND |
| LAND | no | DISARM after observed touchdown | continue landing |
| DISARM | no | MISSION_COMPLETE after observed disarmed | LAND if touchdown certainty is lost |
| FAILSAFE | no | LAND or COMPLETE when already disarmed | close only when safe |

No transition exists from TAKEOFF to EXPLORE. The hover anchor belongs in the
future live backend and must only be captured after the HOVER_STABILIZE dwell.

## Central gate

`evaluate_safety()` checks heartbeat, expected mode, vehicle-state consistency,
finite fresh pose/velocity and uncertainty, fresh map/ToF/range/flow, flow quality,
altitude ceiling, battery, manual stop, nearest obstacle, valid path/subgoal, and
finite command. A transient perception/path failure returns `SAFETY_HOVER`; if the
same reason persists beyond `transient_hold_timeout_s`, it returns `SAFETY_LAND`.
Heartbeat loss, manual stop, low battery, ceiling violation, emergency clearance,
backend failure, and corrupt mission state land or abort immediately.

## Command rules

Commands are finite-checked, vector-speed clamped, vertical/yaw clamped, acceleration
limited per bounded timestep, and stale commands decay toward zero. HOLD and LAND
force zero horizontal velocity; LAND also suppresses yaw. Proposed subgoals are
shortened but never lengthened. The backend must transmit at the configured stable
rate and treat send failures as safety input.

## DO NOT PROCEED TO REAL FLIGHT IF:

- MAVLink ACK plus observed-state backend tests are not passing.
- The production ToF/map feed is not connected and freshness/version checked.
- Any startup service launches a legacy `c/*.c` binary.
- SITL stale-link/stale-map/planner-failure/landing tests have not passed.
- The serial device, FC system/component IDs, GUIDED availability, pre-arm state,
  optical flow, rangefinder, battery, and landed state are not verified props-off.
- A run manifest, current commit/executable/config hashes, or writable log storage
  is missing.
- Props are installed during first integration tests.

Current status meets several of these blockers; real flight is prohibited.
