# Dry-run and SITL

## Fake/dry-run

```bash
make test
./scripts/run_autonomy.sh                 # hover-only default
./scripts/run_autonomy.sh --explore       # synthetic exploration
```

The fake backend executes the real mission machine, map validation/inflation,
frontier clustering/scoring, BFS planner, safety gate, smoother, and land/disarm
sequence. It logs would-be commands, but opens no UART and controls no hardware.
Synthetic maps are permitted only for fake mode. Exploration requires the explicit
`--explore` switch even there.

## SITL

SITL uses the real MAVLink backend and the real C mission/planning/safety stack.
Build ArduCopter separately, then run:

```bash
python3 scripts/run_sitl_integration.py --ardupilot-root /path/to/ardupilot --fault none
python3 scripts/run_sitl_integration.py --ardupilot-root /path/to/ardupilot --fault stale-tof
```

The harness creates a PTY that emits the exact checksummed `legacy-a5-v0` sensor
wire format; the production parser and mapper consume it. It does not inject a fake
occupancy grid. ArduPilot's built-in optical-flow backend reports quality 51, so the
harness explicitly supplies `--minimum-flow-quality 50`; live deployment retains 60.

The normal test records:

- expected heartbeat/system/component identity;
- GUIDED transition ACK and observation;
- arm ACK and armed observation;
- takeoff ACK, altitude rise/target, stable hover dwell;
- exploration start only with fresh real map adapter data;
- stable-rate bounded command streaming;
- LAND, touchdown observation, then disarm.

The fault test stops the validated ToF byte stream after EXPLORE and requires
`HOVER_SCAN/EXPLORE → HOLD → LAND → DISARM`. PTY unit tests cover delayed/missing/
wrong/duplicate/late ACK, target mismatch, malformed frames, pre-arm health, FC
restart, link loss, and airborne disarm refusal. Core mission/safety tests cover
stale pose/map/flow/range/heartbeat, low battery, planner failure, and no progress.
Those faults were not misreported as real-backend SITL cases. Passing SITL is not
a physical-hardware pass.
