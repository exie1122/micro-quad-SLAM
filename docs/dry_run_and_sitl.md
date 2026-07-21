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

`--backend sitl` currently fails closed with `BACKEND_UNAVAILABLE`. This is
intentional: no ArduPilot SITL connector existed in the repository, and pointing
the fake backend at a network endpoint would not test the real MAVLink path.

Before enabling SITL, implement it through `vehicle_backend.h` using the same
message/ACK/observation logic intended for UART. Then automate and record:

- expected heartbeat/system/component identity;
- GUIDED transition ACK and observation;
- arm ACK and armed observation;
- takeoff ACK, altitude rise/target, stable hover dwell;
- exploration start only with fresh real map adapter data;
- stable-rate bounded command streaming;
- injected stale pose, map, ToF, flow, range, and heartbeat;
- planner/no-progress recovery and repeat-failure landing;
- LAND, touchdown observation, then disarm.

Passing SITL is not a physical-hardware pass and must not remove the live opt-in or
synthetic-map prohibition.
