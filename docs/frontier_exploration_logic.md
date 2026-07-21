# Frontier exploration logic

## Map semantics and frames

`OccupancyGrid` stores cell data, width/height, metre resolution, world origin,
monotonic timestamp, revision, NED/ENU frame, validity, and synthetic-source flag.
World-to-cell uses `floor((world-origin)/resolution)` and cell-to-world returns the
cell center. Every access is bounds checked; outside is conservatively blocked.
`CELL_UNKNOWN`, `CELL_OCCUPIED`, and `CELL_INFLATED_OBSTACLE` are never plannable.
`CELL_FRONTIER` and `CELL_ROBOT` are overlays/context and do not replace evidence.

The replay mapper consumes actual SCLOG3 F/R/B/L 8×8 uint16-mm matrices. It takes
the median valid range per sensor column, marks known free ray cells, and marks an
occupied endpoint only within the sensor's bounded range. Existing occupied
evidence is not overwritten by later free ray samples. Pose/yaw are the synchronized
SCLOG record values in local NED convention.

## Clearance

Inflation radius is:

`vehicle radius + prop/guard margin + pose uncertainty margin + mapping uncertainty
margin + additional safety margin`, clamped to at least one grid cell.

Planning runs on a separate inflated copy, preserving the source map. A narrow gap
whose free centerline disappears after inflation is rejected automatically.

## Detection and representative goal

A frontier cell is safe free space with at least one eight-connected unknown
neighbor. Eight-connected frontier cells are clustered with a fixed-capacity queue.
For each cluster the detector calculates size, centroid, unknown-edge gain, and
clearance. A raw centroid is never commanded. The goal is the deterministic safe
cluster member nearest the centroid; row/column ordering breaks equal-distance ties.
Tiny or no-safe-representative clusters are logged with explicit reasons.

## Path and score

A fixed-capacity four-neighbor BFS plans only on known safe cells, bounded by search
radius, explored nodes, path points, total length, and turns. Unknown space is
blocked. Candidates without a valid path are excluded before scoring.

The score is configurable:

`gain_weight*unknown_gain - distance_weight*path_distance - turn_weight*heading -
risk_weight*obstacle_risk - revisit_weight*attempt_penalty -
confidence_weight*uncertainty - complexity_weight*turn_complexity`.

Tie-breaking is total score, then shorter path, then stable cluster ID. The next
subgoal is the farthest path cell within `maximum_frontier_step_m`; long route jumps
are impossible.

## Replanning and progress

The intended cycle is hover → scan → select → plan → one short subgoal → hover →
scan → replan. Measured distance to the subgoal, best distance, divergence, lateral
error, and unexpected velocity belong in live-backend status. First failure enters
HOLD, invalidates the target, scans, and replans. Repeated failures land. The fixed
state machine never repeats an unverified motion command while perception is stale.

The candidate carries revisit and uncertainty fields today; the live integration
must populate attempt history and estimator uncertainty rather than leaving their
replay defaults at a nominal value.
