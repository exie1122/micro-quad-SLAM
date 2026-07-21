#ifndef AUTONOMY_TYPES_H
#define AUTONOMY_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "autonomy_config.h"

typedef enum {
  CELL_UNKNOWN = 0,
  CELL_FREE,
  CELL_OCCUPIED,
  CELL_INFLATED_OBSTACLE,
  CELL_FRONTIER,
  CELL_ROBOT
} CellState;

typedef enum {
  FRAME_LOCAL_NED = 0,
  FRAME_LOCAL_ENU
} MapFrame;

typedef struct {
  CellState *cells;
  uint16_t width;
  uint16_t height;
  float resolution_m;
  float origin_x_m;
  float origin_y_m;
  uint64_t timestamp_ms;
  uint32_t sequence;
  MapFrame frame;
  bool valid;
  bool synthetic;
} OccupancyGrid;

typedef struct {
  float x_m;
  float y_m;
} GridPoint;

typedef struct {
  float x_m;
  float y_m;
  float z_m;
  float vx_mps;
  float vy_mps;
  float vz_mps;
  float yaw_deg;
  float localization_uncertainty_m;
  uint64_t timestamp_ms;
  bool valid;
} VehiclePose;

typedef struct {
  float vx_mps;
  float vy_mps;
  float vz_mps;
  float yaw_rate_dps;
  uint64_t source_timestamp_ms;
  bool valid;
} MotionCommand;

typedef struct {
  GridPoint points[AUTONOMY_MAX_PATH_POINTS];
  uint16_t count;
  float distance_m;
  float minimum_clearance_m;
  uint16_t turn_count;
  bool valid;
} LocalPath;

typedef enum {
  FRONTIER_ACCEPTED = 0,
  FRONTIER_REJECT_TOO_SMALL,
  FRONTIER_REJECT_CLEARANCE,
  FRONTIER_REJECT_NO_SAFE_REPRESENTATIVE,
  FRONTIER_REJECT_OUT_OF_BOUNDS,
  FRONTIER_REJECT_UNREACHABLE,
  FRONTIER_REJECT_PATH_TOO_LONG,
  FRONTIER_REJECT_PATH_TOO_COMPLEX,
  FRONTIER_REJECT_STALE_MAP
} FrontierRejection;

typedef struct {
  uint32_t id;
  uint32_t cell_count;
  GridPoint centroid;
  GridPoint goal;
  float clearance_m;
  float expected_unknown_gain;
  float path_distance_m;
  float heading_change_deg;
  float obstacle_risk;
  float revisit_penalty;
  float uncertainty;
  float path_complexity;
  float score;
  uint16_t path_turns;
  bool reachable;
} FrontierCandidate;

typedef struct {
  uint32_t cluster_id;
  FrontierRejection reason;
} FrontierRejectedCluster;

typedef struct {
  FrontierCandidate accepted[AUTONOMY_MAX_FRONTIERS];
  FrontierRejectedCluster rejected[AUTONOMY_MAX_REJECTIONS];
  uint16_t accepted_count;
  uint16_t rejected_count;
  bool truncated;
} FrontierSet;

typedef enum {
  SAFETY_OK = 0,
  SAFETY_HOVER,
  SAFETY_LAND,
  SAFETY_ABORT
} SafetyDecision;

typedef enum {
  SAFETY_REASON_NONE = 0,
  SAFETY_REASON_HEARTBEAT,
  SAFETY_REASON_MODE,
  SAFETY_REASON_VEHICLE_STATE,
  SAFETY_REASON_POSE,
  SAFETY_REASON_MAP,
  SAFETY_REASON_TOF,
  SAFETY_REASON_RANGE,
  SAFETY_REASON_OPTICAL_FLOW,
  SAFETY_REASON_ALTITUDE,
  SAFETY_REASON_MISSION_RADIUS,
  SAFETY_REASON_BATTERY,
  SAFETY_REASON_MANUAL_STOP,
  SAFETY_REASON_OBSTACLE,
  SAFETY_REASON_PATH,
  SAFETY_REASON_SUBGOAL,
  SAFETY_REASON_COMMAND,
  SAFETY_REASON_BACKEND,
  SAFETY_REASON_STATE_CORRUPTION
} SafetyReason;

typedef struct {
  uint64_t now_ms;
  uint64_t heartbeat_timestamp_ms;
  uint64_t tof_timestamp_ms;
  uint64_t range_timestamp_ms;
  uint64_t optical_flow_timestamp_ms;
  VehiclePose pose;
  const OccupancyGrid *map;
  float range_altitude_m;
  float distance_from_mission_origin_m;
  float battery_v;
  float nearest_obstacle_m;
  uint8_t optical_flow_quality;
  bool heartbeat_valid;
  bool expected_mode;
  bool vehicle_state_valid;
  bool battery_valid;
  bool manual_stop;
  bool backend_failed;
  bool path_valid;
  bool subgoal_valid;
  bool command_valid;
  bool horizontal_motion_requested;
  bool mission_state_valid;
} MissionContext;

typedef struct {
  SafetyDecision decision;
  SafetyReason reason;
} SafetyResult;

#endif
