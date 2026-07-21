#ifndef AUTONOMY_CONFIG_H
#define AUTONOMY_CONFIG_H

#include <stdint.h>

#define AUTONOMY_MAX_CELLS 65536u
#define AUTONOMY_MAX_FRONTIERS 64u
#define AUTONOMY_MAX_PATH_POINTS 512u
#define AUTONOMY_MAX_REJECTIONS 64u

typedef struct {
  float max_horizontal_speed_mps;
  float max_vertical_speed_mps;
  float max_yaw_rate_dps;
  float max_acceleration_mps2;
  float max_command_step_m;
  float command_rate_hz;
  float command_timeout_s;
  float hover_settle_time_s;
  float minimum_hover_dwell_s;
  float maximum_frontier_step_m;
  float maximum_mission_radius_m;
  float maximum_altitude_m;

  float vehicle_radius_m;
  float propeller_guard_margin_m;
  float pose_uncertainty_margin_m;
  float mapping_uncertainty_margin_m;
  float additional_safety_margin_m;
  float emergency_obstacle_clearance_m;

  float map_max_age_s;
  float pose_max_age_s;
  float tof_max_age_s;
  float range_max_age_s;
  float heartbeat_max_age_s;
  float optical_flow_max_age_s;
  float transient_hold_timeout_s;
  float minimum_battery_v;
  uint8_t minimum_optical_flow_quality;

  uint16_t minimum_frontier_cells;
  uint32_t planner_max_nodes;
  uint16_t planner_max_path_points;
  float planner_search_radius_m;
  float planner_max_path_m;
  uint16_t planner_max_turns;

  float information_gain_weight;
  float distance_weight;
  float turn_weight;
  float risk_weight;
  float revisit_weight;
  float confidence_weight;
  float path_complexity_weight;

  float takeoff_target_altitude_m;
  float takeoff_altitude_tolerance_m;
  float stable_horizontal_speed_mps;
  float stable_vertical_speed_mps;
  float subgoal_reached_tolerance_m;
  float progress_minimum_gain_m;
  float progress_divergence_m;
  float progress_timeout_s;
  uint8_t maximum_recovery_failures;
} AutonomyConfig;

AutonomyConfig autonomy_config_defaults(void);
float autonomy_required_clearance_m(const AutonomyConfig *config, float resolution_m);
int autonomy_config_validate(const AutonomyConfig *config);

#endif
