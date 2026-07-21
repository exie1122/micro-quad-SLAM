#include "autonomy_config.h"
#include <math.h>

AutonomyConfig autonomy_config_defaults(void) {
  AutonomyConfig c = {0};
  c.max_horizontal_speed_mps = 0.30f;
  c.max_vertical_speed_mps = 0.20f;
  c.max_yaw_rate_dps = 25.0f;
  c.max_acceleration_mps2 = 0.35f;
  c.max_command_step_m = 0.25f;
  c.command_rate_hz = 20.0f;
  c.command_timeout_s = 0.30f;
  c.hover_settle_time_s = 1.0f;
  c.minimum_hover_dwell_s = 1.5f;
  c.maximum_frontier_step_m = 0.25f;
  c.maximum_mission_radius_m = 4.0f;
  c.maximum_altitude_m = 1.5f;
  c.vehicle_radius_m = 0.10f;
  c.propeller_guard_margin_m = 0.05f;
  c.pose_uncertainty_margin_m = 0.05f;
  c.mapping_uncertainty_margin_m = 0.05f;
  c.additional_safety_margin_m = 0.05f;
  c.emergency_obstacle_clearance_m = 0.18f;
  c.map_max_age_s = 0.80f;
  c.pose_max_age_s = 0.30f;
  c.tof_max_age_s = 0.50f;
  c.range_max_age_s = 0.30f;
  c.heartbeat_max_age_s = 1.0f;
  c.optical_flow_max_age_s = 0.30f;
  c.transient_hold_timeout_s = 2.0f;
  c.minimum_battery_v = 7.1f;
  c.minimum_optical_flow_quality = 60;
  c.minimum_frontier_cells = 3;
  c.planner_max_nodes = 20000;
  c.planner_max_path_points = AUTONOMY_MAX_PATH_POINTS;
  c.planner_search_radius_m = 3.0f;
  c.planner_max_path_m = 5.0f;
  c.planner_max_turns = 24;
  c.information_gain_weight = 1.0f;
  c.distance_weight = 1.2f;
  c.turn_weight = 0.01f;
  c.risk_weight = 2.0f;
  c.revisit_weight = 2.0f;
  c.confidence_weight = 2.0f;
  c.path_complexity_weight = 0.20f;
  c.takeoff_target_altitude_m = 0.70f;
  c.takeoff_altitude_tolerance_m = 0.08f;
  c.stable_horizontal_speed_mps = 0.12f;
  c.stable_vertical_speed_mps = 0.10f;
  c.subgoal_reached_tolerance_m = 0.12f;
  c.progress_minimum_gain_m = 0.03f;
  c.progress_divergence_m = 0.15f;
  c.progress_timeout_s = 3.0f;
  c.maximum_recovery_failures = 3;
  return c;
}

float autonomy_required_clearance_m(const AutonomyConfig *c, float resolution_m) {
  if (!c || !isfinite(resolution_m) || resolution_m <= 0.0f) return NAN;
  float total = c->vehicle_radius_m + c->propeller_guard_margin_m +
                c->pose_uncertainty_margin_m + c->mapping_uncertainty_margin_m +
                c->additional_safety_margin_m;
  return total < resolution_m ? resolution_m : total;
}

int autonomy_config_validate(const AutonomyConfig *c) {
  if (!c) return 0;
  return isfinite(c->max_horizontal_speed_mps) && c->max_horizontal_speed_mps > 0.0f &&
         isfinite(c->max_vertical_speed_mps) && c->max_vertical_speed_mps > 0.0f &&
         isfinite(c->max_acceleration_mps2) && c->max_acceleration_mps2 > 0.0f &&
         isfinite(c->command_rate_hz) && c->command_rate_hz >= 2.0f &&
         c->planner_max_nodes > 0 && c->planner_max_nodes <= AUTONOMY_MAX_CELLS &&
         c->planner_max_path_points > 1 && c->planner_max_path_points <= AUTONOMY_MAX_PATH_POINTS &&
         c->minimum_frontier_cells > 0 && c->maximum_recovery_failures > 0;
}
