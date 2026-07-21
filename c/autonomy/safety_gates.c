#include "safety_gates.h"
#include "occupancy_grid.h"
#include <math.h>

static bool stale(uint64_t now,uint64_t stamp,float max_s){
  return !stamp||now<stamp||(now-stamp)>(uint64_t)(max_s*1000.0f);
}
static SafetyResult result(SafetyDecision d,SafetyReason r){SafetyResult x={d,r};return x;}

SafetyResult evaluate_safety(const MissionContext *x,const AutonomyConfig *c,SafetyMonitor *m){
  if(!x||!c||!m)return result(SAFETY_ABORT,SAFETY_REASON_STATE_CORRUPTION);
  if(!x->mission_state_valid)return result(SAFETY_ABORT,SAFETY_REASON_STATE_CORRUPTION);
  if(x->backend_failed)return result(SAFETY_ABORT,SAFETY_REASON_BACKEND);
  if(x->manual_stop)return result(SAFETY_LAND,SAFETY_REASON_MANUAL_STOP);
  if(!x->heartbeat_valid||stale(x->now_ms,x->heartbeat_timestamp_ms,c->heartbeat_max_age_s))
    return result(SAFETY_LAND,SAFETY_REASON_HEARTBEAT);
  if(x->battery_valid&&(!isfinite(x->battery_v)||x->battery_v<c->minimum_battery_v))
    return result(SAFETY_LAND,SAFETY_REASON_BATTERY);
  if(!isfinite(x->pose.z_m)||x->pose.z_m<0.0f||x->pose.z_m>c->maximum_altitude_m)
    return result(SAFETY_LAND,SAFETY_REASON_ALTITUDE);
  if(!isfinite(x->distance_from_mission_origin_m)||x->distance_from_mission_origin_m>c->maximum_mission_radius_m)
    return result(SAFETY_LAND,SAFETY_REASON_MISSION_RADIUS);
  if(isfinite(x->nearest_obstacle_m)&&x->nearest_obstacle_m<c->emergency_obstacle_clearance_m)
    return result(SAFETY_LAND,SAFETY_REASON_OBSTACLE);
  if(!x->horizontal_motion_requested){m->transient_failure_since_ms=0;m->last_reason=SAFETY_REASON_NONE;return result(SAFETY_OK,SAFETY_REASON_NONE);}
  SafetyReason why=SAFETY_REASON_NONE;
  if(!x->expected_mode)why=SAFETY_REASON_MODE;
  else if(!x->vehicle_state_valid)why=SAFETY_REASON_VEHICLE_STATE;
  else if(!x->pose.valid||stale(x->now_ms,x->pose.timestamp_ms,c->pose_max_age_s)||
          !isfinite(x->pose.x_m)||!isfinite(x->pose.y_m)||!isfinite(x->pose.vx_mps)||!isfinite(x->pose.vy_mps)||
          !isfinite(x->pose.localization_uncertainty_m))why=SAFETY_REASON_POSE;
  else if(!x->map||occupancy_grid_validate(x->map,x->now_ms,(uint64_t)(c->map_max_age_s*1000.0f))!=GRID_OK)why=SAFETY_REASON_MAP;
  else if(stale(x->now_ms,x->tof_timestamp_ms,c->tof_max_age_s))why=SAFETY_REASON_TOF;
  else if(stale(x->now_ms,x->range_timestamp_ms,c->range_max_age_s)||!isfinite(x->range_altitude_m))why=SAFETY_REASON_RANGE;
  else if(stale(x->now_ms,x->optical_flow_timestamp_ms,c->optical_flow_max_age_s)||
          x->optical_flow_quality<c->minimum_optical_flow_quality)why=SAFETY_REASON_OPTICAL_FLOW;
  else if(!x->path_valid)why=SAFETY_REASON_PATH;
  else if(!x->subgoal_valid)why=SAFETY_REASON_SUBGOAL;
  else if(!x->command_valid)why=SAFETY_REASON_COMMAND;
  if(why==SAFETY_REASON_NONE){m->transient_failure_since_ms=0;m->last_reason=why;return result(SAFETY_OK,why);}
  if(m->transient_failure_since_ms==0||m->last_reason!=why)m->transient_failure_since_ms=x->now_ms;
  m->last_reason=why;
  if(x->now_ms>=m->transient_failure_since_ms&&
     x->now_ms-m->transient_failure_since_ms>=(uint64_t)(c->transient_hold_timeout_s*1000.0f))
    return result(SAFETY_LAND,why);
  return result(SAFETY_HOVER,why);
}

const char *safety_reason_name(SafetyReason r){
  static const char *names[]={"none","heartbeat","mode","vehicle_state","pose","map","tof","range","optical_flow","altitude","mission_radius","battery","manual_stop","obstacle","path","subgoal","command","backend","state_corruption"};
  return r>=SAFETY_REASON_NONE&&r<=SAFETY_REASON_STATE_CORRUPTION?names[r]:"unknown";
}
