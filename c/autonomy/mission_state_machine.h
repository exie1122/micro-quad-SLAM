#ifndef MISSION_STATE_MACHINE_H
#define MISSION_STATE_MACHINE_H

#include "autonomy_types.h"

typedef enum {MISSION_INIT=0,MISSION_PREFLIGHT_CHECK,MISSION_WAIT_HEARTBEAT,MISSION_SET_MODE,
  MISSION_ARM_REQUESTED,MISSION_TAKEOFF,MISSION_HOVER_STABILIZE,MISSION_HOVER_SCAN,
  MISSION_SELECT_FRONTIER,MISSION_PLAN_PATH,MISSION_EXPLORE,MISSION_REPLAN,MISSION_HOLD,
  MISSION_RETURN_OR_LAND,MISSION_LAND,MISSION_DISARM,MISSION_FAILSAFE,MISSION_COMPLETE} MissionState;
typedef enum {ACTION_NONE=0,ACTION_SET_SAFE_MODE,ACTION_ARM,ACTION_TAKEOFF,ACTION_HOVER,
  ACTION_SEND_VELOCITY,ACTION_SEND_POSITION_SUBGOAL,ACTION_LAND,ACTION_DISARM_IF_LANDED,ACTION_CLOSE} BackendAction;

typedef struct {
  uint64_t now_ms;bool start_requested;bool exploration_enabled;bool interrupt_requested;
  bool preflight_valid;bool heartbeat_fresh;bool mode_ack;bool mode_observed;bool arm_ack;bool armed_observed;
  bool takeoff_ack;bool altitude_increased;bool altitude_at_target;bool hover_stable;
  bool scan_fresh;bool frontier_available;bool path_available;bool subgoal_reached;bool no_progress;
  bool landed_observed;bool disarmed_observed;SafetyDecision safety_decision;
} MissionInput;
typedef struct {BackendAction action;MissionState state;bool horizontal_motion_allowed;bool state_changed;const char *reason;} MissionOutput;
typedef struct {MissionState state;uint64_t entered_ms;uint64_t stable_since_ms;uint8_t recovery_failures;bool action_sent;} MissionStateMachine;

void mission_state_machine_init(MissionStateMachine *machine,uint64_t now_ms);
MissionOutput mission_state_machine_tick(MissionStateMachine *machine,const MissionInput *input,
                                         const AutonomyConfig *config);
const char *mission_state_name(MissionState state);
bool mission_state_allows_horizontal_motion(MissionState state);

#endif
