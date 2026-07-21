#include "mission_state_machine.h"
#include <string.h>

static MissionOutput output(MissionStateMachine*m,BackendAction a,bool changed,const char*r){MissionOutput o={a,m->state,mission_state_allows_horizontal_motion(m->state),changed,r};return o;}
static void enter(MissionStateMachine*m,MissionState s,uint64_t now){m->state=s;m->entered_ms=now;m->stable_since_ms=0;m->action_sent=false;}
static bool timed_out(const MissionStateMachine*m,uint64_t now,uint64_t ms){return now>=m->entered_ms&&now-m->entered_ms>=ms;}

void mission_state_machine_init(MissionStateMachine*m,uint64_t now){memset(m,0,sizeof(*m));m->state=MISSION_INIT;m->entered_ms=now;}
MissionOutput mission_state_machine_tick(MissionStateMachine*m,const MissionInput*i,const AutonomyConfig*c){
  if(!m||!i||!c){MissionOutput bad={ACTION_CLOSE,MISSION_FAILSAFE,false,true,"invalid_input"};return bad;}
  MissionState old=m->state;const char*reason="tick";BackendAction action=ACTION_NONE;
  if(i->interrupt_requested&&m->state!=MISSION_LAND&&m->state!=MISSION_DISARM){if(i->armed_observed){enter(m,MISSION_LAND,i->now_ms);action=ACTION_LAND;reason="process_interrupt";}else{enter(m,MISSION_COMPLETE,i->now_ms);action=ACTION_CLOSE;reason="process_interrupt_disarmed";}return output(m,action,old!=m->state,reason);}
  if(i->safety_decision==SAFETY_ABORT){enter(m,MISSION_FAILSAFE,i->now_ms);return output(m,ACTION_LAND,old!=m->state,"safety_abort");}
  if(i->safety_decision==SAFETY_LAND&&i->armed_observed&&m->state!=MISSION_LAND&&m->state!=MISSION_DISARM){enter(m,MISSION_LAND,i->now_ms);return output(m,ACTION_LAND,true,"safety_land");}
  switch(m->state){
    case MISSION_INIT:enter(m,MISSION_PREFLIGHT_CHECK,i->now_ms);reason="initialized";break;
    case MISSION_PREFLIGHT_CHECK:if(i->preflight_valid&&i->start_requested){enter(m,MISSION_WAIT_HEARTBEAT,i->now_ms);reason="preflight_passed";}else if(timed_out(m,i->now_ms,5000)){enter(m,MISSION_FAILSAFE,i->now_ms);reason="preflight_timeout";}break;
    case MISSION_WAIT_HEARTBEAT:if(i->heartbeat_fresh){enter(m,MISSION_SET_MODE,i->now_ms);action=ACTION_SET_SAFE_MODE;reason="heartbeat_verified";}else if(timed_out(m,i->now_ms,5000)){enter(m,MISSION_FAILSAFE,i->now_ms);reason="heartbeat_timeout";}break;
    case MISSION_SET_MODE:action=ACTION_SET_SAFE_MODE;if(i->mode_ack&&i->mode_observed){enter(m,MISSION_ARM_REQUESTED,i->now_ms);action=ACTION_ARM;reason="mode_ack_and_observed";}else if(timed_out(m,i->now_ms,4000)){enter(m,MISSION_FAILSAFE,i->now_ms);reason="mode_timeout";}break;
    case MISSION_ARM_REQUESTED:action=ACTION_ARM;if(i->arm_ack&&i->armed_observed){enter(m,MISSION_TAKEOFF,i->now_ms);action=ACTION_TAKEOFF;reason="arm_ack_and_observed";}else if(timed_out(m,i->now_ms,5000)){enter(m,MISSION_FAILSAFE,i->now_ms);reason="arm_timeout";}break;
    case MISSION_TAKEOFF:if(!i->takeoff_ack&&timed_out(m,i->now_ms,3000)){enter(m,MISSION_LAND,i->now_ms);action=ACTION_LAND;reason="takeoff_ack_timeout";}else if(i->takeoff_ack&&!i->altitude_increased&&timed_out(m,i->now_ms,5000)){enter(m,MISSION_LAND,i->now_ms);action=ACTION_LAND;reason="no_altitude_increase";}else if(i->takeoff_ack&&i->altitude_increased&&!i->altitude_at_target&&timed_out(m,i->now_ms,10000)){enter(m,MISSION_LAND,i->now_ms);action=ACTION_LAND;reason="altitude_target_timeout";}else if(i->takeoff_ack&&i->altitude_increased&&i->altitude_at_target){enter(m,MISSION_HOVER_STABILIZE,i->now_ms);action=ACTION_HOVER;reason="takeoff_observed";}break;
    case MISSION_HOVER_STABILIZE:action=ACTION_HOVER;if(i->hover_stable){if(!m->stable_since_ms)m->stable_since_ms=i->now_ms;if(i->now_ms-m->stable_since_ms>=(uint64_t)(c->minimum_hover_dwell_s*1000.0f)){enter(m,MISSION_HOVER_SCAN,i->now_ms);reason="stable_hover_dwell";}}else m->stable_since_ms=0;if(timed_out(m,i->now_ms,8000)){enter(m,MISSION_LAND,i->now_ms);action=ACTION_LAND;reason="hover_stabilization_timeout";}break;
    case MISSION_HOVER_SCAN:action=ACTION_HOVER;if(i->scan_fresh&&timed_out(m,i->now_ms,(uint64_t)(c->hover_settle_time_s*1000.0f))){if(i->exploration_enabled){enter(m,MISSION_SELECT_FRONTIER,i->now_ms);reason="fresh_scan";}else{enter(m,MISSION_HOLD,i->now_ms);reason="exploration_disabled";}}break;
    case MISSION_SELECT_FRONTIER:action=ACTION_HOVER;if(i->frontier_available){enter(m,MISSION_PLAN_PATH,i->now_ms);reason="frontier_selected";}else if(timed_out(m,i->now_ms,2000)){enter(m,MISSION_RETURN_OR_LAND,i->now_ms);reason="no_frontier";}break;
    case MISSION_PLAN_PATH:action=ACTION_HOVER;if(i->path_available){enter(m,MISSION_EXPLORE,i->now_ms);action=ACTION_SEND_POSITION_SUBGOAL;reason="safe_path";}else if(timed_out(m,i->now_ms,1000)){m->recovery_failures++;enter(m,m->recovery_failures>=c->maximum_recovery_failures?MISSION_LAND:MISSION_HOLD,i->now_ms);action=m->state==MISSION_LAND?ACTION_LAND:ACTION_HOVER;reason="planner_failure";}break;
    case MISSION_EXPLORE:action=ACTION_SEND_POSITION_SUBGOAL;if(i->safety_decision==SAFETY_HOVER||i->no_progress){m->recovery_failures++;enter(m,m->recovery_failures>=c->maximum_recovery_failures?MISSION_LAND:MISSION_HOLD,i->now_ms);action=m->state==MISSION_LAND?ACTION_LAND:ACTION_HOVER;reason=i->no_progress?"no_progress":"safety_hold";}else if(i->subgoal_reached){enter(m,MISSION_REPLAN,i->now_ms);action=ACTION_HOVER;reason="subgoal_reached";}break;
    case MISSION_REPLAN:action=ACTION_HOVER;enter(m,MISSION_HOVER_SCAN,i->now_ms);reason="replan_scan";break;
    case MISSION_HOLD:action=ACTION_HOVER;if(i->safety_decision==SAFETY_OK&&i->scan_fresh&&timed_out(m,i->now_ms,(uint64_t)(c->hover_settle_time_s*1000.0f))){enter(m,MISSION_REPLAN,i->now_ms);reason="hold_recovered";}else if(timed_out(m,i->now_ms,(uint64_t)(c->transient_hold_timeout_s*1000.0f))){enter(m,MISSION_LAND,i->now_ms);action=ACTION_LAND;reason="hold_timeout";}break;
    case MISSION_RETURN_OR_LAND:enter(m,MISSION_LAND,i->now_ms);action=ACTION_LAND;reason="mission_complete_land";break;
    case MISSION_LAND:action=ACTION_LAND;if(i->landed_observed){enter(m,MISSION_DISARM,i->now_ms);action=ACTION_DISARM_IF_LANDED;reason="touchdown_confirmed";}break;
    case MISSION_DISARM:action=ACTION_DISARM_IF_LANDED;if(!i->landed_observed){enter(m,MISSION_LAND,i->now_ms);action=ACTION_LAND;reason="lost_landed_confirmation";}else if(i->disarmed_observed){enter(m,MISSION_COMPLETE,i->now_ms);action=ACTION_CLOSE;reason="disarmed_observed";}break;
    case MISSION_FAILSAFE:if(i->armed_observed){enter(m,MISSION_LAND,i->now_ms);action=ACTION_LAND;reason="failsafe_land";}else{enter(m,MISSION_COMPLETE,i->now_ms);action=ACTION_CLOSE;reason="failsafe_disarmed";}break;
    case MISSION_COMPLETE:action=ACTION_CLOSE;break;
    default:enter(m,MISSION_FAILSAFE,i->now_ms);action=ACTION_LAND;reason="state_corruption";break;
  }
  return output(m,action,old!=m->state,reason);
}
bool mission_state_allows_horizontal_motion(MissionState s){return s==MISSION_EXPLORE;}
const char *mission_state_name(MissionState s){static const char*n[]={"INIT","PREFLIGHT_CHECK","WAIT_HEARTBEAT","SET_MODE","ARM_REQUESTED","TAKEOFF","HOVER_STABILIZE","HOVER_SCAN","SELECT_FRONTIER","PLAN_PATH","EXPLORE","REPLAN","HOLD","RETURN_OR_LAND","LAND","DISARM","FAILSAFE","MISSION_COMPLETE"};return s>=MISSION_INIT&&s<=MISSION_COMPLETE?n[s]:"INVALID";}
