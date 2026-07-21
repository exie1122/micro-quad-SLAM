#include "autonomy_config.h"
#include "command_smoother.h"
#include "frontier_detector.h"
#include "frontier_history.h"
#include "frontier_scorer.h"
#include "local_path_planner.h"
#include "mapping_adapter.h"
#include "mission_state_machine.h"
#include "occupancy_grid.h"
#include "recorded_log.h"
#include "progress_monitor.h"
#include "run_logger.h"
#include "safety_gates.h"
#include "vehicle_backend.h"
#include "live_tof_adapter.h"
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

static volatile sig_atomic_t interrupted=0;
static void on_signal(int sig){(void)sig;interrupted=1;}
typedef struct {BackendKind kind;const char*input;const char*run_dir;const char*serial;const char*tof_serial;TofProtocol tof_protocol;bool explore;bool start_mission;bool allow_live;bool live_map;float speed;int baud;int tof_baud;int minimum_flow_quality_override;uint8_t expected_system;uint8_t expected_component;float max_runtime_s;} Options;

static void usage(const char*p){fprintf(stderr,"Usage: %s [--backend fake|replay|sitl|mavlink] [--input SCLOG3] [--run-dir DIR] [--explore]\n       live requires --allow-live-serial --serial /dev/... --tof-serial /dev/... --tof-protocol legacy-a5-v0 --map-source live\n       arming additionally requires --start-mission; exploration additionally requires --explore\n",p);}
static bool parse_options(int argc,char**argv,Options*o){memset(o,0,sizeof(*o));o->kind=BACKEND_FAKE;o->run_dir="runs/direct";o->speed=0.0f;o->baud=115200;o->tof_baud=921600;o->minimum_flow_quality_override=-1;o->expected_system=1;o->expected_component=MAV_COMP_ID_AUTOPILOT1;for(int i=1;i<argc;++i){
  if(!strcmp(argv[i],"--backend")&&i+1<argc){const char*v=argv[++i];if(!strcmp(v,"fake"))o->kind=BACKEND_FAKE;else if(!strcmp(v,"replay"))o->kind=BACKEND_REPLAY;else if(!strcmp(v,"sitl"))o->kind=BACKEND_SITL;else if(!strcmp(v,"mavlink"))o->kind=BACKEND_MAVLINK;else return false;}
  else if(!strcmp(argv[i],"--input")&&i+1<argc)o->input=argv[++i];else if(!strcmp(argv[i],"--run-dir")&&i+1<argc)o->run_dir=argv[++i];
  else if(!strcmp(argv[i],"--serial")&&i+1<argc)o->serial=argv[++i];else if(!strcmp(argv[i],"--allow-live-serial"))o->allow_live=true;
  else if(!strcmp(argv[i],"--tof-serial")&&i+1<argc)o->tof_serial=argv[++i];else if(!strcmp(argv[i],"--tof-protocol")&&i+1<argc)o->tof_protocol=tof_protocol_from_name(argv[++i]);
  else if(!strcmp(argv[i],"--baud")&&i+1<argc)o->baud=atoi(argv[++i]);else if(!strcmp(argv[i],"--tof-baud")&&i+1<argc)o->tof_baud=atoi(argv[++i]);
  else if(!strcmp(argv[i],"--minimum-flow-quality")&&i+1<argc)o->minimum_flow_quality_override=atoi(argv[++i]);
  else if(!strcmp(argv[i],"--expected-system")&&i+1<argc)o->expected_system=(uint8_t)atoi(argv[++i]);else if(!strcmp(argv[i],"--expected-component")&&i+1<argc)o->expected_component=(uint8_t)atoi(argv[++i]);
  else if(!strcmp(argv[i],"--map-source")&&i+1<argc)o->live_map=!strcmp(argv[++i],"live");else if(!strcmp(argv[i],"--explore"))o->explore=true;else if(!strcmp(argv[i],"--start-mission"))o->start_mission=true;
  else if(!strcmp(argv[i],"--speed")&&i+1<argc)o->speed=strtof(argv[++i],NULL);else if(!strcmp(argv[i],"--max-runtime-s")&&i+1<argc)o->max_runtime_s=strtof(argv[++i],NULL);else if(!strcmp(argv[i],"--help")){usage(argv[0]);exit(0);}else return false;
 }return isfinite(o->speed)&&o->speed>=0.0f&&isfinite(o->max_runtime_s)&&o->max_runtime_s>=0.0f&&o->minimum_flow_quality_override<=255&&o->expected_system>0&&o->expected_component>0;}
static bool ensure_dir(const char*d){if(mkdir("runs",0755)&&errno!=EEXIST)return false;if(mkdir(d,0755)&&errno!=EEXIST)return false;return true;}

static PlanStatus plan_best(const OccupancyGrid*g,const VehiclePose*pose,const AutonomyConfig*c,uint64_t now,
                            FrontierWorkspace*fw,PlannerWorkspace*pw,FrontierSet*set,LocalPath*best_path,
                            FrontierCandidate*best,FrontierHistory*history,RunLogger*logger){
  if(!frontier_detect(g,c,now,fw,set)){run_logger_frontier_rejection(logger,now,0,"map_invalid_or_stale");return PLAN_INVALID_INPUT;}
  for(uint16_t i=0;i<set->rejected_count;++i)
    run_logger_frontier_rejection(logger,now,set->rejected[i].cluster_id,frontier_rejection_name(set->rejected[i].reason));
  GridPoint start={pose->x_m,pose->y_m};
  for(uint16_t i=0;i<set->accepted_count;++i){FrontierCandidate*f=&set->accepted[i];LocalPath p;
    PlanStatus status=local_path_plan(g,start,f->goal,c,pw,&p);f->reachable=status==PLAN_OK;
    if(f->reachable){f->path_distance_m=p.distance_m;f->path_turns=p.turn_count;f->path_complexity=(float)p.turn_count;f->obstacle_risk=1.0f/fmaxf(f->clearance_m,0.01f);f->uncertainty=pose->localization_uncertainty_m;f->revisit_penalty=history?frontier_history_penalty(history,f->goal,now,c->maximum_frontier_step_m):0.0f;
      float desired=atan2f(f->goal.y_m-pose->y_m,f->goal.x_m-pose->x_m)*57.2957795f;f->heading_change_deg=remainderf(desired-pose->yaw_deg,360.0f);
    }else run_logger_frontier_rejection(logger,now,f->id,status==PLAN_SEARCH_LIMIT?"planner_node_limit":status==PLAN_TOO_LONG?"path_too_long":status==PLAN_TOO_COMPLEX?"path_too_complex":"unreachable");}
  int pick=frontier_select_best(set->accepted,set->accepted_count,c);
  for(uint16_t i=0;i<set->accepted_count;++i)if(set->accepted[i].reachable)run_logger_frontier(logger,now,&set->accepted[i]);
  if(pick<0) return PLAN_NO_PATH;
  *best=set->accepted[pick];
  PlanStatus status=local_path_plan(g,start,best->goal,c,pw,best_path);return status;
}

static void make_fake_map(MappingAdapter*a,CellState*storage,uint64_t now){mapping_adapter_init(a,storage,48,48,0.15f,0.0f,0.0f,true);for(int y=10;y<=37;++y)for(int x=10;x<=37;++x)occupancy_grid_set(&a->grid,x,y,CELL_FREE);for(int y=19;y<=28;++y)occupancy_grid_set(&a->grid,29,y,CELL_OCCUPIED);a->grid.timestamp_ms=now;a->grid.sequence=1;}
static void process_action(VehicleBackend*b,BackendAction a,const AutonomyConfig*c,const MotionCommand*cmd,GridPoint subgoal){
  switch(a){case ACTION_SET_SAFE_MODE:backend_set_safe_mode(b);break;case ACTION_ARM:backend_arm(b);break;case ACTION_TAKEOFF:backend_takeoff(b,c->takeoff_target_altitude_m);break;case ACTION_HOVER:backend_hover(b);break;case ACTION_SEND_VELOCITY:backend_send_velocity(b,cmd);break;case ACTION_SEND_POSITION_SUBGOAL:backend_send_position_subgoal(b,subgoal,c->takeoff_target_altitude_m);break;case ACTION_LAND:backend_land(b);break;case ACTION_DISARM_IF_LANDED:backend_disarm_if_landed(b);break;case ACTION_CLOSE:backend_close(b);break;default:break;}
}

static int run_fake(const Options*o,RunLogger*logger,const AutonomyConfig*c){
  CellState raw_cells[48*48],inflated_cells[48*48];MappingAdapter map;make_fake_map(&map,raw_cells,1000);
  OccupancyGrid inflated=map.grid;inflated.cells=inflated_cells;if(!occupancy_grid_inflate(&map.grid,&inflated,autonomy_required_clearance_m(c,map.grid.resolution_m)))return 2;
  VehicleBackend backend={.kind=BACKEND_FAKE};backend_connect(&backend);VehiclePose pose={.x_m=0,.y_m=0,.z_m=0,.yaw_deg=0,.localization_uncertainty_m=.04f,.valid=true};
  MissionStateMachine machine;mission_state_machine_init(&machine,1000);SafetyMonitor monitor={0};CommandSmoother smoother={0};FrontierWorkspace fw;PlannerWorkspace pw;FrontierSet frontiers;LocalPath path={0};FrontierCandidate selected={0};GridPoint subgoal={0};bool selected_ok=false,path_ok=false;FrontierHistory history={0};ProgressMonitor progress={0};
  for(uint32_t tick=0;tick<900&&machine.state!=MISSION_COMPLETE;++tick){uint64_t now=1000+(uint64_t)tick*50;map.grid.timestamp_ms=now;inflated.timestamp_ms=now;backend_wait_heartbeat(&backend,now);BackendStatus bs;backend_get_status(&backend,&bs);pose.timestamp_ms=now;pose.z_m=bs.altitude_m;
    if(machine.state==MISSION_SELECT_FRONTIER||machine.state==MISSION_PLAN_PATH||machine.state==MISSION_EXPLORE){PlanStatus ps=plan_best(&inflated,&pose,c,now,&fw,&pw,&frontiers,&path,&selected,&history,logger);selected_ok=ps==PLAN_OK;float max_step=fminf(c->maximum_frontier_step_m,c->max_command_step_m);GridPoint current={pose.x_m,pose.y_m};path_ok=selected_ok&&local_path_next_subgoal(&path,current,max_step,&subgoal)&&local_path_segment_valid(&inflated,current,subgoal);}
    MotionCommand requested={0};requested.source_timestamp_ms=now;requested.valid=true;if(machine.state==MISSION_EXPLORE&&path_ok){float dx=subgoal.x_m-pose.x_m,dy=subgoal.y_m-pose.y_m,d=hypotf(dx,dy);if(d>0.001f){requested.vx_mps=dx/d*c->max_horizontal_speed_mps;requested.vy_mps=dy/d*c->max_horizontal_speed_mps;}}
    MissionContext ctx={.now_ms=now,.heartbeat_timestamp_ms=now,.tof_timestamp_ms=now,.range_timestamp_ms=now,.optical_flow_timestamp_ms=now,.pose=pose,.map=&inflated,.range_altitude_m=pose.z_m,.distance_from_mission_origin_m=hypotf(pose.x_m,pose.y_m),.battery_v=8.0f,.nearest_obstacle_m=1.0f,.optical_flow_quality=100,.heartbeat_valid=true,.expected_mode=bs.mode_safe,.vehicle_state_valid=true,.battery_valid=true,.path_valid=path_ok,.subgoal_valid=path_ok,.command_valid=requested.valid,.horizontal_motion_requested=machine.state==MISSION_EXPLORE,.mission_state_valid=true};
    SafetyResult safety=evaluate_safety(&ctx,c,&monitor);run_logger_telemetry(logger,&ctx,machine.state,bs.armed,bs.mode_safe?4u:0u,path_ok?&path:NULL,path_ok?&subgoal:NULL);bool clamp=false;MotionCommand sent={0};command_smoother_update(&smoother,c,&requested,now,mission_state_allows_horizontal_motion(machine.state)&&safety.decision==SAFETY_OK,machine.state==MISSION_LAND,&sent,&clamp);
    if(machine.state==MISSION_EXPLORE&&path_ok&&(!progress.active||hypotf(progress.target.x_m-subgoal.x_m,progress.target.y_m-subgoal.y_m)>.01f))progress_monitor_start(&progress,(GridPoint){pose.x_m,pose.y_m},subgoal,now);
    ProgressResult progress_result=machine.state==MISSION_EXPLORE?progress_monitor_update(&progress,&pose,now,c):PROGRESS_OK;
    bool reached=progress_result==PROGRESS_REACHED;bool no_progress=progress_result==PROGRESS_STALLED||progress_result==PROGRESS_DIVERGED||progress_result==PROGRESS_EXCESS_SPEED;
    MissionInput in={.now_ms=now,.start_requested=true,.exploration_enabled=o->explore,.interrupt_requested=interrupted||(tick>700),.preflight_valid=true,.heartbeat_fresh=bs.heartbeat,.mode_ack=bs.mode_safe,.mode_observed=bs.mode_safe,.arm_ack=bs.armed,.armed_observed=bs.armed,.takeoff_ack=!bs.landed,.altitude_increased=bs.altitude_m>.10f,.altitude_at_target=fabsf(bs.altitude_m-c->takeoff_target_altitude_m)<c->takeoff_altitude_tolerance_m,.hover_stable=fabsf(pose.vx_mps)<c->stable_horizontal_speed_mps&&fabsf(pose.vy_mps)<c->stable_horizontal_speed_mps,.scan_fresh=true,.frontier_available=selected_ok,.path_available=path_ok,.subgoal_reached=reached,.no_progress=no_progress,.landed_observed=bs.landed,.disarmed_observed=!bs.armed,.safety_decision=safety.decision};
    MissionOutput mo=mission_state_machine_tick(&machine,&in,c);if(mo.state_changed&&machine.state==MISSION_EXPLORE)frontier_history_record(&history,selected.goal,now,false,c->maximum_frontier_step_m);if(no_progress)frontier_history_record(&history,selected.goal,now,true,c->maximum_frontier_step_m);process_action(&backend,mo.action,c,&sent,subgoal);
    if(machine.state==MISSION_EXPLORE&&safety.decision==SAFETY_OK){pose.x_m+=sent.vx_mps*.05f;pose.y_m+=sent.vy_mps*.05f;pose.vx_mps=sent.vx_mps;pose.vy_mps=sent.vy_mps;}else{pose.vx_mps=pose.vy_mps=0.0f;}
    if(mo.state_changed||clamp||safety.decision!=SAFETY_OK)run_logger_event(logger,now,mo.state_changed?"transition":(clamp?"command_clamped":"safety"),mo.reason,machine.state,safety,&sent);
  }
  printf("dry-run final_state=%s explore=%s\n",mission_state_name(machine.state),o->explore?"enabled":"disabled");return machine.state==MISSION_COMPLETE?0:3;
}

static int run_replay(const Options*o,RunLogger*logger,const AutonomyConfig*c){
  if(!o->input){fprintf(stderr,"replay requires --input\n");return 2;}ReplayReader reader;ReplayStatus rs=replay_reader_open(&reader,o->input);if(rs!=REPLAY_OK){fprintf(stderr,"replay rejected: expected explicit SCLOG3 header\n");return 2;}
  static CellState raw[AUTONOMY_MAX_CELLS],inflated_cells[AUTONOMY_MAX_CELLS];MappingAdapter map;bool map_init=false;FrontierWorkspace fw;PlannerWorkspace pw;SafetyMonitor monitor={0};uint64_t decisions=0;Sclog3Record r;
  uint64_t previous_replay_ms=0;
  while((rs=replay_reader_next(&reader,&r))==REPLAY_OK){if(o->speed>0.0f&&previous_replay_ms&&r.host_ms>previous_replay_ms){uint64_t delay_ns=(uint64_t)(((double)(r.host_ms-previous_replay_ms)*1000000.0)/(double)o->speed);struct timespec delay={(time_t)(delay_ns/1000000000u),(long)(delay_ns%1000000000u)};while(nanosleep(&delay,&delay)&&errno==EINTR&&!interrupted){}}previous_replay_ms=r.host_ms;if(!map_init){if(!mapping_adapter_init(&map,raw,128,128,.15f,r.x_m,r.y_m,false)){replay_reader_close(&reader);return 2;}map_init=true;}if(!mapping_adapter_apply_sclog3(&map,&r))continue;
    OccupancyGrid inflated=map.grid;inflated.cells=inflated_cells;if(!occupancy_grid_inflate(&map.grid,&inflated,autonomy_required_clearance_m(c,map.grid.resolution_m)))continue;
    VehiclePose pose={r.x_m,r.y_m,r.alt_m,r.vx_mps,r.vy_mps,r.vz_mps,r.yaw_deg,.05f,r.host_ms,true};FrontierSet fs;LocalPath path;FrontierCandidate best;PlanStatus ps=plan_best(&inflated,&pose,c,r.host_ms,&fw,&pw,&fs,&path,&best,NULL,logger);
    MissionContext ctx={.now_ms=r.host_ms,.heartbeat_timestamp_ms=r.host_ms-r.hb_age_ms,.tof_timestamp_ms=r.host_ms,.range_timestamp_ms=r.host_ms-r.rf_age_ms,.optical_flow_timestamp_ms=r.host_ms-r.of_age_ms,.pose=pose,.map=&inflated,.range_altitude_m=r.rf_m,.distance_from_mission_origin_m=0.0f,.battery_v=8.0f,.nearest_obstacle_m=1.0f,.optical_flow_quality=r.of_q,.heartbeat_valid=r.hb_age_ms<65535,.expected_mode=r.custom_mode==4,.vehicle_state_valid=true,.battery_valid=false,.path_valid=ps==PLAN_OK,.subgoal_valid=ps==PLAN_OK,.command_valid=true,.horizontal_motion_requested=true,.mission_state_valid=true};
    SafetyResult safety=evaluate_safety(&ctx,c,&monitor);run_logger_telemetry(logger,&ctx,MISSION_SELECT_FRONTIER,r.fc_armed!=0,r.custom_mode,ps==PLAN_OK?&path:NULL,NULL);MotionCommand none={.source_timestamp_ms=r.host_ms,.valid=true};run_logger_event(logger,r.host_ms,"replay_decision",ps==PLAN_OK?"safe_path":"no_safe_path",MISSION_SELECT_FRONTIER,safety,&none);decisions++;}
  printf("replay records=%llu decisions=%llu warnings=%llu timestamp_resets=%llu serial_opened=no commands_sent=no\n",(unsigned long long)reader.records,(unsigned long long)decisions,(unsigned long long)reader.warnings,(unsigned long long)reader.timestamp_resets);replay_reader_close(&reader);return decisions?0:3;
}

static uint64_t monotonic_ms(void){struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t)!=0)return 0;return (uint64_t)t.tv_sec*1000u+(uint64_t)t.tv_nsec/1000000u;}
static float closest_tof_m(const TofFrame*f){if(!f||!f->valid)return NAN;uint16_t best=UINT16_MAX;for(size_t s=0;s<TOF_SENSOR_COUNT;s++)for(size_t z=0;z<TOF_ZONES_PER_SENSOR;z++)if(f->range_mm[s][z]<best)best=f->range_mm[s][z];return best==UINT16_MAX?NAN:(float)best*.001f;}

static int run_live(const Options*o,RunLogger*logger,const AutonomyConfig*c){
  VehicleBackend backend;backend_init(&backend,o->kind);
  if(!backend_configure_mavlink(&backend,o->serial,o->baud,o->expected_system,o->expected_component,o->allow_live)||backend_connect(&backend)!=BACKEND_OK){fprintf(stderr,"refused: unable to configure/open MAVLink endpoint\n");return 3;}
  LiveTofAdapter tof;live_tof_adapter_init(&tof);if(!live_tof_adapter_configure(&tof,o->tof_serial,o->tof_baud,o->tof_protocol)||live_tof_adapter_connect(&tof)!=0){fprintf(stderr,"refused: unable to configure/open version-selected production ToF feed\n");backend_close(&backend);return 3;}
  static CellState raw_cells[256u*256u],inflated_cells[256u*256u];static FrontierWorkspace fw;static PlannerWorkspace pw;
  MappingAdapter map;bool map_ready=false;uint32_t applied_tof_sequence=0;OccupancyGrid inflated={0};
  MissionStateMachine machine;uint64_t started=monotonic_ms();mission_state_machine_init(&machine,started);SafetyMonitor monitor={0};CommandSmoother smoother={0};FrontierSet frontiers;LocalPath path={0};FrontierCandidate selected={0};GridPoint subgoal={0};FrontierHistory history={0};ProgressMonitor progress={0};
  bool selected_ok=false,path_ok=false;bool origin_valid=false;GridPoint origin={0};float takeoff_start_altitude=0.0f;bool takeoff_baseline=false;bool restart_land_sent=false;
  fprintf(stderr,"\n*** LIVE/SITL BACKEND ACTIVE: endpoint=%s target=%u/%u; startup is MONITOR-ONLY unless --start-mission was supplied ***\n",o->serial,o->expected_system,o->expected_component);
  for(;;){
    uint64_t now=monotonic_ms();if(!now){interrupted=1;now=started;}
    BackendResult poll_result=backend_poll(&backend,now);int tof_result=live_tof_adapter_poll(&tof,now);BackendStatus bs;backend_get_status(&backend,&bs);
    uint64_t pose_stamp=bs.local_position_ms<bs.attitude_ms?bs.local_position_ms:bs.attitude_ms;
    VehiclePose pose={.x_m=bs.x_m,.y_m=bs.y_m,.z_m=bs.altitude_m,.vx_mps=bs.vx_mps,.vy_mps=bs.vy_mps,.vz_mps=bs.vz_mps,.yaw_deg=bs.yaw_deg,.localization_uncertainty_m=.05f,.timestamp_ms=pose_stamp,.valid=bs.local_position_valid&&bs.attitude_valid};
    const TofFrame*frame=live_tof_adapter_latest(&tof);
    if(!map_ready&&pose.valid&&isfinite(pose.x_m)&&isfinite(pose.y_m)){map_ready=mapping_adapter_init(&map,raw_cells,256,256,.15f,pose.x_m,pose.y_m,false);}
    if(map_ready&&frame&&frame->sequence!=applied_tof_sequence){if(mapping_adapter_apply_live_tof(&map,frame,&pose,(uint64_t)(c->pose_max_age_s*1000.0f)))applied_tof_sequence=frame->sequence;}
    if(map_ready){inflated=map.grid;inflated.cells=inflated_cells;if(!occupancy_grid_inflate(&map.grid,&inflated,autonomy_required_clearance_m(c,map.grid.resolution_m)))inflated.valid=false;}
    if(!o->start_mission){
      if(bs.armed&&!restart_land_sent&&bs.heartbeat){backend_land(&backend);restart_land_sent=true;run_logger_event(logger,now,"restart_recovery","armed_vehicle_observed_land_requested",MISSION_LAND,(SafetyResult){SAFETY_LAND,SAFETY_REASON_BACKEND},NULL);}
      if(interrupted||(o->max_runtime_s>0.0f&&now-started>=(uint64_t)(o->max_runtime_s*1000.0f))){if(bs.armed){backend_land(&backend);struct timespec delay={0,50000000};nanosleep(&delay,NULL);continue;}break;}
      struct timespec delay={0,50000000};nanosleep(&delay,NULL);continue;
    }
    if(machine.state==MISSION_TAKEOFF&&!takeoff_baseline){takeoff_start_altitude=bs.altitude_m;takeoff_baseline=true;}
    if(machine.state==MISSION_SELECT_FRONTIER||machine.state==MISSION_PLAN_PATH||machine.state==MISSION_EXPLORE){
      PlanStatus ps=map_ready&&inflated.valid?plan_best(&inflated,&pose,c,now,&fw,&pw,&frontiers,&path,&selected,&history,logger):PLAN_INVALID_INPUT;selected_ok=ps==PLAN_OK;float max_step=fminf(c->maximum_frontier_step_m,c->max_command_step_m);GridPoint current={pose.x_m,pose.y_m};path_ok=selected_ok&&local_path_next_subgoal(&path,current,max_step,&subgoal)&&local_path_segment_valid(&inflated,current,subgoal);
    }
    MotionCommand requested={.source_timestamp_ms=now,.valid=true};if(machine.state==MISSION_EXPLORE&&path_ok){float dx=subgoal.x_m-pose.x_m,dy=subgoal.y_m-pose.y_m,d=hypotf(dx,dy);if(d>.001f){requested.vx_mps=dx/d*c->max_horizontal_speed_mps;requested.vy_mps=dy/d*c->max_horizontal_speed_mps;}}
    float mission_radius=origin_valid?hypotf(pose.x_m-origin.x_m,pose.y_m-origin.y_m):0.0f;
    uint64_t altitude_stamp=bs.range_valid?bs.range_ms:bs.local_position_ms;float altitude_source=bs.range_valid?bs.range_m:bs.altitude_m;
    MissionContext ctx={.now_ms=now,.heartbeat_timestamp_ms=bs.heartbeat_ms,.tof_timestamp_ms=frame?frame->received_monotonic_ms:0,.range_timestamp_ms=altitude_stamp,.optical_flow_timestamp_ms=bs.optical_flow_ms,.pose=pose,.map=map_ready?&inflated:NULL,.range_altitude_m=altitude_source,.distance_from_mission_origin_m=mission_radius,.battery_v=bs.battery_v,.nearest_obstacle_m=closest_tof_m(frame),.optical_flow_quality=bs.optical_flow_quality,.heartbeat_valid=bs.heartbeat,.expected_mode=bs.mode_safe,.vehicle_state_valid=bs.target_verified&&!bs.operator_reset_required,.battery_valid=bs.battery_valid,.manual_stop=interrupted,.backend_failed=poll_result!=BACKEND_OK||bs.connection_lost||bs.fc_restart_detected||bs.command_rejected||bs.command_timed_out,.path_valid=path_ok,.subgoal_valid=path_ok,.command_valid=requested.valid,.horizontal_motion_requested=machine.state==MISSION_EXPLORE,.mission_state_valid=machine.state<=MISSION_COMPLETE};
    SafetyResult safety=evaluate_safety(&ctx,c,&monitor);bool clamped=false;MotionCommand sent={0};command_smoother_update(&smoother,c,&requested,now,mission_state_allows_horizontal_motion(machine.state)&&safety.decision==SAFETY_OK,machine.state==MISSION_LAND,&sent,&clamped);
    if(machine.state==MISSION_EXPLORE&&path_ok&&(!progress.active||hypotf(progress.target.x_m-subgoal.x_m,progress.target.y_m-subgoal.y_m)>.01f))progress_monitor_start(&progress,(GridPoint){pose.x_m,pose.y_m},subgoal,now);
    ProgressResult progress_result=machine.state==MISSION_EXPLORE?progress_monitor_update(&progress,&pose,now,c):PROGRESS_OK;bool reached=progress_result==PROGRESS_REACHED;bool no_progress=progress_result==PROGRESS_STALLED||progress_result==PROGRESS_DIVERGED||progress_result==PROGRESS_EXCESS_SPEED;
    bool prearm_ready=!bs.prearm_check_available||(bs.prearm_check_healthy&&mavlink_backend_observation_fresh(now,bs.system_status_ms,MAVLINK_OBSERVATION_MAX_AGE_MS));
    bool preflight=now-started>=8000u&&bs.target_verified&&bs.heartbeat&&prearm_ready&&pose.valid&&altitude_stamp>0&&bs.optical_flow_valid&&bs.optical_flow_quality>=c->minimum_optical_flow_quality&&(!o->explore||(map_ready&&frame&&live_tof_adapter_fresh(&tof,now,(uint64_t)(c->tof_max_age_s*1000.0f))));
    MissionInput in={.now_ms=now,.start_requested=o->start_mission,.exploration_enabled=o->explore,.interrupt_requested=interrupted,.preflight_valid=preflight,.heartbeat_fresh=bs.heartbeat,.mode_ack=mavlink_backend_acknowledged(&backend.mavlink,MAV_CMD_DO_SET_MODE),.mode_observed=bs.mode_safe,.arm_ack=mavlink_backend_acknowledged(&backend.mavlink,MAV_CMD_COMPONENT_ARM_DISARM),.armed_observed=bs.armed,.takeoff_ack=mavlink_backend_acknowledged(&backend.mavlink,MAV_CMD_NAV_TAKEOFF),.altitude_increased=takeoff_baseline&&bs.altitude_m>takeoff_start_altitude+.10f,.altitude_at_target=fabsf(bs.altitude_m-c->takeoff_target_altitude_m)<c->takeoff_altitude_tolerance_m,.hover_stable=pose.valid&&fabsf(pose.vx_mps)<c->stable_horizontal_speed_mps&&fabsf(pose.vy_mps)<c->stable_horizontal_speed_mps&&fabsf(pose.vz_mps)<c->stable_vertical_speed_mps,.scan_fresh=map_ready&&frame&&live_tof_adapter_fresh(&tof,now,(uint64_t)(c->tof_max_age_s*1000.0f)),.frontier_available=selected_ok,.path_available=path_ok,.subgoal_reached=reached,.no_progress=no_progress,.landed_observed=bs.landed_valid&&bs.landed,.disarmed_observed=!bs.armed,.safety_decision=safety.decision};
    MissionState before=machine.state;MissionOutput mo=mission_state_machine_tick(&machine,&in,c);
    if(before==MISSION_HOVER_STABILIZE&&machine.state==MISSION_HOVER_SCAN){origin=(GridPoint){pose.x_m,pose.y_m};origin_valid=true;}
    BackendResult action_result=BACKEND_OK;if(mo.action==ACTION_SEND_POSITION_SUBGOAL)action_result=backend_send_velocity(&backend,&sent);else{process_action(&backend,mo.action,c,&sent,subgoal);}
    if(mo.state_changed&&machine.state==MISSION_EXPLORE)frontier_history_record(&history,selected.goal,now,false,c->maximum_frontier_step_m);
    if(no_progress)frontier_history_record(&history,selected.goal,now,true,c->maximum_frontier_step_m);
    run_logger_telemetry(logger,&ctx,machine.state,bs.armed,backend.mavlink.status.custom_mode,path_ok?&path:NULL,path_ok?&subgoal:NULL);if(mo.state_changed||clamped||safety.decision!=SAFETY_OK||action_result>BACKEND_PENDING)run_logger_event(logger,now,mo.state_changed?"transition":(action_result>BACKEND_PENDING?"backend_action_refused":(clamped?"command_clamped":"safety")),mo.reason,machine.state,safety,&sent);
    if(machine.state==MISSION_COMPLETE)break;
    if(o->max_runtime_s>0.0f&&now-started>=(uint64_t)(o->max_runtime_s*1000.0f))interrupted=1;
    if(tof_result<0&&!tof.connected&&machine.state!=MISSION_LAND)run_logger_event(logger,now,"tof_disconnect","hold_then_land",machine.state,(SafetyResult){SAFETY_HOVER,SAFETY_REASON_TOF},NULL);
    struct timespec delay={0,50000000};nanosleep(&delay,NULL);
  }
  live_tof_adapter_close(&tof);backend_close(&backend);return 0;
}

int main(int argc,char**argv){Options o;if(!parse_options(argc,argv,&o)){usage(argv[0]);return 2;}if(!ensure_dir(o.run_dir)){perror("run directory");return 2;}signal(SIGINT,on_signal);signal(SIGTERM,on_signal);AutonomyConfig c=autonomy_config_defaults();if(o.minimum_flow_quality_override>=0)c.minimum_optical_flow_quality=(uint8_t)o.minimum_flow_quality_override;if(!autonomy_config_validate(&c))return 2;RunLogger logger;if(!run_logger_open(&logger,o.run_dir,0)){perror("decision log");return 2;}
  printf("backend=%s exploration=%s (default is fake + disabled)\n",backend_kind_name(o.kind),o.explore?"ENABLED":"disabled");int rc;
  if(o.kind==BACKEND_FAKE)rc=run_fake(&o,&logger,&c);else if(o.kind==BACKEND_REPLAY)rc=run_replay(&o,&logger,&c);else{
    fprintf(stderr,"\n*** LIVE VEHICLE INTERFACE REQUESTED: NO ARMING OR SERIAL ACCESS WILL OCCUR ***\n");
    if(o.kind==BACKEND_MAVLINK&&(!o.allow_live||!o.serial||!o.tof_serial||!o.live_map||o.tof_protocol==TOF_PROTOCOL_NONE)){fprintf(stderr,"refused: mavlink requires --allow-live-serial, --serial, --tof-serial, explicit --tof-protocol legacy-a5-v0, and --map-source live\n");rc=2;}
    else if(o.kind==BACKEND_SITL&&(!o.serial||!o.tof_serial||!o.live_map||o.tof_protocol==TOF_PROTOCOL_NONE)){fprintf(stderr,"refused: SITL requires a real MAVLink endpoint and validated ToF feed\n");rc=2;}
    else if(!mapping_adapter_live_available()){fprintf(stderr,"refused: production map feed unavailable\n");rc=3;}else rc=run_live(&o,&logger,&c);
  }run_logger_close(&logger);return rc;
}
