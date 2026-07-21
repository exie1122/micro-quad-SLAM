#include "vehicle_backend.h"
#include <math.h>
#include <string.h>

static bool non_actuating(const VehicleBackend*b){return b->kind==BACKEND_REPLAY;}
static bool real_backend(const VehicleBackend*b){return b->kind==BACKEND_MAVLINK||b->kind==BACKEND_SITL;}
static BackendResult real_result(int result){return result==0?BACKEND_PENDING:BACKEND_INVALID_STATE;}

void backend_init(VehicleBackend*b,BackendKind kind){if(!b)return;memset(b,0,sizeof(*b));b->kind=kind;mavlink_backend_init(&b->mavlink);}

bool backend_configure_mavlink(VehicleBackend*b,const char*endpoint,int baud,uint8_t system,uint8_t component,bool allow_live){
  if(!b||!real_backend(b)||(b->kind==BACKEND_MAVLINK&&!allow_live)||!mavlink_backend_configure(&b->mavlink,endpoint,baud,system,component))return false;
  if(strlen(endpoint)>=sizeof(b->serial_device))return false;
  memcpy(b->serial_device,endpoint,strlen(endpoint)+1u);b->live_serial_allowed=allow_live;b->configured=true;return true;
}

int backend_attach_mavlink_fd_for_test(VehicleBackend*b,int fd,bool own){if(!b||!real_backend(b)||!b->configured)return -1;return mavlink_backend_attach_fd(&b->mavlink,fd,own);}

static void copy_status(VehicleBackend*b){
  MavlinkLiveStatus*s=&b->mavlink.status;BackendStatus*d=&b->status;
  d->connected=s->connected;d->heartbeat=s->heartbeat_fresh;d->target_verified=s->target_verified;d->mode_safe=s->mode_safe;d->armed=s->armed;d->landed=s->landed;d->landed_valid=s->landed_valid;
  d->local_position_valid=s->local_position_valid;d->attitude_valid=s->attitude_valid;d->range_valid=s->range_valid;d->battery_valid=s->battery_valid;d->optical_flow_valid=s->optical_flow_valid;d->prearm_check_available=s->prearm_check_available;d->prearm_check_healthy=s->prearm_check_healthy;
  d->command_rejected=s->command_rejected;d->command_timed_out=s->command_timed_out;d->connection_lost=s->connection_lost;d->fc_restart_detected=s->fc_restart_detected;d->operator_reset_required=s->operator_reset_required;
  d->x_m=s->x_m;d->y_m=s->y_m;d->altitude_m=s->altitude_m;d->vx_mps=s->vx_mps;d->vy_mps=s->vy_mps;d->vz_mps=s->vz_mps;d->yaw_deg=s->yaw_deg;d->range_m=s->range_m;d->battery_v=s->battery_v;d->optical_flow_quality=s->optical_flow_quality;
  d->heartbeat_ms=s->heartbeat_ms;d->local_position_ms=s->local_position_ms;d->attitude_ms=s->attitude_ms;d->range_ms=s->range_ms;d->battery_ms=s->battery_ms;d->system_status_ms=s->system_status_ms;d->landed_ms=s->landed_ms;d->optical_flow_ms=s->optical_flow_ms;d->last_ack_command=s->last_ack_command;d->last_ack_result=s->last_ack_result;
}

BackendResult backend_connect(VehicleBackend*b){if(!b)return BACKEND_INVALID_STATE;if(real_backend(b)){if(!b->configured)return BACKEND_REFUSED;if(mavlink_backend_connect(&b->mavlink)!=0)return BACKEND_IO_ERROR;copy_status(b);return BACKEND_OK;}b->status.connected=true;return BACKEND_OK;}
BackendResult backend_poll(VehicleBackend*b,uint64_t now){if(!b)return BACKEND_INVALID_STATE;b->now_ms=now;if(!real_backend(b))return BACKEND_OK;if(mavlink_backend_poll(&b->mavlink,now)!=0){copy_status(b);return BACKEND_IO_ERROR;}copy_status(b);return BACKEND_OK;}
BackendResult backend_wait_heartbeat(VehicleBackend*b,uint64_t now){if(!b)return BACKEND_INVALID_STATE;b->now_ms=now;if(real_backend(b)){BackendResult r=backend_poll(b,now);if(r!=BACKEND_OK)return r;}else if(!b->status.connected)return BACKEND_INVALID_STATE;if(b->kind==BACKEND_FAKE){b->status.heartbeat=true;b->status.heartbeat_ms=now;}return b->status.heartbeat?BACKEND_OK:BACKEND_PENDING;}
BackendResult backend_get_status(VehicleBackend*b,BackendStatus*s){if(!b||!s)return BACKEND_INVALID_STATE;if(real_backend(b))copy_status(b);*s=b->status;return BACKEND_OK;}
BackendResult backend_set_safe_mode(VehicleBackend*b){if(!b||non_actuating(b))return BACKEND_REFUSED;if(real_backend(b))return real_result(mavlink_backend_request_mode(&b->mavlink,4u,b->now_ms));b->status.mode_safe=true;return BACKEND_OK;}
BackendResult backend_arm(VehicleBackend*b){if(!b||non_actuating(b))return BACKEND_REFUSED;if(real_backend(b))return real_result(mavlink_backend_request_arm(&b->mavlink,true,b->now_ms));if(b->kind!=BACKEND_FAKE||!b->status.mode_safe||!b->status.heartbeat)return BACKEND_INVALID_STATE;b->status.armed=true;b->status.landed=true;return BACKEND_OK;}
BackendResult backend_takeoff(VehicleBackend*b,float alt){if(!b||non_actuating(b)||!isfinite(alt)||alt<=0.0f)return BACKEND_REFUSED;if(real_backend(b))return real_result(mavlink_backend_request_takeoff(&b->mavlink,alt,b->now_ms));if(b->kind!=BACKEND_FAKE||!b->status.armed)return BACKEND_INVALID_STATE;b->status.altitude_m=alt;b->status.landed=false;return BACKEND_OK;}
BackendResult backend_send_velocity(VehicleBackend*b,const MotionCommand*c){if(!b||!c||!c->valid||non_actuating(b))return BACKEND_REFUSED;if(real_backend(b))return mavlink_backend_send_velocity(&b->mavlink,c,b->now_ms)==0?BACKEND_OK:BACKEND_INVALID_STATE;return b->kind==BACKEND_FAKE&&b->status.armed&&!b->status.landed?BACKEND_OK:BACKEND_INVALID_STATE;}
BackendResult backend_send_position_subgoal(VehicleBackend*b,GridPoint p,float alt){if(!b||!isfinite(p.x_m)||!isfinite(p.y_m)||!isfinite(alt)||non_actuating(b))return BACKEND_REFUSED;if(real_backend(b))return mavlink_backend_send_position(&b->mavlink,p,alt,b->now_ms)==0?BACKEND_OK:BACKEND_INVALID_STATE;MotionCommand c={0};c.valid=true;return backend_send_velocity(b,&c);}
BackendResult backend_hover(VehicleBackend*b){if(!b||non_actuating(b))return BACKEND_REFUSED;if(real_backend(b))return mavlink_backend_send_hover(&b->mavlink,b->now_ms)==0?BACKEND_OK:BACKEND_INVALID_STATE;return b->kind==BACKEND_FAKE&&b->status.armed?BACKEND_OK:BACKEND_INVALID_STATE;}
BackendResult backend_land(VehicleBackend*b){if(!b||non_actuating(b))return BACKEND_REFUSED;if(real_backend(b))return real_result(mavlink_backend_request_land(&b->mavlink,b->now_ms));if(b->kind!=BACKEND_FAKE)return BACKEND_UNAVAILABLE;b->status.altitude_m=0.0f;b->status.landed=true;return BACKEND_OK;}
BackendResult backend_disarm_if_landed(VehicleBackend*b){if(!b||non_actuating(b))return BACKEND_REFUSED;if(real_backend(b))return real_result(mavlink_backend_request_disarm_if_landed(&b->mavlink,b->now_ms));if(b->kind!=BACKEND_FAKE||!b->status.landed)return BACKEND_INVALID_STATE;b->status.armed=false;return BACKEND_OK;}
BackendResult backend_close(VehicleBackend*b){if(!b)return BACKEND_INVALID_STATE;if(real_backend(b))mavlink_backend_close(&b->mavlink);b->status.connected=false;return BACKEND_OK;}
const char *backend_kind_name(BackendKind k){static const char*n[]={"fake","replay","sitl","mavlink"};return k>=BACKEND_FAKE&&k<=BACKEND_MAVLINK?n[k]:"unknown";}
