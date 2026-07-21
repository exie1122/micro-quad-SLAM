#include "vehicle_backend.h"
#include <math.h>
#include <string.h>

static bool non_actuating(const VehicleBackend*b){return b->kind==BACKEND_REPLAY;}
BackendResult backend_connect(VehicleBackend*b){if(!b)return BACKEND_INVALID_STATE;if(b->kind==BACKEND_MAVLINK||b->kind==BACKEND_SITL)return BACKEND_UNAVAILABLE;b->status.connected=true;return BACKEND_OK;}
BackendResult backend_wait_heartbeat(VehicleBackend*b,uint64_t now){if(!b||!b->status.connected)return BACKEND_INVALID_STATE;if(b->kind==BACKEND_FAKE){b->status.heartbeat=true;b->status.heartbeat_ms=now;}return b->status.heartbeat?BACKEND_OK:BACKEND_PENDING;}
BackendResult backend_get_status(VehicleBackend*b,BackendStatus*s){if(!b||!s)return BACKEND_INVALID_STATE;*s=b->status;return BACKEND_OK;}
BackendResult backend_set_safe_mode(VehicleBackend*b){if(!b||non_actuating(b))return BACKEND_REFUSED;if(b->kind!=BACKEND_FAKE)return BACKEND_UNAVAILABLE;b->status.mode_safe=true;return BACKEND_OK;}
BackendResult backend_arm(VehicleBackend*b){if(!b||non_actuating(b))return BACKEND_REFUSED;if(b->kind!=BACKEND_FAKE||!b->status.mode_safe||!b->status.heartbeat)return BACKEND_INVALID_STATE;b->status.armed=true;b->status.landed=true;return BACKEND_OK;}
BackendResult backend_takeoff(VehicleBackend*b,float alt){if(!b||non_actuating(b)||!isfinite(alt)||alt<=0.0f)return BACKEND_REFUSED;if(b->kind!=BACKEND_FAKE||!b->status.armed)return BACKEND_INVALID_STATE;b->status.altitude_m=alt;b->status.landed=false;return BACKEND_OK;}
BackendResult backend_send_velocity(VehicleBackend*b,const MotionCommand*c){if(!b||!c||!c->valid||non_actuating(b))return BACKEND_REFUSED;return b->kind==BACKEND_FAKE&&b->status.armed&&!b->status.landed?BACKEND_OK:BACKEND_INVALID_STATE;}
BackendResult backend_send_position_subgoal(VehicleBackend*b,GridPoint p,float alt){if(!isfinite(p.x_m)||!isfinite(p.y_m)||!isfinite(alt))return BACKEND_REFUSED;MotionCommand c={0};c.valid=true;return backend_send_velocity(b,&c);}
BackendResult backend_hover(VehicleBackend*b){if(!b||non_actuating(b))return BACKEND_REFUSED;return b->kind==BACKEND_FAKE&&b->status.armed?BACKEND_OK:BACKEND_INVALID_STATE;}
BackendResult backend_land(VehicleBackend*b){if(!b||non_actuating(b))return BACKEND_REFUSED;if(b->kind!=BACKEND_FAKE)return BACKEND_UNAVAILABLE;b->status.altitude_m=0.0f;b->status.landed=true;return BACKEND_OK;}
BackendResult backend_disarm_if_landed(VehicleBackend*b){if(!b||non_actuating(b))return BACKEND_REFUSED;if(b->kind!=BACKEND_FAKE||!b->status.landed)return BACKEND_INVALID_STATE;b->status.armed=false;return BACKEND_OK;}
BackendResult backend_close(VehicleBackend*b){if(!b)return BACKEND_INVALID_STATE;b->status.connected=false;return BACKEND_OK;}
const char *backend_kind_name(BackendKind k){static const char*n[]={"fake","replay","sitl","mavlink"};return k>=BACKEND_FAKE&&k<=BACKEND_MAVLINK?n[k]:"unknown";}
