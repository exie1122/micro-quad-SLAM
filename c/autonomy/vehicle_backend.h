#ifndef VEHICLE_BACKEND_H
#define VEHICLE_BACKEND_H

#include "autonomy_types.h"

typedef enum {BACKEND_FAKE=0,BACKEND_REPLAY,BACKEND_SITL,BACKEND_MAVLINK} BackendKind;
typedef enum {BACKEND_OK=0,BACKEND_PENDING,BACKEND_REFUSED,BACKEND_UNAVAILABLE,BACKEND_IO_ERROR,BACKEND_INVALID_STATE} BackendResult;
typedef struct {bool connected,heartbeat,mode_safe,armed,landed;float altitude_m;uint64_t heartbeat_ms;} BackendStatus;
typedef struct {BackendKind kind;BackendStatus status;bool live_serial_allowed;char serial_device[128];} VehicleBackend;

BackendResult backend_connect(VehicleBackend *backend);
BackendResult backend_wait_heartbeat(VehicleBackend *backend,uint64_t now_ms);
BackendResult backend_get_status(VehicleBackend *backend,BackendStatus *status);
BackendResult backend_set_safe_mode(VehicleBackend *backend);
BackendResult backend_arm(VehicleBackend *backend);
BackendResult backend_takeoff(VehicleBackend *backend,float altitude_m);
BackendResult backend_send_velocity(VehicleBackend *backend,const MotionCommand *command);
BackendResult backend_send_position_subgoal(VehicleBackend *backend,GridPoint subgoal,float altitude_m);
BackendResult backend_hover(VehicleBackend *backend);
BackendResult backend_land(VehicleBackend *backend);
BackendResult backend_disarm_if_landed(VehicleBackend *backend);
BackendResult backend_close(VehicleBackend *backend);
const char *backend_kind_name(BackendKind kind);

#endif
