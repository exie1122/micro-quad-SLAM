#ifndef VEHICLE_BACKEND_H
#define VEHICLE_BACKEND_H

#include "autonomy_types.h"
#include "mavlink_backend.h"

typedef enum {BACKEND_FAKE=0,BACKEND_REPLAY,BACKEND_SITL,BACKEND_MAVLINK} BackendKind;
typedef enum {BACKEND_OK=0,BACKEND_PENDING,BACKEND_REFUSED,BACKEND_UNAVAILABLE,BACKEND_IO_ERROR,BACKEND_INVALID_STATE} BackendResult;
typedef struct {
  bool connected,heartbeat,target_verified,mode_safe,armed,landed,landed_valid;
  bool local_position_valid,attitude_valid,range_valid,battery_valid,optical_flow_valid;
  bool prearm_check_available,prearm_check_healthy;
  bool command_rejected,command_timed_out,connection_lost,fc_restart_detected,operator_reset_required;
  float x_m,y_m,altitude_m,vx_mps,vy_mps,vz_mps,yaw_deg,range_m,battery_v;
  uint8_t optical_flow_quality;
  uint64_t heartbeat_ms,local_position_ms,attitude_ms,range_ms,battery_ms,system_status_ms,landed_ms,optical_flow_ms;
  uint16_t last_ack_command;
  uint8_t last_ack_result;
} BackendStatus;

typedef struct {
  BackendKind kind;
  BackendStatus status;
  bool live_serial_allowed;
  bool configured;
  char serial_device[160];
  uint64_t now_ms;
  MavlinkBackend mavlink;
} VehicleBackend;

void backend_init(VehicleBackend *backend,BackendKind kind);
bool backend_configure_mavlink(VehicleBackend *backend,const char *endpoint,int baud,
                               uint8_t expected_system,uint8_t expected_component,
                               bool allow_live_serial);
int backend_attach_mavlink_fd_for_test(VehicleBackend *backend,int fd,bool take_ownership);
BackendResult backend_connect(VehicleBackend *backend);
BackendResult backend_poll(VehicleBackend *backend,uint64_t now_ms);
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
