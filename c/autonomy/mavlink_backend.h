#ifndef MAVLINK_BACKEND_H
#define MAVLINK_BACKEND_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/socket.h>
#include "ardupilotmega/mavlink.h"
#include "autonomy_types.h"

#define MAVLINK_TX_QUEUE_CAPACITY 16u
#define MAVLINK_COMMAND_RETRY_LIMIT 2u
#define MAVLINK_COMMAND_TIMEOUT_MS 1200u
#define MAVLINK_OBSERVATION_MAX_AGE_MS 800u
#define MAVLINK_HEARTBEAT_MAX_AGE_MS 1500u

typedef enum {
  MAV_CONN_CLOSED = 0,
  MAV_CONN_CONNECTING,
  MAV_CONN_MONITORING,
  MAV_CONN_READY,
  MAV_CONN_LOST
} MavlinkConnectionState;

typedef struct {
  uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
  uint16_t length;
  uint16_t offset;
} MavlinkTxFrame;

typedef struct {
  bool active;
  uint16_t command;
  uint32_t correlation_id;
  uint64_t sent_ms;
  uint64_t deadline_ms;
  uint8_t retries;
  float param1;
  mavlink_message_t message;
} MavlinkPendingCommand;

typedef struct {
  bool connected;
  bool heartbeat_fresh;
  bool target_verified;
  bool mode_safe;
  bool armed;
  bool landed;
  bool landed_valid;
  bool local_position_valid;
  bool attitude_valid;
  bool range_valid;
  bool battery_valid;
  bool optical_flow_valid;
  bool prearm_check_available;
  bool prearm_check_healthy;
  bool connection_lost;
  bool fc_restart_detected;
  bool operator_reset_required;
  bool command_timed_out;
  bool command_rejected;
  uint8_t target_system;
  uint8_t target_component;
  uint32_t custom_mode;
  uint16_t last_ack_command;
  uint8_t last_ack_result;
  uint64_t heartbeat_ms;
  uint64_t local_position_ms;
  uint64_t attitude_ms;
  uint64_t range_ms;
  uint64_t battery_ms;
  uint64_t system_status_ms;
  uint64_t landed_ms;
  uint64_t optical_flow_ms;
  float x_m, y_m, altitude_m;
  float vx_mps, vy_mps, vz_mps;
  float yaw_deg;
  float range_m;
  float battery_v;
  uint8_t optical_flow_quality;
  uint32_t accepted_acks;
  uint32_t duplicate_acks;
  uint32_t late_acks;
  uint32_t wrong_acks;
  uint32_t malformed_messages;
  uint32_t reconnects;
  uint32_t fc_restarts;
  uint32_t tx_drops;
} MavlinkLiveStatus;

typedef struct {
  int fd;
  bool owns_fd;
  bool is_datagram;
  bool datagram_peer_known;
  struct sockaddr_storage datagram_peer;
  socklen_t datagram_peer_length;
  char endpoint[160];
  int baud;
  uint8_t own_system;
  uint8_t own_component;
  uint8_t expected_system;
  uint8_t expected_component;
  MavlinkConnectionState connection_state;
  MavlinkLiveStatus status;
  mavlink_status_t parser_status;
  mavlink_message_t parser_message;
  MavlinkTxFrame tx_queue[MAVLINK_TX_QUEUE_CAPACITY];
  uint8_t tx_head, tx_tail, tx_count;
  MavlinkPendingCommand pending;
  uint32_t next_correlation_id;
  uint16_t last_completed_command;
  float last_completed_param1;
  uint16_t last_timed_out_command;
  uint64_t last_completed_ms;
  uint64_t last_timeout_ms;
  uint64_t last_poll_ms;
  uint32_t last_fc_boot_ms;
  bool have_fc_boot_ms;
  bool stream_requests_sent;
} MavlinkBackend;

void mavlink_backend_init(MavlinkBackend *backend);
bool mavlink_backend_configure(MavlinkBackend *backend,const char *endpoint,int baud,
                               uint8_t expected_system,uint8_t expected_component);
int mavlink_backend_attach_fd(MavlinkBackend *backend,int fd,bool take_ownership);
int mavlink_backend_connect(MavlinkBackend *backend);
int mavlink_backend_poll(MavlinkBackend *backend,uint64_t now_ms);
void mavlink_backend_close(MavlinkBackend *backend);
int mavlink_backend_request_mode(MavlinkBackend *backend,uint32_t custom_mode,uint64_t now_ms);
int mavlink_backend_request_arm(MavlinkBackend *backend,bool arm,uint64_t now_ms);
int mavlink_backend_request_takeoff(MavlinkBackend *backend,float altitude_m,uint64_t now_ms);
int mavlink_backend_request_land(MavlinkBackend *backend,uint64_t now_ms);
int mavlink_backend_send_velocity(MavlinkBackend *backend,const MotionCommand *command,
                                  uint64_t now_ms);
int mavlink_backend_send_position(MavlinkBackend *backend,GridPoint subgoal,float altitude_m,
                                  uint64_t now_ms);
int mavlink_backend_send_hover(MavlinkBackend *backend,uint64_t now_ms);
int mavlink_backend_request_disarm_if_landed(MavlinkBackend *backend,uint64_t now_ms);
bool mavlink_backend_acknowledged(const MavlinkBackend *backend,uint16_t command);
bool mavlink_backend_observation_fresh(uint64_t now_ms,uint64_t timestamp_ms,uint64_t max_age_ms);

#endif
