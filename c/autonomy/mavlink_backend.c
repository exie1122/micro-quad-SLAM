#include "mavlink_backend.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/socket.h>

#define GUIDED_CUSTOM_MODE 4u
#define READ_BUDGET_BYTES 4096u
#define GROUND_ALTITUDE_LIMIT_M 0.20f

static speed_t baud_flag(int baud) {
  switch (baud) {
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return (speed_t)0;
  }
}

static bool finite3(float a,float b,float c) {
  return isfinite(a)&&isfinite(b)&&isfinite(c);
}

static void mark_lost(MavlinkBackend *b) {
  b->connection_state=MAV_CONN_LOST;
  b->status.connected=false;
  b->status.connection_lost=true;
  b->status.operator_reset_required=true;
  b->pending.active=false;
}

static int enqueue_message(MavlinkBackend *b,const mavlink_message_t *message) {
  if (!b||!message||b->tx_count>=MAVLINK_TX_QUEUE_CAPACITY) {
    if (b) b->status.tx_drops++;
    return -1;
  }
  MavlinkTxFrame *frame=&b->tx_queue[b->tx_tail];
  frame->length=mavlink_msg_to_send_buffer(frame->bytes,message);
  frame->offset=0u;
  b->tx_tail=(uint8_t)((b->tx_tail+1u)%MAVLINK_TX_QUEUE_CAPACITY);
  b->tx_count++;
  return 0;
}

static int flush_tx(MavlinkBackend *b) {
  while (b->tx_count>0u) {
    MavlinkTxFrame *frame=&b->tx_queue[b->tx_head];
    size_t remaining=(size_t)(frame->length-frame->offset);
    ssize_t n=write(b->fd,frame->bytes+frame->offset,remaining);
    if (n>0) {
      frame->offset=(uint16_t)(frame->offset+(uint16_t)n);
      if (frame->offset==frame->length) {
        b->tx_head=(uint8_t)((b->tx_head+1u)%MAVLINK_TX_QUEUE_CAPACITY);
        b->tx_count--;
      }
      continue;
    }
    if (n<0&&(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR)) return 0;
    mark_lost(b);
    return -1;
  }
  return 0;
}

static bool from_expected_target(const MavlinkBackend *b,const mavlink_message_t *m) {
  return m->sysid==b->expected_system&&m->compid==b->expected_component;
}

static void handle_ack(MavlinkBackend *b,const mavlink_message_t *m,uint64_t now_ms) {
  mavlink_command_ack_t ack;
  mavlink_msg_command_ack_decode(m,&ack);
  if (!from_expected_target(b,m)||
      (ack.target_system!=0u&&ack.target_system!=b->own_system)||
      (ack.target_component!=0u&&ack.target_component!=b->own_component)) {
    b->status.wrong_acks++;
    return;
  }
  b->status.last_ack_command=ack.command;
  b->status.last_ack_result=ack.result;
  if (!b->pending.active) {
    if (ack.command==MAV_CMD_SET_MESSAGE_INTERVAL) return;
    if (ack.command==b->last_completed_command) b->status.duplicate_acks++;
    else if (ack.command==b->last_timed_out_command) b->status.late_acks++;
    else b->status.wrong_acks++;
    return;
  }
  if (ack.command!=b->pending.command) {
    b->status.wrong_acks++;
    return;
  }
  if (ack.result==MAV_RESULT_IN_PROGRESS) {
    b->pending.deadline_ms=now_ms+MAVLINK_COMMAND_TIMEOUT_MS;
    return;
  }
  b->pending.active=false;
  b->last_completed_command=ack.command;
  b->last_completed_param1=b->pending.param1;
  b->last_completed_ms=now_ms;
  if (ack.result==MAV_RESULT_ACCEPTED) {
    b->status.accepted_acks++;
    b->status.command_rejected=false;
  } else {
    b->status.command_rejected=true;
  }
}

static void handle_message(MavlinkBackend *b,const mavlink_message_t *m,uint64_t now_ms) {
  if (m->msgid==MAVLINK_MSG_ID_HEARTBEAT) {
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(m,&hb);
    if (!from_expected_target(b,m)||hb.autopilot!=MAV_AUTOPILOT_ARDUPILOTMEGA) return;
    b->status.target_verified=true;
    b->status.heartbeat_ms=now_ms;
    b->status.custom_mode=hb.custom_mode;
    b->status.mode_safe=hb.custom_mode==GUIDED_CUSTOM_MODE;
    b->status.armed=(hb.base_mode&MAV_MODE_FLAG_SAFETY_ARMED)!=0u;
    b->connection_state=MAV_CONN_READY;
    return;
  }
  if (!b->status.target_verified||!from_expected_target(b,m)) return;
  switch (m->msgid) {
    case MAVLINK_MSG_ID_COMMAND_ACK:
      handle_ack(b,m,now_ms);
      break;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
      mavlink_local_position_ned_t p;
      mavlink_msg_local_position_ned_decode(m,&p);
      if (!finite3(p.x,p.y,p.z)||!finite3(p.vx,p.vy,p.vz)) {
        b->status.malformed_messages++;
        break;
      }
      if (b->have_fc_boot_ms&&p.time_boot_ms<b->last_fc_boot_ms&&
          b->last_fc_boot_ms-p.time_boot_ms>1000u) {
        b->status.fc_restart_detected=true;
        b->status.operator_reset_required=true;
        b->status.fc_restarts++;
        b->pending.active=false;
      }
      b->last_fc_boot_ms=p.time_boot_ms;
      b->have_fc_boot_ms=true;
      b->status.x_m=p.x;b->status.y_m=p.y;b->status.altitude_m=fmaxf(-p.z,0.0f);
      b->status.vx_mps=p.vx;b->status.vy_mps=p.vy;b->status.vz_mps=p.vz;
      b->status.local_position_ms=now_ms;
      b->status.local_position_valid=true;
      break;
    }
    case MAVLINK_MSG_ID_ATTITUDE: {
      mavlink_attitude_t attitude;
      mavlink_msg_attitude_decode(m,&attitude);
      if (!isfinite(attitude.yaw)) {b->status.malformed_messages++;break;}
      b->status.yaw_deg=attitude.yaw*57.29577951308232f;
      b->status.attitude_ms=now_ms;b->status.attitude_valid=true;
      break;
    }
    case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
      mavlink_distance_sensor_t d;
      mavlink_msg_distance_sensor_decode(m,&d);
      if (d.current_distance>=d.min_distance&&d.current_distance<=d.max_distance&&d.current_distance>0u) {
        b->status.range_m=(float)d.current_distance/100.0f;
        b->status.range_ms=now_ms;b->status.range_valid=true;
      }
      break;
    }
    case MAVLINK_MSG_ID_RANGEFINDER: {
      mavlink_rangefinder_t r;
      mavlink_msg_rangefinder_decode(m,&r);
      if (isfinite(r.distance)&&r.distance>=0.0f) {
        b->status.range_m=r.distance;b->status.range_ms=now_ms;b->status.range_valid=true;
      }
      break;
    }
    case MAVLINK_MSG_ID_SYS_STATUS: {
      mavlink_sys_status_t s;
      mavlink_msg_sys_status_decode(m,&s);
      b->status.system_status_ms=now_ms;
      b->status.prearm_check_available=
        (s.onboard_control_sensors_present&MAV_SYS_STATUS_PREARM_CHECK)!=0u||
        (s.onboard_control_sensors_enabled&MAV_SYS_STATUS_PREARM_CHECK)!=0u;
      b->status.prearm_check_healthy=b->status.prearm_check_available&&
        (s.onboard_control_sensors_health&MAV_SYS_STATUS_PREARM_CHECK)!=0u;
      if (s.voltage_battery!=UINT16_MAX&&s.voltage_battery>0u) {
        b->status.battery_v=(float)s.voltage_battery/1000.0f;
        b->status.battery_ms=now_ms;b->status.battery_valid=true;
      }
      break;
    }
    case MAVLINK_MSG_ID_BATTERY_STATUS: {
      mavlink_battery_status_t s;
      mavlink_msg_battery_status_decode(m,&s);
      uint32_t millivolts=0u;
      for (size_t i=0;i<10u&&s.voltages[i]!=UINT16_MAX;i++) millivolts+=s.voltages[i];
      if (millivolts>0u) {
        b->status.battery_v=(float)millivolts/1000.0f;
        b->status.battery_ms=now_ms;b->status.battery_valid=true;
      }
      break;
    }
    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
      mavlink_extended_sys_state_t s;
      mavlink_msg_extended_sys_state_decode(m,&s);
      b->status.landed=s.landed_state==MAV_LANDED_STATE_ON_GROUND;
      b->status.landed_valid=s.landed_state!=MAV_LANDED_STATE_UNDEFINED;
      b->status.landed_ms=now_ms;
      break;
    }
    case MAVLINK_MSG_ID_OPTICAL_FLOW: {
      mavlink_optical_flow_t f;
      mavlink_msg_optical_flow_decode(m,&f);
      b->status.optical_flow_quality=f.quality;
      b->status.optical_flow_ms=now_ms;b->status.optical_flow_valid=true;
      break;
    }
    case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD: {
      mavlink_optical_flow_rad_t f;
      mavlink_msg_optical_flow_rad_decode(m,&f);
      b->status.optical_flow_quality=f.quality;
      b->status.optical_flow_ms=now_ms;b->status.optical_flow_valid=true;
      break;
    }
    default: break;
  }
}

static int request_streams(MavlinkBackend *b) {
  static const uint8_t streams[]={MAV_DATA_STREAM_EXTENDED_STATUS,MAV_DATA_STREAM_POSITION,MAV_DATA_STREAM_EXTRA1,MAV_DATA_STREAM_EXTRA2,MAV_DATA_STREAM_EXTRA3};
  for (size_t i=0;i<sizeof(streams)/sizeof(streams[0]);i++) {
    mavlink_message_t m;
    mavlink_msg_request_data_stream_pack(b->own_system,b->own_component,&m,
      b->expected_system,b->expected_component,streams[i],10u,1u);
    if (enqueue_message(b,&m)!=0) return -1;
  }
  static const struct {uint32_t message_id;uint32_t interval_us;} intervals[]={
    {MAVLINK_MSG_ID_HEARTBEAT,1000000u},{MAVLINK_MSG_ID_LOCAL_POSITION_NED,50000u},
    {MAVLINK_MSG_ID_ATTITUDE,50000u},{MAVLINK_MSG_ID_DISTANCE_SENSOR,50000u},
    {MAVLINK_MSG_ID_SYS_STATUS,500000u},{MAVLINK_MSG_ID_EXTENDED_SYS_STATE,200000u},
    {MAVLINK_MSG_ID_OPTICAL_FLOW,50000u}
  };
  for(size_t i=0;i<sizeof(intervals)/sizeof(intervals[0]);i++){
    mavlink_message_t m;
    mavlink_msg_command_long_pack(b->own_system,b->own_component,&m,b->expected_system,b->expected_component,
      MAV_CMD_SET_MESSAGE_INTERVAL,0u,(float)intervals[i].message_id,(float)intervals[i].interval_us,0,0,0,0,0);
    if(enqueue_message(b,&m)!=0)return -1;
  }
  b->stream_requests_sent=true;
  return 0;
}

void mavlink_backend_init(MavlinkBackend *b) {
  if (!b) return;
  memset(b,0,sizeof(*b));
  b->fd=-1;b->baud=115200;b->own_system=245u;b->own_component=MAV_COMP_ID_ONBOARD_COMPUTER;
  b->expected_system=1u;b->expected_component=MAV_COMP_ID_AUTOPILOT1;
  b->connection_state=MAV_CONN_CLOSED;b->next_correlation_id=1u;
}

bool mavlink_backend_configure(MavlinkBackend *b,const char *endpoint,int baud,uint8_t system,uint8_t component) {
  if (!b||!endpoint||endpoint[0]=='\0'||strlen(endpoint)>=sizeof(b->endpoint)||system==0u||component==0u) return false;
  if (strncmp(endpoint,"udp:",4)!=0&&strncmp(endpoint,"udpin:",6)!=0&&baud_flag(baud)==(speed_t)0) return false;
  memcpy(b->endpoint,endpoint,strlen(endpoint)+1u);b->baud=baud;
  b->expected_system=system;b->expected_component=component;
  return true;
}

int mavlink_backend_attach_fd(MavlinkBackend *b,int fd,bool take_ownership) {
  if (!b||fd<0||b->fd>=0) return -1;
  int flags=fcntl(fd,F_GETFL,0);
  if (flags<0||fcntl(fd,F_SETFL,flags|O_NONBLOCK)<0) return -1;
  b->fd=fd;b->owns_fd=take_ownership;b->status.connected=true;
  b->connection_state=MAV_CONN_MONITORING;
  return 0;
}

static int connect_udp(MavlinkBackend *b,bool bind_input) {
  char spec[160],*colon;
  const char *address=b->endpoint+(bind_input?6u:4u);
  memcpy(spec,address,strlen(address)+1u);
  colon=strrchr(spec,':');
  if (!colon||colon==spec||colon[1]=='\0') return -1;
  *colon='\0';
  char *end=NULL;long port=strtol(colon+1u,&end,10);
  if (!end||*end!='\0'||port<1||port>65535) return -1;
  struct sockaddr_in address4;memset(&address4,0,sizeof(address4));address4.sin_family=AF_INET;address4.sin_port=htons((uint16_t)port);
  if (inet_pton(AF_INET,spec,&address4.sin_addr)!=1) return -1;
  int fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK,0);
  int socket_result=fd<0?-1:(bind_input?bind(fd,(struct sockaddr*)&address4,sizeof(address4)):connect(fd,(struct sockaddr*)&address4,sizeof(address4)));
  if (fd>=0&&socket_result!=0) {close(fd);fd=-1;}
  if (fd<0) return -1;
  b->is_datagram=true;
  b->datagram_peer_known=!bind_input;
  return mavlink_backend_attach_fd(b,fd,true);
}

int mavlink_backend_connect(MavlinkBackend *b) {
  if (!b||b->fd>=0||b->endpoint[0]=='\0'||b->status.operator_reset_required) return -1;
  b->connection_state=MAV_CONN_CONNECTING;
  if (strncmp(b->endpoint,"udp:",4)==0) return connect_udp(b,false);
  if (strncmp(b->endpoint,"udpin:",6)==0) return connect_udp(b,true);
  speed_t speed=baud_flag(b->baud);
  int fd=open(b->endpoint,O_RDWR|O_NOCTTY|O_NONBLOCK);
  if (fd<0) return -1;
  struct termios tio;
  if (tcgetattr(fd,&tio)<0) {close(fd);return -1;}
  cfmakeraw(&tio);cfsetispeed(&tio,speed);cfsetospeed(&tio,speed);
  tio.c_cflag|=CLOCAL|CREAD;tio.c_cflag&=~CSTOPB;tio.c_cflag&=~CRTSCTS;
  if (tcsetattr(fd,TCSANOW,&tio)<0) {close(fd);return -1;}
  return mavlink_backend_attach_fd(b,fd,true);
}

static void command_timeout(MavlinkBackend *b,uint64_t now_ms) {
  if (!b->pending.active||now_ms<b->pending.deadline_ms) return;
  if (b->pending.retries<MAVLINK_COMMAND_RETRY_LIMIT) {
    if (enqueue_message(b,&b->pending.message)==0) {
      b->pending.retries++;b->pending.sent_ms=now_ms;
      b->pending.deadline_ms=now_ms+MAVLINK_COMMAND_TIMEOUT_MS;
      return;
    }
  }
  b->last_timed_out_command=b->pending.command;b->last_timeout_ms=now_ms;
  b->pending.active=false;b->status.command_timed_out=true;
}

int mavlink_backend_poll(MavlinkBackend *b,uint64_t now_ms) {
  if (!b||b->fd<0||b->connection_state==MAV_CONN_LOST||now_ms<b->last_poll_ms) return -1;
  b->last_poll_ms=now_ms;
  uint8_t bytes[512];size_t total=0u;
  for (;;) {
    ssize_t n;
    if (b->is_datagram&&!b->datagram_peer_known) {
      b->datagram_peer_length=sizeof(b->datagram_peer);
      n=recvfrom(b->fd,bytes,sizeof(bytes),0,(struct sockaddr*)&b->datagram_peer,&b->datagram_peer_length);
      if (n>0&&connect(b->fd,(struct sockaddr*)&b->datagram_peer,b->datagram_peer_length)==0)b->datagram_peer_known=true;
    } else n=read(b->fd,bytes,sizeof(bytes));
    if (n>0) {
      total+=(size_t)n;
      for (ssize_t i=0;i<n;i++) {
        uint8_t previous_errors=b->parser_status.parse_error;
        uint16_t previous_drops=b->parser_status.packet_rx_drop_count;
        if (mavlink_parse_char(MAVLINK_COMM_0,bytes[i],&b->parser_message,&b->parser_status)) handle_message(b,&b->parser_message,now_ms);
        b->status.malformed_messages+=(uint8_t)(b->parser_status.parse_error-previous_errors);
        b->status.malformed_messages+=(uint16_t)(b->parser_status.packet_rx_drop_count-previous_drops);
      }
      if (total>=READ_BUDGET_BYTES) break;
      continue;
    }
    if (n==0&&b->is_datagram) break;
    if (n==0) {mark_lost(b);return -1;}
    if (errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR) break;
    mark_lost(b);return -1;
  }
  b->status.heartbeat_fresh=b->status.target_verified&&mavlink_backend_observation_fresh(now_ms,b->status.heartbeat_ms,MAVLINK_HEARTBEAT_MAX_AGE_MS);
  if (b->status.target_verified&&!b->stream_requests_sent&&request_streams(b)!=0) return -1;
  command_timeout(b,now_ms);
  return flush_tx(b);
}

void mavlink_backend_close(MavlinkBackend *b) {
  if (!b) return;
  if (b->fd>=0&&b->owns_fd) close(b->fd);
  b->fd=-1;b->owns_fd=false;b->status.connected=false;b->pending.active=false;
  if (b->connection_state!=MAV_CONN_LOST) b->connection_state=MAV_CONN_CLOSED;
}

static int queue_command(MavlinkBackend *b,uint16_t command,float p1,float p2,float p3,float p4,float p5,float p6,float p7,uint64_t now_ms) {
  if (!b||b->fd<0||!b->status.target_verified||!b->status.heartbeat_fresh||b->pending.active||b->status.operator_reset_required) return -1;
  /* COMMAND_LONG has no transaction ID. Do not create a second transaction while
     the FC may still be applying an already accepted command. */
  if (b->last_completed_command==command&&b->last_completed_param1==p1&&b->status.last_ack_result==MAV_RESULT_ACCEPTED&&
      now_ms>=b->last_completed_ms&&now_ms-b->last_completed_ms<15000u) return -1;
  mavlink_message_t m;
  mavlink_msg_command_long_pack(b->own_system,b->own_component,&m,b->expected_system,b->expected_component,
    command,0u,p1,p2,p3,p4,p5,p6,p7);
  if (enqueue_message(b,&m)!=0) return -1;
  b->pending.active=true;b->pending.command=command;b->pending.correlation_id=b->next_correlation_id++;
  b->pending.sent_ms=now_ms;b->pending.deadline_ms=now_ms+MAVLINK_COMMAND_TIMEOUT_MS;
  b->pending.retries=0u;b->pending.param1=p1;b->pending.message=m;b->status.command_timed_out=false;b->status.command_rejected=false;
  return 0;
}

int mavlink_backend_request_mode(MavlinkBackend *b,uint32_t mode,uint64_t now_ms) {
  return queue_command(b,MAV_CMD_DO_SET_MODE,(float)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,(float)mode,0,0,0,0,0,now_ms);
}
int mavlink_backend_request_arm(MavlinkBackend *b,bool arm,uint64_t now_ms) {
  return queue_command(b,MAV_CMD_COMPONENT_ARM_DISARM,arm?1.0f:0.0f,0,0,0,0,0,0,now_ms);
}
int mavlink_backend_request_takeoff(MavlinkBackend *b,float altitude_m,uint64_t now_ms) {
  if (!isfinite(altitude_m)||altitude_m<=0.0f) return -1;
  return queue_command(b,MAV_CMD_NAV_TAKEOFF,0,0,0,NAN,0,0,altitude_m,now_ms);
}
int mavlink_backend_request_land(MavlinkBackend *b,uint64_t now_ms) {
  return queue_command(b,MAV_CMD_NAV_LAND,0,0,0,NAN,0,0,0,now_ms);
}

static int send_setpoint(MavlinkBackend *b,const mavlink_message_t *m) {
  if (!b||!m||b->fd<0||!b->status.target_verified||!b->status.heartbeat_fresh||
      b->status.operator_reset_required||!b->status.mode_safe||!b->status.armed) return -1;
  return enqueue_message(b,m);
}

int mavlink_backend_send_velocity(MavlinkBackend *b,const MotionCommand *c,uint64_t now_ms) {
  if (!b||!c||!c->valid||!finite3(c->vx_mps,c->vy_mps,c->vz_mps)||!isfinite(c->yaw_rate_dps)) return -1;
  mavlink_message_t m;
  const uint16_t mask=7u|448u|1024u;
  const float deg_to_rad=0.01745329251994329577f;
  mavlink_msg_set_position_target_local_ned_pack(b->own_system,b->own_component,&m,(uint32_t)now_ms,
    b->expected_system,b->expected_component,MAV_FRAME_LOCAL_NED,mask,0,0,0,
    c->vx_mps,c->vy_mps,c->vz_mps,0,0,0,0,c->yaw_rate_dps*deg_to_rad);
  return send_setpoint(b,&m);
}

int mavlink_backend_send_position(MavlinkBackend *b,GridPoint p,float altitude_m,uint64_t now_ms) {
  if (!b||!finite3(p.x_m,p.y_m,altitude_m)||altitude_m<0.0f) return -1;
  mavlink_message_t m;const uint16_t mask=56u|448u|1024u|2048u;
  mavlink_msg_set_position_target_local_ned_pack(b->own_system,b->own_component,&m,(uint32_t)now_ms,
    b->expected_system,b->expected_component,MAV_FRAME_LOCAL_NED,mask,p.x_m,p.y_m,-altitude_m,
    0,0,0,0,0,0,0,0);
  return send_setpoint(b,&m);
}

int mavlink_backend_send_hover(MavlinkBackend *b,uint64_t now_ms) {
  MotionCommand c={0};c.valid=true;c.source_timestamp_ms=now_ms;
  return mavlink_backend_send_velocity(b,&c,now_ms);
}

int mavlink_backend_request_disarm_if_landed(MavlinkBackend *b,uint64_t now_ms) {
  if (!b||!b->status.landed_valid||!b->status.landed||
      !mavlink_backend_observation_fresh(now_ms,b->status.landed_ms,MAVLINK_OBSERVATION_MAX_AGE_MS)) return -1;
  bool local_ground=b->status.local_position_valid&&mavlink_backend_observation_fresh(now_ms,b->status.local_position_ms,MAVLINK_OBSERVATION_MAX_AGE_MS)&&b->status.altitude_m<=GROUND_ALTITUDE_LIMIT_M;
  bool range_ground=b->status.range_valid&&mavlink_backend_observation_fresh(now_ms,b->status.range_ms,MAVLINK_OBSERVATION_MAX_AGE_MS)&&b->status.range_m<=GROUND_ALTITUDE_LIMIT_M;
  if (!local_ground&&!range_ground) return -1;
  return mavlink_backend_request_arm(b,false,now_ms);
}

bool mavlink_backend_acknowledged(const MavlinkBackend *b,uint16_t command) {
  return b&&b->last_completed_command==command&&b->status.last_ack_result==MAV_RESULT_ACCEPTED;
}

bool mavlink_backend_observation_fresh(uint64_t now_ms,uint64_t timestamp_ms,uint64_t max_age_ms) {
  return timestamp_ms>0u&&now_ms>=timestamp_ms&&now_ms-timestamp_ms<=max_age_ms;
}
