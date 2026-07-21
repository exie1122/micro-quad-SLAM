#include "live_tof_adapter.h"
#include "mapping_adapter.h"
#include "mavlink_backend.h"
#include <fcntl.h>
#include <pty.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static int failures=0,checks=0;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x);failures++;}}while(0)

static void raw_pty(int *master,int *slave) {
  CHECK(openpty(master,slave,NULL,NULL,NULL)==0);
  struct termios t;CHECK(tcgetattr(*slave,&t)==0);cfmakeraw(&t);CHECK(tcsetattr(*slave,TCSANOW,&t)==0);
  int flags=fcntl(*master,F_GETFL,0);CHECK(flags>=0&&fcntl(*master,F_SETFL,flags|O_NONBLOCK)==0);
}

static void send_message(int fd,const mavlink_message_t *message) {
  uint8_t data[MAVLINK_MAX_PACKET_LEN];uint16_t length=mavlink_msg_to_send_buffer(data,message);
  CHECK(write(fd,data,length)==length);
}

static void send_heartbeat(int fd,uint8_t sysid,uint8_t compid,uint32_t mode,bool armed) {
  mavlink_message_t m;mavlink_msg_heartbeat_pack(sysid,compid,&m,MAV_TYPE_QUADROTOR,
    MAV_AUTOPILOT_ARDUPILOTMEGA,(uint8_t)(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|(armed?MAV_MODE_FLAG_SAFETY_ARMED:0u)),mode,MAV_STATE_ACTIVE);
  send_message(fd,&m);
}

static void send_ack(int fd,uint16_t command,uint8_t result) {
  mavlink_message_t m;mavlink_msg_command_ack_pack(1,MAV_COMP_ID_AUTOPILOT1,&m,command,result,UINT8_MAX,0,245,MAV_COMP_ID_ONBOARD_COMPUTER);send_message(fd,&m);
}

static void send_position(int fd,uint32_t boot_ms,float altitude_m) {
  mavlink_message_t m;mavlink_msg_local_position_ned_pack(1,MAV_COMP_ID_AUTOPILOT1,&m,boot_ms,0,0,-altitude_m,0,0,0);send_message(fd,&m);
}

static void send_prearm_status(int fd,bool healthy) {
  uint32_t bit=MAV_SYS_STATUS_PREARM_CHECK;mavlink_message_t m;
  mavlink_msg_sys_status_pack(1,MAV_COMP_ID_AUTOPILOT1,&m,bit,bit,healthy?bit:0u,0,12600,-1,-1,0,0,0,0,0,0,0,0,0);send_message(fd,&m);
}

static void send_landed(int fd,bool landed) {
  mavlink_message_t m;mavlink_msg_extended_sys_state_pack(1,MAV_COMP_ID_AUTOPILOT1,&m,MAV_VTOL_STATE_UNDEFINED,landed?MAV_LANDED_STATE_ON_GROUND:MAV_LANDED_STATE_IN_AIR);send_message(fd,&m);
}

static void setup_mavlink(MavlinkBackend *b,int *master,int *slave,uint64_t now) {
  raw_pty(master,slave);mavlink_backend_init(b);CHECK(mavlink_backend_configure(b,"/dev/test",115200,1,MAV_COMP_ID_AUTOPILOT1));CHECK(mavlink_backend_attach_fd(b,*slave,true)==0);
  send_heartbeat(*master,1,MAV_COMP_ID_AUTOPILOT1,4,false);CHECK(mavlink_backend_poll(b,now)==0);CHECK(b->status.target_verified&&b->status.heartbeat_fresh);
}

static void test_mavlink_ack_and_observation(void) {
  MavlinkBackend b;int master,slave;setup_mavlink(&b,&master,&slave,100);
  CHECK(mavlink_backend_request_mode(&b,4,100)==0);CHECK(b.pending.active);
  send_ack(master,MAV_CMD_COMPONENT_ARM_DISARM,MAV_RESULT_ACCEPTED);CHECK(mavlink_backend_poll(&b,200)==0);CHECK(b.pending.active&&b.status.wrong_acks==1);
  send_ack(master,MAV_CMD_DO_SET_MODE,MAV_RESULT_ACCEPTED);CHECK(mavlink_backend_poll(&b,300)==0);CHECK(!b.pending.active&&mavlink_backend_acknowledged(&b,MAV_CMD_DO_SET_MODE));
  send_ack(master,MAV_CMD_DO_SET_MODE,MAV_RESULT_ACCEPTED);CHECK(mavlink_backend_poll(&b,400)==0);CHECK(b.status.duplicate_acks==1);
  /* ACK success is not mode observation: the heartbeat remains authoritative. */
  CHECK(b.status.mode_safe);send_heartbeat(master,1,MAV_COMP_ID_AUTOPILOT1,3,false);CHECK(mavlink_backend_poll(&b,500)==0);CHECK(!b.status.mode_safe);
  close(master);mavlink_backend_close(&b);
}

static void test_mavlink_timeout_late_and_rejection(void) {
  MavlinkBackend b;int master,slave;setup_mavlink(&b,&master,&slave,100);
  CHECK(mavlink_backend_request_arm(&b,true,100)==0);
  CHECK(mavlink_backend_poll(&b,1300)==0);CHECK(b.pending.active&&b.pending.retries==1);
  CHECK(mavlink_backend_poll(&b,2500)==0);CHECK(b.pending.active&&b.pending.retries==2);
  CHECK(mavlink_backend_poll(&b,3700)==0);CHECK(!b.pending.active&&b.status.command_timed_out);
  send_ack(master,MAV_CMD_COMPONENT_ARM_DISARM,MAV_RESULT_ACCEPTED);CHECK(mavlink_backend_poll(&b,3800)==0);CHECK(b.status.late_acks==1&&!b.status.armed);
  send_heartbeat(master,1,MAV_COMP_ID_AUTOPILOT1,4,false);CHECK(mavlink_backend_poll(&b,3900)==0);
  CHECK(mavlink_backend_request_arm(&b,true,3900)==0);send_ack(master,MAV_CMD_COMPONENT_ARM_DISARM,MAV_RESULT_DENIED);CHECK(mavlink_backend_poll(&b,4000)==0);CHECK(b.status.command_rejected&&!b.status.armed);
  close(master);mavlink_backend_close(&b);
}

static void test_mavlink_target_mode_reject_and_malformed(void) {
  MavlinkBackend b;int master,slave;raw_pty(&master,&slave);mavlink_backend_init(&b);CHECK(mavlink_backend_configure(&b,"/dev/test",115200,1,MAV_COMP_ID_AUTOPILOT1));CHECK(mavlink_backend_attach_fd(&b,slave,true)==0);
  send_heartbeat(master,2,MAV_COMP_ID_AUTOPILOT1,4,false);CHECK(mavlink_backend_poll(&b,100)==0);CHECK(!b.status.target_verified);
  send_heartbeat(master,1,MAV_COMP_ID_AUTOPILOT1,3,false);CHECK(mavlink_backend_poll(&b,200)==0);CHECK(b.status.target_verified&&!b.status.mode_safe);
  send_prearm_status(master,false);CHECK(mavlink_backend_poll(&b,225)==0);CHECK(b.status.prearm_check_available&&!b.status.prearm_check_healthy&&b.status.system_status_ms==225);
  send_prearm_status(master,true);CHECK(mavlink_backend_poll(&b,250)==0);CHECK(b.status.prearm_check_healthy&&b.status.battery_valid);
  CHECK(mavlink_backend_request_mode(&b,4,250)==0);send_ack(master,MAV_CMD_DO_SET_MODE,MAV_RESULT_DENIED);CHECK(mavlink_backend_poll(&b,300)==0);CHECK(b.status.command_rejected&&!b.status.mode_safe);
  mavlink_message_t message;mavlink_msg_heartbeat_pack(1,MAV_COMP_ID_AUTOPILOT1,&message,MAV_TYPE_QUADROTOR,MAV_AUTOPILOT_ARDUPILOTMEGA,0,3,MAV_STATE_ACTIVE);uint8_t data[MAVLINK_MAX_PACKET_LEN];uint16_t length=mavlink_msg_to_send_buffer(data,&message);data[length-1]^=0x55u;CHECK(write(master,data,length)==length);CHECK(mavlink_backend_poll(&b,400)==0);CHECK(b.status.heartbeat_ms==200);
  CHECK(mavlink_backend_poll(&b,2000)==0);CHECK(!b.status.heartbeat_fresh);
  close(master);mavlink_backend_close(&b);
}

static void test_mavlink_fails_closed(void) {
  MavlinkBackend b;int master,slave;setup_mavlink(&b,&master,&slave,100);
  send_position(master,5000,0.0f);send_landed(master,false);CHECK(mavlink_backend_poll(&b,200)==0);
  CHECK(mavlink_backend_request_takeoff(&b,0.8f,200)==0);send_ack(master,MAV_CMD_NAV_TAKEOFF,MAV_RESULT_ACCEPTED);CHECK(mavlink_backend_poll(&b,300)==0);CHECK(mavlink_backend_acknowledged(&b,MAV_CMD_NAV_TAKEOFF)&&b.status.altitude_m==0.0f);
  send_position(master,5100,0.25f);CHECK(mavlink_backend_poll(&b,400)==0);CHECK(b.status.altitude_m>0.2f&&b.status.altitude_m<0.8f);
  CHECK(!mavlink_backend_observation_fresh(1500,b.status.local_position_ms,800));
  send_heartbeat(master,1,MAV_COMP_ID_AUTOPILOT1,4,true);CHECK(mavlink_backend_poll(&b,500)==0);
  CHECK(mavlink_backend_request_land(&b,500)==0);send_ack(master,MAV_CMD_NAV_LAND,MAV_RESULT_ACCEPTED);CHECK(mavlink_backend_poll(&b,600)==0);CHECK(!b.status.landed);
  CHECK(mavlink_backend_request_disarm_if_landed(&b,600)<0);
  send_landed(master,true);send_position(master,5200,0.4f);CHECK(mavlink_backend_poll(&b,700)==0);CHECK(mavlink_backend_request_disarm_if_landed(&b,700)<0);
  send_position(master,100,0.0f);CHECK(mavlink_backend_poll(&b,800)==0);CHECK(b.status.fc_restart_detected&&b.status.operator_reset_required);CHECK(mavlink_backend_request_arm(&b,true,800)<0);
  close(master);CHECK(mavlink_backend_poll(&b,900)<0);CHECK(b.status.connection_lost);
  mavlink_backend_close(&b);
  /* A process restart creates monitoring state only; it cannot arm before a verified heartbeat and explicit request. */
  MavlinkBackend restarted;mavlink_backend_init(&restarted);CHECK(!restarted.status.armed&&!restarted.pending.active&&restarted.connection_state==MAV_CONN_CLOSED);
}

static uint8_t checksum(const uint8_t *p,size_t n){uint8_t c=0;for(size_t i=0;i<n;i++)c^=p[i];return c;}
static void make_tof(uint8_t frame[TOF_LEGACY_A5_FRAME_SIZE],uint32_t uptime,uint16_t range) {
  memset(frame,0,TOF_LEGACY_A5_FRAME_SIZE);frame[0]=0xA5;frame[1]=(uint8_t)uptime;frame[2]=(uint8_t)(uptime>>8);frame[3]=(uint8_t)(uptime>>16);frame[4]=(uint8_t)(uptime>>24);
  for(size_t i=0;i<TOF_SENSOR_COUNT*TOF_ZONES_PER_SENSOR;i++){frame[5+i*2]=(uint8_t)range;frame[6+i*2]=(uint8_t)(range>>8);}frame[517]=checksum(frame,517);
}

static void test_tof_parser_and_map(void) {
  LiveTofAdapter a;live_tof_adapter_init(&a);CHECK(!live_tof_adapter_configure(&a,"/dev/test",921600,TOF_PROTOCOL_NONE));CHECK(tof_protocol_from_name("SCLOG3")==TOF_PROTOCOL_NONE);CHECK(live_tof_adapter_configure(&a,"/dev/test",921600,TOF_PROTOCOL_LEGACY_A5_V0));
  uint8_t frame[TOF_LEGACY_A5_FRAME_SIZE];make_tof(frame,1000,1000);
  CHECK(live_tof_adapter_feed(&a,frame,100,100)==0);CHECK(!a.latest.valid);CHECK(live_tof_adapter_feed(&a,frame+100,sizeof(frame)-100,100)==1);CHECK(a.latest.valid&&a.latest.valid_zone_count==256&&a.latest.sequence==1);
  CHECK(live_tof_adapter_feed(&a,frame,sizeof(frame),110)==0&&a.repeated_frames==1);
  make_tof(frame,900,1000);CHECK(live_tof_adapter_feed(&a,frame,sizeof(frame),120)==0&&a.out_of_order_frames==1);
  make_tof(frame,1100,65000);CHECK(live_tof_adapter_feed(&a,frame,sizeof(frame),130)==0&&a.all_invalid_frames==1);
  make_tof(frame,1200,1000);frame[20]^=1u;CHECK(live_tof_adapter_feed(&a,frame,sizeof(frame),140)==0&&a.checksum_errors>0);
  uint8_t control[TOF_CONTROL_A6_FRAME_SIZE]={0xA6,1,2,3,4,5,0};control[6]=checksum(control,6);CHECK(live_tof_adapter_feed(&a,control,sizeof(control),145)==0&&a.control_frames==1);
  make_tof(frame,1300,1000);uint8_t mixed[TOF_LEGACY_A5_FRAME_SIZE+3];mixed[0]=0xA7;mixed[1]=1;mixed[2]=2;memcpy(mixed+3,frame,sizeof(frame));CHECK(live_tof_adapter_feed(&a,mixed,sizeof(mixed),150)==1&&a.malformed_frames>=3);
  CHECK(!live_tof_adapter_fresh(&a,200,100));a.connected=true;CHECK(live_tof_adapter_fresh(&a,200,100));CHECK(!live_tof_adapter_fresh(&a,300,100));
  CellState cells[40*40];MappingAdapter map;CHECK(mapping_adapter_init(&map,cells,40,40,.15f,0,0,false));VehiclePose pose={.x_m=0,.y_m=0,.yaw_deg=0,.timestamp_ms=145,.valid=true};CHECK(mapping_adapter_apply_live_tof(&map,&a.latest,&pose,20));CHECK(map.grid.timestamp_ms==150&&map.grid.sequence==1&&!map.grid.synthetic);
  MappingAdapter fake;CellState fake_cells[100];CHECK(mapping_adapter_init(&fake,fake_cells,10,10,.15f,0,0,true));CHECK(!mapping_adapter_apply_live_tof(&fake,&a.latest,&pose,20));
}

static void test_tof_pty_disconnect_reconnect(void) {
  LiveTofAdapter a;live_tof_adapter_init(&a);CHECK(live_tof_adapter_configure(&a,"/dev/test",921600,TOF_PROTOCOL_LEGACY_A5_V0));int master,slave;raw_pty(&master,&slave);CHECK(live_tof_adapter_attach_fd(&a,slave,true)==0);
  uint8_t frame[TOF_LEGACY_A5_FRAME_SIZE];make_tof(frame,100,800);CHECK(write(master,frame,sizeof(frame))==(ssize_t)sizeof(frame));CHECK(live_tof_adapter_poll(&a,100)==1);close(master);CHECK(live_tof_adapter_poll(&a,200)<0&&a.reset_required);
  live_tof_adapter_explicit_reset(&a);CHECK(!a.reset_required&&a.reconnects==1&&!a.latest.valid);int master2,slave2;raw_pty(&master2,&slave2);CHECK(live_tof_adapter_attach_fd(&a,slave2,true)==0);make_tof(frame,10,900);CHECK(write(master2,frame,sizeof(frame))==(ssize_t)sizeof(frame));CHECK(live_tof_adapter_poll(&a,300)==1);close(master2);live_tof_adapter_close(&a);
}

static void test_tof_truncated_disconnect(void) {
  LiveTofAdapter a;live_tof_adapter_init(&a);CHECK(live_tof_adapter_configure(&a,"/dev/test",921600,TOF_PROTOCOL_LEGACY_A5_V0));int master,slave;raw_pty(&master,&slave);CHECK(live_tof_adapter_attach_fd(&a,slave,true)==0);uint8_t frame[TOF_LEGACY_A5_FRAME_SIZE];make_tof(frame,10,900);CHECK(write(master,frame,100)==100);CHECK(live_tof_adapter_poll(&a,10)==0);close(master);CHECK(live_tof_adapter_poll(&a,20)<0&&a.reset_required&&!a.latest.valid);live_tof_adapter_close(&a);
}

int main(void){test_mavlink_ack_and_observation();test_mavlink_timeout_late_and_rejection();test_mavlink_target_mode_reject_and_malformed();test_mavlink_fails_closed();test_tof_parser_and_map();test_tof_pty_disconnect_reconnect();test_tof_truncated_disconnect();printf("live interface tests: %d checks, %d failures\n",checks,failures);return failures?1:0;}
