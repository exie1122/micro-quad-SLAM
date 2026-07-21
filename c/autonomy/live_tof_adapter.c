#include "live_tof_adapter.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define TOF_READ_BUDGET 4096u
#define TOF_MIN_VALID_MM 100u
#define TOF_MAX_VALID_MM 4000u

const char *tof_protocol_name(TofProtocol protocol) {
  return protocol==TOF_PROTOCOL_LEGACY_A5_V0?"legacy-a5-v0":"none";
}

TofProtocol tof_protocol_from_name(const char *name) {
  return name&&strcmp(name,"legacy-a5-v0")==0?TOF_PROTOCOL_LEGACY_A5_V0:TOF_PROTOCOL_NONE;
}

static speed_t tof_baud_flag(int baud) {
  switch (baud) {
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

static uint8_t xor_checksum(const uint8_t *data,size_t length) {
  uint8_t checksum=0u;
  for (size_t i=0;i<length;i++) checksum^=data[i];
  return checksum;
}

void live_tof_adapter_init(LiveTofAdapter *a) {
  if (!a) return;
  memset(a,0,sizeof(*a));a->fd=-1;a->baud=921600;a->next_sequence=1u;
}

bool live_tof_adapter_configure(LiveTofAdapter *a,const char *device,int baud,TofProtocol protocol) {
  if (!a||!device||device[0]=='\0'||strlen(device)>=sizeof(a->device)||
      protocol!=TOF_PROTOCOL_LEGACY_A5_V0||tof_baud_flag(baud)==(speed_t)0) return false;
  memcpy(a->device,device,strlen(device)+1u);a->baud=baud;a->configured_protocol=protocol;
  return true;
}

int live_tof_adapter_attach_fd(LiveTofAdapter *a,int fd,bool take_ownership) {
  if (!a||fd<0||a->fd>=0||a->configured_protocol!=TOF_PROTOCOL_LEGACY_A5_V0||a->reset_required) return -1;
  int flags=fcntl(fd,F_GETFL,0);
  if (flags<0||fcntl(fd,F_SETFL,flags|O_NONBLOCK)<0) return -1;
  a->fd=fd;a->owns_fd=take_ownership;a->connected=true;
  return 0;
}

int live_tof_adapter_connect(LiveTofAdapter *a) {
  if (!a||a->fd>=0||a->device[0]=='\0'||a->configured_protocol!=TOF_PROTOCOL_LEGACY_A5_V0||a->reset_required) return -1;
  speed_t speed=tof_baud_flag(a->baud);
  int fd=open(a->device,O_RDWR|O_NOCTTY|O_NONBLOCK);
  if (fd<0) return -1;
  struct termios tio;
  if (tcgetattr(fd,&tio)<0) {close(fd);return -1;}
  cfmakeraw(&tio);cfsetispeed(&tio,speed);cfsetospeed(&tio,speed);
  tio.c_cflag|=CLOCAL|CREAD;tio.c_cflag&=~CSTOPB;tio.c_cflag&=~CRTSCTS;
  if (tcsetattr(fd,TCSANOW,&tio)<0) {close(fd);return -1;}
  return live_tof_adapter_attach_fd(a,fd,true);
}

static void discard_prefix(LiveTofAdapter *a,size_t length) {
  if (length>=a->buffered) {a->buffered=0u;return;}
  memmove(a->buffer,a->buffer+length,a->buffered-length);a->buffered-=length;
}

static int accept_a5(LiveTofAdapter *a,uint64_t now_ms) {
  const uint8_t *p=a->buffer;
  if (xor_checksum(p,TOF_LEGACY_A5_FRAME_SIZE-1u)!=p[TOF_LEGACY_A5_FRAME_SIZE-1u]) {
    a->checksum_errors++;return -1;
  }
  uint32_t uptime=(uint32_t)p[1]|((uint32_t)p[2]<<8)|((uint32_t)p[3]<<16)|((uint32_t)p[4]<<24);
  if (a->have_uptime) {
    uint32_t delta=uptime-a->last_uptime_ms;
    if (delta==0u) {a->repeated_frames++;return -1;}
    if (delta>UINT32_MAX/2u) {a->out_of_order_frames++;return -1;}
  }
  TofFrame frame;
  memset(&frame,0,sizeof(frame));frame.protocol=a->configured_protocol;
  frame.sensor_uptime_ms=uptime;frame.sequence=a->next_sequence++;
  frame.received_monotonic_ms=now_ms;
  for (size_t sensor=0;sensor<TOF_SENSOR_COUNT;sensor++) for (size_t zone=0;zone<TOF_ZONES_PER_SENSOR;zone++) {
    size_t offset=5u+(sensor*TOF_ZONES_PER_SENSOR+zone)*2u;
    uint16_t mm=(uint16_t)p[offset]|((uint16_t)p[offset+1u]<<8);
    if (mm>=TOF_MIN_VALID_MM&&mm<=TOF_MAX_VALID_MM) {
      frame.range_mm[sensor][zone]=mm;frame.valid_zone_count++;
    } else frame.range_mm[sensor][zone]=TOF_INVALID_MM;
  }
  if (frame.valid_zone_count==0u) {a->all_invalid_frames++;return -1;}
  frame.valid=true;a->latest=frame;a->last_uptime_ms=uptime;a->have_uptime=true;
  a->last_receive_ms=now_ms;a->accepted_frames++;
  return 1;
}

static int parse_buffer(LiveTofAdapter *a,uint64_t now_ms) {
  int accepted=0;
  while (a->buffered>0u) {
    size_t start=0u;
    while (start<a->buffered&&a->buffer[start]!=0xA5u&&a->buffer[start]!=0xA6u) start++;
    if (start>0u) {a->malformed_frames+=(uint32_t)start;discard_prefix(a,start);continue;}
    size_t needed=a->buffer[0]==0xA5u?TOF_LEGACY_A5_FRAME_SIZE:TOF_CONTROL_A6_FRAME_SIZE;
    if (a->buffered<needed) break;
    if (xor_checksum(a->buffer,needed-1u)!=a->buffer[needed-1u]) {
      a->checksum_errors++;discard_prefix(a,1u);continue;
    }
    if (a->buffer[0]==0xA6u) {a->control_frames++;discard_prefix(a,needed);continue;}
    int result=accept_a5(a,now_ms);
    discard_prefix(a,needed);
    if (result>0) accepted++;
  }
  return accepted;
}

int live_tof_adapter_feed(LiveTofAdapter *a,const uint8_t *data,size_t length,uint64_t now_ms) {
  if (!a||(!data&&length>0u)||a->configured_protocol!=TOF_PROTOCOL_LEGACY_A5_V0||a->reset_required) return -1;
  size_t consumed=0u;int accepted=0;
  while (consumed<length) {
    if (a->buffered==sizeof(a->buffer)) {a->malformed_frames++;discard_prefix(a,1u);}
    size_t space=sizeof(a->buffer)-a->buffered;
    size_t chunk=length-consumed<space?length-consumed:space;
    memcpy(a->buffer+a->buffered,data+consumed,chunk);a->buffered+=chunk;consumed+=chunk;
    int n=parse_buffer(a,now_ms);if (n<0) return -1;accepted+=n;
  }
  return accepted;
}

int live_tof_adapter_poll(LiveTofAdapter *a,uint64_t now_ms) {
  if (!a||a->fd<0||!a->connected||a->reset_required) return -1;
  uint8_t data[512];size_t total=0u;int accepted=0;
  for (;;) {
    ssize_t n=read(a->fd,data,sizeof(data));
    if (n>0) {
      int result=live_tof_adapter_feed(a,data,(size_t)n,now_ms);
      if (result<0) return -1;
      accepted+=result;total+=(size_t)n;if (total>=TOF_READ_BUDGET) break;continue;
    }
    if (n==0) {a->connected=false;a->reset_required=true;return -1;}
    if (errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR) break;
    a->connected=false;a->reset_required=true;return -1;
  }
  return accepted;
}

bool live_tof_adapter_fresh(const LiveTofAdapter *a,uint64_t now_ms,uint64_t max_age_ms) {
  return a&&a->connected&&a->latest.valid&&now_ms>=a->last_receive_ms&&now_ms-a->last_receive_ms<=max_age_ms;
}

const TofFrame *live_tof_adapter_latest(const LiveTofAdapter *a) {return a&&a->latest.valid?&a->latest:NULL;}

void live_tof_adapter_close(LiveTofAdapter *a) {
  if (!a) return;
  if (a->fd>=0&&a->owns_fd) close(a->fd);
  a->fd=-1;a->owns_fd=false;a->connected=false;
}

void live_tof_adapter_explicit_reset(LiveTofAdapter *a) {
  if (!a) return;
  live_tof_adapter_close(a);a->buffered=0u;a->have_uptime=false;a->latest.valid=false;
  a->reset_required=false;a->reconnects++;
}
