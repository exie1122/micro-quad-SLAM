#ifndef LIVE_TOF_ADAPTER_H
#define LIVE_TOF_ADAPTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "tof_protocol.h"

#define TOF_PARSE_BUFFER_CAPACITY (TOF_LEGACY_A5_FRAME_SIZE * 2u)

typedef struct {
  int fd;
  bool owns_fd;
  char device[160];
  int baud;
  TofProtocol configured_protocol;
  uint8_t buffer[TOF_PARSE_BUFFER_CAPACITY];
  size_t buffered;
  TofFrame latest;
  bool have_uptime;
  uint32_t last_uptime_ms;
  uint32_t next_sequence;
  uint64_t last_receive_ms;
  bool connected;
  bool reset_required;
  uint32_t accepted_frames;
  uint32_t checksum_errors;
  uint32_t malformed_frames;
  uint32_t repeated_frames;
  uint32_t out_of_order_frames;
  uint32_t all_invalid_frames;
  uint32_t control_frames;
  uint32_t reconnects;
} LiveTofAdapter;

void live_tof_adapter_init(LiveTofAdapter *adapter);
bool live_tof_adapter_configure(LiveTofAdapter *adapter,const char *device,int baud,
                                TofProtocol protocol);
int live_tof_adapter_attach_fd(LiveTofAdapter *adapter,int fd,bool take_ownership);
int live_tof_adapter_connect(LiveTofAdapter *adapter);
int live_tof_adapter_feed(LiveTofAdapter *adapter,const uint8_t *data,size_t length,
                          uint64_t now_ms);
int live_tof_adapter_poll(LiveTofAdapter *adapter,uint64_t now_ms);
bool live_tof_adapter_fresh(const LiveTofAdapter *adapter,uint64_t now_ms,uint64_t max_age_ms);
const TofFrame *live_tof_adapter_latest(const LiveTofAdapter *adapter);
void live_tof_adapter_close(LiveTofAdapter *adapter);
void live_tof_adapter_explicit_reset(LiveTofAdapter *adapter);

#endif
