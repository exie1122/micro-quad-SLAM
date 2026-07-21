#ifndef RECORDED_LOG_H
#define RECORDED_LOG_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define SCLOG3_RECORD_SIZE 642u
#define SCLOG3_MAGIC 0x334E4353u

typedef struct __attribute__((packed)) {
  uint32_t magic; uint64_t host_ms; uint32_t scan_ms; uint32_t custom_mode;
  float x_m,y_m,yaw_deg,alt_m,alt_max_m,roll_rad,pitch_rad,vx_mps,vy_mps,vz_mps;
  float lpos_alt_m,lpos_alt_filt_m,rf_m,rf_v,of_rate_x,of_rate_y,of_comp_m_x,of_comp_m_y,of_ground_m;
  uint32_t sys_present,sys_health,sys_enabled;
  uint16_t att_age_ms,lpos_age_ms,of_age_ms,rf_age_ms,hb_age_ms,pose_flags;
  uint8_t of_q,alt_src,rf_src,of_src,fc_armed,alt_rf_rejected,ds_orientation,ds_id;
  uint16_t ds_cur_cm; uint8_t grid_raw[512];
} Sclog3Record;

typedef enum {REPLAY_OK=0,REPLAY_EOF,REPLAY_BAD_HEADER,REPLAY_IO_ERROR} ReplayStatus;
typedef struct {FILE *file;uint64_t offset;uint64_t records;uint64_t warnings;uint64_t timestamp_resets;uint64_t previous_timestamp_ms;char path[512];} ReplayReader;

ReplayStatus replay_reader_open(ReplayReader *reader,const char *path);
ReplayStatus replay_reader_next(ReplayReader *reader,Sclog3Record *record);
void replay_reader_close(ReplayReader *reader);

#endif
