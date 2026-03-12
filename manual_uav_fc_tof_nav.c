#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L

// manual_uav_fc_tof_nav.c
//currently testing. no autonomy, just using the TX
//to do:
// - tune PIDs
// - manage magnetometer/gyro drift: yaw has bad problems
//    -possibly due to oscillations

// Manual-flight mapping logger:
// - receives MAVLink pose / flow / altitude data from the FC
// - receives raw 4x8x8 VL53L5CX scan packets over the ToF UART
// - writes one binary record per usable scan for offline 2D map building

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <math.h>
#include <time.h>
#include <stdarg.h>

#include "common/mavlink.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef CRTSCTS
#define CRTSCTS 0
#endif

// ----------------------------- UART config -----------------------------
#define FC_UART  "/dev/ttyS2"
#define FC_BAUD  57600

#define TOF_UART "/dev/ttyS1"
#define TOF_BAUD 115200

// ----------------------------- ArduPilot RANGEFINDER -------------------
#ifndef MAVLINK_MSG_ID_RANGEFINDER
#define MAVLINK_MSG_ID_RANGEFINDER 173
#endif

#define ORIENT_DOWNWARD_FACING 25

// ----------------------------- Logging config --------------------------
#define LOG_SCAN_PATH         "/mnt/sdcard/scanlog.bin"
#define LOG_TXT_PATH          "log.txt"
#define LOG_FLUSH_MS          1000
#define SENSOR_FRESH_MS       400
#define HEARTBEAT_TIMEOUT_MS  3000
#define SCAN_PENDING_MAX_MS   250
#define STATUS_PRINT_MS       1000
#define POSE_RING_SZ          32

static FILE* scan_fp = NULL;
static FILE* txt_log_fp = NULL;
static uint64_t last_flush_ms = 0;

// ----------------------------- ToF frame -------------------------------
#define SCAN_HEADER 0xA5
#define NUM_SENSORS 4
#define TOF_ROWS    8
#define TOF_COLS    8
#define GRID_SIZE   (TOF_ROWS * TOF_COLS)
#define TOTAL_CELLS (NUM_SENSORS * GRID_SIZE)
#define SCAN_BYTES  (1 + 4 + (TOTAL_CELLS * 2) + 1)

static uint8_t tof_rxbuf[SCAN_BYTES];
static int     tof_rxpos = 0;

static bool     have_scan_frame = false;
static uint32_t last_scan_t_ms = 0;
static uint64_t last_scan_host_ms = 0;
static uint8_t  last_scan_grid_raw[TOTAL_CELLS * 2];
static volatile bool scan_new = false;

// Physical sensor order in each scan packet: FRONT, RIGHT, BACK, LEFT.
enum Dir { D_FRONT = 0, D_RIGHT = 1, D_BACK = 2, D_LEFT = 3 };

typedef enum {
  ALT_SRC_NONE = 0,
  ALT_SRC_LPOS = 1,
  ALT_SRC_DISTANCE_SENSOR = 2,
  ALT_SRC_RANGEFINDER = 3
} AltSrc;

typedef enum {
  RF_SRC_NONE = 0,
  RF_SRC_DISTANCE_SENSOR = 1,
  RF_SRC_RANGEFINDER = 2
} RangefinderSrc;

typedef enum {
  OF_SRC_NONE = 0,
  OF_SRC_OPTICAL_FLOW = 1,
  OF_SRC_OPTICAL_FLOW_RAD = 2
} OpticalFlowSrc;

enum {
  POSEF_FC_LINK    = 1u << 0,
  POSEF_FC_ARMED   = 1u << 1,
  POSEF_ATT_FRESH  = 1u << 2,
  POSEF_LPOS_FRESH = 1u << 3,
  POSEF_OF_FRESH   = 1u << 4,
  POSEF_RF_FRESH   = 1u << 5,
  POSEF_SYS_FRESH  = 1u << 6,
  POSEF_ALT_VALID  = 1u << 7
};

// ----------------------------- MAVLink state ---------------------------
static int fc_fd = -1;
static int tof_fd = -1;

static uint8_t g_sysid = 255;
static const uint8_t g_compid = MAV_COMP_ID_ONBOARD_COMPUTER;

static uint8_t fc_sysid = 0;
static uint8_t fc_compid = 0;
static bool have_fc = false;
static bool streams_requested = false;

static uint64_t last_hb_ms = 0;
static uint32_t hb_custom_mode = 0;
static bool fc_armed = false;

// ----------------------------- Mapping gate ----------------------------
static bool mapping_enabled = true;
static bool allow_disarmed_logging = false;

// ----------------------------- Pose / sensor state ---------------------
static bool  have_lpos = false;
static float lpos_x_m = NAN;
static float lpos_y_m = NAN;
static float lpos_vx_mps = NAN;
static float lpos_vy_mps = NAN;
static float lpos_vz_mps = NAN;  // NED sign from LOCAL_POSITION_NED
static float lpos_alt_m = NAN;   // positive up
static float lpos_alt_filt_m = NAN;
static uint64_t lpos_last_update_ms = 0;

static bool  have_att = false;
static float roll_rad = 0.0f;
static float pitch_rad = 0.0f;
static float yaw_rad = 0.0f;
static uint64_t att_last_update_ms = 0;

static bool          have_of = false;
static OpticalFlowSrc of_src = OF_SRC_NONE;
static uint8_t       of_quality = 0;
static float         of_comp_m_x = NAN;
static float         of_comp_m_y = NAN;
static float         of_ground_m = NAN;
static float         of_rate_x = NAN;
static float         of_rate_y = NAN;
static uint64_t      of_last_update_ms = 0;

static bool           have_rangefinder = false;
static float          rangefinder_m = NAN;
static float          rangefinder_v = NAN;
static uint64_t       rangefinder_last_update_ms = 0;
static RangefinderSrc rf_src = RF_SRC_NONE;

static bool     have_ds = false;
static uint8_t  ds_id = 0;
static uint8_t  ds_orientation = 0;
static uint16_t ds_cur_cm = 0;
static uint64_t ds_last_update_ms = 0;

static bool     have_sys = false;
static uint32_t sys_present = 0;
static uint32_t sys_enabled = 0;
static uint32_t sys_health = 0;
static uint64_t sys_last_ms = 0;

static float  alt_est_m = NAN;
static float  alt_max_m = NAN;
static bool   alt_rf_rejected = false;
static AltSrc alt_src = ALT_SRC_NONE;

// ----------------------------- Pose ring -------------------------------
typedef struct {
  uint64_t host_ms;
  float    x_m;
  float    y_m;
  float    vx_mps;
  float    vy_mps;
  float    vz_mps;
  float    alt_m;
  float    lpos_alt_m;
  float    lpos_alt_filt_m;
  float    yaw_deg;
  float    roll_rad;
  float    pitch_rad;
  uint8_t  alt_src;
  uint8_t  valid_xy;
  uint8_t  valid_att;
} pose_sample_t;

static pose_sample_t pose_ring[POSE_RING_SZ];
static int pose_ring_head = 0;
static int pose_ring_count = 0;

// ----------------------------- Timing helpers --------------------------
static uint64_t now_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static float rad2deg(float r) { return r * (180.0f / (float)M_PI); }

static float wrap_deg(float d) {
  while (d >= 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

static float current_heading_deg(void) {
  return wrap_deg(rad2deg(yaw_rad));
}

static float lerp_float(float a, float b, float u) {
  return a + (b - a) * u;
}

static float lerp_angle_deg(float a, float b, float u) {
  float delta = wrap_deg(b - a);
  return wrap_deg(a + delta * u);
}

static uint16_t sat_u16_age(uint64_t ref_ms, uint64_t update_ms) {
  if (update_ms == 0 || ref_ms <= update_ms) return 0;
  uint64_t age = ref_ms - update_ms;
  if (age > 0xFFFFu) age = 0xFFFFu;
  return (uint16_t)age;
}

// ----------------------------- Logging helper --------------------------
static void log_msg(const char* fmt, ...) {
  va_list args;

  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
  fflush(stdout);

  if (txt_log_fp) {
    fprintf(txt_log_fp, "[%.3f] ", now_ms() * 0.001f);
    va_start(args, fmt);
    vfprintf(txt_log_fp, fmt, args);
    va_end(args);
    fflush(txt_log_fp);
  }
}

#define printf log_msg

// ----------------------------- Freshness helpers -----------------------
static bool fc_link_fresh(uint64_t t) {
  return have_fc && last_hb_ms != 0 && (t - last_hb_ms) < HEARTBEAT_TIMEOUT_MS;
}

static bool att_fresh(uint64_t t) {
  return have_att && att_last_update_ms != 0 && (t - att_last_update_ms) < SENSOR_FRESH_MS;
}

static bool lpos_fresh(uint64_t t) {
  return have_lpos && lpos_last_update_ms != 0 && (t - lpos_last_update_ms) < SENSOR_FRESH_MS;
}

static bool of_fresh(uint64_t t) {
  return have_of && of_last_update_ms != 0 && (t - of_last_update_ms) < SENSOR_FRESH_MS;
}

static bool rf_fresh(uint64_t t) {
  return have_rangefinder &&
         rangefinder_last_update_ms != 0 &&
         (t - rangefinder_last_update_ms) < SENSOR_FRESH_MS;
}

static bool sys_fresh(uint64_t t) {
  return have_sys && sys_last_ms != 0 && (t - sys_last_ms) < 1000;
}

static uint16_t pose_flags_for_time(uint64_t t) {
  uint16_t flags = 0;
  if (fc_link_fresh(t)) flags |= POSEF_FC_LINK;
  if (fc_armed) flags |= POSEF_FC_ARMED;
  if (att_fresh(t)) flags |= POSEF_ATT_FRESH;
  if (lpos_fresh(t)) flags |= POSEF_LPOS_FRESH;
  if (of_fresh(t)) flags |= POSEF_OF_FRESH;
  if (rf_fresh(t)) flags |= POSEF_RF_FRESH;
  if (sys_fresh(t)) flags |= POSEF_SYS_FRESH;
  if (isfinite(alt_est_m)) flags |= POSEF_ALT_VALID;
  return flags;
}

static bool pose_valid_for_mapping(uint64_t t) {
  if (!fc_link_fresh(t)) return false;
  if (!att_fresh(t)) return false;
  if (!lpos_fresh(t)) return false;
  if (!isfinite(lpos_x_m) || !isfinite(lpos_y_m)) return false;
  if (!isfinite(alt_est_m)) return false;
  return true;
}

static bool mapping_gate(uint64_t t) {
  if (!mapping_enabled) return false;
  if (!allow_disarmed_logging && !fc_armed) return false;
  return pose_valid_for_mapping(t);
}

// ----------------------------- Pose ring helpers -----------------------
static void pose_ring_push(uint64_t t) {
  pose_sample_t* s = &pose_ring[pose_ring_head];

  s->host_ms = t;
  s->x_m = lpos_x_m;
  s->y_m = lpos_y_m;
  s->vx_mps = lpos_vx_mps;
  s->vy_mps = lpos_vy_mps;
  s->vz_mps = lpos_vz_mps;
  s->alt_m = alt_est_m;
  s->lpos_alt_m = lpos_alt_m;
  s->lpos_alt_filt_m = lpos_alt_filt_m;
  s->yaw_deg = have_att ? current_heading_deg() : NAN;
  s->roll_rad = have_att ? roll_rad : NAN;
  s->pitch_rad = have_att ? pitch_rad : NAN;
  s->alt_src = (uint8_t)alt_src;
  s->valid_xy = pose_valid_for_mapping(t) ? 1u : 0u;
  s->valid_att = att_fresh(t) ? 1u : 0u;

  pose_ring_head = (pose_ring_head + 1) % POSE_RING_SZ;
  if (pose_ring_count < POSE_RING_SZ) pose_ring_count++;
}

static bool pose_ring_sample_at(uint64_t target_ms, pose_sample_t* out) {
  if (!out || pose_ring_count == 0) return false;

  const pose_sample_t* before = NULL;
  const pose_sample_t* after = NULL;
  const pose_sample_t* nearest = NULL;
  uint64_t nearest_dt = UINT64_MAX;

  for (int i = 0; i < pose_ring_count; i++) {
    int idx = (pose_ring_head - pose_ring_count + i + POSE_RING_SZ) % POSE_RING_SZ;
    const pose_sample_t* s = &pose_ring[idx];
    if (s->host_ms == 0) continue;

    uint64_t dt = (s->host_ms > target_ms) ? (s->host_ms - target_ms) : (target_ms - s->host_ms);
    if (dt < nearest_dt) {
      nearest_dt = dt;
      nearest = s;
    }
    if (s->host_ms <= target_ms) before = s;
    if (!after && s->host_ms >= target_ms) after = s;
  }

  if (!before && !after && !nearest) return false;
  if (!before || !after || before == after || after->host_ms == before->host_ms) {
    *out = nearest ? *nearest : (before ? *before : *after);
    return true;
  }

  float u = (float)(target_ms - before->host_ms) / (float)(after->host_ms - before->host_ms);
  if (u < 0.0f) u = 0.0f;
  if (u > 1.0f) u = 1.0f;

  memset(out, 0, sizeof(*out));
  out->host_ms = target_ms;
  out->x_m = lerp_float(before->x_m, after->x_m, u);
  out->y_m = lerp_float(before->y_m, after->y_m, u);
  out->vx_mps = lerp_float(before->vx_mps, after->vx_mps, u);
  out->vy_mps = lerp_float(before->vy_mps, after->vy_mps, u);
  out->vz_mps = lerp_float(before->vz_mps, after->vz_mps, u);
  out->alt_m = lerp_float(before->alt_m, after->alt_m, u);
  out->lpos_alt_m = lerp_float(before->lpos_alt_m, after->lpos_alt_m, u);
  out->lpos_alt_filt_m = lerp_float(before->lpos_alt_filt_m, after->lpos_alt_filt_m, u);
  out->yaw_deg = lerp_angle_deg(before->yaw_deg, after->yaw_deg, u);
  out->roll_rad = lerp_float(before->roll_rad, after->roll_rad, u);
  out->pitch_rad = lerp_float(before->pitch_rad, after->pitch_rad, u);
  out->alt_src = (u < 0.5f) ? before->alt_src : after->alt_src;
  out->valid_xy = before->valid_xy && after->valid_xy;
  out->valid_att = before->valid_att && after->valid_att;
  return true;
}

// ----------------------------- UART open -------------------------------
static int open_uart(const char* dev, int baud) {
  int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    log_msg("open(%s) failed: %s\n", dev, strerror(errno));
    return -1;
  }

  struct termios tio;
  memset(&tio, 0, sizeof(tio));
  if (tcgetattr(fd, &tio) != 0) {
    log_msg("tcgetattr(%s) failed: %s\n", dev, strerror(errno));
    close(fd);
    return -1;
  }

  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CRTSCTS;

  speed_t sp = B57600;
  switch (baud) {
    case 115200: sp = B115200; break;
    case 57600:  sp = B57600; break;
    case 38400:  sp = B38400; break;
    case 19200:  sp = B19200; break;
    default:     sp = B57600; break;
  }

  cfsetispeed(&tio, sp);
  cfsetospeed(&tio, sp);

  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    log_msg("tcsetattr(%s) failed: %s\n", dev, strerror(errno));
    close(fd);
    return -1;
  }

  return fd;
}

// ----------------------------- MAVLink send ----------------------------
static void mav_send(const mavlink_message_t* msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t n = mavlink_msg_to_send_buffer(buf, msg);
  if (fc_fd >= 0) {
    int ret = write(fc_fd, buf, n);
    (void)ret;
  }
}

static void send_command_long_tgt(uint8_t tgt_sys,
                                  uint8_t tgt_comp,
                                  uint16_t cmd,
                                  float p1,
                                  float p2,
                                  float p3,
                                  float p4,
                                  float p5,
                                  float p6,
                                  float p7) {
  mavlink_message_t m;
  mavlink_msg_command_long_pack(
      g_sysid, g_compid, &m,
      tgt_sys, tgt_comp,
      cmd, 0,
      p1, p2, p3, p4, p5, p6, p7);
  mav_send(&m);
}

static void send_own_heartbeat_tick(uint64_t t) {
  static uint64_t last_ms = 0;
  if ((t - last_ms) < 1000) return;
  last_ms = t;

  mavlink_message_t m;
  mavlink_msg_heartbeat_pack(
      g_sysid, g_compid, &m,
      MAV_TYPE_ONBOARD_CONTROLLER,
      MAV_AUTOPILOT_INVALID,
      0, 0,
      MAV_STATE_ACTIVE);
  mav_send(&m);
}

// ----------------------------- Stream requests -------------------------
static void request_streams(void) {
  if (!have_fc) return;

  const uint8_t tgt_sys = fc_sysid;
  const uint8_t tgt_comp = fc_compid ? fc_compid : 0;

  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_SYS_STATUS,         500000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE,            50000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_LOCAL_POSITION_NED,  50000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_OPTICAL_FLOW,        50000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_OPTICAL_FLOW_RAD,    50000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_DISTANCE_SENSOR,    100000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, (float)MAVLINK_MSG_ID_RANGEFINDER, 100000.0f, 0,0,0,0,0);

  streams_requested = true;
  printf("Requested mapping streams from FC sys=%u comp=%u\n",
         (unsigned)tgt_sys,
         (unsigned)tgt_comp);
}

// ----------------------------- MAVLink handlers ------------------------
static void handle_heartbeat(const mavlink_message_t* msg) {
  mavlink_heartbeat_t hb;
  mavlink_msg_heartbeat_decode(msg, &hb);

  last_hb_ms = now_ms();
  hb_custom_mode = hb.custom_mode;
  fc_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
}

static void handle_sys_status(const mavlink_message_t* msg) {
  mavlink_sys_status_t s;
  mavlink_msg_sys_status_decode(msg, &s);
  sys_present = s.onboard_control_sensors_present;
  sys_enabled = s.onboard_control_sensors_enabled;
  sys_health = s.onboard_control_sensors_health;
  sys_last_ms = now_ms();
  have_sys = true;
}

static void handle_attitude(const mavlink_message_t* msg) {
  mavlink_attitude_t a;
  mavlink_msg_attitude_decode(msg, &a);
  roll_rad = a.roll;
  pitch_rad = a.pitch;
  yaw_rad = a.yaw;
  att_last_update_ms = now_ms();
  have_att = true;
}

static void handle_optical_flow(const mavlink_message_t* msg) {
  mavlink_optical_flow_t o;
  mavlink_msg_optical_flow_decode(msg, &o);

  have_of = true;
  of_src = OF_SRC_OPTICAL_FLOW;
  of_quality = o.quality;
  of_comp_m_x = o.flow_comp_m_x;
  of_comp_m_y = o.flow_comp_m_y;
  of_ground_m = o.ground_distance;
  of_rate_x = o.flow_rate_x;
  of_rate_y = o.flow_rate_y;
  of_last_update_ms = now_ms();
}

static void handle_optical_flow_rad(const mavlink_message_t* msg) {
  mavlink_optical_flow_rad_t o;
  mavlink_msg_optical_flow_rad_decode(msg, &o);

  have_of = true;
  of_src = OF_SRC_OPTICAL_FLOW_RAD;
  of_quality = o.quality;
  of_ground_m = (o.distance >= 0.0f) ? o.distance : NAN;

  const float dt = (float)o.integration_time_us * 1e-6f;
  if (dt > 1e-6f) {
    of_rate_x = o.integrated_x / dt;
    of_rate_y = o.integrated_y / dt;
  } else {
    of_rate_x = NAN;
    of_rate_y = NAN;
  }

  if (!isnan(of_ground_m)) {
    of_comp_m_x = o.integrated_x * of_ground_m;
    of_comp_m_y = o.integrated_y * of_ground_m;
  } else {
    of_comp_m_x = NAN;
    of_comp_m_y = NAN;
  }

  of_last_update_ms = now_ms();
}

static void handle_local_position_ned(const mavlink_message_t* msg) {
  mavlink_local_position_ned_t p;
  mavlink_msg_local_position_ned_decode(msg, &p);

  float alt = -p.z;
  if (!(alt > -5.0f && alt < 50.0f)) return;

  have_lpos = true;
  lpos_x_m = p.x;
  lpos_y_m = p.y;
  lpos_vx_mps = p.vx;
  lpos_vy_mps = p.vy;
  lpos_vz_mps = p.vz;
  lpos_alt_m = alt;

  uint64_t t = now_ms();
  if (isnan(lpos_alt_filt_m)) {
    lpos_alt_filt_m = alt;
  } else {
    const float alpha = 0.18f;
    lpos_alt_filt_m = (1.0f - alpha) * lpos_alt_filt_m + alpha * alt;
  }
  lpos_last_update_ms = t;
}

static void handle_distance_sensor(const mavlink_message_t* msg) {
  mavlink_distance_sensor_t d;
  mavlink_msg_distance_sensor_decode(msg, &d);

  have_ds = true;
  ds_id = d.id;
  ds_orientation = d.orientation;
  ds_cur_cm = d.current_distance;
  ds_last_update_ms = now_ms();

  if (d.current_distance > 0 && d.current_distance < 60000 &&
      d.orientation == ORIENT_DOWNWARD_FACING) {
    rangefinder_m = (float)d.current_distance * 0.01f;
    rangefinder_last_update_ms = ds_last_update_ms;
    have_rangefinder = true;
    rf_src = RF_SRC_DISTANCE_SENSOR;
  }
}

static uint32_t rd_u32_le(const uint8_t* p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static float rd_f32_le(const uint8_t* p) {
  uint32_t u = rd_u32_le(p);
  float f;
  memcpy(&f, &u, sizeof(f));
  return f;
}

static void handle_rangefinder_msg(const mavlink_message_t* msg) {
  if (msg->len < 8) return;

  const uint8_t* p = (const uint8_t*)_MAV_PAYLOAD(msg);
  float dist = rd_f32_le(p + 0);
  float volt = rd_f32_le(p + 4);
  if (!isnan(dist) && dist > 0.0f && dist < 60.0f) {
    rangefinder_m = dist;
    rangefinder_v = volt;
    rangefinder_last_update_ms = now_ms();
    have_rangefinder = true;
    rf_src = RF_SRC_RANGEFINDER;
  }
}

// ----------------------------- FC UART pump ----------------------------
static void pump_fc_uart(void) {
  uint8_t buf[256];
  static mavlink_status_t status;
  static mavlink_message_t msg;

  while (true) {
    int n = (int)read(fc_fd, buf, sizeof(buf));
    if (n <= 0) break;

    for (int i = 0; i < n; i++) {
      if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) continue;

      if (!have_fc && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        fc_sysid = msg.sysid;
        fc_compid = msg.compid;
        have_fc = true;
        streams_requested = false;
        printf("FC connected: sys=%u comp=%u\n",
               (unsigned)fc_sysid,
               (unsigned)fc_compid);
      }

      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:           handle_heartbeat(&msg); break;
        case MAVLINK_MSG_ID_SYS_STATUS:          handle_sys_status(&msg); break;
        case MAVLINK_MSG_ID_ATTITUDE:            handle_attitude(&msg); break;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:  handle_local_position_ned(&msg); break;
        case MAVLINK_MSG_ID_OPTICAL_FLOW:        handle_optical_flow(&msg); break;
        case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:    handle_optical_flow_rad(&msg); break;
        case MAVLINK_MSG_ID_DISTANCE_SENSOR:     handle_distance_sensor(&msg); break;
        case MAVLINK_MSG_ID_RANGEFINDER:         handle_rangefinder_msg(&msg); break;
        default: break;
      }
    }
  }
}

// ----------------------------- ToF parsing -----------------------------
static uint8_t xor8(const uint8_t* p, int len) {
  uint8_t c = 0;
  for (int i = 0; i < len; i++) c ^= p[i];
  return c;
}

static void accept_scan_frame(const uint8_t* frame) {
  last_scan_t_ms = rd_u32_le(frame + 1);
  memcpy(last_scan_grid_raw, frame + 5, sizeof(last_scan_grid_raw));
  last_scan_host_ms = now_ms();
  have_scan_frame = true;
  scan_new = true;
}

static void pump_tof_uart(void) {
  uint8_t buf[512];
  int n = (int)read(tof_fd, buf, sizeof(buf));
  if (n <= 0) return;

  for (int i = 0; i < n; i++) {
    uint8_t b = buf[i];

    if (tof_rxpos == 0) {
      if (b != SCAN_HEADER) continue;
    }

    tof_rxbuf[tof_rxpos++] = b;
    if (tof_rxpos != SCAN_BYTES) continue;

    uint8_t c = xor8(tof_rxbuf, SCAN_BYTES - 1);
    if (c == tof_rxbuf[SCAN_BYTES - 1]) {
      accept_scan_frame(tof_rxbuf);
    }
    tof_rxpos = 0;
  }
}

// ----------------------------- Altitude estimate -----------------------
static const char* alt_src_name(AltSrc s) {
  switch (s) {
    case ALT_SRC_LPOS:            return "LPOS";
    case ALT_SRC_DISTANCE_SENSOR: return "DIST";
    case ALT_SRC_RANGEFINDER:     return "RF";
    default:                      return "NONE";
  }
}

static void update_alt_estimate(void) {
  uint64_t t = now_ms();
  bool lpos_ok = lpos_fresh(t) && isfinite(lpos_alt_filt_m);
  bool rf_ok = rf_fresh(t) && isfinite(rangefinder_m);

  float max_alt = NAN;
  if (lpos_ok) {
    float a = lpos_alt_filt_m;
    if (a < -1.0f) a = -1.0f;
    if (a > 50.0f) a = 50.0f;
    max_alt = a;
  }
  if (rf_ok) {
    float rf = rangefinder_m;
    if (rf < 0.0f) rf = 0.0f;
    if (rf > 10.0f) rf = 10.0f;
    max_alt = isnan(max_alt) ? rf : fmaxf(max_alt, rf);
  }
  alt_max_m = max_alt;

  alt_rf_rejected = false;
  alt_est_m = NAN;
  alt_src = ALT_SRC_NONE;

  if (rf_ok) {
    float rf = rangefinder_m;
    bool sane = true;
    if (lpos_ok && fabsf(rf - lpos_alt_filt_m) > 0.80f) sane = false;
    if (sane) {
      alt_est_m = rf;
      alt_src = (rf_src == RF_SRC_DISTANCE_SENSOR) ? ALT_SRC_DISTANCE_SENSOR : ALT_SRC_RANGEFINDER;
    } else {
      alt_rf_rejected = true;
    }
  }

  if (alt_src == ALT_SRC_NONE && lpos_ok) {
    alt_est_m = lpos_alt_filt_m;
    alt_src = ALT_SRC_LPOS;
  }
}

// ----------------------------- Logging ---------------------------------
static void log_init(void) {
  scan_fp = fopen(LOG_SCAN_PATH, "ab");
  if (!scan_fp) {
    fprintf(stderr, "WARN: cannot open %s: %s\n", LOG_SCAN_PATH, strerror(errno));
  } else {
    fseek(scan_fp, 0, SEEK_END);
    long sz = ftell(scan_fp);
    if (sz <= 0) {
      const char hdr[] = "SCLOG3\n";
      fwrite(hdr, 1, sizeof(hdr) - 1, scan_fp);
      fflush(scan_fp);
    }
  }

  txt_log_fp = fopen(LOG_TXT_PATH, "a");
  if (!txt_log_fp) {
    fprintf(stderr, "WARN: cannot open %s: %s\n", LOG_TXT_PATH, strerror(errno));
  } else {
    fprintf(txt_log_fp, "\n--- MANUAL MAPPING SESSION START ---\n");
    fflush(txt_log_fp);
  }

  last_flush_ms = now_ms();
}

static void log_flush_if_due(uint64_t t) {
  if ((t - last_flush_ms) < LOG_FLUSH_MS) return;
  last_flush_ms = t;
  if (scan_fp) fflush(scan_fp);
  if (txt_log_fp) fflush(txt_log_fp);
}

typedef struct __attribute__((packed)) {
  uint32_t magic;      // 'SCN3'
  uint64_t host_ms;
  uint32_t scan_ms;
  uint32_t custom_mode;

  float x_m;
  float y_m;
  float yaw_deg;
  float alt_m;
  float alt_max_m;

  float roll_rad;
  float pitch_rad;

  float vx_mps;
  float vy_mps;
  float vz_mps;

  float lpos_alt_m;
  float lpos_alt_filt_m;
  float rf_m;
  float rf_v;
  float of_rate_x;
  float of_rate_y;
  float of_comp_m_x;
  float of_comp_m_y;
  float of_ground_m;

  uint32_t sys_present;
  uint32_t sys_health;
  uint32_t sys_enabled;

  uint16_t att_age_ms;
  uint16_t lpos_age_ms;
  uint16_t of_age_ms;
  uint16_t rf_age_ms;
  uint16_t hb_age_ms;
  uint16_t pose_flags;

  uint8_t  of_q;
  uint8_t  alt_src;
  uint8_t  rf_src;
  uint8_t  of_src;
  uint8_t  fc_armed;
  uint8_t  alt_rf_rejected;
  uint8_t  ds_orientation;
  uint8_t  ds_id;
  uint16_t ds_cur_cm;

  uint8_t  grid_raw[512];
} scanrec_t;

static void log_scan_record(void) {
  if (!scan_new) return;

  uint64_t now = now_ms();
  uint64_t sample_ms = last_scan_host_ms;
  if (sample_ms == 0) {
    scan_new = false;
    return;
  }

  if (!scan_fp) {
    scan_new = false;
    return;
  }

  if (!mapping_enabled || (!allow_disarmed_logging && !fc_armed)) {
    scan_new = false;
    return;
  }

  if (!pose_valid_for_mapping(sample_ms)) {
    if ((now - last_scan_host_ms) > SCAN_PENDING_MAX_MS) {
      static uint64_t last_drop_log_ms = 0;
      if ((now - last_drop_log_ms) > 1000) {
        last_drop_log_ms = now;
        printf("Dropping scan: pose invalid at scan time (alt_src=%s flags=0x%02x)\n",
               alt_src_name(alt_src),
               (unsigned)pose_flags_for_time(sample_ms));
      }
      scan_new = false;
    }
    return;
  }

  pose_sample_t pose;
  if (!pose_ring_sample_at(sample_ms, &pose)) {
    memset(&pose, 0, sizeof(pose));
    pose.host_ms = sample_ms;
    pose.x_m = lpos_x_m;
    pose.y_m = lpos_y_m;
    pose.vx_mps = lpos_vx_mps;
    pose.vy_mps = lpos_vy_mps;
    pose.vz_mps = lpos_vz_mps;
    pose.alt_m = alt_est_m;
    pose.lpos_alt_m = lpos_alt_m;
    pose.lpos_alt_filt_m = lpos_alt_filt_m;
    pose.yaw_deg = current_heading_deg();
    pose.roll_rad = roll_rad;
    pose.pitch_rad = pitch_rad;
    pose.alt_src = (uint8_t)alt_src;
    pose.valid_xy = 1u;
    pose.valid_att = 1u;
  }

  scanrec_t r;
  memset(&r, 0, sizeof(r));
  r.magic = 0x334E4353u; // 'SCN3'
  r.host_ms = last_scan_host_ms;
  r.scan_ms = last_scan_t_ms;
  r.custom_mode = hb_custom_mode;

  r.x_m = pose.x_m;
  r.y_m = pose.y_m;
  r.yaw_deg = pose.yaw_deg;
  r.alt_m = pose.alt_m;
  r.alt_max_m = alt_max_m;

  r.roll_rad = pose.roll_rad;
  r.pitch_rad = pose.pitch_rad;
  r.vx_mps = pose.vx_mps;
  r.vy_mps = pose.vy_mps;
  r.vz_mps = pose.vz_mps;
  r.lpos_alt_m = pose.lpos_alt_m;
  r.lpos_alt_filt_m = pose.lpos_alt_filt_m;

  r.rf_m = rf_fresh(sample_ms) ? rangefinder_m : NAN;
  r.rf_v = rangefinder_v;
  r.of_rate_x = of_rate_x;
  r.of_rate_y = of_rate_y;
  r.of_comp_m_x = of_comp_m_x;
  r.of_comp_m_y = of_comp_m_y;
  r.of_ground_m = of_ground_m;

  r.sys_present = have_sys ? sys_present : 0;
  r.sys_health = have_sys ? sys_health : 0;
  r.sys_enabled = have_sys ? sys_enabled : 0;

  r.att_age_ms = sat_u16_age(sample_ms, att_last_update_ms);
  r.lpos_age_ms = sat_u16_age(sample_ms, lpos_last_update_ms);
  r.of_age_ms = sat_u16_age(sample_ms, of_last_update_ms);
  r.rf_age_ms = sat_u16_age(sample_ms, rangefinder_last_update_ms);
  r.hb_age_ms = sat_u16_age(sample_ms, last_hb_ms);
  r.pose_flags = pose_flags_for_time(sample_ms);

  r.of_q = of_fresh(sample_ms) ? of_quality : 0;
  r.alt_src = pose.alt_src;
  r.rf_src = (uint8_t)rf_src;
  r.of_src = (uint8_t)of_src;
  r.fc_armed = fc_armed ? 1u : 0u;
  r.alt_rf_rejected = alt_rf_rejected ? 1u : 0u;
  r.ds_orientation = ds_orientation;
  r.ds_id = ds_id;
  r.ds_cur_cm = ds_cur_cm;

  memcpy(r.grid_raw, last_scan_grid_raw, sizeof(r.grid_raw));
  fwrite(&r, 1, sizeof(r), scan_fp);
  scan_new = false;
}

// ----------------------------- Service loop ----------------------------
static void service_tick(void) {
  uint64_t t = now_ms();

  send_own_heartbeat_tick(t);

  if (have_fc && !streams_requested) {
    request_streams();
  }

  if (have_fc && !fc_link_fresh(t) && last_hb_ms != 0) {
    printf("FC heartbeat stale; waiting for reconnection.\n");
    have_fc = false;
    streams_requested = false;
  }

  update_alt_estimate();
  pose_ring_push(t);
  log_scan_record();
  log_flush_if_due(t);

  static uint64_t last_status_ms = 0;
  if ((t - last_status_ms) >= STATUS_PRINT_MS) {
    last_status_ms = t;
    printf("STATUS armed=%d map=%d gate=%d x=%.2f y=%.2f yaw=%.1f alt=%.2f alt_src=%s of_q=%u rf=%.2f scan=%d mode=%u\n",
           fc_armed ? 1 : 0,
           mapping_enabled ? 1 : 0,
           mapping_gate(t) ? 1 : 0,
           lpos_x_m, lpos_y_m,
           have_att ? current_heading_deg() : NAN,
           alt_est_m,
           alt_src_name(alt_src),
           (unsigned)(of_fresh(t) ? of_quality : 0),
           rangefinder_m,
           scan_new ? 1 : 0,
           (unsigned)hb_custom_mode);
  }
}

// ----------------------------- main ------------------------------------
int main(int argc, char** argv) {
  setvbuf(stdout, NULL, _IOLBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--sysid") == 0 && (i + 1) < argc) {
      int v = atoi(argv[i + 1]);
      if (v > 0 && v < 255) {
        g_sysid = (uint8_t)v;
      }
      i++;
    } else if (strcmp(argv[i], "--log-when-disarmed") == 0) {
      allow_disarmed_logging = true;
    }
  }

  log_init();

  printf("MODE=MANUAL_MAPPING_LOGGER\n");
  printf("LOG: SCAN=%s TXT=%s allow_disarmed=%d sysid=%u\n",
         LOG_SCAN_PATH,
         LOG_TXT_PATH,
         allow_disarmed_logging ? 1 : 0,
         (unsigned)g_sysid);

  fc_fd = open_uart(FC_UART, FC_BAUD);
  if (fc_fd < 0) return 1;
  printf("Opened FC UART: %s @%d\n", FC_UART, FC_BAUD);

  tof_fd = open_uart(TOF_UART, TOF_BAUD);
  if (tof_fd < 0) return 1;
  printf("Opened ToF UART: %s @%d\n", TOF_UART, TOF_BAUD);

  while (1) {
    struct pollfd pfd[2];
    pfd[0].fd = fc_fd;
    pfd[0].events = POLLIN;
    pfd[1].fd = tof_fd;
    pfd[1].events = POLLIN;

    poll(pfd, 2, 20);

    if (pfd[0].revents & POLLIN) pump_fc_uart();
    if (pfd[1].revents & POLLIN) pump_tof_uart();

    service_tick();
  }

  return 0;
}
