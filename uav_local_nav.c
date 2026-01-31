// uav_fc_tof_nav.c  (stability-first “ETH-ish” single-drone behavior)
//
// CHANGELOG (critical behavior fixes):
// 1) If want_arm goes false at ANY time, we now enter LANDING/DISARMING (no more stuck in TAKEOFF).
// 2) Proper GUIDED takeoff: MAV_CMD_NAV_TAKEOFF is issued once after arming.
// 3) Disarm: normal disarm first; if still armed while essentially on-ground, we force disarm (param2=21196).
// 4) Send our own MAVLink HEARTBEAT at 1 Hz (helps with some failsafe configs).
// 5) Print STATUSTEXT from FC immediately (and flush).
//
// NEW in this revision (takeoff-not-starting fixes):
// A) After NAV_TAKEOFF, do NOT send SET_POSITION_TARGET_LOCAL_NED velocity commands for ~2s (or until takeoff starts).
// B) Print EXTENDED_SYS_STATE.landed_state every loop.
// C) If NAV_TAKEOFF ACCEPTED but mot_avg doesn't rise > ~1150 within ~1–2s: print "takeoff not started"
//    and enter conservative SET_ATTITUDE_TARGET thrust ramp until RF>0.15m OR landed_state != ON_GROUND.
// D) Cache last STATUSTEXT and re-print it on unexpected disarm for context.
// E) Fix compile: use mavlink_msg_set_attitude_target_encode (pack signature varies with extensions).
// F) Fix MAP_W/H as true integer constants (avoid VLA-at-file-scope warning).
//
// NEW (drift/flip fix):
// G) Never command LOCAL_NED translation/position hold until XY is stable.
//    - Add vel_xy_allowed() + XY_STABLE_HOLD_MS timer.
//    - In HOVER/EXPLORE/TURNING-pause: if XY not stable -> BODY-frame zero velocity + yaw hold only.
//    - Delay hover position lock until XY stable for >= 1s.
//    - Keep LANDING/CEILING vertical commands in LOCAL_NED (world vertical).




//IMPORTANT VARIABLES:

//PRINT_HZ is a macro that controls how often status is printed to console
//HOVER_TEST_ONLY when true makes the UAV only hover in place after takeoff for testing purposes
//PRINT_LANDED_STATE_EACH_TICK when set to 1 makes the code print the landed state every tick for debugging

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

#include "common/mavlink.h"

// ----------------------------- UART config -----------------------------
#define FC_UART  "/dev/ttyS2"
#define FC_BAUD  57600

#define TOF_UART "/dev/ttyS1"
#define TOF_BAUD 115200

// ----------------------------- ArduPilot RANGEFINDER -------------------
#ifndef MAVLINK_MSG_ID_RANGEFINDER
#define MAVLINK_MSG_ID_RANGEFINDER 173
#endif

// MAV_SENSOR_ORIENTATION enum uses ROTATION_*_FACING values.
// Downward-facing is commonly 25.
#define ORIENT_DOWNWARD_FACING 25

// ----------------------------- Logging paths ---------------------------
#define LOG_CSV_PATH      "/mnt/sdcard/navlog.csv"
#define LOG_SCAN_PATH     "/mnt/sdcard/scanlog.bin"
#define LOG_HZ            20
#define LOG_FLUSH_MS      1000

static FILE* log_fp  = NULL;
static FILE* scan_fp = NULL;
static uint64_t last_log_ms   = 0;
static uint64_t last_flush_ms = 0;

// ----------------------------- ToF frame ------------------------------
#define SCAN_HEADER 0xA5
#define NUM_SENSORS 4
#define GRID_SIZE   64
#define TOTAL_CELLS (NUM_SENSORS * GRID_SIZE)
#define SCAN_BYTES  (1 + 4 + (TOTAL_CELLS * 2) + 1) // 518

static uint8_t tof_rxbuf[SCAN_BYTES];
static int     tof_rxpos = 0;

// Control frames from ESP32 hub (ARM/DISARM pass-through) share the same UART stream:
#define CTRL_HEADER 0xA6
#define CTRL_BYTES  7

static uint8_t ctrl_rxbuf[CTRL_BYTES];
static int     ctrl_rxpos = 0;

// Last-good ToF frame for offline mapping
static bool     have_scan_frame = false;
static uint32_t last_scan_t_ms  = 0;
static uint64_t last_scan_host_ms = 0;
static uint8_t  last_scan_grid_raw[TOTAL_CELLS * 2]; // 512 bytes (LE u16 mm), physical order F/R/B/L
static volatile bool scan_new = false;   // set on receive, consumed by logger + mapper

// Physical order in the packet is FRONT, RIGHT, BACK, LEFT
enum Dir { D_FRONT=0, D_RIGHT=1, D_BACK=2, D_LEFT=3 };

// ----------------------------- ToF processing --------------------------
#define TOF_COLS 8
#define TOF_ROWS 8

static float tof_beams_m[4][TOF_COLS];       // [dir][col] distance (m)
static float tof_min_m[4] = { NAN, NAN, NAN, NAN };   // min across beams (per dir)
static float tof_filt_m[4] = { NAN, NAN, NAN, NAN };  // filtered dir-min

// -------------------------- Stability-first params ---------------------
static const float TAKEOFF_TARGET_M = 0.50f;
static const float CEIL_M = 0.70f;

// ToF constraints
static const float TOF_MAX_RANGE_M = 4.00f;
static const float TOF_FOV_DEG     = 63.0f;

// How close is "too close" (stop & turn)
static const float FRONT_STOP_M = 0.60f;
static const float SIDE_SAFE_M  = 0.80f;

// Exploration forward speed (m/s) and max strafe (m/s)
static const float FWD_VEL = 0.35f;
static const float STRAFE_VEL = 0.20f;

// Gentle yaw turn rate
static const float YAW_RATE_DPS = 20.0f;

// ---------------------- Liftoff assist (no-RC bootstrap) ---------------
#define RC_CH_ROLL     1
#define RC_CH_PITCH    2
#define RC_CH_THROTTLE 3
#define RC_CH_YAW      4

static const uint16_t RC_NEUTRAL_US = 1500;
static const uint16_t ASSIST_THR_US_MIN = 1300;   // tune per craft
static const uint16_t ASSIST_THR_US_MAX = 1600;   // tune per craft
static const uint64_t ASSIST_SEND_PERIOD_MS = 50; // 20 Hz
static const uint64_t ASSIST_TOTAL_MS = 1800;     // throttle window
static const float    ASSIST_EXIT_ALT_M = 0.28f;  // consider airborne if above this
static const uint64_t ASSIST_ABORT_MS = 2600;     // if not airborne by then -> disarm
static const uint64_t ASSIST_OVERRIDE_EFFECT_MS = 400;
static const float    ASSIST_MOTOR_DELTA_MIN = 40.0f; // us delta avg motor outputs to consider override effective

// ---------------------- TAKEOFF "not started" gating + thrust ramp -----
#define PRINT_LANDED_STATE_EACH_TICK 1

static const uint64_t TAKEOFF_NO_VEL_MS         = 2000;   // hold-off SET_POSITION_TARGET_LOCAL_NED after NAV_TAKEOFF
static const float    TAKEOFF_MOT_START_US      = 1150.0f; // mot_avg threshold meaning “spool started”
static const uint64_t TAKEOFF_START_CHECK_MS    = 1500;   // after ACCEPTED, if mot_avg not > threshold => fallback

static const uint64_t TO_RAMP_SEND_MS           = 50;     // 20 Hz attitude target
static const uint64_t TO_RAMP_TOTAL_MS          = 1800;   // ramp duration
static const uint64_t TO_RAMP_ABORT_MS          = 2600;   // abort to other fallback if still not off ground
static const float    TO_RAMP_THR_MIN           = 0.15f;  // conservative
static const float    TO_RAMP_THR_MAX           = 0.60f;  // conservative cap
static const float    TO_RAMP_EXIT_M            = 0.15f;  // exit once RF > 0.15m or landed_state != ON_GROUND

static uint64_t takeoff_accept_ms               = 0;
static uint64_t takeoff_no_vel_until_ms         = 0;
static bool     takeoff_started                 = false;
static uint64_t takeoff_started_ms              = 0;
static bool     takeoff_not_started_printed     = false;
static bool     takeoff_thr_ramp_active         = false;
static uint64_t takeoff_thr_ramp_start_ms       = 0;
static uint64_t takeoff_thr_ramp_last_send_ms   = 0;

// ----------------------------- Battery safety (2S LiHV) ----------------
static const float    BATT_ARM_MIN_VPC   = 3.70f; // refuse arm if below (on ground)
static const float    BATT_LAND_VPC      = 3.55f; // start LAND if below (in flight, sustained)
static const float    BATT_EMERG_VPC     = 3.35f; // emergency LAND/DISARM behavior if sustained
static const uint64_t BATT_LOW_HOLD_MS   = 1200;  // require sustained low for this long
static const uint64_t BATT_FRESH_MS      = 2000;  // battery message freshness window
static uint64_t batt_last_ms             = 0;
static uint64_t batt_low_since_ms        = 0;
static uint64_t batt_emerg_since_ms      = 0;
static uint64_t batt_last_warn_ms        = 0;

// ----------------------------- Mapping ---------------------------------
#define MAP_RES_M   0.10f
#define MAP_SIZE_M  50.0f
// NOTE: MAP_W/H must be integer constant expressions (no float math) for file-scope arrays.
#define MAP_W       500
#define MAP_H       500

static int8_t occ_grid[MAP_W * MAP_H];     // log-odds grid (0 unknown)
static int8_t occ_grid_tmp[MAP_W * MAP_H]; // for recentering shifts
static bool   map_inited = false;
static float  map_origin_x = NAN; // world NED x at map center
static float  map_origin_y = NAN; // world NED y at map center

static const int8_t LO_FREE_DEC = 1;
static const int8_t LO_OCC_INC  = 6;
static const int8_t LO_MIN = -80;
static const int8_t LO_MAX =  80;

static inline int8_t clamp_lo(int v) {
  if (v < LO_MIN) return LO_MIN;
  if (v > LO_MAX) return LO_MAX;
  return (int8_t)v;
}

static inline bool world_to_grid(float x, float y, int* gx, int* gy) {
  if (!map_inited) return false;
  float dx = x - map_origin_x;
  float dy = y - map_origin_y;
  int ix = (int)lrintf(dx / MAP_RES_M) + (MAP_W / 2);
  int iy = (int)lrintf(dy / MAP_RES_M) + (MAP_H / 2);
  if (ix < 0 || ix >= MAP_W || iy < 0 || iy >= MAP_H) return false;
  *gx = ix; *gy = iy;
  return true;
}

static inline int idx(int gx, int gy) { return gy * MAP_W + gx; }

// Keyframe flags (stored in scan records)
#define KF_NONE         0
#define KF_TAKEOFF      (1u<<0)
#define KF_TURN_START   (1u<<1)
#define KF_TURN_END     (1u<<2)
#define KF_LAND_START   (1u<<3)
#define KF_LIFTOFF_AST  (1u<<4)
#define KF_MAP_RECENTER (1u<<5)
#define KF_BATT_LAND    (1u<<6)
#define KF_BATT_EMERG   (1u<<7)

static uint8_t pending_kf_flags = 0;

// Frontier-ish evaluation
static const uint64_t FRONTIER_EVAL_MS = 1200;
static uint64_t last_frontier_eval_ms = 0;
static bool turning_dir_forced = false;
static int  forced_turn_dir = D_RIGHT;

// Post-turn settle pause
static const uint64_t POST_TURN_PAUSE_MS = 450;
static uint64_t explore_pause_until_ms = 0;

static void raycast_update(float x0, float y0, float x1, float y1, bool hit_occ) {
  int x0g, y0g, x1g, y1g;
  if (!world_to_grid(x0, y0, &x0g, &y0g)) return;
  if (!world_to_grid(x1, y1, &x1g, &y1g)) return;

  int dx = abs(x1g - x0g);
  int sx = (x0g < x1g) ? 1 : -1;
  int dy = -abs(y1g - y0g);
  int sy = (y0g < y1g) ? 1 : -1;
  int err = dx + dy;

  int x = x0g, y = y0g;

  while (1) {
    bool is_end = (x == x1g && y == y1g);
    int k = idx(x, y);

    if (!is_end) {
      int v = (int)occ_grid[k] - LO_FREE_DEC;
      occ_grid[k] = clamp_lo(v);
    } else {
      if (hit_occ) {
        int v = (int)occ_grid[k] + LO_OCC_INC;
        occ_grid[k] = clamp_lo(v);
      } else {
        int v = (int)occ_grid[k] - (LO_FREE_DEC / 2);
        occ_grid[k] = clamp_lo(v);
      }
      break;
    }

    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x += sx; }
    if (e2 <= dx) { err += dx; y += sy; }

    if (x < 0 || x >= MAP_W || y < 0 || y >= MAP_H) break;
  }
}

static void map_update_from_beams(float x_m, float y_m, float yaw_deg) {
  if (!map_inited) return;

  static const float dir_center_deg[4] = { 0.0f, 90.0f, 180.0f, -90.0f };
  const float half_fov = TOF_FOV_DEG * 0.5f;

  for (int d = 0; d < 4; d++) {
    for (int c = 0; c < TOF_COLS; c++) {
      float dist = tof_beams_m[d][c];
      if (isnan(dist)) continue;
      if (dist <= 0.05f) continue;

      bool hit_occ = (dist < (TOF_MAX_RANGE_M - 0.05f));
      if (dist > TOF_MAX_RANGE_M) dist = TOF_MAX_RANGE_M;

      float u = ((float)c - 3.5f) / 3.5f;
      float col_off = u * half_fov;

      float ang_deg = yaw_deg + dir_center_deg[d] + col_off;
      float ang = ang_deg * ((float)M_PI / 180.0f);
      float ex = x_m + dist * cosf(ang);
      float ey = y_m + dist * sinf(ang);

      raycast_update(x_m, y_m, ex, ey, hit_occ);
    }
  }
}

static void map_recenter_shift(int sx_cells, int sy_cells) {
  memset(occ_grid_tmp, 0, sizeof(occ_grid_tmp));

  for (int y = 0; y < MAP_H; y++) {
    int sy = y + sy_cells;
    if (sy < 0 || sy >= MAP_H) continue;
    for (int x = 0; x < MAP_W; x++) {
      int sx = x + sx_cells;
      if (sx < 0 || sx >= MAP_W) continue;
      occ_grid_tmp[idx(x,y)] = occ_grid[idx(sx,sy)];
    }
  }

  memcpy(occ_grid, occ_grid_tmp, sizeof(occ_grid));
}

static void map_recentre_if_needed(float x_m, float y_m) {
  if (!map_inited) return;

  const float half = MAP_SIZE_M * 0.5f;        // 25m
  const float thresh = half * 0.60f;           // 15m
  float dx = x_m - map_origin_x;
  float dy = y_m - map_origin_y;

  if (fabsf(dx) < thresh && fabsf(dy) < thresh) return;

  int sx_cells = (int)lrintf(dx / MAP_RES_M);
  int sy_cells = (int)lrintf(dy / MAP_RES_M);

  const int max_shift = (int)(half / MAP_RES_M * 0.5f); // <= ~125 cells
  if (sx_cells >  max_shift) sx_cells =  max_shift;
  if (sx_cells < -max_shift) sx_cells = -max_shift;
  if (sy_cells >  max_shift) sy_cells =  max_shift;
  if (sy_cells < -max_shift) sy_cells = -max_shift;

  if (sx_cells == 0 && sy_cells == 0) return;

  map_recenter_shift(sx_cells, sy_cells);

  map_origin_x += (float)sx_cells * MAP_RES_M;
  map_origin_y += (float)sy_cells * MAP_RES_M;

  pending_kf_flags |= KF_MAP_RECENTER;
  printf("Map recenter: shift (%d,%d) cells => new origin (%.2f,%.2f)\n",
         sx_cells, sy_cells, map_origin_x, map_origin_y);
}

// Frontier scoring: count unknown/free/occupied along short rays.
static int frontier_score_dir(float x_m, float y_m, float yaw_deg, float offset_deg) {
  if (!map_inited) return 0;

  static const float ray_offs_deg[3] = { 0.0f, 15.0f, -15.0f };
  const float max_range = 2.5f;
  const float step = MAP_RES_M * 2.0f;

  int unknown = 0, freec = 0, occ = 0;

  for (int r = 0; r < 3; r++) {
    float ang = (yaw_deg + offset_deg + ray_offs_deg[r]) * ((float)M_PI / 180.0f);
    float ca = cosf(ang);
    float sa = sinf(ang);

    for (float d = step; d <= max_range; d += step) {
      float px = x_m + d * ca;
      float py = y_m + d * sa;
      int gx, gy;
      if (!world_to_grid(px, py, &gx, &gy)) break;
      int8_t v = occ_grid[idx(gx,gy)];

      if (v >= -1 && v <= 1) unknown++;
      else if (v > 10) occ++;
      else if (v < -10) freec++;
    }
  }

  int score = unknown * 3 + freec * 1 - occ * 4;
  return score;
}

// ----------------------------- MAVLink state ---------------------------
static int fc_fd  = -1;
static int tof_fd = -1;

// Our sysid/compid
static uint8_t g_sysid  = 255; // GCS-like
static const uint8_t g_compid = MAV_COMP_ID_ONBOARD_COMPUTER;

// FC sysid/compid learned from heartbeat
static uint8_t fc_sysid = 0;
static uint8_t fc_compid = 0;
static bool have_fc = false;

static uint64_t last_hb_ms = 0;
static uint32_t hb_custom_mode = 0;
static bool fc_armed = false;
static bool fc_armed_prev = false;

// COMMAND_ACK tracking (last seen)
static bool     have_ack = false;
static uint16_t last_ack_cmd = 0;
static uint8_t  last_ack_res = 0;

// Takeoff ACK tracking
static bool     have_takeoff_ack = false;
static uint8_t  takeoff_ack_res = 0;
static uint64_t takeoff_ack_ms  = 0;

// Landed state
static uint8_t landed_state = MAV_LANDED_STATE_UNDEFINED;
static bool have_ext = false;

// Battery
static float batt_v_total = NAN;
static float batt_vpc = NAN;
static int   batt_cells = 0;

// LOCAL_POSITION_NED altitude
static bool  have_lpos = false;
static float lpos_alt_m = NAN;        // positive up
static float lpos_alt_filt_m = NAN;
static uint64_t lpos_last_update_ms = 0;

// EKF local position (NED)
static bool  have_xy = false;
static float lpos_x_m = NAN;
static float lpos_y_m = NAN;
static float lpos_vx_mps = NAN;
static float lpos_vy_mps = NAN;
static float lpos_vz_mps = NAN;

// Optical flow
static bool     have_of = false;
static uint8_t  of_quality = 0;
static float    of_comp_m_x = NAN;
static float    of_comp_m_y = NAN;
static float    of_ground_m = NAN;
static float    of_rate_x   = NAN;
static float    of_rate_y   = NAN;
static uint64_t of_last_update_ms = 0;

// Attitude
static bool  have_att = false;
static float roll_rad = 0.0f;
static float pitch_rad = 0.0f;
static float yaw_rad = 0.0f;

// Downward-facing rangefinder
static bool     have_rangefinder = false;
static float    rangefinder_m = NAN;
static float    rangefinder_v = NAN;
static uint64_t rangefinder_last_update_ms = 0;

// Debug: last DISTANCE_SENSOR seen
static bool     have_ds = false;
static uint8_t  ds_id = 0;
static uint8_t  ds_orientation = 0;
static uint16_t ds_cur_cm = 0;
static uint64_t ds_last_update_ms = 0;

// SYS_STATUS health
static bool     have_sys = false;
static uint32_t sys_present = 0;
static uint32_t sys_enabled = 0;
static uint32_t sys_health  = 0;
static uint64_t sys_last_ms = 0;

// SERVO_OUTPUT_RAW (motor outputs proxy)
static bool     have_servo = false;
static uint16_t servo_raw[8] = {0};
static uint64_t servo_last_ms = 0;

// STATUSTEXT cache (for disarm context)
static char     last_statustext[64] = {0};
static uint8_t  last_statustext_sev = 0;
static uint64_t last_statustext_ms  = 0;

// ----------------------------- State machine ---------------------------
typedef enum {
  ST_WAIT_LINK = 0,
  ST_IDLE,
  ST_ARMING,
  ST_TAKEOFF,
  ST_LIFTOFF_ASSIST,
  ST_HOVER,
  ST_EXPLORE,
  ST_TURNING,
  ST_LANDING,
  ST_DISARMING
} State;

static const char* state_name(State s) {
  switch (s) {
    case ST_WAIT_LINK: return "WAIT_LINK";
    case ST_IDLE: return "IDLE";
    case ST_ARMING: return "ARMING";
    case ST_TAKEOFF: return "TAKEOFF";
    case ST_LIFTOFF_ASSIST: return "LIFTOFF_ASSIST";
    case ST_HOVER: return "HOVER";
    case ST_EXPLORE: return "EXPLORE";
    case ST_TURNING: return "TURNING";
    case ST_LANDING: return "LANDING";
    case ST_DISARMING: return "DISARMING";
    default: return "?";
  }
}

static State st = ST_WAIT_LINK;
static bool want_arm = false;

// Yaw target lock
static bool  have_yaw_target = false;
static float yaw_target_deg = 0.0f;

// Hover-hold test mode
static const bool HOVER_TEST_ONLY = false;
static bool  hover_hold_valid = false;
static float hover_hold_x_m = NAN;
static float hover_hold_y_m = NAN;
static float hover_hold_z_ned_m = NAN;
static float hover_hold_yaw_deg = NAN;
static uint64_t hover_hold_set_ms = 0;
static uint64_t hover_enter_ms = 0;

// Turning planner
static bool     turning_init = false;
static int      turning_dir  = D_RIGHT;
static float    turn_target_deg = 0.0f;
static uint64_t turn_start_ms = 0;

// Ceiling logic
static bool ceiling_active = false;
static float alt_est_m = NAN;

typedef enum {
  ALT_SRC_NONE = 0,
  ALT_SRC_LPOS,
  ALT_SRC_RANGEFINDER,
  ALT_SRC_ON_GROUND
} AltSrc;

static AltSrc alt_src = ALT_SRC_NONE;

// Takeoff command latch
static bool     takeoff_sent = false;
static uint64_t takeoff_sent_ms = 0;

// Landing latch
static bool     land_mode_sent = false;
static uint64_t land_mode_sent_ms = 0;

// Rate limit for spammy commands
static uint64_t last_arm_cmd_ms = 0;
static uint64_t last_mode_cmd_ms = 0;
static uint64_t last_disarm_cmd_ms = 0;

// Liftoff assist internals
static uint64_t assist_start_ms = 0;
static uint64_t assist_last_send_ms = 0;
static float    assist_alt0 = NAN;
static bool     assist_baseline_set = false;
static float    assist_motor_avg0 = NAN;
static bool     assist_warned_override = false;

// Disarm timer fix (single global)
static uint64_t disarm_start_ms = 0;

// ----------------------------- Timing helpers --------------------------
#define PRINT_HZ 2
static uint64_t now_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static float rad2deg(float r) { return r * (180.0f / (float)M_PI); }
static float deg2rad(float d) { return d * ((float)M_PI / 180.0f); }

static float wrap_deg(float d) {
  while (d >= 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

static float current_heading_deg(void) {
  return wrap_deg(rad2deg(yaw_rad));
}

static const char* landed_state_name(uint8_t s) {
  switch (s) {
    case MAV_LANDED_STATE_UNDEFINED: return "UNDEFINED";
    case MAV_LANDED_STATE_ON_GROUND: return "ON_GROUND";
    case MAV_LANDED_STATE_IN_AIR:    return "IN_AIR";
    case MAV_LANDED_STATE_TAKEOFF:   return "TAKEOFF";
    case MAV_LANDED_STATE_LANDING:   return "LANDING";
    default: return "?";
  }
}

// ----------------------------- UART open -------------------------------
static int open_uart(const char* dev, int baud) {
  int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    fprintf(stderr, "open(%s) failed: %s\n", dev, strerror(errno));
    return -1;
  }

  struct termios tio;
  memset(&tio, 0, sizeof(tio));
  if (tcgetattr(fd, &tio) != 0) {
    fprintf(stderr, "tcgetattr(%s) failed: %s\n", dev, strerror(errno));
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
    default: sp = B57600; break;
  }
  cfsetispeed(&tio, sp);
  cfsetospeed(&tio, sp);

  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    fprintf(stderr, "tcsetattr(%s) failed: %s\n", dev, strerror(errno));
    close(fd);
    return -1;
  }

  return fd;
}

// ----------------------------- MAVLink send ----------------------------
static void mav_send(const mavlink_message_t* msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t n = mavlink_msg_to_send_buffer(buf, msg);
  if (fc_fd >= 0) (void)write(fc_fd, buf, n);
}

static void send_command_long_tgt(uint8_t tgt_sys, uint8_t tgt_comp, uint16_t cmd,
                                  float p1,float p2,float p3,float p4,float p5,float p6,float p7) {
  mavlink_message_t m;
  mavlink_msg_command_long_pack(
      g_sysid, g_compid, &m,
      tgt_sys, tgt_comp,
      cmd, 0,
      p1,p2,p3,p4,p5,p6,p7
  );
  mav_send(&m);
}

static void send_command_long(uint16_t cmd,
                              float p1,float p2,float p3,float p4,float p5,float p6,float p7) {
  send_command_long_tgt(fc_sysid, fc_compid, cmd, p1,p2,p3,p4,p5,p6,p7);
}

static void send_request_data_stream(uint8_t tgt_sys, uint8_t tgt_comp,
                                     uint8_t stream_id, uint16_t rate_hz, uint8_t start_stop) {
  mavlink_message_t m;
  mavlink_msg_request_data_stream_pack(
      g_sysid, g_compid, &m,
      tgt_sys, tgt_comp,
      stream_id, rate_hz, start_stop
  );
  mav_send(&m);
}

// ----------------------------- Our heartbeat ---------------------------
static void send_own_heartbeat_tick(uint64_t t) {
  static uint64_t last_ms = 0;
  if (t - last_ms < 1000) return;
  last_ms = t;

  mavlink_message_t m;
  mavlink_msg_heartbeat_pack(
      g_sysid, g_compid, &m,
      MAV_TYPE_ONBOARD_CONTROLLER,
      MAV_AUTOPILOT_INVALID,
      0, 0,
      MAV_STATE_ACTIVE
  );
  mav_send(&m);
}

// ----------------------------- MAVLink control -------------------------
static void set_mode_custom(uint32_t custom_mode) {
  if (!have_fc) return;

  uint64_t t = now_ms();
  if (t - last_mode_cmd_ms < 800) return;
  last_mode_cmd_ms = t;

  {
    mavlink_message_t m;
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    mavlink_msg_set_mode_pack(g_sysid, g_compid, &m, fc_sysid, base_mode, custom_mode);
    mav_send(&m);
  }

  send_command_long(MAV_CMD_DO_SET_MODE,
                    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)custom_mode, 0,0,0,0,0);
}

static void set_mode_guided(void) {
  printf("Requesting mode GUIDED ... (current=%u)\n", (unsigned)hb_custom_mode);
  set_mode_custom(4);
}

static void set_mode_land(void) {
  printf("Requesting mode LAND ... (current=%u)\n", (unsigned)hb_custom_mode);
  set_mode_custom(9);
}

static void set_mode_stabilize(void) {
  printf("Requesting mode STABILIZE ... (current=%u)\n", (unsigned)hb_custom_mode);
  set_mode_custom(0);
}

static void arm_fc(void) {
  if (!have_fc) return;

  uint64_t t = now_ms();
  if (t - last_arm_cmd_ms < 800) return;
  last_arm_cmd_ms = t;

  printf("Requesting ARM...\n");
  send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1, 0,0,0,0,0,0);
}

static void disarm_fc(void) {
  if (!have_fc) return;

  uint64_t t = now_ms();
  if (t - last_disarm_cmd_ms < 50) return;
  last_disarm_cmd_ms = t;

  printf("Requesting DISARM...\n");
  send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0, 0,0,0,0,0,0);
}

static void disarm_fc_force(void) {
  if (!have_fc) return;

  uint64_t t = now_ms();
  if (t - last_disarm_cmd_ms < 800) return;
  last_disarm_cmd_ms = t;

  printf("Requesting FORCE DISARM (21196)...\n");
  send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196,0,0,0,0,0);
}

static void guided_takeoff(float alt_m) {
  if (!have_fc) return;
  printf("Requesting GUIDED TAKEOFF to %.2fm...\n", alt_m);
  send_command_long(MAV_CMD_NAV_TAKEOFF, 0,0,0,0,0,0, alt_m);
}

// Velocity command with selectable MAV_FRAME
static void send_vel_frame(float vx, float vy, float vz, float yaw_rate_deg_s, uint8_t frame) {
  if (!have_fc) return;

  uint16_t mask =
      (1<<0)|(1<<1)|(1<<2) |      // ignore position
      (1<<6)|(1<<7)|(1<<8) |      // ignore accel
      (1<<10);                    // ignore yaw (use yaw_rate)

  mavlink_message_t m;
  mavlink_msg_set_position_target_local_ned_pack(
      g_sysid, g_compid, &m,
      (uint32_t)now_ms(),
      fc_sysid, fc_compid,
      frame,
      mask,
      0,0,0,
      vx,vy,vz,
      0,0,0,
      0, deg2rad(yaw_rate_deg_s)
  );
  mav_send(&m);
}

// Position+Yaw hold command in LOCAL_NED
static void send_pos_yaw_ned(float x, float y, float z_down, float yaw_deg) {
  if (!have_fc) return;

  uint16_t mask =
      (1<<3)|(1<<4)|(1<<5) |     // ignore vx,vy,vz
      (1<<6)|(1<<7)|(1<<8) |     // ignore ax,ay,az
      (1<<11);                   // ignore yaw_rate

  mavlink_message_t m;
  mavlink_msg_set_position_target_local_ned_pack(
      g_sysid, g_compid, &m,
      (uint32_t)now_ms(),
      fc_sysid, fc_compid,
      MAV_FRAME_LOCAL_NED,
      mask,
      x, y, z_down,
      0,0,0,
      0,0,0,
      deg2rad(yaw_deg), 0
  );
  mav_send(&m);
}

// Conservative attitude+thrust setpoint (GUIDED) using ENCODE to avoid pack() signature mismatches.
static void send_attitude_target_thrust(float thrust, float yaw_deg) {
  if (!have_fc) return;

  if (thrust < 0.0f) thrust = 0.0f;
  if (thrust > 0.75f) thrust = 0.75f;

  mavlink_set_attitude_target_t at;
  memset(&at, 0, sizeof(at));
  at.time_boot_ms = (uint32_t)now_ms();
  at.target_system = fc_sysid;
  at.target_component = fc_compid;

  // type_mask bits:
  // bit0..2 ignore body rates; bit3 ignore attitude; bit4 ignore thrust
  // We want: attitude + thrust, ignore body rates.
  at.type_mask = (1u<<0) | (1u<<1) | (1u<<2);

  float yaw = deg2rad(yaw_deg);
  at.q[0] = cosf(yaw * 0.5f);
  at.q[1] = 0.0f;
  at.q[2] = 0.0f;
  at.q[3] = sinf(yaw * 0.5f);

  at.body_roll_rate = 0.0f;
  at.body_pitch_rate = 0.0f;
  at.body_yaw_rate = 0.0f;

  at.thrust = thrust;

#if defined(MAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_THRUST_BODY_LEN)
  // If dialect includes extension thrust_body[], keep zero.
  for (int i = 0; i < MAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_THRUST_BODY_LEN; i++) {
    at.thrust_body[i] = 0.0f;
  }
#endif

  mavlink_message_t m;
  mavlink_msg_set_attitude_target_encode(g_sysid, g_compid, &m, &at);
  mav_send(&m);
}

static float yaw_hold_rate(void) {
  if (!have_yaw_target || !have_att) return 0.0f;
  float err = wrap_deg(yaw_target_deg - current_heading_deg());
  float yr = err * 1.2f;
  if (yr >  YAW_RATE_DPS) yr =  YAW_RATE_DPS;
  if (yr < -YAW_RATE_DPS) yr = -YAW_RATE_DPS;
  return yr;
}

// RC override helpers
static void rc_override_send(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4) {
  if (!have_fc) return;

  mavlink_message_t m;
  mavlink_rc_channels_override_t o;
  memset(&o, 0xFF, sizeof(o));

  o.target_system = fc_sysid;
  o.target_component = fc_compid;

  o.chan1_raw = ch1;
  o.chan2_raw = ch2;
  o.chan3_raw = ch3;
  o.chan4_raw = ch4;

  mavlink_msg_rc_channels_override_encode(g_sysid, g_compid, &m, &o);
  mav_send(&m);
}

static void rc_override_release(void) {
  rc_override_send(UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX);
}

static float servo_motor_avg(void) {
  float s = 0.0f;
  for (int i = 0; i < 4; i++) s += (float)servo_raw[i];
  return s * 0.25f;
}

static bool sys_fresh(uint64_t t) {
  return have_sys && (t - sys_last_ms) < 1000;
}

static bool sys_health_bit(uint32_t bit, uint64_t t) {
  if (!sys_fresh(t)) return true;
  return (sys_health & bit) != 0;
}

static bool hard_nogo(uint64_t t) {
  if (!sys_fresh(t)) return false;
  if (!sys_health_bit(MAV_SYS_STATUS_SENSOR_3D_GYRO, t)) return true;
  if (!sys_health_bit(MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS, t)) return true;
  return false;
}

static bool z_ctrl_ok(uint64_t t) {
  if (!sys_fresh(t)) return true;
  return sys_health_bit(MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL, t);
}

static bool xy_ctrl_ok(uint64_t t) {
  if (!sys_fresh(t)) return true;
  return sys_health_bit(MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL, t);
}

static bool of_fresh(uint64_t t) {
  return have_of && (t - of_last_update_ms) < 400;
}

static bool batt_fresh(uint64_t t) {
  return (batt_last_ms != 0) && (t - batt_last_ms) < BATT_FRESH_MS && !isnan(batt_vpc) && batt_cells > 0;
}

// Pose quality gating for mapping updates:
static bool pose_good_for_mapping(uint64_t t) {
  bool lpos_fresh = have_lpos && (t - lpos_last_update_ms) < 400;
  if (!lpos_fresh) return false;
  if (!have_att) return false;
  if (!xy_ctrl_ok(t)) return false;
  if (!z_ctrl_ok(t)) return false;

  if (of_fresh(t)) {
    if (of_quality < 50) return false;
  }

  return true;
}

// ----------------------------- Drift/flip fix gating -------------------
#if defined(MAV_FRAME_BODY_OFFSET_NED)
  #define MOVE_FRAME MAV_FRAME_BODY_OFFSET_NED
#else
  #define MOVE_FRAME MAV_FRAME_BODY_NED
#endif

static const uint64_t XY_STABLE_HOLD_MS = 1000;     // must be "good" for this long before we allow pos hold / exploration translation
static uint64_t xy_ok_since_ms = 0;

static bool vel_xy_allowed(uint64_t t) {
  // We only allow world-frame position/translation control when estimator says XY control is OK.
  if (!xy_ctrl_ok(t)) return false;
  if (!have_att) return false;

  bool lpos_fresh = have_lpos && (t - lpos_last_update_ms) < 400;
  if (!lpos_fresh) return false;

  // If optical flow is present, require decent quality (prevents sideways lunge from garbage flow)
  if (of_fresh(t) && of_quality < 50) return false;

  // Avoid allowing XY right at ground / takeoff dust-up
  if (!isnan(alt_est_m) && alt_est_m < 0.12f) return false;

  return true;
}

static bool vel_xy_stable(uint64_t t) {
  bool ok = vel_xy_allowed(t);
  if (ok) {
    if (xy_ok_since_ms == 0) xy_ok_since_ms = t;
    if ((t - xy_ok_since_ms) >= XY_STABLE_HOLD_MS) return true;
    return false;
  } else {
    xy_ok_since_ms = 0;
    return false;
  }
}

// Hover in place (but only lock position once XY is stable)
static void hover_hold_tick(uint64_t t, bool allow_pos_hold) {
  bool lpos_fresh = have_lpos && (t - lpos_last_update_ms) < 400;
  bool yaw_ok = have_att;

  if (!allow_pos_hold) {
    // Do NOT lock position. Just command zero body-frame velocity + yaw hold.
    send_vel_frame(0,0,0, yaw_hold_rate(), MOVE_FRAME);
    return;
  }

  if (!hover_hold_valid && lpos_fresh && yaw_ok && !isnan(alt_est_m)) {
    hover_hold_x_m = lpos_x_m;
    hover_hold_y_m = lpos_y_m;
    hover_hold_z_ned_m = -alt_est_m;
    hover_hold_yaw_deg = have_yaw_target ? yaw_target_deg : current_heading_deg();
    hover_hold_set_ms = t;
    hover_hold_valid = true;
  }

  if (hover_hold_valid && lpos_fresh && yaw_ok) {
    send_pos_yaw_ned(hover_hold_x_m, hover_hold_y_m, hover_hold_z_ned_m, hover_hold_yaw_deg);
  } else {
    send_vel_frame(0,0,0, yaw_hold_rate(), MOVE_FRAME);
  }
}

// ----------------------------- Requests --------------------------------
static void request_streams(void) {
  const uint8_t tgt_sys  = fc_sysid;
  const uint8_t tgt_comp = 0;

  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_SYS_STATUS,          200000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,    50000.0f, 0,0,0,0,0);

  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_BATTERY_STATUS,      200000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_DISTANCE_SENSOR,     100000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_EXTENDED_SYS_STATE,  200000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE,             50000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_LOCAL_POSITION_NED,   50000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_OPTICAL_FLOW,         50000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_OPTICAL_FLOW_RAD,     50000.0f, 0,0,0,0,0);

  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, (float)MAVLINK_MSG_ID_RANGEFINDER,  100000.0f, 0,0,0,0,0);

  send_request_data_stream(tgt_sys, tgt_comp, MAV_DATA_STREAM_EXTRA3, 20, 1);
}

// ----------------------------- MAVLink decode handlers -----------------
static void handle_heartbeat(const mavlink_message_t *msg) {
  mavlink_heartbeat_t hb;
  mavlink_msg_heartbeat_decode(msg, &hb);

  last_hb_ms = now_ms();
  hb_custom_mode = hb.custom_mode;
  fc_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
}

static void handle_command_ack(const mavlink_message_t *msg) {
  mavlink_command_ack_t a;
  mavlink_msg_command_ack_decode(msg, &a);
  have_ack = true;
  last_ack_cmd = a.command;
  last_ack_res = a.result;

  if (a.command == MAV_CMD_NAV_TAKEOFF) {
    have_takeoff_ack = true;
    takeoff_ack_res = a.result;
    takeoff_ack_ms = now_ms();

    if (a.result == MAV_RESULT_ACCEPTED) {
      takeoff_accept_ms = takeoff_ack_ms;
    }
  }
}

static void handle_extended_sys_state(const mavlink_message_t *msg) {
  mavlink_extended_sys_state_t s;
  mavlink_msg_extended_sys_state_decode(msg, &s);
  landed_state = s.landed_state;
  have_ext = true;
}

static void handle_sys_status(const mavlink_message_t *msg) {
  mavlink_sys_status_t s;
  mavlink_msg_sys_status_decode(msg, &s);
  sys_present = s.onboard_control_sensors_present;
  sys_enabled = s.onboard_control_sensors_enabled;
  sys_health  = s.onboard_control_sensors_health;
  sys_last_ms = now_ms();
  have_sys = true;
}

static void handle_servo_output_raw(const mavlink_message_t *msg) {
  mavlink_servo_output_raw_t so;
  mavlink_msg_servo_output_raw_decode(msg, &so);
  servo_raw[0] = so.servo1_raw;
  servo_raw[1] = so.servo2_raw;
  servo_raw[2] = so.servo3_raw;
  servo_raw[3] = so.servo4_raw;
  servo_raw[4] = so.servo5_raw;
  servo_raw[5] = so.servo6_raw;
  servo_raw[6] = so.servo7_raw;
  servo_raw[7] = so.servo8_raw;
  servo_last_ms = now_ms();
  have_servo = true;
}

static void handle_battery_status(const mavlink_message_t *msg) {
  mavlink_battery_status_t b;
  mavlink_msg_battery_status_decode(msg, &b);

  float sum_v = 0.0f;
  int cells = 0;
  for (int i = 0; i < 10; i++) {
    if (b.voltages[i] > 0 && b.voltages[i] < 20000) {
      sum_v += (float)b.voltages[i] * 0.001f;
      cells++;
    }
  }
  if (cells > 0) {
    batt_v_total = sum_v;
    batt_cells = cells;
    batt_vpc = sum_v / (float)cells;
    batt_last_ms = now_ms();
  }
}

static void handle_attitude(const mavlink_message_t *msg) {
  mavlink_attitude_t a;
  mavlink_msg_attitude_decode(msg, &a);
  roll_rad = a.roll;
  pitch_rad = a.pitch;
  yaw_rad = a.yaw;
  have_att = true;
}

static void handle_optical_flow(const mavlink_message_t *msg) {
  mavlink_optical_flow_t o;
  mavlink_msg_optical_flow_decode(msg, &o);

  have_of      = true;
  of_quality   = o.quality;
  of_comp_m_x  = o.flow_comp_m_x;
  of_comp_m_y  = o.flow_comp_m_y;
  of_last_update_ms = now_ms();

  of_ground_m = o.ground_distance;
  of_rate_x   = o.flow_rate_x;
  of_rate_y   = o.flow_rate_y;
}

static void handle_optical_flow_rad(const mavlink_message_t *msg) {
  mavlink_optical_flow_rad_t o;
  mavlink_msg_optical_flow_rad_decode(msg, &o);

  have_of    = true;
  of_quality = o.quality;
  of_last_update_ms = now_ms();

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
}

static void handle_local_position_ned(const mavlink_message_t *msg) {
  mavlink_local_position_ned_t p;
  mavlink_msg_local_position_ned_decode(msg, &p);

  float alt = -p.z;
  if (!(alt > -5.0f && alt < 50.0f)) return;

  have_lpos = true;

  lpos_x_m    = p.x;
  lpos_y_m    = p.y;
  lpos_vx_mps = p.vx;
  lpos_vy_mps = p.vy;
  lpos_vz_mps = p.vz;

  lpos_alt_m = alt;

  uint64_t t = now_ms();
  if (isnan(lpos_alt_filt_m)) {
    lpos_alt_filt_m = alt;
    lpos_last_update_ms = t;
    return;
  }

  const float alpha = 0.18f;
  lpos_alt_filt_m = (1.0f - alpha) * lpos_alt_filt_m + alpha * alt;
  lpos_last_update_ms = t;
}

static void handle_distance_sensor(const mavlink_message_t *msg) {
  mavlink_distance_sensor_t d;
  mavlink_msg_distance_sensor_decode(msg, &d);

  have_ds = true;
  ds_id = d.id;
  ds_orientation = d.orientation;
  ds_cur_cm = d.current_distance;
  ds_last_update_ms = now_ms();

  if (d.current_distance > 0 && d.current_distance < 60000) {
    if (d.orientation == ORIENT_DOWNWARD_FACING) {
      rangefinder_m = (float)d.current_distance * 0.01f;
      rangefinder_last_update_ms = ds_last_update_ms;
      have_rangefinder = true;
    }
  }
}

static uint32_t rd_u32_le2(const uint8_t* p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static float rd_f32_le(const uint8_t* p) {
  uint32_t u = rd_u32_le2(p);
  float f;
  memcpy(&f, &u, sizeof(f));
  return f;
}

static void handle_rangefinder_msg(const mavlink_message_t *msg) {
  if (msg->len < 8) return;
  const uint8_t* p = (const uint8_t*)_MAV_PAYLOAD(msg);
  float dist = rd_f32_le(p + 0);
  float volt = rd_f32_le(p + 4);

  if (!isnan(dist) && dist > 0.0f && dist < 60.0f) {
    rangefinder_m = dist;
    rangefinder_v = volt;
    rangefinder_last_update_ms = now_ms();
    have_rangefinder = true;
  }
}

static void handle_statustext(const mavlink_message_t *msg) {
  mavlink_statustext_t s;
  mavlink_msg_statustext_decode(msg, &s);

  char buf[52];
  memset(buf, 0, sizeof(buf));
  memcpy(buf, s.text, 50);

  // cache
  memset(last_statustext, 0, sizeof(last_statustext));
  strncpy(last_statustext, buf, sizeof(last_statustext)-1);
  last_statustext_sev = s.severity;
  last_statustext_ms = now_ms();

  printf("FC STATUSTEXT sev=%u: %s\n", (unsigned)s.severity, buf);
  fflush(stdout);
}

// ----------------------------- FC UART pump ----------------------------
static void pump_fc_uart(void) {
  uint8_t buf[256];
  int n = (int)read(fc_fd, buf, sizeof(buf));
  if (n <= 0) return;

  static mavlink_status_t status;
  static mavlink_message_t msg;

  for (int i = 0; i < n; i++) {
    if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {

      if (!have_fc && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        fc_sysid = msg.sysid;
        fc_compid = msg.compid;
        have_fc = true;
        printf("FC connected: sys=%u comp=%u\n", (unsigned)fc_sysid, (unsigned)fc_compid);
        request_streams();
      }

      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:            handle_heartbeat(&msg); break;
        case MAVLINK_MSG_ID_COMMAND_ACK:         handle_command_ack(&msg); break;
        case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:  handle_extended_sys_state(&msg); break;
        case MAVLINK_MSG_ID_SYS_STATUS:          handle_sys_status(&msg); break;
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:    handle_servo_output_raw(&msg); break;
        case MAVLINK_MSG_ID_BATTERY_STATUS:      handle_battery_status(&msg); break;
        case MAVLINK_MSG_ID_ATTITUDE:            handle_attitude(&msg); break;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:  handle_local_position_ned(&msg); break;
        case MAVLINK_MSG_ID_OPTICAL_FLOW:        handle_optical_flow(&msg); break;
        case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:    handle_optical_flow_rad(&msg); break;
        case MAVLINK_MSG_ID_DISTANCE_SENSOR:     handle_distance_sensor(&msg); break;
        case MAVLINK_MSG_ID_RANGEFINDER:         handle_rangefinder_msg(&msg); break;
        case MAVLINK_MSG_ID_STATUSTEXT:          handle_statustext(&msg); break;
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

static uint32_t rd_u32_le(const uint8_t* p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static uint16_t rd_u16_le(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static float robust_col_dist_m(const uint8_t* sensor_grid_u16le, int col) {
  float best = NAN;
  float second = NAN;

  for (int row = 0; row < TOF_ROWS; row++) {
    int i = row * TOF_COLS + col;
    uint16_t mm = rd_u16_le(sensor_grid_u16le + i * 2);
    if (mm == 0xFFFF || mm == 0) continue;
    float m = (float)mm * 0.001f;
    if (m <= 0.02f) continue;
    if (m > TOF_MAX_RANGE_M) m = TOF_MAX_RANGE_M;

    if (isnan(best) || m < best) {
      second = best;
      best = m;
    } else if (isnan(second) || m < second) {
      second = m;
    }
  }

  if (!isnan(second)) return second;
  return best;
}

static void compute_beams_and_minima(const uint8_t* frame) {
  const uint8_t* p = &frame[5];

  for (int d = 0; d < 4; d++) {
    const uint8_t* grid = p + (d * GRID_SIZE * 2);
    float dir_min = NAN;

    for (int c = 0; c < TOF_COLS; c++) {
      float m = robust_col_dist_m(grid, c);
      tof_beams_m[d][c] = m;
      if (!isnan(m) && (isnan(dir_min) || m < dir_min)) dir_min = m;
    }

    tof_min_m[d] = dir_min;
  }
}

static void accept_scan_frame(const uint8_t* frame) {
  last_scan_t_ms = rd_u32_le2(frame + 1);
  memcpy(last_scan_grid_raw, frame + 5, sizeof(last_scan_grid_raw));
  last_scan_host_ms = now_ms();
  have_scan_frame = true;
  scan_new = true;

  compute_beams_and_minima(frame);
}

static void accept_ctrl_frame(const uint8_t* frame) {
  uint8_t cmd = frame[1];
  uint32_t seq = rd_u32_le2(frame + 2);

  if (cmd == 0) {
    want_arm = false;
    printf("CTRL: DISARM (seq=%u)\n", (unsigned)seq);
  } else if (cmd == 1) {
    want_arm = true;
    printf("CTRL: ARM (seq=%u)\n", (unsigned)seq);
  } else {
    printf("CTRL: unknown cmd=%u (seq=%u)\n", (unsigned)cmd, (unsigned)seq);
  }
}

static void pump_tof_uart(void) {
  uint8_t buf[512];
  int n = (int)read(tof_fd, buf, sizeof(buf));
  if (n <= 0) return;

  for (int i = 0; i < n; i++) {
    uint8_t b = buf[i];

    // --- CTRL parser ---
    if (ctrl_rxpos == 0) {
      if (b == CTRL_HEADER) {
        ctrl_rxbuf[ctrl_rxpos++] = b;
        continue;
      }
    } else {
      ctrl_rxbuf[ctrl_rxpos++] = b;
      if (ctrl_rxpos == CTRL_BYTES) {
        uint8_t c = xor8(ctrl_rxbuf, CTRL_BYTES-1);
        if (c == ctrl_rxbuf[CTRL_BYTES-1]) {
          accept_ctrl_frame(ctrl_rxbuf);
        }
        ctrl_rxpos = 0;
      }
      continue;
    }

    // --- SCAN parser ---
    if (tof_rxpos == 0) {
      if (b != SCAN_HEADER) continue;
    }

    tof_rxbuf[tof_rxpos++] = b;

    if (tof_rxpos == SCAN_BYTES) {
      uint8_t c = xor8(tof_rxbuf, SCAN_BYTES-1);
      if (c == tof_rxbuf[SCAN_BYTES-1]) {
        accept_scan_frame(tof_rxbuf);
      }
      tof_rxpos = 0;
    }
  }
}

// ----------------------------- Filtering --------------------------------
static void tof_filter_tick(void) {
  const float alpha = 0.20f;
  for (int d = 0; d < 4; d++) {
    float v = tof_min_m[d];
    if (isnan(v)) continue;
    if (isnan(tof_filt_m[d])) tof_filt_m[d] = v;
    else tof_filt_m[d] = (1.0f - alpha) * tof_filt_m[d] + alpha * v;
  }
}

static void update_alt_estimate(void) {
  uint64_t tnow = now_ms();
  bool lpos_fresh  = have_lpos && (tnow - lpos_last_update_ms) < 400;
  bool range_fresh = have_rangefinder && (tnow - rangefinder_last_update_ms) < 400;
  bool near_ground = false;
  if (have_ext && landed_state == MAV_LANDED_STATE_ON_GROUND) near_ground = true;

  have_xy = lpos_fresh;

  if (range_fresh && !isnan(rangefinder_m)) {
    float a = rangefinder_m;
    if (a < 0.0f) a = 0.0f;
    if (a > 10.0f) a = 10.0f;
    alt_est_m = a;
    alt_src = ALT_SRC_RANGEFINDER;
  } else if (lpos_fresh) {
    float a = lpos_alt_filt_m;
    if (a < 0.0f) a = 0.0f;
    if (a > 10.0f) a = 10.0f;
    alt_est_m = a;
    alt_src = ALT_SRC_LPOS;
  } else if (near_ground) {
    alt_est_m = 0.0f;
    alt_src = ALT_SRC_ON_GROUND;
  } else {
    alt_src = ALT_SRC_NONE;
  }

  if (!isnan(alt_est_m) && alt_est_m >= CEIL_M) ceiling_active = true;
  if (!isnan(alt_est_m) && alt_est_m <= (CEIL_M - 0.10f)) ceiling_active = false;
}

// ----------------------------- Logging (pose + scans) ------------------
static const char* alt_src_name(AltSrc s) {
  switch (s) {
    case ALT_SRC_LPOS: return "LPOS";
    case ALT_SRC_RANGEFINDER: return "RF";
    case ALT_SRC_ON_GROUND: return "GND";
    default: return "?";
  }
}

static void log_init(void) {
  log_fp = fopen(LOG_CSV_PATH, "a");
  if (!log_fp) {
    fprintf(stderr, "WARN: cannot open %s: %s\n", LOG_CSV_PATH, strerror(errno));
  } else {
    fseek(log_fp, 0, SEEK_END);
    long sz = ftell(log_fp);
    if (sz <= 0) {
      fprintf(log_fp,
              "t_ms,state,want_arm,armed,mode,yaw_deg,alt_m,alt_src,x_m,y_m,vx_mps,vy_mps,"
              "rf_m,of_q,of_rate_x,of_rate_y,"
              "tof_f,tof_r,tof_b,tof_l,batt_v,batt_cells\n");
      fflush(log_fp);
    }
  }

  scan_fp = fopen(LOG_SCAN_PATH, "ab");
  if (!scan_fp) {
    fprintf(stderr, "WARN: cannot open %s: %s\n", LOG_SCAN_PATH, strerror(errno));
  } else {
    fseek(scan_fp, 0, SEEK_END);
    long sz = ftell(scan_fp);
    if (sz <= 0) {
      const char hdr[] = "SCLOG2\n";
      fwrite(hdr, 1, sizeof(hdr)-1, scan_fp);
      fflush(scan_fp);
    }
  }

  last_flush_ms = now_ms();
  last_log_ms   = 0;
}

static void log_flush_if_due(uint64_t t) {
  if (t - last_flush_ms < LOG_FLUSH_MS) return;
  last_flush_ms = t;
  if (log_fp) fflush(log_fp);
  if (scan_fp) fflush(scan_fp);
}

typedef struct __attribute__((packed)) {
  uint32_t magic;       // 'SCN2'
  uint32_t host_ms;
  uint32_t scan_ms;

  float    x_m;
  float    y_m;
  float    yaw_deg;
  float    alt_m;

  float    roll_rad;
  float    pitch_rad;

  float    rf_m;
  float    of_rate_x;
  float    of_rate_y;
  uint8_t  of_q;

  uint8_t  state;
  uint8_t  kf_flags;
  uint16_t _pad0;

  uint32_t sys_health;

  uint8_t  grid_raw[512];
} scanrec_t;

static void log_scan_record(void) {
  if (!scan_fp || !have_scan_frame) return;

  scanrec_t r;
  memset(&r, 0, sizeof(r));

  r.magic   = 0x324E4353u; // 'SCN2'
  r.host_ms = (uint32_t)last_scan_host_ms;
  r.scan_ms = last_scan_t_ms;

  r.x_m   = have_xy ? lpos_x_m : NAN;
  r.y_m   = have_xy ? lpos_y_m : NAN;
  r.yaw_deg = have_att ? current_heading_deg() : NAN;
  r.alt_m = alt_est_m;

  r.roll_rad = have_att ? roll_rad : NAN;
  r.pitch_rad = have_att ? pitch_rad : NAN;

  r.rf_m  = rangefinder_m;
  r.of_rate_x = of_rate_x;
  r.of_rate_y = of_rate_y;
  r.of_q = of_quality;
  r.state = (uint8_t)st;

  r.kf_flags = pending_kf_flags;
  pending_kf_flags = 0;

  r.sys_health = have_sys ? sys_health : 0;

  memcpy(r.grid_raw, last_scan_grid_raw, sizeof(r.grid_raw));

  fwrite(&r, 1, sizeof(r), scan_fp);
}

static void log_tick(uint64_t t) {
  const uint64_t period_ms = 1000 / LOG_HZ;
  if (log_fp && (t - last_log_ms) >= period_ms) {
    last_log_ms = t;

    fprintf(log_fp,
            "%llu,%s,%d,%d,%u,",
            (unsigned long long)t,
            state_name(st),
            want_arm ? 1 : 0,
            fc_armed ? 1 : 0,
            (unsigned)hb_custom_mode);

    if (have_att) fprintf(log_fp, "%.3f,", current_heading_deg());
    else fprintf(log_fp, "nan,");

    if (!isnan(alt_est_m)) fprintf(log_fp, "%.3f,", alt_est_m);
    else fprintf(log_fp, "nan,");
    fprintf(log_fp, "%s,", alt_src_name(alt_src));

    if (have_xy) fprintf(log_fp, "%.3f,%.3f,%.3f,%.3f,",
                         lpos_x_m, lpos_y_m, lpos_vx_mps, lpos_vy_mps);
    else fprintf(log_fp, "nan,nan,nan,nan,");

    bool rf_fresh = have_rangefinder && (t - rangefinder_last_update_ms) < 400;
    if (rf_fresh && !isnan(rangefinder_m)) fprintf(log_fp, "%.3f,", rangefinder_m);
    else fprintf(log_fp, "nan,");

    bool of_ok = of_fresh(t);
    if (of_ok) fprintf(log_fp, "%u,", (unsigned)of_quality);
    else fprintf(log_fp, "0,");

    if (of_ok && !isnan(of_rate_x) && !isnan(of_rate_y)) fprintf(log_fp, "%.4f,%.4f,", of_rate_x, of_rate_y);
    else fprintf(log_fp, "nan,nan,");

    fprintf(log_fp, "%.3f,%.3f,%.3f,%.3f,",
            tof_filt_m[D_FRONT], tof_filt_m[D_RIGHT], tof_filt_m[D_BACK], tof_filt_m[D_LEFT]);

    if (!isnan(batt_v_total) && batt_cells > 0) fprintf(log_fp, "%.3f,%d\n", batt_v_total, batt_cells);
    else fprintf(log_fp, "nan,0\n");
  }

  if (scan_new) {
    scan_new = false;
    log_scan_record();

    if (map_inited && have_xy) {
      map_recentre_if_needed(lpos_x_m, lpos_y_m);
    }

    if (map_inited && pose_good_for_mapping(t)) {
      map_update_from_beams(lpos_x_m, lpos_y_m, current_heading_deg());
    }
  }

  log_flush_if_due(t);
}

// ----------------------------- Behavior --------------------------------
static void enter_state(State ns) {
  if (st == ns) return;

  if (st == ST_LIFTOFF_ASSIST && ns != ST_LIFTOFF_ASSIST) {
    rc_override_release();
  }

  if (ns == ST_TAKEOFF) {
    takeoff_sent = false;
    takeoff_sent_ms = 0;
    have_takeoff_ack = false;

    takeoff_accept_ms = 0;
    takeoff_no_vel_until_ms = 0;
    takeoff_started = false;
    takeoff_started_ms = 0;
    takeoff_not_started_printed = false;
    takeoff_thr_ramp_active = false;
    takeoff_thr_ramp_start_ms = 0;
    takeoff_thr_ramp_last_send_ms = 0;

    pending_kf_flags |= KF_TAKEOFF;
  }

  if (ns == ST_LIFTOFF_ASSIST) {
    assist_start_ms = now_ms();
    assist_last_send_ms = 0;
    assist_alt0 = alt_est_m;
    assist_baseline_set = false;
    assist_motor_avg0 = NAN;
    assist_warned_override = false;
    pending_kf_flags |= KF_LIFTOFF_AST;
  }

  if (ns == ST_HOVER) {
    hover_enter_ms = now_ms();
    hover_hold_valid = false; // force re-capture only after XY is stable
  }

  if (ns == ST_LANDING) {
    land_mode_sent = false;
    land_mode_sent_ms = 0;
    pending_kf_flags |= KF_LAND_START;
  }

  if (ns == ST_TURNING) {
    pending_kf_flags |= KF_TURN_START;
  }

  if (st == ST_TURNING && ns != ST_TURNING) {
    turning_init = false;
    pending_kf_flags |= KF_TURN_END;
    explore_pause_until_ms = now_ms() + POST_TURN_PAUSE_MS;
  }

  st = ns;
}

static int open_side_dir(void) {
  float r = tof_filt_m[D_RIGHT];
  float l = tof_filt_m[D_LEFT];
  float b = tof_filt_m[D_BACK];

  float best = -1.0f;
  int best_dir = D_RIGHT;

  if (!isnan(r) && r > best) { best = r; best_dir = D_RIGHT; }
  if (!isnan(l) && l > best) { best = l; best_dir = D_LEFT; }
  if (!isnan(b) && b > best) { best = b; best_dir = D_BACK; }

  return best_dir;
}

static int choose_turn_dir_frontier(uint64_t t) {
  if (!map_inited || !have_xy || !have_att) return open_side_dir();

  int sR = frontier_score_dir(lpos_x_m, lpos_y_m, current_heading_deg(), +90.0f);
  int sL = frontier_score_dir(lpos_x_m, lpos_y_m, current_heading_deg(), -90.0f);
  int sB = frontier_score_dir(lpos_x_m, lpos_y_m, current_heading_deg(), 180.0f);

  float r = isnan(tof_filt_m[D_RIGHT]) ? 0.0f : tof_filt_m[D_RIGHT];
  float l = isnan(tof_filt_m[D_LEFT])  ? 0.0f : tof_filt_m[D_LEFT];
  float b = isnan(tof_filt_m[D_BACK])  ? 0.0f : tof_filt_m[D_BACK];

  sR += (int)(r * 5.0f);
  sL += (int)(l * 5.0f);
  sB += (int)(b * 5.0f);

  int best_dir = D_RIGHT;
  int best_score = sR;
  if (sL > best_score) { best_score = sL; best_dir = D_LEFT; }
  if (sB > best_score) { best_score = sB; best_dir = D_BACK; }

  return best_dir;
}

static void liftoff_assist_tick(uint64_t t) {
  if (t - assist_start_ms < 150) {
    set_mode_stabilize();
  }

  if (!assist_baseline_set && have_servo && (t - servo_last_ms) < 200) {
    assist_motor_avg0 = servo_motor_avg();
    assist_baseline_set = true;
  }

  if (t - assist_last_send_ms >= ASSIST_SEND_PERIOD_MS) {
    assist_last_send_ms = t;

    uint64_t dt = t - assist_start_ms;
    float u = (dt >= ASSIST_TOTAL_MS) ? 1.0f : (float)dt / (float)ASSIST_TOTAL_MS;
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;

    uint16_t thr = (uint16_t)lrintf((1.0f - u) * (float)ASSIST_THR_US_MIN + u * (float)ASSIST_THR_US_MAX);
    rc_override_send(RC_NEUTRAL_US, RC_NEUTRAL_US, thr, RC_NEUTRAL_US);
  }

  if (!assist_warned_override &&
      assist_baseline_set &&
      (t - assist_start_ms) > ASSIST_OVERRIDE_EFFECT_MS &&
      have_servo && (t - servo_last_ms) < 200) {

    float avg = servo_motor_avg();
    if (!isnan(assist_motor_avg0) && (avg - assist_motor_avg0) < ASSIST_MOTOR_DELTA_MIN) {
      assist_warned_override = true;
      printf("WARN: RC override appears IGNORED (motor avg delta %.1fus). "
             "Check ArduPilot MAVLink RC input acceptance (MAV_GCS_SYSID/MAV_OPTIONS) and RC override timeout params.\n",
             (avg - assist_motor_avg0));
    }
  }

  if (!isnan(alt_est_m) && alt_est_m > ASSIST_EXIT_ALT_M) {
    printf("LIFTOFF_ASSIST: detected alt=%.2fm, switching back to GUIDED+TAKEOFF.\n", alt_est_m);
    rc_override_release();
    set_mode_guided();
    guided_takeoff(TAKEOFF_TARGET_M);
    enter_state(ST_TAKEOFF);
    return;
  }

  if ((t - assist_start_ms) > ASSIST_ABORT_MS) {
    printf("LIFTOFF_ASSIST: timeout (alt=%.2fm). Disarming for safety.\n", alt_est_m);
    rc_override_release();
    enter_state(ST_DISARMING);
    return;
  }
}

static bool arm_allowed_by_battery(uint64_t t) {
  if (!batt_fresh(t)) return true;
  if (batt_vpc >= BATT_ARM_MIN_VPC) return true;
  return false;
}

static void battery_failsafe_tick(uint64_t t) {
  if (!batt_fresh(t)) return;

  if (!fc_armed) {
    if (want_arm && batt_vpc < BATT_ARM_MIN_VPC) {
      if (t - batt_last_warn_ms > 1200) {
        batt_last_warn_ms = t;
        printf("BATT NO-GO: Vpc=%.2f < %.2f. Refusing arm to protect 2S LiHV.\n", batt_vpc, BATT_ARM_MIN_VPC);
      }
    }
    batt_low_since_ms = 0;
    batt_emerg_since_ms = 0;
    return;
  }

  if (batt_vpc < BATT_EMERG_VPC) {
    if (batt_emerg_since_ms == 0) batt_emerg_since_ms = t;
    if ((t - batt_emerg_since_ms) > BATT_LOW_HOLD_MS) {
      pending_kf_flags |= KF_BATT_EMERG;
      if (st != ST_LANDING && st != ST_DISARMING) {
        printf("BATT EMERG: Vpc=%.2f < %.2f sustained -> LANDING.\n", batt_vpc, BATT_EMERG_VPC);
        enter_state(ST_LANDING);
      }
    }
  } else {
    batt_emerg_since_ms = 0;
  }

  if (batt_vpc < BATT_LAND_VPC) {
    if (batt_low_since_ms == 0) batt_low_since_ms = t;
    if ((t - batt_low_since_ms) > BATT_LOW_HOLD_MS) {
      pending_kf_flags |= KF_BATT_LAND;
      if (st != ST_LANDING && st != ST_DISARMING) {
        printf("BATT LOW: Vpc=%.2f < %.2f sustained -> LANDING.\n", batt_vpc, BATT_LAND_VPC);
        enter_state(ST_LANDING);
      }
    }
  } else {
    batt_low_since_ms = 0;
  }
}

static bool takeoff_off_ground(uint64_t t) {
  bool rf_fresh = have_rangefinder && (t - rangefinder_last_update_ms) < 400;
  if (have_ext && landed_state != MAV_LANDED_STATE_ON_GROUND) return true;
  if (rf_fresh && !isnan(rangefinder_m) && rangefinder_m > TO_RAMP_EXIT_M) return true;
  if (!isnan(alt_est_m) && alt_est_m > TO_RAMP_EXIT_M) return true;
  return false;
}

static void takeoff_thrust_ramp_tick(uint64_t t) {
  // Send SET_ATTITUDE_TARGET at 20Hz with a gentle thrust ramp.
  if (takeoff_thr_ramp_start_ms == 0) takeoff_thr_ramp_start_ms = t;

  if ((t - takeoff_thr_ramp_last_send_ms) >= TO_RAMP_SEND_MS) {
    takeoff_thr_ramp_last_send_ms = t;

    uint64_t dt = t - takeoff_thr_ramp_start_ms;
    float u = (dt >= TO_RAMP_TOTAL_MS) ? 1.0f : (float)dt / (float)TO_RAMP_TOTAL_MS;
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;

    float thr = (1.0f - u) * TO_RAMP_THR_MIN + u * TO_RAMP_THR_MAX;

    float yaw = have_yaw_target ? yaw_target_deg : (have_att ? current_heading_deg() : 0.0f);
    send_attitude_target_thrust(thr, yaw);
  }
}

static void control_tick(void) {
  uint64_t t = now_ms();

  send_own_heartbeat_tick(t);

  update_alt_estimate();
  tof_filter_tick();
  log_tick(t);

  battery_failsafe_tick(t);

#if PRINT_LANDED_STATE_EACH_TICK
  if (have_ext) {
    printf("EXT_SYS_STATE.landed_state=%u(%s)\n", (unsigned)landed_state, landed_state_name(landed_state));
  } else {
    printf("EXT_SYS_STATE.landed_state=?\n");
  }
#endif

  static uint64_t t_last_print = 0;
  if (t - t_last_print >= (1000 / PRINT_HZ)) {
    t_last_print = t;

    bool xy_stable = vel_xy_stable(t);

    printf("st=%s want=%d HB=%d mode=%u armed=%d alt=",
           state_name(st), want_arm?1:0, (last_hb_ms?1:0),
           (unsigned)hb_custom_mode, fc_armed?1:0);

    if (isnan(alt_est_m)) printf("?");
    else printf("%.2f", alt_est_m);

    printf("(%s)", alt_src_name(alt_src));
    printf(" CEIL=%d", ceiling_active?1:0);

    printf(" landed=");
    if (!have_ext) printf("?");
    else printf("%u(%s)", (unsigned)landed_state, landed_state_name(landed_state));

    printf(" sys=");
    if (!sys_fresh(t)) printf("?");
    else {
      printf("Z=%d XY=%d GYR=%d MOT=%d",
             sys_health_bit(MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL, t)?1:0,
             sys_health_bit(MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL, t)?1:0,
             sys_health_bit(MAV_SYS_STATUS_SENSOR_3D_GYRO, t)?1:0,
             sys_health_bit(MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS, t)?1:0);
    }

    printf(" xyOK=%d", xy_stable ? 1 : 0);

    printf(" lpos=");
    bool lpos_fresh2 = have_lpos && (t - lpos_last_update_ms) < 400;
    if (!lpos_fresh2 || isnan(lpos_alt_filt_m)) printf("?");
    else printf("%.2f", lpos_alt_filt_m);

    printf(" rf=");
    bool rf_fresh = have_rangefinder && (t - rangefinder_last_update_ms) < 400;
    if (!rf_fresh || isnan(rangefinder_m)) printf("?");
    else printf("%.2f", rangefinder_m);
    if (rf_fresh && !isnan(rangefinder_v)) printf("V=%.2f", rangefinder_v);

    if (have_ds) {
      bool ds_fresh = (t - ds_last_update_ms) < 1000;
      printf(" ds(id=%u ori=%u cur=%.2fm%s)",
             (unsigned)ds_id, (unsigned)ds_orientation,
             (float)ds_cur_cm * 0.01f,
             ds_fresh ? "" : " stale");
    }

    printf(" yaw=");
    if (!have_att) printf("?");
    else printf("%.1f", current_heading_deg());
    if (have_yaw_target) printf("->%.1f", yaw_target_deg);

    printf(" tof(F/R/B/L)=%.2f/%.2f/%.2f/%.2f",
           tof_filt_m[D_FRONT], tof_filt_m[D_RIGHT], tof_filt_m[D_BACK], tof_filt_m[D_LEFT]);

    bool of_ok = of_fresh(t);
    printf(" of=");
    if (!of_ok) {
      printf("?");
    } else {
      printf("q=%u", (unsigned)of_quality);
      if (!isnan(of_comp_m_x) && !isnan(of_comp_m_y))
        printf(" d=%.3f/%.3f", of_comp_m_x, of_comp_m_y);
      if (!isnan(of_rate_x) && !isnan(of_rate_y))
        printf(" r=%.2f/%.2f", of_rate_x, of_rate_y);
      if (!isnan(of_ground_m))
        printf(" z=%.2f", of_ground_m);
    }

    if (!isnan(batt_v_total) && batt_cells > 0)
      printf(" V=%.2f (%dc) Vpc=%.2f", batt_v_total, batt_cells, batt_vpc);

    if (have_servo && (t - servo_last_ms) < 300) {
      printf(" mot_avg=%.1f", servo_motor_avg());
    }

    if (have_ack) {
      printf(" ack(cmd=%u res=%u)", (unsigned)last_ack_cmd, (unsigned)last_ack_res);
      have_ack = false;
    }

    if (map_inited) printf(" map=ON(%dx%d@%.2fm)", MAP_W, MAP_H, MAP_RES_M);
    else printf(" map=OFF");

    printf("\n");
    fflush(stdout);
  }

  if (!have_fc) {
    if (st != ST_WAIT_LINK) enter_state(ST_WAIT_LINK);
    return;
  }

  if (hard_nogo(t)) {
    printf("NO-GO: SYS_STATUS indicates gyro or motor outputs unhealthy. Disarming/refusing takeoff.\n");
    if (fc_armed) enter_state(ST_DISARMING);
    else enter_state(ST_IDLE);
    return;
  }

  if (fc_armed_prev && !fc_armed && want_arm &&
      st != ST_LANDING && st != ST_DISARMING && st != ST_IDLE) {

    printf("WARN: FC DISARMED unexpectedly in %s. landed=%u(%s)\n",
           state_name(st),
           (unsigned)landed_state, landed_state_name(landed_state));

    if (last_statustext_ms != 0) {
      printf("  last STATUSTEXT (%llums ago) sev=%u: %s\n",
             (unsigned long long)(t - last_statustext_ms),
             (unsigned)last_statustext_sev,
             last_statustext);
    }
    fflush(stdout);

    enter_state(ST_IDLE);
  }
  fc_armed_prev = fc_armed;


  if (!want_arm && fc_armed) {
  enter_state(ST_DISARMING);
  }

  //ONLY USE THIS PART WHEN HOVER IS STABLE

  // if (!want_arm && fc_armed) {
  //   bool near_ground = (!isnan(alt_est_m) && alt_est_m < 0.10f);
  //   if (near_ground || (have_ext && landed_state == MAV_LANDED_STATE_ON_GROUND)) {
  //     enter_state(ST_DISARMING);
  //   } else {
  //     enter_state(ST_LANDING);
  //   }
  // }

  // Ceiling safety: keep WORLD vertical (LOCAL_NED) so this doesn't add sideways component when tilted.
  if (ceiling_active && fc_armed) {
    send_vel_frame(0,0, +0.30f, 0, MAV_FRAME_LOCAL_NED);
    return;
  }

  switch (st) {
    case ST_WAIT_LINK:
      enter_state(ST_IDLE);
      break;

    case ST_IDLE:
      if (want_arm && !arm_allowed_by_battery(t)) {
        break;
      }
      if (want_arm && !fc_armed) enter_state(ST_ARMING);
      else if (!want_arm && fc_armed) enter_state(ST_DISARMING);
      else if (want_arm && fc_armed) enter_state(ST_TAKEOFF);
      break;

    case ST_ARMING:
      if (!arm_allowed_by_battery(t)) {
        enter_state(ST_IDLE);
        break;
      }
      if (!fc_armed) {
        set_mode_guided();
        arm_fc();
      } else {
        enter_state(ST_TAKEOFF);
      }
      break;

    case ST_TAKEOFF: {
      if (hb_custom_mode != 4) set_mode_guided();

      // If takeoff ACK denied/temp rejected -> assist immediately
      if (have_takeoff_ack && (t - takeoff_ack_ms) < 2000) {
        if (takeoff_ack_res == MAV_RESULT_DENIED ||
            takeoff_ack_res == MAV_RESULT_TEMPORARILY_REJECTED) {
          printf("TAKEOFF ACK rejected (res=%u) -> LIFTOFF_ASSIST\n", (unsigned)takeoff_ack_res);
          enter_state(ST_LIFTOFF_ASSIST);
          break;
        }
      }

      // Issue guided takeoff command once (and occasionally retry).
      if (!takeoff_sent) {
        guided_takeoff(TAKEOFF_TARGET_M);
        takeoff_sent = true;
        takeoff_sent_ms = t;
        takeoff_no_vel_until_ms = t + TAKEOFF_NO_VEL_MS;
      } else {
        if (!takeoff_started && (t - takeoff_sent_ms) > 3000) {
          guided_takeoff(TAKEOFF_TARGET_M);
          takeoff_sent_ms = t;
          takeoff_no_vel_until_ms = t + TAKEOFF_NO_VEL_MS;
        }
      }

      // Detect "takeoff started" using either motors or off-ground cues.
      bool servo_fresh = have_servo && (t - servo_last_ms) < 250;
      float mot_avg = servo_fresh ? servo_motor_avg() : NAN;
      bool mot_started = servo_fresh && (mot_avg > TAKEOFF_MOT_START_US);
      bool off_ground = takeoff_off_ground(t);

      if (!takeoff_started && (mot_started || off_ground)) {
        takeoff_started = true;
        takeoff_started_ms = t;
        printf("TAKEOFF: started (mot_avg=%.1f, rf=%.2f, landed=%u(%s))\n",
               servo_fresh ? mot_avg : -1.0f,
               (have_rangefinder ? rangefinder_m : NAN),
               (unsigned)landed_state, landed_state_name(landed_state));
      }

      // If NAV_TAKEOFF accepted but motors never ramp: trigger fallback thrust ramp.
      if (!takeoff_started &&
          have_takeoff_ack &&
          takeoff_ack_res == MAV_RESULT_ACCEPTED) {

        uint64_t ref = (takeoff_accept_ms != 0) ? takeoff_accept_ms : takeoff_ack_ms;

        if (!takeoff_thr_ramp_active &&
            !takeoff_not_started_printed &&
            ref != 0 &&
            (t - ref) >= TAKEOFF_START_CHECK_MS) {

          if (servo_fresh && mot_avg <= TAKEOFF_MOT_START_US) {
            takeoff_not_started_printed = true;
            printf("takeoff not started (NAV_TAKEOFF accepted, mot_avg=%.1f). Entering conservative thrust ramp...\n", mot_avg);
            takeoff_thr_ramp_active = true;
            takeoff_thr_ramp_start_ms = t;
            takeoff_thr_ramp_last_send_ms = 0;
          }
        }
      }

      if (takeoff_thr_ramp_active) {
        if (!have_yaw_target && have_att) {
          have_yaw_target = true;
          yaw_target_deg = current_heading_deg();
        }

        takeoff_thrust_ramp_tick(t);

        if (takeoff_off_ground(t) || (servo_fresh && mot_avg > TAKEOFF_MOT_START_US)) {
          printf("TAKEOFF_RAMP: exit (rf=%.2f, landed=%u(%s), mot_avg=%.1f). Re-issuing GUIDED TAKEOFF.\n",
                 (have_rangefinder ? rangefinder_m : NAN),
                 (unsigned)landed_state, landed_state_name(landed_state),
                 servo_fresh ? mot_avg : -1.0f);

          takeoff_thr_ramp_active = false;
          takeoff_started = true;
          takeoff_started_ms = t;

          guided_takeoff(TAKEOFF_TARGET_M);
          takeoff_no_vel_until_ms = t + TAKEOFF_NO_VEL_MS;
        } else if ((t - takeoff_thr_ramp_start_ms) > TO_RAMP_ABORT_MS) {
          printf("TAKEOFF_RAMP: timeout (still not off ground). Falling back to LIFTOFF_ASSIST.\n");
          takeoff_thr_ramp_active = false;
          enter_state(ST_LIFTOFF_ASSIST);
        }

        break;
      }

      if (!z_ctrl_ok(t) && !takeoff_started && !isnan(alt_est_m) && alt_est_m < 0.10f && (t - takeoff_sent_ms) > 1200) {
        printf("Z ALT CTRL unhealthy + takeoff stalled -> LIFTOFF_ASSIST\n");
        enter_state(ST_LIFTOFF_ASSIST);
        break;
      }

      if (!takeoff_started && (t - takeoff_sent_ms) > 4500) {
        printf("TAKEOFF stalled (alt=%.2f, mot_avg=%.1f) -> LIFTOFF_ASSIST\n",
               alt_est_m, (have_servo ? servo_motor_avg() : -1.0f));
        enter_state(ST_LIFTOFF_ASSIST);
        break;
      }

      // Transition to hover when at target (BUT we will not lock position until XY stable)
      if (!isnan(alt_est_m) && alt_est_m >= (TAKEOFF_TARGET_M - 0.05f)) {
        have_yaw_target = have_att;
        yaw_target_deg = current_heading_deg();
        enter_state(ST_HOVER);
      }
    } break;

    case ST_LIFTOFF_ASSIST:
      liftoff_assist_tick(t);
      break;

    case ST_HOVER: {
      if (!have_yaw_target && have_att) {
        have_yaw_target = true;
        yaw_target_deg = current_heading_deg();
      }

      bool xy_stable = vel_xy_stable(t);

      // Delay: don't lock hover position until XY has been stable for >=1s.
      hover_hold_tick(t, xy_stable);

      // Initialize map only when XY becomes stable and we have a valid hover lock snapshot
      if (!map_inited && xy_stable && hover_hold_valid) {
        map_origin_x = hover_hold_x_m;
        map_origin_y = hover_hold_y_m;
        memset(occ_grid, 0, sizeof(occ_grid));
        map_inited = true;
        printf("Map init: origin (x=%.2f, y=%.2f), grid %dx%d @ %.2fm\n",
               map_origin_x, map_origin_y, MAP_W, MAP_H, MAP_RES_M);
      }

      if (HOVER_TEST_ONLY) break;

      // Only proceed to explore if XY is stable (prevents drift-flip immediately after takeoff).
      if (xy_stable && (t - hover_enter_ms) > 1200) {
        enter_state(ST_EXPLORE);
      }
    } break;

    case ST_EXPLORE: {
      // If we aren't stable, DO NOT translate. Hold still in BODY frame.
      bool xy_stable = vel_xy_stable(t);
      if (!xy_stable) {
        send_vel_frame(0,0,0, yaw_hold_rate(), MOVE_FRAME);
        break;
      }

      if (t < explore_pause_until_ms) {
        send_vel_frame(0,0,0, yaw_hold_rate(), MOVE_FRAME);
        break;
      }

      float f = tof_filt_m[D_FRONT];
      bool front_close = (!isnan(f) && f < FRONT_STOP_M);
      if (front_close) {
        turning_dir_forced = false;
        enter_state(ST_TURNING);
        break;
      }

      if (map_inited && have_xy && have_att && (t - last_frontier_eval_ms) > FRONTIER_EVAL_MS) {
        last_frontier_eval_ms = t;

        int sF = frontier_score_dir(lpos_x_m, lpos_y_m, current_heading_deg(), 0.0f);
        int sR = frontier_score_dir(lpos_x_m, lpos_y_m, current_heading_deg(), +90.0f);
        int sL = frontier_score_dir(lpos_x_m, lpos_y_m, current_heading_deg(), -90.0f);
        int sB = frontier_score_dir(lpos_x_m, lpos_y_m, current_heading_deg(), 180.0f);

        int best = sF;
        int best_dir = D_FRONT;
        if (sR > best) { best = sR; best_dir = D_RIGHT; }
        if (sL > best) { best = sL; best_dir = D_LEFT; }
        if (sB > best) { best = sB; best_dir = D_BACK; }

        if (best_dir != D_FRONT && best > (sF + 35)) {
          float dist_ok = 0.0f;
          if (best_dir == D_RIGHT) dist_ok = tof_filt_m[D_RIGHT];
          else if (best_dir == D_LEFT) dist_ok = tof_filt_m[D_LEFT];
          else dist_ok = tof_filt_m[D_BACK];

          if (!isnan(dist_ok) && dist_ok > SIDE_SAFE_M) {
            printf("Frontier turn: scores F/R/L/B = %d/%d/%d/%d -> dir=%d\n", sF,sR,sL,sB,best_dir);
            turning_dir_forced = true;
            forced_turn_dir = best_dir;
            enter_state(ST_TURNING);
            break;
          }
        }
      }

      // Forward creep relative to the DRONE (BODY frame), with yaw hold.
      send_vel_frame(+FWD_VEL, 0.0f, 0.0f, yaw_hold_rate(), MOVE_FRAME);
    } break;

    case ST_TURNING: {
      // If XY isn't stable, we still allow yaw-only turning (body frame), but no translation.
      if (!turning_init) {
        if (turning_dir_forced) {
          turning_dir = forced_turn_dir;
          turning_dir_forced = false;
        } else {
          turning_dir = choose_turn_dir_frontier(t);
        }

        float cur = current_heading_deg();
        float delta = 0.0f;
        if (turning_dir == D_RIGHT) delta = +90.0f;
        else if (turning_dir == D_LEFT) delta = -90.0f;
        else delta = 180.0f;

        turn_target_deg = wrap_deg(cur + delta);
        turn_start_ms = t;
        turning_init = true;
      }

      float cur = current_heading_deg();
      float err = wrap_deg(turn_target_deg - cur);

      float yr = err * 0.8f;
      if (yr > YAW_RATE_DPS) yr = YAW_RATE_DPS;
      if (yr < -YAW_RATE_DPS) yr = -YAW_RATE_DPS;

      // Yaw-only command in BODY frame (avoids relying on world-frame while estimator is shaky)
      send_vel_frame(0,0,0, yr, MOVE_FRAME);

      if (fabsf(err) < 6.0f || (t - turn_start_ms) > 6000) {
        have_yaw_target = true;
        yaw_target_deg = turn_target_deg;
        turning_init = false;
        enter_state(ST_EXPLORE);
      }
    } break;

    case ST_LANDING: {
      if (!land_mode_sent) {
        set_mode_land();
        land_mode_sent = true;
        land_mode_sent_ms = t;
      } else {
        if ((t - land_mode_sent_ms) > 2000) {
          set_mode_land();
          land_mode_sent_ms = t;
        }
      }

      // Keep WORLD vertical descent (LOCAL_NED) to avoid lateral component when tilted.
      send_vel_frame(0,0,+0.15f, 0, MAV_FRAME_LOCAL_NED);

      bool near_ground = (!isnan(alt_est_m) && alt_est_m < 0.10f);
      if (near_ground || (have_ext && landed_state == MAV_LANDED_STATE_ON_GROUND)) {
        enter_state(ST_DISARMING);
      }
    } break;

    case ST_DISARMING: {
      bool near_ground = (!isnan(alt_est_m) && alt_est_m < 0.10f);
      bool on_ground = (have_ext && landed_state == MAV_LANDED_STATE_ON_GROUND);

      if (fc_armed) {
        disarm_fc();

        if (disarm_start_ms == 0) disarm_start_ms = t;

        if ((t - disarm_start_ms) > 2000 && (near_ground || on_ground)) {
          disarm_fc_force();
        }
      } else {
        disarm_start_ms = 0;
        enter_state(ST_IDLE);
      }
    } break;

    default:
      enter_state(ST_IDLE);
      break;
  }
}

// ----------------------------- main ------------------------------------
int main(int argc, char** argv) {
  // usage: ./uav_fc_tof_nav --sysid 252
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--sysid") == 0 && (i + 1) < argc) {
      int v = atoi(argv[i + 1]);
      if (v > 0 && v < 255) {
        g_sysid = (uint8_t)v;
        printf("Using MAVLink sysid=%u\n", (unsigned)g_sysid);
      }
      i++;
    }
  }

  fc_fd = open_uart(FC_UART, FC_BAUD);
  if (fc_fd < 0) return 1;
  printf("Opened FC UART: %s @%d\n", FC_UART, FC_BAUD);

  tof_fd = open_uart(TOF_UART, TOF_BAUD);
  if (tof_fd < 0) return 1;
  printf("Opened ToF UART: %s @%d\n", TOF_UART, TOF_BAUD);

  log_init();

  while (1) {
    struct pollfd pfd[2];
    pfd[0].fd = fc_fd;  pfd[0].events = POLLIN;
    pfd[1].fd = tof_fd; pfd[1].events = POLLIN;

    poll(pfd, 2, 20);

    if (pfd[0].revents & POLLIN) pump_fc_uart();
    if (pfd[1].revents & POLLIN) pump_tof_uart();

    control_tick();
  }

  return 0;
}
