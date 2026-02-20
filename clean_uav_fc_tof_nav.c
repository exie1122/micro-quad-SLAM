// uav_fc_local_nav.c  (stability-first “ETH-ish” single-drone behavior)
//
// STABILITY/DEMO REVISION:
// - Hover-only behavior (exploration removed for minimal hover testing)
// - Faster XY lock: capture first fresh LOCAL_POSITION_NED once airborne > HOVER_CAPTURE_MIN_ALT_M
// - Always hold Z using a Z-only LOCAL_NED setpoint (ignore X/Y) even while XY is not yet locked
// - Ceiling uses MAX of available altitude sources (RF and LPOS) so a “stuck-low” RF can’t hide a real climb
// - DISARM: force-disarm immediately when want_arm drops (param2=21196), any state/mode
// - Line-buffered stdout / unbuffered stderr for reliable logging
// - Disable per-tick EXT_SYS_STATE spam by default (printing at 50Hz can wreck timing)
//
// IMPORTANT NOTE:
// Companion code can *request* poshold/althold, but if your FC estimator/OF/rangefinder is bad,
// the FC will drift anyway. This script makes the companion-side behavior as strict as possible
// (continuous setpoint streaming + gating), and failsafes to LAND if pose/alt is unhealthy.

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
#define LOG_TXT_PATH      "log.txt"
#define LOG_HZ            20
#define LOG_FLUSH_MS      1000

static FILE* log_fp  = NULL;
static FILE* scan_fp = NULL;
static FILE* txt_log_fp = NULL;
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
// Require a short confirmation window for DISARM to reject spurious headers in the ToF stream.
#define CTRL_DISARM_CONFIRM_MS 500
#define CTRL_DISARM_MIN_STREAK 2

static uint8_t ctrl_rxbuf[CTRL_BYTES];
static int     ctrl_rxpos = 0;

// Last-good ToF frame for offline logging
static bool     have_scan_frame = false;
static uint32_t last_scan_t_ms  = 0;
static uint64_t last_scan_host_ms = 0;
static uint8_t  last_scan_grid_raw[TOTAL_CELLS * 2]; // 512 bytes (LE u16 mm), physical order F/R/B/L
static volatile bool scan_new = false;   // set on receive, consumed by logger

// Physical order in the packet is FRONT, RIGHT, BACK, LEFT
enum Dir { D_FRONT=0, D_RIGHT=1, D_BACK=2, D_LEFT=3 };

// ----------------------------- ToF processing --------------------------
#define TOF_COLS 8
#define TOF_ROWS 8

static float tof_beams_m[4][TOF_COLS];             // [dir][col] distance (m)
static float tof_min_m[4]  = { NAN,NAN,NAN,NAN };  // min across beams (per dir)
static float tof_filt_m[4] = { NAN,NAN,NAN,NAN };  // filtered dir-min

// -------------------------- Stability-first params ---------------------
// Hover/PosHold parameters (indoor stability-first)
static const float HOVER_TARGET_M = 0.45f;          // desired hover height (meters above ground)
static const float TAKEOFF_TARGET_M = 0.35f;        // first stop when climbing
static const float CEIL_M = 0.90f;                  // hard max altitude (meters)
static const float CEIL_MARGIN_M = 0.05f;            // margin to command descent before the ceiling
static const float HOVER_CAPTURE_MIN_ALT_M = 0.15f;  // don't lock XY until clearly airborne
static const uint64_t PREARM_STABLE_MS = 400;       // require sensors/control stable before arming
static const bool REQUIRE_RANGEFINDER_FOR_HOVER = true;
static const bool REQUIRE_OPTICAL_FLOW_FOR_HOVER = true;

// ToF constraints
static const float TOF_MAX_RANGE_M = 4.00f;

// ---------------------- Liftoff assist (no-RC bootstrap) ---------------
#define RC_CH_ROLL     1
#define RC_CH_PITCH    2
#define RC_CH_THROTTLE 3
#define RC_CH_YAW      4

static const uint16_t RC_NEUTRAL_US = 1500;
static const uint16_t ASSIST_THR_US_MIN = 1550;   // quicker liftoff start near neutral
static const uint16_t ASSIST_THR_US_MAX = 1850;   // push harder to break ground
static const uint64_t ASSIST_SEND_PERIOD_MS = 40; // 25 Hz
static const uint64_t ASSIST_TOTAL_MS = 800;      // fast ramp
static const uint64_t ASSIST_ABORT_MS = 2000;     // bail sooner
static const uint64_t ASSIST_OVERRIDE_EFFECT_MS = 250;
static const float    ASSIST_MOTOR_DELTA_MIN = 15.0f; // lower delta threshold

// ---------------------- TAKEOFF gating + thrust ramp -------------------
#define PRINT_LANDED_STATE_EACH_TICK 0  // set to 1 only for short debugging; hurts timing at 50Hz

static const uint64_t TAKEOFF_NO_VEL_MS        = 900;     // pause streaming setpoints right after NAV_TAKEOFF
static const uint64_t TAKEOFF_RAMP_DELAY_MS    = 700;     // start ramp a bit sooner
static const float    TAKEOFF_MOT_START_US      = 1150.0f;
static const uint64_t TAKEOFF_TIMEOUT_MS        = 8000;

static bool     takeoff_started                 = false;
static uint64_t takeoff_started_ms              = 0;
static float    takeoff_alt0_m                  = NAN;    // altitude snapshot when NAV_TAKEOFF sent

FILE *csv_fp = NULL;
// Data storage
float vib_x=0, vib_y=0, vib_z=0;
uint32_t clip0=0, clip1=0, clip2=0;
uint16_t motor_pwm[4] = {0};
int32_t esc_rpm[4] = {0};

// ----------------------------- Battery safety (2S LiHV) ----------------
static const float    BATT_ARM_MIN_VPC   = 3.70f;
static const float    BATT_LAND_VPC      = 3.55f;
static const float    BATT_EMERG_VPC     = 3.35f;
static const uint64_t BATT_LOW_HOLD_MS   = 1200;
static const uint64_t BATT_FRESH_MS      = 2000;
static uint64_t batt_last_ms             = 0;
static uint64_t batt_low_since_ms        = 0;
static uint64_t batt_emerg_since_ms      = 0;
static uint64_t batt_last_warn_ms        = 0;
static bool     batt_valid               = false;
static uint64_t last_batt_log_ms         = 0;

// ----------------------------- Keyframes (scan log) --------------------
#define KF_NONE         0
#define KF_TAKEOFF      (1u<<0)
#define KF_LAND_START   (1u<<1)
#define KF_LIFTOFF_AST  (1u<<2)
#define KF_BATT_LAND    (1u<<3)
#define KF_BATT_EMERG   (1u<<4)

static uint8_t pending_kf_flags = 0;

// ----------------------------- RC channel mapping ----------------------
static int  rcmap_roll = 1;
static int  rcmap_pitch = 2;
static int  rcmap_throttle = 3;
static int  rcmap_yaw = 4;
static bool rcmap_roll_ok = false;
static bool rcmap_pitch_ok = false;
static bool rcmap_throttle_ok = false;
static bool rcmap_yaw_ok = false;
static bool rcmap_known = false;
static uint64_t rcmap_last_request_ms = 0;

// ----------------------------- MAVLink state ---------------------------
static int fc_fd  = -1;
static int tof_fd = -1;

// Our sysid/compid
static uint8_t g_sysid  = 255;
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
static float batt_v_total_sys = NAN;
static uint64_t batt_sys_last_ms = 0;

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

// RC Input (snapshot)
static bool     have_rcin = false;
static uint16_t rcin[16] = {0};
static uint8_t  rcin_rssi = 0;
static uint64_t rcin_last_ms = 0;

// ----------------------------- Ring Buffer -----------------------------
#define SNAP_RING_SZ 32

typedef struct {
  uint64_t t_ms;
  uint8_t  state;
  uint32_t mode;
  bool     armed;
  uint8_t  landed;
  
  float    roll, pitch, yaw;
  float    x, y, z;
  float    vx, vy, vz;
  float    alt_est;
  uint8_t  alt_src;

  float    rf_m;
  uint8_t  of_q;
  float    of_rx, of_ry;
  
  bool     xy_ok, z_ok, gyr_ok, mot_ok;

  float    batt_v;
  int      batt_c;
  float    batt_vpc;
  
  uint16_t mot[4];
  uint16_t rc[4];
  uint8_t  rssi;
} snapshot_t;

static snapshot_t snap_ring[SNAP_RING_SZ];
static int        snap_head = 0;

static void snap_add(uint64_t t);
static void snap_dump(void);

// ----------------------------- State machine ---------------------------
typedef enum {
  ST_WAIT_LINK = 0,
  ST_IDLE,
  ST_ARMING,
  ST_TAKEOFF,
  ST_LIFTOFF_ASSIST,
  ST_HOVER,
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

// We separate “Z/YAW hold is active” from “XY lock is valid”.
// - We always clamp Z (Z-only setpoint) once we start controlling.
// - We lock XY as soon as LOCAL_POSITION_NED is fresh and airborne above HOVER_CAPTURE_MIN_ALT_M.
static float hover_hold_yaw_deg = NAN;
static bool  hover_xy_locked = false;      // true only after XY lock achieved
static bool  hover_xy_prelock_valid = false;
static float hover_prelock_x_m = 0.0f;
static float hover_prelock_y_m = 0.0f;
static uint64_t hover_xy_lock_ms = 0;
static float hover_lock_x_m = 0.0f;
static float hover_lock_y_m = 0.0f;

// Ceiling logic
static bool ceiling_active = false;
static float alt_est_m = NAN;       // selected altitude estimate
static float alt_max_m = NAN;       // MAX of available sources for ceiling safety

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

// Takeoff attitude thrust ramp (GUIDED) to overcome idle/hover refusal
static bool     takeoff_att_ramp_active = false;
static uint64_t takeoff_att_ramp_start_ms = 0;

// Disarm timer
static uint64_t disarm_start_ms = 0;

// Stale-sensor hysteresis
static uint32_t lpos_stale_count = 0;
static uint32_t rf_stale_count   = 0;
static uint32_t alt_stale_count  = 0;
static const uint32_t STALE_FAIL_TICKS = 40; // ~1–2s depending on loop rate

// ----------------------------- Timing helpers --------------------------
#define PRINT_HZ 1
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

// ----------------------------- Logging Helper --------------------------
static void log_msg(const char* fmt, ...) {
  va_list args;

  // 1. Print to console (stdout)
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
  fflush(stdout);

  // 2. Print to log file (if open)
  if (txt_log_fp) {
    // Add timestamp prefix
    fprintf(txt_log_fp, "[%.3f] ", now_ms() * 0.001f);
    
    va_start(args, fmt);
    vfprintf(txt_log_fp, fmt, args);
    va_end(args);

    // Robustness: flush
    fflush(txt_log_fp);
  }
}

// Redirect all normal printf calls to log_msg
#define printf log_msg

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
    default: sp = B57600; break;
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

static void request_param_read(const char* name) {
  if (!have_fc) return;
  mavlink_message_t m;
  mavlink_msg_param_request_read_pack(
      g_sysid, g_compid, &m,
      fc_sysid, fc_compid,
      name, -1
  );
  mav_send(&m);
}

static bool rcmap_unique(void) {
  int v[4] = { rcmap_roll, rcmap_pitch, rcmap_throttle, rcmap_yaw };
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 4; j++) {
      if (v[i] == v[j]) return false;
    }
  }
  return true;
}

static void update_rcmap_known(void) {
  if (!(rcmap_roll_ok && rcmap_pitch_ok && rcmap_throttle_ok && rcmap_yaw_ok)) return;
  if (!rcmap_unique()) {
    rcmap_known = false;
    printf("RCMAP invalid (duplicate channels): roll=%d pitch=%d thr=%d yaw=%d\n",
           rcmap_roll, rcmap_pitch, rcmap_throttle, rcmap_yaw);
    return;
  }
  if (!rcmap_known) {
    rcmap_known = true;
    printf("RCMAP: roll=%d pitch=%d thr=%d yaw=%d\n",
           rcmap_roll, rcmap_pitch, rcmap_throttle, rcmap_yaw);
  }
}

static void request_rcmap_params(void) {
  rcmap_last_request_ms = now_ms();
  request_param_read("RCMAP_ROLL");
  request_param_read("RCMAP_PITCH");
  request_param_read("RCMAP_THROTTLE");
  request_param_read("RCMAP_YAW");
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
static bool set_mode_custom(uint32_t custom_mode, const char* name) {
  if (!have_fc) return false;
  if ((uint32_t)hb_custom_mode == custom_mode) return false;

  uint64_t t = now_ms();
  if (t - last_mode_cmd_ms < 800) return false;
  last_mode_cmd_ms = t;

  if (name) {
    printf("Requesting mode %s ... (current=%u)\n", name, (unsigned)hb_custom_mode);
  }

  {
    mavlink_message_t m;
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    mavlink_msg_set_mode_pack(g_sysid, g_compid, &m, fc_sysid, base_mode, custom_mode);
    mav_send(&m);
  }

  send_command_long(MAV_CMD_DO_SET_MODE,
                    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)custom_mode, 0,0,0,0,0);
  return true;
}

static void set_mode_guided(void) {
  (void)set_mode_custom(4, "GUIDED");
}

static void set_mode_land(void) {
  (void)set_mode_custom(9, "LAND");
}

static void set_mode_stabilize(void) {
  (void)set_mode_custom(0, "STABILIZE");
}

static void arm_fc(void) {
  if (!have_fc) return;

  uint64_t t = now_ms();
  if (t - last_arm_cmd_ms < 800) return;
  last_arm_cmd_ms = t;

  printf("Requesting ARM...\n");
  send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, 1, 0,0,0,0,0,0);
}

static void disarm_fc_force(void) {
  if (!have_fc) return;

  static uint64_t last_disarm_log_ms = 0;

  uint64_t t = now_ms();
  if (t - last_disarm_cmd_ms < 800) return;
  last_disarm_cmd_ms = t;

  if (t - last_disarm_log_ms > 2000) {
    last_disarm_log_ms = t;
    printf("Requesting FORCE DISARM (21196)...\n");
  }
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

  // Log command (throttled)
  static uint64_t last_log = 0;
  uint64_t t = now_ms();
  if (t - last_log > 2000) {
    last_log = t;
    char buf[64];
    snprintf(buf, sizeof(buf), "CMD_VEL frame=%u vx=%.2f vy=%.2f vz=%.2f yr=%.1f", 
       frame, vx, vy, vz, yaw_rate_deg_s);
    if(log_fp) { fprintf(log_fp, "# %s\n", buf); }
    if(txt_log_fp) { fprintf(txt_log_fp, "[%.3f] %s\n", t*0.001f, buf); }
  }

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

  // Log command (throttled)
  static uint64_t last_log = 0;
  uint64_t t = now_ms();
  if (t - last_log > 2000) {
    last_log = t;
    char buf[64];
    snprintf(buf, sizeof(buf), "CMD_POS x=%.2f y=%.2f z=%.2f yaw=%.1f", x, y, z_down, yaw_deg);
    if(log_fp) { fprintf(log_fp, "# %s\n", buf); }
    if(txt_log_fp) { fprintf(txt_log_fp, "[%.3f] %s\n", t*0.001f, buf); }
  }

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

// Z-only + Yaw hold command in LOCAL_NED (ignore X/Y positions)
static void send_z_yaw_ned(float z_down, float yaw_deg) {
  if (!have_fc) return;

  static uint64_t last_log = 0;
  uint64_t t = now_ms();
  if (t - last_log > 2000) {
    last_log = t;
    char buf[64];
    snprintf(buf, sizeof(buf), "CMD_Z_YAW z=%.2f yaw=%.1f", z_down, yaw_deg);
    if(log_fp) { fprintf(log_fp, "# %s\n", buf); }
    if(txt_log_fp) { fprintf(txt_log_fp, "[%.3f] %s\n", t*0.001f, buf); }
  }

  uint16_t mask =
      (1<<0)|(1<<1) |            // ignore x,y position
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
      0.0f, 0.0f, z_down,
      0,0,0,
      0,0,0,
      deg2rad(yaw_deg), 0
  );
  mav_send(&m);
}

// Conservative attitude+thrust setpoint (GUIDED) using ENCODE.
static void send_attitude_target_thrust(float thrust, float yaw_deg) {
  if (!have_fc) return;

  static uint64_t last_log = 0;
  uint64_t t = now_ms();
  if (t - last_log > 500) { // Log this one more frequently (ramp)
    last_log = t;
    char buf[64];
    snprintf(buf, sizeof(buf), "CMD_ATT thr=%.2f yaw=%.1f", thrust, yaw_deg);
    if(log_fp) { fprintf(log_fp, "# %s\n", buf); }
    if(txt_log_fp) { fprintf(txt_log_fp, "[%.3f] %s\n", t*0.001f, buf); }
  }

  if (thrust < 0.0f) thrust = 0.0f;
  if (thrust > 0.90f) thrust = 0.90f;

  mavlink_set_attitude_target_t at;
  memset(&at, 0, sizeof(at));
  at.time_boot_ms = (uint32_t)now_ms();
  at.target_system = fc_sysid;
  at.target_component = fc_compid;

  // ignore body rates, use attitude + thrust
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
  for (int i = 0; i < MAVLINK_MSG_SET_ATTITUDE_TARGET_FIELD_THRUST_BODY_LEN; i++) {
    at.thrust_body[i] = 0.0f;
  }
#endif

  mavlink_message_t m;
  mavlink_msg_set_attitude_target_encode(g_sysid, g_compid, &m, &at);
  mav_send(&m);
}

// RC override helpers
static void set_rc_override_chan(mavlink_rc_channels_override_t* o, int ch, uint16_t v) {
  switch (ch) {
    case 1: o->chan1_raw = v; break;
    case 2: o->chan2_raw = v; break;
    case 3: o->chan3_raw = v; break;
    case 4: o->chan4_raw = v; break;
    case 5: o->chan5_raw = v; break;
    case 6: o->chan6_raw = v; break;
    case 7: o->chan7_raw = v; break;
    case 8: o->chan8_raw = v; break;
    default: break;
  }
}

static void rc_override_send_mapped(uint16_t roll_us, uint16_t pitch_us, uint16_t thr_us, uint16_t yaw_us) {
  if (!have_fc) return;
  
  // Use defaults if unknown to prevent the "Stabilize Flip"
  int r = rcmap_known ? rcmap_roll : 1;
  int p = rcmap_known ? rcmap_pitch : 2;
  int t = rcmap_known ? rcmap_throttle : 3;
  int y = rcmap_known ? rcmap_yaw : 4;

  static uint64_t last_log = 0;
  uint64_t t_ms = now_ms();
  if (t_ms - last_log > 1000) {
    last_log = t_ms;
    char buf[64];
    snprintf(buf, sizeof(buf), "CMD_RC OVR r=%u p=%u t=%u y=%u", roll_us, pitch_us, thr_us, yaw_us);
    if(log_fp) { fprintf(log_fp, "# %s\n", buf); }
    if(txt_log_fp) { fprintf(txt_log_fp, "[%.3f] %s\n", t_ms*0.001f, buf); }
  }

  mavlink_message_t m;
  mavlink_rc_channels_override_t o;
  memset(&o, 0xFF, sizeof(o));

  o.target_system = fc_sysid;
  o.target_component = fc_compid;

  set_rc_override_chan(&o, r, roll_us);
  set_rc_override_chan(&o, p, pitch_us);
  set_rc_override_chan(&o, t, thr_us);
  set_rc_override_chan(&o, y, yaw_us);

  mavlink_msg_rc_channels_override_encode(g_sysid, g_compid, &m, &o);
  mav_send(&m);
}

static void rc_override_release(void) {
  if (!have_fc) return;
  mavlink_message_t m;
  mavlink_rc_channels_override_t o;
  memset(&o, 0xFF, sizeof(o));
  o.target_system = fc_sysid;
  o.target_component = fc_compid;
  mavlink_msg_rc_channels_override_encode(g_sysid, g_compid, &m, &o);
  mav_send(&m);
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

  // Gyro health remains a hard requirement.
  if (!sys_health_bit(MAV_SYS_STATUS_SENSOR_3D_GYRO, t)) return true;

  // Motor outputs: only fail if enabled AND unhealthy. ArduPilot often marks
  // motor outputs “unhealthy” while disarmed because they’re simply disabled.
  bool mot_enabled = (sys_enabled & MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS) != 0;
  bool mot_healthy = sys_health_bit(MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS, t);
  if (mot_enabled && !mot_healthy) return true;

  return false;
}

static bool z_ctrl_ok(uint64_t t) {
  if (!sys_fresh(t)) return true;
  bool z_enabled = (sys_enabled & MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL) != 0;
  bool z_healthy = sys_health_bit(MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL, t);
  return z_enabled ? z_healthy : true;
}

static bool xy_ctrl_ok(uint64_t t) {
  if (!sys_fresh(t)) return true;
  bool xy_enabled = (sys_enabled & MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL) != 0;
  bool xy_healthy = sys_health_bit(MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL, t);
  return xy_enabled ? xy_healthy : true;
}

static bool of_fresh(uint64_t t) {
  return have_of && (t - of_last_update_ms) < 400;
}

static bool batt_vpc_valid(float vpc) {
  return !isnan(vpc) && vpc >= 1.0f && vpc <= 6.0f;
}

static bool batt_sys_fresh(uint64_t t) {
  return (batt_sys_last_ms != 0) && (t - batt_sys_last_ms) < BATT_FRESH_MS && !isnan(batt_v_total_sys);
}

static bool batt_fresh(uint64_t t) {
  return (batt_last_ms != 0) && (t - batt_last_ms) < BATT_FRESH_MS && batt_vpc_valid(batt_vpc) && batt_cells > 0;
}

static bool batt_vpc_sample(uint64_t t, float* out_vpc) {
  if (batt_fresh(t)) {
    *out_vpc = batt_vpc;
    return true;
  }
  if (batt_sys_fresh(t)) {
    int cells = (batt_cells > 0) ? batt_cells : 2;
    float vpc = batt_v_total_sys / (float)cells;
    if (batt_vpc_valid(vpc)) {
      *out_vpc = vpc;
      return true;
    }
  }
  return false;
}

// ----------------------------- Drift/flip fix gating -------------------
static const uint64_t XY_STABLE_HOLD_MS = 1000;
static uint64_t xy_ok_since_ms = 0;
static uint64_t prearm_ok_since_ms = 0;

static bool vel_xy_allowed(uint64_t t) {
  if (!xy_ctrl_ok(t)) return false;
  if (!have_att) return false;

  bool lpos_fresh = have_lpos && (t - lpos_last_update_ms) < 400;
  if (!lpos_fresh) return false;

  // Permit lower OF quality before lock; rely on velocity stability instead of high Q.
  if (of_fresh(t) && of_quality < 30) return false;

  if (!isnan(alt_max_m) && alt_max_m < 0.12f) return false;

  return true;
}

static bool vel_xy_stable(uint64_t t) {
  bool ok = vel_xy_allowed(t);
  if (ok) {
    if (xy_ok_since_ms == 0) xy_ok_since_ms = t;
    return (t - xy_ok_since_ms) >= XY_STABLE_HOLD_MS;
  } else {
    xy_ok_since_ms = 0;
    return false;
  }
}

// ----------------------------- Hover / PosHold -------------------------
static bool hover_ready_now(uint64_t t) {
  bool lpos_fresh  = have_lpos && (t - lpos_last_update_ms) < 400;
  bool rf_fresh    = have_rangefinder && (t - rangefinder_last_update_ms) < 400;
  bool rf_ok       = rf_fresh && !isnan(rangefinder_m);
  bool of_ok       = of_fresh(t) && of_quality >= 30;

  if (!have_att) return false;
  if (!lpos_fresh) return false;

  if (!xy_ctrl_ok(t) || !z_ctrl_ok(t)) return false;

  if (REQUIRE_RANGEFINDER_FOR_HOVER) {
    if (!rf_ok) return false;
  } else {
    if (isnan(alt_est_m)) return false;
  }

  if (REQUIRE_OPTICAL_FLOW_FOR_HOVER) {
    if (!of_ok) {
      // Before arming, allow arming even if flow not yet healthy; hover loop will still enforce later.
      if (fc_armed) return false;
    }
  }

  if (isnan(alt_max_m)) return false;
  return true;
}

static bool hover_ready_stable(uint64_t t) {
  bool ok = hover_ready_now(t);
  if (ok) {
    if (prearm_ok_since_ms == 0) prearm_ok_since_ms = t;
    return (t - prearm_ok_since_ms) >= PREARM_STABLE_MS;
  } else {
    prearm_ok_since_ms = 0;
    return false;
  }
}

static float hover_target_z_down(void) {
  float z_up = HOVER_TARGET_M;
  float max_up = CEIL_M - CEIL_MARGIN_M;
  if (max_up < 0.10f) max_up = 0.10f;
  if (z_up > max_up) z_up = max_up;
  return -z_up;
}

static void init_hover_targets_on_ground(uint64_t t) {
  (void)t;
  hover_xy_locked = false;
  hover_xy_prelock_valid = false;
  hover_xy_lock_ms = 0;
  hover_lock_x_m = 0.0f;
  hover_lock_y_m = 0.0f;
  hover_prelock_x_m = 0.0f;
  hover_prelock_y_m = 0.0f;

  if (have_att) {
    hover_hold_yaw_deg = current_heading_deg();
    have_yaw_target = true;
    yaw_target_deg = hover_hold_yaw_deg;
  } else {
    hover_hold_yaw_deg = 0.0f;
  }
}

static void hover_hold_tick(uint64_t t) {
  if (!have_att) return;

  uint64_t lpos_age_ms = have_lpos ? (t - lpos_last_update_ms) : UINT64_MAX;
  bool lpos_recent = lpos_age_ms < 400;

  // Capture a prelock XY snapshot as soon as we have a fresh sample and are above the min altitude.
  if (!hover_xy_prelock_valid && lpos_recent && isfinite(lpos_x_m) && isfinite(lpos_y_m) &&
      !isnan(alt_max_m) && alt_max_m > HOVER_CAPTURE_MIN_ALT_M) {
    hover_prelock_x_m = lpos_x_m;
    hover_prelock_y_m = lpos_y_m;
    hover_xy_prelock_valid = true;
    printf("HOVER: prelock XY captured (x=%.2f y=%.2f)\n", hover_prelock_x_m, hover_prelock_y_m);
  }

  // Lock XY only after stability criteria are met.
  if (!hover_xy_locked && vel_xy_stable(t)) {
    if (hover_xy_prelock_valid) {
      hover_lock_x_m = hover_prelock_x_m;
      hover_lock_y_m = hover_prelock_y_m;
    } else if (lpos_recent && isfinite(lpos_x_m) && isfinite(lpos_y_m)) {
      hover_lock_x_m = lpos_x_m;
      hover_lock_y_m = lpos_y_m;
    }
    hover_xy_locked = true;
    hover_xy_lock_ms = t;
    printf("HOVER: XY LOCKED at x=%.2f y=%.2f\n", hover_lock_x_m, hover_lock_y_m);
  }

  float yaw = have_yaw_target ? yaw_target_deg : current_heading_deg();
  float z_down = hover_target_z_down();

  if (!hover_xy_locked || !lpos_recent) {
    // Always stream Z+Yaw even if XY data is missing; XY hold requires a fresh lock.
    send_z_yaw_ned(z_down, yaw);
  } else {
    send_pos_yaw_ned(hover_lock_x_m, hover_lock_y_m, z_down, yaw);
  }
}

// ----------------------------- Requests --------------------------------
static void request_streams(void) {
  const uint8_t tgt_sys  = fc_sysid;
  const uint8_t tgt_comp = 0;

  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_SYS_STATUS,          200000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,     50000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_RC_CHANNELS,          200000.0f, 0,0,0,0,0);

  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_BATTERY_STATUS,      200000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_DISTANCE_SENSOR,     100000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_EXTENDED_SYS_STATE,  200000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE,             50000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_LOCAL_POSITION_NED,   50000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_OPTICAL_FLOW,         50000.0f, 0,0,0,0,0);
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_OPTICAL_FLOW_RAD,     50000.0f, 0,0,0,0,0);

  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, (float)MAVLINK_MSG_ID_RANGEFINDER,  100000.0f, 0,0,0,0,0);
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
  uint64_t t = now_ms();
  static uint64_t last_ack_log_ms = 0;
  static uint16_t last_ack_log_cmd = 0;
  static uint8_t last_ack_log_res = 0;

  if (a.command == MAV_CMD_NAV_TAKEOFF) {
    have_takeoff_ack = true;
    takeoff_ack_res = a.result;
    takeoff_ack_ms = now_ms();
  }

  if (a.command != last_ack_log_cmd ||
      a.result != last_ack_log_res ||
      (t - last_ack_log_ms) > 1000) {
    last_ack_log_ms = t;
    last_ack_log_cmd = a.command;
    last_ack_log_res = a.result;
    log_msg("ACK: cmd=%u res=%u\n", (unsigned)a.command, (unsigned)a.result);
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
  
  uint64_t t = sys_last_ms;
  bool do_log = (t - last_batt_log_ms) > 1000;
  if(do_log) {
    last_batt_log_ms = t;
    if(log_fp) fprintf(log_fp, "# BATT_RAW: src=SYS_STATUS voltage_battery_mV=%u\n", s.voltage_battery);
    if(txt_log_fp) fprintf(txt_log_fp, "[%.3f] BATT_RAW: src=SYS_STATUS voltage_battery_mV=%u\n", t*0.001f, s.voltage_battery);
  }

  if (s.voltage_battery > 0 && s.voltage_battery < 60000) {
    batt_v_total_sys = (float)s.voltage_battery * 0.001f;
    batt_sys_last_ms = sys_last_ms;
    
    // Compute provisional calc for logging if this is our only source
    if (do_log && (t - batt_last_ms) > 2000) { // If BATTERY_STATUS is stale
       if (batt_v_total_sys < 3.0f || batt_v_total_sys > 30.0f) {
           batt_valid = false;
           if(log_fp) fprintf(log_fp, "# BATT_INVALID: ignoring battery (sys V=%.2f)\n", batt_v_total_sys);
       } else {
           // We don't update batt_valid=true here blindly because we lack cells. 
           // We'll let battery_failsafe logic handle the merge or infer.
           // But user asked to log BATT_CALC right after.
           int cells = (batt_cells > 0) ? batt_cells : 3; // minimal guess for logging
           float vpc = batt_v_total_sys / cells;
           if(log_fp) fprintf(log_fp, "# BATT_CALC: Vpack=%.2f cells=%d Vpc=%.2f (SYS)\n", batt_v_total_sys, cells, vpc);
       }
    }
  }
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
  // Also store to our logging array:
  motor_pwm[0] = so.servo1_raw;
  motor_pwm[1] = so.servo2_raw;
  motor_pwm[2] = so.servo3_raw;
  motor_pwm[3] = so.servo4_raw;
  servo_last_ms = now_ms();
  have_servo = true; 
}

static void handle_vibration(const mavlink_message_t *msg) {
  mavlink_vibration_t v;
  mavlink_msg_vibration_decode(msg, &v);
  vib_x = v.vibration_x;
  vib_y = v.vibration_y;
  vib_z = v.vibration_z;
  clip0 = v.clipping_0;
  clip1 = v.clipping_1;
  clip2 = v.clipping_2;
}

static void handle_esc_status(const mavlink_message_t *msg) {
  mavlink_esc_status_t e;
  mavlink_msg_esc_status_decode(msg, &e);
  esc_rpm[0] = e.rpm[0];
  esc_rpm[1] = e.rpm[1];
  esc_rpm[2] = e.rpm[2];
  esc_rpm[3] = e.rpm[3];
}

static void handle_battery_status(const mavlink_message_t *msg) {
  mavlink_battery_status_t b;
  mavlink_msg_battery_status_decode(msg, &b);

  uint64_t t = now_ms();
  bool do_log = (t - last_batt_log_ms) > 1000;
  if(do_log) {
    last_batt_log_ms = t;
    if(log_fp) {
        fprintf(log_fp, "# BATT_RAW: src=BATTERY_STATUS voltages=[");
        for(int i=0; i<4 && i<10; i++) fprintf(log_fp, "%u ", b.voltages[i]);
        fprintf(log_fp, "...]\n");
    }
    if(txt_log_fp) fprintf(txt_log_fp, "[%.3f] BATT_RAW: src=BATTERY_STATUS\n", t*0.001f);
  }

  float sum_v = 0.0f;
  int cells = 0;
  for (int i = 0; i < 10; i++) {
    if (b.voltages[i] > 0 && b.voltages[i] < 20000) {
      sum_v += (float)b.voltages[i] * 0.001f;
      cells++;
    }
  }
  
  if (cells > 0) {
    int cells_used = cells;
    if (cells == 1 && sum_v > 6.0f) {
      int inferred = (int)lrintf(sum_v / 4.0f);
      if (inferred < 2) inferred = 2;
      if (inferred > 6) inferred = 6;
      cells_used = inferred;
    }
    
    float new_vpc = sum_v / (float)cells_used;
    bool v_ok = (sum_v >= 3.0f && sum_v <= 30.0f);
    bool c_ok = (cells_used > 0 && cells_used <= 8);
    bool vpc_ok = (new_vpc >= 2.5f && new_vpc <= 4.8f);

    if (v_ok && c_ok && vpc_ok) {
        batt_v_total = sum_v;
        batt_cells = cells_used;
        batt_vpc = new_vpc;
        batt_last_ms = t;
        batt_valid = true;
        if(do_log && log_fp) fprintf(log_fp, "# BATT_CALC: Vpack=%.2f cells=%d Vpc=%.2f\n", batt_v_total, batt_cells, batt_vpc);
    } else {
        batt_valid = false;
        if(do_log) {
            if(log_fp) fprintf(log_fp, "# BATT_INVALID: ignoring battery this cycle (Vpack=%.2f cells=%d Vpc=%.2f)\n", sum_v, cells_used, new_vpc);
            printf("BATT_INVALID: ignoring battery gating this cycle\n");
        }
    }
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

  memset(last_statustext, 0, sizeof(last_statustext));
  strncpy(last_statustext, buf, sizeof(last_statustext)-1);
  last_statustext_sev = s.severity;
  last_statustext_ms = now_ms();

  printf("FC STATUSTEXT sev=%u: %s\n", (unsigned)s.severity, buf);
  fflush(stdout);
}

static void handle_param_value(const mavlink_message_t *msg) {
  mavlink_param_value_t p;
  mavlink_msg_param_value_decode(msg, &p);

  char id[17];
  memset(id, 0, sizeof(id));
  memcpy(id, p.param_id, 16);

  int v = (int)lrintf(p.param_value);
  if (v < 1 || v > 18) return;

  if (strcmp(id, "RCMAP_ROLL") == 0) {
    rcmap_roll = v;
    rcmap_roll_ok = true;
  } else if (strcmp(id, "RCMAP_PITCH") == 0) {
    rcmap_pitch = v;
    rcmap_pitch_ok = true;
  } else if (strcmp(id, "RCMAP_THROTTLE") == 0) {
    rcmap_throttle = v;
    rcmap_throttle_ok = true;
  } else if (strcmp(id, "RCMAP_YAW") == 0) {
    rcmap_yaw = v;
    rcmap_yaw_ok = true;
  } else {
    return;
  }

  update_rcmap_known();
}

static void handle_rc_channels(const mavlink_message_t *msg) {
  mavlink_rc_channels_t r;
  mavlink_msg_rc_channels_decode(msg, &r);
  
  rcin[0] = r.chan1_raw;
  rcin[1] = r.chan2_raw;
  rcin[2] = r.chan3_raw;
  rcin[3] = r.chan4_raw;
  rcin[4] = r.chan5_raw;
  rcin[5] = r.chan6_raw;
  rcin[6] = r.chan7_raw;
  rcin[7] = r.chan8_raw;
  // up to 18 channels supported in struct but we capture 8 usually
  rcin_rssi = r.rssi;
  rcin_last_ms = now_ms();
  have_rcin = true;
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
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {

        if (!have_fc && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          fc_sysid = msg.sysid;
          fc_compid = msg.compid;
          have_fc = true;
          printf("FC connected: sys=%u comp=%u\n", (unsigned)fc_sysid, (unsigned)fc_compid);
          request_streams();
          request_rcmap_params();
        }

        switch (msg.msgid) {
          case MAVLINK_MSG_ID_HEARTBEAT:            handle_heartbeat(&msg); break;
          case MAVLINK_MSG_ID_COMMAND_ACK:         handle_command_ack(&msg); break;
          case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:  handle_extended_sys_state(&msg); break;
          case MAVLINK_MSG_ID_SYS_STATUS:          handle_sys_status(&msg); break;
          case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:    handle_servo_output_raw(&msg); break;
          case MAVLINK_MSG_ID_VIBRATION:           handle_vibration(&msg); break;
          case MAVLINK_MSG_ID_ESC_STATUS:          handle_esc_status(&msg); break;
          case MAVLINK_MSG_ID_BATTERY_STATUS:      handle_battery_status(&msg); break;
          case MAVLINK_MSG_ID_ATTITUDE:            handle_attitude(&msg); break;
          case MAVLINK_MSG_ID_LOCAL_POSITION_NED:  handle_local_position_ned(&msg); break;
          case MAVLINK_MSG_ID_OPTICAL_FLOW:        handle_optical_flow(&msg); break;
          case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:    handle_optical_flow_rad(&msg); break;
          case MAVLINK_MSG_ID_DISTANCE_SENSOR:     handle_distance_sensor(&msg); break;
          case MAVLINK_MSG_ID_RANGEFINDER:         handle_rangefinder_msg(&msg); break;
          case MAVLINK_MSG_ID_PARAM_VALUE:         handle_param_value(&msg); break;
          case MAVLINK_MSG_ID_STATUSTEXT:          handle_statustext(&msg); break;
          case MAVLINK_MSG_ID_RC_CHANNELS:         handle_rc_channels(&msg); break;
          default: break;
        }
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
  last_scan_t_ms = rd_u32_le(frame + 1);
  memcpy(last_scan_grid_raw, frame + 5, sizeof(last_scan_grid_raw));
  last_scan_host_ms = now_ms();
  have_scan_frame = true;
  scan_new = true;

  compute_beams_and_minima(frame);
}

static void accept_ctrl_frame(const uint8_t* frame) {
  uint8_t cmd = frame[1];
  uint32_t seq = rd_u32_le(frame + 2);

  uint64_t t = now_ms();
  static uint32_t last_disarm_seq = 0;
  static uint8_t disarm_streak = 0;
  static uint64_t disarm_first_ms = 0;
  static uint64_t last_ctrl_log_ms = 0;

  if (cmd == 0) {
    bool seq_ok = (seq == last_disarm_seq) || (seq == (last_disarm_seq + 1));
    bool new_window = (disarm_first_ms == 0) ||
                      (t - disarm_first_ms) > CTRL_DISARM_CONFIRM_MS ||
                      (!seq_ok && disarm_streak > 0);

    if (new_window) {
      disarm_first_ms = t;
      disarm_streak = 1;
      last_disarm_seq = seq;
      if (t - last_ctrl_log_ms > 1000) {
        last_ctrl_log_ms = t;
        printf("CTRL: DISARM pending (seq=%u)\n", (unsigned)seq);
      }
      return;
    }

    disarm_streak++;
    last_disarm_seq = seq;
    if (disarm_streak < CTRL_DISARM_MIN_STREAK) {
      if (t - last_ctrl_log_ms > 1000) {
        last_ctrl_log_ms = t;
        printf("CTRL: DISARM pending (seq=%u)\n", (unsigned)seq);
      }
      return;
    }

    disarm_streak = 0;
    disarm_first_ms = 0;
    want_arm = false;
    printf("CTRL: DISARM confirmed (seq=%u)\n", (unsigned)seq);
  } else if (cmd == 1) {
    disarm_streak = 0;
    disarm_first_ms = 0;
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

  // Compute max altitude for ceiling safety (independent of chosen alt_est source)
  float max_alt = NAN;
  if (lpos_fresh && !isnan(lpos_alt_filt_m)) {
    float a = lpos_alt_filt_m;
    if (a < -1.0f) a = -1.0f;
    if (a > 50.0f) a = 50.0f;
    max_alt = isnan(max_alt) ? a : fmaxf(max_alt, a);
  }
  if (range_fresh && !isnan(rangefinder_m)) {
    float rf = rangefinder_m;
    if (rf < 0.0f) rf = 0.0f;
    if (rf > 10.0f) rf = 10.0f;
    max_alt = isnan(max_alt) ? rf : fmaxf(max_alt, rf);
  }
  if (near_ground) {
    max_alt = isnan(max_alt) ? 0.0f : fmaxf(max_alt, 0.0f);
  }
  alt_max_m = max_alt;

  // Pick best altitude estimate for control logic (RF preferred if sane)
  float  new_alt = NAN;
  AltSrc new_src = ALT_SRC_NONE;

  if (range_fresh && !isnan(rangefinder_m)) {
    float rf = rangefinder_m;
    if (rf < 0.0f) rf = 0.0f;
    if (rf > 10.0f) rf = 10.0f;

    bool rf_ok = true;

    bool airborne_hint = false;
    if (have_ext && landed_state != MAV_LANDED_STATE_ON_GROUND) airborne_hint = true;
    if (lpos_fresh && !isnan(lpos_alt_filt_m) && lpos_alt_filt_m > 0.20f) airborne_hint = true;
    if (airborne_hint && rf < 0.05f) rf_ok = false;

    if (lpos_fresh && !isnan(lpos_alt_filt_m) && fabsf(rf - lpos_alt_filt_m) > 0.80f) rf_ok = false;

    if (rf_ok) {
      new_alt = rf;
      new_src = ALT_SRC_RANGEFINDER;
    }
  }

  if (new_src == ALT_SRC_NONE && lpos_fresh && !isnan(lpos_alt_filt_m)) {
    float a = lpos_alt_filt_m;
    if (a < -1.0f) a = -1.0f;
    if (a > 50.0f) a = 50.0f;
    new_alt = a;
    new_src = ALT_SRC_LPOS;
  }

  if (new_src == ALT_SRC_NONE && near_ground) {
    new_alt = 0.0f;
    new_src = ALT_SRC_ON_GROUND;
  }

  alt_est_m = new_alt;
  alt_src = new_src;

  // Ceiling hysteresis uses alt_max_m to avoid “stuck-low RF hides climb”
  if (!isnan(alt_max_m) && alt_max_m >= CEIL_M) ceiling_active = true;
  if (!isnan(alt_max_m) && alt_max_m <= (CEIL_M - 0.10f)) ceiling_active = false;
}

// ----------------------------- Logging ---------------------------------
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

  txt_log_fp = fopen(LOG_TXT_PATH, "a");
  if (!txt_log_fp) {
    fprintf(stderr, "WARN: cannot open %s: %s\n", LOG_TXT_PATH, strerror(errno));
  } else {
    // Write a session start marker
    fprintf(txt_log_fp, "\n--- SESSION START ---\n");
    fflush(txt_log_fp);
  }

  last_flush_ms = now_ms();
  last_log_ms   = 0;
}

static void log_flush_if_due(uint64_t t) {
  if (t - last_flush_ms < LOG_FLUSH_MS) return;
  last_flush_ms = t;
  if (log_fp) fflush(log_fp);
  if (scan_fp) fflush(scan_fp);
  if (txt_log_fp) fflush(txt_log_fp);
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
  }

  log_flush_if_due(t);
}

// ----------------------------- Behavior --------------------------------
static void enter_state(State ns, const char* reason) {
  if (st == ns) return;

  bool leaving_hover = (st == ST_HOVER && ns != ST_HOVER);
  bool entering_hover = (ns == ST_HOVER);

  if (leaving_hover) {
    hover_xy_locked = false;
    hover_xy_prelock_valid = false;
    hover_xy_lock_ms = 0;
    hover_lock_x_m = hover_lock_y_m = 0.0f;
    hover_prelock_x_m = hover_prelock_y_m = 0.0f;
    printf("HOVER: reset XY capture (leaving %s)\n", state_name(ns));
  }

  if (st == ST_LIFTOFF_ASSIST && ns != ST_LIFTOFF_ASSIST) {
    // ensure no stale overrides; attitude target ramp uses GUIDED and stops automatically.
  }

  if (ns == ST_TAKEOFF) {
    takeoff_sent = false;
    takeoff_sent_ms = 0;
    have_takeoff_ack = false;

    takeoff_started = false;
    takeoff_started_ms = 0;
    takeoff_att_ramp_active = false;
    takeoff_att_ramp_start_ms = 0;
    takeoff_alt0_m = alt_max_m;

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

  if (entering_hover) {
    hover_xy_locked = false;
    hover_xy_prelock_valid = false;
    hover_xy_lock_ms = 0;
    hover_lock_x_m = hover_lock_y_m = 0.0f;
    hover_prelock_x_m = hover_prelock_y_m = 0.0f;
    printf("HOVER: reset XY capture (enter)\n");
  }

  if (ns == ST_LANDING) {
    land_mode_sent = false;
    land_mode_sent_ms = 0;
    pending_kf_flags |= KF_LAND_START;
  }

  printf("STATE: %s -> %s reason=%s\n", state_name(st), state_name(ns), reason ? reason : "unknown");
  if (txt_log_fp) {
    fprintf(txt_log_fp, "[%.3f] STATE: %s -> %s reason=%s\n", now_ms()*0.001f, 
            state_name(st), state_name(ns), reason ? reason : "unknown");
  }

  // Dump buffer on failure/unexpected transitions
  bool is_fail = (ns == ST_DISARMING && st != ST_LANDING && st != ST_IDLE) ||
                 (ns == ST_LANDING && st != ST_LANDING) ||
                 (ns == ST_LIFTOFF_ASSIST);

  if (is_fail) {
      snap_dump();
  }

  st = ns;
}

// ----------------------------- Forward decls ---------------------------
static bool takeoff_off_ground(uint64_t t);

// ----------------------------- Liftoff assist --------------------------
// ArduPilot often ignores RC_OVERRIDE by default. Fall back to attitude+thrust ramp.
static void liftoff_assist_tick(uint64_t t) {
  // Ensure we're in GUIDED so SET_ATTITUDE_TARGET is accepted.
  if (t - assist_start_ms < 150) {
    set_mode_guided();
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

    // Ease-out ramp for quicker breakaway without a hard jump.
    float ue = sqrtf(u);
    float thr = (1.0f - ue) * (float)ASSIST_THR_US_MIN + ue * (float)ASSIST_THR_US_MAX;
    // Map PWM-ish range to normalized thrust 0..1 (rough). Assume 1000-2000µs.
    float thr_norm = (thr - 1000.0f) / 1000.0f;
    if (thr_norm < 0.0f) thr_norm = 0.0f;
    if (thr_norm > 1.0f) thr_norm = 1.0f;

    float yaw = have_att ? current_heading_deg() : 0.0f;
    send_attitude_target_thrust(thr_norm, yaw);
  }

  if (!assist_warned_override &&
      assist_baseline_set &&
      (t - assist_start_ms) > ASSIST_OVERRIDE_EFFECT_MS &&
      have_servo && (t - servo_last_ms) < 200) {

    float avg = servo_motor_avg();
    if (!isnan(assist_motor_avg0) && (avg - assist_motor_avg0) < ASSIST_MOTOR_DELTA_MIN) {
      assist_warned_override = true;
      printf("WARN: liftoff thrust ramp appears ineffective (motor avg delta %.1fus).\n", (avg - assist_motor_avg0));
    }
  }

  if (takeoff_off_ground(t)) {
    printf("LIFTOFF_ASSIST: off-ground (altMAX=%.2fm, landed=%u(%s)). Switching back to GUIDED+TAKEOFF.\n",
           alt_max_m, (unsigned)landed_state, landed_state_name(landed_state));
    set_mode_guided();
    guided_takeoff(TAKEOFF_TARGET_M);
    enter_state(ST_TAKEOFF, "Assist Success");
    return;
  }

  if ((t - assist_start_ms) > ASSIST_ABORT_MS) {
    printf("LIFTOFF_ASSIST: timeout (alt=%.2fm). Disarming for safety.\n", alt_max_m);
    enter_state(ST_DISARMING, "Assist Timeout");
    return;
  }
}

// Takeoff attitude-based thrust ramp (GUIDED) used before giving up to liftoff assist.
static void takeoff_att_ramp_tick(uint64_t t) {
  if (!takeoff_att_ramp_active) return;
  if (takeoff_att_ramp_start_ms == 0) takeoff_att_ramp_start_ms = t;

  // 25 Hz
  static uint64_t last_send = 0;
  if (t - last_send < 40) return;
  last_send = t;

  uint64_t dt = t - takeoff_att_ramp_start_ms;
  float u = (dt >= 700) ? 1.0f : (float)dt / 700.0f;
  if (u < 0.0f) u = 0.0f;

  // Allow a bit more ceiling for heavier craft and ramp faster.
  float thr = (1.0f - u) * 0.50f + u * 0.95f;
  float yaw = have_yaw_target ? yaw_target_deg : (have_att ? current_heading_deg() : 0.0f);
  send_attitude_target_thrust(thr, yaw);

  if (takeoff_off_ground(t) || dt > 1400) {
    takeoff_att_ramp_active = false;
  }
}

// ----------------------------- Battery failsafe ------------------------
static bool arm_allowed_by_battery(uint64_t t) {
  if (!batt_valid) return true; // Fail-open if invalid
  return batt_vpc >= BATT_ARM_MIN_VPC;
}

static void battery_failsafe_tick(uint64_t t) {
  if (!batt_valid) {
      // Invalid battery - fail open, no actions.
      return;
  }
  
  float vpc = batt_vpc;

  if (!fc_armed) {
    if (want_arm && vpc < BATT_ARM_MIN_VPC) {
      if (t - batt_last_warn_ms > 1200) {
        batt_last_warn_ms = t;
        printf("BATT NO-GO: Vpc=%.2f < %.2f. Refusing arm.\n", vpc, BATT_ARM_MIN_VPC);
      }
    }
    batt_low_since_ms = 0;
    batt_emerg_since_ms = 0;
    return;
  }

  // Critical / Emergency
  if (vpc < BATT_EMERG_VPC) {
    if (batt_emerg_since_ms == 0) batt_emerg_since_ms = t;
    if ((t - batt_emerg_since_ms) > BATT_LOW_HOLD_MS) {
      pending_kf_flags |= KF_BATT_EMERG;
      if (st != ST_LANDING && st != ST_DISARMING) {
        printf("BATT EMERG: Vpc=%.2f < %.2f sustained -> LANDING (suppressed).\n", vpc, BATT_EMERG_VPC);
        // enter_state(ST_LANDING, "BATT_EMERG");
      }
    }
  } else {
    batt_emerg_since_ms = 0;
  }

  // Low (Land)
  if (vpc < BATT_LAND_VPC) {
    if (batt_low_since_ms == 0) batt_low_since_ms = t;
    if ((t - batt_low_since_ms) > BATT_LOW_HOLD_MS) {
      pending_kf_flags |= KF_BATT_LAND;
      if (st != ST_LANDING && st != ST_DISARMING) {
        // Step 4: Disable companion-enforced LAND on low battery, just log
        printf("BATT LOW: Vpc=%.2f < %.2f sustained (LANDING suppressed).\n", vpc, BATT_LAND_VPC);
        // enter_state(ST_LANDING, "BATT_LOW");
      }
    }
  } else {
    batt_low_since_ms = 0;
  }
}

// ----------------------------- Takeoff helpers -------------------------
static bool takeoff_off_ground(uint64_t t) {
  bool rf_fresh = have_rangefinder && (t - rangefinder_last_update_ms) < 400;
  if (have_ext && landed_state != MAV_LANDED_STATE_ON_GROUND) return true;
  if (rf_fresh && !isnan(rangefinder_m) && rangefinder_m > 0.05f) return true;
  if (!isnan(alt_max_m) && alt_max_m > 0.05f) return true;
  return false;
}

static void snap_add(uint64_t t) {
  snapshot_t* s = &snap_ring[snap_head];
  
  s->t_ms = t;
  s->state = (uint8_t)st;
  s->mode = (uint32_t)hb_custom_mode;
  s->armed = fc_armed;
  s->landed = have_ext ? landed_state : 255; // 255=unknown
  
  if (have_att) {
    s->roll = rad2deg(roll_rad);
    s->pitch = rad2deg(pitch_rad);
    s->yaw = current_heading_deg();
  } else {
    s->roll = s->pitch = s->yaw = NAN;
  }
  
  if (have_xy) {
    s->x = lpos_x_m;
    s->y = lpos_y_m;
    s->vx = lpos_vx_mps;
    s->vy = lpos_vy_mps;
  } else {
    s->x = s->y = s->vx = s->vy = NAN;
  }
  s->z = have_lpos ? lpos_alt_m : NAN;
  s->vz = have_lpos ? lpos_vz_mps : NAN;
  
  s->alt_est = alt_est_m;
  s->alt_src = (uint8_t)alt_src;

  bool rf_fresh = have_rangefinder && (t - rangefinder_last_update_ms) < 400;
  s->rf_m = rf_fresh ? rangefinder_m : NAN;
  
  bool of_ok = of_fresh(t);
  s->of_q = of_ok ? of_quality : 0;
  // If we have flow_rate, use it, else comp_m if available? Usually rates.
  s->of_rx = of_ok ? of_rate_x : NAN;
  s->of_ry = of_ok ? of_rate_y : NAN;
  
  s->xy_ok = xy_ctrl_ok(t);
  s->z_ok = z_ctrl_ok(t);
  
  if (sys_fresh(t)) {
    s->gyr_ok = sys_health_bit(MAV_SYS_STATUS_SENSOR_3D_GYRO, t);
    bool mot_enabled = (sys_enabled & MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS) != 0;
    bool mot_healthy = sys_health_bit(MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS, t);
    s->mot_ok = mot_enabled ? mot_healthy : true;
  } else {
    s->gyr_ok = s->mot_ok = false;
  }

  // Battery telemetry raw debug
  s->batt_v = batt_v_total;
  s->batt_c = batt_cells;
  s->batt_vpc = batt_vpc;

  if (have_servo && (t - servo_last_ms) < 300) {
    for(int i=0; i<4; i++) s->mot[i] = servo_raw[i];
  } else {
    for(int i=0; i<4; i++) s->mot[i] = 0;
  }

  if (have_rcin && (t - rcin_last_ms) < 1000) {
    for(int i=0; i<4; i++) s->rc[i] = rcin[i];
    s->rssi = rcin_rssi;
  } else {
    for(int i=0; i<4; i++) s->rc[i] = 0;
    s->rssi = 0;
  }

  snap_head = (snap_head + 1) % SNAP_RING_SZ;
}

static void print_snap_line(FILE* f, const snapshot_t* s) {
  if (!f) return;
  
  // Tms, state, mode, arm, land
  fprintf(f, "%llu %-7s M%-2u %c%c ", 
    (unsigned long long)s->t_ms, 
    state_name((State)s->state), 
    (unsigned)s->mode, 
    s->armed ? 'A':'d', 
    (s->landed==MAV_LANDED_STATE_ON_GROUND)?'G':
    (s->landed==MAV_LANDED_STATE_IN_AIR)?'A':
    (s->landed==MAV_LANDED_STATE_TAKEOFF)?'T':
    (s->landed==MAV_LANDED_STATE_LANDING)?'L':'?'
  );

  // Att
  fprintf(f, "R%.1f P%.1f Y%.1f ", s->roll, s->pitch, s->yaw);
  
  // Pos/Vel
  fprintf(f, "X%.1f Y%.1f Z%.1f Vz%.2f ", s->x, s->y, s->z, s->vz);
  
  // Alt
  fprintf(f, "Alt%.2f(%s) RF%.2f ", s->alt_est, alt_src_name((AltSrc)s->alt_src), s->rf_m);
  
  // EKF/Sys
  fprintf(f, "[%c%c%c%c] ", 
    s->xy_ok?'X':'.', s->z_ok?'Z':'.', s->gyr_ok?'G':'.', s->mot_ok?'M':'.');
    
  // OF
  fprintf(f, "OF(Q%u %.2f %.2f) ", s->of_q, s->of_rx, s->of_ry);
  
  // Batt (Debug)
  fprintf(f, "B(%.2fV %dc %.2f) ", s->batt_v, s->batt_c, s->batt_vpc);
  
  // Mot
  fprintf(f, "M(%u %u %u %u) ", s->mot[0], s->mot[1], s->mot[2], s->mot[3]);

  // RC
  if (s->rc[0] > 0) {
    fprintf(f, "RC(%u %u %u %u SI%u) ", s->rc[0], s->rc[1], s->rc[2], s->rc[3], s->rssi);
  }
  
  fprintf(f, "\n");
}

static void log_snapshot_tick(uint64_t t) {
  snap_add(t);
  
  // Current snapshot is at (head-1)
  int idx = (snap_head + SNAP_RING_SZ - 1) % SNAP_RING_SZ;
  const snapshot_t* s = &snap_ring[idx];
  
  print_snap_line(stdout, s);
  if (txt_log_fp) {
    print_snap_line(txt_log_fp, s);
  }
}

static void snap_dump(void) {
  printf("\n--- PRE-FAIL DUMP (%d lines) ---\n", SNAP_RING_SZ);
  if (txt_log_fp) fprintf(txt_log_fp, "\n--- PRE-FAIL DUMP ---\n");
  
  // Print from oldest to newest
  int idx = snap_head; // oldest
  for (int i=0; i<SNAP_RING_SZ; i++) {
    const snapshot_t* s = &snap_ring[idx];
    if (s->t_ms > 0) { // skip empty slots
       print_snap_line(stdout, s);
       if (txt_log_fp) print_snap_line(txt_log_fp, s);
    }
    idx = (idx + 1) % SNAP_RING_SZ;
  }
  printf("--- END PRE-FAIL DUMP ---\n\n");
  if (txt_log_fp) fprintf(txt_log_fp, "--- END ---\n");
  fflush(stdout);
  if (txt_log_fp) fflush(txt_log_fp);
}

// ----------------------------- Main control loop -----------------------
static void control_tick(void) {
  uint64_t t = now_ms();

  send_own_heartbeat_tick(t);

  update_alt_estimate();
  tof_filter_tick();
  log_tick(t);

  battery_failsafe_tick(t);

  static uint64_t t_last_snap = 0;
  if (t - t_last_snap >= 100) { // 10 Hz
    t_last_snap = t;
    log_snapshot_tick(t);
    
    // Clear ACK after printing
    if (have_ack) have_ack = false;
  }

  if (!have_fc) {
    if (st != ST_WAIT_LINK) enter_state(ST_WAIT_LINK, "No FC Link");
    return;
  }

  if (!rcmap_known && (t - rcmap_last_request_ms) > 2000) {
    request_rcmap_params();
  }

  if (hard_nogo(t)) {
    printf("NO-GO: SYS_STATUS indicates gyro or motor outputs unhealthy.\n");
    if (fc_armed) enter_state(ST_DISARMING, "Health Fail");
    else enter_state(ST_IDLE, "Health Fail");
    return;
  }

  // Unexpected disarm
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

    enter_state(ST_IDLE, "Unexpected Disarm");
  }
  fc_armed_prev = fc_armed;

  // want_arm dropped while armed -> FORCE DISARM immediately (any state/mode)
  if (!want_arm && fc_armed) {
    last_disarm_cmd_ms = 0;
    disarm_fc_force();
    enter_state(ST_DISARMING, "User Abort");
    return;
  }

  // Ceiling safety: always clamp down. Prefer full poshold if XY locked; otherwise Z-only.
  if (ceiling_active && fc_armed) {
    if (!have_yaw_target && have_att) {
      have_yaw_target = true;
      yaw_target_deg = current_heading_deg();
    }
    float yaw = have_yaw_target ? yaw_target_deg : (have_att ? current_heading_deg() : 0.0f);
    float safe_z = hover_target_z_down();

    if (hover_xy_locked && have_att) {
      send_pos_yaw_ned(hover_lock_x_m, hover_lock_y_m, safe_z, yaw);
    } else {
      // Stay Z-only while XY not locked to avoid fighting bad estimates.
      send_z_yaw_ned(safe_z, yaw);
    }
    return;
  }

  // Hover failsafe with hysteresis: require sustained stale inputs before LAND.
  if (fc_armed && (st == ST_HOVER)) {
    bool lpos_ok = have_lpos && (t - lpos_last_update_ms) < 400;
    bool alt_ok  = !isnan(alt_max_m);
    bool rf_ok   = have_rangefinder && (t - rangefinder_last_update_ms) < 400 && !isnan(rangefinder_m);

    lpos_stale_count = lpos_ok ? 0 : (lpos_stale_count + 1);
    alt_stale_count  = alt_ok  ? 0 : (alt_stale_count + 1);
    rf_stale_count   = rf_ok   ? 0 : (rf_stale_count + 1);

    bool lpos_fail = lpos_stale_count > STALE_FAIL_TICKS;
    bool alt_fail  = alt_stale_count  > STALE_FAIL_TICKS;
    bool rf_fail   = REQUIRE_RANGEFINDER_FOR_HOVER && (rf_stale_count > STALE_FAIL_TICKS);

    if (lpos_fail || alt_fail || rf_fail) {
      printf("FAILSAFE: pose/alt stale (lpos=%u rf=%u alt=%u) -> LANDING\n",
             lpos_stale_count, rf_stale_count, alt_stale_count);
      enter_state(ST_LANDING, "Failsafe (Sensors)");
    }
  } else {
    lpos_stale_count = rf_stale_count = alt_stale_count = 0;
  }

  switch (st) {
    case ST_WAIT_LINK:
      enter_state(ST_IDLE, "Link OK");
      break;

    case ST_IDLE:
      if (want_arm && !arm_allowed_by_battery(t)) break;

      if (want_arm && !fc_armed) {
        if (!hover_ready_stable(t)) {
          set_mode_guided();
          break;
        }
        if (!have_yaw_target && have_att) {
          have_yaw_target = true;
          yaw_target_deg = current_heading_deg();
        }
        init_hover_targets_on_ground(t);
        enter_state(ST_ARMING, "Want Arm");
      } else if (!want_arm && fc_armed) {
        enter_state(ST_DISARMING, "Already Armed");
      } else if (want_arm && fc_armed) {
        enter_state(ST_TAKEOFF, "Resume Takeoff");
      }
      break;

    case ST_ARMING:
      if (!arm_allowed_by_battery(t)) {
        enter_state(ST_IDLE, "Batt Fail");
        break;
      }

      if (!hover_ready_stable(t)) {
        set_mode_guided();
        break;
      }

      init_hover_targets_on_ground(t);

      if (!fc_armed) {
        set_mode_guided();
        arm_fc();
      } else {
        enter_state(ST_TAKEOFF, "Armed");
      }
      break;

    case ST_TAKEOFF: {
      if (hb_custom_mode != 4) set_mode_guided();

      // One-shot XY capture (if not already captured)
      if (!hover_xy_prelock_valid &&
          have_lpos && (t - lpos_last_update_ms) < 400 &&
          isfinite(lpos_x_m) && isfinite(lpos_y_m) &&
          !isnan(alt_max_m) && alt_max_m > HOVER_CAPTURE_MIN_ALT_M) {
        hover_prelock_x_m = lpos_x_m;
        hover_prelock_y_m = lpos_y_m;
        hover_xy_prelock_valid = true;
        printf("HOVER: prelock XY (takeoff) x=%.2f y=%.2f\n", hover_prelock_x_m, hover_prelock_y_m);
      }

      if (!takeoff_sent) {
        guided_takeoff(TAKEOFF_TARGET_M); // single takeoff command, keep streaming position targets
        takeoff_sent = true;
        takeoff_sent_ms = t;
        if (isnan(takeoff_alt0_m)) {
          takeoff_alt0_m = !isnan(alt_max_m) ? alt_max_m : alt_est_m;
        }
      }

      bool servo_fresh = have_servo && (t - servo_last_ms) < 250;
      float mot_avg = servo_fresh ? servo_motor_avg() : NAN;
      bool mot_started = servo_fresh && (mot_avg > TAKEOFF_MOT_START_US);
      bool off_ground = takeoff_off_ground(t);
      bool alt_rising = (!isnan(takeoff_alt0_m) && !isnan(alt_max_m) && (alt_max_m - takeoff_alt0_m) > 0.05f);

      // Delay attitude thrust ramp until after NAV_TAKEOFF has time to spool and only if no rise yet.
      if (!takeoff_started && !takeoff_att_ramp_active && takeoff_sent &&
          (t - takeoff_sent_ms) > TAKEOFF_RAMP_DELAY_MS &&
          !mot_started && !alt_rising && !off_ground) {
        takeoff_att_ramp_active = true;
        takeoff_att_ramp_start_ms = t;
      }

      // Stream setpoints only after the no-vel window and not while ramping.
      bool allow_stream = takeoff_sent && (t - takeoff_sent_ms) >= TAKEOFF_NO_VEL_MS && !takeoff_att_ramp_active;
      if (allow_stream) {
        float yaw = have_yaw_target ? yaw_target_deg :
                    (have_att ? current_heading_deg() : 0.0f);
        float z_down = hover_target_z_down();

        if (hover_xy_locked) {
          send_pos_yaw_ned(hover_lock_x_m, hover_lock_y_m, z_down, yaw);
        } else {
          send_z_yaw_ned(z_down, yaw);
        }
      }

      // Run the attitude thrust ramp while waiting for lift
      takeoff_att_ramp_tick(t);
      if (!takeoff_started && !takeoff_att_ramp_active) {
        // If we're already airborne (or motors clearly spooled) but missed the trigger, mark started.
        bool inferred_air = (have_ext && landed_state != MAV_LANDED_STATE_ON_GROUND) ||
                            (!isnan(alt_max_m) && alt_max_m > 0.05f) ||
                            (servo_fresh && mot_avg > (TAKEOFF_MOT_START_US + 150));
        if (inferred_air) {
          takeoff_started = true;
          takeoff_started_ms = t;
          if (have_att) {
            have_yaw_target = true;
            yaw_target_deg = current_heading_deg();
          }
          printf("TAKEOFF: ramp ended, inferring liftoff (altMAX=%.2f mot_avg=%.1f)\n",
                 alt_max_m, mot_avg);
        } else {
          // Ramp finished without lift -> hand off to liftoff assist
          printf("TAKEOFF thrust ramp finished with no lift -> LIFTOFF_ASSIST\n");
          enter_state(ST_LIFTOFF_ASSIST, "Takeoff Ramp Fail");
          break;
        }
      }

      if (!takeoff_started && (mot_started || off_ground)) {
        takeoff_started = true;
        takeoff_started_ms = t;
        if (have_att) {
          have_yaw_target = true;
          yaw_target_deg = current_heading_deg();
        }
        printf("TAKEOFF: started (mot_avg=%.1f, altMAX=%.2f, landed=%u(%s))\n",
               servo_fresh ? mot_avg : -1.0f,
               alt_max_m,
               (unsigned)landed_state, landed_state_name(landed_state));
      }

      if (!takeoff_started && (t - takeoff_sent_ms) > TAKEOFF_TIMEOUT_MS) {
        printf("TAKEOFF stalled (altMAX=%.2f, mot_avg=%.1f) -> LIFTOFF_ASSIST\n",
               alt_max_m, (have_servo ? servo_motor_avg() : -1.0f));
        enter_state(ST_LIFTOFF_ASSIST, "Takeoff Stalled");
        break;
      }

      if (!isnan(alt_max_m) && alt_max_m >= (TAKEOFF_TARGET_M - 0.05f)) {
        if (!have_yaw_target && have_att) {
          have_yaw_target = true;
          yaw_target_deg = current_heading_deg();
        }
        enter_state(ST_HOVER, "Target Height Reached");
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

      // Continuous hover clamp: Z always; XY once locked.
      hover_hold_tick(t);
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

      // WORLD vertical descent
      send_vel_frame(0,0,+0.15f, 0, MAV_FRAME_LOCAL_NED);

      bool near_ground = (!isnan(alt_max_m) && alt_max_m < 0.10f);
      if (near_ground || (have_ext && landed_state == MAV_LANDED_STATE_ON_GROUND)) {
        enter_state(ST_DISARMING, "Landed");
      }
    } break;

    case ST_DISARMING: {
      if (fc_armed) {
        if (disarm_start_ms == 0) disarm_start_ms = t;
        disarm_fc_force();
      } else {
        disarm_start_ms = 0;
        enter_state(ST_IDLE, "Disarmed");
      }
    } break;

    default:
      enter_state(ST_IDLE, "Default");
      break;
  }

  if (csv_fp) {
      float alt = isnan(lpos_alt_m) ? 0 : lpos_alt_m;
      fprintf(csv_fp, "%lu,%s,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%.2f,%.2f,%.2f,%ld,%ld,%ld,%ld\n",
          t, state_name(st), alt,
          roll_rad * 57.2958f, pitch_rad * 57.2958f, yaw_rad * 57.2958f,
          motor_pwm[0], motor_pwm[1], motor_pwm[2], motor_pwm[3],
          vib_x, vib_y, vib_z,
          (long)esc_rpm[0], (long)esc_rpm[1], (long)esc_rpm[2], (long)esc_rpm[3]);
      
      static int flush_ctr = 0;
      if (++flush_ctr >= 50) {
          fflush(csv_fp);
          flush_ctr = 0;
      }
  }
}

// ----------------------------- main ------------------------------------
int main(int argc, char** argv) {
  setvbuf(stdout, NULL, _IOLBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);

  log_init();

  csv_fp = fopen("flight_data.csv", "w");
  if (csv_fp) {
    fprintf(csv_fp, "Time_ms,State,Alt,Roll,Pitch,Yaw,Mot1,Mot2,Mot3,Mot4,VibX,VibY,VibZ,RPM1,RPM2,RPM3,RPM4\n");
    fflush(csv_fp);
  } else {
    fprintf(stderr, "Failed to open flight_data.csv\n");
  }

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

  printf("MODE=HOVER_ONLY\n");
  printf("LOG: stdout + %s (human-readable), CSV=%s, SCAN=%s\n",
         LOG_TXT_PATH, LOG_CSV_PATH, LOG_SCAN_PATH);

  fc_fd = open_uart(FC_UART, FC_BAUD);
  if (fc_fd < 0) return 1;
  printf("Opened FC UART: %s @%d\n", FC_UART, FC_BAUD);

  tof_fd = open_uart(TOF_UART, TOF_BAUD);
  if (tof_fd < 0) return 1;
  printf("Opened ToF UART: %s @%d\n", TOF_UART, TOF_BAUD);

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
