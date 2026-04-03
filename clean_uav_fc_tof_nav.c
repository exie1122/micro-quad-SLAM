//2026-04-02
//first iteration of code that enables stable autonomous hover. 

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
#include <signal.h>

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

#ifndef MAVLINK_MSG_ID_EKF_STATUS_REPORT
#define MAVLINK_MSG_ID_EKF_STATUS_REPORT 193
#endif

// MAV_SENSOR_ORIENTATION enum uses ROTATION_*_FACING values.
// Downward-facing is commonly 25.
#define ORIENT_DOWNWARD_FACING 25

// ----------------------------- Logging paths ---------------------------
#define LOG_CSV_PATH          "/mnt/sdcard/navlog.csv"
#define LOG_SCAN_PATH         "/mnt/sdcard/scanlog.bin"
#define LOG_TXT_PATH          "/mnt/sdcard/log.txt"
#define LOG_HZ                20
#define LOG_FLUSH_MS          1000
#define SCAN_PENDING_MAX_MS   250
#define POSE_RING_SZ          32
#define HEARTBEAT_TIMEOUT_MS  3000

static FILE* log_fp  = NULL;
static FILE* scan_fp = NULL;
static FILE* txt_log_fp = NULL;
static uint64_t last_log_ms    = 0;
static uint64_t last_flush_ms  = 0;
static uint64_t session_start_ms = 0;

static volatile sig_atomic_t g_shutdown = 0;

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
static const float HOVER_TARGET_M = 0.55f;          // desired hover height (meters above ground)
static const float TAKEOFF_TARGET_M = 0.50f;        // first stop when climbing
static const float CEIL_M = 0.95f;                  // hard max altitude (meters)
static const float CEIL_MARGIN_M = 0.20f;            // start ceiling protection well before the hard ceiling
static const float HOVER_CAPTURE_MIN_ALT_M = 0.15f;  // don't lock XY until clearly airborne
static const uint64_t PREARM_STABLE_MS = 400;       // require sensors/control stable before arming
static const bool REQUIRE_RANGEFINDER_FOR_HOVER = true;
static const bool REQUIRE_OPTICAL_FLOW_FOR_HOVER = true;
static const uint64_t SENSOR_FRESH_MS = 400;
static const uint64_t BOOTSTRAP_GRACE_MS = 2000;
static const uint64_t BOOTSTRAP_FAILSAFE_MS = 1800;
static const uint64_t FULL_HOVER_FAILSAFE_MS = 900;
static const uint64_t XY_LOCK_DWELL_MS = 500;
static const float VX_LOCK_MAX_MPS = 0.30f;
static const float VY_LOCK_MAX_MPS = 0.20f;
static const uint64_t PRELOCK_TIMEOUT_MS = 4000;    // land if hover never settles enough to achieve XY lock
static const float PRELOCK_MAX_DRIFT_M = 1.00f;     // abort if prelock drift carries us too far from hover entry

// EKF variance gates for XY lock
static const float EKF_POS_VAR_LOCK_MAX = 0.30f;    // max pos_horiz_variance to acquire XY lock
static const float EKF_POS_VAR_BREAK    = 0.50f;    // break XY lock if variance exceeds this
static const float POSTLOCK_MAX_DRIFT_M = 0.30f;    // break XY lock if position drifts this far from anchor
static const float TAKEOFF_XY_SANITY_GROUND_MAX_M = 0.10f; // require RF to indicate near-ground before TAKEOFF
static const float TAKEOFF_XY_SANITY_MAX_MPS = 0.15f;      // max allowed on-ground LPOS speed before TAKEOFF
static const uint64_t TAKEOFF_XY_SANITY_DWELL_MS = 300;    // require a short stable dwell before TAKEOFF

// ToF constraints
static const float TOF_MAX_RANGE_M = 4.00f;

// Yaw drift guardian: if the actual heading drifts more than this from the
// commanded target (e.g. due to magnetometer interference near walls),
// accept the new heading rather than letting ArduPilot fight to rotate back.
static const float YAW_DRIFT_ACCEPT_DEG = 20.0f;
// If the yaw drift guardian fires this many times within the window, the
// heading is spinning uncontrollably — land instead of chasing it.
static const int      YAW_DRIFT_MAX_ACCEPTS  = 4;
static const uint64_t YAW_DRIFT_WINDOW_MS    = 2000;
static const uint64_t XY_RELOCK_COOLDOWN_MS  = 400;
static const float    YAW_RELOCK_ERR_MAX_DEG = 10.0f;
static const uint64_t YAW_DRIFT_MIN_STRIKE_GAP_MS = 200;
static const uint64_t YAW_CEIL_SUPPRESS_MS   = 500;

// Provisional yaw target convergence: before first XY lock, allow the
// provisional yaw target to track a new stable heading.
static const float    PROV_YAW_STABLE_DEG   = 5.0f;   // heading must stay within this of candidate
static const uint64_t PROV_YAW_STABLE_MS    = 400;    // stable for this long to accept

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

static const uint64_t TAKEOFF_RAMP_DELAY_MS    = 700;     // start ramp a bit sooner
static const float    TAKEOFF_MOT_START_US      = 1150.0f;
static const uint64_t TAKEOFF_TIMEOUT_MS        = 8000;
static const uint64_t TAKEOFF_HANDOFF_OFFGROUND_MS = 400;
static const float    TAKEOFF_HANDOFF_MIN_ALT_M = 0.35f;   // stay in takeoff until we are clearly above ground effect
static const float    TAKEOFF_HANDOFF_MAX_CLIMB_MPS = 0.15f; // wait for climb rate to settle before hover handoff
static const float    TAKEOFF_HANDOFF_MAX_XY_MPS = 0.20f;  // do not enter hover while still sliding laterally
static const uint64_t TAKEOFF_GROUNDED_ABORT_MS = 1000;    // abort if we fall back near the ground after liftoff
static const float    TAKEOFF_HANDOFF_MAX_TILT_DEG = 20.0f;

// Takeoff brake sub-phase: after target height reached, hold Z+yaw and wait
// for velocity/altitude to settle before entering HOVER.
static const uint64_t TAKEOFF_BRAKE_DWELL_MS     = 300;    // require settled for this long
static const float    TAKEOFF_BRAKE_VXY_MAX_MPS  = 0.25f;  // max lateral velocity during brake
static const float    TAKEOFF_BRAKE_VZ_MAX_MPS   = 0.35f;  // max vertical velocity (ceiling descent sends 0.15)
static const float    TAKEOFF_BRAKE_ALT_ERR_MAX_M= 0.40f;  // max altitude error from target (allows overshoot settling)
static const uint8_t  TAKEOFF_BRAKE_MIN_FLOW_Q   = 30;     // min optical flow quality (matches vel_xy_motion_ok)
static const float    TAKEOFF_BRAKE_YAW_RATE_MAX = 15.0f;  // max yaw rate (deg/s) to allow brake settle
static const uint64_t TAKEOFF_BRAKE_TIMEOUT_MS   = 5000;   // abort to LANDING if brake never settles

static const float    CEILING_DESCENT_BASE_MPS  = 0.10f;   // min descent speed at ceiling (reduced to limit momentum)
static const float    CEILING_DESCENT_MAX_MPS   = 1.00f;   // max descent command at ceiling
static const float    CEILING_DESCENT_NOLOCK_MAX_MPS = 0.15f; // cap descent when no XY lock (prevents momentum buildup)
static const float    CEILING_DESCENT_FLOOR_MPS = 0.02f;   // absolute min near taper bottom (prevents momentum buildup)
static const float    CEILING_DESCENT_GAIN      = 1.2f;    // overshoot-proportional descent gain (was 2.0)
static const float    CEIL_APPROACH_ZONE_M      = 0.30f;   // start climb-rate limiting this far below ceiling trigger
static const float    CEIL_TAPER_DEPTH_M        = 0.15f;   // descent taper zone depth below trigger (was 0.10)
static const float    RF_MAX_SLEW_MPS           = 2.0f;    // max RF change rate when airborne (m/s)
static const float    ALT_MAX_DECAY_ALPHA       = 0.10f;   // peak-hold decay for alt_max (~1s tau at 10Hz)
static const float    CEILING_RELEASE_VZ_MAX    = 0.10f;   // max descent Vz to allow ceiling release (tighter for smooth handoff)

static bool     takeoff_started                 = false;
static uint64_t takeoff_started_ms              = 0;
static uint64_t takeoff_off_ground_since_ms     = 0;
static bool     takeoff_was_airborne            = false;
static uint64_t takeoff_grounded_since_ms       = 0;
static float    takeoff_alt0_m                  = NAN;    // altitude snapshot when NAV_TAKEOFF sent
static bool     takeoff_att_ramp_was_active     = false;
static bool     takeoff_brake_active            = false;
static uint64_t takeoff_brake_start_ms          = 0;
static uint64_t takeoff_brake_settled_since_ms  = 0;
static float    takeoff_brake_prev_yaw_deg      = NAN;
static uint64_t takeoff_brake_prev_yaw_ms       = 0;

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
static bool     have_att = false;
static float    roll_rad = 0.0f;
static float    pitch_rad = 0.0f;
static float    yaw_rad = 0.0f;
static uint64_t att_last_update_ms = 0;

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

// EKF_STATUS_REPORT variance
static bool     have_ekf = false;
static float    ekf_vel_var        = NAN;
static float    ekf_pos_horiz_var  = NAN;
static float    ekf_pos_vert_var   = NAN;
static uint64_t ekf_last_ms = 0;

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
static bool takeoff_off_ground(uint64_t t);

// ----------------------------- Pose ring (mapping) --------------------
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
static int           pose_ring_head  = 0;
static int           pose_ring_count = 0;

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

typedef enum {
  HPH_GROUND = 0,
  HPH_ARM_SPOOL,
  HPH_TAKEOFF_Z_ONLY,
  HPH_PRELOCK_HOVER,
  HPH_XY_LOCK,
  HPH_FULL_HOVER,
  HPH_LANDING
} HoverPhase;

static const char* hover_phase_name(HoverPhase p) {
  switch (p) {
    case HPH_GROUND: return "GROUND";
    case HPH_ARM_SPOOL: return "ARM_SPOOL";
    case HPH_TAKEOFF_Z_ONLY: return "TAKEOFF_Z_ONLY";
    case HPH_PRELOCK_HOVER: return "PRELOCK_HOVER";
    case HPH_XY_LOCK: return "XY_LOCK";
    case HPH_FULL_HOVER: return "FULL_HOVER";
    case HPH_LANDING: return "LANDING";
    default: return "?";
  }
}

// Yaw target lock
static bool  have_yaw_target = false;
static float yaw_target_deg = 0.0f;

// We separate “Z/YAW hold is active” from “XY lock is valid”.
// - We always clamp Z (Z-only setpoint) once we start controlling.
// - We lock XY only after fresh LPOS velocity remains below the lock thresholds for the full dwell.
static float hover_hold_yaw_deg = NAN;
static uint64_t yaw_drift_window_start_ms = 0;
static int      yaw_drift_accept_count    = 0;
static bool     hover_yaw_latched = false;
static uint64_t xy_relock_block_until_ms = 0;
static uint64_t yaw_drift_last_strike_ms = 0;
static float    yaw_drift_last_strike_err = 0.0f;
static uint64_t yaw_spin_suppress_until_ms = 0;
// Provisional yaw target convergence state
static float    prov_yaw_candidate_deg = NAN;
static uint64_t prov_yaw_candidate_ms  = 0;
static bool  hover_xy_locked = false;      // true only after XY lock achieved
static uint64_t hover_xy_lock_ms = 0;
static float hover_lock_x_m = 0.0f;
static float hover_lock_y_m = 0.0f;
static bool     xy_lock_candidate_active = false;
static uint64_t xy_lock_candidate_since_ms = 0;
static HoverPhase hover_phase = HPH_GROUND;
static uint64_t hover_enter_ms = 0;                // when we entered ST_HOVER
static uint64_t bootstrap_grace_until_ms = 0;
static bool     bootstrap_grace_was_active = false;
static uint64_t hover_health_fail_since_ms = 0;
static uint64_t hover_ready_fail_log_ms = 0;
static char     hover_ready_fail_last_reason[160] = {0};

// Ceiling logic
static bool ceiling_active = false;
static float ceiling_alt_now_m = NAN;   // instantaneous max altitude across current sensors
static float alt_est_m = NAN;       // selected altitude estimate
static float rf_slew_m = NAN;          // slew-rate-limited rangefinder for alt estimation
static uint64_t rf_slew_last_ms = 0;   // timestamp for RF slew limiter
static float alt_max_m = NAN;       // MAX of available sources for ceiling safety
static bool alt_rf_rejected = false;

typedef enum {
  ALT_SRC_NONE = 0,
  ALT_SRC_LPOS,
  ALT_SRC_RANGEFINDER,
  ALT_SRC_ON_GROUND
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

static AltSrc         alt_src              = ALT_SRC_NONE;
static RangefinderSrc rf_src               = RF_SRC_NONE;
static OpticalFlowSrc of_src               = OF_SRC_NONE;
static bool           mapping_enabled      = true;
static bool           allow_disarmed_logging = false;

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

static float hover_yaw_setpoint_deg(void) {
  if (have_yaw_target) return yaw_target_deg;
  if (!isnan(hover_hold_yaw_deg)) return hover_hold_yaw_deg;
  if (have_att) return current_heading_deg();
  return 0.0f;
}

static float ceiling_control_alt_m(void) {
  if (!isnan(ceiling_alt_now_m)) return ceiling_alt_now_m;
  if (!isnan(alt_est_m)) return alt_est_m;
  return alt_max_m;
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

static void append_reason(char* buf, size_t buf_sz, const char* token) {
  if (!buf || buf_sz == 0 || !token || !token[0]) return;

  size_t len = strlen(buf);
  if (len >= buf_sz - 1) return;

  if (len > 0) {
    snprintf(buf + len, buf_sz - len, ",%s", token);
  } else {
    snprintf(buf, buf_sz, "%s", token);
  }
}

static bool flight_log_state(State s) {
  return s == ST_ARMING ||
         s == ST_TAKEOFF ||
         s == ST_LIFTOFF_ASSIST ||
         s == ST_HOVER ||
         s == ST_LANDING ||
         s == ST_DISARMING;
}

static bool flight_log_active(void) {
  return want_arm || fc_armed || flight_log_state(st);
}

static bool lpos_fresh(uint64_t t) {
  return have_lpos && (t - lpos_last_update_ms) < SENSOR_FRESH_MS;
}

static bool rf_fresh(uint64_t t) {
  return have_rangefinder && (t - rangefinder_last_update_ms) < SENSOR_FRESH_MS;
}

static bool bootstrap_grace_active(uint64_t t) {
  return bootstrap_grace_until_ms != 0 && t < bootstrap_grace_until_ms;
}

static HoverPhase current_hover_phase(void) {
  if (!fc_armed) return HPH_GROUND;

  if (st == ST_ARMING) return HPH_ARM_SPOOL;
  if (st == ST_TAKEOFF || st == ST_LIFTOFF_ASSIST) return HPH_TAKEOFF_Z_ONLY;
  if (st == ST_HOVER && hover_xy_locked) return HPH_FULL_HOVER;
  if (st == ST_HOVER && xy_lock_candidate_active) return HPH_XY_LOCK;
  if (st == ST_HOVER) return HPH_PRELOCK_HOVER;
  if (st == ST_LANDING) return HPH_LANDING;
  return HPH_GROUND;
}

static bool phase_uses_z_only(HoverPhase phase) {
  return phase == HPH_TAKEOFF_Z_ONLY || phase == HPH_PRELOCK_HOVER || phase == HPH_XY_LOCK;
}

static float hover_target_alt_m_for_phase(HoverPhase phase) {
  float target_m = (phase == HPH_FULL_HOVER) ? HOVER_TARGET_M : TAKEOFF_TARGET_M;
  float max_up = CEIL_M - CEIL_MARGIN_M;
  if (max_up < 0.10f) max_up = 0.10f;
  if (target_m > max_up) target_m = max_up;
  return target_m;
}

static float hover_target_z_down_for_phase(HoverPhase phase) {
  return -hover_target_alt_m_for_phase(phase);
}

static float hover_target_z_down(void) {
  return hover_target_z_down_for_phase(current_hover_phase());
}

static float logged_z_target_down(void) {
  if (!fc_armed || ceiling_active) return NAN;
  if (st == ST_HOVER) return hover_target_z_down();
  return NAN;
}

static float logged_cmd_vz_ned(void) {
  if (!fc_armed) return NAN;
  if (ceiling_active) return CEILING_DESCENT_BASE_MPS;
  if (st == ST_LANDING) return 0.15f;
  return NAN;
}

static void log_bootstrap_event(uint64_t t, const char* event, const char* reason) {
  HoverPhase phase = current_hover_phase();
  bool lpos_ok = lpos_fresh(t);
  bool rf_ok = rf_fresh(t);
  bool of_ok = have_of && (t - of_last_update_ms) < SENSOR_FRESH_MS;
  float vx = have_xy ? lpos_vx_mps : NAN;
  float vy = have_xy ? lpos_vy_mps : NAN;

  printf("EVENT: %s phase=%s reason=%s vx=%.2f vy=%.2f flow_q=%u rf_fresh=%d lpos_fresh=%d z_tgt=%.2f grace=%d\n",
         event ? event : "UNKNOWN",
         hover_phase_name(phase),
         (reason && reason[0]) ? reason : "-",
         vx, vy,
         of_ok ? (unsigned)of_quality : 0u,
         rf_ok ? 1 : 0,
         lpos_ok ? 1 : 0,
         hover_target_z_down_for_phase(phase),
         bootstrap_grace_active(t) ? 1 : 0);
}

static void sync_hover_phase(uint64_t t) {
  HoverPhase phase = current_hover_phase();
  if (phase == hover_phase) return;

  hover_phase = phase;
  log_bootstrap_event(t, "PHASE", hover_phase_name(phase));
  if (phase_uses_z_only(phase)) {
    log_bootstrap_event(t, "Z_ONLY_ACTIVE", "bootstrap_z_only");
  }
}

static void start_bootstrap_grace(uint64_t t, const char* reason) {
  uint64_t new_until = t + BOOTSTRAP_GRACE_MS;
  bool was_active = bootstrap_grace_active(t);
  if (new_until > bootstrap_grace_until_ms) bootstrap_grace_until_ms = new_until;

  if (!was_active) {
    bootstrap_grace_was_active = true;
    log_bootstrap_event(t, "GRACE_START", reason);
  }
}

static void poll_bootstrap_grace(uint64_t t) {
  bool active = bootstrap_grace_active(t);
  if (!active && bootstrap_grace_was_active) {
    bootstrap_grace_was_active = false;
    log_bootstrap_event(t, "GRACE_END", "bootstrap_window_elapsed");
  }
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

// Ceiling hold: XY position hold (if locked) + Z velocity + yaw hold.
// Used during ceiling descent so the drone doesn't drift in XY.
static void send_ceiling_hold(float vz_down) {
  if (!have_fc) return;

  float yaw = hover_yaw_setpoint_deg();

  static uint64_t last_log = 0;
  uint64_t t = now_ms();
  if (t - last_log > 2000) {
    last_log = t;
    char buf[80];
    snprintf(buf, sizeof(buf), "CMD_CEIL xy_lock=%d vz=%.2f yaw=%.1f",
             hover_xy_locked, vz_down, yaw);
    if(log_fp) { fprintf(log_fp, "# %s\n", buf); }
    if(txt_log_fp) { fprintf(txt_log_fp, "[%.3f] %s\n", t*0.001f, buf); }
  }

  uint16_t mask;
  float px = 0.0f, py = 0.0f;

  if (hover_xy_locked) {
    // Hold locked XY position, command Z velocity, hold yaw
    px = hover_lock_x_m;
    py = hover_lock_y_m;
    mask = (1<<2)              // ignore z position (use vz instead)
         | (1<<3)|(1<<4)      // ignore vx,vy (position controller handles XY)
         | (1<<6)|(1<<7)|(1<<8) // ignore accel
         | (1<<11);           // ignore yaw_rate
  } else {
    // No XY lock — command zero XY velocity, Z velocity, hold yaw
    mask = (1<<0)|(1<<1)|(1<<2) // ignore position
         | (1<<6)|(1<<7)|(1<<8) // ignore accel
         | (1<<11);             // ignore yaw_rate
  }

  mavlink_message_t m;
  mavlink_msg_set_position_target_local_ned_pack(
      g_sysid, g_compid, &m,
      (uint32_t)now_ms(),
      fc_sysid, fc_compid,
      MAV_FRAME_LOCAL_NED,
      mask,
      px, py, 0.0f,
      0.0f, 0.0f, vz_down,
      0,0,0,
      deg2rad(yaw), 0
  );
  mav_send(&m);
}

// Pre-lock hover: brake lateral motion with XY velocity control while keeping
// altitude and yaw under position control. This avoids chasing a distant
// preliminary XY anchor and re-triggering high-velocity lock failures.
static void send_prelock_hold(float vx, float vy) {
  if (!have_fc) return;

  float z_down = hover_target_z_down();
  float yaw = hover_yaw_setpoint_deg();

  static uint64_t last_log = 0;
  uint64_t t = now_ms();
  if (t - last_log > 2000) {
    last_log = t;
    char buf[80];
    snprintf(buf, sizeof(buf), "CMD_PRELOCK vx=%.2f vy=%.2f z=%.2f yaw=%.1f", vx, vy, z_down, yaw);
    if(log_fp) { fprintf(log_fp, "# %s\n", buf); }
    if(txt_log_fp) { fprintf(txt_log_fp, "[%.3f] %s\n", t*0.001f, buf); }
  }

  uint16_t mask =
      (1<<0)|(1<<1)          // ignore x,y position (use vx,vy)
    | (1<<5)                 // ignore vz (use z position)
    | (1<<6)|(1<<7)|(1<<8)   // ignore accel
    | (1<<11);               // ignore yaw_rate

  mavlink_message_t m;
  mavlink_msg_set_position_target_local_ned_pack(
      g_sysid, g_compid, &m,
      (uint32_t)now_ms(),
      fc_sysid, fc_compid,
      MAV_FRAME_LOCAL_NED,
      mask,
      0.0f, 0.0f, z_down,
      vx, vy, 0.0f,
      0,0,0,
      deg2rad(yaw), 0
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
    snprintf(buf, sizeof(buf), "CMD_Z z=%.2f yaw=%.1f", z_down, yaw_deg);
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
  if (thrust > 0.80f) thrust = 0.80f;

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

static bool ekf_fresh(uint64_t t) {
  return have_ekf && (t - ekf_last_ms) < 1000;
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
  return xy_enabled && xy_healthy;
}

static bool of_fresh(uint64_t t) {
  return have_of && (t - of_last_update_ms) < SENSOR_FRESH_MS;
}

static bool fc_link_fresh(uint64_t t) {
  return have_fc && last_hb_ms != 0 && (t - last_hb_ms) < HEARTBEAT_TIMEOUT_MS;
}

static bool att_fresh(uint64_t t) {
  return have_att && att_last_update_ms != 0 && (t - att_last_update_ms) < SENSOR_FRESH_MS;
}

// ----------------------------- Pose ring helpers -----------------------
static float lerp_float(float a, float b, float u) {
  return a + (b - a) * u;
}

static float lerp_angle_deg(float a, float b, float u) {
  float delta = b - a;
  while (delta >= 180.0f)  delta -= 360.0f;
  while (delta < -180.0f)  delta += 360.0f;
  float r = a + delta * u;
  while (r >= 180.0f)  r -= 360.0f;
  while (r < -180.0f)  r += 360.0f;
  return r;
}

static uint16_t sat_u16_age(uint64_t ref_ms, uint64_t update_ms) {
  if (update_ms == 0 || ref_ms <= update_ms) return 0;
  uint64_t age = ref_ms - update_ms;
  if (age > 0xFFFFu) age = 0xFFFFu;
  return (uint16_t)age;
}

static uint16_t pose_flags_for_time(uint64_t t) {
  uint16_t flags = 0;
  if (fc_link_fresh(t))  flags |= POSEF_FC_LINK;
  if (fc_armed)          flags |= POSEF_FC_ARMED;
  if (att_fresh(t))      flags |= POSEF_ATT_FRESH;
  if (lpos_fresh(t))     flags |= POSEF_LPOS_FRESH;
  if (of_fresh(t))       flags |= POSEF_OF_FRESH;
  if (rf_fresh(t))       flags |= POSEF_RF_FRESH;
  if (sys_fresh(t))      flags |= POSEF_SYS_FRESH;
  if (isfinite(alt_est_m)) flags |= POSEF_ALT_VALID;
  return flags;
}

static bool pose_valid_for_mapping(uint64_t t) {
  if (!fc_link_fresh(t)) return false;
  if (!att_fresh(t))     return false;
  if (!lpos_fresh(t))    return false;
  if (!isfinite(lpos_x_m) || !isfinite(lpos_y_m)) return false;
  if (!isfinite(alt_est_m)) return false;
  return true;
}

static bool mapping_gate(uint64_t t) {
  if (!mapping_enabled) return false;
  if (!allow_disarmed_logging && !fc_armed) return false;
  return pose_valid_for_mapping(t);
}

static void pose_ring_push(uint64_t t) {
  pose_sample_t* s = &pose_ring[pose_ring_head];

  s->host_ms        = t;
  s->x_m            = lpos_x_m;
  s->y_m            = lpos_y_m;
  s->vx_mps         = lpos_vx_mps;
  s->vy_mps         = lpos_vy_mps;
  s->vz_mps         = lpos_vz_mps;
  s->alt_m          = alt_est_m;
  s->lpos_alt_m     = lpos_alt_m;
  s->lpos_alt_filt_m = lpos_alt_filt_m;
  s->yaw_deg        = have_att ? current_heading_deg() : NAN;
  s->roll_rad       = have_att ? roll_rad : NAN;
  s->pitch_rad      = have_att ? pitch_rad : NAN;
  s->alt_src        = (uint8_t)alt_src;
  s->valid_xy       = pose_valid_for_mapping(t) ? 1u : 0u;
  s->valid_att      = att_fresh(t) ? 1u : 0u;

  pose_ring_head = (pose_ring_head + 1) % POSE_RING_SZ;
  if (pose_ring_count < POSE_RING_SZ) pose_ring_count++;
}

static bool pose_ring_sample_at(uint64_t target_ms, pose_sample_t* out) {
  if (!out || pose_ring_count == 0) return false;

  const pose_sample_t* before  = NULL;
  const pose_sample_t* after   = NULL;
  const pose_sample_t* nearest = NULL;
  uint64_t nearest_dt = UINT64_MAX;

  for (int i = 0; i < pose_ring_count; i++) {
    int idx = (pose_ring_head - pose_ring_count + i + POSE_RING_SZ) % POSE_RING_SZ;
    const pose_sample_t* s = &pose_ring[idx];
    if (s->host_ms == 0) continue;

    uint64_t dt = (s->host_ms > target_ms) ? (s->host_ms - target_ms)
                                            : (target_ms  - s->host_ms);
    if (dt < nearest_dt) { nearest_dt = dt; nearest = s; }
    if (s->host_ms <= target_ms)  before = s;
    if (!after && s->host_ms >= target_ms) after = s;
  }

  if (!before && !after && !nearest) return false;
  if (!before || !after || before == after ||
      after->host_ms == before->host_ms) {
    *out = nearest ? *nearest : (before ? *before : *after);
    return true;
  }

  float u = (float)(target_ms - before->host_ms) /
            (float)(after->host_ms - before->host_ms);
  if (u < 0.0f) u = 0.0f;
  if (u > 1.0f) u = 1.0f;

  memset(out, 0, sizeof(*out));
  out->host_ms          = target_ms;
  out->x_m              = lerp_float(before->x_m,              after->x_m,              u);
  out->y_m              = lerp_float(before->y_m,              after->y_m,              u);
  out->vx_mps           = lerp_float(before->vx_mps,           after->vx_mps,           u);
  out->vy_mps           = lerp_float(before->vy_mps,           after->vy_mps,           u);
  out->vz_mps           = lerp_float(before->vz_mps,           after->vz_mps,           u);
  out->alt_m            = lerp_float(before->alt_m,            after->alt_m,            u);
  out->lpos_alt_m       = lerp_float(before->lpos_alt_m,       after->lpos_alt_m,       u);
  out->lpos_alt_filt_m  = lerp_float(before->lpos_alt_filt_m,  after->lpos_alt_filt_m,  u);
  out->yaw_deg          = lerp_angle_deg(before->yaw_deg,      after->yaw_deg,          u);
  out->roll_rad         = lerp_float(before->roll_rad,         after->roll_rad,         u);
  out->pitch_rad        = lerp_float(before->pitch_rad,        after->pitch_rad,        u);
  out->alt_src          = (u < 0.5f) ? before->alt_src : after->alt_src;
  out->valid_xy         = before->valid_xy && after->valid_xy;
  out->valid_att        = before->valid_att && after->valid_att;
  return true;
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
static uint64_t prearm_ok_since_ms = 0;
static uint64_t takeoff_xy_ok_since_ms = 0;

static bool z_bootstrap_ready_now(uint64_t t, char* reasons, size_t reasons_sz) {
  if (reasons && reasons_sz > 0) reasons[0] = '\0';

  bool att_ok = have_att;
  bool z_ok = z_ctrl_ok(t);
  bool rf_ok = rf_fresh(t) && !isnan(rangefinder_m);
  bool lpos_ok = lpos_fresh(t) && !isnan(lpos_alt_m);
  bool alt_ok = rf_ok || lpos_ok;

  if (!att_ok) append_reason(reasons, reasons_sz, "no_att");
  if (!z_ok) append_reason(reasons, reasons_sz, "z_ctrl_unhealthy");
  if (!alt_ok) append_reason(reasons, reasons_sz, "no_fresh_vertical_data");

  return reasons == NULL || reasons[0] == '\0';
}

static void log_hover_ready_failure(uint64_t t, const char* reason) {
  bool changed = strncmp(hover_ready_fail_last_reason, reason ? reason : "", sizeof(hover_ready_fail_last_reason)) != 0;
  if (!changed && (t - hover_ready_fail_log_ms) < 1000) return;

  hover_ready_fail_log_ms = t;
  snprintf(hover_ready_fail_last_reason, sizeof(hover_ready_fail_last_reason), "%s", reason ? reason : "");
  log_bootstrap_event(t, "HOVER_READY_FAIL", reason);
}

static bool hover_ready_now_internal(uint64_t t, bool strict, bool log_fail, char* reasons, size_t reasons_sz) {
  if (reasons && reasons_sz > 0) reasons[0] = '\0';

  if (!z_bootstrap_ready_now(t, reasons, reasons_sz)) {
    if (log_fail) log_hover_ready_failure(t, reasons);
    return false;
  }

  if (!strict) return true;

  bool lpos_ok = lpos_fresh(t);
  bool xy_ok = xy_ctrl_ok(t);
  bool rf_ok = rf_fresh(t) && !isnan(rangefinder_m);
  bool alt_ok = !isnan(alt_max_m);
  bool of_ok = of_fresh(t) && of_quality >= 30;

  if (!lpos_ok) append_reason(reasons, reasons_sz, "lpos_stale");
  if (!xy_ok) append_reason(reasons, reasons_sz, "xy_ctrl_unhealthy");
  if (REQUIRE_RANGEFINDER_FOR_HOVER && !rf_ok) append_reason(reasons, reasons_sz, "rangefinder_stale");
  if (!REQUIRE_RANGEFINDER_FOR_HOVER && !alt_ok) append_reason(reasons, reasons_sz, "alt_unknown");
  if (REQUIRE_OPTICAL_FLOW_FOR_HOVER && !of_ok) append_reason(reasons, reasons_sz, "flow_unhealthy");
  if (!alt_ok) append_reason(reasons, reasons_sz, "alt_stale");

  bool ok = reasons == NULL || reasons[0] == '\0';
  if (!ok && log_fail) log_hover_ready_failure(t, reasons);
  return ok;
}

static bool hover_ready_now(uint64_t t) {
  char reasons[160];
  return hover_ready_now_internal(t, true, true, reasons, sizeof(reasons));
}

static bool bootstrap_ready_stable(uint64_t t) {
  char reasons[160];
  bool ok = hover_ready_now_internal(t, false, false, reasons, sizeof(reasons));
  if (ok) {
    if (prearm_ok_since_ms == 0) prearm_ok_since_ms = t;
    return (t - prearm_ok_since_ms) >= PREARM_STABLE_MS;
  }

  prearm_ok_since_ms = 0;
  return false;
}

static bool xy_height_trustworthy_now(uint64_t t) {
  float alt_now = NAN;
  if (rf_fresh(t) && !isnan(rangefinder_m)) {
    alt_now = rangefinder_m;
  } else if (!isnan(alt_est_m) && alt_src != ALT_SRC_ON_GROUND) {
    alt_now = alt_est_m;
  }
  return !isnan(alt_now) && alt_now >= HOVER_CAPTURE_MIN_ALT_M;
}

static bool takeoff_xy_sanity_ready(uint64_t t) {
  bool rf_ground = rf_fresh(t) && !isnan(rangefinder_m) &&
                   rangefinder_m <= TAKEOFF_XY_SANITY_GROUND_MAX_M;
  bool lpos_ok = lpos_fresh(t) && have_xy &&
                 isfinite(lpos_x_m) && isfinite(lpos_y_m) &&
                 isfinite(lpos_vx_mps) && isfinite(lpos_vy_mps);
  bool vel_ok = lpos_ok &&
                fabsf(lpos_vx_mps) < TAKEOFF_XY_SANITY_MAX_MPS &&
                fabsf(lpos_vy_mps) < TAKEOFF_XY_SANITY_MAX_MPS;
  bool ok = rf_ground && lpos_ok && vel_ok;

  if (ok) {
    if (takeoff_xy_ok_since_ms == 0) takeoff_xy_ok_since_ms = t;
    return (t - takeoff_xy_ok_since_ms) >= TAKEOFF_XY_SANITY_DWELL_MS;
  }

  takeoff_xy_ok_since_ms = 0;
  return false;
}

static bool vel_xy_motion_ok(uint64_t t, char* reasons, size_t reasons_sz) {
  if (reasons && reasons_sz > 0) reasons[0] = '\0';

  if (!xy_ctrl_ok(t)) append_reason(reasons, reasons_sz, "xy_ctrl_unhealthy");
  if (!have_att) append_reason(reasons, reasons_sz, "no_att");
  if (!lpos_fresh(t)) append_reason(reasons, reasons_sz, "lpos_stale");
  if (!isfinite(lpos_vx_mps) || !isfinite(lpos_vy_mps)) append_reason(reasons, reasons_sz, "vel_nan");
  if (of_fresh(t) && of_quality < 30) append_reason(reasons, reasons_sz, "flow_q_low");
  if (!xy_height_trustworthy_now(t)) append_reason(reasons, reasons_sz, "alt_too_low");
  if (isfinite(lpos_vx_mps) && fabsf(lpos_vx_mps) >= VX_LOCK_MAX_MPS) append_reason(reasons, reasons_sz, "vx_high");
  if (isfinite(lpos_vy_mps) && fabsf(lpos_vy_mps) >= VY_LOCK_MAX_MPS) append_reason(reasons, reasons_sz, "vy_high");
  if (ekf_fresh(t) && ekf_pos_horiz_var > EKF_POS_VAR_LOCK_MAX) append_reason(reasons, reasons_sz, "ekf_pos_var_high");

  return reasons == NULL || reasons[0] == '\0';
}

static bool vel_xy_stable(uint64_t t) {
  // Block candidate-start entirely during relock cooldown
  if (t < xy_relock_block_until_ms) {
    if (xy_lock_candidate_active) {
      xy_lock_candidate_active = false;
      xy_lock_candidate_since_ms = 0;
    }
    return false;
  }

  char reasons[160];
  bool ok = vel_xy_motion_ok(t, reasons, sizeof(reasons));

  if (ok) {
    if (!xy_lock_candidate_active) {
      xy_lock_candidate_active = true;
      xy_lock_candidate_since_ms = t;
      log_bootstrap_event(t, "XY_LOCK_CANDIDATE_START", "low_xy_velocity");
    }
    return (t - xy_lock_candidate_since_ms) >= XY_LOCK_DWELL_MS;
  }

  if (xy_lock_candidate_active) {
    xy_lock_candidate_active = false;
    xy_lock_candidate_since_ms = 0;
    log_bootstrap_event(t, "XY_LOCK_CANDIDATE_FAIL", reasons);
  }
  return false;
}

// ----------------------------- Hover / PosHold -------------------------
static void init_hover_targets_on_ground(uint64_t t) {
  (void)t;
  hover_xy_locked = false;
  hover_xy_lock_ms = 0;
  hover_enter_ms = 0;
  hover_lock_x_m = 0.0f;
  hover_lock_y_m = 0.0f;
  xy_lock_candidate_active = false;
  xy_lock_candidate_since_ms = 0;
  hover_health_fail_since_ms = 0;
  hover_ready_fail_log_ms = 0;
  hover_ready_fail_last_reason[0] = '\0';
  hover_yaw_latched = false;
  xy_relock_block_until_ms = 0;
  yaw_drift_last_strike_ms = 0;
  yaw_drift_last_strike_err = 0.0f;
  yaw_spin_suppress_until_ms = 0;
  prov_yaw_candidate_deg = NAN;
  prov_yaw_candidate_ms = 0;

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

  bool lpos_ok = lpos_fresh(t) && isfinite(lpos_x_m) && isfinite(lpos_y_m);
  if (!hover_xy_locked && lpos_ok && (!isfinite(hover_lock_x_m) || !isfinite(hover_lock_y_m))) {
    hover_lock_x_m = lpos_x_m;
    hover_lock_y_m = lpos_y_m;
    printf("HOVER: deferred XY capture (%.2f, %.2f)\n", hover_lock_x_m, hover_lock_y_m);
  }

  // Block re-lock during cooldown after XY_LOCK_BREAK
  if (!hover_xy_locked && t < xy_relock_block_until_ms) {
    xy_lock_candidate_active = false;
    xy_lock_candidate_since_ms = 0;
  }

  // Provisional yaw target convergence: before first XY lock, if the heading
  // has settled at a new value far from the original target, update the target.
  // This handles the case where takeoff yaw spin shifts the heading — once it
  // stabilizes, we accept the new heading rather than blocking lock forever.
  if (!hover_yaw_latched && have_yaw_target && have_att) {
    float cur_h = current_heading_deg();
    float prov_err = fabsf(wrap_deg(cur_h - yaw_target_deg));
    if (prov_err > YAW_RELOCK_ERR_MAX_DEG) {
      // Heading is far from target — track a candidate
      if (prov_yaw_candidate_ms == 0) {
        prov_yaw_candidate_deg = cur_h;
        prov_yaw_candidate_ms = t;
      } else {
        float cand_err = fabsf(wrap_deg(cur_h - prov_yaw_candidate_deg));
        if (cand_err > PROV_YAW_STABLE_DEG) {
          // Heading still moving — reset candidate
          prov_yaw_candidate_deg = cur_h;
          prov_yaw_candidate_ms = t;
        } else if ((t - prov_yaw_candidate_ms) >= PROV_YAW_STABLE_MS) {
          // Heading stable at new value — update provisional target
          printf("HOVER: provisional yaw target updated %.1f -> %.1f (stable %lums)\n",
                 yaw_target_deg, cur_h, (unsigned long)(t - prov_yaw_candidate_ms));
          log_bootstrap_event(t, "PROV_YAW_UPDATE", "heading_converged");
          yaw_target_deg = cur_h;
          hover_hold_yaw_deg = cur_h;
          have_yaw_target = true;
          prov_yaw_candidate_ms = 0;
          prov_yaw_candidate_deg = NAN;
        }
      }
    } else {
      // Close enough to current target — no update needed
      prov_yaw_candidate_ms = 0;
      prov_yaw_candidate_deg = NAN;
    }
  }

  if (!hover_xy_locked && vel_xy_stable(t) && lpos_ok
      && t >= xy_relock_block_until_ms) {
    // Block lock if a yaw strike happened recently
    if (yaw_drift_last_strike_ms != 0 &&
        (t - yaw_drift_last_strike_ms) <= XY_RELOCK_COOLDOWN_MS) {
      static uint64_t last_strike_block_log_ms = 0;
      if (t - last_strike_block_log_ms > 500) {
        last_strike_block_log_ms = t;
        printf("HOVER: XY lock blocked (recent yaw strike %llums ago)\n",
               (unsigned long long)(t - yaw_drift_last_strike_ms));
      }
      log_bootstrap_event(t, "XY_LOCK_BLOCKED", "recent_yaw_strike");
      goto skip_xy_lock;
    }
    // Yaw gate: require heading close to provisional/latched target
    float yaw_err = fabsf(wrap_deg(current_heading_deg() - yaw_target_deg));
    if (yaw_err >= YAW_RELOCK_ERR_MAX_DEG) {
      static uint64_t last_yaw_block_log_ms = 0;
      if (t - last_yaw_block_log_ms > 500) {
        last_yaw_block_log_ms = t;
        printf("HOVER: XY lock blocked (yaw_err=%.1f > %.1f)\n",
               yaw_err, YAW_RELOCK_ERR_MAX_DEG);
      }
      log_bootstrap_event(t, "XY_LOCK_BLOCKED",
                          hover_yaw_latched ? "relock_yaw_err" : "first_lock_yaw_err");
      goto skip_xy_lock;
    }
    bool is_relock = hover_yaw_latched;
    hover_lock_x_m = lpos_x_m;
    hover_lock_y_m = lpos_y_m;
    hover_xy_locked = true;
    hover_xy_lock_ms = t;
    xy_lock_candidate_active = false;
    xy_lock_candidate_since_ms = 0;
    // Latch yaw only on the first lock of this hover session
    if (!hover_yaw_latched) {
      yaw_target_deg = current_heading_deg();
      hover_hold_yaw_deg = yaw_target_deg;
      have_yaw_target = true;
      hover_yaw_latched = true;
    }
    log_bootstrap_event(t, "XY_LOCK_SUCCESS",
                        is_relock ? "relock" : "velocity_dwell_met");
  }
skip_xy_lock: (void)0;

  // Break XY lock if EKF position variance spikes or actual position
  // drifts too far from the lock anchor (EKF origin shift / aiding loss).
  if (hover_xy_locked && lpos_ok) {
    bool var_bad = ekf_fresh(t) && ekf_pos_horiz_var > EKF_POS_VAR_BREAK;
    float dx = lpos_x_m - hover_lock_x_m;
    float dy = lpos_y_m - hover_lock_y_m;
    float drift = hypotf(dx, dy);
    bool drifted = drift >= POSTLOCK_MAX_DRIFT_M;

    if (var_bad || drifted) {
      printf("HOVER: XY lock broken (%s, drift=%.2fm, ekf_ph=%.2f) -> re-acquire\n",
             var_bad ? "ekf_var" : "drift", drift, ekf_pos_horiz_var);
      log_bootstrap_event(t, "XY_LOCK_BREAK",
                          var_bad ? "ekf_pos_var_high" : "postlock_drift");
      hover_xy_locked = false;
      hover_xy_lock_ms = 0;
      xy_lock_candidate_active = false;
      xy_lock_candidate_since_ms = 0;
      // Reset prelock timer so we get a fresh window to re-lock
      hover_enter_ms = t;
      // Snap preliminary anchor to current position
      hover_lock_x_m = lpos_x_m;
      hover_lock_y_m = lpos_y_m;
      // Cooldown before allowing re-lock
      xy_relock_block_until_ms = t + XY_RELOCK_COOLDOWN_MS;
      // Do NOT touch yaw — latched yaw stays
    }
  }

  float yaw = have_yaw_target ? yaw_target_deg : current_heading_deg();
  float z_down = hover_target_z_down();

  if (hover_xy_locked) {
    send_pos_yaw_ned(hover_lock_x_m, hover_lock_y_m, z_down, yaw);
  } else if (lpos_ok) {
    send_prelock_hold(0.0f, 0.0f);
  } else {
    send_z_yaw_ned(z_down, yaw);
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
  send_command_long_tgt(tgt_sys, tgt_comp, MAV_CMD_SET_MESSAGE_INTERVAL, (float)MAVLINK_MSG_ID_EKF_STATUS_REPORT, 200000.0f, 0,0,0,0,0);
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
  roll_rad  = a.roll;
  pitch_rad = a.pitch;
  yaw_rad   = a.yaw;
  att_last_update_ms = now_ms();
  have_att  = true;
}

static void handle_optical_flow(const mavlink_message_t *msg) {
  mavlink_optical_flow_t o;
  mavlink_msg_optical_flow_decode(msg, &o);

  have_of      = true;
  of_src       = OF_SRC_OPTICAL_FLOW;
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
  of_src     = OF_SRC_OPTICAL_FLOW_RAD;
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
      rf_src = RF_SRC_DISTANCE_SENSOR;
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
    rf_src = RF_SRC_RANGEFINDER;
  }
}

static void handle_ekf_status_report(const mavlink_message_t *msg) {
  if (msg->len < 22) return;
  const uint8_t* p = (const uint8_t*)_MAV_PAYLOAD(msg);
  ekf_vel_var       = rd_f32_le(p + 0);
  ekf_pos_horiz_var = rd_f32_le(p + 4);
  ekf_pos_vert_var  = rd_f32_le(p + 8);
  ekf_last_ms = now_ms();
  have_ekf = true;
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
          case MAVLINK_MSG_ID_EKF_STATUS_REPORT:   handle_ekf_status_report(&msg); break;
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

    // Whichever parser is mid-frame gets the byte — this prevents the
    // CTRL parser from stealing bytes out of a 518-byte scan frame when
    // a data byte happens to match CTRL_HEADER (0xA6).

    // --- CTRL parser continuation (mid-frame) ---
    if (ctrl_rxpos > 0) {
      ctrl_rxbuf[ctrl_rxpos++] = b;
      if (ctrl_rxpos == CTRL_BYTES) {
        uint8_t c = xor8(ctrl_rxbuf, CTRL_BYTES-1);
        if (c == ctrl_rxbuf[CTRL_BYTES-1]) {
          accept_ctrl_frame(ctrl_rxbuf);
        } else {
          printf("CTRL_DBG: checksum FAIL (got=0x%02X want=0x%02X) bytes=[%02X %02X %02X %02X %02X %02X %02X]\n",
                 c, ctrl_rxbuf[CTRL_BYTES-1],
                 ctrl_rxbuf[0], ctrl_rxbuf[1], ctrl_rxbuf[2], ctrl_rxbuf[3],
                 ctrl_rxbuf[4], ctrl_rxbuf[5], ctrl_rxbuf[6]);
        }
        ctrl_rxpos = 0;
      }
      continue;
    }

    // --- SCAN parser continuation (mid-frame) ---
    if (tof_rxpos > 0) {
      tof_rxbuf[tof_rxpos++] = b;
      if (tof_rxpos == SCAN_BYTES) {
        uint8_t c = xor8(tof_rxbuf, SCAN_BYTES-1);
        if (c == tof_rxbuf[SCAN_BYTES-1]) {
          accept_scan_frame(tof_rxbuf);
        }
        tof_rxpos = 0;
      }
      continue;
    }

    // --- Neither parser active: check for frame headers ---
    if (b == CTRL_HEADER) {
      printf("CTRL_DBG: got header 0xA6 (tof_rxpos=%d ctrl_rxpos=%d)\n", tof_rxpos, ctrl_rxpos);
      ctrl_rxbuf[ctrl_rxpos++] = b;
    } else if (b == SCAN_HEADER) {
      tof_rxbuf[tof_rxpos++] = b;
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

  // Reject rangefinder when tilted beyond 30 deg — beam no longer points down
  bool rf_tilt_ok = true;
  if (have_att && (fabsf(roll_rad) > 0.52f || fabsf(pitch_rad) > 0.52f))
    rf_tilt_ok = false;

  // --- Slew-rate-limited RF for altitude estimation ---
  // Prevents terrain discontinuities (flying over pillows, bed edges, etc.)
  // from corrupting the altitude estimate.  Raw RF is still used for ceiling
  // TRIGGER so the ceiling activates immediately on real climbs.
  float rf_slew = NAN;
  if (range_fresh && rf_tilt_ok && !isnan(rangefinder_m)) {
    float rf_raw = rangefinder_m;
    if (rf_raw < 0.0f) rf_raw = 0.0f;
    if (rf_raw > 10.0f) rf_raw = 10.0f;
    rf_slew = rf_raw;

    bool airborne = (have_ext && landed_state != MAV_LANDED_STATE_ON_GROUND);
    if (airborne && !isnan(rf_slew_m) && rf_slew_last_ms > 0) {
      float dt_s = (float)(tnow - rf_slew_last_ms) * 0.001f;
      if (dt_s > 0.001f && dt_s < 1.0f) {
        float max_delta = RF_MAX_SLEW_MPS * dt_s;
        float delta = rf_slew - rf_slew_m;
        if (fabsf(delta) > max_delta) {
          rf_slew = rf_slew_m + copysignf(max_delta, delta);
        }
      }
    }
    rf_slew_m = rf_slew;
    rf_slew_last_ms = tnow;
  }
  // Reset slew state on the ground so takeoff starts with a clean baseline
  if (!fc_armed || near_ground) {
    rf_slew_m = NAN;
    rf_slew_last_ms = 0;
  }

  // Compute max altitude for ceiling safety (independent of chosen alt_est source)
  float max_alt = NAN;
  if (lpos_fresh && !isnan(lpos_alt_filt_m)) {
    float a = lpos_alt_filt_m;
    if (a < -1.0f) a = -1.0f;
    if (a > 50.0f) a = 50.0f;
    max_alt = isnan(max_alt) ? a : fmaxf(max_alt, a);
  }
  if (range_fresh && rf_tilt_ok && !isnan(rangefinder_m)) {
    float rf = rangefinder_m;
    if (rf < 0.0f) rf = 0.0f;
    if (rf > 10.0f) rf = 10.0f;
    max_alt = isnan(max_alt) ? rf : fmaxf(max_alt, rf);
  }
  if (near_ground) {
    max_alt = isnan(max_alt) ? 0.0f : fmaxf(max_alt, 0.0f);
  }
  ceiling_alt_now_m = max_alt;
  // Decaying peak hold: rises instantly (safety — catch real ceiling approaches),
  // decays slowly so terrain RF jumps don't prematurely release ceiling mode.
  if (!isnan(alt_max_m) && !isnan(max_alt) && fc_armed) {
    if (max_alt >= alt_max_m) {
      alt_max_m = max_alt;
    } else {
      alt_max_m = alt_max_m * (1.0f - ALT_MAX_DECAY_ALPHA) + max_alt * ALT_MAX_DECAY_ALPHA;
    }
  } else {
    alt_max_m = max_alt;
  }

  // Pick best altitude estimate for control logic
  // Uses slew-limited RF to reject terrain discontinuities (pillows, bed edges, etc.)
  float  new_alt = NAN;
  AltSrc new_src = ALT_SRC_NONE;
  alt_rf_rejected = false;

  if (!isnan(rf_slew)) {
    float rf = rf_slew;

    bool rf_ok = true;

    bool airborne_hint = false;
    if (have_ext && landed_state != MAV_LANDED_STATE_ON_GROUND) airborne_hint = true;
    if (lpos_fresh && !isnan(lpos_alt_filt_m) && lpos_alt_filt_m > 0.20f) airborne_hint = true;
    if (airborne_hint && rf < 0.05f) rf_ok = false;

    if (lpos_fresh && !isnan(lpos_alt_filt_m) && fabsf(rf - lpos_alt_filt_m) > 0.80f) rf_ok = false;

    if (rf_ok) {
      new_alt = rf;
      new_src = ALT_SRC_RANGEFINDER;
    } else {
      alt_rf_rejected = true;
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

  // Ceiling hysteresis uses alt_max_m to avoid “stuck-low RF hides climb”.
  // Trigger before the hard ceiling so the FC has room to arrest an indoor climb.
  const float ceiling_trigger_m = CEIL_M - CEIL_MARGIN_M;
  const float ceiling_release_m = ceiling_trigger_m - CEIL_TAPER_DEPTH_M;
  bool ceiling_trigger_now  = !isnan(ceiling_alt_now_m) && ceiling_alt_now_m >= ceiling_trigger_m;
  bool ceiling_trigger_peak = !isnan(alt_max_m) && alt_max_m >= ceiling_trigger_m;
  if (ceiling_trigger_now || ceiling_trigger_peak) ceiling_active = true;
  // Don't release ceiling mode while still descending fast — handing off to
  // the hover position controller with high downward velocity causes a crash.
  float ceil_vz = isnan(lpos_vz_mps) ? 0.0f : lpos_vz_mps;
  float ceiling_release_alt = ceiling_control_alt_m();
  if (ceiling_active && !isnan(ceiling_release_alt) &&
      ceiling_release_alt <= ceiling_release_m && ceil_vz < CEILING_RELEASE_VZ_MAX)
    ceiling_active = false;
  if (!fc_armed) ceiling_active = false;
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
              "t_ms,state,want_arm,armed,mode,alt_max_m,alt_est_m,alt_src,rf_m,rf_fresh,rf_rejected,"
              "lpos_alt_m,lpos_alt_filt_m,z_tgt_down_m,cmd_vz_ned_mps,ceiling_active,"
              "takeoff_sent,takeoff_started,off_ground,hover_xy_locked,"
              "x_m,y_m,vx_mps,vy_mps,yaw_deg,of_q,of_rate_x,of_rate_y,"
              "tof_f,tof_r,tof_b,tof_l,batt_v,batt_cells,"
              "ekf_ph_var,ekf_vel_var\n");
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
      const char hdr[] = "SCLOG3\n";
      fwrite(hdr, 1, sizeof(hdr)-1, scan_fp);
      fflush(scan_fp);
    }
  }

  txt_log_fp = fopen(LOG_TXT_PATH, "a");
  if (!txt_log_fp) {
    fprintf(stderr, "WARN: cannot open %s: %s\n", LOG_TXT_PATH, strerror(errno));
  } else {
    // Write a session start banner with wall-clock time so battery-swap
    // boundaries are obvious when reviewing the log.
    char timebuf[32] = "unknown";
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    if (tm_info) strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", tm_info);
    fprintf(txt_log_fp,
            "\n"
            "================================================================\n"
            "  SESSION START  %s\n"
            "================================================================\n",
            timebuf);
    fflush(txt_log_fp);
    session_start_ms = now_ms();
  }

  last_flush_ms = now_ms();
  last_log_ms   = 0;
}

static void log_close(const char* reason) {
  if (txt_log_fp) {
    char timebuf[32] = "unknown";
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    if (tm_info) strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", tm_info);
    uint64_t elapsed_ms = now_ms() - session_start_ms;
    unsigned int elapsed_s  = (unsigned int)(elapsed_ms / 1000);
    unsigned int elapsed_m  = elapsed_s / 60;
    elapsed_s %= 60;
    fprintf(txt_log_fp,
            "\n"
            "================================================================\n"
            "  SESSION END    %s  (%s)  duration=%um%02us\n"
            "================================================================\n\n",
            timebuf, reason ? reason : "unknown", elapsed_m, elapsed_s);
    fflush(txt_log_fp);
    fclose(txt_log_fp);
    txt_log_fp = NULL;
  }
  if (log_fp)  { fclose(log_fp);  log_fp  = NULL; }
  if (scan_fp) { fclose(scan_fp); scan_fp = NULL; }
}

static void sig_handler(int sig) {
  (void)sig;
  g_shutdown = 1;
}

static void log_flush_if_due(uint64_t t) {
  if (t - last_flush_ms < LOG_FLUSH_MS) return;
  last_flush_ms = t;
  if (log_fp) fflush(log_fp);
  if (scan_fp) fflush(scan_fp);
  if (txt_log_fp) fflush(txt_log_fp);
}

typedef struct __attribute__((packed)) {
  uint32_t magic;        // 'SCN3'
  uint64_t host_ms;
  uint32_t scan_ms;
  uint32_t custom_mode;

  float    x_m;
  float    y_m;
  float    yaw_deg;
  float    alt_m;
  float    alt_max_m;

  float    roll_rad;
  float    pitch_rad;

  float    vx_mps;
  float    vy_mps;
  float    vz_mps;

  float    lpos_alt_m;
  float    lpos_alt_filt_m;
  float    rf_m;
  float    rf_v;
  float    of_rate_x;
  float    of_rate_y;
  float    of_comp_m_x;
  float    of_comp_m_y;
  float    of_ground_m;

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
    pose.host_ms       = sample_ms;
    pose.x_m           = lpos_x_m;
    pose.y_m           = lpos_y_m;
    pose.vx_mps        = lpos_vx_mps;
    pose.vy_mps        = lpos_vy_mps;
    pose.vz_mps        = lpos_vz_mps;
    pose.alt_m         = alt_est_m;
    pose.lpos_alt_m    = lpos_alt_m;
    pose.lpos_alt_filt_m = lpos_alt_filt_m;
    pose.yaw_deg       = current_heading_deg();
    pose.roll_rad      = roll_rad;
    pose.pitch_rad     = pitch_rad;
    pose.alt_src       = (uint8_t)alt_src;
    pose.valid_xy      = 1u;
    pose.valid_att     = 1u;
  }

  scanrec_t r;
  memset(&r, 0, sizeof(r));
  r.magic       = 0x334E4353u; // 'SCN3'
  r.host_ms     = last_scan_host_ms;
  r.scan_ms     = last_scan_t_ms;
  r.custom_mode = hb_custom_mode;

  r.x_m              = pose.x_m;
  r.y_m              = pose.y_m;
  r.yaw_deg          = pose.yaw_deg;
  r.alt_m            = pose.alt_m;
  r.alt_max_m        = alt_max_m;

  r.roll_rad         = pose.roll_rad;
  r.pitch_rad        = pose.pitch_rad;
  r.vx_mps           = pose.vx_mps;
  r.vy_mps           = pose.vy_mps;
  r.vz_mps           = pose.vz_mps;
  r.lpos_alt_m       = pose.lpos_alt_m;
  r.lpos_alt_filt_m  = pose.lpos_alt_filt_m;

  r.rf_m             = rf_fresh(sample_ms) ? rangefinder_m : NAN;
  r.rf_v             = rangefinder_v;
  r.of_rate_x        = of_rate_x;
  r.of_rate_y        = of_rate_y;
  r.of_comp_m_x      = of_comp_m_x;
  r.of_comp_m_y      = of_comp_m_y;
  r.of_ground_m      = of_ground_m;

  r.sys_present      = have_sys ? sys_present : 0;
  r.sys_health       = have_sys ? sys_health  : 0;
  r.sys_enabled      = have_sys ? sys_enabled : 0;

  r.att_age_ms       = sat_u16_age(sample_ms, att_last_update_ms);
  r.lpos_age_ms      = sat_u16_age(sample_ms, lpos_last_update_ms);
  r.of_age_ms        = sat_u16_age(sample_ms, of_last_update_ms);
  r.rf_age_ms        = sat_u16_age(sample_ms, rangefinder_last_update_ms);
  r.hb_age_ms        = sat_u16_age(sample_ms, last_hb_ms);
  r.pose_flags       = pose_flags_for_time(sample_ms);

  r.of_q             = of_fresh(sample_ms) ? of_quality : 0;
  r.alt_src          = pose.alt_src;
  r.rf_src           = (uint8_t)rf_src;
  r.of_src           = (uint8_t)of_src;
  r.fc_armed         = fc_armed ? 1u : 0u;
  r.alt_rf_rejected  = alt_rf_rejected ? 1u : 0u;
  r.ds_orientation   = ds_orientation;
  r.ds_id            = ds_id;
  r.ds_cur_cm        = ds_cur_cm;

  memcpy(r.grid_raw, last_scan_grid_raw, sizeof(r.grid_raw));
  fwrite(&r, 1, sizeof(r), scan_fp);
  scan_new = false;
}

static void log_tick(uint64_t t) {
  const uint64_t period_ms = 1000 / LOG_HZ;

  if (log_fp && flight_log_active() && (t - last_log_ms) >= period_ms) {
    last_log_ms = t;
    bool rf_now_fresh = rf_fresh(t);
    bool off_ground = takeoff_off_ground(t);

    fprintf(log_fp,
            "%llu,%s,%d,%d,%u,",
            (unsigned long long)t,
            state_name(st),
            want_arm ? 1 : 0,
            fc_armed ? 1 : 0,
            (unsigned)hb_custom_mode);

    if (!isnan(alt_max_m)) fprintf(log_fp, "%.3f,", alt_max_m);
    else fprintf(log_fp, "nan,");
    if (!isnan(alt_est_m)) fprintf(log_fp, "%.3f,", alt_est_m);
    else fprintf(log_fp, "nan,");
    fprintf(log_fp, "%s,", alt_src_name(alt_src));

    if (have_rangefinder) fprintf(log_fp, "%.3f,", rangefinder_m);
    else fprintf(log_fp, "nan,");
    fprintf(log_fp, "%d,%d,", rf_now_fresh ? 1 : 0, alt_rf_rejected ? 1 : 0);

    if (have_lpos) fprintf(log_fp, "%.3f,%.3f,", lpos_alt_m, lpos_alt_filt_m);
    else fprintf(log_fp, "nan,nan,");

    if (!isnan(logged_z_target_down())) fprintf(log_fp, "%.3f,", logged_z_target_down());
    else fprintf(log_fp, "nan,");
    if (!isnan(logged_cmd_vz_ned())) fprintf(log_fp, "%.3f,", logged_cmd_vz_ned());
    else fprintf(log_fp, "nan,");
    fprintf(log_fp, "%d,%d,%d,%d,%d,",
            ceiling_active ? 1 : 0,
            takeoff_sent ? 1 : 0,
            takeoff_started ? 1 : 0,
            off_ground ? 1 : 0,
            hover_xy_locked ? 1 : 0);

    if (have_xy) fprintf(log_fp, "%.3f,%.3f,%.3f,%.3f,",
                         lpos_x_m, lpos_y_m, lpos_vx_mps, lpos_vy_mps);
    else fprintf(log_fp, "nan,nan,nan,nan,");

    if (have_att) fprintf(log_fp, "%.3f,", current_heading_deg());
    else fprintf(log_fp, "nan,");

    bool of_ok = of_fresh(t);
    if (of_ok) fprintf(log_fp, "%u,", (unsigned)of_quality);
    else fprintf(log_fp, "0,");

    if (of_ok && !isnan(of_rate_x) && !isnan(of_rate_y)) fprintf(log_fp, "%.4f,%.4f,", of_rate_x, of_rate_y);
    else fprintf(log_fp, "nan,nan,");

    fprintf(log_fp, "%.3f,%.3f,%.3f,%.3f,",
            tof_filt_m[D_FRONT], tof_filt_m[D_RIGHT], tof_filt_m[D_BACK], tof_filt_m[D_LEFT]);

    if (!isnan(batt_v_total) && batt_cells > 0) fprintf(log_fp, "%.3f,%d,", batt_v_total, batt_cells);
    else fprintf(log_fp, "nan,0,");

    if (ekf_fresh(t)) fprintf(log_fp, "%.3f,%.3f\n", ekf_pos_horiz_var, ekf_vel_var);
    else fprintf(log_fp, "nan,nan\n");
  }

  log_scan_record();

  log_flush_if_due(t);
}

// ----------------------------- Behavior --------------------------------
static void enter_state(State ns, const char* reason) {
  if (st == ns) return;

  bool leaving_hover = (st == ST_HOVER && ns != ST_HOVER);
  bool entering_hover = (ns == ST_HOVER);

  if (leaving_hover) {
    hover_xy_locked = false;
    hover_xy_lock_ms = 0;
    hover_enter_ms = 0;
    hover_lock_x_m = hover_lock_y_m = 0.0f;
    xy_lock_candidate_active = false;
    xy_lock_candidate_since_ms = 0;
    printf("HOVER: reset XY capture (leaving %s)\n", state_name(ns));
  }

  if (st == ST_LIFTOFF_ASSIST && ns != ST_LIFTOFF_ASSIST) {
    // ensure no stale overrides; attitude target ramp uses GUIDED and stops automatically.
  }

  if (ns == ST_TAKEOFF) {
    takeoff_xy_ok_since_ms = 0;
    takeoff_sent = false;
    takeoff_sent_ms = 0;
    have_takeoff_ack = false;

    takeoff_started = false;
    takeoff_started_ms = 0;
    takeoff_off_ground_since_ms = 0;
    takeoff_was_airborne = false;
    takeoff_grounded_since_ms = 0;
    takeoff_att_ramp_active = false;
    takeoff_att_ramp_start_ms = 0;
    takeoff_alt0_m = alt_max_m;
    takeoff_att_ramp_was_active = false;
    takeoff_brake_active = false;
    takeoff_brake_start_ms = 0;
    takeoff_brake_settled_since_ms = 0;
    takeoff_brake_prev_yaw_deg = NAN;
    takeoff_brake_prev_yaw_ms = 0;

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
    hover_xy_lock_ms = 0;
    hover_enter_ms = now_ms();
    hover_yaw_latched = false;
    xy_relock_block_until_ms = 0;
    // Defer XY capture — let hover_hold_tick recapture once LPOS is fresh
    // and XY estimation is trustworthy. This avoids latching a position
    // while the EKF is still converging after takeoff.
    hover_lock_x_m = hover_lock_y_m = NAN;
    xy_lock_candidate_active = false;
    xy_lock_candidate_since_ms = 0;
    if (have_att) {
      yaw_target_deg = current_heading_deg();
      hover_hold_yaw_deg = yaw_target_deg;
      have_yaw_target = true;
    }
    printf("HOVER: preliminary XY capture (%.2f, %.2f)\n", hover_lock_x_m, hover_lock_y_m);
  }

  if (ns == ST_LANDING) {
    land_mode_sent = false;
    land_mode_sent_ms = 0;
    pending_kf_flags |= KF_LAND_START;
  }

  printf("STATE: %s -> %s reason=%s\n", state_name(st), state_name(ns), reason ? reason : "unknown");
  if (txt_log_fp) {
    fprintf(txt_log_fp, "\n[%.3f] >>> STATE: %s -> %s  reason=%s <<<\n\n", now_ms()*0.001f,
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
  sync_hover_phase(now_ms());
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
    want_arm = false;
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
  bool landed_air = have_ext && landed_state == MAV_LANDED_STATE_IN_AIR;

  bool rf_ok = rf_fresh && !isnan(rangefinder_m) && rangefinder_m > 0.08f;
  bool alt_ok = !isnan(alt_max_m) && alt_max_m > 0.08f;
  bool rise_ok = (!isnan(takeoff_alt0_m) && !isnan(alt_max_m) && (alt_max_m - takeoff_alt0_m) > 0.05f);

  // EKF landed state can flip to IN_AIR while the vehicle is still skimming
  // the ground. Require some actual altitude evidence before treating takeoff
  // as complete, otherwise we can hand off into HOVER at a few centimeters.
  return rf_ok || alt_ok || rise_ok || (landed_air && (rf_ok || alt_ok || rise_ok));
}

static bool takeoff_attitude_stable(void) {
  if (!have_att) return false;
  return fabsf(rad2deg(roll_rad)) <= TAKEOFF_HANDOFF_MAX_TILT_DEG &&
         fabsf(rad2deg(pitch_rad)) <= TAKEOFF_HANDOFF_MAX_TILT_DEG;
}

static bool takeoff_near_ground(uint64_t t) {
  bool landed_ground = have_ext && landed_state == MAV_LANDED_STATE_ON_GROUND;
  bool rf_ground = rf_fresh(t) && !isnan(rangefinder_m) && rangefinder_m < 0.06f;
  bool alt_ground = !isnan(alt_est_m) && alt_est_m < 0.08f;
  return landed_ground || rf_ground || alt_ground;
}

static bool takeoff_confidently_grounded(uint64_t t) {
  bool landed_ground = have_ext && landed_state == MAV_LANDED_STATE_ON_GROUND;
  bool rf_ground = rf_fresh(t) && !isnan(rangefinder_m) && rangefinder_m < 0.05f;
  return landed_ground || rf_ground;
}

// Check whether the takeoff brake sub-phase has settled enough for HOVER handoff.
// Returns true when vx, vy, vz, alt error, flow quality, and LPOS are all
// within thresholds and have been stable for the full dwell period.
static bool takeoff_brake_settled(uint64_t t) {
  if (!lpos_fresh(t) || !isfinite(lpos_vx_mps) || !isfinite(lpos_vy_mps) ||
      !isfinite(lpos_vz_mps)) {
    takeoff_brake_settled_since_ms = 0;
    return false;
  }

  // Lateral velocity check
  if (fabsf(lpos_vx_mps) > TAKEOFF_BRAKE_VXY_MAX_MPS ||
      fabsf(lpos_vy_mps) > TAKEOFF_BRAKE_VXY_MAX_MPS) {
    takeoff_brake_settled_since_ms = 0;
    return false;
  }

  // Vertical velocity check
  if (fabsf(lpos_vz_mps) > TAKEOFF_BRAKE_VZ_MAX_MPS) {
    takeoff_brake_settled_since_ms = 0;
    return false;
  }

  // Altitude error from hover target
  float alt_now = ceiling_control_alt_m();
  if (isnan(alt_now) || fabsf(alt_now - TAKEOFF_TARGET_M) > TAKEOFF_BRAKE_ALT_ERR_MAX_M) {
    takeoff_brake_settled_since_ms = 0;
    return false;
  }

  // Flow quality gate
  if (!of_fresh(t) || of_quality < TAKEOFF_BRAKE_MIN_FLOW_Q) {
    takeoff_brake_settled_since_ms = 0;
    return false;
  }

  // Yaw rate gate: don't settle while the drone is spinning
  if (have_att) {
    float cur_yaw = current_heading_deg();
    if (!isnan(takeoff_brake_prev_yaw_deg) && takeoff_brake_prev_yaw_ms != 0) {
      float dt_s = (t - takeoff_brake_prev_yaw_ms) * 0.001f;
      if (dt_s > 0.02f) {
        float yaw_rate = fabsf(wrap_deg(cur_yaw - takeoff_brake_prev_yaw_deg)) / dt_s;
        takeoff_brake_prev_yaw_deg = cur_yaw;
        takeoff_brake_prev_yaw_ms = t;
        if (yaw_rate > TAKEOFF_BRAKE_YAW_RATE_MAX) {
          takeoff_brake_settled_since_ms = 0;
          return false;
        }
      }
    } else {
      takeoff_brake_prev_yaw_deg = cur_yaw;
      takeoff_brake_prev_yaw_ms = t;
    }
  }

  // All checks passed — start or continue dwell
  if (takeoff_brake_settled_since_ms == 0) takeoff_brake_settled_since_ms = t;
  return (t - takeoff_brake_settled_since_ms) >= TAKEOFF_BRAKE_DWELL_MS;
}

static bool takeoff_hover_handoff_ready(uint64_t t, bool off_ground) {
  if (!off_ground) {
    takeoff_off_ground_since_ms = 0;
    return false;
  }

  if (takeoff_off_ground_since_ms == 0) {
    takeoff_off_ground_since_ms = t;
    return false;
  }

  if ((t - takeoff_off_ground_since_ms) < TAKEOFF_HANDOFF_OFFGROUND_MS) return false;
  // Do not enter hover-hold while still in extreme ground effect. The hold
  // logic assumes we are clearly airborne before it starts clamping altitude.
  float current_alt = ceiling_control_alt_m();
  if (isnan(current_alt) || current_alt < TAKEOFF_HANDOFF_MIN_ALT_M) return false;
  if (!lpos_fresh(t) || !isfinite(lpos_vz_mps) ||
      !isfinite(lpos_vx_mps) || !isfinite(lpos_vy_mps)) {
    return false;
  }
  if (lpos_vz_mps < -TAKEOFF_HANDOFF_MAX_CLIMB_MPS) return false;
  if (fabsf(lpos_vx_mps) > TAKEOFF_HANDOFF_MAX_XY_MPS ||
      fabsf(lpos_vy_mps) > TAKEOFF_HANDOFF_MAX_XY_MPS) {
    return false;
  }
  return takeoff_attitude_stable();
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

  // EKF variance (pos_horiz, vel)
  if (have_ekf) fprintf(f, "EKF(ph%.2f v%.2f) ", ekf_pos_horiz_var, ekf_vel_var);

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
    fflush(txt_log_fp);
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
  if (txt_log_fp) fprintf(txt_log_fp, "--- END PRE-FAIL DUMP ---\n");
  fflush(stdout);
  if (txt_log_fp) fflush(txt_log_fp);
}

// ----------------------------- Main control loop -----------------------
static void control_tick(void) {
  uint64_t t = now_ms();

  send_own_heartbeat_tick(t);

  update_alt_estimate();
  tof_filter_tick();
  pose_ring_push(t);
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
    goto control_tick_done;
  }

  if (!rcmap_known && (t - rcmap_last_request_ms) > 2000) {
    request_rcmap_params();
  }

  if (hard_nogo(t)) {
    printf("NO-GO: SYS_STATUS indicates gyro or motor outputs unhealthy.\n");
    if (fc_armed) enter_state(ST_DISARMING, "Health Fail");
    else enter_state(ST_IDLE, "Health Fail");
    goto control_tick_done;
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

  if (!fc_armed_prev && fc_armed) {
    start_bootstrap_grace(t, "fc_armed");
    if (txt_log_fp) {
      fprintf(txt_log_fp, "\n--- ARMED   t=%llums ---\n\n", (unsigned long long)t);
      fflush(txt_log_fp);
    }
  }
  if (fc_armed_prev && !fc_armed) {
    bootstrap_grace_until_ms = 0;
    bootstrap_grace_was_active = false;
    hover_health_fail_since_ms = 0;
    if (txt_log_fp) {
      fprintf(txt_log_fp, "\n--- DISARMED t=%llums ---\n\n", (unsigned long long)t);
      fflush(txt_log_fp);
    }
  }
  fc_armed_prev = fc_armed;

  poll_bootstrap_grace(t);
  sync_hover_phase(t);

  bool ceiling_override_pending = false;
  float ceiling_override_vz = NAN;

  // want_arm dropped while armed -> FORCE DISARM immediately (any state/mode)
  if (!want_arm && fc_armed) {
    last_disarm_cmd_ms = 0;
    disarm_fc_force();
    enter_state(ST_DISARMING, "User Abort");
    goto control_tick_done;
  }

  // Yaw drift guardian (runs before ceiling checks which skip hover_hold_tick).
  // Detects magnetometer-induced yaw drift.  Does NOT chase the drifted
  // heading — holds the latched target and forces XY re-acquire on each
  // nonfatal strike.  Lands if strikes exceed the window limit.
  if (fc_armed && have_att && have_yaw_target && (st == ST_TAKEOFF || st == ST_HOVER)) {
    float yaw_err = wrap_deg(current_heading_deg() - yaw_target_deg);
    if (fabsf(yaw_err) > YAW_DRIFT_ACCEPT_DEG && t >= yaw_spin_suppress_until_ms) {
      // Debounce: ignore if last strike was too recent
      if (yaw_drift_last_strike_ms != 0 &&
          (t - yaw_drift_last_strike_ms) < YAW_DRIFT_MIN_STRIKE_GAP_MS) {
        // Inside debounce gap — skip this tick entirely
      } else {
        float abs_err = fabsf(yaw_err);

        // Recovery detection: if error is decreasing from last strike,
        // the heading is recovering — don't escalate.
        if (yaw_drift_last_strike_err > 0.0f && abs_err < yaw_drift_last_strike_err) {
          yaw_drift_last_strike_ms = t;  // refresh debounce timer
          yaw_drift_last_strike_err = abs_err;  // track improving error
          printf("YAW_DRIFT: heading=%.1f target=%.1f err=%.1f -> recovering (prev_strike_err=%.1f, %d/%d)\n",
                 current_heading_deg(), yaw_target_deg, yaw_err,
                 yaw_drift_last_strike_err, yaw_drift_accept_count, YAW_DRIFT_MAX_ACCEPTS);
          goto control_tick_done;
        }

        // Count the strike (error is new or worsening)
        if (yaw_drift_window_start_ms == 0 ||
            (t - yaw_drift_window_start_ms) > YAW_DRIFT_WINDOW_MS) {
          yaw_drift_window_start_ms = t;
          yaw_drift_accept_count = 1;
        } else {
          yaw_drift_accept_count++;
        }
        yaw_drift_last_strike_ms = t;
        yaw_drift_last_strike_err = abs_err;

        if (yaw_drift_accept_count >= YAW_DRIFT_MAX_ACCEPTS) {
          // TEMPORARY: suppress the yaw-spin auto-land while debugging
          // torque-induced heading excursions.
          printf("YAW_DRIFT: %d strikes in %llums -> auto-land suppressed (yaw spin)\n",
                 yaw_drift_accept_count,
                 (unsigned long long)(t - yaw_drift_window_start_ms));
          yaw_drift_window_start_ms = 0;
          yaw_drift_accept_count = 0;
          yaw_drift_last_strike_ms = 0;
          yaw_drift_last_strike_err = 0.0f;
          goto control_tick_done;
        }

        printf("YAW_DRIFT: heading=%.1f target=%.1f err=%.1f -> holding latched target (%d/%d in window)\n",
               current_heading_deg(), yaw_target_deg, yaw_err,
               yaw_drift_accept_count, YAW_DRIFT_MAX_ACCEPTS);

        // Nonfatal strike: if XY-locked in hover, drop lock and force re-acquire
        if (st == ST_HOVER && hover_xy_locked) {
          hover_xy_locked = false;
          hover_xy_lock_ms = 0;
          xy_lock_candidate_active = false;
          xy_lock_candidate_since_ms = 0;
          hover_enter_ms = t;
          xy_relock_block_until_ms = t + XY_RELOCK_COOLDOWN_MS;
          if (lpos_fresh(t) && isfinite(lpos_x_m) && isfinite(lpos_y_m)) {
            hover_lock_x_m = lpos_x_m;
            hover_lock_y_m = lpos_y_m;
          }
          printf("YAW_DRIFT: XY lock dropped -> re-acquire (cooldown %llums)\n",
                 (unsigned long long)XY_RELOCK_COOLDOWN_MS);
          log_bootstrap_event(t, "XY_LOCK_BREAK", "yaw_drift_strike");
        }
        // Do NOT rewrite yaw_target_deg or hover_hold_yaw_deg
        // Skip the rest of this control tick to prevent re-acquire on same tick
        goto control_tick_done;
      }
    } else if (t < yaw_spin_suppress_until_ms) {
      // Ceiling recovery suppression — clear partial strike history so
      // a recoverable transient doesn't carry into post-recovery.
      yaw_drift_window_start_ms = 0;
      yaw_drift_accept_count = 0;
      yaw_drift_last_strike_ms = 0;
      yaw_drift_last_strike_err = 0.0f;
    } else {
      // Yaw error back below threshold — reset all drift state
      yaw_drift_last_strike_ms = 0;
      yaw_drift_last_strike_err = 0.0f;
    }
  } else {
    yaw_drift_window_start_ms = 0;
    yaw_drift_accept_count = 0;
    yaw_drift_last_strike_ms = 0;
    yaw_drift_last_strike_err = 0.0f;
  }

  // Pre-ceiling approach ramp: smoothly limit climb rate as altitude approaches
  // the ceiling trigger.  Activates earlier and more gently than the old binary
  // brake, preventing the position controller from building overshoot momentum.
  float ceiling_alt = ceiling_control_alt_m();
  const float ceiling_trigger_m_c = CEIL_M - CEIL_MARGIN_M;
  const float approach_bottom = ceiling_trigger_m_c - CEIL_APPROACH_ZONE_M;
  if (fc_armed && !ceiling_active && !isnan(ceiling_alt) && !isnan(lpos_vz_mps)
      && ceiling_alt > approach_bottom && lpos_vz_mps < 0.0f) {
    // frac: 0 at bottom of approach zone, 1 at ceiling trigger
    float frac = (ceiling_alt - approach_bottom) / CEIL_APPROACH_ZONE_M;
    if (frac > 1.0f) frac = 1.0f;
    // Allowed climb rate tapers: 0.30 m/s at zone entry → 0.05 m/s at trigger
    float max_up_mps = 0.20f * (1.0f - frac) + 0.03f * frac;
    float cur_up = -lpos_vz_mps;  // positive = climbing (NED sign flip)
    if (cur_up > max_up_mps) {
      // Command enough downward velocity to bleed off the excess climb rate
      float brake_vz = (cur_up - max_up_mps) * 1.2f + 0.05f;
      float brake_max = hover_xy_locked ? CEILING_DESCENT_MAX_MPS
                                        : CEILING_DESCENT_NOLOCK_MAX_MPS;
      if (brake_vz > brake_max) brake_vz = brake_max;
      ceiling_override_pending = true;
      ceiling_override_vz = brake_vz;
    }
  }

  // Ceiling safety: proportional descent with velocity compensation.
  // Uses reduced gain and wider taper zone to limit downward momentum,
  // so the hover controller doesn't need aggressive thrust to arrest descent.
  if (ceiling_active && fc_armed) {
    // Suppress yaw-drift strikes during ceiling descent and briefly after
    yaw_spin_suppress_until_ms = t + YAW_CEIL_SUPPRESS_MS;
    const float ceiling_trigger_m = CEIL_M - CEIL_MARGIN_M;
    const float ceiling_release_m = ceiling_trigger_m - CEIL_TAPER_DEPTH_M;
    float control_alt = ceiling_control_alt_m();
    float overshoot = (!isnan(control_alt)) ? (control_alt - CEIL_M) : 0.0f;
    if (overshoot < 0.0f) overshoot = 0.0f;
    float vz_cmd = CEILING_DESCENT_BASE_MPS + overshoot * CEILING_DESCENT_GAIN;
    // Counteract upward velocity (negative lpos_vz = climbing in NED)
    float cur_vz = isnan(lpos_vz_mps) ? 0.0f : lpos_vz_mps;
    if (cur_vz < 0.0f)
      vz_cmd += (-cur_vz);
    // Taper descent as we approach the release altitude so hover can take over smoothly
    if (!isnan(control_alt) && control_alt < ceiling_trigger_m) {
      float frac = (control_alt - ceiling_release_m) / (ceiling_trigger_m - ceiling_release_m);
      if (frac < 0.1f) frac = 0.1f;
      if (frac > 1.0f) frac = 1.0f;
      vz_cmd *= frac;
    }
    if (vz_cmd < CEILING_DESCENT_FLOOR_MPS) vz_cmd = CEILING_DESCENT_FLOOR_MPS;
    float max_vz = hover_xy_locked ? CEILING_DESCENT_MAX_MPS : CEILING_DESCENT_NOLOCK_MAX_MPS;
    if (vz_cmd > max_vz) vz_cmd = max_vz;
    ceiling_override_pending = true;
    ceiling_override_vz = vz_cmd;
  }

  if (fc_armed && (st == ST_TAKEOFF || st == ST_HOVER)) {
    HoverPhase phase = current_hover_phase();
    bool strict_hover_checks = (phase == HPH_FULL_HOVER) && !bootstrap_grace_active(t);
    uint64_t fail_limit_ms = strict_hover_checks ? FULL_HOVER_FAILSAFE_MS : BOOTSTRAP_FAILSAFE_MS;
    char reasons[160];
    bool ready = strict_hover_checks
      ? hover_ready_now(t)
      : hover_ready_now_internal(t, false, false, reasons, sizeof(reasons));

    if (strict_hover_checks && !ready) {
      hover_ready_now_internal(t, true, false, reasons, sizeof(reasons));
    }

    if (ready) {
      hover_health_fail_since_ms = 0;
    } else {
      if (hover_health_fail_since_ms == 0) hover_health_fail_since_ms = t;
      if ((t - hover_health_fail_since_ms) >= fail_limit_ms) {
        log_bootstrap_event(t, "LANDING_FAILSAFE", reasons);
        enter_state(ST_LANDING, strict_hover_checks ? "Hover Health Fail" : "Bootstrap Health Fail");
        goto control_tick_done;
      }
    }
  } else {
    hover_health_fail_since_ms = 0;
  }

  switch (st) {
    case ST_WAIT_LINK:
      enter_state(ST_IDLE, "Link OK");
      break;

    case ST_IDLE:
      if (want_arm && !arm_allowed_by_battery(t)) break;

      if (want_arm && !fc_armed) {
        if (!bootstrap_ready_stable(t)) {
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
        if (!takeoff_xy_sanity_ready(t)) break;
        enter_state(ST_TAKEOFF, "Resume Takeoff");
      }
      break;

    case ST_ARMING:
      if (!arm_allowed_by_battery(t)) {
        enter_state(ST_IDLE, "Batt Fail");
        break;
      }

      if (!bootstrap_ready_stable(t)) {
        set_mode_guided();
        break;
      }

      init_hover_targets_on_ground(t);

      if (!fc_armed) {
        set_mode_guided();
        arm_fc();
      } else {
        if (!takeoff_xy_sanity_ready(t)) break;
        enter_state(ST_TAKEOFF, "Armed");
      }
      break;

    case ST_TAKEOFF: {
      if (hb_custom_mode != 4) set_mode_guided();

      if (!takeoff_sent) {
        guided_takeoff(TAKEOFF_TARGET_M);
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
      if (off_ground) {
        takeoff_was_airborne = true;
        takeoff_grounded_since_ms = 0;
      }
      bool alt_rising = (!isnan(takeoff_alt0_m) && !isnan(alt_max_m) && (alt_max_m - takeoff_alt0_m) > 0.05f);
      bool handoff_ready = takeoff_hover_handoff_ready(t, off_ground);

      // Delay attitude thrust ramp until after NAV_TAKEOFF has time to spool and only if no rise yet.
      if (!takeoff_started && !takeoff_att_ramp_active && takeoff_sent &&
          (t - takeoff_sent_ms) > TAKEOFF_RAMP_DELAY_MS &&
          !mot_started && !alt_rising && !off_ground) {
        takeoff_att_ramp_active = true;
        takeoff_att_ramp_start_ms = t;
      }

      // Run the attitude thrust ramp while waiting for lift
      takeoff_att_ramp_tick(t);
      if (takeoff_att_ramp_active) takeoff_att_ramp_was_active = true;
      if (!takeoff_started && takeoff_att_ramp_was_active && !takeoff_att_ramp_active) {
        // If we're already airborne (or motors clearly spooled) but missed the trigger, mark started.
        bool inferred_air = (have_ext && landed_state != MAV_LANDED_STATE_ON_GROUND) ||
                            (!isnan(alt_max_m) && alt_max_m > 0.08f) ||
                            (servo_fresh && mot_avg > (TAKEOFF_MOT_START_US + 150));
        if (inferred_air) {
          takeoff_started = true;
          takeoff_started_ms = t;
          if (have_att) {
            have_yaw_target = true;
            yaw_target_deg = current_heading_deg();
          }
          start_bootstrap_grace(t, "takeoff_started");
          log_bootstrap_event(t, "TAKEOFF_START", "ramp_inferred");
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
        start_bootstrap_grace(t, "takeoff_started");
        log_bootstrap_event(t, "TAKEOFF_START", mot_started ? "motor_start" : "off_ground");
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

      if (ceiling_active) {
        takeoff_grounded_since_ms = 0;
      } else if (takeoff_was_airborne && !off_ground && takeoff_near_ground(t)) {
        if (takeoff_grounded_since_ms == 0) takeoff_grounded_since_ms = t;
        if ((t - takeoff_grounded_since_ms) >= TAKEOFF_GROUNDED_ABORT_MS) {
          bool motors_low = !servo_fresh || mot_avg < (TAKEOFF_MOT_START_US + 25.0f);
          if (motors_low && takeoff_confidently_grounded(t)) {
            printf("TAKEOFF: back on ground after liftoff -> DISARMING\n");
            want_arm = false;
            enter_state(ST_DISARMING, "Takeoff Grounded");
          } else {
            printf("TAKEOFF: back near ground after liftoff -> LANDING\n");
            enter_state(ST_LANDING, "Takeoff Recovery");
          }
          break;
        }
      } else {
        takeoff_grounded_since_ms = 0;
      }

      float takeoff_alt_now = ceiling_control_alt_m();
      bool target_height_reached = !isnan(takeoff_alt_now) && takeoff_alt_now >= (TAKEOFF_TARGET_M - 0.05f);

      // Enter brake sub-phase once height is reached or we're confirmed off-ground
      if (!takeoff_brake_active && (target_height_reached || handoff_ready)) {
        takeoff_brake_active = true;
        takeoff_brake_start_ms = t;
        takeoff_brake_settled_since_ms = 0;
        if (!have_yaw_target && have_att) {
          have_yaw_target = true;
          yaw_target_deg = current_heading_deg();
        }
        printf("TAKEOFF_BRAKE: entering brake (alt=%.2f vz=%.2f vx=%.2f vy=%.2f)\n",
               takeoff_alt_now, lpos_vz_mps, lpos_vx_mps, lpos_vy_mps);
        log_bootstrap_event(t, "TAKEOFF_BRAKE_START",
                            target_height_reached ? "target_height" : "off_ground");
      }

      // Brake sub-phase: hold Z+yaw, wait for full settling
      if (takeoff_brake_active) {
        // Send Z+yaw hold unless ceiling is already commanding descent
        if (!ceiling_active) {
          float z_down = -TAKEOFF_TARGET_M;
          float yaw = have_yaw_target ? yaw_target_deg : current_heading_deg();
          send_z_yaw_ned(z_down, yaw);
        }

        if (takeoff_brake_settled(t)) {
          printf("TAKEOFF_BRAKE: settled (%.0fms dwell, alt=%.2f vz=%.2f vx=%.2f vy=%.2f flow_q=%u) -> HOVER\n",
                 (float)TAKEOFF_BRAKE_DWELL_MS,
                 takeoff_alt_now, lpos_vz_mps, lpos_vx_mps, lpos_vy_mps,
                 (unsigned)of_quality);
          log_bootstrap_event(t, "TAKEOFF_BRAKE_DONE", "settled");
          enter_state(ST_HOVER, "Brake Settled");
          break;
        }

        // Timeout: brake never settled
        if ((t - takeoff_brake_start_ms) >= TAKEOFF_BRAKE_TIMEOUT_MS) {
          printf("TAKEOFF_BRAKE: timeout (%.1fs, vz=%.2f vx=%.2f vy=%.2f flow_q=%u) -> LANDING\n",
                 (float)(t - takeoff_brake_start_ms) * 0.001f,
                 lpos_vz_mps, lpos_vx_mps, lpos_vy_mps,
                 (unsigned)of_quality);
          log_bootstrap_event(t, "LANDING_FAILSAFE", "takeoff_brake_timeout");
          enter_state(ST_LANDING, "Brake Timeout");
          break;
        }
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

      // Continuous hover clamp: Z-only until XY lock, then full XYZ hold.
      hover_hold_tick(t);
      sync_hover_phase(t);

      if (!hover_xy_locked) {
        if (hover_enter_ms != 0 && (t - hover_enter_ms) >= PRELOCK_TIMEOUT_MS) {
          log_bootstrap_event(t, "LANDING_FAILSAFE", "prelock_timeout");
          printf("HOVER: prelock timeout (%.1fs without XY lock) -> LANDING\n",
                 (float)(t - hover_enter_ms) * 0.001f);
          enter_state(ST_LANDING, "Prelock Timeout");
          break;
        }

        if (lpos_fresh(t) && isfinite(lpos_x_m) && isfinite(lpos_y_m) &&
            isfinite(hover_lock_x_m) && isfinite(hover_lock_y_m)) {
          float dx = lpos_x_m - hover_lock_x_m;
          float dy = lpos_y_m - hover_lock_y_m;
          float drift_m = hypotf(dx, dy);
          if (drift_m >= PRELOCK_MAX_DRIFT_M) {
            log_bootstrap_event(t, "LANDING_FAILSAFE", "prelock_drift");
            printf("HOVER: prelock drift %.2fm from entry anchor -> LANDING\n", drift_m);
            enter_state(ST_LANDING, "Prelock Drift");
            break;
          }
        }
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

      // WORLD vertical descent
      send_vel_frame(0,0,+0.15f, 0, MAV_FRAME_LOCAL_NED);

      bool near_ground = (!isnan(alt_max_m) && alt_max_m < 0.10f);
      if (near_ground || (have_ext && landed_state == MAV_LANDED_STATE_ON_GROUND)) {
        want_arm = false;
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

  if (ceiling_override_pending && fc_armed &&
      (st == ST_TAKEOFF || st == ST_HOVER)) {
    send_ceiling_hold(ceiling_override_vz);
  }

control_tick_done:
  if (csv_fp && flight_log_active()) {
      bool off_ground = takeoff_off_ground(t);
      bool rf_now_fresh = rf_fresh(t);
      fprintf(csv_fp, "%llu,%s,%d,%d,%u,%.3f,%.3f,%s,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%u,%u,%u,%u,%.2f,%.2f,%.2f,%ld,%ld,%ld,%ld\n",
          (unsigned long long)t, state_name(st),
          want_arm ? 1 : 0, fc_armed ? 1 : 0, (unsigned)hb_custom_mode,
          alt_max_m, alt_est_m, alt_src_name(alt_src),
          rangefinder_m, rf_now_fresh ? 1 : 0, alt_rf_rejected ? 1 : 0,
          lpos_alt_m, lpos_alt_filt_m,
          logged_z_target_down(), logged_cmd_vz_ned(),
          ceiling_active ? 1 : 0,
          takeoff_sent ? 1 : 0,
          takeoff_started ? 1 : 0,
          off_ground ? 1 : 0,
          hover_xy_locked ? 1 : 0,
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

  signal(SIGINT,  sig_handler);
  signal(SIGTERM, sig_handler);

  log_init();

  csv_fp = fopen("flight_data.csv", "w");
  if (csv_fp) {
    fprintf(csv_fp,
            "Time_ms,State,WantArm,Armed,Mode,AltMax_m,AltEst_m,AltSrc,Rangefinder_m,RfFresh,RfRejected,"
            "LposAlt_m,LposAltFilt_m,ZTgtDown_m,CmdVzNed_mps,CeilingActive,"
            "TakeoffSent,TakeoffStarted,OffGround,HoverXYLocked,"
            "Roll_deg,Pitch_deg,Yaw_deg,Mot1,Mot2,Mot3,Mot4,VibX,VibY,VibZ,RPM1,RPM2,RPM3,RPM4\n");
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

  while (!g_shutdown) {
    struct pollfd pfd[2];
    pfd[0].fd = fc_fd;  pfd[0].events = POLLIN;
    pfd[1].fd = tof_fd; pfd[1].events = POLLIN;

    poll(pfd, 2, 20);

    if (pfd[0].revents & POLLIN) pump_fc_uart();
    if (pfd[1].revents & POLLIN) pump_tof_uart();

    control_tick();
  }

  log_close("SIGINT/SIGTERM");
  return 0;
}
