// uav_local_nav.c
// MVP: Pose ingestion (FC), ToF ingestion (ESP32), local occupancy grid, reactive nav + simple exploration.
// Deterministic memory. No heap. No ROS. No OpenCV.

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <math.h>

#include "common/mavlink.h"

/* ===================== CONFIG ===================== */

// FC MAVLink UART
#define FC_UART_DEV   "/dev/ttyS4"
#define FC_UART_BAUD  B57600

// ESP32 ToF UART (CHANGE THIS to whatever you wired)
#define TOF_UART_DEV  "/dev/ttyS3"
#define TOF_UART_BAUD B921600  // choose what you actually use

#define MY_SYSID   255
#define MY_COMPID  MAV_COMP_ID_ONBOARD_COMPUTER

// Control loop rates
#define CTRL_HZ         20
#define MAP_HZ          10
#define HEARTBEAT_HZ    1

// Local grid (rolling window) — small + deterministic
#define GRID_W 160
#define GRID_H 160
#define GRID_RES_M 0.10f   // 10cm per cell -> 16m x 16m window

// Occupancy values
// 0=unknown, 1=free, 2=occupied
static uint8_t grid[GRID_H][GRID_W];

// Robot radius + obstacle inflate (in meters)
#define ROBOT_RADIUS_M  0.25f
#define INFLATE_M       0.20f

// ToF model
#define TOF_ROWS 8
#define TOF_COLS 8
#define TOF_POINTS (TOF_ROWS*TOF_COLS)
#define TOF_MAX_M 4.0f
#define TOF_MIN_M 0.15f

// Exploration / avoidance gains
#define V_FWD_MAX   1.0f     // m/s
#define V_SIDE_MAX  0.6f     // m/s
#define YAW_RATE_MAX 0.8f    // rad/s

#define AVOID_GAIN  1.5f
#define TURN_GAIN   1.0f

// If "front arc" has obstacle closer than this, treat as blocked
#define FRONT_BLOCK_M 1.2f

/* ===================== TIME ===================== */

static uint64_t now_us(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* ===================== UART ===================== */

static int open_uart(const char *dev, speed_t baud) {
  int fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) { perror("open_uart"); return -1; }

  struct termios tio;
  memset(&tio, 0, sizeof(tio));
  if (tcgetattr(fd, &tio) != 0) { perror("tcgetattr"); close(fd); return -1; }

  cfsetispeed(&tio, baud);
  cfsetospeed(&tio, baud);

  tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

  tio.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
  tio.c_lflag &= ~(ICANON | ECHO | ISIG);
  tio.c_oflag &= ~OPOST;

  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 1; // 0.1s timeout

  if (tcsetattr(fd, TCSANOW, &tio) != 0) { perror("tcsetattr"); close(fd); return -1; }
  return fd;
}

/* ===================== MAVLink TX ===================== */

static void mav_write(int fd, const mavlink_message_t *msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  (void)write(fd, buf, len);
}

static void send_heartbeat(int fd) {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(
    MY_SYSID, MY_COMPID, &msg,
    MAV_TYPE_ONBOARD_CONTROLLER,
    MAV_AUTOPILOT_INVALID,
    0, 0, MAV_STATE_ACTIVE
  );
  mav_write(fd, &msg);
}

static void request_msg_rate(int fd, uint8_t sys, uint8_t comp, uint32_t msgid, float hz) {
  mavlink_message_t msg;
  float interval_us = (hz > 0.0f) ? (1000000.0f / hz) : -1.0f;
  mavlink_msg_command_long_pack(
    MY_SYSID, MY_COMPID, &msg,
    sys, comp,
    MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    (float)msgid,
    interval_us,
    0,0,0,0,0
  );
  mav_write(fd, &msg);
}

// Velocity control in LOCAL_NED (body-frame variant exists too, but start simple).
static void send_vel_ned(int fd, uint8_t sys, uint8_t comp,
                        float vx, float vy, float vz, float yaw_rate) {
  // SET_POSITION_TARGET_LOCAL_NED
  // type_mask bits: ignore position, accel, yaw; use velocity + yaw_rate
  // https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
  const uint16_t IGNORE_PX   = (1<<0);
  const uint16_t IGNORE_PY   = (1<<1);
  const uint16_t IGNORE_PZ   = (1<<2);
  const uint16_t IGNORE_AFX  = (1<<6);
  const uint16_t IGNORE_AFY  = (1<<7);
  const uint16_t IGNORE_AFZ  = (1<<8);
  const uint16_t FORCE_SET   = (1<<9);
  const uint16_t IGNORE_YAW  = (1<<10);

  uint16_t type_mask = IGNORE_PX | IGNORE_PY | IGNORE_PZ |
                       IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ |
                       FORCE_SET | IGNORE_YAW;

  mavlink_message_t msg;
  mavlink_msg_set_position_target_local_ned_pack(
    MY_SYSID, MY_COMPID, &msg,
    0, // time_boot_ms (ok as 0)
    sys, comp,
    MAV_FRAME_LOCAL_NED,
    type_mask,
    0,0,0,         // x,y,z ignored
    vx, vy, vz,     // velocities used
    0,0,0,         // accel ignored
    0.0f,          // yaw ignored
    yaw_rate
  );
  mav_write(fd, &msg);
}

/* ===================== POSE ===================== */

typedef struct {
  float x, y, z;       // NED (meters)
  float vx, vy, vz;    // NED (m/s)
  float yaw;           // radians
  uint64_t t_us;
  bool have_xy;
  bool have_yaw;
} pose_t;

static pose_t cur_pose;

/* ===================== ToF packet (ESP32 -> Luckfox) ===================== */
/*
  You must adapt this to your real packet format.

  MVP packet format:
    uint16_t sync = 0xA55A
    uint16_t len  = 8*8*2 + 8  (or total bytes after header)
    uint32_t t_us_low (optional)
    uint16_t dist_mm[64]
    uint16_t crc16 (optional)  -- ignored here

  We'll implement a minimal framed parser:
    find sync, read fixed payload = 2+2+4 + 64*2 = 136 bytes total (no crc)
*/
#define TOF_SYNC 0xA55A
#define TOF_PKT_BYTES (2+2+4 + TOF_POINTS*2)

typedef struct {
  uint16_t dist_mm[TOF_POINTS];
  uint64_t t_us_rx;
  bool valid;
} tof_frame_t;

static tof_frame_t last_tof;

/* ===================== GRID helpers ===================== */

static void grid_clear(void) {
  memset(grid, 0, sizeof(grid));
}

// Convert world (NED x,y) to grid cell indices in rolling window centered at pose
static bool world_to_cell(float wx, float wy, const pose_t *p, int *cx, int *cy) {
  // rolling window center = current pose (x,y)
  float dx = wx - p->x;
  float dy = wy - p->y;

  int ix = (int)lroundf(dx / GRID_RES_M) + (GRID_W/2);
  int iy = (int)lroundf(dy / GRID_RES_M) + (GRID_H/2);

  if (ix < 0 || ix >= GRID_W || iy < 0 || iy >= GRID_H) return false;
  *cx = ix; *cy = iy;
  return true;
}

static void set_cell(int cx, int cy, uint8_t v) {
  if ((unsigned)cx < GRID_W && (unsigned)cy < GRID_H) {
    // occupied dominates free dominates unknown
    if (v == 2) grid[cy][cx] = 2;
    else if (v == 1 && grid[cy][cx] == 0) grid[cy][cx] = 1;
  }
}

// Simple Bresenham raycast marking free along ray, occupied at end
static void raycast_free_then_occ(int x0,int y0,int x1,int y1) {
  int dx = abs(x1-x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1-y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  int x = x0, y = y0;
  while (1) {
    if (x == x1 && y == y1) break;
    set_cell(x,y,1); // free
    int e2 = 2*err;
    if (e2 >= dy) { err += dy; x += sx; }
    if (e2 <= dx) { err += dx; y += sy; }
    if ((unsigned)x >= GRID_W || (unsigned)y >= GRID_H) return;
  }
  set_cell(x1,y1,2); // occupied endpoint
}

/* ===================== ToF -> points in body frame ===================== */
/*
  You must know your ToF sensor orientation. For MVP:
  - Treat 8x8 as a forward-facing depth grid.
  - Approximate each cell as a ray with fixed angular offsets.

  We'll define a simple pinhole-ish model:
    horizontal FOV ~ 45 deg, vertical FOV ~ 45 deg (adjust to VL53L5CX real FOV later)
*/
#define TOF_HFOV_RAD (45.0f * (float)M_PI/180.0f)
#define TOF_VFOV_RAD (45.0f * (float)M_PI/180.0f)

// For each cell, compute a unit direction in sensor frame (forward = +X, left = +Y)
static void tof_cell_dir(int r, int c, float *dx, float *dy) {
  float u = ((float)c + 0.5f) / (float)TOF_COLS; // 0..1
  float v = ((float)r + 0.5f) / (float)TOF_ROWS; // 0..1
  float ax = (u - 0.5f) * TOF_HFOV_RAD;
  // ignore vertical for 2D mapping; use forward distance only.
  // Approx: project onto ground plane using horizontal angle only.
  *dx = cosf(ax);
  *dy = sinf(ax);
}

/* ===================== MAP update ===================== */

static void integrate_tof_into_grid(const pose_t *p, const tof_frame_t *tf) {
  if (!p->have_xy || !p->have_yaw || !tf->valid) return;

  // Sensor origin assumed at drone position projected to map.
  int x0 = GRID_W/2;
  int y0 = GRID_H/2;

  // For each ToF cell, compute endpoint in world and raycast
  for (int r=0; r<TOF_ROWS; r++) {
    for (int c=0; c<TOF_COLS; c++) {
      int idx = r*TOF_COLS + c;
      float d_m = tf->dist_mm[idx] * 0.001f;
      if (d_m < TOF_MIN_M || d_m > TOF_MAX_M) continue;

      float bx, by;
      tof_cell_dir(r,c,&bx,&by); // body/sensor frame ray (2D)

      // rotate by yaw into world
      float cy = cosf(p->yaw), sy = sinf(p->yaw);
      float wx_dir = bx*cy - by*sy;
      float wy_dir = bx*sy + by*cy;

      float end_x = p->x + wx_dir * d_m;
      float end_y = p->y + wy_dir * d_m;

      int x1,y1;
      if (!world_to_cell(end_x,end_y,p,&x1,&y1)) continue;

      // Raycast from center of grid
      raycast_free_then_occ(x0,y0,x1,y1);
    }
  }
}

/* ===================== NAV: obstacle avoidance + simple exploration ===================== */

// Return min distance in "front arc" based on last ToF (very cheap)
static float front_min_m(const tof_frame_t *tf) {
  if (!tf->valid) return 999.0f;
  // use middle rows, middle cols as "front"
  int cols_mid0 = 3, cols_mid1 = 4;
  float best = 999.0f;
  for (int r=2; r<=5; r++) {
    for (int c=cols_mid0; c<=cols_mid1; c++) {
      float d = tf->dist_mm[r*TOF_COLS + c] * 0.001f;
      if (d >= TOF_MIN_M && d <= TOF_MAX_M && d < best) best = d;
    }
  }
  return best;
}

// Compute a repulsive lateral/yaw signal from ToF asymmetry (no grid needed)
static void avoidance_cmd(const tof_frame_t *tf, float *out_vx, float *out_vy, float *out_yawrate) {
  *out_vx = V_FWD_MAX;
  *out_vy = 0.0f;
  *out_yawrate = 0.0f;

  if (!tf->valid) return;

  // Compute left vs right "pressure"
  float left = 0.0f, right = 0.0f;
  for (int r=2; r<=5; r++) {
    for (int c=0; c<TOF_COLS; c++) {
      float d = tf->dist_mm[r*TOF_COLS + c] * 0.001f;
      if (d < TOF_MIN_M || d > TOF_MAX_M) continue;
      float w = 1.0f / fmaxf(d, 0.25f); // closer = higher
      if (c < 4) left += w; else right += w;
    }
  }

  float bias = (right - left); // positive -> more obstacle on left => turn right
  *out_vy = fmaxf(fminf(bias * 0.15f, V_SIDE_MAX), -V_SIDE_MAX);
  *out_yawrate = fmaxf(fminf(bias * 0.35f, YAW_RATE_MAX), -YAW_RATE_MAX);

  // If front is blocked, slow forward and turn harder
  float fmin = front_min_m(tf);
  if (fmin < FRONT_BLOCK_M) {
    float s = fmin / FRONT_BLOCK_M; // 0..1
    if (s < 0.0f) s = 0.0f;
    *out_vx = V_FWD_MAX * s * 0.4f; // slow a lot
    *out_yawrate = fmaxf(fminf(bias * 0.8f + (bias>=0?0.4f:-0.4f), YAW_RATE_MAX), -YAW_RATE_MAX);
  }
}

/* ===================== ToF UART parser ===================== */

static bool read_tof_frame(int fd, tof_frame_t *out) {
  // Minimal sync+fixed length parser. Deterministic.
  static uint8_t buf[TOF_PKT_BYTES];
  static int have = 0;

  uint8_t tmp[256];
  int n = (int)read(fd, tmp, sizeof(tmp));
  if (n <= 0) return false;

  for (int i=0; i<n; i++) {
    // shift in byte
    if (have < TOF_PKT_BYTES) buf[have++] = tmp[i];

    // if buffer has at least 2 bytes, check sync at start
    if (have >= 2) {
      uint16_t sync = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
      if (sync != TOF_SYNC) {
        // drop first byte, shift left by 1
        memmove(buf, buf+1, (size_t)(have-1));
        have -= 1;
        continue;
      }
    }

    // full packet?
    if (have == TOF_PKT_BYTES) {
      // parse
      uint16_t sync = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
      uint16_t len  = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
      (void)sync;
      (void)len;

      uint32_t t_low = (uint32_t)buf[4] |
                       ((uint32_t)buf[5] << 8) |
                       ((uint32_t)buf[6] << 16) |
                       ((uint32_t)buf[7] << 24);
      (void)t_low; // optional: could be used for debugging alignment

      for (int k=0; k<TOF_POINTS; k++) {
        int off = 8 + k*2;
        out->dist_mm[k] = (uint16_t)buf[off] | ((uint16_t)buf[off+1] << 8);
      }
      out->t_us_rx = now_us();
      out->valid = true;

      have = 0;
      return true;
    }
  }

  return false;
}

/* ===================== MAVLink RX ===================== */

static void process_fc_bytes(const uint8_t *bytes, int n,
                             mavlink_message_t *rx, mavlink_status_t *st,
                             uint8_t *t_sys, uint8_t *t_comp, bool *got_hb,
                             bool *printed_hb) {
  for (int i=0; i<n; i++) {
    if (mavlink_parse_char(MAVLINK_COMM_0, bytes[i], rx, st)) {
      if (rx->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        *t_sys = rx->sysid;
        *t_comp = rx->compid;
        *got_hb = true;
        if (!*printed_hb) {
          printf("HB sys=%u comp=%u\n", *t_sys, *t_comp);
          fflush(stdout);
          *printed_hb = true;
        }
      }

      // Pose sources (you MUST enable/request these on ArduPilot)
      if (rx->msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(rx, &att);
        cur_pose.yaw = att.yaw;        // radians
        cur_pose.have_yaw = true;
        cur_pose.t_us = now_us();
      }

      if (rx->msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
        mavlink_local_position_ned_t lp;
        mavlink_msg_local_position_ned_decode(rx, &lp);
        cur_pose.x = lp.x;
        cur_pose.y = lp.y;
        cur_pose.z = lp.z;
        cur_pose.vx = lp.vx;
        cur_pose.vy = lp.vy;
        cur_pose.vz = lp.vz;
        cur_pose.have_xy = true;
        cur_pose.t_us = now_us();
      }
    }
  }
}

/* ===================== MAIN ===================== */

int main(void) {
  int fc_fd  = open_uart(FC_UART_DEV, FC_UART_BAUD);
  if (fc_fd < 0) return 1;

  int tof_fd = open_uart(TOF_UART_DEV, TOF_UART_BAUD);
  if (tof_fd < 0) {
    printf("Warning: ToF UART not opened; mapping/nav will be blind.\n");
    fflush(stdout);
  }

  grid_clear();
  memset(&cur_pose, 0, sizeof(cur_pose));
  last_tof.valid = false;

  mavlink_status_t st;
  mavlink_message_t rx;

  uint8_t target_sys = 0, target_comp = 0;
  bool got_hb = false, printed_hb = false;
  bool requested = false;

  uint64_t last_hb = 0, last_ctrl = 0, last_map = 0;

  while (1) {
    // --- FC RX ---
    uint8_t fc_buf[256];
    int n = (int)read(fc_fd, fc_buf, sizeof(fc_buf));
    if (n > 0) {
      process_fc_bytes(fc_buf, n, &rx, &st, &target_sys, &target_comp, &got_hb, &printed_hb);
    }

    uint64_t t = now_us();

    // --- periodic heartbeat ---
    if (t - last_hb >= (1000000ULL / HEARTBEAT_HZ)) {
      send_heartbeat(fc_fd);
      last_hb = t;
    }

    if (!got_hb) continue;

    // --- request message rates once ---
    if (!requested) {
      // You can adjust rates; keep modest.
      request_msg_rate(fc_fd, target_sys, target_comp, MAVLINK_MSG_ID_ATTITUDE, 50.0f);
      request_msg_rate(fc_fd, target_sys, target_comp, MAVLINK_MSG_ID_LOCAL_POSITION_NED, 30.0f);
      requested = true;
    }

    // --- ToF RX (optional) ---
    if (tof_fd >= 0) {
      tof_frame_t tf;
      if (read_tof_frame(tof_fd, &tf)) {
        last_tof = tf;
      }
    }

    // --- MAP update ---
    if (t - last_map >= (1000000ULL / MAP_HZ)) {
      integrate_tof_into_grid(&cur_pose, &last_tof);
      last_map = t;
    }

    // --- CONTROL loop ---
    if (t - last_ctrl >= (1000000ULL / CTRL_HZ)) {
      float vx, vy, yaw_rate;
      avoidance_cmd(&last_tof, &vx, &vy, &yaw_rate);

      // If you want purely local NED motion, this is fine.
      // If you prefer BODY frame commands, switch MAV_FRAME and transform velocities.
      send_vel_ned(fc_fd, target_sys, target_comp, vx, vy, 0.0f, yaw_rate);

      last_ctrl = t;
    }
  }

  return 0;
}
