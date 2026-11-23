// Test harness: compiles the real frontier.c (main renamed) and exercises the fixed code paths.
#define main frontier_main
#include "frontier.c"
#undef main
#include <pty.h>
#include <assert.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { fails++; fprintf(stderr, "FAIL: " __VA_ARGS__); fprintf(stderr, "\n"); } else { fprintf(stderr, "ok:   " __VA_ARGS__); fprintf(stderr, "\n"); } } while (0)

// ArduCopter 4.6.3 GCS_Mavlink.cpp acceptance rule, transcribed.
static const char* ap_result(uint16_t m) {
  bool pos_ign = m & PT_IGN_POS, vel_ign = m & PT_IGN_VEL, acc_ign = m & PT_IGN_ACC;
  if (!pos_ign && !vel_ign) return "posvelaccel";
  if (pos_ign && !vel_ign) return "velaccel";
  if (pos_ign && vel_ign && !acc_ign) return "accel";
  if (!pos_ign && vel_ign && acc_ign) return "pos";
  return "INIT(REJECTED)";
}
static uint16_t pending_mask(void) {
  mavlink_set_position_target_local_ned_t p;
  mavlink_msg_set_position_target_local_ned_decode(&sp_pending, &p);
  return p.type_mask;
}
static void fresh_pose(float x, float y, float alt, float yaw_deg) {
  uint64_t t = now_ms();
  have_fc = true; last_hb_ms = t; fc_armed = true;
  have_att = true; att_last_update_ms = t; yaw_rad = deg2rad(yaw_deg); roll_rad = pitch_rad = 0;
  have_lpos = true; lpos_last_update_ms = t; lpos_x_m = x; lpos_y_m = y; lpos_alt_m = alt; lpos_alt_filt_m = alt;
  lpos_vx_mps = lpos_vy_mps = lpos_vz_mps = 0; have_xy = true;
  alt_est_m = alt; alt_src = ALT_SRC_RANGEFINDER;
}

static void test_masks(void) {
  fprintf(stderr, "\n== 1. setpoint masks vs ArduPilot 4.6.3 rule\n");
  fresh_pose(0.1f, 0.2f, 0.5f, 0);
  struct { const char* name; void (*fn)(void); } cases[] = {0};
  (void)cases;
  #define MASKCASE(label, stmt) do { sp_have_pending = false; stmt; CHECK(sp_have_pending, "%s queued", label); \
      const char* r = ap_result(pending_mask()); CHECK(strcmp(r, "INIT(REJECTED)") != 0, "%-38s -> %s", label, r); } while (0)
  st = ST_HOVER_SCAN;
  MASKCASE("send_prelock_hold (lpos fresh)", send_prelock_hold(0.1f, -0.2f));
  MASKCASE("send_pos_yaw_ned", send_pos_yaw_ned(1, 2, -0.55f, 10));
  MASKCASE("send_z_yaw_ned", send_z_yaw_ned(-0.55f, 10));
  MASKCASE("send_vel_frame", send_vel_frame(0, 0, 0.15f, 0, MAV_FRAME_LOCAL_NED));
  hover_xy_locked = true; hover_lock_x_m = 0.1f; hover_lock_y_m = 0.2f;
  MASKCASE("send_ceiling_hold (locked)", send_ceiling_hold(0.2f));
  hover_xy_locked = false;
  MASKCASE("send_ceiling_hold (unlocked)", send_ceiling_hold(0.2f));
  lpos_last_update_ms = now_ms() - 5000;   // stale LPOS -> fallback path
  MASKCASE("send_prelock_hold (lpos stale)", send_prelock_hold(0.1f, -0.2f));
  // fallback alt hold sign: drone at 0.5 m, target 0.55 -> must climb (vz NED negative)
  alt_est_m = 0.40f;
  float vz = alt_hold_vz_down(-0.55f);
  CHECK(vz < 0 && vz >= -ALT_HOLD_VZ_MAX, "alt_hold_vz_down: 0.40 m -> target 0.55 m gives climb vz=%.2f", vz);
  // Show what the OLD masks would have done
  uint16_t old_prelock = (1<<0)|(1<<1)|(1<<5)|(1<<6)|(1<<7)|(1<<8)|(1<<11);
  uint16_t old_zonly   = (1<<0)|(1<<1)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7)|(1<<8)|(1<<11);
  uint16_t old_ceil    = (1<<2)|(1<<3)|(1<<4)|(1<<6)|(1<<7)|(1<<8)|(1<<11);
  fprintf(stderr, "      (old masks: prelock->%s zonly->%s ceil_locked->%s)\n",
          ap_result(old_prelock), ap_result(old_zonly), ap_result(old_ceil));
}

static void test_link(void) {
  fprintf(stderr, "\n== 2. link writer on an undrained pty: whole frames only, bounded queue\n");
  int master, slave;
  if (openpty(&master, &slave, NULL, NULL, NULL) != 0) { fprintf(stderr, "openpty failed\n"); fails++; return; }
  struct termios tio; tcgetattr(slave, &tio); cfmakeraw(&tio); tcsetattr(slave, TCSANOW, &tio);
  fcntl(slave, F_SETFL, fcntl(slave, F_GETFL) | O_NONBLOCK);
  fc_fd = slave; fc_sysid = 1; fc_compid = 1;
  fresh_pose(0, 0, 0.5f, 0);
  fc_tx_dropped_setpoints = 0; fc_tx_failed_writes = 0;
  const int N = 6000;   // enough to fill the pty buffer (pty TIOCOUTQ reads 0, so this tests the EAGAIN path)
  for (int i = 0; i < N; i++) { send_pos_yaw_ned((float)i, 0, -0.5f, 0); flush_setpoint(); }
  int q = fc_txq_bytes();
  unsigned lost = fc_tx_dropped_setpoints + fc_tx_failed_writes;
  CHECK(lost > 0, "buffer full -> whole setpoints refused: %u of %d (dropped=%u nospace=%u)", lost, N,
        (unsigned)fc_tx_dropped_setpoints, (unsigned)fc_tx_failed_writes);
  // drain and parse every byte
  fcntl(master, F_SETFL, fcntl(master, F_GETFL) | O_NONBLOCK);
  uint8_t buf[8192]; int n, total = 0, ok_frames = 0;
  mavlink_status_t stt; memset(&stt, 0, sizeof(stt)); mavlink_message_t msg;
  while ((n = (int)read(master, buf, sizeof(buf))) > 0) {
    total += n;
    for (int i = 0; i < n; i++) if (mavlink_parse_char(MAVLINK_COMM_1, buf[i], &msg, &stt)) ok_frames++;
  }
  int sent = N - (int)lost;
  CHECK(ok_frames == sent && stt.parse_error == 0, "bytes=%d frames parsed=%d (sent %d), parse_errors=%u, txq before drain=%d B",
        total, ok_frames, sent, (unsigned)stt.parse_error, q);
  // Commands still go out whole even while the setpoint queue is full
  for (int i = 0; i < 50; i++) { send_pos_yaw_ned(0, 0, -0.5f, 0); flush_setpoint(); }
  arm_fc();
  int cmd_frames = 0; memset(&stt, 0, sizeof(stt));
  while ((n = (int)read(master, buf, sizeof(buf))) > 0)
    for (int i = 0; i < n; i++) if (mavlink_parse_char(MAVLINK_COMM_2, buf[i], &msg, &stt) && msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) cmd_frames++;
  CHECK(cmd_frames == 1 && stt.parse_error == 0, "ARM command delivered intact behind a full setpoint queue (cmd frames=%d, parse_errors=%u)", cmd_frames, (unsigned)stt.parse_error);
  close(master); close(slave); fc_fd = -1;
}

// --- scanlog replay (same record layout as log_scan_record) ---
static void test_map(const char* path, double t_start, double t_end, float ox, float oy) {
  fprintf(stderr, "\n== 3. C mapping/selection on real scans: %s [%.2f, %.2f]\n", path, t_start, t_end);
  FILE* f = fopen(path, "rb"); if (!f) { fprintf(stderr, "cannot open %s\n", path); fails++; return; }
  static uint8_t data[1<<20]; size_t len = fread(data, 1, sizeof(data), f); fclose(f);
  nav_mode = NAV_MODE_FRONTIER_BASIC;
  frontier_reset_map(ox, oy);
  int used = 0; float lx = 0, ly = 0, lyaw = 0;
  for (size_t i = 0; i + sizeof(scanrec_t) <= len; i++) {
    if (memcmp(data + i, "SCN3", 4) != 0) continue;
    scanrec_t r; memcpy(&r, data + i, sizeof(r)); i += sizeof(r) - 1;
    double ts = r.host_ms / 1000.0;
    if (ts < t_start || ts > t_end) continue;
    fresh_pose(r.x_m, r.y_m, r.alt_m, r.yaw_deg);
    roll_rad = r.roll_rad; pitch_rad = r.pitch_rad;
    usleep(1500);
    uint64_t t = now_ms();
    pose_ring_push(t);
    memcpy(last_scan_grid_raw, r.grid_raw, sizeof(last_scan_grid_raw));
    have_scan_frame = true; last_scan_host_ms = t;
    frontier_update_map_from_latest_scan();
    used++; lx = r.x_m; ly = r.y_m; lyaw = r.yaw_deg;
  }
  int nfree = 0, nocc = 0;
  for (int y = 0; y < FRONTIER_GRID_H; y++) for (int x = 0; x < FRONTIER_GRID_W; x++) {
    OccupancyCell c = frontier_cell_value(x, y); if (c == OCC_FREE) nfree++; else if (c == OCC_OCCUPIED) nocc++; }
  fprintf(stderr, "      scans=%d free=%d occ=%d bbox x[%d..%d] y[%d..%d]\n", used, nfree, nocc,
          frontier_bb_x0, frontier_bb_x1, frontier_bb_y0, frontier_bb_y1);
  fresh_pose(lx, ly, 0.5f, lyaw);
  FrontierCluster c; char why[160];
  uint64_t t0 = now_ms();
  bool ok = frontier_select_target(lx, ly, &c, why, sizeof(why));
  uint64_t dt = now_ms() - t0;
  if (ok) {
    float d = hypotf(c.target_x_m - lx, c.target_y_m - ly);
    CHECK(d >= FRONTIER_ARRIVAL_TOL_M, "target (%.2f,%.2f) step=%.2f m from pose (%.2f,%.2f), cluster=%d cells, select took %llu ms",
          c.target_x_m, c.target_y_m, d, lx, ly, c.cell_count, (unsigned long long)dt);
  } else {
    fprintf(stderr, "      no target: %s (select took %llu ms)\n", why, (unsigned long long)dt);
  }
}

static void test_yaw_sweep(void) {
  fprintf(stderr, "\n== 4. HOVER_SCAN yaw sweep state machine (FC heading follows target)\n");
  fresh_pose(0, 0, 0.55f, 10.0f);
  nav_mode = NAV_MODE_FRONTIER_BASIC;
  st = ST_HOVER;            // so enter_state(HOVER_SCAN) runs entry logic
  enter_state(ST_HOVER_SCAN, "test");
  hover_xy_locked = true; hover_lock_x_m = 0; hover_lock_y_m = 0; hover_yaw_latched = true;
  have_yaw_target = true; yaw_target_deg = 10.0f;
  float start = yaw_target_deg, max_step = 0, prev = yaw_target_deg;
  uint64_t t0 = now_ms();
  while (now_ms() - t0 < 6000 && frontier_scan_yaw_phase != 2) {
    fresh_pose(0, 0, 0.55f, yaw_target_deg);   // FC tracks yaw instantly
    frontier_tick(now_ms());
    float step = fabsf(wrap_deg(yaw_target_deg - prev)); if (step > max_step) max_step = step; prev = yaw_target_deg;
    sp_have_pending = false;
    usleep(CONTROL_PERIOD_MS * 1000);
  }
  CHECK(frontier_scan_yaw_phase == 2, "sweep finished (phase=%d) in %.1fs", frontier_scan_yaw_phase, (now_ms() - t0) / 1000.0);
  CHECK(fabsf(wrap_deg(yaw_target_deg - start)) > 44.0f && max_step <= FRONTIER_SCAN_YAW_STEP_DEG + 0.01f && max_step < YAW_DRIFT_ACCEPT_DEG,
        "rotated %.1f deg, largest single step %.1f deg (< guardian %.0f deg)", wrap_deg(yaw_target_deg - start), max_step, YAW_DRIFT_ACCEPT_DEG);
  CHECK(frontier_scan_yaw_dir == -1, "next sweep reverses direction (dir=%d)", frontier_scan_yaw_dir);
}

int main(void) {
  txt_log_fp = NULL; log_fp = NULL;
  test_masks();
  test_link();
  const char* d = getenv("SCANDIR");
  char p[512];
  snprintf(p, sizeof(p), "%s/4_frontierTest_scanlog.bin", d); test_map(p, 38.875, 48.874, 0.11f, -0.16f);
  snprintf(p, sizeof(p), "%s/3_hallWay_scanlog.bin", d);      test_map(p, 129.823, 152.893, 0.54f, 0.12f);
  test_yaw_sweep();
  fprintf(stderr, "\n%s (%d failures)\n", fails ? "FAILED" : "ALL CHECKS PASSED", fails);
  return fails ? 1 : 0;
}
