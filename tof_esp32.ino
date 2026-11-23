// Post-mortem fixes (see ../analysis/REPORT.md):
//  - Zones whose target_status is not a valid range (5, or 9 = valid with large pulse) are sent
//    as 0xFFFF. Previously every zone's distance_mm was sent even when the sensor flagged it
//    invalid, and the companion mapped those as obstacles.
//  - A frame is sent only when at least one sensor produced new data, without the fixed
//    delay(80) and the per-loop status printout (which re-bound every I2C bus a second time).
//    Measured frame rate before: ~5 Hz (196 ms) against a 10 Hz ranging setting.
//  - ESP-NOW commands go through a small queue so two frames arriving 8 ms apart are both
//    forwarded (the single g_cmd slot could overwrite one).
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <HardwareSerial.h>

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define I2C_FREQ 400000

#define TRANSMIT_TX 8
#define TRANSMIT_RX 7

#define NUM_SENSORS 4
#define GRID_SIZE   64
#define TOTAL_CELLS (NUM_SENSORS * GRID_SIZE)

enum Dir { FRONT = 0, RIGHT, BACK, LEFT };

const int dirToSensor[4] = {
  3,
  1,
  2,
  0
};

struct BusPins { uint8_t sda; uint8_t scl; };

BusPins pins[NUM_SENSORS] = {
  {4, 3},
  {2, 1},
  {12, 13},
  {6, 5}
};

#define SCAN_HEADER 0xA5
#define SCAN_BYTES  (1 + 4 + (TOTAL_CELLS * 2) + 1)
static uint8_t txbuf[SCAN_BYTES];

#define CTRL_HEADER 0xA6
#define CTRL_BYTES  7

enum : uint8_t { CMD_DISARM = 0, CMD_ARM = 1 };

struct __attribute__((packed)) ArmMsg {
  uint8_t  magic;
  uint8_t  cmd;
  uint32_t seq;
  uint32_t t_ms;
};

static constexpr uint8_t ESPNOW_CHANNEL = 1;

// Small single-producer (ESP-NOW callback) / single-consumer (loop) queue.
#define CMD_Q_LEN 8
static volatile uint8_t  g_qCmd[CMD_Q_LEN];
static volatile uint32_t g_qSeq[CMD_Q_LEN];
static volatile uint8_t  g_qHead = 0;   // written by callback
static volatile uint8_t  g_qTail = 0;   // written by loop
static portMUX_TYPE      g_qMux = portMUX_INITIALIZER_UNLOCKED;

static bool g_newData[4] = {false, false, false, false};

SparkFun_VL53L5CX tof[NUM_SENSORS];
VL53L5CX_ResultsData dataArr[NUM_SENSORS];
bool ok[NUM_SENSORS] = {false, false, false, false};

HardwareSerial SBC(1);

static uint8_t checksum8(const uint8_t *buf, size_t len) {
  uint8_t c = 0;
  for (size_t i = 0; i < len; i++) c ^= buf[i];
  return c;
}

static inline void wr_u32_le(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline void wr_u16_le(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

void bindBus(int i) {
  Wire.end();
  delayMicroseconds(200);
  Wire.begin(pins[i].sda, pins[i].scl);
  Wire.setClock(I2C_FREQ);
  delayMicroseconds(200);
}

// VL53L5CX target_status: 5 = valid, 9 = valid (large pulse). Everything else (no target,
// sigma/phase failures, wrap-around, ...) is unreliable and reported as 0xFFFF.
static inline bool zoneValid(const VL53L5CX_ResultsData &m, int zone) {
  if (m.nb_target_detected[zone] == 0) return false;
  uint8_t st = m.target_status[zone * VL53L5CX_NB_TARGET_PER_ZONE];
  return st == 5 || st == 9;
}

uint16_t getCell(const VL53L5CX_ResultsData &m, int row, int col) {
  int x = 7 - col;                 // ST returns zones mirrored; flip so col 0 = left (from behind)
  int zone = x + row * 8;
  if (!zoneValid(m, zone)) return 0xFFFF;
  int16_t d = m.distance_mm[zone * VL53L5CX_NB_TARGET_PER_ZONE];
  return (d <= 0) ? 0xFFFF : (uint16_t)d;
}

static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  if (len != (int)sizeof(ArmMsg)) return;

  ArmMsg m;
  memcpy(&m, data, sizeof(m));
  if (m.magic != 0xC3) return;
  if (m.cmd != CMD_ARM && m.cmd != CMD_DISARM) return;

  portENTER_CRITICAL(&g_qMux);       // ESP-NOW callbacks run in the WiFi task, not an ISR
  uint8_t next = (uint8_t)((g_qHead + 1) % CMD_Q_LEN);
  if (next != g_qTail) {           // drop if full (8 pending commands is already abnormal)
    g_qCmd[g_qHead] = m.cmd;
    g_qSeq[g_qHead] = m.seq;
    g_qHead = next;
  }
  portEXIT_CRITICAL(&g_qMux);
}

static void espnowInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  esp_now_init();
  esp_now_register_recv_cb(onEspNowRecv);
}

static void forwardCmdToLuckfox(uint8_t cmd, uint32_t seq) {
  uint8_t b[CTRL_BYTES];
  b[0] = CTRL_HEADER;
  b[1] = cmd;
  wr_u32_le(&b[2], seq);
  b[6] = checksum8(b, 6);
  SBC.write(b, CTRL_BYTES);
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  delay(100);

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  SBC.begin(115200, SERIAL_8N1, TRANSMIT_RX, TRANSMIT_TX);

  espnowInit();

  Serial.println("=== SENSOR INIT START ===");

  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("Sensor %d -> SDA=%d SCL=%d\n", i, pins[i].sda, pins[i].scl);

    bindBus(i);

    if (!tof[i].begin()) {
      ok[i] = false;
      Serial.println("INIT FAILED");
      continue;
    }

    Serial.println("INIT OK");

    tof[i].setResolution(64);
    tof[i].setRangingFrequency(10);
    tof[i].startRanging();

    ok[i] = true;
  }

  Serial.println("=== SENSOR INIT DONE ===");
}

void loop() {
  // Forward every queued ESP-NOW command (ARM/DISARM are sent twice by the M5Stack and the
  // companion requires two matching frames).
  for (;;) {
    uint8_t cmd = 0; uint32_t seq = 0; bool have = false;
    portENTER_CRITICAL(&g_qMux);
    if (g_qTail != g_qHead) {
      cmd = g_qCmd[g_qTail]; seq = g_qSeq[g_qTail];
      g_qTail = (uint8_t)((g_qTail + 1) % CMD_Q_LEN);
      have = true;
    }
    portEXIT_CRITICAL(&g_qMux);
    if (!have) break;
    Serial.println("Received command");
    forwardCmdToLuckfox(cmd, seq);
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!ok[i]) continue;
    bindBus(i);
    if (tof[i].isDataReady()) {
      tof[i].getRangingData(&dataArr[i]);
      g_newData[i] = true;
    }
  }

  // Send a frame once every initialised sensor has new data (or after 150 ms so one slow or
  // failed sensor can't stall the others). Sensors without data since the last frame are sent
  // as 0xFFFF so the companion never maps an old frame at a new pose.
  // If no sensor has new data, send nothing: the companion's scan_fresh() then goes stale and
  // its failsafes act, instead of being fed an all-invalid "fresh" frame.
  static uint32_t lastSendMs = 0;
  bool allNew = true, someNew = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!ok[i]) continue;
    if (g_newData[i]) someNew = true; else allNew = false;
  }
  uint32_t nowMs = millis();
  bool send = someNew && (allNew || (nowMs - lastSendMs) >= 150);
  if (!send) { delay(2); return; }
  lastSendMs = nowMs;

  memset(txbuf, 0, sizeof(txbuf));
  txbuf[0] = SCAN_HEADER;
  wr_u32_le(&txbuf[1], nowMs);

  int off = 5;
  for (int d = 0; d < 4; d++) {
    int s = dirToSensor[d];
    bool fresh = ok[s] && g_newData[s];
    for (int r = 0; r < 8; r++) {
      for (int c = 0; c < 8; c++) {
        uint16_t v = fresh ? getCell(dataArr[s], r, c) : 0xFFFF;
        wr_u16_le(&txbuf[off], v);
        off += 2;
      }
    }
  }
  for (int i = 0; i < NUM_SENSORS; i++) g_newData[i] = false;

  txbuf[SCAN_BYTES - 1] = checksum8(txbuf, SCAN_BYTES - 1);
  SBC.write(txbuf, SCAN_BYTES);
}
