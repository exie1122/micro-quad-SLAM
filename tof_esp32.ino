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

// ----------------- Physical orientation -----------------
enum Dir { FRONT = 0, RIGHT, BACK, LEFT };

// Physical direction → wiring index
const int dirToSensor[4] = {
  3,  // FRONT -> S4
  1,  // RIGHT -> S2
  2,  // BACK  -> S3
  0   // LEFT  -> S1
};

// ----------------- I2C bus pins -----------------
struct BusPins { uint8_t sda; uint8_t scl; };

BusPins pins[NUM_SENSORS] = {
  {2,  1},   // S1 (left)
  {4,  3},   // S2 (right)
  {12, 13},  // S3 (back)
  {11, 10}   // S4 (front)
};

// ----------------- UART ToF Packet (unchanged) -----------------
#define SCAN_HEADER 0xA5
#define SCAN_BYTES  (1 + 4 + (TOTAL_CELLS * 2) + 1) // 518
static uint8_t txbuf[SCAN_BYTES];

// ----------------- UART Control Frame to Luckfox -----------------
#define CTRL_HEADER 0xA6
#define CTRL_BYTES  7

enum : uint8_t { CMD_DISARM = 0, CMD_ARM = 1 };

struct __attribute__((packed)) ArmMsg {
  uint8_t  magic;   // 0xC3
  uint8_t  cmd;     // 0=DISARM, 1=ARM
  uint32_t seq;
  uint32_t t_ms;
};

static constexpr uint8_t ESPNOW_CHANNEL = 1;

static volatile bool     g_haveCmd = false;
static volatile uint8_t  g_cmd     = 0;
static volatile uint32_t g_seq     = 0;

// ----------------- Globals -----------------
SparkFun_VL53L5CX tof[NUM_SENSORS];
VL53L5CX_ResultsData dataArr[NUM_SENSORS];
bool ok[NUM_SENSORS] = {false, false, false, false};

HardwareSerial SBC(1);

// ----------------- Utilities -----------------
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

// SparkFun orientation correction
uint16_t getCell(const VL53L5CX_ResultsData &m, int row, int col) {
  int x = 7 - col;
  return m.distance_mm[x + row * 8];
}

// ----------------- ESP-NOW -----------------
static void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  if (len != (int)sizeof(ArmMsg)) return;

  ArmMsg m;
  memcpy(&m, data, sizeof(m));
  if (m.magic != 0xC3) return;
  if (m.cmd != CMD_ARM && m.cmd != CMD_DISARM) return;

  g_cmd = m.cmd;
  g_seq = m.seq;
  g_haveCmd = true;
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

// Forward command to Luckfox over UART3 stream (same as ToF stream)
static void forwardCmdToLuckfox(uint8_t cmd, uint32_t seq) {
  uint8_t b[CTRL_BYTES];
  b[0] = CTRL_HEADER;
  b[1] = cmd;
  wr_u32_le(&b[2], seq);
  b[6] = checksum8(b, 6);
  SBC.write(b, CTRL_BYTES);
}

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);

  // Initialize WiFi FIRST
  WiFi.mode(WIFI_STA);
  delay(100);   // <-- critical on ESP32-S3

  // Print ONLY own MAC once (guaranteed correct)
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  SBC.begin(115200, SERIAL_8N1, TRANSMIT_RX, TRANSMIT_TX);

  espnowInit();

  // Init sensors (no printing)
  for (int i = 0; i < NUM_SENSORS; i++) {
    bindBus(i);
    if (!tof[i].begin()) {
      ok[i] = false;
      continue;
    }
    tof[i].setResolution(64);
    tof[i].setRangingFrequency(10);
    tof[i].startRanging();
    ok[i] = true;
  }
}

// ----------------- Main loop -----------------
void loop() {
  // 0) If new ESP-NOW cmd arrived, forward to Luckfox immediately
  if (g_haveCmd) {
    Serial.println("Recieved Arm Command!");
    uint8_t  cmd = g_cmd;
    uint32_t seq = g_seq;
    g_haveCmd = false;
    forwardCmdToLuckfox(cmd, seq);
  }

  // 1) Snapshot all sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!ok[i]) continue;
    bindBus(i);
    if (tof[i].isDataReady()) {
      tof[i].getRangingData(&dataArr[i]);
    }
  }

  // 2) Build fixed 518-byte packet in physical order: FRONT → RIGHT → BACK → LEFT
  memset(txbuf, 0, sizeof(txbuf));
  txbuf[0] = SCAN_HEADER;

  uint32_t tms = millis();
  wr_u32_le(&txbuf[1], tms);

  int off = 5;
  for (int d = 0; d < 4; d++) {
    int s = dirToSensor[d];
    for (int r = 0; r < 8; r++) {
      for (int c = 0; c < 8; c++) {
        uint16_t v = ok[s] ? getCell(dataArr[s], r, c) : 0xFFFF;
        wr_u16_le(&txbuf[off], v);
        off += 2;
      }
    }
  }

  txbuf[SCAN_BYTES - 1] = checksum8(txbuf, SCAN_BYTES - 1);

  // 3) Publish ToF frame
  SBC.write(txbuf, SCAN_BYTES);

  delay(80);
}
