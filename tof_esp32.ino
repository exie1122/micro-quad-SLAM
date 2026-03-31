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

static volatile bool     g_haveCmd = false;
static volatile uint8_t  g_cmd     = 0;
static volatile uint32_t g_seq     = 0;

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

uint16_t getCell(const VL53L5CX_ResultsData &m, int row, int col) {
  int x = 7 - col;
  return m.distance_mm[x + row * 8];
}

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
  if (g_haveCmd) {
    Serial.println("Recieved Arm Command!");
    uint8_t  cmd = g_cmd;
    uint32_t seq = g_seq;
    g_haveCmd = false;
    forwardCmdToLuckfox(cmd, seq);
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!ok[i]) continue;
    bindBus(i);
    if (tof[i].isDataReady()) {
      tof[i].getRangingData(&dataArr[i]);
    }
  }

  Serial.println("---- SENSOR STATUS ----");

  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("Sensor %d: ", i);

    if (!ok[i]) {
      Serial.println("NOT INITIALIZED");
      continue;
    }

    bindBus(i);

    if (tof[i].isDataReady()) {
      Serial.println("DATA READY");
    } else {
      Serial.println("NO DATA");
    }
  }

  Serial.println("-----------------------");

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

  SBC.write(txbuf, SCAN_BYTES);

  delay(80);
}
