#include "M5CoreS3.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>

// ================= ESP-NOW CONFIG =================
static uint8_t PEER_MAC[6] = { 0x9C, 0x13, 0x9E, 0xF4, 0x27, 0x0C };
static constexpr uint8_t ESPNOW_CHANNEL = 1;

enum : uint8_t { CMD_DISARM = 0, CMD_ARM = 1 };

struct __attribute__((packed)) ArmMsg {
  uint8_t  magic;
  uint8_t  cmd;
  uint32_t seq;
  uint32_t t_ms;
};

static uint32_t seqNo = 0;

// ================= COLORS =================
static constexpr uint16_t C_BG     = 0x0000;
static constexpr uint16_t C_TEXT   = 0xFFFF;
static constexpr uint16_t C_MUTED  = 0x7BEF;
static constexpr uint16_t C_GREEN  = 0x07E0;
static constexpr uint16_t C_RED    = 0xF800;
static constexpr uint16_t C_YELLOW = 0xFFE0;
static constexpr uint16_t C_DARK   = 0x18E3;

// ================= UI =================
struct Rect {
  int x,y,w,h;
  bool contains(int px,int py) const {
    return px>=x && px<x+w && py>=y && py<y+h;
  }
};

static Rect R_ARM, R_DISARM, R_STATUS, R_BATT;

// ================= SEND FEEDBACK =================
static volatile bool sendCbFired = false;
static volatile bool lastSendOk  = false;

static void onDataSent(const wifi_tx_info_t*, esp_now_send_status_t status) {
  lastSendOk  = (status == ESP_NOW_SEND_SUCCESS);
  sendCbFired = true;
}

// ================= BATTERY =================
static float readBatteryVoltageV() {
  float raw = (float)CoreS3.Power.getBatteryVoltage();
  return (raw > 20.0f) ? raw / 1000.0f : raw;
}

static int readBatteryPercent() {
  return (int)CoreS3.Power.getBatteryLevel();
}

static void drawBattery(bool force=false) {
  static int lastPct=-1, lastVx100=-1;

  int pct = readBatteryPercent();
  int vx  = (int)(readBatteryVoltageV()*100);

  if (!force && pct==lastPct && vx==lastVx100) return;
  lastPct=pct; lastVx100=vx;

  CoreS3.Display.fillRect(R_BATT.x, R_BATT.y, R_BATT.w, R_BATT.h, C_BG);

  char buf[24];
  snprintf(buf,sizeof(buf),"%d%% %.2fV",pct,vx/100.0f);

  CoreS3.Display.setTextDatum(top_right);
  CoreS3.Display.setTextSize(2);
  CoreS3.Display.setTextColor(C_MUTED, C_BG);
  CoreS3.Display.drawString(buf,R_BATT.x+R_BATT.w,R_BATT.y+2);
}

// ================= DRAW HELPERS =================
static void fillRound(const Rect&r,uint16_t c){ CoreS3.Display.fillRoundRect(r.x,r.y,r.w,r.h,22,c); }
static void drawRound(const Rect&r,uint16_t c){ CoreS3.Display.drawRoundRect(r.x,r.y,r.w,r.h,22,c); }

static void drawTitle() {
  CoreS3.Display.setTextDatum(top_left);
  CoreS3.Display.setTextColor(C_TEXT,C_BG);
  CoreS3.Display.setTextSize(2);
  CoreS3.Display.setCursor(18,14);
  CoreS3.Display.print("ARM / DISARM");

  drawBattery(true);
  CoreS3.Display.drawFastHLine(16,56,CoreS3.Display.width()-32,C_DARK);
}

static void drawStatus(const char* msg,uint16_t col) {
  CoreS3.Display.fillRect(R_STATUS.x,R_STATUS.y,R_STATUS.w,R_STATUS.h,C_BG);
  CoreS3.Display.setTextDatum(middle_left);
  CoreS3.Display.setTextSize(2);
  CoreS3.Display.setTextColor(col,C_BG);
  CoreS3.Display.drawString(msg,18,R_STATUS.y+R_STATUS.h/2);
}

// ================= BUTTONS =================
static constexpr int ARM_BAR_H = 12;
static constexpr int ARM_BAR_PAD = 18;

static void drawButton(
  const Rect& r,
  uint16_t base,
  const char* label,
  bool pressed,
  bool isArm = false,
  int armPct = 0
) {
  const int RADIUS = 22;
  const int PAD    = 14;
  const int BAR_H  = 12;
  const int GAP    = 8;

  int dx = pressed ? 2 : 0;
  int dy = pressed ? 2 : 0;

  // Background
  CoreS3.Display.fillRoundRect(r.x, r.y, r.w, r.h, RADIUS, C_BG);
  CoreS3.Display.fillRoundRect(r.x+dx, r.y+dy, r.w, r.h, RADIUS, base);
  CoreS3.Display.drawRoundRect(r.x+dx, r.y+dy, r.w, r.h, RADIUS, C_TEXT);

  // ---------- TEXT ZONE ----------
  int textTop = r.y + PAD;
  int textBottom = r.y + r.h - PAD;

  if (isArm) {
    // reserve space for progress bar
    textBottom -= (BAR_H + GAP);
  }

  int textY = (textTop + textBottom) / 2;

  CoreS3.Display.setTextDatum(middle_center);
  CoreS3.Display.setTextSize(3);
  CoreS3.Display.setTextColor(C_TEXT, base);
  CoreS3.Display.drawString(label, r.x + r.w / 2 + dx, textY + dy);

  // ---------- ARM PROGRESS BAR ----------
  if (isArm) {
    int barW = r.w - PAD * 2;
    int barX = r.x + PAD + dx;
    int barY = r.y + r.h - PAD - BAR_H + dy;

    CoreS3.Display.fillRoundRect(barX, barY, barW, BAR_H, 6, C_DARK);

    int fw = (barW * armPct) / 100;
    if (fw > 0) {
      CoreS3.Display.fillRoundRect(barX, barY, fw, BAR_H, 6, C_YELLOW);
    }
  }
}


// ================= ESP-NOW =================
static void espnowInit(){
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL,WIFI_SECOND_CHAN_NONE);
  esp_now_init();
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t p{};
  memcpy(p.peer_addr,PEER_MAC,6);
  p.channel=ESPNOW_CHANNEL;
  p.encrypt=false;
  p.ifidx=WIFI_IF_STA;
  esp_now_add_peer(&p);
}

static void sendCmd(uint8_t cmd){
  ArmMsg m{0xC3,cmd,++seqNo,millis()};
  sendCbFired=false;
  esp_now_send(PEER_MAC,(uint8_t*)&m,sizeof(m));
  drawStatus(cmd==CMD_ARM?"Sending ARM...":"Sending DISARM...",C_YELLOW);
}

// ================= TOUCH =================
static bool touching=false, armTriggered=false;
static int sx,sy,lastX,lastY;
static uint32_t holdMs;

static constexpr uint32_t ARM_HOLD_MS = 650;

void setup(){
  CoreS3.begin();
  CoreS3.Display.fillScreen(C_BG);

  int W=CoreS3.Display.width();
  int H=CoreS3.Display.height();

  R_BATT={W-180,10,165,24};
  R_STATUS={0,H-56,W,56};

  int pad=20;
  int btnH=(H-140)/2;
  R_ARM   ={pad,70,W-pad*2,btnH};
  R_DISARM={pad,70+btnH+18,W-pad*2,btnH};

  espnowInit();
  drawTitle();
  drawButton(R_ARM,C_GREEN,"ARM",false,true,0);
  drawButton(R_DISARM,C_RED,"DISARM",false);
  drawStatus("READY",C_MUTED);
}

void loop(){
  CoreS3.update();
  drawBattery(false);

  bool nowTouch=CoreS3.Touch.getCount()>0;
  if (nowTouch){
    auto t=CoreS3.Touch.getDetail(0);
    lastX=t.x; lastY=t.y;
  }

  if (nowTouch && !touching){
    touching=true; armTriggered=false;
    sx=lastX; sy=lastY; holdMs=millis();
  }

  if (nowTouch && touching && R_ARM.contains(sx,sy)){
    int pct=min(100,(int)((millis()-holdMs)*100/ARM_HOLD_MS));
    drawButton(R_ARM,C_GREEN,"ARM",true,true,pct);
    if (!armTriggered && pct>=100){
      armTriggered=true;
      sendCmd(CMD_ARM);
    }
  }

  if (!nowTouch && touching){
    touching=false;
    drawButton(R_ARM,C_GREEN,"ARM",false,true,0);
    drawButton(R_DISARM,C_RED,"DISARM",false);
    if (R_DISARM.contains(sx,sy) && R_DISARM.contains(lastX,lastY))
      sendCmd(CMD_DISARM);
  }

  if (sendCbFired){
    sendCbFired=false;
    drawStatus(lastSendOk?"SENT OK":"SEND FAILED",lastSendOk?C_GREEN:C_RED);
  }

  delay(10);
}
