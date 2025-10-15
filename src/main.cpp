#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <FS.h>
#include <Wire.h>
#include <ctype.h>
#include <WiFiClientSecure.h>
#include <time.h>            // 提供 configTime() 與 getLocalTime() 用於 NTP 時間同步
#include <U8g2lib.h>         // OLED 顯示函式庫 (U8g2)
#include <ESP32Ping.h>       // 新增：用來檢測指定 IP 是否被佔用
#include <vector>
// --- forward declarations ---
// --- forward declarations ---
static inline void oledKick(const char* why);   // ← 改成 static inline
// ====== Telegram Keyboard Hide Timer ======
static unsigned long gKbHideAt = 0;   // 0 = 不倒數，>0 = millis() 到期自動關鍵盤
// ====== UI：即時訊息（兩行）與 WiFi 條形 ======
struct UiNote {
  char line1[22];      // 128 寬配 6x10/7x13 字體約 20~21 字
  char line2[22];
  unsigned long until; // ms：到期就自動隱藏
} gUi = {{0},{0},0};

static inline void uiShow(const String& a, const String& b="", uint32_t ms=3500){
  strncpy(gUi.line1, a.c_str(), sizeof(gUi.line1)-1);
  strncpy(gUi.line2, b.c_str(), sizeof(gUi.line2)-1);
  gUi.until = millis() + ms;
  oledKick("uiShow");  // ★ 顯示提示訊息時，順便喚醒 OLED
}

static int rssiBars(long rssi){
  // 大約門檻：>= -55:4格, >= -65:3, >= -75:2, >= -85:1, 其餘 0
  if (rssi >= -55) return 4;
  if (rssi >= -65) return 3;
  if (rssi >= -75) return 2;
  if (rssi >= -85) return 1;
  return 0;
}

// =========================【OLED 顯示設定】=========================
// 使用 I2C OLED 顯示模組（SSD1306 128x64）
// SDA=21, SCL=22（ESP32 預設 I2C 腳位）
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0, /* reset=*/ U8X8_PIN_NONE /* 使用 Wire 預設 SCL/SDA 腳位 */
);
// =========================【OLED 螢幕保護（可調）】=========================
static uint32_t      gOledSleepMs   = 60000;      // 60s
static unsigned long gOledLastKick  = 0;          // 最近活動時間
static bool          gOledPowerSave = false;      // 是否已省電關閉

// 活動喚醒：任何事件呼叫即可；若在睡眠中會立即點亮
static inline void oledKick(const char* why ){
  gOledLastKick = millis();
  if (gOledPowerSave){
    u8g2.setPowerSave(0);
    gOledPowerSave = false;
  }
}

// 全域函式宣告
void drawOled(bool forceSetup);      // 顯示 OLED 畫面（是否強制顯示設定模式）
static inline void tgEnqueue(const String& s);  // 推播訊息加入佇列

// =========================【繼電器控制區】=========================
// 6 路繼電器 GPIO 腳位設定（避免佔用 I2C 的 21/22 腳）
static const bool RELAY_ACTIVE_HIGH = true;   // true=HIGH 啟動繼電器
const int RELAY_PINS[] = { 12, 13, 14, 26, 27, 32 };

// 繼電器數量（自動由陣列大小決定）
#ifndef RELAY_COUNT
  #define RELAY_COUNT (sizeof(RELAY_PINS)/sizeof(RELAY_PINS[0]))
#endif
//在裝置端加一顆「⚙️ 開啟設定」鍵
#ifndef WEBAPP_URL
  #define WEBAPP_URL "https://lemel0501.github.io/YQ-webapp/"  // 你的 GitHub Pages
#endif

// 每一路繼電器的非阻塞測試狀態
static bool          gTestActive[RELAY_COUNT] = {false};     // 是否正在測試
static unsigned long gTestUntil [RELAY_COUNT] = {0};         // 測試結束時間 (millis)
static unsigned long gTestStart[RELAY_COUNT] = {0};          // 測試開始時間

// =========================【異常 DI 輸入區】=========================
// 6 路數位輸入 (DI) 腳位，用於偵測異常訊號，低有效 (INPUT_PULLUP)
static const int  ALARM_COUNT = 6;
const  int ALARM_PINS[ALARM_COUNT] = { 16, 17, 18, 19, 23, 25 };
bool gAlarmLatched[ALARM_COUNT] = {0,0,0,0,0,0};

// 輸入去抖與前一次狀態紀錄
static bool gAlarmLast[ALARM_COUNT] = {1,1,1,1,1,1};         // 上次輸入狀態 (1=未觸發)
static unsigned long gAlarmDebounceMs[ALARM_COUNT] = {0,0,0,0,0,0};
static const unsigned long ALARM_DEBOUNCE = 40;              // 去抖時間 (ms)

// =========================【工件計數器區】=========================
// 設定 2 組計數器，GPIO36 與 GPIO39（低有效，需要外部10k上拉到3.3V）
static const int CNT_COUNT = 2;
const int CNT_PINS[CNT_COUNT] = {4, 15};         // ⚠ 目前使用 GPIO4 / GPIO15
static const bool CNT_ACTIVE_LOW = true;         // 訊號為低有效

// 計數狀態相關全域變數
volatile uint32_t gCntAck[CNT_COUNT] = {0,0};    // 推播成功後需清除的計數量
static const unsigned long CNT_DEBOUNCE = 20;    // ms 去抖時間
static const uint32_t      CNT_MIN_US   = 3000;  // μs 兩次觸發最短間隔 (避免抖動)

// 狀態追蹤用變數
static int  gCntLast[CNT_COUNT] = {1,1};         // 上次輸入狀態
static unsigned long gCntDeb[CNT_COUNT] = {0,0}; // 去抖計時
static uint32_t gCount[CNT_COUNT] = {0,0};       // 計數器累積數值
static int16_t  lastCntKey[CNT_COUNT] = {-1,-1}; // 上次按鍵索引
static bool gCntArmed[CNT_COUNT] = {false,false}; // 是否允許下一次計數
static unsigned long gCntWarmupUntil = 0;        // 啟動後暖機時間
volatile uint32_t gCntIsr[CNT_COUNT] = {0,0};    // 中斷計數
volatile uint32_t gCntLastUs[CNT_COUNT] = {0,0}; // 上次觸發時間戳 (us)
volatile bool gCntResetReq[CNT_COUNT] = {false,false}; // 是否請求重置
static uint32_t gCntShown[2] = {0,0};            // OLED 顯示用的數值



// =========================【計數器中斷服務程式 (ISR)】=========================
// 工件計數 #0 的 ISR
void IRAM_ATTR cnt0_isr(){
  uint32_t now = micros();
  if (gCntArmed[0] && (now - gCntLastUs[0] > CNT_MIN_US)) {
    gCntLastUs[0] = now;
    gCntIsr[0]++;          // 計數加 1 (快照)
    gCntArmed[0] = false;  // 立即去武裝，必須等回 HIGH 才能再次計數
  }
}

// 工件計數 #1 的 ISR
void IRAM_ATTR cnt1_isr(){
  uint32_t now = micros();
  if (gCntArmed[1] && (now - gCntLastUs[1] > CNT_MIN_US)) {
    gCntLastUs[1] = now;
    gCntIsr[1]++;
    gCntArmed[1] = false;  // 同上：避免抖動誤計數
  }
}

// =========================【異常 DI 推播訊息】=========================
// 預設異常輸入訊息，可在設定頁修改
String gAlarmMsg[ALARM_COUNT] = {
  "異常CH1", "異常CH2", "異常CH3", "異常CH4", "異常CH5", "異常CH6"
};

// =========================【應用設定資料結構】=========================
// 繼電器排程設定
struct Sched {
  uint8_t  hh = 8, mm = 0;   // 時間 (HH:MM)
  uint32_t hold = 3;         // 保持秒數
  String   msg = "Relay!";   // 推播訊息
};

// 工件計數設定
struct CounterCfg {
  uint8_t hh = 8, mm = 0;    // 每日推播時間 (HH:MM)
  String  msg = "工件計數";   // 推播訊息
  uint32_t target = 0;       // 達標數量 (0=停用達標推播)
};

// 系統設定主結構
struct AppConfig {
  String ssid, pass;           // WiFi 帳號密碼
  String token, chat;          // Telegram token & chat ID
  Sched  sch[RELAY_COUNT];     // 各繼電器排程
  String aMsg[ALARM_COUNT];    // 異常 DI 訊息
  uint8_t wdMask = 0x7F;       // 星期遮罩 (bit0=Mon … bit6=Sun，預設全開)
  CounterCfg cnt[2];           // 兩組工件計數器設定
} cfg;


static void tgSendControlKeyboard(const String& chatId){
  String kb = F(
    "{\"keyboard\":["
      "[{\"text\":\"⚙️ 開啟設定\",\"web_app\":{\"url\":\"https://lemel0501.github.io/YQ-webapp/\"}}],"
      "[{\"text\":\"/check_token\"}]"
    "],"
    "\"resize_keyboard\":true,"
    "\"one_time_keyboard\":true"   // ← 按一次自動收起，避免長駐造成「像舊的沒變」
    "}"
  );

  WiFiClientSecure cli; cli.setInsecure();
  if (!cli.connect("api.telegram.org", 443)) return;
  String body = String("{\"chat_id\":\"")+chatId+"\",\"text\":\"已送出設定鍵盤。\",\"reply_markup\":"+kb+"}";
  String req  = "POST /bot"+cfg.token+"/sendMessage HTTP/1.1\r\nHost: api.telegram.org\r\nContent-Type: application/json\r\nContent-Length: "+String(body.length())+"\r\nConnection: close\r\n\r\n"+body;
  cli.print(req);
    // ★ 發完鍵盤後，啟動 10 秒自動關閉倒數
    gKbHideAt = millis() + 10000UL;
}

// 送出可「內嵌開啟 WebApp」的 inline keyboard 按鈕
static void tgSendInlineOpen(const String& chatId){
  WiFiClientSecure cli; cli.setInsecure();
  if (!cli.connect("api.telegram.org", 443)) return;

  String kb = F(
    "{\"inline_keyboard\":["
      "[{\"text\":\"⚙️ 開啟設定 (WebApp)\",\"web_app\":{\"url\":\"https://lemel0501.github.io/YQ-webapp/\"}}]"
    "]}"
  );

  String body = String("{\"chat_id\":\"")+chatId+
                "\",\"text\":\"點按下方按鈕開啟設定頁：\",\"reply_markup\":"+kb+"}";
  String req = "POST /bot"+cfg.token+"/sendMessage HTTP/1.1\r\n"
               "Host: api.telegram.org\r\n"
               "Content-Type: application/json\r\n"
               "Content-Length: "+String(body.length())+"\r\n"
               "Connection: close\r\n\r\n"+body;
  cli.print(req);
}

// 關閉 Telegram 鍵盤（remove_keyboard）
static void tgHideKeyboard(const String& chatId){
  WiFiClientSecure cli; cli.setInsecure();
  if (!cli.connect("api.telegram.org", 443)) return;

  String body = String("{\"chat_id\":\"")+chatId+
                "\",\"text\":\"✅ 已關閉鍵盤。\",\"reply_markup\":{\"remove_keyboard\":true}}";
  String req = "POST /bot"+cfg.token+"/sendMessage HTTP/1.1\r\n"
               "Host: api.telegram.org\r\n"
               "Content-Type: application/json\r\n"
               "Content-Length: "+String(body.length())+"\r\n"
               "Connection: close\r\n\r\n"+body;
  cli.print(req);
}




// =========================【工具函式區】=========================
// URL 編碼工具 (for Telegram)
String urlEncode(const String& s){
  String o; char buf[4];
  for (size_t i=0;i<s.length();++i){
    unsigned char c = (unsigned char)s[i];
    if (isalnum(c) || c=='-'||c=='_'||c=='.'||c=='~') o += (char)c;
    else { snprintf(buf,sizeof(buf),"%%%02X",c); o += buf; }
  }
  return o;
}

// 繼電器結束訊息
static inline String endMsg(int ch){
  return cfg.sch[ch].msg + "關閉";
}

// =========================【Telegram 傳送訊息】=========================
// 嚴格檢查 200 OK 與 ok:true
bool sendTelegram(const String& text){
  if (!WiFi.isConnected()) { Serial.println("[TG] WiFi not connected"); return false; }
  if (!cfg.token.length() || !cfg.chat.length()) { Serial.println("[TG] token/chat empty"); return false; }

  WiFiClientSecure cli; cli.setInsecure();
  if (!cli.connect("api.telegram.org", 443)) { Serial.println("[TG] connect fail"); return false; }

  // POST 請求組合
  String body = "chat_id=" + urlEncode(cfg.chat) + "&text=" + urlEncode(text);
  String req  = "POST /bot" + cfg.token + "/sendMessage HTTP/1.1\r\n";
  req += "Host: api.telegram.org\r\n";
  req += "Content-Type: application/x-www-form-urlencoded\r\n";
  req += "Content-Length: " + String(body.length()) + "\r\n";
  req += "Connection: close\r\n\r\n" + body;

  cli.print(req);

  // 等待回應 (最多 5 秒)
  unsigned long t0 = millis();
  while (!cli.available() && millis()-t0 < 5000) delay(10);
  if (!cli.available()) { Serial.println("[TG] no response"); return false; }

  // 解析 HTTP 狀態列
  String status = cli.readStringUntil('\n');  
  status.trim();
  bool http200 = status.indexOf(" 200 ") > 0;

  // 跳過 header
  while (cli.connected()) {
    String h = cli.readStringUntil('\n');
    if (h == "\r" || h.length()==0) break;
  }
  // 讀取 body
  String resp;
  while (cli.available()) resp += (char)cli.read();
  cli.stop();

  bool okField = (resp.indexOf("\"ok\":true") >= 0);
  if (!http200 || !okField) {
    Serial.println("[TG] send fail");
    Serial.println(status);
    Serial.println(resp);
    return false;
  }
  return true;
}


// ===== 讀取 WebApp 回傳（web_app_data）的輕量輪詢 =====
static long tgUpdateOffset = 0; // 供 getUpdates 去重

// =========================【Telegram 非阻塞佇列任務】=========================
// 推播訊息結構
struct TgMsg { char text[256]; };
static QueueHandle_t tgQ = nullptr;

// 佇列推送（ISR 外不可直接 send，必須用 enqueue）
static inline void tgEnqueue(const String& s){
  if (!tgQ || !s.length()) return;
  TgMsg m{}; strncpy(m.text, s.c_str(), sizeof(m.text)-1);
  xQueueSend(tgQ, &m, 0);
}

// 任務：負責實際送信 + 重試機制 (不影響主迴圈)
static void tgTask(void*){
  const TickType_t base = pdMS_TO_TICKS(400);
  for(;;){
    TgMsg m{};
    if (xQueueReceive(tgQ, &m, portMAX_DELAY) == pdTRUE){
      Serial.print("[TG] dequeued: "); Serial.println(m.text);
      bool sent = false;
      for (int attempt=0; attempt<3 && !sent; ++attempt){
        sent = sendTelegram(String(m.text));
        if (!sent) { Serial.printf("[TG] retry %d\n", attempt+1); vTaskDelay(base * (attempt + 1)); }
      }
      Serial.println(sent ? "[TG] ok" : "[TG] failed");
    }
  }
}

// =========================【系統參數設定】=========================
static const uint32_t MAX_HOLD_SEC = 3600;  // 繼電器保持上限 (1 小時)
static const uint32_t MIN_HOLD_SEC = 1;     // 繼電器保持下限 (1 秒)

static const int  AP_MODE_PIN   = 33;       // AP 模式觸發腳位
static const bool AP_ACTIVE_LOW = true;     // LOW = 啟用 AP

#define RTC_IMPL_PCF8563        // 使用 PCF8563 RTC
// #define RTC_IMPL_DS3231       // (可切換為 DS3231)

// I2C 與 RTC 設定
static const int I2C_SDA    = 21;
static const int I2C_SCL    = 22;
static const int RTC_INT_PIN = 5;           // RTC 中斷腳位 (選用)

// 狀態旗標
static bool gOnlineNotifiedOnce = false;    // 上線通知僅一次
static unsigned long gCloseApAt = 0;        // 延遲關閉 AP 時間
static bool gRtcReady = false;              // RTC 是否準備好

// 繼電器啟動 (帶保持秒數)
static inline void startRelayTimed(int ch, uint32_t holdSec) {
  if (ch < 0 || ch >= RELAY_COUNT) return;
  if (holdSec == 0) holdSec = 1;

  int pin = RELAY_PINS[ch];

  if (gTestActive[ch]) {
    // 若已啟動 → 延長保持時間
    unsigned long addMs = holdSec * 1000UL;
    gTestUntil[ch] += addMs;
    tgEnqueue("CH" + String(ch+1) + " 正在保持中，依排程延長 " + String(holdSec) + " 秒");
    return;
  }

  digitalWrite(pin, RELAY_ACTIVE_HIGH ? HIGH : LOW);
  gTestActive[ch] = true;
  gTestStart[ch]  = millis();
  gTestUntil[ch]  = millis() + holdSec * 1000UL;
}

WebServer srv(80);   // 建立 WebServer (HTTP port 80)

// 停止指定繼電器，並送出原因訊息
static inline void stopRelayIfActive(int ch, const char* reason){
  if (ch < 0 || ch >= RELAY_COUNT) return;
  if (!gTestActive[ch]) return;
  int pin = RELAY_PINS[ch];
  digitalWrite(pin, RELAY_ACTIVE_HIGH ? LOW : HIGH);
  gTestActive[ch] = false;
  tgEnqueue("CH" + String(ch+1) + " 測試結束(" + String(reason ? reason : "中止") + ")");
  uiShow("CH"+String(ch+1)+" 停止", reason?reason:"中止");
}

// =========================【RTC 抽象介面定義】=========================
struct YqDateTime {
  int year, month, day, hour, minute, second;
};

class IRtc {
public:
  virtual bool begin() = 0;                       // 初始化
  virtual bool lostPower() = 0;                   // 是否掉電
  virtual void adjust(const YqDateTime& dt) = 0;  // 設定時間
  virtual YqDateTime now() = 0;                   // 讀取時間
  virtual bool setDailyAlarm(uint8_t hh, uint8_t mm) = 0; // 設定每日鬧鐘
  virtual bool clearAlarmFlag() = 0;              // 清除鬧鐘旗標
  virtual ~IRtc() {}
};


// =========================【RTC 實作：PCF8563T】=========================
#ifdef RTC_IMPL_PCF8563
#include <RTClib.h>   // 僅使用 DateTime 型別（若不需要可移除）

// PCF8563 I2C 位址
static const uint8_t PCF8563_ADDR = 0x51;

// BCD 轉換工具
uint8_t dec2bcd(uint8_t v){ return ((v/10)<<4) | (v%10); }
uint8_t bcd2dec(uint8_t v){ return ((v>>4)*10) + (v&0x0F); }

// PCF8563 RTC 類別實作 (繼承 IRtc 抽象介面)
class RtcPCF8563 : public IRtc {
public:
  bool begin() override {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.beginTransmission(PCF8563_ADDR);
    return (Wire.endTransmission() == 0);   // 傳回是否可連線
  }

  bool lostPower() override {
    // PCF8563 沒有停電旗標，用秒暫存器 bit7 (VL) 判斷
    uint8_t sec = readReg(0x02);
    return (sec & 0x80);  // 1=時間資料失效
  }

  void adjust(const YqDateTime& dt) override {
    // 設定年月日時分秒
    Wire.beginTransmission(PCF8563_ADDR);
    Wire.write(0x02); // 秒起始
    Wire.write(dec2bcd(dt.second) & 0x7F);
    Wire.write(dec2bcd(dt.minute) & 0x7F);
    Wire.write(dec2bcd(dt.hour)   & 0x3F);
    Wire.write(dec2bcd(dt.day)    & 0x3F);
    Wire.write(0x01); // weekday 暫設為 1
    Wire.write(dec2bcd(dt.month)  & 0x1F);
    Wire.write(dec2bcd(dt.year % 100));  // 只存兩位數
    Wire.endTransmission();
  }

  YqDateTime now() override {
    // 讀取當前時間
    YqDateTime t{};
    Wire.beginTransmission(PCF8563_ADDR);
    Wire.write(0x02);
    Wire.endTransmission(false);
    Wire.requestFrom(PCF8563_ADDR, (uint8_t)7);

    uint8_t ss = Wire.read();
    uint8_t mm = Wire.read();
    uint8_t hh = Wire.read();
    uint8_t d  = Wire.read();
    Wire.read(); // weekday (略過)
    uint8_t mo = Wire.read();
    uint8_t yy = Wire.read();

    t.second = bcd2dec(ss & 0x7F);
    t.minute = bcd2dec(mm & 0x7F);
    t.hour   = bcd2dec(hh & 0x3F);
    t.day    = bcd2dec(d  & 0x3F);
    t.month  = bcd2dec(mo & 0x1F);
    t.year   = 2000 + bcd2dec(yy);

    return t;
  }

  bool setDailyAlarm(uint8_t hh, uint8_t mm) override {
    // 設定每日鬧鐘，只比對分與小時
    writeReg(0x09, dec2bcd(mm) & 0x7F);
    writeReg(0x0A, dec2bcd(hh) & 0x3F);
    writeReg(0x0B, 0x80);   // 關閉日比對
    writeReg(0x0C, 0x80);   // 關閉週比對

    // 啟用 Alarm Interrupt Enable
    uint8_t ctl2 = readReg(0x01);
    ctl2 |= 0x02;  // AIE=1
    writeReg(0x01, ctl2);
    return true;
  }

  bool clearAlarmFlag() override {
    // 清除 AF (Alarm Flag)
    uint8_t ctl2 = readReg(0x01);
    ctl2 &= ~0x08;
    writeReg(0x01, ctl2);
    return true;
  }

private:
  // I2C 讀寫封裝
  uint8_t readReg(uint8_t r){
    Wire.beginTransmission(PCF8563_ADDR);
    Wire.write(r);
    Wire.endTransmission(false);
    Wire.requestFrom(PCF8563_ADDR, (uint8_t)1);
    return Wire.read();
  }
  void writeReg(uint8_t r, uint8_t v){
    Wire.beginTransmission(PCF8563_ADDR);
    Wire.write(r);
    Wire.write(v);
    Wire.endTransmission();
  }
};
#endif


// =========================【RTC 實作：DS3231】=========================
#ifdef RTC_IMPL_DS3231
#include <RTClib.h>
RTC_DS3231 ds;

// DS3231 RTC 類別實作 (繼承 IRtc 抽象介面)
class RtcDS3231Wrap : public IRtc {
public:
  bool begin() override {
    Wire.begin(I2C_SDA, I2C_SCL);
    return ds.begin();
  }

  bool lostPower() override {
    return ds.lostPower();
  }

  void adjust(const YqDateTime& dt) override {
    ds.adjust(DateTime(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second));
  }

  YqDateTime now() override {
    DateTime n = ds.now();
    return YqDateTime{ n.year(), n.month(), n.day(), n.hour(), n.minute(), n.second() };
  }

  bool setDailyAlarm(uint8_t hh, uint8_t mm) override {
    // 使用 Alarm2，比對 HH:MM，每日觸發
    ds.clearAlarm(2);
    ds.writeSqwPinMode(DS3231_OFF);
    ds.setAlarm2(DateTime(2025,1,1, hh, mm, 0), DS3231_A2_Minutes);
    return true;
  }

  bool clearAlarmFlag() override {
    ds.clearAlarm(1);
    ds.clearAlarm(2);
    return true;
  }
};
#endif


// =========================【RTC 選用物件】=========================
#if defined(RTC_IMPL_PCF8563)
RtcPCF8563 RTC;           // 使用 PCF8563
#elif defined(RTC_IMPL_DS3231)
RtcDS3231Wrap RTC;        // 使用 DS3231
#else
#error "請定義 RTC_IMPL_PCF8563 或 RTC_IMPL_DS3231"
#endif


// =========================【SPIFFS 設定檔存取工具】=========================
// 格式化數字為兩位字串 (01,02,...)
String fmt2(int v){ char b[8]; snprintf(b,sizeof(b),"%02d",v); return String(b); }

// 讀取文字檔
String readTextFile(const char* path){
  File f = SPIFFS.open(path, "r");
  if (!f) return String();
  String s = f.readString();
  f.close();
  return s;
}

// 寫入文字檔
bool writeTextFile(const char* path, const String& s){
  File f = SPIFFS.open(path, "w");
  if (!f) return false;
  f.print(s);
  f.close();
  return true;
}


// =========================【上線推播：notifyOnline】=========================
// 用法：在成功連上 Wi-Fi 後於主循環週期性呼叫；此函式會：
// 1) 進行 NTP 對時並把時間寫進 RTC 與 /rtc.txt
// 2) 取本機 IP，避免 10 秒內重複同 IP 推播
// 3) 成功推播一次後，整機僅通知一次（gOnlineNotifiedOnce）
//
// 作用/功能：避免頻繁上線通知；確保 RTC 有被校時與可回溯 IP 快照
void notifyOnline() {
  if (gOnlineNotifiedOnce) return;  // ★ 僅一次
  static String lastIpNoti;
  static unsigned long lastNotiMs = 0;

  if (!WiFi.isConnected() || !cfg.token.length() || !cfg.chat.length()) return;

  // NTP 對時（UTC+8）
  configTime(8*3600, 0, "pool.ntp.org", "time.google.com", "time.windows.com");
  struct tm tinfo;
  if (getLocalTime(&tinfo, 5000)) {
    YqDateTime dt{ tinfo.tm_year+1900, tinfo.tm_mon+1, tinfo.tm_mday,
                   tinfo.tm_hour, tinfo.tm_min, tinfo.tm_sec };
    RTC.adjust(dt);
    char snap[20];
    snprintf(snap, sizeof(snap), "%04d-%02d-%02d %02d:%02d",
             dt.year, dt.month, dt.day, dt.hour, dt.minute);
    writeTextFile("/rtc.txt", String(snap));
  }

  String ip = WiFi.localIP().toString();

  // 去重：同一個 IP 在 10 秒內只通知一次
  if (ip == lastIpNoti && millis() - lastNotiMs < 10000) return;
  lastIpNoti = ip; lastNotiMs = millis();

  bool ok = sendTelegram("\xF0\x9F\x93\xB6 裝置已上線，IP：" + ip);
  if (ok) {
    gOnlineNotifiedOnce = true;  // ★ 僅第一次成功才封印
    writeTextFile("/ip.txt", ip);
    tgSendControlKeyboard(cfg.chat);

  }
}


// =========================【設定檔：載入 loadConfig】=========================
// 用法：開機時呼叫一次，把 /config.txt 讀入 cfg
// 檔案格式：極簡 key=value（見 saveConfig 的輸出）
// 作用/功能：還原 WiFi / TG / 排程 / 星期遮罩 / 計數器達標等
void loadConfig(){
  File f = SPIFFS.open("/config.txt", "r");
  if (!f) return;
  while (f.available()){
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;
    int eq = line.indexOf('=');
    if (eq < 0) continue;
    String k = line.substring(0, eq);
    String v = line.substring(eq+1);

    // Wi-Fi 與 Telegram
    if (k == "ssid") cfg.ssid = v;
    else if (k == "pass") cfg.pass = v;
    else if (k == "token") cfg.token = v;
    else if (k == "chat") cfg.chat = v;
    // OLED 休眠秒數（5~3600）
    else if (k == "oled") {
      long sec = v.toInt();
      if (sec < 5) sec = 5;
      if (sec > 3600) sec = 3600;
      gOledSleepMs = (uint32_t)sec * 1000UL;
    }

    // 六路繼電器：時間/保持/訊息
    else if (k.startsWith("t")) { // t{i}=HH:MM
      int i = k.substring(1).toInt();
      if (i>=0 && i<RELAY_COUNT) {
        cfg.sch[i].hh = v.substring(0,2).toInt();
        cfg.sch[i].mm = v.substring(3,5).toInt();
      }
    }
    else if (k.startsWith("h")) { // h{i}=holdSec
      int i = k.substring(1).toInt();
      if (i>=0 && i<RELAY_COUNT) cfg.sch[i].hold = (uint32_t)v.toInt();
    }
    else if (k.startsWith("m")) { // m{i}=ON訊息
      int i = k.substring(1).toInt();
      if (i>=0 && i<RELAY_COUNT) cfg.sch[i].msg = v;
    }

    // 異常 DI 自訂訊息
    else if (k.startsWith("am")) { // am{i}
      int i = k.substring(2).toInt();
      if (i>=0 && i<ALARM_COUNT) cfg.aMsg[i] = v;
    }

    // 星期遮罩（十進位 0~127；bit0=Mon … bit6=Sun）
    else if (k == "wd") {
      long m = v.toInt();
      if (m < 0) m = 0;
      if (m > 127) m = 127;
      cfg.wdMask = (uint8_t)m;
    }

    // 工件計數：ct{i}=HH:MM、cm{i}=訊息、cn{i}=達標門檻(0=停用)
    else if (k.startsWith("ct")) {
      int i = k.substring(2).toInt();
      if (i>=0 && i<2) { cfg.cnt[i].hh = v.substring(0,2).toInt(); cfg.cnt[i].mm = v.substring(3,5).toInt(); }
    }
    else if (k.startsWith("cm")) {
      int i = k.substring(2).toInt();
      if (i>=0 && i<2) cfg.cnt[i].msg = v;
    }
    else if (k.startsWith("cn")) {
      int i = k.substring(2).toInt();
      if (i>=0 && i<2) cfg.cnt[i].target = (uint32_t)v.toInt();
    }
  }
  f.close();
}


// =========================【設定檔：儲存 saveConfig】=========================
// 用法：設定頁送出或程式內修改後呼叫；會覆寫 /config.txt
// 作用/功能：把 cfg 目前內容序列化為 key=value 文字檔
void saveConfig(){
  String s;
  s += "ssid="+cfg.ssid+"\n";
  s += "pass="+cfg.pass+"\n";
  s += "token="+cfg.token+"\n";
  s += "chat="+cfg.chat+"\n";

  for (int i=0;i<RELAY_COUNT;i++){
    s += "t"+String(i)+"="+fmt2(cfg.sch[i].hh)+":"+fmt2(cfg.sch[i].mm)+"\n";
    s += "h"+String(i)+"="+String(cfg.sch[i].hold)+"\n";
    s += "m"+String(i)+"="+cfg.sch[i].msg+"\n";
  }

  for (int i=0;i<ALARM_COUNT;i++){
    s += "am"+String(i)+"="+ (cfg.aMsg[i].length()? cfg.aMsg[i] : gAlarmMsg[i]) +"\n";
  }

  s += "wd="+String(cfg.wdMask)+"\n";

  for (int i=0;i<2;i++){
    s += "ct"+String(i)+"="+fmt2(cfg.cnt[i].hh)+":"+fmt2(cfg.cnt[i].mm)+"\n";
    s += "cm"+String(i)+"="+cfg.cnt[i].msg+"\n";
    s += "cn"+String(i)+"="+String(cfg.cnt[i].target)+"\n";
  }
  s += "oled=" + String(gOledSleepMs / 1000UL) + "\n";

  // ★ 真正寫檔
  writeTextFile("/config.txt", s);
}

  // 粗略擷取工具（不使用 ArduinoJson）
  static String jsonGet(const String& s, const char* key){
    // 尋找 "key":
    String k = String("\"") + key + "\":";
    int p = s.indexOf(k);
    if (p < 0) return "";
    p += k.length();
    // 跳過空白與引號
    while (p < (int)s.length() && (s[p]==' '||s[p]=='\"')) { if (s[p]=='\"') { // 如果是字串，取到下一個引號
        int q = s.indexOf('\"', p+1);
        if (q>p) return s.substring(p+1, q);
      } p++;
    }
    // 數字/布林/物件字串粗略擷取（直到逗號或右大括號）
    int q = p;
    while (q < (int)s.length() && s[q]!=',' && s[q]!='}' && s[q]!=']' && s[q]!='\n') q++;
    String v = s.substring(p, q); v.trim();
    // 去除包住的引號
    if (v.length()>=2 && v[0]=='\"' && v[v.length()-1]=='\"') v = v.substring(1, v.length()-1);
    return v;
  }
  // 在 applyWebAppConfig(const String& data) 一進來就加上這段「解包 payload」的小段
String unwrapPayloadIfAny(const String& s){
  // 找 "payload":{ ... } 的大括號範圍，回傳純 payload 內容；找不到就回原字串
  int p = s.indexOf("\"payload\"");
  if (p < 0) return s;
  // 找到冒號
  p = s.indexOf(':', p);
  if (p < 0) return s;
  // 找到第一個 '{'
  while (p < (int)s.length() && s[p] != '{') p++;
  if (p >= (int)s.length()) return s;
  int depth = 0, i = p;
  for (; i < (int)s.length(); ++i){
    char c = s[i];
    if (c == '{') depth++;
    else if (c == '}'){
      depth--;
      if (depth == 0){ // [p..i] 是 payload 物件
        return s.substring(p, i+1);
      }
    }
  }
  return s;
}

// 將 WebApp 的 data(JSON) 套用到 cfg 後 save
static void applyWebAppConfig(const String& raw) {
  Serial.printf("[CFG] applyWebAppConfig: %d bytes\n", raw.length());
  String data = unwrapPayloadIfAny(raw);   // ★ 修正：參數名改 raw，這裡才建立 data 變數

  // （可選）驗證 ct1（HH:MM 或空白）
  auto validHHMM = [](const String& v)->bool{
    if (!v.length()) return true; // 空白=停用
    if (v.length() < 4) return false;
    int hh = v.substring(0,2).toInt();
    int mm = v.substring(3,5).toInt();
    return hh>=0 && hh<=23 && mm>=0 && mm<=59;
  };

  uint8_t mask = 0;
  int pos = data.indexOf("\"wd\":[");
  if (pos >= 0) {
    int end = data.indexOf("]", pos);
    if (end > pos) {
      String arr = data.substring(pos+6, end);
      int bit = 0; int i = 0;
      while (i < (int)arr.length() && bit < 7) {
        while (i < (int)arr.length() && (arr[i]==' '||arr[i]==',')) i++;
        if (i < (int)arr.length() && (arr[i]=='1')) mask |= (1u << bit);
        while (i < (int)arr.length() && arr[i]!=',') i++;
        bit++;
      }
      cfg.wdMask = mask;
    }
  }
  // ——— 定時排程：ts(時間字串), hm(保持分), hs(保持秒) ———
  auto parseArray = [&](const String& key)->String {
    int p = data.indexOf(String("\"") + key + "\":[");
    if (p < 0) return "";
    int end = data.indexOf("]", p);
    if (end < 0) return "";
    return data.substring(p + key.length() + 4, end); // 內容不含 [ ]
  };

  String tsArr = parseArray("ts");
  String hmArr = parseArray("hm");
  String hsArr = parseArray("hs");

  if (tsArr.length() || hmArr.length() || hsArr.length()) {
    // 先拆出 6 筆
    auto splitCSV = [&](const String& s, bool isString)->std::vector<String>{
      std::vector<String> out;
      int i=0, n=s.length();
      while (i<n && (int)out.size()<6) {
        // 跳逗點與空白
        while (i<n && (s[i]==','||s[i]==' ')) i++;
        if (i>=n) break;
        if (isString && s[i]=='\"') {
          int q = s.indexOf('\"', i+1);
          if (q<0) break;
          out.push_back(s.substring(i+1, q));
          i = q+1;
        } else {
          int q=i;
          while (q<n && s[q]!=',') q++;
          String v = s.substring(i, q); v.trim();
          out.push_back(v);
          i = q+1;
        }
      }
      return out;
    };

    auto ts = splitCSV(tsArr, true);
    auto hm = splitCSV(hmArr, false);
    auto hs = splitCSV(hsArr, false);

    for (int i=0;i<6;i++){
      // 時間
      if (i < (int)ts.size() && ts[i].length() >= 4) {
        int hh = ts[i].substring(0,2).toInt();
        int mm = ts[i].substring(3,5).toInt();
        if (hh>=0 && hh<=23) cfg.sch[i].hh = hh;
        if (mm>=0 && mm<=59) cfg.sch[i].mm = mm;
      }
      // 保持
      long m = (i < (int)hm.size()) ? hm[i].toInt() : 0;
      long s = (i < (int)hs.size()) ? hs[i].toInt() : 0;
      long hold = m*60 + s;
      if (hold < 0) hold = 0;
      if (hold > 3600) hold = 3600;
      cfg.sch[i].hold = (uint32_t)hold;
    }
  }

    // ——— 計數器 #1 ———
    String cm0 = jsonGet(data, "cm0");
    if (cm0.length()) cfg.cnt[0].msg = cm0;
    String cn0 = jsonGet(data, "cn0");
    if (cn0.length()) {
      long v = cn0.toInt();
      if (v < 0) v = 0;
      cfg.cnt[0].target = (uint32_t)v;
    }
    // ——— 計數器 #2 ———
    String ct1 = jsonGet(data, "ct1"); // 例如 "17:30"
    if (ct1.length() >= 4) { cfg.cnt[1].hh = ct1.substring(0,2).toInt(); cfg.cnt[1].mm = ct1.substring(3,5).toInt(); }
    String cm1 = jsonGet(data, "cm1");
    if (cm1.length()) cfg.cnt[1].msg = cm1;
    // ===== 解析 t[] / hm[] / hs[] → cfg.sch[i].hh/mm/hold =====
auto parseStrArray = [&](const String& key, String out[], int n){
  int pos = data.indexOf(String("\"")+key+"\":[");
  if (pos < 0) return false;
  int end = data.indexOf("]", pos);
  if (end < 0) return false;
  String arr = data.substring(pos + key.length() + 4, end);
  int i=0, p=0;
  while (i<n && p < (int)arr.length()){
    int s = arr.indexOf('\"', p); if (s<0) break;
    int e = arr.indexOf('\"', s+1); if (e<0) break;
    out[i++] = arr.substring(s+1, e);
    p = e+1;
  }
  return (i>0);
};
auto parseIntArray = [&](const String& key, int out[], int n){
  int pos = data.indexOf(String("\"")+key+"\":[");
  if (pos < 0) return false;
  int end = data.indexOf("]", pos);
  if (end < 0) return false;
  String arr = data.substring(pos + key.length() + 4, end);
  int i=0, p=0;
  while (i<n && p < (int)arr.length()){
    // 取出下一個數字（逗號分隔）
    int q = arr.indexOf(',', p); if (q<0) q = arr.length();
    String token = arr.substring(p, q); token.trim();
    out[i++] = token.toInt();
    p = q + 1;
  }
  return (i>0);
};

String t[RELAY_COUNT];
int    hm[RELAY_COUNT] = {0};
int    hs[RELAY_COUNT] = {0};

bool hasT  = parseStrArray("t",  t,  RELAY_COUNT);
bool hasHM = parseIntArray("hm", hm, RELAY_COUNT);
bool hasHS = parseIntArray("hs", hs, RELAY_COUNT);

if (hasT || hasHM || hasHS){
  for (int i=0;i<RELAY_COUNT;i++){
    // 時間 "HH:MM"
    if (hasT && t[i].length()>=4){
      int colon = t[i].indexOf(':');
      if (colon>0){
        int hh = t[i].substring(0, colon).toInt();
        int mm = t[i].substring(colon+1).toInt();
        if (hh<0) hh=0; if (hh>23) hh=23;
        if (mm<0) mm=0; if (mm>59) mm=59;
        cfg.sch[i].hh = hh;
        cfg.sch[i].mm = mm;
      }
    }
    // 保持時間：分/秒 → hold（秒）
    if (hasHM || hasHS){
      long hold = (long)cfg.sch[i].hold;  // 先用原值
      if (hasHM) hold = (long)hm[i] * 60L + (hasHS ? hs[i] : 0);
      else if (hasHS) hold = (long)hs[i];
      if (hold < 0) hold = 0;
      if (hold > 3600) hold = 3600;
      cfg.sch[i].hold = (uint16_t)hold;
    }
  }
}

    saveConfig();
    tgEnqueue("⚙️ WebApp 已更新：定時與保持時間已套用");
  }
  
  // 輪詢 getUpdates，抓取 web_app_data.data
  static void tgUpdatePollLoop(){
    static unsigned long last = 0;
    if (millis() - last < 1500) return;  // 1.5s 節流
    last = millis();
  
    if (!WiFi.isConnected() || !cfg.token.length()) return;
  
    WiFiClientSecure cli; cli.setInsecure();
    if (!cli.connect("api.telegram.org", 443)) return;
  
    String url = "/bot" + cfg.token + "/getUpdates?timeout=0&limit=5";
    if (tgUpdateOffset) url += "&offset=" + String(tgUpdateOffset);
  
    String req = "GET " + url + " HTTP/1.1\r\nHost: api.telegram.org\r\nConnection: close\r\n\r\n";
    cli.print(req);
  
    unsigned long t0 = millis();
    while (!cli.available() && millis()-t0 < 5000) delay(10);
    if (!cli.available()) { cli.stop(); return; }
  
    // 跳 header
    while (cli.connected()){
      String h = cli.readStringUntil('\n');
      if (h == "\r" || h.length()==0) break;
    }
  
    String body;
    while (cli.available()) body += (char)cli.read();
    cli.stop();
  
    // 逐條擷取 result 陣列裡的物件；只找 web_app_data
    int pos = 0;
    while (true){
      int upd = body.indexOf("\"update_id\":", pos);
      if (upd < 0) break;
      int colon = body.indexOf(':', upd);
      int comma = body.indexOf(',', colon+1);
      long uid = body.substring(colon+1, comma).toInt();
      tgUpdateOffset = uid + 1; // 下一輪從下一筆
// ---- 取得本筆 update 的文字內容（如 /panel、/check_token）----
int nextUpd = body.indexOf("\"update_id\":", comma+1);
int scopeEnd = (nextUpd > 0) ? nextUpd : body.length();
int tpos = body.indexOf("\"text\":\"", comma);
if (tpos > 0 && tpos < scopeEnd) {
  int q1 = body.indexOf('\"', tpos + 7);
  int q2 = body.indexOf('\"', q1 + 1);
  String txt = (q1 > 0 && q2 > q1) ? body.substring(q1+1, q2) : "";

  String up = txt; up.toUpperCase();

  if (up == "/PANEL" || txt == "⚙️ 開啟設定" || up == "PANEL") {
    if (cfg.chat.length()) {
      tgHideKeyboard(cfg.chat);       // ★ 收起舊鍵盤
      tgSendControlKeyboard(cfg.chat); // 可留著（顯示文字鍵盤）
      tgSendInlineOpen(cfg.chat);      // ★ 關鍵：送 inline WebApp 按鈕（內嵌開啟）
    }
  }

 // if (up == "/CHECK_TOKEN") {
    //if (cfg.chat.length()) {
     // tgHideKeyboard(cfg.chat);
     // tgEnqueue("✅ 已關閉鍵盤");
   // }
  //}
}

      int wad = body.indexOf("\"web_app_data\"", comma);
      if (wad > 0){
        int dataPos = body.indexOf("\"data\":", wad);
        if (dataPos > 0) {
          int q1 = body.indexOf('\"', dataPos + 7);
          if (q1 > 0) {
            int i = q1 + 1; bool esc=false;
            while (i < (int)body.length()) { char c=body[i]; if (esc){esc=false;i++;continue;}
              if (c=='\\'){esc=true;i++;continue;} if (c=='\"') break; i++; }
            if (i < (int)body.length()) {
              String raw = body.substring(q1+1, i);
              String payload; payload.reserve(raw.length());
              for (int k=0;k<(int)raw.length();k++){ char c=raw[k];
                if (c=='\\' && k+1<(int)raw.length()){
                  char n=raw[k+1]; if (n=='\"'){payload+='\"';k++;continue;}
                  if (n=='\\'){payload+='\\';k++;continue;}
                  if (n=='n'){payload+='\n';k++;continue;}
                  if (n=='r'){payload+='\r';k++;continue;}
                  if (n=='t'){payload+='\t';k++;continue;}
                } payload+=c;
              }
              Serial.printf("[TG] got web_app_data, %d bytes\n", payload.length());
              tgEnqueue("🛰 收到 WebApp 設定，開始套用…");
              applyWebAppConfig(payload);
            }
          }
        }
    

      }
      pos = comma + 1;
    }
  }
  

// =========================【工具：安全顯示 IP】=========================
// 用法：顯示於網頁或日誌；AP 則回 10.10.0.1 類，STA 回本機 DHCP IP
String safeIP() {
  if (WiFi.getMode() == WIFI_AP) return WiFi.softAPIP().toString();
  if (WiFi.isConnected()) return WiFi.localIP().toString();
  return String("0.0.0.0");
}


// =========================【工具：現在時間字串】=========================
// 用法：網頁模板 {{NOW}} 置換、或日誌顯示使用
// 優先使用 NTP（getLocalTime），失敗則退回 RTC
String nowString() {
  struct tm t;
  if (getLocalTime(&t, 50)) {
    char buf[20];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M", &t);
    return String(buf);
  }
  if (gRtcReady) {
    YqDateTime n = RTC.now();
    return String(n.year) + "-" + fmt2(n.month) + "-" + fmt2(n.day) + " " +
           fmt2(n.hour) + ":" + fmt2(n.minute);
  }
  return String("--");
}


// =========================【頁面模板渲染：renderIndex】=========================
// 用法：HTTP GET "/" 時產生 index.html 的最終頁面字串
// 功能：讀取 /index.html → 依多種 {{KEY}} 變數做替換，包含：
//   {{IP}} / {{NOW}} / {{RTC_STATUS}} / {{SHOW_SECRETS}} / {{WD0..6}}
//   計數器：{{CT0/1}} {{CM0/1}} {{CN0/1}}
//   排程：{{T0..}} {{HM0..}} {{HS0..}} {{M0..}} {{MON0..}} {{MOFF0..}}
//   異常 DI：{{AM0..5}}
String renderIndex(){
  String html = readTextFile("/index.html");

  // (NEW) RTC 狀態顯示：若 RTC 未 ready 視為需校時
  bool vl = gRtcReady ? RTC.lostPower() : true;
  html.replace("{{IP}}", safeIP());
  html.replace("{{NOW}}", nowString());
  html.replace("{{RTC_STATUS}}", vl ? "\xE2\x9A\xA0\xEF\xB8\x8F RTC 掉電/未校時" : "\xE2\x9C\x85 RTC 正常");

  // ===== 敏感欄位顯示策略 =====
  // AP / AP+STA：顯示卡片但欄位清空；STA：隱藏卡片
  wifi_mode_t md = WiFi.getMode();
  bool isAP = (md == WIFI_AP || md == WIFI_AP_STA);
  if (isAP) {
    html.replace("{{SSID}}",  "");
    html.replace("{{PASS}}",  "");
    html.replace("{{TOKEN}}", "");
    html.replace("{{CHAT}}",  "");
    html.replace("{{SHOW_SECRETS}}", "1");
  } else {
    html.replace("{{SSID}}",  "");
    html.replace("{{PASS}}",  "");
    html.replace("{{TOKEN}}", "");
    html.replace("{{CHAT}}",  "");
    html.replace("{{SHOW_SECRETS}}", "0");
  }

  // ===== 星期 checkbox（bit0=Mon … bit6=Sun）=====
  for (int i = 0; i < 7; ++i) {
    String key = String("{{WD") + i + "}}";
    html.replace(key, ((cfg.wdMask >> i) & 0x01) ? "checked" : "");
  }

  // ===== 計數器 {{CT/CM/CN}} =====
  for (int i=0;i<2;i++){
    html.replace(String("{{CT")+i+"}}", fmt2(cfg.cnt[i].hh)+":"+fmt2(cfg.cnt[i].mm));
    html.replace(String("{{CM")+i+"}}", cfg.cnt[i].msg);
    html.replace(String("{{CN")+i+"}}", String(cfg.cnt[i].target));  // ★ 達標門檻
  }

  // ===== 繼電器排程區塊 =====
  for (int i = 0; i < RELAY_COUNT; i++){
    // HH:MM
    html.replace(String("{{T")+i+"}}", fmt2(cfg.sch[i].hh)+":"+fmt2(cfg.sch[i].mm));

    // 保持時間轉換為「分/秒」雙欄顯示
    uint32_t sec = cfg.sch[i].hold;
    uint32_t hm  = sec / 60;
    uint32_t hs  = sec % 60;
    html.replace(String("{{HM")+i+"}}", String(hm));
    html.replace(String("{{HS")+i+"}}", String(hs));

    // 推播訊息（預設 "RELAY{i}"）
    String mon = cfg.sch[i].msg.length() ? cfg.sch[i].msg
                                         : ("RELAY" + String(i+1));
    // 舊版相容 {{M}}；新版分離 ON/OFF 文案
    html.replace(String("{{M")+i+"}}", mon);
    html.replace(String("{{MON")+i+"}}", mon);
    html.replace(String("{{MOFF")+i+"}}", mon + "關閉");
  }

  // ===== 異常 DI 訊息 =====
  for (int i=0;i<ALARM_COUNT;i++){
    html.replace(String("{{AM")+i+"}}", gAlarmMsg[i]);
  }

  return html;
}


// =========================【HTTP：首頁處理】=========================
// 用法：HTTP GET "/" 時呼叫
// 功能：渲染 index.html 模板並回傳
void handleRoot(){
  oledKick("http");                 // ★ 只要有 HTTP 存取 → 喚醒
  String page = renderIndex();
  srv.send(200, "text/html; charset=utf-8", page);
}


// =========================【設定變更摘要工具】=========================
// 用法：addChangeIf(changes, "標題", 舊值, 新值)
// 功能：若 before != after，就把「標題：before → after」加入變更摘要
static void addChangeIf(std::vector<String>& diff, const String& title,
                        const String& before, const String& after){
  if (before != after) diff.push_back(title + "：" + before + " → " + after);
}

// 工具：把 HH,MM 組成 "HH:MM" 字串
static String hhmm(uint8_t hh, uint8_t mm){
  char b[6]; snprintf(b, sizeof(b), "%02d:%02d", hh, mm); return String(b);
}


// =========================【HTTP：儲存設定 handleSave】=========================
// 用法：HTTP POST "/save" 送出設定表單時呼叫
// 作用：讀取表單 → 寫入 cfg → 比對差異 → 存檔 → （必要時）推播摘要 → 導頁
// 重點：
//  1) Wi-Fi 憑證僅允許在 AP 模式更新（避免誤清空）
//  2) 繼電器保持時間做上下限保護（MIN_HOLD_SEC~MAX_HOLD_SEC）
//  3) 變更摘要：最多列 12 條
void handleSave(){
  AppConfig old = cfg;                 // ← 舊設定快照，用於差異比對
  std::vector<String> changes;         // ← 本次修改摘要清單

  // ---------- 1) 敏感欄位：僅有值才更新，避免空值洗掉 ----------
  String newSsid = srv.arg("ssid");
  String newPass = srv.arg("pass");
  if (srv.hasArg("token") && srv.arg("token").length()) cfg.token = srv.arg("token");
  if (srv.hasArg("chat")  && srv.arg("chat").length())  cfg.chat  = srv.arg("chat");

  // ---------- 2) 異常 DI 訊息（先更新，再比對摘要） ----------
  for (int i = 0; i < ALARM_COUNT; i++) {
    String k = "am" + String(i);
    if (srv.hasArg(k)) {
      cfg.aMsg[i]  = srv.arg(k);
      // 若空字串就維持舊預設 gAlarmMsg[i]（不覆寫為空）
      gAlarmMsg[i] = cfg.aMsg[i].length() ? cfg.aMsg[i] : gAlarmMsg[i];
    }
    addChangeIf(changes, "DI"+String(i+1)+" 訊息", old.aMsg[i], cfg.aMsg[i]);
  }

  // ---------- 3) 六路排程（時間 / 保持 / 訊息） ----------
  for (int i = 0; i < RELAY_COUNT; i++) {
    // 時間 t{i} = "HH:MM"
    if (srv.hasArg("t"+String(i))) {
      String t = srv.arg("t"+String(i));
      int hh = constrain(t.substring(0,2).toInt(), 0, 23);
      int mm = constrain(t.substring(3,5).toInt(), 0, 59);
      cfg.sch[i].hh = (uint8_t)hh;
      cfg.sch[i].mm = (uint8_t)mm;
    }

    // 保持（分秒轉秒，並做上/下限保護）
    uint32_t hm = srv.hasArg("hm"+String(i)) ? (uint32_t)srv.arg("hm"+String(i)).toInt() : 0;
    uint32_t hs = srv.hasArg("hs"+String(i)) ? (uint32_t)srv.arg("hs"+String(i)).toInt() : 0;
    uint32_t sec = hm*60 + hs;
    if (sec < MIN_HOLD_SEC) sec = MIN_HOLD_SEC;
    if (sec > MAX_HOLD_SEC) sec = MAX_HOLD_SEC;
    cfg.sch[i].hold = sec;

    // 訊息（優先新版 mon{i}，沒有才退回舊 m{i}）
    if (srv.hasArg("mon"+String(i)) && srv.arg("mon"+String(i)).length())
      cfg.sch[i].msg = srv.arg("mon"+String(i));
    else if (srv.hasArg("m"+String(i)))
      cfg.sch[i].msg = srv.arg("m"+String(i));

    // 統一在這裡做變更摘要
    addChangeIf(changes, "CH"+String(i+1)+" 時間",
                hhmm(old.sch[i].hh, old.sch[i].mm),
                hhmm(cfg.sch[i].hh, cfg.sch[i].mm));
    addChangeIf(changes, "CH"+String(i+1)+" 保持(秒)",
                String(old.sch[i].hold), String(cfg.sch[i].hold));
    addChangeIf(changes, "CH"+String(i+1)+" 訊息",
                old.sch[i].msg, cfg.sch[i].msg);
  }

  // ---------- 4) 星期遮罩 wd0..wd6（Mon..Sun） ----------
  uint8_t wdMaskTmp = 0;
  for (int i = 0; i < 7; ++i) {
    String k = "wd" + String(i);
    if (srv.hasArg(k)) wdMaskTmp |= (1u << i);
  }
  cfg.wdMask = wdMaskTmp;
  if (old.wdMask != cfg.wdMask) {
    changes.push_back("星期勾選變更（Mon..Sun bitmask）：" + String(old.wdMask) + " → " + String(cfg.wdMask));
  }

  // ---------- 5) 工件計數（每日時間 / 訊息 / 達標門檻） ----------
  for (int i=0;i<2;i++){
    if (srv.hasArg("ct"+String(i))) {
      String t = srv.arg("ct"+String(i));
      cfg.cnt[i].hh = t.substring(0,2).toInt();
      cfg.cnt[i].mm = t.substring(3,5).toInt();
    }
    if (srv.hasArg("cm"+String(i))) cfg.cnt[i].msg = srv.arg("cm"+String(i));
    if (srv.hasArg("cn"+String(i))) {
      long v = srv.arg("cn"+String(i)).toInt();
      if (v < 0) v = 0;                // 下限保護
      cfg.cnt[i].target = (uint32_t)v; // 0=停用達標推播
    }

    addChangeIf(changes, "計數器#"+String(i+1)+" 回報時間",
                hhmm(old.cnt[i].hh, old.cnt[i].mm),
                hhmm(cfg.cnt[i].hh, cfg.cnt[i].mm));
    addChangeIf(changes, "計數器#"+String(i+1)+" 訊息",
                old.cnt[i].msg, cfg.cnt[i].msg);
    addChangeIf(changes, "計數器#"+String(i+1)+" 達標門檻",
                String(old.cnt[i].target), String(cfg.cnt[i].target));
  }

  // ---------- 6) Wi-Fi 憑證僅在 AP 模式允許修改 ----------
  bool allowCredEdit = (WiFi.getMode() == WIFI_AP);
  if (allowCredEdit) {
    if (newSsid.length()) cfg.ssid = newSsid;
    if (newPass.length()) cfg.pass = newPass;
    if (old.ssid != cfg.ssid) changes.push_back("Wi-Fi SSID 已更新");
    if (old.pass != cfg.pass) changes.push_back("Wi-Fi 密碼已更新");
  }
  // Token/Chat 可於任何模式修改
  if (old.token != cfg.token) changes.push_back("Telegram Token 已更新");
  if (old.chat  != cfg.chat ) changes.push_back("Telegram Chat ID 已更新");

  // ---------- 7) 寫入設定檔 ----------
  saveConfig();

  // ---------- 7-1) 若有變更 → 推播摘要（最多 12 條） ----------
  if (!changes.empty()) {
    String msg = "⚙️ 設定已更新（" + String(changes.size()) + " 項）\n";
    for (size_t i=0;i<changes.size() && i<12;i++){
      msg += "• " + changes[i] + "\n";
    }
    if (changes.size() > 12) msg += "…（其餘略）";
    tgEnqueue(msg);
  }

  // ---------- 8) 導頁流程 ----------
  // 8A) AP 模式且有 SSID → 嘗試連線，並顯示成功/失敗提示頁
  if (allowCredEdit && cfg.ssid.length()) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) delay(100);

    String page;
    if (WiFi.status() == WL_CONNECTED) {
      // 成功：NTP → RTC 快照（/rtc.txt）＋ 顯示取得的 IP
      configTime(8*3600, 0, "pool.ntp.org", "time.google.com", "time.windows.com");
      struct tm tinfo;
      if (getLocalTime(&tinfo, 5000)) {
        YqDateTime dt{ tinfo.tm_year+1900, tinfo.tm_mon+1, tinfo.tm_mday,
                       tinfo.tm_hour, tinfo.tm_min, tinfo.tm_sec };
        RTC.adjust(dt);
        char snap[20];
        snprintf(snap, sizeof(snap), "%04d-%02d-%02d %02d:%02d",
                 dt.year, dt.month, dt.day, dt.hour, dt.minute);
        writeTextFile("/rtc.txt", String(snap));
      }
        // （可選）支援從 /save 帶入 oled=秒數
      if (srv.hasArg("oled")) {
      long sec = srv.arg("oled").toInt();
      if (sec < 5) sec = 5;
      if (sec > 3600) sec = 3600;
      gOledSleepMs = (uint32_t)sec * 1000UL;
     }

      String ip = WiFi.localIP().toString();
      page =
        "<!doctype html><meta charset='utf-8'>"
        "<title>設定已儲存</title>"
        "<body style='font-family:system-ui;line-height:1.6'>"
        "<h3>設定已儲存 ✅</h3>"
        "<p>已連上路由，取得位址：<b>" + ip + "</b></p>"
        "<ol><li><b>請把電腦/手機改連回你的路由 Wi-Fi</b></li>"
        "<li>再用這個網址開啟：<a href='http://" + ip + "/'>http://" + ip + "/</a></li></ol>"
        "<p>AP 將於 5 秒後自動關閉。</p>"
        "</body>";
      gCloseApAt = millis() + 5000;  // 延遲關 AP
    } else {
      // 失敗：留在 AP 模式，提示回 10.10.0.1 重新設定
      page =
        "<!doctype html><meta charset='utf-8'>"
        "<title>設定已儲存</title>"
        "<body style='font-family:system-ui;line-height:1.6'>"
        "<h3>設定已儲存，但目前連不上路由 ⚠️</h3>"
        "<p>請確認 SSID/密碼無誤。裝置仍維持 AP 模式，"
        "可回 <a href='http://10.10.0.1/'>http://10.10.0.1/</a> 重新設定。</p>"
        "</body>";
    }
    srv.send(200, "text/html; charset=utf-8", page);
    return;
  }

  // 8B) 非 AP（或 AP 但未改 Wi-Fi）→ 以 302 導回首頁
  srv.sendHeader("Location", "/?saved=1");
  srv.send(302);
}



// =========================【HTTP：手動校時 handleSetTime】=========================
// 用法：HTTP POST /set-time?when=YYYY-MM-DD HH:MM
// 作用：把網頁送來的字串時間寫進 RTC，並把快照寫入 /rtc.txt，最後 302 回首頁
void handleSetTime(){
  String when = srv.arg("when"); // "YYYY-MM-DD HH:MM"
  if (when.length() >= 16){
    YqDateTime dt{};
    dt.year   = when.substring(0,4).toInt();
    dt.month  = when.substring(5,7).toInt();
    dt.day    = when.substring(8,10).toInt();
    dt.hour   = when.substring(11,13).toInt();
    dt.minute = when.substring(14,16).toInt();
    dt.second = 0;

    RTC.adjust(dt);
    writeTextFile("/rtc.txt", when);  // 例如 "2025-09-05 14:35"
#if defined(RTC_IMPL_PCF8563)
    // PCF8563：寫秒寄存器時會清 VL，已由 adjust() 處理
#endif
  }
  srv.sendHeader("Location","/");
  srv.send(302);
}


// =========================【工具：繼電器脈衝輸出 pulseRelay】=========================
// 用法：pulseRelay(ch, sec)
// 作用：指定通道繼電器吸合 holdSec 秒；過程中持續 srv.handleClient() 避免阻塞
void pulseRelay(int ch, uint32_t holdSec){
  if (ch < 0 || ch >= RELAY_COUNT) return;
  int pin = RELAY_PINS[ch];

  digitalWrite(pin, RELAY_ACTIVE_HIGH ? HIGH : LOW);
  unsigned long until = millis() + (unsigned long)holdSec * 1000UL;

  // ★ 保持期間不停服務 HTTP，避免 /set-time、/save 被卡住
  while ((long)(millis() - until) < 0) {
    srv.handleClient();
    delay(2);
    yield();
  }
  digitalWrite(pin, RELAY_ACTIVE_HIGH ? LOW : HIGH);
}


// =========================【HTTP：單一路測試 handleTestRelay】=========================
// 用法：HTTP GET /test-relay?ch={0..RELAY_COUNT-1}
// 作用：若該路尚在保持，先 stop → 再 start；即時回傳文字，不跳轉；同時推播開始文案
void handleTestRelay(){
  int ch = srv.hasArg("ch") ? srv.arg("ch").toInt() : 0;
  if (ch < 0 || ch >= RELAY_COUNT){ srv.send(400,"text/plain","bad ch"); return; }

  // 若同一路還在保持 → 先強制釋放（等於重啟）
  if (gTestActive[ch]) {
    stopRelayIfActive(ch, "重啟");
  }

  // 立即回應文字，不擋主流程
  String resp = "已觸發 CH" + String(ch+1) + "（保持 " 
              + String(cfg.sch[ch].hold) + " 秒），訊息：" 
              + cfg.sch[ch].msg;
  srv.send(200, "text/plain; charset=utf-8", resp);

  // 啟動計時並推播
  startRelayTimed(ch, cfg.sch[ch].hold);
  uiShow("TEST CH"+String(ch+1), "保持 "+String(cfg.sch[ch].hold)+"s");
  tgEnqueue(cfg.sch[ch].msg);
}


// =========================【HTTP：自檢流程 handleSelfTest】=========================
// 用法：HTTP GET /self-test
// 作用：依序測試 6 路，每路以目前設定 hold（但最多 3 秒），過程不中斷 HTTP，並推播開始/結束
void handleSelfTest(){
  String report;
  for (int i = 0; i < RELAY_COUNT; ++i) {
    if (gTestActive[i]) { report += "CH" + String(i+1) + ": busy\n"; continue; }

    uint32_t hold = cfg.sch[i].hold;
    if (hold > 3) hold = 3;  // 自檢上限 3 秒，避免測太久

    tgEnqueue("CH" + String(i+1) + " 自檢開始（保持 " + String(hold) + " 秒）");
    startRelayTimed(i, hold);

    // 等待該路測試結束（利用現有非阻塞旗標）
    unsigned long waitStart = millis();
    while (gTestActive[i]) {
      srv.handleClient();
      if (millis() - waitStart > (hold * 1000UL + 1500)) break; // 超時保險
      delay(5);
      yield();
    }

    tgEnqueue("CH" + String(i+1) + " 自檢結束");
    report += "CH" + String(i+1) + ": OK\n";
    delay(50);
  }
  srv.send(200, "text/plain; charset=utf-8", report);
}


// =========================【Wi-Fi：AP 模式 startAP】=========================
// 用法：在需要進入設定頁時呼叫；建立 SSID=Y&Q_Notify、密碼=88888888，IP=10.10.0.1
// 作用：提供使用者進入設定頁的熱點（AP）
void startAP() {
  WiFi.mode(WIFI_AP);
  IPAddress apIP(10,10,0,1), gateway(10,10,0,1), subnet(255,255,255,0);
  WiFi.softAPConfig(apIP, gateway, subnet);
  WiFi.softAP("Y&Q_Notify", "88888888");
  Serial.printf("[AP 模式, IP: %s]\n", WiFi.softAPIP().toString().c_str());
}


// =========================【Wi-Fi：STA 連線 startSTA】=========================
// 用法：開機後若 cfg.ssid 非空 → 嘗試連線路由；成功後做 NTP→RTC、寫 /rtc.txt
// 回傳：true=成功連上；false=失敗
bool startSTA() {
  if (!cfg.ssid.length()) return false;

  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(100);

  if (WiFi.status() == WL_CONNECTED) {
    // 成功連線後：NTP 同步 → RTC（寫入 /rtc.txt 快照）
    configTime(8*3600, 0, "pool.ntp.org", "time.google.com", "time.windows.com");
    struct tm tinfo;
    if (getLocalTime(&tinfo, 5000)) {
      YqDateTime dt{ tinfo.tm_year+1900, tinfo.tm_mon+1, tinfo.tm_mday,
                     tinfo.tm_hour, tinfo.tm_min, tinfo.tm_sec };
      RTC.adjust(dt);
      char snap[20];
      snprintf(snap, sizeof(snap), "%04d-%02d-%02d %02d:%02d",
               dt.year, dt.month, dt.day, dt.hour, dt.minute);
      writeTextFile("/rtc.txt", String(snap));
      Serial.println("[RTC] startSTA：已 NTP→RTC");
    }
    return true;
  }
  return false;
}


// =========================【Wi-Fi：阻塞等待工具 wifiConnectWait】=========================
// 用法：if (wifiConnectWait(15000)) { ... }；預設 15 秒
// 作用：在指定時間內等到 WL_CONNECTED；回傳 true/false
static bool wifiConnectWait(unsigned long ms=15000){
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis()-t0 < ms) delay(150);
  return WiFi.status() == WL_CONNECTED;
}


// =========================【Wi-Fi：初始化 beginWiFi】=========================
// 用法：setup() 開機時呼叫一次
// 作用：根據 AP 鍵與是否已有憑證，決定進 AP 或 STA；先恢復 DHCP 預設
void beginWiFi() {
  WiFi.persistent(false);   // 不寫入 NVS，避免磨損
  WiFi.setSleep(false);     // 關閉省電，減少延遲

  pinMode(AP_MODE_PIN, INPUT_PULLUP);
  bool apRequested = (digitalRead(AP_MODE_PIN) == (AP_ACTIVE_LOW ? LOW : HIGH));
  bool haveCreds   = cfg.ssid.length() > 0;

  // ★ 永遠先回 DHCP（清掉靜態 IP）
  WiFi.config(0U,0U,0U);

  if (apRequested || !haveCreds) {
    Serial.println("[WiFi] AP 模式（GPIO 觸發或尚未設定 SSID）");
    startAP();
    return;
  }

  if (startSTA()) {
    Serial.print("[WiFi] STA IP = "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] STA 連線失敗；維持 STA（不自動開 AP），稍後再嘗試或按 GPIO 進入 AP");
    WiFi.mode(WIFI_STA);  // 保持 STA，等待 /save 修正憑證
  }
}


// =========================【時間來源：取目前小時/分鐘 getHM】=========================
// 用法：if (getHM(h,m)) {...}
// 作用：多層回退策略（NTP -> RTC -> /rtc.txt -> 最近快取 60 秒）
// 回傳：true=取得 h/m 成功；false=失敗（皆不可用且無快取）
static bool getHM(int &h, int &m){
  static bool     haveCache = false;
  static int      cacheH = 0, cacheM = 0;
  static uint32_t lastOkMs = 0;

  struct tm t;
  // 1) 系統時鐘（NTP 校時後最精準）
  if (getLocalTime(&t, 200)) { // 200ms 等待
    h = t.tm_hour; m = t.tm_min;
    cacheH = h; cacheM = m; haveCache = true; lastOkMs = millis();
    return true;
  }
  // 2) 退回 RTC
  if (gRtcReady) {
    YqDateTime n = RTC.now();
    h = n.hour; m = n.minute;
    cacheH = h; cacheM = m; haveCache = true; lastOkMs = millis();
    return true;
  }
  // 3) 再退回 /rtc.txt 快照
  String snap = readTextFile("/rtc.txt");
  if (snap.length() >= 16) {
    h = snap.substring(11,13).toInt();
    m = snap.substring(14,16).toInt();
    cacheH = h; cacheM = m; haveCache = true; lastOkMs = millis();
    return true;
  }
  // 4) 最後：若 60 秒內剛成功過，就沿用快取
  if (haveCache && (millis() - lastOkMs) < 60000UL) {
    h = cacheH; m = cacheM;
    return true;
  }
  return false;
}

// 工具：tm_wday(0=Sun..6=Sat) 轉 Mon=0..Sun=6
static inline int weekdayMon0_from_tm(const tm& t){
  int sun0 = t.tm_wday;
  return (sun0 == 0) ? 6 : (sun0 - 1);
}

// 本日是否在星期遮罩允許內（未取到時間時一律放行，以免整機停擺）
static bool weekdayEnabled(){
  struct tm tinfo;
  if (!getLocalTime(&tinfo, 50)) return true; // 放行
  int w = weekdayMon0_from_tm(tinfo);         // 0..6
  return ((cfg.wdMask >> w) & 0x01) != 0;
}


// =========================【排程觸發：每分鐘比對 schedulerLoop】=========================
// 用法：在 loop() 內高頻呼叫；本函式內已做 0.5 秒節流
// 作用：當前 HH:MM 符合任一路排程 → 推播對應訊息 + 啟動對應繼電器保持
// 去重：以 curKey(HH*60+MM) + lastTrigKey[i] 避免同分鐘重複觸發
unsigned long lastMinTick = 0;
int curH_cache = -1, curM_cache = -1;
uint8_t lastTriggeredMinute[RELAY_COUNT] = {255,255,255,255,255,255}; //（保留相容）
int16_t lastTrigKey[RELAY_COUNT] = { -1,-1,-1,-1,-1,-1 };

void schedulerLoop(){
  static unsigned long lastTick = 0;
  if (millis() - lastTick < 500) return;   // 0.5 秒節流
  lastTick = millis();

  int curH, curM;
  if (!getHM(curH, curM)) return;

  // （可用於除錯展示 dow；目前邏輯未強制依賴）
  int dow = -1;
  struct tm t;
  if (getLocalTime(&t, 50)) {
    dow = (t.tm_wday + 6) % 7; // Mon=0 … Sun=6
  } else if (gRtcReady) {
    YqDateTime n = RTC.now();
    // 若要嚴謹判斷 dow，可在校時時順便快照 dow
  }

  int curKey = curH * 60 + curM;

  for (int i = 0; i < RELAY_COUNT; ++i) {
    if (!weekdayEnabled()) continue;   // 今天未勾選 → 跳過全排程

    if (curH == cfg.sch[i].hh && curM == cfg.sch[i].mm) {
      if (lastTrigKey[i] != curKey) {  // 同分鐘不重複
        lastTrigKey[i] = curKey;

        Serial.printf("[SCH] CH%d match %02d:%02d, hold=%us\n",
                      i+1, curH, curM, (unsigned)cfg.sch[i].hold);

        // 推播當路自訂訊息（ON 文案）
        tgEnqueue(cfg.sch[i].msg);
        uiShow("SCH CH"+String(i+1)+" 開始", "保持 "+String(cfg.sch[i].hold)+"s");

        // 啟動對應繼電器（非阻塞狀態管理）
        startRelayTimed(i, cfg.sch[i].hold);
      }
    }
    // 否則：不重設 lastTrigKey[i]；下個分鐘 curKey 變動自然可再觸發
  }
}


// =========================【選用：RTC 鬧鐘中斷】=========================
// 用法：若有接 RTC INT 腳並啟用鬧鐘，可在此 ISR 置旗標，主迴圈再消化
void IRAM_ATTR rtcIntIsr(){
  // 僅置旗標，避免在 ISR 做重工作
}
volatile bool gRtcAlarm = false;

// Forward declarations（若 handler 定義在後面）
void handleSelfTest();   // 自檢
void handleDiag();       // 診斷頁（之後補實作）


// =========================【心跳燈（LEDC）】=========================
// 用法：setup() 初始化 LEDC 後，在 loop() 週期性呼叫 heartbeatLoop()
// 作用：僅在 Wi-Fi 已連線（WL_CONNECTED）時顯示「兩閃一停」心跳節奏
static const int HB_PIN = 2;
static const int HB_CH  = 4;    // 任一可用 LEDC channel
static const int HB_HZ  = 1000;

static unsigned long HB_ON1_MS   = 200;
static unsigned long HB_OFF1_MS  = 200;
static unsigned long HB_ON2_MS   = 200;
static unsigned long HB_PAUSE_MS = 650;
static uint8_t       HB_BRIGHT   = 120;  // 0..255

enum HbStage {HB_ON1, HB_OFF1, HB_ON2, HB_PAUSE};
static HbStage       hbStage = HB_ON1;
static unsigned long hbNext  = 0;

static inline void hbSet(uint8_t duty){
  ledcWrite(HB_CH, duty); // 0~255
}

void heartbeatLoop(){
  // 僅在已連線狀態顯示心跳；未連線時熄燈
  if (WiFi.status() != WL_CONNECTED) {
    hbSet(0);
    return;
  }

  unsigned long now = millis();
  if (now < hbNext) return;

  switch(hbStage){
    case HB_ON1:   hbSet(HB_BRIGHT); hbNext = now + HB_ON1_MS;   hbStage = HB_OFF1;  break;
    case HB_OFF1:  hbSet(0);         hbNext = now + HB_OFF1_MS;  hbStage = HB_ON2;   break;
    case HB_ON2:   hbSet(HB_BRIGHT); hbNext = now + HB_ON2_MS;   hbStage = HB_PAUSE; break;
    case HB_PAUSE: hbSet(0);         hbNext = now + HB_PAUSE_MS; hbStage = HB_ON1;   break;
  }
}
// ===== CORS 工具（外部瀏覽器直接呼叫裝置 API 會用到）=====
static inline void addCorsHeaders() {
  srv.sendHeader("Access-Control-Allow-Origin", "*");
  srv.sendHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
  srv.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}
// =========================【系統初始化 setup()】=========================
void setup() {
  Serial.begin(115200);
  delay(100);

  // --- 顯示與推播 ---
  u8g2.begin();  // 初始化 OLED
  oledKick("boot");  // ★ 開機先喚醒（並初始化時間點）
  tgQ = xQueueCreate(20, sizeof(TgMsg));  // 建立 Telegram 佇列
  xTaskCreatePinnedToCore(tgTask, "tgTask", 8192, nullptr, 1, nullptr, 0); // 建議跑 Core0

  // --- SPIFFS ---
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
  }

  // --- 心跳燈 LEDC ---
  ledcSetup(HB_CH, HB_HZ, 8);
  ledcAttachPin(HB_PIN, HB_CH);
  hbSet(0);

  // --- 載入設定檔 ---
  loadConfig();

  // --- 繼電器腳位 ---
  for (int i = 0; i < RELAY_COUNT; ++i) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], RELAY_ACTIVE_HIGH ? LOW : HIGH);
  }

  // --- DI 訊息初始化 ---
  for (int i = 0; i < ALARM_COUNT; i++) {
    gAlarmMsg[i] = cfg.aMsg[i].length() ? cfg.aMsg[i] : gAlarmMsg[i];
  }
  for (int i = 0; i < ALARM_COUNT; i++) {
    pinMode(ALARM_PINS[i], INPUT_PULLUP);   // 低有效，GPIO 對 GND
    gAlarmLast[i] = digitalRead(ALARM_PINS[i]);
  }

  // --- 工件計數腳位 ---
  for (int i = 0; i < CNT_COUNT; i++) {
    pinMode(CNT_PINS[i], INPUT_PULLUP);    // 4/15 有內建上拉
    gCntLast[i]  = digitalRead(CNT_PINS[i]);
    gCntArmed[i] = true;                   // 開機先武裝
    int edge = CNT_ACTIVE_LOW ? FALLING : RISING;
    attachInterrupt(digitalPinToInterrupt(CNT_PINS[i]),
                    (i==0)? cnt0_isr : cnt1_isr, edge);
  }

  // --- RTC 初始化 ---
  gRtcReady = RTC.begin();
  if (!gRtcReady) {
    Serial.println("RTC begin failed（退回系統時間）");
  } else if (RTC.lostPower()) {
    Serial.println("[RTC] VL=1：曾掉電，嘗試載入 /rtc.txt 快照");
    String snap = readTextFile("/rtc.txt");
    snap.trim();
    if (snap.length() >= 16) {
      YqDateTime dt{};
      dt.year   = snap.substring(0,4).toInt();
      dt.month  = snap.substring(5,7).toInt();
      dt.day    = snap.substring(8,10).toInt();
      dt.hour   = snap.substring(11,13).toInt();
      dt.minute = snap.substring(14,16).toInt();
      dt.second = 0;
      RTC.adjust(dt);
      Serial.println("[RTC] 已用快照對時");
    } else {
      Serial.println("[RTC] 找不到快照，請到設定頁手動校時");
    }
  }
  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RTC_INT_PIN), [](){ gRtcAlarm = true; }, FALLING);

  // --- Wi-Fi 事件：取得 IP 時推播 ---
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      notifyOnline();  // 取得 IP（含重新連線）就推播
    }
  });

  // --- HTTP API：重設計數器 ---
  srv.on("/count-reset", HTTP_POST, [](){
    int ch = srv.hasArg("ch") ? srv.arg("ch").toInt() : 0;
    if (ch < 0 || ch >= CNT_COUNT){ srv.send(400,"text/plain","bad ch"); return; }

    noInterrupts();
    gCntIsr[ch] = 0;   // 清 ISR 快照
    interrupts();

    gCntShown[ch] = 0; // 清畫面側
    gCount[ch]    = 0; // 清對外顯示值
    srv.send(200,"text/plain; charset=utf-8","OK");
  });

  // --- Wi-Fi ---
  beginWiFi();
  Serial.print("IP: "); Serial.println(WiFi.localIP());  // AP 模式下會是 192.168.4.1

  // --- WebServer 路由綁定 ---
  srv.on("/",           HTTP_GET,  handleRoot);
  srv.on("/save",       HTTP_POST, handleSave);
  srv.on("/set-time",   HTTP_POST, handleSetTime);
  srv.on("/test-relay", HTTP_GET,  handleTestRelay);
  srv.on("/self-test",  HTTP_GET,  handleSelfTest);
  srv.on("/diag",       HTTP_GET,  handleDiag);
  srv.on("/tg",         HTTP_GET, [](){
    String text = srv.hasArg("text") ? srv.arg("text") : "ping";
    bool ok = sendTelegram("[/tg] " + text);
    srv.send(200, "text/plain", ok ? "sent" : "fail");
  }); 
  // 允許瀏覽器預檢
srv.on("/webapp-save", HTTP_OPTIONS, [](){
  addCorsHeaders();
  srv.send(200, "text/plain", "");
});

// 外部瀏覽器可直接 POST JSON 到這裡
// Header: Content-Type: application/json
// Body:   你的設定 JSON（與 Telegram WebApp 的 payload 內容一致）
srv.on("/webapp-save", HTTP_POST, [](){
  addCorsHeaders();
  String body = srv.arg("plain");
  if (!body.length()) {
    srv.send(400, "application/json", "{\"ok\":false,\"msg\":\"no body\"}");
    return;
  }
  Serial.printf("[CFG] /webapp-save %d bytes\n", body.length());
  tgEnqueue("🛰 收到外部瀏覽器設定，開始套用…");
  applyWebAppConfig(body);       // ← 直接沿用你現有的解析邏輯
  srv.send(200, "application/json", "{\"ok\":true}");
});

    // 即時設定 OLED 休眠秒數：/set-oled-sleep?sec=120
  srv.on("/set-oled-sleep", HTTP_GET, [](){
    if (!srv.hasArg("sec")) { srv.send(400, "text/plain", "need sec"); return; }
    long sec = srv.arg("sec").toInt();
    if (sec < 5) sec = 5;
    if (sec > 3600) sec = 3600;
    gOledSleepMs = (uint32_t)sec * 1000UL;
    saveConfig();                          // ★ 同步存檔
    oledKick("api");                       // ★ 修改當下喚醒
    srv.send(200, "text/plain; charset=utf-8", "OK, sleep=" + String(sec) + "s");
  });
  srv.on("/panel", HTTP_GET, [](){
    if (!cfg.chat.length()) { srv.send(400,"text/plain","no chat"); return; }
    tgHideKeyboard(cfg.chat);         // 先把舊的收掉
    tgSendControlKeyboard(cfg.chat);  // 再送新的 WebApp 鍵（已啟動10秒自動關）
    tgSendInlineOpen(cfg.chat);       // 額外送一顆 Inline WebApp 按鈕（不殘留）
    srv.send(200, "text/plain; charset=utf-8", "OK");
  });
  

// 單獨送一顆 Inline WebApp 按鈕（不殘留、最保險）
srv.on("/open", HTTP_GET, [](){
  if (!cfg.chat.length()) { srv.send(400,"text/plain","no chat"); return; }
  tgSendInlineOpen(cfg.chat);
  srv.send(200, "text/plain; charset=utf-8", "OK");
});


  srv.begin();
  Serial.println("WebServer started");
}


// =========================【診斷頁 handleDiag】=========================
// 用法：HTTP GET /diag
// 作用：輸出目前時間、Wi-Fi 狀態、RTC 狀態、各路繼電器狀態、計數器狀態
void handleDiag(){
  struct tm t;
  bool got = getLocalTime(&t, 10);  // 嘗試抓目前系統時間

  String s;
  s += "NTP: " + String(got ? "OK" : "NG") + "\n";
  if (got){
    char buf[40];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);
    s += "Now: "; s += buf; s += "\n\n";
  } else {
    s += "Now: <no system time>\n\n";
  }

  for (int i = 0; i < RELAY_COUNT; ++i){
    char hhmm[6];
    snprintf(hhmm, sizeof(hhmm), "%02d:%02d", cfg.sch[i].hh, cfg.sch[i].mm);
    s += "CH"; s += (i+1);
    s += " -> "; s += hhmm;
    s += "  hold="; s += cfg.sch[i].hold; s += "s  msg="; s += cfg.sch[i].msg; s += "\n";
  }

  s += "\nWiFiMode: ";
  wifi_mode_t md = WiFi.getMode();
  s += (md==WIFI_AP)?"AP":(md==WIFI_STA)?"STA":(md==WIFI_AP_STA)?"AP+STA":"OFF";
  s += "  IP="; s += safeIP(); s += "\n";

  s += "RTC Ready: "; s += gRtcReady?"YES":"NO"; s += "\n";

  for (int i = 0; i < RELAY_COUNT; ++i){
    s += "CH"; s += (i+1);
    s += gTestActive[i] ? " ACTIVE" : " idle";
    if (gTestActive[i]) {
      long msLeft = (long)(gTestUntil[i] - millis());
      s += "  left="; s += (msLeft>0?msLeft:0); s += "ms";
    }
    s += "\n";
  }

  s += "\n[Counter]\n";
  for(int i=0;i<CNT_COUNT;i++){
    s += "CNT"; s += i;
    s += " count="; s += gCount[i];
    s += " time="; s += fmt2(cfg.cnt[i].hh); s += ":"; s += fmt2(cfg.cnt[i].mm);
    s += " msg="; s += cfg.cnt[i].msg;
    s += " target="; s += cfg.cnt[i].target;  // ★ 新增
    s += "\n";
  }

  srv.send(200, "text/plain; charset=utf-8", s);
}

// =========================【主循環 loop()】=========================
void loop() {
  // ---------- AP 觸發鍵（長按切 AP + 冷卻） ----------
  // 用法：長按 AP_MODE_PIN 進 AP；已連線需長按 5s，未連線 1.2s；切換後 30s 冷卻
  static unsigned long apSenseStart = 0;
  static unsigned long apLastToggleMs = 0;
  const unsigned long AP_COOLDOWN_MS = 30000;         // 30 秒冷卻
  const unsigned long AP_HOLD_MS_CONNECTED    = 5000; // 已連線：必須長按 5 秒
  const unsigned long AP_HOLD_MS_DISCONNECTED = 1200; // 未連線：1.2 秒

  bool apPinActive  = (digitalRead(AP_MODE_PIN) == (AP_ACTIVE_LOW ? LOW : HIGH));
  bool staConnected = (WiFi.status() == WL_CONNECTED);
  unsigned long needHold = staConnected ? AP_HOLD_MS_CONNECTED : AP_HOLD_MS_DISCONNECTED;

  if (apPinActive) {
    if (apSenseStart == 0) apSenseStart = millis();
    if ((millis() - apSenseStart) > needHold) {
      if (millis() - apLastToggleMs > AP_COOLDOWN_MS) {
        if (WiFi.getMode() != WIFI_AP) {
          oledKick("ap-key");               // ★ 長按即將切 AP → 喚醒
          Serial.println("[WiFi] 長按觸發 → 切換到 AP 模式");
          startAP();
          apLastToggleMs = millis();
        }
      }
    }
  } else {
    apSenseStart = 0;
  }

  // ---------- HTTP 服務（維持即時回應） ----------
  srv.handleClient();
  tgUpdatePollLoop();  // ★ 輪詢 Telegram，接收 WebApp 回傳設定
  // ★ 10 秒自動關閉鍵盤
if (gKbHideAt && (long)(millis() - gKbHideAt) >= 0) {
  gKbHideAt = 0;
  if (cfg.chat.length()) tgHideKeyboard(cfg.chat);
}


  // ---------- Wi-Fi 看門狗（每 3 秒輕量重連，避免干擾 AP 手動模式） ----------
  {
    static unsigned long lastWiFiCheck = 0;
    if (millis() - lastWiFiCheck >= 3000) {
      lastWiFiCheck = millis();
      if (WiFi.getMode() == WIFI_STA || WiFi.getMode() == WIFI_AP_STA) {
        if (WiFi.status() != WL_CONNECTED && cfg.ssid.length()) {
          WiFi.disconnect(false);
          WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());
        }
      }
    }
  }

  // ---------- 心跳燈（僅連線狀態顯示兩閃一停） ----------
  heartbeatLoop();

  // =========================【工件計數 re-arm】=========================
  // 用法：每圈檢查；只有在輸入回到「閒置電平」且去抖完成才重新武裝（允許下一次計數）
  for (int ci = 0; ci < CNT_COUNT; ++ci) {
    int v = digitalRead(CNT_PINS[ci]);
    if (v != gCntLast[ci]) {
      gCntLast[ci] = v;
      gCntDeb[ci]  = millis();                 // 電平變化 → 重置去抖計時
    } else {
      int idleLevel = CNT_ACTIVE_LOW ? HIGH : LOW; // 閒置電平
      if (!gCntArmed[ci] && v == idleLevel && (millis() - gCntDeb[ci]) >= CNT_DEBOUNCE) {
        gCntArmed[ci] = true;                  // 放開且穩定於閒置電平 → 允許下一次計數
      }
    }
  }

  // ---------- OLED 畫面（AP 顯示 SETUP；STA 顯示 IP 等） ----------
  bool forceSetup = (WiFi.getMode() == WIFI_AP);
  drawOled(forceSetup);

  // ---------- 延後關 AP（成功頁 5s 後只留 STA） ----------
  if (gCloseApAt && millis() >= gCloseApAt) {
    gCloseApAt = 0;
    Serial.println("[WiFi] 關閉 AP，改為僅 STA");
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
  }

  // ---------- 分鐘級排程 ----------
  schedulerLoop();

  // ---------- RTC 鬧鐘旗標清除 ----------
  if (gRtcAlarm) {
    gRtcAlarm = false;
    RTC.clearAlarmFlag();
  }

  // =========================【繼電器保持收斂（非阻塞）】=========================
  // 正常收斂：時間到即釋放；保險收斂：超過 hold+5s 強制釋放
  for (int ch = 0; ch < RELAY_COUNT; ++ch) {
    // 正常收斂
    if (gTestActive[ch] && (long)(millis() - gTestUntil[ch]) >= 0) {
      int pin = RELAY_PINS[ch];
      digitalWrite(pin, RELAY_ACTIVE_HIGH ? LOW : HIGH);
      tgEnqueue(endMsg(ch));
      uiShow("CH"+String(ch+1)+" 結束", "");
      gTestActive[ch] = false;
    }
    // 保險收斂
    if (gTestActive[ch]) {
      unsigned long holdMs = (unsigned long)cfg.sch[ch].hold * 1000UL;
      if (millis() - gTestStart[ch] > holdMs + 5000UL) {
        int pin = RELAY_PINS[ch];
        digitalWrite(pin, RELAY_ACTIVE_HIGH ? LOW : HIGH);
        tgEnqueue(endMsg(ch));
        uiShow("CH"+String(ch+1)+" 結束", "");
        gTestActive[ch] = false;
      }
    }
  }

  // =========================【異常 DI 監看】=========================
  // 低有效、去彈跳；LOW 觸發推播一次，回 HIGH 解除鎖存
  for (int ai = 0; ai < ALARM_COUNT; ++ai) {
    int v = digitalRead(ALARM_PINS[ai]);
    if (v != gAlarmLast[ai]) {
      gAlarmDebounceMs[ai] = millis();
      gAlarmLast[ai] = v;
    } else if ((millis() - gAlarmDebounceMs[ai]) > ALARM_DEBOUNCE) {

      if (v == LOW && !gAlarmLatched[ai]) {
        gAlarmLatched[ai] = true;
        oledKick("di");                            // ★ DI 觸發 → 喚醒
        tgEnqueue("⚠️ DI" + String(ai+1) + "：" + gAlarmMsg[ai]);
      }
      if (v == HIGH && gAlarmLatched[ai]) {
        gAlarmLatched[ai] = false;
        // 如需「恢復通知」可在此 enqueue
      }
    }
  }

  // =========================【工件計數：從 ISR 快照同步到顯示/對外值】=========================
  {
    static unsigned long last = 0;

    // 若有外部要求歸零則立即同步（兩路）
    if (gCntResetReq[0]) { gCntResetReq[0] = false; gCntShown[0] = 0; gCount[0] = 0; }
    if (gCntResetReq[1]) { gCntResetReq[1] = false; gCntShown[1] = 0; gCount[1] = 0; }

    if (millis() - last >= 10) {  // 10ms 節流
      last = millis();
      noInterrupts();
      uint32_t s0 = gCntIsr[0], s1 = gCntIsr[1];  // 取快照
      interrupts();
    // ★★ 在同步前先判斷是否「變多」：只要任一路快照比目前顯示值大，就代表有新計數
     bool cntChanged = (s0 > gCntShown[0]) || (s1 > gCntShown[1]);
     if (cntChanged) oledKick("count");  // ★ 計數有變 → 喚醒 OLED

      while (gCntShown[0] < s0) { gCntShown[0]++; Serial.printf("[CNT0] +1 -> %lu\n", (unsigned long)gCntShown[0]); }
      while (gCntShown[1] < s1) { gCntShown[1]++; Serial.printf("[CNT1] +1 -> %lu\n", (unsigned long)gCntShown[1]); }

      gCount[0] = gCntShown[0];
      gCount[1] = gCntShown[1];
    }
  }

  // =========================【工件計數：推播規則】=========================
  // (#1) 達標即推播（cn0>0），推播後把 #1 完整清零（ISR/顯示/對外）→ 可反覆達標
  int h, m;
  if (cfg.cnt[0].target > 0) {
    noInterrupts();
    uint32_t snap0 = gCntIsr[0];
    interrupts();

    if (snap0 >= cfg.cnt[0].target) {
      noInterrupts();
      uint32_t qty = gCntIsr[0];  // 取量
      gCntIsr[0]   = 0;           // 清 ISR 計數
      interrupts();

      gCntShown[0] = 0;           // 同步清畫面
      gCount[0]    = 0;           // 同步清對外

      String msg = cfg.cnt[0].msg + " 數量=" + String(qty) + "（達標）";
      tgEnqueue(msg);
      uiShow("CNT#1 達標", "數量="+String(qty));

    }
  }

  // (#2) 每日定時回報（兩路；但若 #1 啟用達標模式，就略過 #1 的每日回報）
  if (getHM(h, m)) {
    int key = h * 60 + m;
    for (int ci = 0; ci < CNT_COUNT; ++ci) {
      if (ci == 0 && cfg.cnt[0].target > 0) continue; // #1 啟用達標模式 → 略過每日回報
      if (!weekdayEnabled()) continue;
      if (h == cfg.cnt[ci].hh && m == cfg.cnt[ci].mm) {
        static int16_t lastCntKey[CNT_COUNT] = {-1, -1}; // 同分鐘去重
        if (lastCntKey[ci] == key) continue;
        lastCntKey[ci] = key;

        noInterrupts();
        uint32_t qty = gCntIsr[ci];  // 取快照
        gCntIsr[ci] = 0;             // 清 ISR 計數
        gCntShown[ci] = 0;           // 同步清畫面
        gCount[ci]    = 0;           // 同步清顯示
        interrupts();

        if (qty == 0) continue;      // 0 不推播

        // 若其他地方剛要求歸零，也在這裡消除旗標（雙保險）
        if (gCntResetReq[0]) { gCntShown[0] = 0; gCntResetReq[0] = false; }
        if (gCntResetReq[1]) { gCntShown[1] = 0; gCntResetReq[1] = false; }

        String msg = cfg.cnt[ci].msg + " 數量=" + String(qty);
        tgEnqueue(msg);
        uiShow("CNT#"+String(ci+1)+" 回報", "數量="+String(qty));
      }
    }
  }
}

// ====== OLED 顯示（獨立函式；不要放在 loop() 裡）======
void drawOled(bool forceSetup){
  // ----- 省電：逾時未操作 → 關閉 OLED；有活動 → 自動點亮 -----
  unsigned long now = millis();
  bool sleeping = (gOledSleepMs > 0) && ((now - gOledLastKick) >= gOledSleepMs);

  if (!forceSetup && sleeping) {
    if (!gOledPowerSave) {
      u8g2.clearBuffer();
      u8g2.sendBuffer();       // 清一次
      u8g2.setPowerSave(1);    // ★ 關閉 OLED 面板
      gOledPowerSave = true;
    }
    return;                    // ★ 睡眠中不再繪製
  } else {
    if (gOledPowerSave) {
      u8g2.setPowerSave(0);    // ★ 醒來立刻點亮
      gOledPowerSave = false;
    }
  }
  u8g2.clearBuffer();

  // 標題列
  u8g2.setFont(u8g2_font_8x13_tf);
  u8g2.drawStr(0, 12, "Y&Q_Notify");

  if (forceSetup) {
    // AP 設定模式
    u8g2.setFont(u8g2_font_logisoso18_tf);  u8g2.drawStr(0, 40, "SETUP MODE");
    u8g2.setFont(u8g2_font_7x13_tf);        u8g2.drawStr(8, 60, "10.10.0.1");
    u8g2.sendBuffer();
    return;
  }

  // ====== 主體內容（即時訊息優先）======
  bool hasNote = (gUi.until && millis() < gUi.until);

  if (WiFi.status() == WL_CONNECTED) {
    // 第一行：IP + WiFi Bars
    String ip = WiFi.localIP().toString();
    long rssi  = WiFi.RSSI();
    int  bars  = rssiBars(rssi);

    char top[32];
    snprintf(top, sizeof(top), "IP %s", ip.c_str());

    u8g2.setFont(u8g2_font_7x13_tf);
    u8g2.drawStr(0, 28, top);

    // 右側畫 WiFi 條（簡化 4 格）
    int x0 = 118, y0 = 28; // 右上角靠近
    for (int i=0;i<4;i++){
      int h = 3 + i*2;     // 一根比一根高
      int x = x0 + i - 4;  // 挨在一起
      int y = y0;
      if (i < bars) u8g2.drawBox(x, y-h, 2, h);
      else          u8g2.drawFrame(x, y-h, 2, h);
    }

    if (hasNote) {
      // 中段顯示即時事件（大字 + 小字）
      u8g2.setFont(u8g2_font_logisoso18_tf);
      u8g2.drawStr(0, 48, gUi.line1);
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(0, 62, gUi.line2);
    } else {
      // 無即時事件 → 顯示系統狀態：繼電器 / DI / 計數
      // 行2：繼電器狀態（哪幾路正在保持）
      u8g2.setFont(u8g2_font_6x10_tf);
      String rel = "REL:";
      for (int i=0;i<RELAY_COUNT;i++){
        rel += gTestActive[i] ? String(i+1) : String("-");
      }
      u8g2.drawStr(0, 44, rel.c_str());

      // 行3：DI 狀態（低有效：LOW=!）
      String di = "DI :";
      for (int i=0;i<ALARM_COUNT;i++){
        int v = digitalRead(ALARM_PINS[i]);
        di += (v==LOW) ? "!" : ".";
      }
      u8g2.drawStr(0, 56, di.c_str());

      // 行4：計數顯示（即時值）
      char cnt[32];
      snprintf(cnt, sizeof(cnt), "C1:%lu  C2:%lu",
               (unsigned long)gCount[0], (unsigned long)gCount[1]);
      u8g2.drawStr(0, 68-4, cnt);  // 微上移避免出界
    }
  } else {
    // 未連線：顯示 NO WIFI 與 AP 提示
    u8g2.setFont(u8g2_font_logisoso18_tf);  u8g2.drawStr(0, 42, "NO WIFI");
    u8g2.setFont(u8g2_font_6x10_tf);        u8g2.drawStr(0, 60, "按鍵長按進入 AP");
  }

  u8g2.sendBuffer();
}

