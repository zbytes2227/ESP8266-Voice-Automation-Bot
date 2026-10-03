/*
 * ============================================================
 *  ESP8266 VOICE AUTOMATION BOT  v2
 *  Voice Recognition V3 + OLED UI + HTTP Action Dispatcher
 * ============================================================
 *  All v1 functionality preserved exactly:
 *    - Pins, OLED UI, 3-button debounce interface
 *    - VR3 training / loading / recognition (packet parser UNCHANGED)
 *    - CRC-protected dual-slot WiFi storage
 *    - Serial debug console
 *
 *  v2 additions:
 *    - Per-record (R0-R79) HTTP GET/POST action config
 *    - LittleFS binary config storage (/cmd/rNN.bin, one file/record)
 *    - Priority-ordered auto-load of enabled+trained records at boot
 *    - HTTP action executor:  recognition -> GET/POST -> OLED result
 *    - Always-on web config portal when WiFi connected
 *    - Rich OLED state machine (BOOTING/VR ONLINE/LISTENING/etc.)
 *
 *  Serial commands:
 *    h help | s state | p ping VR | m map | d deep diag
 *    c dump active cfg | f reload cfg from flash | r reboot
 *
 *  Only proven VR library APIs used (no invented calls):
 *    begin, train, load, clear, checkRecord, checkRecognizer,
 *    receive_pkt, flushInput, VR::errorString
 *
 *  New library requirements (all part of standard ESP8266 Arduino core):
 *    ESP8266HTTPClient, WiFiClient
 * ============================================================
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include <stdarg.h>
#include "VoiceRecognitionV3-ESP8266.h"

// ============================================================
// PIN MAP  (unchanged from v1)
// ============================================================
#define OLED_SDA       D2
#define OLED_SCL       D1
#define OLED_ADDR_A    0x3C
#define OLED_ADDR_B    0x3D

#define VR_RX          D5       // ESP RX <- VR3 TX (use level divider)
#define VR_TX          D6       // ESP TX -> VR3 RX
#define VR_BAUD        9600

#define BTN_UP         D3       // GPIO0  (don't hold during boot)
#define BTN_DOWN       D7
#define BTN_SELECT     D0       // GPIO16 (NO internal pull-up: external 10k to 3V3 REQUIRED)

#define BATTERY_PIN       A0
#define ADC_FULLSCALE_V   3.20f
#define BATT_DIVIDER      2.0f

#define VR_MAX_RECORD  80
#define VR_MAX_ACTIVE  7

// ============================================================
// v2 CONSTANTS
// ============================================================
#define RC_MAGIC       0x52434602UL   // "RCFv2" - change if struct layout changes
#define RC_NAME_LEN    24
#define RC_URL_LEN     128
#define RC_BODY_LEN    64
#define HTTP_TIMEOUT   5000           // ms for outgoing HTTP requests

// ============================================================
// TYPES  (original types first, then v2 additions)
// ============================================================
enum Button { NO_BUTTON, UP_PRESSED, DOWN_PRESSED, SELECT_PRESSED };

struct BtnState {
  uint8_t  pin;
  bool     raw, stable;
  uint32_t changedAt, pressedAt, lastRepeat;
};

struct WiFiConfig {
  uint32_t magic, sequence;
  char     ssid[33], password[65];
  uint32_t crc;
};

struct RecogResult {
  int     len;
  uint8_t cmd, sub, group, index, sigLen;
  uint8_t sig[16];
  uint8_t physical;
};

#define PARSE_MATCH    0
#define PARSE_NOMATCH  1
#define PARSE_INVALID  2
#define PARSE_BADINDEX 3

#define UI_LISTEN  0
#define UI_MATCH   1
#define UI_NOMATCH 2
#define UI_BAD     3
#define UI_BADIDX  4

struct RecUI {
  uint8_t  state, record, index;
  uint32_t hits, nomatch, bad, timeouts;
  int      lastRecord;
  uint32_t lastEventMs;
};

// v2: Per-record action config stored in LittleFS (/cmd/rNN.bin)
// sizeof = 4+24+128+64+1+1+1+1+4 = 228 bytes
struct RecordConfig {
  uint32_t magic;               // RC_MAGIC or 0 = unconfigured
  char     name[RC_NAME_LEN];   // human label e.g. "STOP"
  char     url[RC_URL_LEN];     // HTTP endpoint URL
  char     body[RC_BODY_LEN];   // POST body (empty string for GET)
  uint8_t  method;              // 0=GET, 1=POST
  uint8_t  enabled;             // 0=disabled, 1=enabled
  uint8_t  priority;            // 0=highest priority for auto-load
  uint8_t  _pad;                // struct alignment pad
  uint32_t crc;                 // CRC32 over all preceding bytes
};

// v2: In-RAM cache of configs for currently active records (max 7)
struct ActiveCfg {
  uint8_t record;
  char    name[RC_NAME_LEN];
  char    url[RC_URL_LEN];
  char    body[RC_BODY_LEN];
  uint8_t method;
  bool    enabled;
};

// ============================================================
// LOGGING  (Serial 115200)  — unchanged
// ============================================================
static void logPrint(char lvl, const char *tag, const char *fmt, ...) {
  char msg[220];
  va_list ap; va_start(ap, fmt); vsnprintf(msg, sizeof(msg), fmt, ap); va_end(ap);
  Serial.printf("[%9lu][%c][%-6s] %s\n", (unsigned long)millis(), lvl, tag, msg);
}
#define LOGE(tag,...) logPrint('E', tag, __VA_ARGS__)
#define LOGW(tag,...) logPrint('W', tag, __VA_ARGS__)
#define LOGI(tag,...) logPrint('I', tag, __VA_ARGS__)
#define LOGD(tag,...) logPrint('D', tag, __VA_ARGS__)

static void logHex(const char *tag, const char *label, const uint8_t *d, int n) {
  Serial.printf("[%9lu][D][%-6s] %s (%d bytes):", (unsigned long)millis(), tag, label, n);
  for (int i = 0; i < n; i++) Serial.printf(" %02X", d[i]);
  Serial.println();
}

static const char *wlStrShim(int s) {
  switch (s) {
    case WL_CONNECTED:      return "CONNECTED";
    case WL_NO_SSID_AVAIL:  return "NO_SSID";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED";
    case WL_WRONG_PASSWORD: return "WRONG_PASSWORD";
    case WL_IDLE_STATUS:    return "IDLE";
    case WL_DISCONNECTED:   return "DISCONNECTED";
    default:                return "UNKNOWN";
  }
}

// ============================================================
// GLOBALS
// ============================================================
Adafruit_SSD1306 display(128, 64, &Wire, -1);
VR myVR(VR_RX, VR_TX);

uint8_t vrPacket[96];
uint8_t activeRecords[8];
uint8_t activeRecordCount = 0;
bool    activeMapValid    = false;
bool    vrOnline          = false;
bool    vrBusy            = false;
int8_t  trainedState[VR_MAX_RECORD];  // -1 unknown, 0 empty, 1 trained

// v2: active config cache
ActiveCfg activeCfg[VR_MAX_ACTIVE];
uint8_t   activeCfgCount = 0;

ESP8266WebServer server(80);
DNSServer        dnsServer;
bool     portalRunning = false;
bool     portalSaved   = false;
uint32_t portalCloseAt = 0;
bool     handlersSet   = false;
bool     serverRunning = false;   // v2: server always-on when WiFi up

const uint32_t WIFI_MAGIC = 0x42544F31UL;
const char *WIFI_A = "/wifi_a.bin";
const char *WIFI_B = "/wifi_b.bin";
WiFiConfig wifiConfig;
bool wifiConfigValid = false;

static BtnState btnS[3] = {
  { BTN_UP,     HIGH, HIGH, 0, 0, 0 },
  { BTN_DOWN,   HIGH, HIGH, 0, 0, 0 },
  { BTN_SELECT, HIGH, HIGH, 0, 0, 0 }
};
static const char *btnNames[3] = { "UP", "DOWN", "SELECT" };

// ============================================================
// BUTTONS  (debounced, non-blocking, auto-repeat on UP/DOWN)
// — unchanged from v1
// ============================================================
Button readButton() {
  static const Button ids[3] = { UP_PRESSED, DOWN_PRESSED, SELECT_PRESSED };
  uint32_t now = millis();
  for (uint8_t i = 0; i < 3; i++) {
    BtnState &b = btnS[i];
    bool raw = digitalRead(b.pin);
    if (raw != b.raw) { b.raw = raw; b.changedAt = now; }
    if (raw != b.stable && (now - b.changedAt) >= 25) {
      b.stable = raw;
      if (raw == LOW) {
        b.pressedAt = now; b.lastRepeat = now;
        LOGD("BTN", "%s pressed", btnNames[i]);
        return ids[i];
      }
    } else if (b.stable == LOW && i < 2 &&
               (now - b.pressedAt) > 500 && (now - b.lastRepeat) > 90) {
      b.lastRepeat = now;
      return ids[i];
    }
  }
  return NO_BUTTON;
}

// ============================================================
// OLED HELPERS  (128x64, text size 1 = 21 cols x 8 rows)
// — unchanged from v1 except oledBigState() added
// ============================================================
static void oledClear() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
}

static void drawHeader(const char *title, const char *right = "") {
  display.fillRect(0, 0, 128, 9, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 1);
  display.print(title);
  if (right && right[0]) {
    int w = strlen(right) * 6;
    display.setCursor(126 - w, 1);
    display.print(right);
  }
  display.setTextColor(SSD1306_WHITE);
}

static void drawFooter(const char *text) {
  display.drawFastHLine(0, 54, 128, SSD1306_WHITE);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(text);
}

static uint8_t drawWrapped(int16_t x, int16_t y, const String &s,
                           uint8_t cols, uint8_t maxRows) {
  uint8_t row = 0; uint16_t i = 0;
  while (i < s.length() && row < maxRows) {
    display.setCursor(x, y + row * 8);
    display.print(s.substring(i, i + cols));
    i += cols; row++;
  }
  return row;
}

static void showMessage(const String &a, const String &b = "",
                        uint16_t ms = 1500) {
  LOGI("UI", "message: \"%s\" / \"%s\"", a.c_str(), b.c_str());
  oledClear();
  drawWrapped(0, 6,  a, 21, 2);
  drawWrapped(0, 30, b, 21, 3);
  display.display();
  delay(ms);
}

static void drawProgress(const char *title, const String &line,
                         uint8_t pct, const char *hint) {
  oledClear(); drawHeader(title);
  display.setCursor(0, 16); display.print(line);
  display.drawRect(0, 30, 128, 12, SSD1306_WHITE);
  display.fillRect(2, 32, (uint16_t)pct * 124 / 100, 8, SSD1306_WHITE);
  display.setCursor(0, 46); display.print(pct); display.print("%");
  drawFooter(hint); display.display();
}

static void bootScreen(const char *text, uint8_t percent) {
  oledClear();
  display.setTextSize(2); display.println("BOT");
  display.setTextSize(1); display.setCursor(0, 25); display.println(text);
  display.drawRect(0, 44, 128, 11, SSD1306_WHITE);
  display.fillRect(2, 46, (uint16_t)percent * 124 / 100, 7, SSD1306_WHITE);
  display.setCursor(0, 57); display.print(percent); display.print("%");
  display.display();
  LOGI("BOOT", "%s (%u%%)", text, percent);
}

// v2: Two-line status screen for simple state display
static void oledBigState(const char *header, const char *line1,
                         const char *line2 = "", const char *footer = "SEL=menu") {
  oledClear(); drawHeader(header);
  display.setCursor(0, 14); display.print(line1);
  if (line2 && line2[0]) { display.setCursor(0, 24); display.print(line2); }
  drawFooter(footer); display.display();
}

// ============================================================
// VOICE MODULE CORE  — unchanged from v1
// ============================================================
static String vrRetStr(int ret) {
  if (ret >= 0) return String("ok");
  return String(VR::errorString(ret));
}

// checkRecord() on UNTRAINED record: module replies status 0x00, library
// reports VR_ERROR. That IS a valid response meaning "empty", not a comm failure.
static bool vrIsNotTrained(int ret) {
  return ret < 0 && vrRetStr(ret) == "VR_ERROR";
}

static String listActive() {
  if (!activeRecordCount) return String("none");
  String s;
  for (uint8_t i = 0; i < activeRecordCount; i++) {
    if (i) s += ","; s += String((int)activeRecords[i]);
  }
  return s;
}

static String fmtTrainedRanges() {
  String s; int i = 0;
  while (i < VR_MAX_RECORD) {
    if (trainedState[i] == 1) {
      int j = i;
      while (j + 1 < VR_MAX_RECORD && trainedState[j + 1] == 1) j++;
      if (s.length()) s += ",";
      s += String(i);
      if (j == i + 1)     { s += ","; s += String(j); }
      else if (j > i + 1) { s += "-"; s += String(j); }
      i = j + 1;
    } else { i++; }
  }
  if (!s.length()) s = "none";
  return s;
}

static int trainedCountKnown() {
  int c = 0;
  for (int i = 0; i < VR_MAX_RECORD; i++) if (trainedState[i] == 1) c++;
  return c;
}

static bool refreshActiveMap() {
  uint8_t rec[16]; memset(rec, 0xEE, sizeof(rec));
  uint32_t t0 = millis();
  int n = myVR.checkRecognizer(rec, sizeof(rec));
  uint32_t dt = millis() - t0;
  LOGD("VR", "checkRecognizer -> ret=%d (%s) in %lums",
       n, vrRetStr(n).c_str(), (unsigned long)dt);
  if (n < 0) {
    vrOnline = false; activeMapValid = false;
    LOGE("VR", "NO RESPONSE. Check: VR TX->D5, VR RX->D6, power, GND, baud %d", VR_BAUD);
    return false;
  }
  vrOnline = true;
  logHex("VR", "recognizer raw", rec, n > 16 ? 16 : n);
  if (n < 11) { LOGE("VR", "reply too short (%d, need >=11)", n); activeMapValid = false; return false; }
  if (rec[0] > 8) { LOGE("VR", "valid-count %u impossible (>8)", rec[0]); activeMapValid = false; return false; }
  activeRecordCount = rec[0];
  for (uint8_t i = 0; i < activeRecordCount; i++) activeRecords[i] = rec[i + 1];
  activeMapValid = true;
  LOGI("VR", "MAP: active=%u list=[%s] allRec=%u mask=0x%02X grp=0x%02X",
       activeRecordCount, listActive().c_str(), rec[8], rec[9], rec[10]);
  for (uint8_t i = 0; i < activeRecordCount; i++)
    LOGD("VR", "  recognizer index %u -> physical record %u", i, activeRecords[i]);
  uint8_t contiguous = (activeRecordCount >= 8) ? 0xFF : (uint8_t)((1 << activeRecordCount) - 1);
  if (rec[9] != contiguous)
    LOGD("VR", "note: valid-pos mask 0x%02X differs from contiguous 0x%02X (informational)", rec[9], contiguous);
  return true;
}

// Scans records 0..79. Returns trained count, -1=module not responding, -2=cancelled.
static int scanTrained() {
  LOGI("SCAN", "=== scanning %d records ===", VR_MAX_RECORD);
  uint32_t t0 = millis(); int count = 0, consecFails = 0;
  for (uint8_t r = 0; r < VR_MAX_RECORD; r++) {
    if ((r % 2) == 0) {
      char line[24]; snprintf(line, sizeof(line), "Checking R%02u/79", r);
      drawProgress("VOICE SCAN", String(line), (uint16_t)r * 100 / VR_MAX_RECORD, "SEL=cancel");
    }
    if (readButton() == SELECT_PRESSED) { LOGW("SCAN", "cancelled at R%u", r); return -2; }
    uint8_t st = 0xFF; uint32_t c0 = millis();
    int ret = myVR.checkRecord(r, &st); uint32_t dt = millis() - c0;
    if (ret < 0 && vrIsNotTrained(ret)) {
      consecFails = 0; vrOnline = true; trainedState[r] = 0;
      LOGD("SCAN", "R%02u empty (VR_ERROR=not trained) %lums", r, (unsigned long)dt);
    } else if (ret < 0) {
      trainedState[r] = -1; consecFails++;
      LOGE("SCAN", "R%02u FAILED ret=%d (%s) %lums [streak %d]",
           r, ret, vrRetStr(ret).c_str(), (unsigned long)dt, consecFails);
      if (consecFails >= 3) {
        vrOnline = false; LOGE("SCAN", "3 failures -> module not responding, aborting");
        return -1;
      }
    } else {
      consecFails = 0; vrOnline = true;
      trainedState[r] = (st == 0x01) ? 1 : 0;
      if (st == 0x01) count++;
      LOGD("SCAN", "R%02u status=0x%02X -> %s %lums",
           r, st, st == 0x01 ? "TRAINED" : "empty", (unsigned long)dt);
    }
    yield();
  }
  LOGI("SCAN", "=== done: %d trained [%s] in %lums ===",
       count, fmtTrainedRanges().c_str(), (unsigned long)(millis() - t0));
  return count;
}

// ============================================================
// SERIAL CONSOLE + BACKGROUND POLL
// — extended in v2 with 'c' (dump config) and 'f' (reload)
// ============================================================
static String serialLine;

static void printState() {
  LOGI("STATE", "uptime=%lus heap=%u chip=%06X cpu=%uMHz",
       (unsigned long)(millis() / 1000), ESP.getFreeHeap(),
       ESP.getChipId(), ESP.getCpuFreqMHz());
  LOGI("STATE", "wifi=%s ip=%s rssi=%d",
       wlStrShim(WiFi.status()), WiFi.localIP().toString().c_str(), WiFi.RSSI());
  LOGI("STATE", "vrOnline=%d vrBusy=%d mapValid=%d active=%u [%s] trained=%d [%s]",
       vrOnline, vrBusy, activeMapValid, activeRecordCount, listActive().c_str(),
       trainedCountKnown(), fmtTrainedRanges().c_str());
}

static void printHelp() {
  Serial.println(F("--- SERIAL COMMANDS ---"));
  Serial.println(F(" h  help       s  state       m  recognizer map"));
  Serial.println(F(" p  ping VR    d  deep diag   c  dump active cfg"));
  Serial.println(F(" f  reload cfg r  reboot"));
  Serial.println(F("-----------------------"));
}

// Forward declaration needed for 'f' command
static void rebuildActiveCache();

static void handleSerialCommand(String cmd) {
  cmd.trim(); if (!cmd.length()) return;
  char c = cmd.charAt(0);
  LOGI("CONSOLE", "command '%s'", cmd.c_str());
  bool needsVR = (c == 'p' || c == 'm' || c == 'd');
  if (needsVR && vrBusy) { LOGW("CONSOLE", "VR busy - ignored"); return; }
  if      (c == 'h' || c == '?') printHelp();
  else if (c == 's') printState();
  else if (c == 'p' || c == 'm') refreshActiveMap();
  else if (c == 'd') { refreshActiveMap(); int n = scanTrained(); LOGI("DIAG", "scan=%d", n); printState(); }
  else if (c == 'c') {
    LOGI("CONFIG", "active cache: %u entries", activeCfgCount);
    for (uint8_t i = 0; i < activeCfgCount; i++)
      LOGI("CONFIG", "  [%u] R%u \"%s\" %s \"%s\" en=%d",
           i, activeCfg[i].record, activeCfg[i].name,
           activeCfg[i].method ? "POST" : "GET",
           activeCfg[i].url, (int)activeCfg[i].enabled);
  }
  else if (c == 'f') { LOGI("CONFIG", "reloading cfg cache from flash"); rebuildActiveCache(); }
  else if (c == 'r') { LOGW("CONSOLE", "rebooting..."); delay(200); ESP.restart(); }
  else LOGW("CONSOLE", "unknown command (h=help)");
}

static void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (serialLine.length()) { handleSerialCommand(serialLine); serialLine = ""; }
    } else if (serialLine.length() < 24) { serialLine += ch; }
  }
}

// v2: pollBackground also pumps web server and DNS
static void pollBackground() {
  pollSerial();
  if (serverRunning) server.handleClient();
  if (portalRunning) dnsServer.processNextRequest();
  yield();
}

static bool waitSelect(uint32_t timeoutMs = 0) {
  uint32_t t = millis();
  while (true) {
    if (readButton() == SELECT_PRESSED) return true;
    if (timeoutMs && (millis() - t) > timeoutMs) return false;
    pollBackground(); delay(5);
  }
}

static void showVRError(const char *action, int ret) {
  LOGE("VR", "%s failed: ret=%d (%s)", action, ret, vrRetStr(ret).c_str());
  oledClear(); drawHeader("VR ERROR");
  display.setCursor(0, 12); display.print(action);
  display.setCursor(0, 21); display.print(vrRetStr(ret).substring(0, 21));
  display.setCursor(0, 32); display.print("Check wiring:");
  display.setCursor(0, 40); display.print("VR TX>D5  VR RX>D6");
  display.setCursor(0, 47); display.print("VCC+GND, 9600 baud");
  drawFooter("SEL=ok"); display.display(); waitSelect();
}

// ============================================================
// CRC / WIFI STORAGE  — unchanged from v1
// ============================================================
uint32_t crc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320UL : (crc >> 1);
  }
  return ~crc;
}

uint32_t wifiCRC(const WiFiConfig &c) {
  return crc32((const uint8_t *)&c, sizeof(WiFiConfig) - sizeof(uint32_t));
}

bool validWiFiConfig(const WiFiConfig &c) {
  return c.magic == WIFI_MAGIC && c.ssid[0] != '\0' && c.crc == wifiCRC(c);
}

bool readWiFiFile(const char *path, WiFiConfig &c) {
  if (!LittleFS.exists(path)) return false;
  File f = LittleFS.open(path, "r");
  if (!f || f.size() != sizeof(WiFiConfig)) { if (f) f.close(); return false; }
  size_t n = f.read((uint8_t *)&c, sizeof(c)); f.close();
  return n == sizeof(c) && validWiFiConfig(c);
}

bool loadWiFiConfig() {
  WiFiConfig a, b;
  bool va = readWiFiFile(WIFI_A, a), vb = readWiFiFile(WIFI_B, b);
  LOGD("WIFI", "slots: A=%s B=%s", va ? "valid" : "none", vb ? "valid" : "none");
  if (!va && !vb) { wifiConfigValid = false; memset(&wifiConfig, 0, sizeof(wifiConfig)); return false; }
  if (va && (!vb || a.sequence >= b.sequence)) wifiConfig = a; else wifiConfig = b;
  wifiConfigValid = true;
  LOGI("WIFI", "saved: \"%s\" (seq %lu)", wifiConfig.ssid, (unsigned long)wifiConfig.sequence);
  return true;
}

bool saveWiFiConfig(const String &ssid, const String &password) {
  loadWiFiConfig();
  if (wifiConfigValid && ssid == String(wifiConfig.ssid) &&
      password == String(wifiConfig.password)) {
    LOGI("WIFI", "credentials unchanged, no write"); return true;
  }
  WiFiConfig c; memset(&c, 0, sizeof(c));
  c.magic = WIFI_MAGIC;
  c.sequence = wifiConfigValid ? wifiConfig.sequence + 1 : 1;
  ssid.toCharArray(c.ssid, sizeof(c.ssid));
  password.toCharArray(c.password, sizeof(c.password));
  c.crc = wifiCRC(c);
  const char *target = (c.sequence & 1) ? WIFI_A : WIFI_B;
  File f = LittleFS.open(target, "w");
  if (!f) { LOGE("WIFI", "cannot open %s for write", target); return false; }
  size_t written = f.write((const uint8_t *)&c, sizeof(c)); f.flush(); f.close();
  if (written != sizeof(c)) {
    LOGE("WIFI", "short write %u/%u", (unsigned)written, (unsigned)sizeof(c));
    return false;
  }
  WiFiConfig verify;
  if (!readWiFiFile(target, verify)) { LOGE("WIFI", "verify-read failed"); return false; }
  wifiConfig = verify; wifiConfigValid = true;
  LOGI("WIFI", "saved \"%s\" to %s", c.ssid, target);
  return true;
}

void forgetWiFi() {
  LittleFS.remove(WIFI_A); LittleFS.remove(WIFI_B);
  memset(&wifiConfig, 0, sizeof(wifiConfig));
  wifiConfigValid = false;
  WiFi.disconnect(true);
  LOGW("WIFI", "saved WiFi forgotten");
}

// ============================================================
// WIFI CONNECTION  — unchanged from v1
// ============================================================
bool connectSavedWiFi(uint32_t timeoutMs = 15000) {
  if (!wifiConfigValid) return false;
  LOGI("WIFI", "connecting to \"%s\" (timeout %lums)",
       wifiConfig.ssid, (unsigned long)timeoutMs);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiConfig.ssid, wifiConfig.password);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    uint8_t p = (uint8_t)(((millis() - start) * 100UL) / timeoutMs);
    if (p > 100) p = 100;
    drawProgress("WIFI", String("Connecting..."), p, "SEL=skip");
    if (readButton() == SELECT_PRESSED) {
      LOGW("WIFI", "connect skipped by user"); WiFi.disconnect(); return false;
    }
    delay(100); yield();
  }
  bool ok = WiFi.status() == WL_CONNECTED;
  LOGI("WIFI", "result: %s ip=%s",
       wlStrShim(WiFi.status()), WiFi.localIP().toString().c_str());
  return ok;
}

// ============================================================
// v2: RECORD CONFIG STORAGE
//   One binary file per record: /cmd/r00.bin ... /cmd/r79.bin
//   Each file is exactly sizeof(RecordConfig) = 228 bytes.
//   Only the changed record's file is written (minimises flash wear).
// ============================================================
static String cfgPath(uint8_t r) {
  char buf[16]; snprintf(buf, sizeof(buf), "/cmd/r%02u.bin", r); return String(buf);
}

static void initDefaultRecordConfig(uint8_t r, RecordConfig &cfg) {
  memset(&cfg, 0, sizeof(cfg));
  cfg.method   = 0;      // GET
  cfg.enabled  = 0;      // disabled
  cfg.priority = (r < 255) ? r : 255;  // default priority = record index
}

static bool readRecordConfig(uint8_t r, RecordConfig &cfg) {
  String path = cfgPath(r);
  if (!LittleFS.exists(path)) { initDefaultRecordConfig(r, cfg); return false; }
  File f = LittleFS.open(path, "r");
  if (!f || f.size() != sizeof(RecordConfig)) {
    if (f) f.close(); initDefaultRecordConfig(r, cfg); return false;
  }
  size_t n = f.read((uint8_t *)&cfg, sizeof(cfg)); f.close();
  if (n != sizeof(cfg)) { initDefaultRecordConfig(r, cfg); return false; }
  if (cfg.magic != RC_MAGIC) { initDefaultRecordConfig(r, cfg); return false; }
  uint32_t expected = crc32((const uint8_t *)&cfg, sizeof(RecordConfig) - sizeof(uint32_t));
  if (cfg.crc != expected) {
    LOGW("CONFIG", "R%u CRC mismatch (corrupted), using defaults", r);
    initDefaultRecordConfig(r, cfg); return false;
  }
  return true;
}

static bool writeRecordConfig(uint8_t r, RecordConfig &cfg) {
  cfg.magic = RC_MAGIC; cfg._pad = 0;
  // Safety: ensure all string fields are null-terminated within bounds
  cfg.name[RC_NAME_LEN - 1] = '\0';
  cfg.url[RC_URL_LEN   - 1] = '\0';
  cfg.body[RC_BODY_LEN - 1] = '\0';
  cfg.crc = crc32((const uint8_t *)&cfg, sizeof(RecordConfig) - sizeof(uint32_t));

  String path = cfgPath(r);
  File f = LittleFS.open(path, "w");   // LittleFS creates dir implicitly
  if (!f) { LOGE("CONFIG", "cannot open %s for write", path.c_str()); return false; }
  size_t n = f.write((const uint8_t *)&cfg, sizeof(cfg)); f.flush(); f.close();
  if (n != sizeof(cfg)) {
    LOGE("CONFIG", "short write %s: %u/%u", path.c_str(), (unsigned)n, (unsigned)sizeof(cfg));
    return false;
  }
  LOGI("CONFIG", "R%u saved -> %s", r, path.c_str());
  return true;
}

// Rebuild in-RAM cache of configs for all currently active records
static void rebuildActiveCache() {
  activeCfgCount = 0;
  for (uint8_t i = 0; i < activeRecordCount && i < VR_MAX_ACTIVE; i++) {
    uint8_t r = activeRecords[i];
    RecordConfig cfg; readRecordConfig(r, cfg);
    activeCfg[i].record = r;
    strncpy(activeCfg[i].name, cfg.name, RC_NAME_LEN - 1);
    activeCfg[i].name[RC_NAME_LEN - 1] = '\0';
    strncpy(activeCfg[i].url, cfg.url, RC_URL_LEN - 1);
    activeCfg[i].url[RC_URL_LEN - 1] = '\0';
    strncpy(activeCfg[i].body, cfg.body, RC_BODY_LEN - 1);
    activeCfg[i].body[RC_BODY_LEN - 1] = '\0';
    activeCfg[i].method  = cfg.method;
    activeCfg[i].enabled = (cfg.enabled != 0);
    activeCfgCount++;
    LOGI("CACHE", "[%u] R%u \"%s\" %s en=%d",
         i, r, cfg.name, cfg.method ? "POST" : "GET", (int)cfg.enabled);
  }
  LOGI("CACHE", "rebuilt: %u entries", activeCfgCount);
}

// Find cache entry for a physical record number (nullptr if not found)
static const ActiveCfg *findActiveCfg(uint8_t record) {
  for (uint8_t i = 0; i < activeCfgCount; i++)
    if (activeCfg[i].record == record) return &activeCfg[i];
  return nullptr;
}

// ============================================================
// v2: HTTP ACTION EXECUTOR
//   OLED flow: RECOGNIZED Rx -> EXECUTING -> API SUCCESS/FAILED
// ============================================================
static void executeAction(uint8_t record) {
  // Resolve config: try in-RAM cache first, fall back to flash read
  char name[RC_NAME_LEN] = {0};
  char url[RC_URL_LEN]   = {0};
  char body[RC_BODY_LEN] = {0};
  uint8_t method  = 0;
  bool    enabled = false;

  const ActiveCfg *ac = findActiveCfg(record);
  if (ac) {
    strncpy(name, ac->name, RC_NAME_LEN - 1);
    strncpy(url,  ac->url,  RC_URL_LEN  - 1);
    strncpy(body, ac->body, RC_BODY_LEN - 1);
    method  = ac->method;
    enabled = ac->enabled;
  } else {
    RecordConfig cfg;
    readRecordConfig(record, cfg);
    strncpy(name, cfg.name, RC_NAME_LEN - 1);
    strncpy(url,  cfg.url,  RC_URL_LEN  - 1);
    strncpy(body, cfg.body, RC_BODY_LEN - 1);
    method  = cfg.method;
    enabled = (cfg.enabled != 0);
  }

  // ---- OLED: RECOGNIZED ----
  oledClear(); drawHeader("RECOGNIZED");
  display.setTextSize(2);
  char rbuf[8]; snprintf(rbuf, sizeof(rbuf), "R%02u", record);
  display.setCursor(20, 14); display.print(rbuf);
  display.setTextSize(1);
  if (name[0]) { display.setCursor(0, 36); display.print(name); }
  display.display();
  LOGI("RECOG", "[ACTION] RECOGNIZED R%u \"%s\"", record, name);

  if (!url[0] || !enabled) {
    LOGI("ACTION", "R%u: %s – no HTTP dispatch",
         record, !url[0] ? "no URL configured" : "disabled");
    delay(1500); return;
  }

  delay(700);  // Show recognized screen briefly

  // ---- OLED: EXECUTING ----
  oledClear(); drawHeader("EXECUTING");
  display.setCursor(0, 12); display.print(method ? "POST" : "GET");
  char shortUrl[22]; strncpy(shortUrl, url, 21); shortUrl[21] = '\0';
  display.setCursor(0, 22); display.print(shortUrl);
  display.display();
  LOGI("ACTION", "R%u -> %s %s", record, method ? "POST" : "GET", url);

  if (WiFi.status() != WL_CONNECTED) {
    LOGW("ACTION", "WiFi offline – cannot execute");
    oledClear(); drawHeader("API FAILED");
    display.setTextSize(2); display.setCursor(0, 14); display.print("OFFLINE");
    display.setTextSize(1); display.setCursor(0, 36); display.print("WiFi not connected");
    display.display(); delay(1500); return;
  }

  // ---- HTTP Request ----
  WiFiClient wifiClient;
  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT);
  bool beginOk = http.begin(wifiClient, String(url));
  int code = -1;

  if (!beginOk) {
    LOGE("HTTP", "R%u: http.begin() failed (malformed URL?)", record);
    code = -2;
  } else {
    if (method == 1) {
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      code = http.POST(String(body));
    } else {
      code = http.GET();
    }
    http.end();
  }

  LOGI("HTTP", "R%u %s -> HTTP %d", record, method ? "POST" : "GET", code);

  // ---- OLED: API SUCCESS / API FAILED ----
  bool ok = (code >= 200 && code < 300);
  oledClear(); drawHeader(ok ? "API SUCCESS" : "API FAILED");
  display.setTextSize(2);
  display.setCursor(0, 14); display.print(ok ? "OK" : "FAIL");
  display.setTextSize(1);
  char codeBuf[24];
  if      (code == -2) snprintf(codeBuf, sizeof(codeBuf), "Bad URL");
  else if (code == -1) snprintf(codeBuf, sizeof(codeBuf), "Timeout/No reply");
  else                 snprintf(codeBuf, sizeof(codeBuf), "HTTP %d", code);
  display.setCursor(0, 36); display.print(codeBuf);
  display.setCursor(0, 46); display.print(shortUrl);
  display.display();
  LOGI("ACTION", "result: %s (%s)", ok ? "SUCCESS" : "FAILED", codeBuf);
  delay(1500);
}

// ============================================================
// v2: AUTO-LOAD ENABLED RECORDS AT BOOT (priority sorted)
//   Collects all trained+enabled records, sorts by priority
//   (0=highest), loads the first VR_MAX_ACTIVE into the recogniser.
// ============================================================
static void autoLoadEnabled() {
  LOGI("LOAD", "=== autoLoadEnabled ===");
  uint8_t cRec[VR_MAX_RECORD];
  uint8_t cPri[VR_MAX_RECORD];
  uint8_t count = 0;

  for (uint8_t r = 0; r < VR_MAX_RECORD; r++) {
    if (trainedState[r] != 1) continue;       // not trained – skip
    RecordConfig cfg; readRecordConfig(r, cfg);
    if (!cfg.enabled) continue;               // disabled – skip
    cRec[count] = r;
    cPri[count] = cfg.priority;
    count++;
    LOGD("LOAD", "candidate R%u priority=%u", r, cfg.priority);
  }

  // Insertion sort ascending by priority (0 = highest = loaded first)
  for (uint8_t i = 1; i < count; i++) {
    uint8_t kr = cRec[i], kp = cPri[i]; int8_t j = (int8_t)i - 1;
    while (j >= 0 && cPri[j] > kp) {
      cRec[j + 1] = cRec[j]; cPri[j + 1] = cPri[j]; j--;
    }
    cRec[j + 1] = kr; cPri[j + 1] = kp;
  }

  uint8_t toLoad = (count < VR_MAX_ACTIVE) ? count : VR_MAX_ACTIVE;
  LOGI("LOAD", "candidates=%u, loading %u (hw limit %u)", count, toLoad, VR_MAX_ACTIVE);

  if (toLoad == 0) {
    // ---- First-run / no-config fallback ----------------------------------------
    // No records have been enabled via the portal yet, but trained records exist.
    // Load the first VR_MAX_ACTIVE trained records in index order so the bot is
    // immediately usable without needing to open the web portal first.
    // (Once the user saves a record config with enabled=1, priority sorting kicks in.)
    // ----------------------------------------------------------------------------
    int trainedTotal = trainedCountKnown();
    if (trainedTotal == 0) {
      LOGW("LOAD", "no trained records at all – nothing to auto-load");
      return;
    }
    LOGW("LOAD", "no enabled configs yet (fresh device?) – fallback: loading first %d trained records in index order", VR_MAX_ACTIVE);
    count = 0;
    for (uint8_t r = 0; r < VR_MAX_RECORD && count < VR_MAX_ACTIVE; r++) {
      if (trainedState[r] == 1) { cRec[count] = r; cPri[count] = r; count++; }
    }
    toLoad = count;
    LOGI("LOAD", "fallback load list: %u records", toLoad);
  }

  char line[32];
  snprintf(line, sizeof(line), "clear + load %u records", toLoad);
  drawProgress("LOADING", String(line), 10, "please wait");

  int cret = myVR.clear();
  LOGI("LOAD", "clear() -> %d (%s)", cret, vrRetStr(cret).c_str());
  if (cret < 0) { LOGE("LOAD", "clear failed"); showVRError("AutoLoad: clear", cret); return; }

  uint8_t okCount = 0;
  for (uint8_t i = 0; i < toLoad; i++) {
    uint8_t r = cRec[i];
    uint8_t pct = (uint8_t)(20 + (uint16_t)i * 75 / toLoad);
    snprintf(line, sizeof(line), "R%u [P%u]  %u/%u", r, cPri[i], i + 1, toLoad);
    drawProgress("LOADING", String(line), pct, "please wait");
    LOGI("LOAD", "load(R%u) priority=%u (%u/%u)", r, cPri[i], i + 1, toLoad);
    int ret = myVR.load(r);
    LOGI("LOAD", "load(R%u) -> %d (%s)", r, ret, vrRetStr(ret).c_str());
    if (ret >= 0) okCount++;
    yield();
  }

  refreshActiveMap();
  rebuildActiveCache();

  snprintf(line, sizeof(line), "Loaded %u/%u records", okCount, toLoad);
  drawProgress("LOADING", String(line), 100, "done");
  LOGI("LOAD", "auto-load done: %u/%u loaded, active=[%s]",
       okCount, toLoad, listActive().c_str());
  delay(600);
}

// ============================================================
// v2: WEB CONFIG PORTAL  (PROGMEM SPA)
// ============================================================
const char PORTAL_PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>VoiceBot Config</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font:14px/1.5 Arial,sans-serif;background:#0d0d14;color:#cdd6f4;min-height:100vh}
header{background:#1e1e2e;padding:12px 16px;display:flex;align-items:center;justify-content:space-between;box-shadow:0 2px 8px #0008;position:sticky;top:0;z-index:10}
header h1{font-size:18px;color:#89dceb;letter-spacing:.3px}
#hst{font-size:11px;color:#a6adc8;text-align:right;line-height:1.4}
nav{display:flex;background:#181825;border-bottom:1px solid #2a2a3e;position:sticky;top:52px;z-index:9}
nav button{flex:1;padding:10px 4px;background:none;color:#6c7086;border:none;border-bottom:2px solid transparent;cursor:pointer;font-size:13px;transition:.2s}
nav button.on{color:#89dceb;border-bottom-color:#89dceb}
.tab{display:none;padding:12px 14px;max-width:600px;margin:0 auto}
.tab.on{display:block}
.fbar{display:flex;gap:8px;margin-bottom:10px}
.fbar input,.fbar select{flex:1;padding:7px 10px;background:#1e1e2e;border:1px solid #313244;color:#cdd6f4;border-radius:6px;font-size:13px}
.fbar select{flex:0 0 130px}
#recList{display:flex;flex-direction:column;gap:6px}
.card{background:#1e1e2e;border-radius:10px;border-left:3px solid #313244;overflow:hidden}
.card.en{border-left-color:#89dceb}.card.dis{border-left-color:#45475a;opacity:.7}
.ch{display:flex;align-items:center;gap:8px;padding:10px 12px;cursor:pointer;user-select:none}
.rl{font-size:11px;color:#89dceb;font-weight:700;min-width:30px}
.rn{flex:1;font-size:13px;color:#cdd6f4;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;max-width:120px}
.bx{font-size:10px;padding:2px 7px;border-radius:4px;font-weight:600}
.bg{background:#1c3a2a;color:#a6e3a1}.bp{background:#3a2a1c;color:#fab387}
.bo{background:#2a2a3e;color:#6c7086}.bu{background:#3a2000;color:#fe640b}
.pr{font-size:10px;color:#45475a;min-width:24px;text-align:right}
.arr{color:#45475a;font-size:12px}
.bd{display:none;padding:12px;border-top:1px solid #313244;background:#181825}
.bd.on{display:block}
label{font-size:11px;color:#6c7086;display:block;margin:8px 0 3px}
input[type=text],input[type=number],select,textarea{width:100%;padding:7px 10px;background:#11111b;border:1px solid #313244;color:#cdd6f4;border-radius:6px;font-size:13px;outline:none}
input:focus,select:focus,textarea:focus{border-color:#89dceb}
textarea{height:56px;resize:vertical;font-family:monospace}
.r2{display:flex;gap:8px}.r2>*{flex:1}
.ckr{display:flex;align-items:center;gap:6px;margin:10px 0 6px}
.ckr input{width:16px;height:16px;accent-color:#89dceb}
.bts{display:flex;gap:8px;margin-top:10px}
.btn{padding:8px 14px;border:none;border-radius:6px;cursor:pointer;font-size:13px;font-weight:600}
.bs{background:#89dceb;color:#11111b}.bt{background:#1e1e2e;color:#89dceb;border:1px solid #89dceb}
.res{font-size:12px;margin-top:8px;padding:6px 10px;border-radius:6px;display:none}
.ok{background:#1c3a2a;color:#a6e3a1}.er{background:#3a1c1c;color:#f38ba8}
.wc{background:#1e1e2e;border-radius:10px;padding:16px;margin-bottom:12px}
.wc h3{color:#89dceb;margin-bottom:12px;font-size:15px}
.wb{display:flex;gap:8px;margin-top:12px}
#wmsg{margin-top:10px;font-size:13px;color:#a6adc8;line-height:1.4}
.sr{display:flex;justify-content:space-between;padding:7px 0;border-bottom:1px solid #2a2a3e;font-size:13px}
.sv{color:#89dceb;text-align:right;max-width:65%;word-break:break-all}
.em{color:#45475a;text-align:center;padding:24px;font-size:13px}
</style></head><body>
<header>
  <h1>&#127908; VoiceBot</h1>
  <div id="hst">Loading...</div>
</header>
<nav>
  <button class="on" onclick="tab('cmd',this)">Commands</button>
  <button onclick="tab('wifi',this)">WiFi</button>
  <button onclick="tab('stat',this)">Status</button>
</nav>
<div id="cmd" class="tab on">
  <div class="fbar">
    <input id="fs" placeholder="Search name / URL..." oninput="render()">
    <select id="fv" onchange="render()">
      <option value="all">All 80</option>
      <option value="en" selected>Enabled</option>
      <option value="tr">Trained</option>
    </select>
  </div>
  <div id="recList"><div class="em">Loading records...</div></div>
</div>
<div id="wifi" class="tab">
  <div class="wc">
    <h3>WiFi Setup</h3>
    <div id="wst" style="font-size:13px;color:#a6adc8;margin-bottom:10px">Checking...</div>
    <label>Nearby Networks</label>
    <select id="wn" onchange="wp()" style="margin-bottom:6px"><option>Scanning...</option></select>
    <label>SSID</label><input id="ws" placeholder="WiFi SSID">
    <label>Password</label><input id="wp2" type="password" placeholder="Password">
    <div class="wb">
      <button class="btn bs" onclick="wconn()" style="flex:1">Connect &amp; Save</button>
      <button class="btn" onclick="wfgt()" style="flex:1;background:#3a1c1c;color:#f38ba8;border:none">Forget WiFi</button>
    </div>
    <div id="wmsg"></div>
  </div>
</div>
<div id="stat" class="tab">
  <div class="wc">
    <h3>System Status</h3>
    <div id="srows"><div class="em">Loading...</div></div>
    <button class="btn bs" onclick="loadStat()" style="margin-top:12px;width:100%">Refresh</button>
  </div>
</div>
<script>
var cfg=[],tr=new Set(),openIdx=-1;
function tab(id,btn){
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('on'));
  document.querySelectorAll('nav button').forEach(b=>b.classList.remove('on'));
  document.getElementById(id).classList.add('on');btn.classList.add('on');
  if(id==='stat')loadStat();if(id==='wifi')wscan();
}
function esc(s){return String(s||'').replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');}
function badge(c){
  if(!c.enabled)return'<span class="bx bo">OFF</span>';
  return c.method===1?'<span class="bx bp">POST</span>':'<span class="bx bg">GET</span>';
}
function render(){
  var fs=document.getElementById('fs').value.toLowerCase();
  var fv=document.getElementById('fv').value;
  var h='',n=0;
  for(var i=0;i<80;i++){
    var c=cfg[i]||{r:i,name:'',url:'',body:'',method:0,enabled:false,priority:128,trained:false};
    if(fv==='en'&&!c.enabled)continue;
    if(fv==='tr'&&!tr.has(i))continue;
    var srch='r'+i+(c.name||'').toLowerCase()+(c.url||'').toLowerCase();
    if(fs&&!srch.includes(fs))continue;
    var istr=String(i).padStart(2,'0');
    var tb=tr.has(i)?'':'<span class="bx bu">UNTRAINED</span>';
    var isOpen=(openIdx===i);
    h+='<div class="card '+(c.enabled?'en':'dis')+'" id="card'+i+'">';
    h+='<div class="ch" onclick="tog('+i+')">';
    h+='<span class="rl">R'+istr+'</span>';
    h+='<span class="rn">'+(c.name?esc(c.name):'<span style="color:#45475a;font-style:italic">unnamed</span>')+'</span>';
    h+=tb+badge(c)+'<span class="pr">P'+(c.priority||0)+'</span><span class="arr">'+(isOpen?'&#9650;':'&#9660;')+'</span></div>';
    if(isOpen){
      h+='<div class="bd on" id="bd'+i+'">';
      h+='<div class="r2"><div><label>Command Name</label>';
      h+='<input type="text" id="n'+i+'" value="'+esc(c.name)+'" maxlength="23" placeholder="e.g. STOP"></div>';
      h+='<div><label>Priority (0=first)</label>';
      h+='<input type="number" id="p'+i+'" value="'+(c.priority||0)+'" min="0" max="255"></div></div>';
      h+='<label>Method</label><select id="m'+i+'" onchange="mc('+i+')">';
      h+='<option value="0"'+(c.method===0?' selected':'')+'>GET</option>';
      h+='<option value="1"'+(c.method===1?' selected':'')+'>POST</option></select>';
      h+='<label>URL</label><input type="text" id="u'+i+'" value="'+esc(c.url)+'" placeholder="http://192.168.1.x:3000/api/command">';
      h+='<div id="br'+i+'"'+(c.method!==1?' style="display:none"':'')+'>'; 
      h+='<label>POST Body</label><textarea id="b'+i+'">'+esc(c.body)+'</textarea></div>';
      h+='<div class="ckr"><input type="checkbox" id="e'+i+'"'+(c.enabled?' checked':'');
      h+='>Enable this command (auto-loaded by priority)</div>';
      h+='<div class="bts"><button class="btn bs" onclick="save('+i+')" style="flex:1">&#128190; Save</button>';
      h+='<button class="btn bt" onclick="tst('+i+')" style="flex:1">&#9654; Test</button></div>';
      h+='<div id="res'+i+'" class="res"></div></div>';
    }
    h+='</div>';n++;
  }
  if(!n)h='<div class="em">No records match filter</div>';
  document.getElementById('recList').innerHTML=h;
}
function tog(i){
  openIdx=(openIdx===i?-1:i);render();
  if(openIdx===i){var el=document.getElementById('card'+i);setTimeout(()=>el&&el.scrollIntoView({behavior:'smooth',block:'nearest'}),50);}
}
function mc(i){
  var el=document.getElementById('br'+i);
  if(el)el.style.display=document.getElementById('m'+i).value==='1'?'':'none';
}
function gv(id){var e=document.getElementById(id);return e?e.value:'';}
async function save(i){
  var d=new URLSearchParams({r:i,name:gv('n'+i),url:gv('u'+i),body:gv('b'+i),
    method:gv('m'+i),enabled:document.getElementById('e'+i).checked?'1':'0',priority:gv('p'+i)});
  var res=document.getElementById('res'+i);res.style.display='';res.className='res';res.textContent='Saving...';
  try{
    var r=await fetch('/api/record',{method:'POST',body:d});var t=await r.text();
    res.className='res '+(r.ok?'ok':'er');
    res.textContent=r.ok?'&#10003; Saved to flash':'&#10007; '+t;
    if(r.ok){cfg[i]={r:i,name:gv('n'+i),url:gv('u'+i),body:gv('b'+i),
      method:parseInt(gv('m'+i)),enabled:document.getElementById('e'+i).checked,
      priority:parseInt(gv('p'+i)),trained:tr.has(i)};}
  }catch(e){res.className='res er';res.textContent='&#10007; '+e;}
}
async function tst(i){
  var res=document.getElementById('res'+i);res.style.display='';res.className='res';
  res.textContent='Testing... (up to 5s, please wait)';
  try{
    var r=await fetch('/api/test?r='+i,{method:'POST'});var t=await r.text();
    res.className='res '+(r.ok?'ok':'er');res.textContent=(r.ok?'&#10003; ':'&#10007; ')+t;
  }catch(e){res.className='res er';res.textContent='&#10007; '+e;}
}
async function loadAll(){
  try{
    var[rc,rs]=await Promise.all([fetch('/api/config'),fetch('/api/status')]);
    cfg=await rc.json();var st=await rs.json();
    tr=new Set(st.trained||[]);
    document.getElementById('hst').innerHTML=
      (st.wifi?'&#128246; '+esc(st.ssid):'&#128308; Offline')+'<br>VR:'+(st.vr?'&#9989;':'&#10060;')+' Slots:'+st.ac+'/7';
    render();
  }catch(e){document.getElementById('recList').innerHTML='<div class="em">Load failed: '+e+'</div>';}
}
async function loadStat(){
  try{
    var r=await fetch('/api/status');var s=await r.json();
    var rows=[['WiFi',s.wifi?'&#9989; '+esc(s.ssid):'&#10060; Offline'],['IP',s.ip||'-'],
      ['RSSI',s.rssi?s.rssi+' dBm':'-'],['VR Module',s.vr?'&#9989; Online':'&#10060; Offline'],
      ['Active Records',s.ac+'/7: ['+s.al+']'],['Trained',s.tc+' records'],
      ['Config Files',s.cf+' saved'],['Heap',s.heap+' bytes free'],
      ['Uptime',s.up+'s'],['Chip ID','0x'+s.chip]];
    document.getElementById('srows').innerHTML=
      rows.map(([k,v])=>'<div class="sr"><span>'+k+'</span><span class="sv">'+v+'</span></div>').join('');
  }catch(e){document.getElementById('srows').innerHTML='<div class="em">Load failed</div>';}
}
function wscan(){
  fetch('/api/status').then(r=>r.json()).then(s=>{
    document.getElementById('wst').textContent=s.wifi?'Connected: '+s.ssid+' ('+s.ip+')':'Not connected';
  }).catch(()=>{});
  fetch('/scan').then(r=>r.json()).then(a=>{
    var sel=document.getElementById('wn');sel.innerHTML='';
    if(!a.length){sel.innerHTML='<option>No networks found</option>';return;}
    a.forEach(x=>{var o=document.createElement('option');o.value=x.ssid;
      o.textContent=x.ssid+' ('+x.rssi+'dBm)';sel.appendChild(o);});
    document.getElementById('ws').value=a[0].ssid;
  }).catch(()=>{});
}
function wp(){document.getElementById('ws').value=document.getElementById('wn').value;}
async function wconn(){
  var s=document.getElementById('ws').value,p=document.getElementById('wp2').value;
  if(!s){document.getElementById('wmsg').textContent='Enter SSID';return;}
  document.getElementById('wmsg').textContent='Connecting... (up to 15s)';
  try{
    var r=await fetch('/save',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body:'ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(p)});
    document.getElementById('wmsg').innerHTML=await r.text();
  }catch(e){document.getElementById('wmsg').textContent='Link dropped – check OLED for WIFI CONNECTED';}
}
async function wfgt(){
  if(!confirm('Forget saved WiFi credentials?'))return;
  try{await fetch('/api/forget_wifi',{method:'POST'});
    document.getElementById('wmsg').textContent='WiFi cleared. Reconnect via AP mode.';}
  catch(e){document.getElementById('wmsg').textContent='Done (connection dropped as expected)';}
}
loadAll();
</script></body></html>
)HTML";

// ============================================================
// JSON HELPER
// ============================================================
String jsonEscape(const String &s) {
  String out;
  for (uint16_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if      (c == '\\') out += "\\\\";
    else if (c == '"')  out += "\\\"";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else                out += c;
  }
  return out;
}

// ============================================================
// v2: WEB SERVER HANDLERS
// ============================================================

// GET / -> full config portal SPA
void handleRoot() { server.send_P(200, "text/html", PORTAL_PAGE); }

// GET /scan -> WiFi scan JSON  (unchanged from v1)
void handleScan() {
  LOGI("HTTP", "GET /scan");
  int n = WiFi.scanNetworks(false, true);
  String json = "["; bool first = true;
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i); if (!ssid.length()) continue;
    if (!first) json += ","; first = false;
    json += "{\"ssid\":\""; json += jsonEscape(ssid);
    json += "\",\"rssi\":"; json += String(WiFi.RSSI(i)); json += "}";
  }
  json += "]"; WiFi.scanDelete();
  LOGI("HTTP", "scan: %d networks", n);
  server.send(200, "application/json", json);
}

// POST /save -> WiFi credentials save  (unchanged from v1)
void handleSave() {
  String ssid = server.arg("ssid"); String password = server.arg("pass"); ssid.trim();
  LOGI("HTTP", "POST /save for \"%s\"", ssid.c_str());
  if (!ssid.length()) {
    server.send(400, "text/html", "<h3>SSID required</h3><a href='/'>Back</a>"); return;
  }
  WiFi.mode(WIFI_AP_STA); WiFi.begin(ssid.c_str(), password.c_str());
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 12000) { delay(100); yield(); }
  if (WiFi.status() != WL_CONNECTED) {
    LOGW("HTTP", "connect failed: %s", wlStrShim(WiFi.status()));
    WiFi.disconnect();
    server.send(200, "text/html",
                "<h3>Connection failed</h3><p>Check SSID/password.</p><a href='/'>Try again</a>");
    return;
  }
  if (!saveWiFiConfig(ssid, password)) {
    server.send(500, "text/html", "<h3>Connected but flash save failed</h3>"); return;
  }
  server.send(200, "text/html", "<h3>Connected &amp; saved!</h3><p>You can close this page.</p>");
  // Do NOT tear down server inside its own handler; flag for portalLoop()
  portalSaved = true; portalCloseAt = millis() + 1500;
}

// GET /api/config -> JSON array of all 80 record configs (streamed chunk-by-chunk)
void handleApiConfig() {
  LOGI("HTTP", "GET /api/config");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  server.sendContent("[");
  for (uint8_t r = 0; r < VR_MAX_RECORD; r++) {
    RecordConfig cfg; readRecordConfig(r, cfg);
    if (r > 0) server.sendContent(",");
    // Build JSON item as String to avoid fixed-size buffer overflow with long URLs
    String item = "{\"r\":";
    item += (int)r;
    item += ",\"name\":\"";   item += jsonEscape(String(cfg.name)); item += "\"";
    item += ",\"url\":\"";    item += jsonEscape(String(cfg.url));  item += "\"";
    item += ",\"body\":\"";   item += jsonEscape(String(cfg.body)); item += "\"";
    item += ",\"method\":";   item += (int)cfg.method;
    item += ",\"enabled\":";  item += cfg.enabled  ? "true" : "false";
    item += ",\"priority\":"; item += (int)cfg.priority;
    item += ",\"trained\":";  item += (trainedState[r] == 1) ? "true" : "false";
    item += "}";
    server.sendContent(item);
    yield();
  }
  server.sendContent("]");
  LOGI("HTTP", "/api/config sent (80 records)");
}

// POST /api/record -> save one record's config to flash
void handleApiRecord() {
  String rStr = server.arg("r");
  if (!rStr.length()) { server.send(400, "text/plain", "Missing r"); return; }
  int rIdx = rStr.toInt();
  if (rIdx < 0 || rIdx >= VR_MAX_RECORD) {
    server.send(400, "text/plain", "r out of range (0-79)"); return;
  }

  RecordConfig cfg; memset(&cfg, 0, sizeof(cfg));
  server.arg("name").toCharArray(cfg.name, RC_NAME_LEN);
  server.arg("url") .toCharArray(cfg.url,  RC_URL_LEN);
  server.arg("body").toCharArray(cfg.body, RC_BODY_LEN);
  cfg.method   = (server.arg("method")   == "1") ? 1 : 0;
  cfg.enabled  = (server.arg("enabled")  == "1") ? 1 : 0;
  cfg.priority = (uint8_t)constrain(server.arg("priority").toInt(), 0, 255);

  LOGI("CONFIG", "portal save R%d \"%s\" %s %s en=%u pri=%u",
       rIdx, cfg.name, cfg.method ? "POST" : "GET",
       cfg.url, cfg.enabled, cfg.priority);

  if (!writeRecordConfig((uint8_t)rIdx, cfg)) {
    server.send(500, "text/plain", "Flash write failed"); return;
  }
  rebuildActiveCache();   // refresh RAM cache immediately
  server.send(200, "text/plain", "OK");
}

// POST /api/test?r=N -> fire HTTP action for record N, return result to browser
void handleApiTest() {
  String rStr = server.arg("r");
  if (!rStr.length()) { server.send(400, "text/plain", "Missing r"); return; }
  int rIdx = rStr.toInt();
  if (rIdx < 0 || rIdx >= VR_MAX_RECORD) {
    server.send(400, "text/plain", "r out of range"); return;
  }
  LOGI("HTTP", "POST /api/test?r=%d", rIdx);

  RecordConfig cfg;
  if (!readRecordConfig((uint8_t)rIdx, cfg) || !cfg.url[0]) {
    server.send(400, "text/plain", "No URL configured for R" + String(rIdx)); return;
  }
  if (!cfg.enabled) { server.send(200, "text/plain", "Record disabled (not executed)"); return; }
  if (WiFi.status() != WL_CONNECTED) { server.send(503, "text/plain", "WiFi offline"); return; }

  WiFiClient wc; HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT);
  if (!http.begin(wc, String(cfg.url))) {
    server.send(500, "text/plain", "http.begin failed (bad URL?)"); return;
  }
  int code;
  if (cfg.method == 1) {
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    code = http.POST(String(cfg.body));
  } else {
    code = http.GET();
  }
  http.end();
  LOGI("HTTP", "test R%d -> HTTP %d", rIdx, code);

  char result[32]; snprintf(result, sizeof(result), "HTTP %d", code);
  if (code >= 200 && code < 300) server.send(200, "text/plain", String(result));
  else                            server.send(502, "text/plain", String(result));
}

// GET /api/status -> JSON system status
void handleApiStatus() {
  // Trained list
  String trainedList = "["; bool first = true;
  for (int i = 0; i < VR_MAX_RECORD; i++) {
    if (trainedState[i] == 1) {
      if (!first) trainedList += ","; first = false; trainedList += String(i);
    }
  }
  trainedList += "]";

  // Count config files present
  int cfgFiles = 0;
  for (uint8_t r = 0; r < VR_MAX_RECORD; r++) {
    if (LittleFS.exists(cfgPath(r))) cfgFiles++;
    yield();
  }

  String json = "{";
  json += "\"wifi\":";  json += (WiFi.status() == WL_CONNECTED ? "true" : "false");
  json += ",\"ssid\":\"";  json += jsonEscape(WiFi.SSID());    json += "\"";
  json += ",\"ip\":\"";    json += WiFi.localIP().toString();   json += "\"";
  json += ",\"rssi\":";    json += WiFi.RSSI();
  json += ",\"vr\":";      json += vrOnline   ? "true" : "false";
  json += ",\"ac\":";      json += activeRecordCount;
  json += ",\"al\":\"";    json += listActive();                json += "\"";
  json += ",\"tc\":";      json += trainedCountKnown();
  json += ",\"cf\":";      json += cfgFiles;
  json += ",\"heap\":";    json += ESP.getFreeHeap();
  json += ",\"up\":";      json += (millis() / 1000);
  char chip[8]; snprintf(chip, sizeof(chip), "%06X", ESP.getChipId());
  json += ",\"chip\":\"";  json += chip;                        json += "\"";
  json += ",\"trained\":"; json += trainedList;
  json += "}";
  server.send(200, "application/json", json);
}

// POST /api/forget_wifi -> clear saved WiFi credentials
void handleApiForgetWifi() {
  LOGI("HTTP", "POST /api/forget_wifi");
  forgetWiFi();
  server.send(200, "text/plain", "WiFi credentials cleared");
}

// Register all handlers and start server (idempotent – safe to call multiple times)
void startWebServer() {
  if (serverRunning) return;
  if (!handlersSet) {
    server.on("/",                HTTP_GET,  handleRoot);
    server.on("/scan",            HTTP_GET,  handleScan);
    server.on("/save",            HTTP_POST, handleSave);
    server.on("/api/config",      HTTP_GET,  handleApiConfig);
    server.on("/api/record",      HTTP_POST, handleApiRecord);
    server.on("/api/test",        HTTP_POST, handleApiTest);
    server.on("/api/status",      HTTP_GET,  handleApiStatus);
    server.on("/api/forget_wifi", HTTP_POST, handleApiForgetWifi);
    server.onNotFound([]() {
      if (portalRunning) {
        // AP captive portal: redirect everything to config page
        server.sendHeader("Location", "/", true);
        server.send(302, "text/plain", "");
      } else {
        server.send(404, "text/plain", "Not found");
      }
    });
    handlersSet = true;
    LOGI("HTTP", "handlers registered");
  }
  server.begin();
  serverRunning = true;
  LOGI("HTTP", "web server started on port 80");
}

void stopPortal() {
  LOGI("PORTAL", "stopping portal");
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  // Only stop the web server if we are NOT connected as STA.
  // When connected via STA, the server keeps running for the always-on portal.
  if (WiFi.status() != WL_CONNECTED) {
    server.stop(); serverRunning = false;
    LOGI("HTTP", "web server stopped (offline mode)");
  }
  portalRunning = false; portalSaved = false;
}

void portalLoop() {
  if (!portalRunning) return;
  dnsServer.processNextRequest();
  server.handleClient();
  if (portalSaved && millis() > portalCloseAt) stopPortal();
  yield();
}

void startWiFiPortal() {
  portalRunning = true; portalSaved = false;
  WiFi.mode(WIFI_AP_STA);
  String apName = "BOT-SETUP-" + String(ESP.getChipId(), HEX); apName.toUpperCase();
  const char *apPassword = "botsetup123";
  WiFi.softAP(apName.c_str(), apPassword); delay(300);
  IPAddress ip = WiFi.softAPIP();
  dnsServer.start(53, "*", ip);
  startWebServer();  // registers handlers (once) and starts server

  oledClear(); drawHeader("WIFI SETUP");
  display.setCursor(0, 12); display.print("AP: "); display.print(apName.substring(0, 17));
  display.setCursor(0, 21); display.print("PW: "); display.print(apPassword);
  display.setCursor(0, 30); display.print("IP: "); display.print(ip);
  display.setCursor(0, 41); display.print("Join AP, open browser");
  drawFooter("SEL=skip (offline)"); display.display();
  LOGI("PORTAL", "AP=%s PW=%s http://%s", apName.c_str(), apPassword, ip.toString().c_str());
}

// Blocking WiFi setup portal; SELECT skips to offline mode
void runPortal() {
  startWiFiPortal();
  while (portalRunning) {
    portalLoop();
    if (readButton() == SELECT_PRESSED) {
      LOGW("PORTAL", "skipped by user -> continuing offline");
      stopPortal(); break;
    }
    pollSerial(); delay(2);
  }
}

// ============================================================
// MENU  — unchanged from v1
// ============================================================
int menu(const char *title, const char *items[], uint8_t count) {
  uint8_t selected = 0, top = 0; bool redraw = true;
  while (true) {
    if (redraw) {
      if (selected < top) top = selected;
      if (selected >= top + 5) top = selected - 4;
      oledClear(); char pos[10]; snprintf(pos, sizeof(pos), "%u/%u", selected + 1, count);
      drawHeader(title, pos);
      for (uint8_t row = 0; row < 5; row++) {
        uint8_t index = top + row; if (index >= count) break;
        uint8_t y = 12 + row * 8;
        if (index == selected) {
          display.fillRect(0, y - 1, 128, 9, SSD1306_WHITE);
          display.setTextColor(SSD1306_BLACK);
        } else { display.setTextColor(SSD1306_WHITE); }
        display.setCursor(2, y);
        display.print(index == selected ? ">" : " ");
        display.print(items[index]);
      }
      display.setTextColor(SSD1306_WHITE);
      drawFooter("UP/DN move  SEL ok");
      display.display(); redraw = false;
    }
    Button b = readButton();
    if (b == UP_PRESSED)    { selected = (selected == 0) ? count - 1 : selected - 1; redraw = true; }
    if (b == DOWN_PRESSED)  { selected = (selected + 1 >= count) ? 0 : selected + 1; redraw = true; }
    if (b == SELECT_PRESSED) { LOGI("MENU", "%s: [%u] %s", title, selected, items[selected]); return selected; }
    pollBackground(); delay(5);
  }
}

// ============================================================
// RECORD PICKER  — unchanged from v1
// ============================================================
static int lastPicked = 0;

static int pickRecord(const char *title, const char *goLabel) {
  int v = lastPicked; bool redraw = true;
  bool dirty = (v < VR_MAX_RECORD && trainedState[v] < 0), checkFailed = false;
  uint32_t changedAt = millis();
  refreshActiveMap();
  while (true) {
    if (redraw) {
      oledClear(); drawHeader(title);
      display.setTextSize(3);
      if (v >= VR_MAX_RECORD) { display.setCursor(28, 16); display.print("BACK"); }
      else { char buf[6]; snprintf(buf, sizeof(buf), "R%02d", v); display.setCursor(37, 16); display.print(buf); }
      display.setTextSize(1);
      if (v < VR_MAX_RECORD) {
        const char *state = "checking...";
        if      (checkFailed)       state = "VR NO REPLY";
        else if (trainedState[v] == 1) state = "TRAINED";
        else if (trainedState[v] == 0) state = "EMPTY";
        display.setCursor(0, 44); display.print(state);
        bool loaded = false;
        for (uint8_t i = 0; i < activeRecordCount; i++) if (activeRecords[i] == v) loaded = true;
        if (loaded) { display.setCursor(84, 44); display.print("[LOADED]"); }
      }
      char foot[24]; snprintf(foot, sizeof(foot), "UP/DN pick  SEL=%s",
                              v >= VR_MAX_RECORD ? "back" : goLabel);
      drawFooter(foot); display.display(); redraw = false;
    }
    Button b = readButton(); bool moved = false;
    if (b == UP_PRESSED)    { v = (v == 0) ? VR_MAX_RECORD : v - 1; moved = true; }
    if (b == DOWN_PRESSED)  { v = (v >= VR_MAX_RECORD) ? 0 : v + 1; moved = true; }
    if (b == SELECT_PRESSED) {
      if (v >= VR_MAX_RECORD) { LOGI("PICK", "BACK"); return -1; }
      lastPicked = v; LOGI("PICK", "picked R%d", v); return v;
    }
    if (moved) { changedAt = millis(); checkFailed = false; dirty = (v < VR_MAX_RECORD && trainedState[v] < 0); redraw = true; }
    if (dirty && v < VR_MAX_RECORD && (millis() - changedAt) > 250) {
      uint8_t st = 0xFF; int ret = myVR.checkRecord((uint8_t)v, &st);
      if      (ret < 0 && vrIsNotTrained(ret)) trainedState[v] = 0;
      else if (ret < 0)                         checkFailed = true;
      else                                      trainedState[v] = (st == 0x01) ? 1 : 0;
      dirty = false; redraw = true;
    }
    pollBackground(); delay(5);
  }
}

// ============================================================
// MODULE TEST  — unchanged from v1
// ============================================================
static void moduleTest() {
  bool first = true;
  while (true) {
    Button b = first ? UP_PRESSED : readButton(); first = false;
    if (b == SELECT_PRESSED) return;
    if (b == UP_PRESSED) {
      LOGI("TEST", "=== VR module link test ===");
      oledClear(); drawHeader("MODULE TEST");
      display.setCursor(0, 20); display.print("Testing..."); display.display();

      uint32_t t1 = millis(); bool t1ok = refreshActiveMap(); uint32_t d1 = millis() - t1;
      uint8_t st = 0xFF;
      uint32_t t2 = millis(); int r2 = myVR.checkRecord(0, &st); uint32_t d2 = millis() - t2;
      if (r2 < 0 && vrIsNotTrained(r2)) { r2 = 0; st = 0x00; }
      LOGI("TEST", "T1 recognizer=%s (%lums)  T2 checkR(0) ret=%d st=0x%02X (%lums)",
           t1ok ? "PASS" : "FAIL", (unsigned long)d1, r2, st, (unsigned long)d2);

      oledClear(); drawHeader("MODULE TEST", (t1ok && r2 >= 0) ? "PASS" : "FAIL");
      char line[24];
      snprintf(line, sizeof(line), "T1 map   : %s %lums", t1ok ? "OK" : "FAIL", (unsigned long)d1);
      display.setCursor(0, 12); display.print(line);
      if (r2 >= 0) snprintf(line, sizeof(line), "T2 rec R0: OK s=%02X", st);
      else         snprintf(line, sizeof(line), "T2 rec R0: FAIL");
      display.setCursor(0, 20); display.print(line);
      if (t1ok) {
        snprintf(line, sizeof(line), "Loaded %u/7", activeRecordCount);
        display.setCursor(0, 30); display.print(line);
        display.setCursor(0, 38); display.print(listActive().substring(0, 21));
        display.setCursor(0, 46); display.print("VR MODULE ONLINE");
      } else {
        display.setCursor(0, 30); display.print("VR TX>D5 (divider)");
        display.setCursor(0, 38); display.print("VR RX>D6  VCC+GND");
        display.setCursor(0, 46); display.print("NOT RESPONDING");
      }
      drawFooter("UP=retry  SEL=back"); display.display();
    }
    pollBackground(); delay(5);
  }
}

// ============================================================
// VOICE STATUS  — unchanged from v1
// ============================================================
static void voiceStatus() {
  bool rescan = true;
  while (true) {
    if (rescan) {
      rescan = false; vrBusy = true;
      int count = scanTrained(); bool mapOk = (count >= 0) ? refreshActiveMap() : false;
      vrBusy = false;
      if (count == -1) { showVRError("Status scan", VR::TIMEOUT); return; }
      if (count == -2) return;
      oledClear(); drawHeader("VOICE STATUS", mapOk ? "VR:OK" : "VR:??");
      char line[24]; snprintf(line, sizeof(line), "Trained %d/%d", count, VR_MAX_RECORD);
      display.setCursor(0, 12); display.print(line);
      drawWrapped(0, 20, fmtTrainedRanges(), 21, 2);
      if (mapOk) {
        snprintf(line, sizeof(line), "Loaded %u/7", activeRecordCount);
        display.setCursor(0, 38); display.print(line);
        drawWrapped(0, 46, listActive(), 21, 1);
      } else { display.setCursor(0, 38); display.print("Loaded: read failed"); }
      drawFooter("UP=rescan  SEL=back"); display.display();
    }
    Button b = readButton();
    if (b == SELECT_PRESSED) return;
    if (b == UP_PRESSED)     rescan = true;
    pollBackground(); delay(5);
  }
}

// ============================================================
// TRAIN RECORD  — unchanged from v1
// ============================================================
static void trainRecord() {
  int r = pickRecord("TRAIN RECORD", "train"); if (r < 0) return;
  LOGI("TRAIN", "=========== TRAIN R%d ===========", r);
  if (!refreshActiveMap()) { showVRError("Train pre-check", VR::TIMEOUT); return; }

  for (int s = 3; s >= 1; s--) {
    oledClear(); char t[24]; snprintf(t, sizeof(t), "TRAIN R%02d", r); drawHeader(t);
    display.setCursor(0, 12); display.print("Say the word clearly,");
    display.setCursor(0, 20); display.print("pause ~1-2 sec, then");
    display.setCursor(0, 28); display.print("say it AGAIN.");
    display.setCursor(0, 38); display.print("Mic 10-20cm away");
    char foot[24]; snprintf(foot, sizeof(foot), "Starting in %d..  SEL=x", s);
    drawFooter(foot); display.display();
    uint32_t t0 = millis();
    while (millis() - t0 < 1000) {
      if (readButton() == SELECT_PRESSED) {
        LOGW("TRAIN", "cancelled"); showMessage("TRAIN CANCELLED"); return;
      }
      pollBackground(); delay(5);
    }
  }

  oledClear(); char t2[24]; snprintf(t2, sizeof(t2), "TRAINING R%02d", r);
  drawHeader(t2, "LIVE");
  display.setTextSize(2); display.setCursor(6, 16); display.print("SPEAK NOW"); display.setTextSize(1);
  display.setCursor(0, 38); display.print("Say it, wait, say it");
  display.setCursor(0, 46); display.print("again. Please wait..");
  drawFooter("module is listening"); display.display();

  vrBusy = true;
  uint8_t result[64]; memset(result, 0, sizeof(result));
  LOGI("TRAIN", "calling myVR.train(%d, result) - BLOCKS until module finishes", r);
  uint32_t t0 = millis();
  int ret = myVR.train((uint8_t)r, result);
  uint32_t dt = millis() - t0;
  LOGI("TRAIN", "train() returned %d (%s) after %lums", ret, vrRetStr(ret).c_str(), (unsigned long)dt);
  logHex("TRAIN", "result buffer", result, 32);

  uint8_t st = 0xFF; int cr = myVR.checkRecord((uint8_t)r, &st); vrBusy = false;
  if (cr < 0 && vrIsNotTrained(cr)) { st = 0x00; cr = 0; }
  bool verified = (cr >= 0 && st == 0x01);
  LOGI("TRAIN", "verify: checkRecord ret=%d status=0x%02X -> %s",
       cr, st, verified ? "TRAINED (verified)" : "NOT trained");
  trainedState[r] = (cr >= 0) ? (st == 0x01 ? 1 : 0) : -1;

  oledClear(); char h[24]; snprintf(h, sizeof(h), "TRAIN R%02d", r);
  if (ret >= 0 && verified) {
    drawHeader(h, "OK");
    display.setTextSize(2); display.setCursor(0, 14); display.print("SUCCESS"); display.setTextSize(1);
    display.setCursor(0, 34); display.print("Saved & verified in");
    display.setCursor(0, 42); display.print("module. Next: LOAD.");
  } else if (ret >= 0) {
    drawHeader(h, "?");
    display.setCursor(0, 12); display.print("Module said OK but");
    display.setCursor(0, 20); display.print("record reads EMPTY");
    display.setCursor(0, 30); display.print("(status 0x"); display.print(st, HEX); display.print(")");
    display.setCursor(0, 42); display.print("Try training again");
  } else {
    drawHeader(h, "FAIL");
    display.setTextSize(2); display.setCursor(0, 14); display.print("FAILED"); display.setTextSize(1);
    display.setCursor(0, 34); display.print(vrRetStr(ret).substring(0, 21));
    display.setCursor(0, 42); display.print(ret == VR::TIMEOUT ? "No voice heard/louder" : "See serial log");
  }
  drawFooter("SEL=ok"); display.display(); waitSelect();
}

// ============================================================
// LOAD / CLEAR  — same logic as v1, +rebuildActiveCache() calls
// ============================================================
static bool loadAll() {
  LOGI("LOAD", "=========== LOAD ALL (first %d trained) ===========", VR_MAX_ACTIVE);
  vrBusy = true;
  int total = scanTrained();
  if (total == -2) { vrBusy = false; return false; }
  if (total == -1) { vrBusy = false; showVRError("Load: scan", VR::TIMEOUT); return false; }
  if (total == 0)  { vrBusy = false; showMessage("NO TRAINED RECORDS", "Use Train Record first", 1800); return false; }

  uint8_t want[VR_MAX_ACTIVE]; uint8_t wantN = 0;
  for (uint8_t r = 0; r < VR_MAX_RECORD && wantN < VR_MAX_ACTIVE; r++)
    if (trainedState[r] == 1) want[wantN++] = r;
  logHex("LOAD", "records to load", want, wantN);

  drawProgress("LOADING", String("clear + load ") + String(wantN), 30, "please wait");
  int cret = myVR.clear();
  LOGI("LOAD", "clear() -> %d (%s)", cret, vrRetStr(cret).c_str());
  if (cret < 0) { vrBusy = false; showVRError("Clear before load", cret); return false; }

  uint8_t okCount = 0;
  for (uint8_t i = 0; i < wantN; i++) {
    drawProgress("LOADING", String("Loading R") + String((int)want[i]),
                 40 + i * 55 / wantN, "please wait");
    uint32_t t0 = millis(); int ret = myVR.load(want[i]);
    LOGI("LOAD", "load(R%u) -> %d (%s) %lums",
         want[i], ret, vrRetStr(ret).c_str(), (unsigned long)(millis() - t0));
    if (ret >= 0) okCount++;
  }

  bool mapOk = refreshActiveMap(); vrBusy = false;
  uint8_t matched = 0;
  if (mapOk) for (uint8_t i = 0; i < wantN; i++)
    for (uint8_t j = 0; j < activeRecordCount; j++)
      if (activeRecords[j] == want[i]) matched++;
  bool good = mapOk && matched == wantN;
  LOGI("LOAD", "requested=%u callsOK=%u verified=%u => %s", wantN, okCount, matched, good ? "SUCCESS" : "MISMATCH");

  rebuildActiveCache();  // v2: refresh in-RAM config cache

  oledClear(); drawHeader("LOAD ALL", good ? "OK" : "CHECK");
  char line[24];
  snprintf(line, sizeof(line), "Trained : %d", total); display.setCursor(0, 12); display.print(line);
  snprintf(line, sizeof(line), "Loaded  : %u/%u", mapOk ? activeRecordCount : 0, wantN);
  display.setCursor(0, 20); display.print(line);
  display.setCursor(0, 30); display.print("In module:");
  display.setCursor(0, 38); display.print(mapOk ? listActive().substring(0, 21) : String("read failed"));
  display.setCursor(0, 46); display.print(good ? "Verified. Go RECOGNIZE" : "Mismatch - see serial");
  drawFooter("SEL=ok"); display.display(); waitSelect(); return good;
}

static void loadOne() {
  int r = pickRecord("LOAD ONE", "load"); if (r < 0) return;
  LOGI("LOAD", "=========== LOAD ONE R%d ===========", r);
  vrBusy = true;
  uint8_t st = 0xFF; int cr = myVR.checkRecord((uint8_t)r, &st);
  LOGI("LOAD", "checkRecord(R%d) ret=%d status=0x%02X", r, cr, st);
  if (cr < 0 && vrIsNotTrained(cr)) { st = 0x00; cr = 0; }
  if (cr < 0) { vrBusy = false; showVRError("Check record", cr); return; }
  if (st != 0x01) {
    vrBusy = false; trainedState[r] = 0;
    showMessage("NOT TRAINED", String("R") + String(r) + " is empty", 1600); return;
  }
  trainedState[r] = 1;
  if (!refreshActiveMap()) { vrBusy = false; showVRError("Read recognizer", VR::TIMEOUT); return; }
  for (uint8_t i = 0; i < activeRecordCount; i++)
    if (activeRecords[i] == r) {
      vrBusy = false; showMessage("ALREADY LOADED", String("R") + String(r), 1400); return;
    }
  if (activeRecordCount >= VR_MAX_ACTIVE) {
    vrBusy = false; showMessage("RECOGNIZER FULL (7)", "Clear Loaded first", 1800); return;
  }
  int ret = myVR.load((uint8_t)r);
  LOGI("LOAD", "load(R%d) -> %d (%s)", r, ret, vrRetStr(ret).c_str());
  bool mapOk = refreshActiveMap(); vrBusy = false;
  bool inModule = false;
  if (mapOk) for (uint8_t i = 0; i < activeRecordCount; i++) if (activeRecords[i] == r) inModule = true;
  LOGI("LOAD", "verify: R%d in module = %s", r, inModule ? "YES" : "NO");

  rebuildActiveCache();  // v2: refresh in-RAM config cache

  if (inModule) showMessage("LOADED OK", String("R") + String(r) + "  active: " + listActive(), 1800);
  else if (ret < 0) showMessage("LOAD FAILED", vrRetStr(ret), 1800);
  else showMessage("LOAD UNVERIFIED", "Module didn't list it", 1800);
}

static void clearLoaded() {
  LOGI("CLEAR", "clearing recognizer (trained data NOT erased)");
  vrBusy = true; int ret = myVR.clear();
  LOGI("CLEAR", "clear() -> %d (%s)", ret, vrRetStr(ret).c_str());
  bool mapOk = refreshActiveMap(); vrBusy = false;

  rebuildActiveCache();  // v2: cache now empty

  if (ret >= 0 && mapOk && activeRecordCount == 0)
    showMessage("LOADED LIST CLEARED", "Trained words kept", 1500);
  else showMessage("CLEAR FAILED", vrRetStr(ret), 1600);
}

// ============================================================
// RECOGNITION PACKET PARSER  — UNCHANGED from v1 (proven protocol)
//   [0]=HEAD [1]=len [2]=CMD_VR [3]=status [4]=group
//   [5]=PHYSICAL RECORD (0xFF=no match) [6]=recognizer index
//   [7]=sigLen [8..]=signature [n-1]=END
// ============================================================
static int parseRecognition(const uint8_t *p, int n, RecogResult &r) {
  memset(&r, 0, sizeof(r)); r.len = n;
  if (n < 9)                    { LOGW("PARSE", "too short: %d bytes (<9)", n); return PARSE_INVALID; }
  if (p[0] != FRAME_HEAD)       { LOGW("PARSE", "bad head 0x%02X", p[0]); return PARSE_INVALID; }
  if (p[1] != (uint8_t)(n - 2)) { LOGW("PARSE", "length field %u != n-2 (%d)", p[1], n - 2); return PARSE_INVALID; }
  if (p[2] != FRAME_CMD_VR)     { LOGW("PARSE", "cmd 0x%02X is not FRAME_CMD_VR", p[2]); return PARSE_INVALID; }
  if (p[n - 1] != FRAME_END)    { LOGW("PARSE", "bad end byte 0x%02X", p[n - 1]); return PARSE_INVALID; }
  r.cmd = p[2]; r.sub = p[3]; r.group = p[4]; r.physical = p[5]; r.index = p[6]; r.sigLen = p[7];
  int avail = n - 9;
  if (r.sigLen > avail) { LOGW("PARSE", "sigLen %u > payload (%d)", r.sigLen, avail); return PARSE_INVALID; }
  for (uint8_t i = 0; i < r.sigLen && i < sizeof(r.sig); i++) r.sig[i] = p[8 + i];
  if (r.physical == 0xFF) return PARSE_NOMATCH;
  if (r.physical >= VR_MAX_RECORD) return PARSE_BADINDEX;
  bool known = false;
  for (uint8_t i = 0; i < activeRecordCount; i++) if (activeRecords[i] == r.physical) known = true;
  if (!known) LOGW("PARSE", "R%u not in last-known loaded list [%s] (list may be stale)",
                   r.physical, listActive().c_str());
  return PARSE_MATCH;
}

// ============================================================
// DRAW RECOGNIZE  — unchanged from v1 (used by recognizeMode)
// ============================================================
static void drawRecognize(const RecUI &u) {
  oledClear(); char buf[24];
  if (u.state == UI_MATCH) {
    display.fillRect(0, 0, 128, 54, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(6, 3); display.print("** VOICE MATCHED **");
    display.setTextSize(3);
    snprintf(buf, sizeof(buf), "R%02u", u.record);
    display.setCursor(37, 16); display.print(buf);
    display.setTextSize(1);
    snprintf(buf, sizeof(buf), "recognizer slot %u", u.index);
    display.setCursor(6, 44); display.print(buf);
    display.setTextColor(SSD1306_WHITE);
  } else if (u.state == UI_NOMATCH) {
    drawHeader("RECOGNIZE", "heard");
    display.setTextSize(2); display.setCursor(4, 18); display.print("NO MATCH"); display.setTextSize(1);
    display.setCursor(0, 40); display.print("Sound heard, but not");
    display.setCursor(0, 47); display.print("a loaded word");
  } else if (u.state == UI_BAD) {
    drawHeader("RECOGNIZE", "error");
    display.setTextSize(2); display.setCursor(4, 18); display.print("BAD PACKET"); display.setTextSize(1);
    display.setCursor(0, 42); display.print("See serial log");
  } else if (u.state == UI_BADIDX) {
    drawHeader("RECOGNIZE", "error");
    display.setTextSize(2); display.setCursor(4, 18); display.print("MAP ERROR"); display.setTextSize(1);
    snprintf(buf, sizeof(buf), "record no. %u invalid", u.record);
    display.setCursor(0, 42); display.print(buf);
  } else {
    static const char spin[] = "|/-\\";
    char rs[3] = { spin[(millis() / 150) % 4], 0, 0 };
    drawHeader("RECOGNIZE", rs);
    display.setCursor(0, 11); display.print("Slots:"); display.print(listActive().substring(0, 15));
    display.setTextSize(2); display.setCursor(10, 21); display.print("LISTENING"); display.setTextSize(1);
    int p = (millis() / 10) % 208; if (p > 104) p = 208 - p;
    display.drawRect(0, 39, 128, 5, SSD1306_WHITE);
    display.fillRect(p + 1, 40, 22, 3, SSD1306_WHITE);
    display.setCursor(0, 46);
    if (u.lastRecord >= 0) {
      snprintf(buf, sizeof(buf), "Last:R%02d %lus ago", u.lastRecord,
               (unsigned long)((millis() - u.lastEventMs) / 1000));
      display.print(buf);
    } else { display.print("Last: none yet"); }
  }
  display.drawFastHLine(0, 54, 128, SSD1306_WHITE);
  display.setTextColor(SSD1306_WHITE); display.setCursor(0, 56);
  snprintf(buf, sizeof(buf), "H:%lu N:%lu B:%lu SEL=exit",
           (unsigned long)u.hits, (unsigned long)u.nomatch, (unsigned long)u.bad);
  display.print(buf); display.display();
}

// ============================================================
// recognizeMode  — unchanged from v1 (kept for Voice menu access)
// ============================================================
static void recognizeMode() {
  LOGI("RECOG", "=========== RECOGNIZE MODE (manual) ===========");
  if (!refreshActiveMap()) { showVRError("Read recognizer", VR::TIMEOUT); return; }
  if (activeRecordCount == 0) {
    LOGW("RECOG", "nothing loaded");
    oledClear(); drawHeader("RECOGNIZE", "empty");
    display.setCursor(0, 14); display.print("NOTHING LOADED");
    display.setCursor(0, 28); display.print("SEL  = load trained");
    display.setCursor(0, 36); display.print("DOWN = cancel");
    display.display();
    while (true) {
      Button b = readButton();
      if (b == DOWN_PRESSED || b == UP_PRESSED) return;
      if (b == SELECT_PRESSED) { if (!loadAll()) return; break; }
      pollBackground(); delay(5);
    }
    if (!refreshActiveMap() || activeRecordCount == 0) return;
  }
  LOGI("RECOG", "listening. active=[%s]", listActive().c_str());
  for (uint8_t i = 0; i < activeRecordCount; i++)
    LOGI("RECOG", "  slot %u = record %u", i, activeRecords[i]);

  vrBusy = true; myVR.flushInput();
  RecUI ui; memset(&ui, 0, sizeof(ui)); ui.lastRecord = -1; ui.state = UI_LISTEN;
  uint32_t polls = 0, errors = 0, events = 0, stateUntil = 0, lastDraw = 0, lastBeat = millis();
  drawRecognize(ui);

  while (true) {
    Button b = readButton();
    if (b == SELECT_PRESSED) break;
    if (b == UP_PRESSED) { ui.hits = ui.nomatch = ui.bad = ui.timeouts = 0; polls = errors = events = 0; LOGI("RECOG", "counters reset"); }
    pollSerial();
    int n = myVR.receive_pkt(vrPacket, 60); polls++;
    if (n > 0) {
      events++;
      LOGI("RECOG", "----- EVENT #%lu (%d bytes) -----", (unsigned long)events, n);
      logHex("RECOG", "raw packet", vrPacket, n);
      RecogResult r; int pr = parseRecognition(vrPacket, n, r);
      LOGD("RECOG", "cmd=0x%02X status=0x%02X grp=0x%02X phys=%u idx=%u sigLen=%u",
           r.cmd, r.sub, r.group, r.physical, r.index, r.sigLen);
      if (r.sigLen) logHex("RECOG", "signature", r.sig, r.sigLen > 16 ? 16 : r.sigLen);
      if (pr == PARSE_MATCH) {
        ui.state = UI_MATCH; ui.record = r.physical; ui.index = r.index;
        ui.hits++; ui.lastRecord = r.physical; ui.lastEventMs = millis();
        stateUntil = millis() + 1600;
        LOGI("RECOG", ">>> MATCH: R%u (slot %u) hits=%lu", r.physical, r.index, (unsigned long)ui.hits);
      } else if (pr == PARSE_NOMATCH) {
        ui.state = UI_NOMATCH; ui.nomatch++; stateUntil = millis() + 1200;
        LOGI("RECOG", ">>> NO MATCH (0xFF)");
      } else if (pr == PARSE_BADINDEX) {
        ui.state = UI_BADIDX; ui.record = r.physical; ui.bad++; stateUntil = millis() + 1600;
        LOGE("RECOG", ">>> INVALID R%u", r.physical);
      } else {
        ui.state = UI_BAD; ui.bad++; stateUntil = millis() + 1200;
        LOGE("RECOG", ">>> INVALID PACKET");
      }
      drawRecognize(ui); lastDraw = millis();
    } else if (n == VR::TIMEOUT) { ui.timeouts++; }
    else {
      errors++;
      if (errors <= 5 || (errors % 50) == 0)
        LOGW("RECOG", "receive_pkt error %d (%s) [total %lu]", n, vrRetStr(n).c_str(), (unsigned long)errors);
    }
    if (ui.state != UI_LISTEN && millis() > stateUntil) { ui.state = UI_LISTEN; lastDraw = 0; }
    if (millis() - lastDraw >= 70) { drawRecognize(ui); lastDraw = millis(); }
    if (millis() - lastBeat >= 5000) {
      lastBeat = millis();
      LOGI("RECOG", "heartbeat: polls=%lu idle=%lu hits=%lu nomatch=%lu bad=%lu rxErr=%lu heap=%u",
           (unsigned long)polls, (unsigned long)ui.timeouts, (unsigned long)ui.hits,
           (unsigned long)ui.nomatch, (unsigned long)ui.bad, (unsigned long)errors, ESP.getFreeHeap());
    }
    yield();
  }
  vrBusy = false;
  LOGI("RECOG", "exit. hits=%lu nomatch=%lu bad=%lu",
       (unsigned long)ui.hits, (unsigned long)ui.nomatch, (unsigned long)ui.bad);
}

// ============================================================
// VOICE MENU  — unchanged from v1
// ============================================================
static void voiceMenu() {
  const char *items[] = {
    "Module Test", "Status (scan 80)", "Train Record",
    "Load All Trained", "Load One Record", "Clear Loaded",
    "RECOGNIZE (live)", "Back"
  };
  while (true) {
    int s = menu("VOICE", items, 8);
    switch (s) {
      case 0: moduleTest();    break;
      case 1: voiceStatus();   break;
      case 2: trainRecord();   break;
      case 3: loadAll();       break;
      case 4: loadOne();       break;
      case 5: clearLoaded();   break;
      case 6: recognizeMode(); break;
      case 7: return;
    }
  }
}

// ============================================================
// SETTINGS SCREENS  — unchanged from v1, wifiInfo shows portal URL
// ============================================================
void wifiSetup() {
  runPortal();
  if (WiFi.status() == WL_CONNECTED) {
    startWebServer();  // ensure server is running after portal
    showMessage("WIFI CONNECTED", WiFi.localIP().toString(), 1500);
  }
}

void wifiInfo() {
  oledClear(); drawHeader("WIFI INFO");
  if (WiFi.status() == WL_CONNECTED) {
    display.setCursor(0, 12); display.print("SSID: "); display.print(WiFi.SSID());
    display.setCursor(0, 22); display.print("IP:   "); display.print(WiFi.localIP());
    display.setCursor(0, 32); display.print("RSSI: "); display.print(WiFi.RSSI()); display.print(" dBm");
    display.setCursor(0, 42); display.print("http://"); display.print(WiFi.localIP());
  } else {
    display.setCursor(0, 20); display.print("NOT CONNECTED");
    display.setCursor(0, 32); display.print(wlStrShim(WiFi.status()));
  }
  drawFooter("SEL=back"); display.display(); waitSelect();
}

void batteryInfo() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 8; i++) { sum += analogRead(BATTERY_PIN); delay(2); }
  float raw = sum / 8.0f;
  float volts = (raw / 1023.0f) * ADC_FULLSCALE_V * BATT_DIVIDER;
  int pct = (int)((volts - 3.3f) / (4.2f - 3.3f) * 100.0f);
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
  LOGI("BATT", "adc=%.1f volts=%.2f est=%d%%", raw, volts, pct);
  oledClear(); drawHeader("BATTERY");
  display.setTextSize(2); display.setCursor(0, 16); display.print(volts, 2); display.print("V");
  display.setTextSize(1);
  display.setCursor(0, 38); display.print("Li-ion est: "); display.print(pct); display.print("%");
  display.setCursor(0, 46); display.print("ADC raw: "); display.print((int)raw);
  drawFooter("SEL=back"); display.display(); waitSelect();
}

void forgetWiFiAndSetup() { forgetWiFi(); showMessage("WIFI CLEARED", "Opening setup...", 900); wifiSetup(); }

void settingsMenu() {
  const char *items[] = { "WiFi Setup", "Forget WiFi", "WiFi Info", "Battery", "Back" };
  while (true) {
    int s = menu("SETTINGS", items, 5);
    switch (s) {
      case 0: wifiSetup(); break; case 1: forgetWiFiAndSetup(); break;
      case 2: wifiInfo();  break; case 3: batteryInfo();        break; case 4: return;
    }
  }
}

void aboutMenu() {
  oledClear(); drawHeader("ABOUT");
  display.setCursor(0, 12); display.print("ESP8266 Bot + VR3  v2");
  display.setCursor(0, 20); display.print("Chip: "); display.print(ESP.getChipId(), HEX);
  display.setCursor(0, 28); display.print("Heap: "); display.print(ESP.getFreeHeap());
  display.setCursor(0, 36); display.print("Up:   "); display.print(millis() / 1000); display.print("s");
  display.setCursor(0, 44); display.print("VR:   "); display.print(vrOnline ? "online" : "unknown");
  drawFooter("SEL=back"); display.display(); printState(); waitSelect();
}

void mainMenu() {
  const char *items[] = { "Voice Recognition", "Settings", "WiFi Info", "About" };
  while (true) {
    int s = menu("MAIN MENU", items, 4);
    switch (s) {
      case 0: voiceMenu();    break;
      case 1: settingsMenu(); break;
      case 2: wifiInfo();     break;
      case 3: aboutMenu();    break;
    }
  }
}

// ============================================================
// v2: LISTEN LOOP  —  the primary automation loop
//
//   On boot, this loop runs immediately after autoLoadEnabled().
//   Flow:
//     receive_pkt -> parseRecognition (UNCHANGED protocol) ->
//     PARSE_MATCH  -> executeAction(record) -> drawListenScreen
//     SELECT press -> mainMenu() -> return -> refresh -> resume
// ============================================================

// OLED screen for the idle/listening state
static void drawListenScreen(const RecUI &u) {
  oledClear();
  static const char spin[] = "|/-\\";
  char rs[3] = { spin[(millis() / 200) % 4], 0, 0 };
  drawHeader("LISTENING", rs);

  // Show up to 3 loaded records with name snippets
  display.setCursor(0, 11);
  if (activeCfgCount == 0) {
    display.print("No records loaded");
  } else {
    String slots;
    for (uint8_t i = 0; i < activeCfgCount && i < 3; i++) {
      if (i) slots += " ";
      char rbuf[10]; snprintf(rbuf, sizeof(rbuf), "R%u", activeCfg[i].record);
      slots += String(rbuf);
      if (activeCfg[i].name[0]) {
        slots += ":"; slots += String(activeCfg[i].name).substring(0, 4);
      }
    }
    if (activeCfgCount > 3) slots += "..";
    display.print(slots.substring(0, 21));
  }

  display.setTextSize(2); display.setCursor(16, 22); display.print("LISTENING"); display.setTextSize(1);

  // Animated sweep bar proves the loop is alive
  int p = (millis() / 10) % 208; if (p > 104) p = 208 - p;
  display.drawRect(0, 40, 128, 5, SSD1306_WHITE);
  display.fillRect(p + 1, 41, 22, 3, SSD1306_WHITE);

  if (u.lastRecord >= 0) {
    char buf[24];
    snprintf(buf, sizeof(buf), "Last:R%02d %lus ago",
             u.lastRecord, (unsigned long)((millis() - u.lastEventMs) / 1000));
    display.setCursor(0, 47); display.print(buf);
  }
  drawFooter("SEL=menu");
  display.display();
}

static void listenLoop() {
  LOGI("RECOG", "=========== LISTEN LOOP (auto mode) ===========");

  // Ensure at least some records are loaded
  refreshActiveMap();
  if (activeRecordCount == 0) {
    LOGW("RECOG", "nothing loaded on entry – attempting auto-load");
    vrBusy = true; autoLoadEnabled(); vrBusy = false;
    refreshActiveMap();
  }
  rebuildActiveCache();

  if (activeRecordCount == 0) {
    LOGW("RECOG", "still no records after auto-load attempt");
    oledClear(); drawHeader("LISTENING");
    display.setCursor(0, 16); display.print("No records loaded.");
    display.setCursor(0, 26); display.print("Menu > Train Record");
    display.setCursor(0, 36); display.print("then enable + save.");
    drawFooter("SEL=menu"); display.display();
    waitSelect();  // wait for SELECT to open menu; returns to loop()
    return;
  }

  LOGI("RECOG", "auto-listen: active=[%s]", listActive().c_str());
  for (uint8_t i = 0; i < activeRecordCount; i++)
    LOGI("RECOG", "  slot %u = R%u \"%s\" %s",
         i, activeRecords[i],
         (i < activeCfgCount) ? activeCfg[i].name : "",
         (i < activeCfgCount && activeCfg[i].enabled) ? "en" : "dis");

  vrBusy = true; myVR.flushInput();

  RecUI ui; memset(&ui, 0, sizeof(ui)); ui.lastRecord = -1; ui.state = UI_LISTEN;
  uint32_t polls = 0, errors = 0, events = 0;
  uint32_t lastDraw = 0, lastBeat = millis();
  drawListenScreen(ui);

  while (true) {
    Button b = readButton();

    // SELECT: pause listen loop, open main menu, then resume
    if (b == SELECT_PRESSED) {
      vrBusy = false;
      LOGI("RECOG", "pausing listen loop for main menu");
      mainMenu();
      // After menu returns, refresh everything and continue
      vrBusy = true;
      refreshActiveMap();
      rebuildActiveCache();
      myVR.flushInput();
      lastDraw = 0; lastBeat = millis();
      drawListenScreen(ui);
      continue;
    }

    if (b == UP_PRESSED) {
      ui.hits = ui.nomatch = ui.bad = ui.timeouts = 0;
      polls = errors = events = 0;
      LOGI("RECOG", "counters reset");
    }

    pollSerial();
    // Server / DNS pumped by pollBackground in receive_pkt gaps
    // but also explicitly here to ensure responsiveness
    if (serverRunning)  server.handleClient();
    if (portalRunning)  dnsServer.processNextRequest();

    // Poll VR module (60ms timeout – same as v1)
    int n = myVR.receive_pkt(vrPacket, 60); polls++;

    if (n > 0) {
      events++;
      LOGI("RECOG", "----- LISTEN EVENT #%lu (%d bytes) -----", (unsigned long)events, n);
      logHex("RECOG", "raw packet", vrPacket, n);

      RecogResult r; int pr = parseRecognition(vrPacket, n, r);
      LOGD("RECOG", "cmd=0x%02X status=0x%02X grp=0x%02X phys=%u idx=%u sigLen=%u",
           r.cmd, r.sub, r.group, r.physical, r.index, r.sigLen);
      if (r.sigLen) logHex("RECOG", "sig", r.sig, r.sigLen > 16 ? 16 : r.sigLen);

      if (pr == PARSE_MATCH) {
        ui.hits++; ui.lastRecord = r.physical; ui.lastEventMs = millis();
        LOGI("RECOG", ">>> MATCH: R%u (slot %u)  hits=%lu", r.physical, r.index, (unsigned long)ui.hits);
        // Dispatch HTTP action – VR released during this time
        vrBusy = false;
        executeAction(r.physical);   // RECOGNIZED -> EXECUTING -> API SUCCESS/FAILED
        vrBusy = true;
        myVR.flushInput();           // discard any events queued during action
        lastDraw = 0;                // force screen refresh
      } else if (pr == PARSE_NOMATCH) {
        ui.nomatch++;
        LOGI("RECOG", ">>> NO MATCH (0xFF) – sound heard, no loaded word matched");
      } else if (pr == PARSE_BADINDEX) {
        ui.bad++;
        LOGE("RECOG", ">>> INVALID record number %u", r.physical);
      } else {
        ui.bad++;
        LOGE("RECOG", ">>> INVALID PACKET (see PARSE warnings above)");
      }

    } else if (n == VR::TIMEOUT) {
      ui.timeouts++;  // normal: nobody spoke
    } else {
      errors++;
      if (errors <= 5 || (errors % 50) == 0)
        LOGW("RECOG", "receive_pkt error %d (%s) [total %lu]",
             n, vrRetStr(n).c_str(), (unsigned long)errors);
    }

    // Refresh OLED periodically (spinner + last-match info)
    if (millis() - lastDraw >= 70) { drawListenScreen(ui); lastDraw = millis(); }

    // Heartbeat serial log every 5 s
    if (millis() - lastBeat >= 5000) {
      lastBeat = millis();
      LOGI("RECOG",
           "heartbeat: polls=%lu idle=%lu hits=%lu nomatch=%lu bad=%lu rxErr=%lu heap=%u",
           (unsigned long)polls, (unsigned long)ui.timeouts, (unsigned long)ui.hits,
           (unsigned long)ui.nomatch, (unsigned long)ui.bad,
           (unsigned long)errors, ESP.getFreeHeap());
    }
    yield();
  }
  // Not normally reached; listenLoop is re-entered from loop()
  vrBusy = false;
}

// ============================================================
// I2C SCAN  — unchanged from v1
// ============================================================
static void i2cScan() {
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { LOGI("I2C", "device at 0x%02X", a); found++; }
  }
  if (!found) LOGE("I2C", "NO I2C devices. Check OLED SDA=D2 SCL=D1, VCC, GND");
}

// ============================================================
// SETUP  — v2 boot sequence
// ============================================================
void setup() {
  Serial.begin(115200); delay(200); Serial.println();
  Serial.println(F("=============================================="));
  Serial.println(F(" ESP8266 VOICE AUTOMATION BOT  v2"));
  Serial.println(F("=============================================="));
  LOGI("BOOT", "chip=%06X cpu=%uMHz flash=%uKB sdk=%s",
       ESP.getChipId(), ESP.getCpuFreqMHz(),
       ESP.getFlashChipSize() / 1024, ESP.getSdkVersion());
  LOGI("BOOT", "reset: %s  heap: %u", ESP.getResetReason().c_str(), ESP.getFreeHeap());

  for (int i = 0; i < VR_MAX_RECORD; i++) trainedState[i] = -1;

  pinMode(BTN_UP,     INPUT_PULLUP);
  pinMode(BTN_DOWN,   INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT);  // GPIO16: needs external 10k pull-up to 3V3
  LOGI("BOOT", "button idle: UP=%d DOWN=%d SELECT=%d (all should be 1)",
       digitalRead(BTN_UP), digitalRead(BTN_DOWN), digitalRead(BTN_SELECT));
  if (digitalRead(BTN_SELECT) == LOW)
    LOGW("BOOT", "SELECT (D0) reads LOW while idle -> missing 10k pull-up to 3V3 or shorted");

  Wire.begin(OLED_SDA, OLED_SCL); Wire.setClock(400000); i2cScan();
  bool oled = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR_A);
  if (!oled) {
    LOGW("OLED", "no OLED at 0x%02X, trying 0x%02X", OLED_ADDR_A, OLED_ADDR_B);
    oled = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR_B);
  }
  if (!oled) {
    while (true) { LOGE("OLED", "SSD1306 init FAILED. Halted."); delay(2000); }
  }
  LOGI("OLED", "SSD1306 OK");
  WiFi.persistent(false);

  // ---- Step 1: OLED BOOT ----
  bootScreen("BOOTING...", 5);

  // ---- Step 2: LittleFS ----
  bootScreen("Loading storage...", 15);
  if (!LittleFS.begin()) {
    LOGE("FS", "mount failed, formatting");
    bootScreen("Formatting FS...", 18);
    LittleFS.format();
    if (!LittleFS.begin()) {
      LOGE("FS", "LittleFS FAILED even after format");
      oledClear(); display.println("FILESYSTEM FAILED"); display.display();
      while (true) delay(1000);
    }
  }
  LOGI("FS", "LittleFS OK");

  // ---- Step 3: Check VR module ----
  bootScreen("Check VR module...", 25);
  myVR.begin(VR_BAUD); delay(500);
  bool vrOk = refreshActiveMap();
  if (vrOk) {
    bootScreen("VR ONLINE", 35);
    LOGI("BOOT", "VR module ONLINE");
  } else {
    bootScreen("VR ERROR (continue)", 35);
    LOGW("BOOT", "VR module NO REPLY (menu still usable; use Module Test)");
  }
  delay(400);

  // ---- Step 4: Scan trained records ----
  bootScreen("Scanning trained...", 42);
  if (vrOk) {
    vrBusy = true;
    int tc = scanTrained(); vrBusy = false;
    if      (tc >= 0)  LOGI("BOOT", "scan done: %d trained records", tc);
    else if (tc == -1) LOGW("BOOT", "scan aborted (module not responding)");
    else               LOGW("BOOT", "scan cancelled");
    char scanLine[28]; snprintf(scanLine, sizeof(scanLine), "Found %d trained", tc >= 0 ? tc : 0);
    bootScreen(scanLine, 55);
  } else {
    bootScreen("Scan skipped (VR off)", 55);
  }
  delay(300);

  // ---- Step 5: Load record config ----
  bootScreen("Loading config...", 60);
  int cfgFound = 0;
  for (uint8_t r = 0; r < VR_MAX_RECORD; r++) {
    if (LittleFS.exists(cfgPath(r))) cfgFound++; yield();
  }
  LOGI("CONFIG", "%d config files found in /cmd/", cfgFound);
  char cfgLine[28]; snprintf(cfgLine, sizeof(cfgLine), "%d configs found", cfgFound);
  bootScreen(cfgLine, 63);

  // ---- Step 6: Auto-load enabled+trained records by priority ----
  bootScreen("Auto-loading...", 68);
  if (vrOk) {
    vrBusy = true; autoLoadEnabled(); vrBusy = false;
  } else {
    LOGW("BOOT", "skipping auto-load (VR offline)");
  }

  // ---- Step 7: Connect saved WiFi (non-blocking on failure) ----
  bootScreen("WiFi connecting...", 78);
  loadWiFiConfig();
  bool connected = wifiConfigValid ? connectSavedWiFi() : false;

  if (connected) {
    bootScreen("WIFI CONNECTED", 88);
    LOGI("WIFI", "connected: %s  ip=%s", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    startWebServer();  // Always-on config portal
    LOGI("HTTP", "portal: http://%s", WiFi.localIP().toString().c_str());
    delay(400);
  } else {
    LOGW("BOOT", "WiFi not connected -> starting AP portal (SELECT=skip)");
    bootScreen("WIFI OFFLINE", 88);
    delay(300);
    runPortal();  // Blocking AP setup; SELECT skips to offline mode
    if (WiFi.status() == WL_CONNECTED) {
      startWebServer();
      LOGI("HTTP", "portal (post-AP): http://%s", WiFi.localIP().toString().c_str());
    }
  }

  // ---- Ready ----
  bootScreen("LISTENING", 100);
  delay(500);

  oledClear(); drawHeader("READY");
  bool wifiUp = (WiFi.status() == WL_CONNECTED);
  display.setCursor(0, 12);
  display.print(wifiUp ? "WiFi: " : "WiFi: OFFLINE");
  if (wifiUp) { display.print(WiFi.SSID()); display.setCursor(0, 22); display.print("http://"); display.print(WiFi.localIP()); }
  display.setCursor(0, 32); display.print("VR:   "); display.print(vrOnline ? "online" : "NO REPLY");
  display.setCursor(0, 42); display.print("Slots: "); display.print(activeRecordCount); display.print("/7 loaded");
  display.setCursor(0, 52); display.print("Starting listen...");
  display.display();
  printHelp();
  delay(1000);
}

// ============================================================
// LOOP  — v2: listenLoop() is the primary mode
//   SELECT from within listenLoop() opens mainMenu().
//   On menu exit, listenLoop() is re-entered automatically.
// ============================================================
void loop() {
  listenLoop();
}
