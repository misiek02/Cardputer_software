// Servo Control - sterowanie dwoma serwami SG90 z Cardputer ADV
//
// Podlaczenie domyslne (port Grove HY2.0-4P):
//   Grove GND (czarny)  -> brazowy przewod obu serw
//   Grove 5V  (czerwony)-> czerwony przewod obu serw (lub zewn. 5V, patrz README)
//   G1 (bialy)          -> pomaranczowy (sygnal) serwa 1
//   G2 (zolty)          -> pomaranczowy (sygnal) serwa 2
// Piny mozna zmienic w aplikacji (klawisz p) - wybor zapisuje sie w pamieci.
//
// Ekran pokazuje symulacje: ramie (orczyk) obraca sie z predkoscia zblizona
// do prawdziwego SG90, wiec widac ruch nawet bez podlaczonych serw.
//
// Klawisze:
//   1 / 2 / TAB wybor serwa
//   , /         w lewo / w prawo (strzalki), mozna przytrzymac
//   ; .         krok wiekszy / mniejszy (1, 5, 15, 45 stopni)
//   c           oba serwa na 90 stopni
//   0 / 9       0 / 180 stopni
//   l           tryb "link" - oba serwa ruszaja sie razem
//   d           tryb demo (sweep tam i z powrotem)
//   x           wlacz/wylacz sygnal PWM
//   t           typ wybranego serwa: 180 st. (pozycyjne) / 360 st. (ciagle obroty)
//   [ ]         trim punktu STOP serwa 360 (-/+ 5 us)
//   p           konfiguracja pinow
//   w           Wi-Fi: sterowanie z telefonu / komputera przez przegladarke
//   h           pomoc
//
// Sterowanie przez siec, dwa tryby (klawisz m na ekranie Wi-Fi):
//  - wlasna siec: Cardputer tworzy siec "ServoCtl-XXXX" (haslo i QR na ekranie Wi-Fi),
//    strona http://192.168.4.1/; opcjonalnie laczy sie tez z istniejaca siecia,
//  - istniejaca siec: tylko klient istniejacej sieci (wybor sieci: klawisz n),
//    strona pod adresem IP z ekranu Wi-Fi lub http://servo.local/.
// Strona: src/web_ui.h, komendy WebSocket: handleNetCmd().

#include <M5Cardputer.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
#include "web_ui.h"

// ---------- konfiguracja ----------
// Piny, ktore mozna przypisac serwom. Pominiete: G8/G9 (wewn. I2C klawiatury).
// G40/G14/G39 to magistrala karty SD - dzialaja, bo ta aplikacja nie uzywa SD.
struct PinOpt {
  uint8_t gpio;
  const char *where;
  bool sdBus;
};
static constexpr PinOpt PIN_OPTS[] = {
    {1, "Grove bialy", false}, {2, "Grove zolty", false}, {3, "EXT pin 1", false},
    {4, "EXT pin 3", false},   {6, "EXT pin 5", false},   {5, "EXT pin 13", false},
    {13, "EXT pin 12", false}, {15, "EXT pin 14", false}, {40, "EXT pin 7", true},
    {14, "EXT pin 9", true},   {39, "EXT pin 11", true},
};
static constexpr int NUM_PIN_OPTS = sizeof(PIN_OPTS) / sizeof(PIN_OPTS[0]);
static constexpr int PULSE_MIN_US = 500;      // SG90: ~0.5 ms = 0 st.
static constexpr int PULSE_MAX_US = 2400;     // SG90: ~2.4 ms = 180 st.
// serwo 360 (ciagle obroty): 90 = STOP, 0 / 180 = pelna predkosc w lewo / w prawo
static constexpr int CONT_MID_US = 1500;      // srodek = stop (korekta: trim)
static constexpr int CONT_SPAN_US = 500;      // 1000..2000 us
static constexpr int TRIM_MAX_US = 200;
static constexpr int TRIM_STEP_US = 5;
static constexpr int PWM_FREQ_HZ = 50;        // okres 20 ms
static constexpr ledc_timer_bit_t PWM_RES = LEDC_TIMER_14_BIT;
static constexpr ledc_timer_t PWM_TIMER = LEDC_TIMER_1;  // podswietlenie uzywa innego timera
static constexpr ledc_channel_t PWM_CH[2] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1};

static constexpr float SIM_SPEED_DPS = 500.0f;  // SG90: ~0.1 s / 60 st.
static constexpr int STEPS[] = {1, 5, 15, 45};
static constexpr int NUM_STEPS = sizeof(STEPS) / sizeof(STEPS[0]);
static constexpr int REPEAT_DELAY_MS = 350;
static constexpr int REPEAT_RATE_MS = 50;
static constexpr uint32_t FRAME_MS = 16;

// ---------- stan ----------
static int target[2] = {90, 90};    // kat zadany (wysylany do serwa)
static float simPos[2] = {90, 90};  // symulowana pozycja ramienia
static int selected = 0;
static int stepIdx = 1;
static bool linked = false;
static bool demo = false;
static bool outputOn = true;
static bool showHelp = false;
static int demoDir = 1;
static int pinIdx[2] = {0, 1};  // indeksy w PIN_OPTS (domyslnie G1, G2)
static bool cont[2] = {false, false};  // true = serwo 360 (ciagle obroty)
static int trimUs[2] = {0, 0};         // korekta punktu STOP serwa 360

// ekran konfiguracji pinow
static bool pinScreen = false;
static int pinEdit[2];
static int pinRow = 0;

static Preferences prefs;
static bool dirty = true;

// siec
enum StaState { STA_NONE, STA_CONNECTING, STA_OK, STA_FAIL };
enum NetMode { NET_AP = 0, NET_STA = 1 };  // wlasna siec (+opcjonalnie istniejaca) / tylko istniejaca
static NetMode netMode = NET_AP;
static uint32_t restartAt = 0;              // odroczony restart Wi-Fi (zmiana trybu ze strony)
static constexpr uint32_t STA_RETRY_MS = 5000;
static bool wifiOn = false;
static bool wifiScreen = false;
static bool qrShowsUrl = false;
static String apSsid, apPass, staSsid, staPass;
static StaState staState = STA_NONE;
static uint32_t staStart = 0;
static constexpr uint32_t STA_TIMEOUT_MS = 15000;
static constexpr uint32_t NET_STATE_MS = 40;  // max ~25 aktualizacji/s do przegladarek
static const IPAddress AP_IP(192, 168, 4, 1);
static WebServer http(80);
static WebSocketsServer wsrv(81);
static DNSServer dns;
static bool forceBroadcast = false;

// wybor istniejacej sieci na Cardputerze: skan -> lista -> haslo
enum NetUi { NU_NONE, NU_SCAN, NU_LIST, NU_PASS };
struct ScanNet {
  String ssid;
  int rssi;
  bool open;
};
static NetUi netUi = NU_NONE;
static std::vector<ScanNet> scanNets;
static int scanSel = 0;
static bool scanFailed = false;
static bool scanStarted = false;
static uint32_t scanDeadline = 0;
static constexpr uint32_t SCAN_RETRY_MS = 4000;  // tyle ponawiamy start skanu
static bool scanTempRadio = false;  // radio wlaczone tylko na czas skanu
static String passEdit;
static constexpr int LIST_ROWS = 9;

static M5Canvas canvas(&M5Cardputer.Display);

// ---------- kolory ----------
static uint16_t C_BG, C_PANEL, C_TRACK, C_ACCENT, C_IDLE, C_OFF, C_BODY, C_TEXT, C_DIM, C_OK, C_BAD;

static void initColors() {
  C_BG = canvas.color565(10, 14, 22);
  C_PANEL = canvas.color565(24, 30, 44);
  C_TRACK = canvas.color565(40, 48, 66);
  C_ACCENT = canvas.color565(255, 150, 30);
  C_IDLE = canvas.color565(90, 160, 230);
  C_OFF = canvas.color565(90, 90, 100);
  C_BODY = canvas.color565(30, 90, 200);  // niebieska obudowa SG90
  C_TEXT = canvas.color565(235, 238, 245);
  C_DIM = canvas.color565(120, 130, 150);
  C_OK = canvas.color565(70, 210, 110);
  C_BAD = canvas.color565(235, 70, 70);
}

// ---------- serwa ----------
static int angleToUs(int i, int deg) {
  if (cont[i]) return CONT_MID_US + trimUs[i] + (deg - 90) * CONT_SPAN_US / 90;
  return PULSE_MIN_US + (PULSE_MAX_US - PULSE_MIN_US) * deg / 180;
}

static uint32_t angleToDuty(int i, int deg) {
  return (uint32_t)angleToUs(i, deg) * ((1u << PWM_RES) - 1) / (1000000u / PWM_FREQ_HZ);
}

// predkosc serwa 360 w procentach (-100..100)
static int speedPct(int deg) { return (deg - 90) * 100 / 90; }

static void servoWrite(int i) {
  const uint32_t duty = outputOn ? angleToDuty(i, target[i]) : 0;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH[i], duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CH[i]);
}

static void servoSetup() {
  ledc_timer_config_t t = {};
  t.speed_mode = LEDC_LOW_SPEED_MODE;
  t.duty_resolution = PWM_RES;
  t.timer_num = PWM_TIMER;
  t.freq_hz = PWM_FREQ_HZ;
  t.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&t);

  for (int i = 0; i < 2; i++) {
    ledc_channel_config_t c = {};
    c.gpio_num = PIN_OPTS[pinIdx[i]].gpio;
    c.speed_mode = LEDC_LOW_SPEED_MODE;
    c.channel = PWM_CH[i];
    c.timer_sel = PWM_TIMER;
    c.duty = angleToDuty(i, target[i]);
    c.hpoint = 0;
    ledc_channel_config(&c);
  }
}

static int findPinOpt(int gpio) {
  for (int k = 0; k < NUM_PIN_OPTS; k++)
    if (PIN_OPTS[k].gpio == gpio) return k;
  return -1;
}

static void loadPins() {
  prefs.begin("servoctl", true);
  const int a = findPinOpt(prefs.getUChar("pin0", 1));
  const int b = findPinOpt(prefs.getUChar("pin1", 2));
  for (int i = 0; i < 2; i++) {
    char key[8];
    snprintf(key, sizeof(key), "cont%d", i);
    cont[i] = prefs.getBool(key, false);
    snprintf(key, sizeof(key), "trim%d", i);
    trimUs[i] = constrain(prefs.getShort(key, 0), -TRIM_MAX_US, TRIM_MAX_US);
  }
  prefs.end();
  if (a >= 0 && b >= 0 && a != b) {
    pinIdx[0] = a;
    pinIdx[1] = b;
  }
}

static void savePins() {
  prefs.begin("servoctl", false);
  prefs.putUChar("pin0", PIN_OPTS[pinIdx[0]].gpio);
  prefs.putUChar("pin1", PIN_OPTS[pinIdx[1]].gpio);
  prefs.end();
}

static void saveServoType(int i) {
  char key[8];
  prefs.begin("servoctl", false);
  snprintf(key, sizeof(key), "cont%d", i);
  prefs.putBool(key, cont[i]);
  snprintf(key, sizeof(key), "trim%d", i);
  prefs.putShort(key, trimUs[i]);
  prefs.end();
}

// zmiana typu serwa; potem STOP / srodek, zeby serwo 360 nie ruszylo samo
static void setServoType(int i, bool c) {
  if (cont[i] == c) return;
  cont[i] = c;
  target[i] = 90;
  simPos[i] = 90;
  servoWrite(i);
  saveServoType(i);
  dirty = true;
}

static void setTrim(int i, int us) {
  us = constrain(us, -TRIM_MAX_US, TRIM_MAX_US);
  if (us == trimUs[i]) return;
  trimUs[i] = us;
  servoWrite(i);
  saveServoType(i);
  dirty = true;
}

// przepina kanaly PWM na nowe piny; stare piny wracaja do stanu domyslnego
static void applyPins(const int newIdx[2]) {
  for (int i = 0; i < 2; i++) {
    if (newIdx[i] != pinIdx[i]) gpio_reset_pin((gpio_num_t)PIN_OPTS[pinIdx[i]].gpio);
  }
  for (int i = 0; i < 2; i++) {
    pinIdx[i] = newIdx[i];
    ledc_channel_config_t c = {};
    c.gpio_num = PIN_OPTS[pinIdx[i]].gpio;
    c.speed_mode = LEDC_LOW_SPEED_MODE;
    c.channel = PWM_CH[i];
    c.timer_sel = PWM_TIMER;
    c.duty = outputOn ? angleToDuty(i, target[i]) : 0;
    c.hpoint = 0;
    ledc_channel_config(&c);
  }
}

static void closePinMode(bool save);

static void setAngle(int i, int deg) {
  deg = constrain(deg, 0, 180);
  if (deg == target[i]) return;
  target[i] = deg;
  servoWrite(i);
  dirty = true;
}

static void setActive(int deg) {
  if (linked) {
    setAngle(0, deg);
    setAngle(1, deg);
  } else {
    setAngle(selected, deg);
  }
}

static void moveActive(int delta) {
  if (linked) {
    setAngle(0, target[0] + delta);
    setAngle(1, target[1] + delta);
  } else {
    setAngle(selected, target[selected] + delta);
  }
}

// symulacja: ramie dojezdza do celu ze stala predkoscia (tylko gdy PWM wlaczony)
static void updateSim(float dt) {
  if (!outputOn) return;
  for (int i = 0; i < 2; i++) {
    if (cont[i]) {  // serwo 360: wskazowka predkosci, bez opoznienia
      if (simPos[i] != target[i]) dirty = true;
      simPos[i] = target[i];
      continue;
    }
    const float diff = target[i] - simPos[i];
    if (fabsf(diff) < 0.01f) continue;
    const float step = SIM_SPEED_DPS * dt;
    simPos[i] = fabsf(diff) <= step ? (float)target[i] : simPos[i] + copysignf(step, diff);
    dirty = true;
  }
}

// ---------- siec: Wi-Fi, strona WWW, WebSocket ----------
static void loadNet() {
  char buf[16];
  snprintf(buf, sizeof(buf), "ServoCtl-%04X", (unsigned)(ESP.getEfuseMac() >> 32) & 0xFFFF);
  apSsid = buf;

  prefs.begin("servoctl", false);
  wifiOn = prefs.getBool("wifi", false);
  netMode = prefs.getUChar("net_mode", NET_AP) == NET_STA ? NET_STA : NET_AP;
  staSsid = prefs.getString("sta_ssid", "");
  staPass = prefs.getString("sta_pass", "");
  apPass = prefs.getString("ap_pass", "");
  if (apPass.length() < 8) {  // losowe haslo przy pierwszym uruchomieniu
    snprintf(buf, sizeof(buf), "%08lu", (unsigned long)(esp_random() % 100000000UL));
    apPass = buf;
    prefs.putString("ap_pass", apPass);
  }
  prefs.end();
}

static void staConnect() {
  WiFi.disconnect(false);
  if (staSsid.isEmpty()) {
    staState = STA_NONE;
  } else {
    WiFi.begin(staSsid.c_str(), staPass.c_str());
    staState = STA_CONNECTING;
    staStart = millis();
  }
  dirty = true;
}

static void saveStaCreds(NetMode m) {
  prefs.begin("servoctl", false);
  prefs.putString("sta_ssid", staSsid);
  prefs.putString("sta_pass", staPass);
  prefs.putUChar("net_mode", m);
  prefs.end();
}

static String jsonEscape(const String &in) {
  String out;
  for (char c : in) {
    if (c == '"' || c == '\\') out += '\\';
    if ((uint8_t)c >= 0x20) out += c;
  }
  return out;
}

static String stateJson() {
  char buf[128];
  snprintf(buf, sizeof(buf),
           "{\"t\":[%d,%d],\"o\":%d,\"l\":%d,\"d\":%d,\"s\":%d,\"k\":%d,\"g\":[%d,%d],\"c\":[%d,%d],\"r\":[%d,%d]}",
           target[0], target[1], outputOn, linked, demo, selected, stepIdx, PIN_OPTS[pinIdx[0]].gpio,
           PIN_OPTS[pinIdx[1]].gpio, cont[0], cont[1], trimUs[0], trimUs[1]);
  return buf;
}

// komendy z przegladarki: "a i deg", "b deg", "s i", "k n", "l 0|1", "d 0|1", "x 0|1", "p g1 g2",
// "m i 0|1" (typ serwa: 180 / 360), "r i us" (trim serwa 360)
static void handleNetCmd(const char *msg) {
  int a = 0, b = 0;
  const int n = sscanf(msg + 1, "%d %d", &a, &b);
  switch (msg[0]) {
    case 'a': {
      if (n < 2) return;
      const int i = a & 1, o = i ^ 1;
      b = constrain(b, 0, 180);
      demo = false;
      if (linked) setAngle(o, target[o] + (b - target[i]));  // link: drugie o te sama roznice
      setAngle(i, b);
      break;
    }
    case 'b':
      if (n < 1) return;
      demo = false;
      setAngle(0, a);
      setAngle(1, a);
      break;
    case 's': selected = a & 1; break;
    case 'k': stepIdx = constrain(a, 0, NUM_STEPS - 1); break;
    case 'l': linked = a; break;
    case 'd': demo = a; break;
    case 'x':
      outputOn = a;
      servoWrite(0);
      servoWrite(1);
      break;
    case 'p': {
      if (n < 2) return;
      const int idx[2] = {findPinOpt(a), findPinOpt(b)};
      if (idx[0] < 0 || idx[1] < 0 || idx[0] == idx[1]) return;
      applyPins(idx);
      savePins();
      if (pinScreen) closePinMode(false);
      break;
    }
    case 'm':
      if (n < 2) return;
      setServoType(a & 1, b);
      break;
    case 'r':
      if (n < 2) return;
      setTrim(a & 1, b);
      break;
    default: return;
  }
  dirty = true;
}

static void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED:
      forceBroadcast = true;
      dirty = true;
      break;
    case WStype_DISCONNECTED: dirty = true; break;
    case WStype_TEXT: {
      char buf[32];
      len = min(len, sizeof(buf) - 1);
      memcpy(buf, payload, len);
      buf[len] = 0;
      handleNetCmd(buf);
      break;
    }
    default: break;
  }
}

static void sendInfo() {
  String j = "{\"ap\":\"" + apSsid + "\",\"ssid\":\"" + jsonEscape(staSsid) + "\",\"sta\":" + String((int)staState) +
             ",\"mode\":" + String((int)netMode) + ",\"ip\":\"" + WiFi.localIP().toString() + "\",\"pins\":[";
  for (int k = 0; k < NUM_PIN_OPTS; k++) {
    if (k) j += ',';
    j += "[" + String(PIN_OPTS[k].gpio) + ",\"" + PIN_OPTS[k].where + "\"," + (PIN_OPTS[k].sdBus ? "1" : "0") + "]";
  }
  j += "]}";
  http.send(200, "application/json", j);
}

static void setupRoutes() {
  http.on("/", HTTP_GET, [] {
    http.sendHeader("Cache-Control", "no-cache");
    http.send_P(200, "text/html; charset=utf-8", WEB_UI);
  });
  http.on("/info", HTTP_GET, sendInfo);
  http.on("/wifi", HTTP_POST, [] {
    staSsid = http.arg("ssid");
    staPass = http.arg("pass");
    // tryb "tylko istniejaca siec" wymaga SSID - inaczej Cardputer bylby nieosiagalny
    const NetMode m = (http.arg("mode") == "1" && !staSsid.isEmpty()) ? NET_STA : NET_AP;
    saveStaCreds(m);
    http.send(200, "text/plain", "ok");
    if (m != netMode) {
      netMode = m;
      restartAt = millis() + 500;  // po wyslaniu odpowiedzi (zmiana trybu zrywa polaczenie)
    } else {
      staConnect();
    }
  });
  // "captive portal": telefon podlaczony do sieci Cardputera sam otwiera strone sterowania
  http.onNotFound([] {
    if (http.client().localIP() == AP_IP && http.hostHeader() != AP_IP.toString()) {
      http.sendHeader("Location", "http://192.168.4.1/", true);
      http.send(302, "text/plain", "");
    } else {
      http.send(404, "text/plain", "not found");
    }
  });
}

static void wifiStart() {
  static bool routes = false;
  if (!routes) {
    setupRoutes();
    wsrv.onEvent(onWsEvent);
    routes = true;
  }
  WiFi.persistent(false);  // dane sieci trzymamy sami w Preferences
  if (netMode == NET_AP) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(apSsid.c_str(), apPass.c_str());
    dns.start(53, "*", AP_IP);
  } else {
    WiFi.mode(WIFI_STA);
  }
  WiFi.setSleep(false);    // mniejsze opoznienia sterowania
  WiFi.setHostname("servo");
  WiFi.setAutoReconnect(false);  // ponawianiem zarzadza netLoop (auto-reconnect blokuje skanowanie)
  http.begin();
  wsrv.begin();
  if (MDNS.begin("servo")) MDNS.addService("http", "tcp", 80);
  staConnect();
  wifiOn = true;
}

static void wifiStop() {
  wsrv.close();
  http.stop();
  dns.stop();
  MDNS.end();
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  staState = STA_NONE;
  wifiOn = false;
}

static void setWifi(bool on) {
  if (on == wifiOn) return;
  on ? wifiStart() : wifiStop();
  prefs.begin("servoctl", false);
  prefs.putBool("wifi", wifiOn);
  prefs.end();
  dirty = true;
}

static void wifiRestart() {
  if (wifiOn) wifiStop();
  wifiStart();
  dirty = true;
}

static void setNetMode(NetMode m) {
  if (m == netMode) return;
  netMode = m;
  prefs.begin("servoctl", false);
  prefs.putUChar("net_mode", netMode);
  prefs.end();
  if (wifiOn) wifiRestart();
  dirty = true;
}

// ---- wybor sieci na Cardputerze ----
// proba startu skanu; ESP32 odrzuca skan, gdy STA jest w trakcie laczenia
// albo radio dopiero startuje - wtedy pollScan ponawia probe
static bool tryStartScan() {
  WiFi.scanDelete();
  const int16_t r = WiFi.scanNetworks(true, false);
  Serial.printf("[scan] start -> %d\n", r);
  return r != WIFI_SCAN_FAILED;
}

static void startScan() {
  if (!wifiOn && !scanTempRadio) {
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    scanTempRadio = true;
  }
  // przerywamy laczenie (wznawia sie po wyborze sieci / wyjsciu z listy)
  if (staState != STA_OK) {
    WiFi.setAutoReconnect(false);
    WiFi.disconnect(false);
    if (wifiOn) staState = STA_NONE;
  }
  scanStarted = tryStartScan();
  scanDeadline = millis() + SCAN_RETRY_MS;
  netUi = NU_SCAN;
  dirty = true;
}

static void pollScan() {
  if (netUi != NU_SCAN) return;
  static uint32_t lastTry = 0;
  const uint32_t now = millis();
  if (!scanStarted) {
    if ((int32_t)(now - scanDeadline) < 0) {
      if (now - lastTry < 250) return;
      lastTry = now;
      WiFi.disconnect(false);
      scanStarted = tryStartScan();
      return;
    }
  }
  const int16_t n = scanStarted ? WiFi.scanComplete() : (int16_t)WIFI_SCAN_FAILED;
  if (n == WIFI_SCAN_RUNNING) return;
  if (n == WIFI_SCAN_FAILED && scanStarted && (int32_t)(now - scanDeadline) < 0) {
    scanStarted = false;  // skan przerwany - jeszcze raz
    return;
  }
  Serial.printf("[scan] wynik %d\n", n);
  scanNets.clear();
  scanFailed = n < 0;
  for (int k = 0; k < n; k++) {
    const String s = WiFi.SSID(k);
    if (s.isEmpty()) continue;
    bool dup = false;
    for (auto &e : scanNets)
      if (e.ssid == s) {
        e.rssi = max(e.rssi, (int)WiFi.RSSI(k));
        dup = true;
      }
    if (!dup) scanNets.push_back({s, WiFi.RSSI(k), WiFi.encryptionType(k) == WIFI_AUTH_OPEN});
  }
  WiFi.scanDelete();
  std::sort(scanNets.begin(), scanNets.end(), [](const ScanNet &a, const ScanNet &b) { return a.rssi > b.rssi; });
  scanSel = 0;
  for (int k = 0; k < (int)scanNets.size(); k++)
    if (scanNets[k].ssid == staSsid) scanSel = k;  // zapamietana siec na poczatku kursora
  netUi = NU_LIST;
  dirty = true;
}

static void closeNetUi() {
  netUi = NU_NONE;
  if (scanTempRadio && !wifiOn) WiFi.mode(WIFI_OFF);
  scanTempRadio = false;
  if (wifiOn && staState == STA_NONE) staConnect();  // wznow przerwane laczenie
  dirty = true;
}

// wybrana siec: zapis, tryb "istniejaca siec" i laczenie
static void joinNetwork(const String &ssid, const String &pass) {
  staSsid = ssid;
  staPass = pass;
  saveStaCreds(NET_STA);
  scanTempRadio = false;  // radio zostaje wlaczone - przejmuje je wifiStart
  netUi = NU_NONE;
  netMode = NET_STA;
  if (wifiOn) wifiRestart();
  else setWifi(true);
  dirty = true;
}

static void pickNetwork() {
  if (scanNets.empty()) return;
  const ScanNet &e = scanNets[scanSel];
  if (e.open) {
    joinNetwork(e.ssid, "");
    return;
  }
  passEdit = e.ssid == staSsid ? staPass : "";
  netUi = NU_PASS;
  dirty = true;
}

static void netLoop() {
  pollScan();
  if (!wifiOn) return;
  const uint32_t now = millis();
  if (restartAt && (int32_t)(now - restartAt) >= 0) {
    restartAt = 0;
    wifiRestart();
    return;
  }
  if (netMode == NET_AP) dns.processNextRequest();
  http.handleClient();
  wsrv.loop();

  if (netUi == NU_SCAN) return;  // w trakcie skanu nie ruszamy laczenia
  const bool up = WiFi.status() == WL_CONNECTED;
  if ((staState == STA_CONNECTING || staState == STA_FAIL) && up) {
    staState = STA_OK;
    dirty = true;
  } else if (staState == STA_CONNECTING && now - staStart > STA_TIMEOUT_MS) {
    // tryb z wlasna siecia: przestajemy probowac - ciagle skanowanie zakloca siec Cardputera;
    // tryb "tylko istniejaca siec": ponawiamy co STA_RETRY_MS
    staState = STA_FAIL;
    WiFi.disconnect(false);
    dirty = true;
  } else if (staState == STA_FAIL && netMode == NET_STA && now - staStart > STA_TIMEOUT_MS + STA_RETRY_MS) {
    staConnect();
  } else if (staState == STA_OK && !up) {
    staConnect();  // utrata polaczenia - laczymy od nowa (auto-reconnect frameworka wylaczony)
  }

  // stan do przegladarek tylko gdy sie zmienil (lub ktos sie podlaczyl)
  static uint32_t lastSent = 0;
  static String lastJson;
  if (now - lastSent >= NET_STATE_MS) {
    lastSent = now;
    String j = stateJson();
    if (forceBroadcast || j != lastJson) {
      wsrv.broadcastTXT(j);
      lastJson = j;
      forceBroadcast = false;
    }
  }
}

// ---------- UI ----------
// 0 st. = ramie w lewo, 90 = do gory, 180 = w prawo (zgodnie ze strzalkami)
static void polar(int cx, int cy, float r, float deg, float &x, float &y) {
  const float rad = deg * (float)M_PI / 180.0f;
  x = cx - r * cosf(rad);
  y = cy - r * sinf(rad);
}

static void drawDial(int i, int cx) {
  const int cy = 86, R = 44;
  const bool active = linked || i == selected;
  const uint16_t acc = !outputOn ? C_OFF : (active ? C_ACCENT : C_IDLE);

  // etykieta i szerokosc impulsu
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  if (active) {
    canvas.fillRoundRect(cx - 56, 16, 20, 11, 3, acc);
    canvas.setTextColor(C_BG);
  } else {
    canvas.setTextColor(acc);
  }
  canvas.drawString(i == 0 ? "S1" : "S2", cx - 52, 18);
  canvas.setTextColor(C_DIM);
  canvas.drawString(String("G") + PIN_OPTS[pinIdx[i]].gpio, cx - 32, 18);
  canvas.setTextDatum(top_right);
  canvas.setTextColor(C_DIM);
  canvas.drawString(String(outputOn ? angleToUs(i, target[i]) : 0) + "us", cx + 56, 18);

  // tor i wypelnienie lukiem (fillArc: 180 = lewo, 270 = gora, 360 = prawo)
  // serwo 360: luk od srodka (STOP) w strone predkosci
  canvas.fillArc(cx, cy, R - 5, R, 180, 360, C_TRACK);
  if (cont[i]) {
    const float a = 180 + simPos[i];
    if (fabsf(simPos[i] - 90) > 0.5f) canvas.fillArc(cx, cy, R - 5, R, min(a, 270.0f), max(a, 270.0f), acc);
  } else if (simPos[i] > 0.5f) {
    canvas.fillArc(cx, cy, R - 5, R, 180, 180 + simPos[i], acc);
  }

  // podzialka co 30 st.
  for (int t = 0; t <= 180; t += 30) {
    float x0, y0, x1, y1;
    const float len = (t % 90 == 0) ? 6 : 3;
    polar(cx, cy, R + 2, t, x0, y0);
    polar(cx, cy, R + 2 + len, t, x1, y1);
    canvas.drawLine(x0, y0, x1, y1, C_DIM);
  }

  // znacznik celu, gdy ramie jeszcze jedzie
  if (fabsf(target[i] - simPos[i]) > 0.5f) {
    float tx, ty;
    polar(cx, cy, R - 2.5f, target[i], tx, ty);
    canvas.fillCircle(tx, ty, 3, C_TEXT);
  }

  // obudowa SG90
  canvas.fillRoundRect(cx - 22, cy - 6, 44, 22, 3, C_BODY);
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(canvas.color565(170, 200, 255));
  canvas.drawString(cont[i] ? "360" : "SG90", cx, cy + 10);

  // orczyk (ramie)
  float ex, ey, bx, by;
  polar(cx, cy, R - 11, simPos[i], ex, ey);
  polar(cx, cy, 9, simPos[i] + 180, bx, by);  // krotszy koniec z drugiej strony
  const uint16_t arm = outputOn ? C_TEXT : C_OFF;
  canvas.drawWideLine(bx, by, ex, ey, 3.0f, arm);
  canvas.fillCircle(ex, ey, 3, arm);
  canvas.fillCircle(cx, cy, 6, arm);
  canvas.fillCircle(cx, cy, 2, C_BODY);

  // kat (serwo 360: predkosc w % i trim)
  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.setTextDatum(top_center);
  canvas.setTextColor(outputOn ? C_TEXT : C_OFF);
  if (cont[i]) {
    const int sp = speedPct(target[i]);
    canvas.drawString(sp == 0 ? String("STOP") : (sp > 0 ? "+" : "") + String(sp) + "%", cx, 106);
    if (trimUs[i]) {
      canvas.setFont(&fonts::Font0);
      canvas.setTextDatum(top_right);
      canvas.setTextColor(C_DIM);
      canvas.drawString("tr" + String(trimUs[i]), cx + 58, 110);
    }
    return;
  }
  const String deg = String((int)lroundf(simPos[i]));
  canvas.drawString(deg, cx, 106);
  canvas.drawCircle(cx + canvas.textWidth(deg) / 2 + 4, 109, 2, outputOn ? C_TEXT : C_OFF);  // znak stopnia
}

static void drawHelp() {
  canvas.fillRoundRect(6, 6, 228, 123, 6, C_PANEL);
  canvas.drawRoundRect(6, 6, 228, 123, 6, C_ACCENT);
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  static const char *lines[][2] = {
      {"1 2 TAB", "wybor serwa"},     {", /", "ruch lewo / prawo"},
      {"; .", "krok wiekszy/mniejszy"}, {"c", "oba na 90"},
      {"0 9", "0 / 180 stopni"},      {"l d", "link (oba razem) / demo"},
      {"x", "PWM on/off"},            {"t [ ]", "typ 180/360, trim STOP"},
      {"p", "konfiguracja pinow"},    {"w", "Wi-Fi / sterowanie WWW"},
      {"h", "zamknij pomoc"},
  };
  int y = 12;
  for (auto &l : lines) {
    canvas.setTextColor(C_ACCENT);
    canvas.drawString(l[0], 16, y);
    canvas.setTextColor(C_TEXT);
    canvas.drawString(l[1], 78, y);
    y += 10;
  }
}

static void drawChip(int &x, int y, const char *txt, uint16_t bg, uint16_t fg) {
  const int w = canvas.textWidth(txt) + 6;
  canvas.fillRoundRect(x, y, w, 11, 3, bg);
  canvas.setTextColor(fg);
  canvas.drawString(txt, x + 3, y + 2);
  x += w + 3;
}

// jedna komorka schematu zlacza
static void drawPinCell(int x, int y, int w, const char *label, int gpio) {
  uint16_t bg = C_PANEL, fg = C_DIM;
  if (gpio >= 0) {
    const int k = findPinOpt(gpio);
    if (k < 0) {
      fg = C_TRACK;  // niedostepny (G8/G9)
    } else if (k == pinEdit[0]) {
      bg = C_ACCENT;
      fg = C_BG;
    } else if (k == pinEdit[1]) {
      bg = C_IDLE;
      fg = C_BG;
    } else {
      fg = C_TEXT;
    }
  } else if (label[0] == '5') {
    fg = C_BAD;  // zasilanie
  }
  canvas.fillRoundRect(x, y, w, 10, 2, bg);
  canvas.setTextColor(fg);
  canvas.drawString(label, x + w / 2, y + 5);
}

static void drawPinScreen() {
  canvas.setFont(&fonts::Font0);

  // lewa strona: wybor pinow
  for (int i = 0; i < 2; i++) {
    const int y = 18 + i * 46;
    const bool rowSel = pinRow == i;
    const uint16_t acc = i == 0 ? C_ACCENT : C_IDLE;
    if (rowSel) canvas.drawRoundRect(2, y - 2, 116, 43, 4, acc);

    canvas.setTextDatum(top_left);
    canvas.setTextColor(acc);
    canvas.drawString(i == 0 ? "Serwo 1" : "Serwo 2", 8, y + 1);

    const PinOpt &o = PIN_OPTS[pinEdit[i]];
    canvas.setFont(&fonts::FreeSansBold9pt7b);
    canvas.setTextDatum(top_center);
    canvas.setTextColor(C_TEXT);
    canvas.drawString(String("G") + o.gpio, 60, y + 11);
    if (rowSel) {
      canvas.fillTriangle(14, y + 18, 20, y + 13, 20, y + 23, acc);
      canvas.fillTriangle(106, y + 18, 100, y + 13, 100, y + 23, acc);
    }
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(o.sdBus ? C_BAD : C_DIM);
    canvas.drawString(o.sdBus ? "linia karty SD" : o.where, 60, y + 30);
  }
  canvas.setTextDatum(top_left);
  canvas.setTextColor(C_DIM);
  canvas.drawString("GND/5V: Grove lub", 6, 108);
  canvas.drawString("EXT pin 4 / pin 6", 6, 117);

  // prawa strona: schemat zlacz
  static const char *EXT_L[] = {"G3", "G4", "G6", "G40", "G14", "G39", "G5"};
  static const int EXT_LG[] = {3, 4, 6, 40, 14, 39, 5};
  static const char *EXT_R[] = {"5VIN", "GND", "5VOUT", "G8", "G9", "G13", "G15"};
  static const int EXT_RG[] = {-1, -1, -1, 8, 9, 13, 15};
  canvas.setTextDatum(middle_center);
  canvas.setTextColor(C_DIM);
  canvas.drawString("EXT 2.54", 180, 21);
  for (int r = 0; r < 7; r++) {
    const int y = 27 + r * 11;
    drawPinCell(126, y, 52, EXT_L[r], EXT_LG[r]);
    drawPinCell(182, y, 52, EXT_R[r], EXT_RG[r]);
  }
  canvas.setTextColor(C_DIM);
  canvas.drawString("Grove", 180, 108);
  static const char *GR[] = {"GND", "5V", "G2", "G1"};
  static const int GRG[] = {-1, -1, 2, 1};
  for (int k = 0; k < 4; k++) drawPinCell(126 + k * 27, 113, 26, GR[k], GRG[k]);
}

static void drawWifiScreen() {
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  int y = 18;
  auto line = [&](const String &txt, uint16_t col, int gap = 10) {
    canvas.setTextColor(col);
    canvas.drawString(txt, 4, y);
    y += gap;
  };
  const char *modeName = netMode == NET_AP ? "wlasna siec" : "istniejaca siec";

  if (!wifiOn) {
    line("Wi-Fi wylaczone.", C_TEXT, 13);
    line(String("Tryb: ") + modeName, C_ACCENT, 13);
    if (netMode == NET_AP) {
      line("Cardputer tworzy wlasna siec", C_DIM);
      line("i strone WWW do sterowania.", C_DIM, 13);
    } else {
      line("Cardputer laczy sie z siecia", C_DIM);
      line(staSsid.isEmpty() ? String("(nie wybrano - klawisz n)") : staSsid.substring(0, 36), C_DIM, 13);
    }
    line("ENTER - wlacz Wi-Fi", C_TEXT);
    line("m     - zmien tryb", C_TEXT);
    line("n     - wybierz istniejaca siec", C_TEXT);
    return;
  }

  auto staLines = [&]() {
    switch (staState) {
      case STA_NONE: line(staSsid.isEmpty() ? "n - wybierz" : "-", C_DIM); break;
      case STA_CONNECTING: line(staSsid.substring(0, 21), C_TEXT); line("laczenie...", C_IDLE); break;
      case STA_OK: line(staSsid.substring(0, 21), C_TEXT); line(WiFi.localIP().toString(), C_OK); break;
      case STA_FAIL:
        line(staSsid.substring(0, 21), C_TEXT);
        line(netMode == NET_STA ? "brak, ponawiam..." : "brak polaczenia (r)", C_BAD);
        break;
    }
  };

  String qr;
  if (netMode == NET_AP) {
    line("Tryb: wlasna siec", C_DIM);
    line(apSsid, C_TEXT);
    line("haslo " + apPass, C_TEXT);
    line(AP_IP.toString(), C_ACCENT, 14);
    line("Istniejaca siec", C_DIM);
    staLines();
    if (qrShowsUrl) {
      qr = "http://" + (staState == STA_OK ? WiFi.localIP().toString() : AP_IP.toString()) + "/";
    } else {
      qr = "WIFI:T:WPA;S:" + apSsid + ";P:" + apPass + ";;";
    }
  } else {
    line("Tryb: istniejaca siec", C_DIM, 14);
    staLines();
    y += 4;
    line("strona: adres IP", C_DIM);
    line("lub servo.local", C_DIM);
    if (staState == STA_OK) qr = "http://" + WiFi.localIP().toString() + "/";
  }
  y = max(y + 4, 106);
  line("przegladarki: " + String(wsrv.connectedClients()), wsrv.connectedClients() ? C_OK : C_DIM);

  // QR: dolaczenie do sieci Cardputera albo adres strony
  canvas.fillRoundRect(134, 16, 104, 104, 4, qr.isEmpty() ? C_PANEL : TFT_WHITE);
  canvas.setTextDatum(top_center);
  canvas.setTextColor(C_DIM);
  if (qr.isEmpty()) {
    canvas.drawString("brak IP", 186, 64);
  } else {
    canvas.qrcode(qr.c_str(), 137, 19, 98);
    canvas.drawString(qr.startsWith("WIFI") ? "QR: polacz z siecia" : "QR: strona", 186, 3);
  }
}

// wybor istniejacej sieci: skan / lista / haslo
static void drawNetUi() {
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  if (netUi == NU_SCAN) {
    canvas.setTextColor(C_IDLE);
    canvas.drawString("skanowanie sieci...", 4, 24);
    return;
  }
  if (netUi == NU_LIST) {
    if (scanNets.empty()) {
      canvas.setTextColor(scanFailed ? C_BAD : C_DIM);
      canvas.drawString(scanFailed ? "blad skanowania" : "nie znaleziono sieci", 4, 24);
      canvas.setTextColor(C_DIM);
      canvas.drawString("r - skanuj ponownie", 4, 36);
      canvas.drawString("Cardputer widzi tylko sieci 2.4 GHz", 4, 56);
      canvas.drawString("(5 GHz nie pojawi sie na liscie).", 4, 66);
      return;
    }
    const int top = scanSel < LIST_ROWS ? 0 : scanSel - LIST_ROWS + 1;
    for (int r = 0; r < LIST_ROWS && top + r < (int)scanNets.size(); r++) {
      const int k = top + r, y = 17 + r * 12;
      const ScanNet &e = scanNets[k];
      const bool sel = k == scanSel;
      if (sel) canvas.fillRoundRect(2, y - 1, 236, 11, 2, C_ACCENT);
      canvas.setTextDatum(top_left);
      canvas.setTextColor(sel ? C_BG : (e.ssid == staSsid ? C_OK : C_TEXT));
      canvas.drawString(e.ssid.substring(0, 28), 6, y + 1);
      canvas.setTextDatum(top_right);
      canvas.setTextColor(sel ? C_BG : C_DIM);
      canvas.drawString(String(e.open ? "" : "* ") + e.rssi, 234, y + 1);
    }
    return;
  }
  // NU_PASS
  const ScanNet &e = scanNets[scanSel];
  canvas.setTextColor(C_DIM);
  canvas.drawString("Siec", 4, 20);
  canvas.setTextColor(C_TEXT);
  canvas.drawString(e.ssid.substring(0, 32), 40, 20);
  canvas.setTextColor(C_DIM);
  canvas.drawString("Haslo (wpisz z klawiatury):", 4, 38);
  canvas.fillRoundRect(2, 50, 236, 36, 3, C_PANEL);
  canvas.setTextColor(C_TEXT);
  constexpr int PER_LINE = 38;
  const String shown = passEdit + "_";
  for (int l = 0; l * PER_LINE < (int)shown.length() && l < 3; l++)
    canvas.drawString(shown.substring(l * PER_LINE, (l + 1) * PER_LINE), 6, 53 + l * 11);
  canvas.setTextColor(C_DIM);
  canvas.drawString("Po polaczeniu Cardputer przejdzie", 4, 94);
  canvas.drawString("w tryb \"istniejaca siec\".", 4, 104);
}

// naglowek + tresc + stopka ekranu pomocniczego
static void drawScreen(const char *title, const char *badge, uint16_t badgeCol, void (*body)(), const char *footer) {
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(C_TEXT);
  canvas.drawString(title, 4, 3);
  if (badge) {
    canvas.setTextColor(badgeCol);
    canvas.drawString(badge, 8 + canvas.textWidth(title), 3);
  }
  canvas.drawFastHLine(0, 13, 240, C_TRACK);
  body();
  canvas.fillRect(0, 125, 240, 10, C_PANEL);
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(C_DIM);
  canvas.drawString(footer, 2, 126);
  canvas.pushSprite(0, 0);
  dirty = false;
}

static void draw() {
  canvas.fillSprite(C_BG);

  if (pinScreen) {
    drawScreen("KONFIGURACJA PINOW", nullptr, 0, drawPinScreen, ";. serwo  ,/ pin  ENT zapisz  ESC");
    return;
  }

  if (netUi != NU_NONE) {
    drawScreen("WYBOR SIECI", nullptr, 0, drawNetUi,
               netUi == NU_PASS ? "ENT polacz  DEL usun  ESC wroc"
                                : ";. wybor  ENT dalej  r skanuj  ESC");
    return;
  }

  if (wifiScreen) {
    drawScreen("WI-FI", wifiOn ? "ON" : "OFF", wifiOn ? C_OK : C_BAD, drawWifiScreen,
               wifiOn ? "ENT wyl m tryb n siec q QR r ESC" : "ENT wlacz  m tryb  n siec  ESC");
    return;
  }

  // naglowek
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(C_TEXT);
  canvas.drawString("SERVO CONTROL", 4, 3);
  canvas.setTextDatum(top_right);
  canvas.setTextColor(outputOn ? C_OK : C_BAD);
  canvas.drawString(outputOn ? "PWM ON" : "PWM OFF", 236, 3);
  if (wifiOn) {
    const int n = wsrv.connectedClients();
    canvas.setTextColor(n ? C_OK : C_IDLE);
    canvas.drawString(n ? "WEB " + String(n) : String("WiFi"), 186, 3);
  }
  canvas.drawFastHLine(0, 13, 240, C_TRACK);

  drawDial(0, 60);
  drawDial(1, 180);
  canvas.drawFastVLine(120, 18, 100, C_TRACK);

  // stopka
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  int x = 4;
  char buf[16];
  snprintf(buf, sizeof(buf), "krok %d", STEPS[stepIdx]);
  drawChip(x, 123, buf, C_TRACK, C_TEXT);
  if (linked) drawChip(x, 123, "LINK", C_ACCENT, C_BG);
  if (demo) drawChip(x, 123, "DEMO", C_IDLE, C_BG);
  canvas.setTextDatum(top_right);
  canvas.setTextColor(C_DIM);
  canvas.drawString("h = pomoc", 236, 125);

  if (showHelp) drawHelp();

  canvas.pushSprite(0, 0);
  dirty = false;
}

// ---------- klawiatura ----------
// zwraca true jesli klawisz moze sie powtarzac przy przytrzymaniu
static void stepPin(int dir) {
  int k = pinEdit[pinRow];
  do {
    k = (k + dir + NUM_PIN_OPTS) % NUM_PIN_OPTS;
  } while (k == pinEdit[pinRow ^ 1]);  // ten sam pin nie moze miec dwoch serw
  pinEdit[pinRow] = k;
}

static void openPinMode() {
  pinEdit[0] = pinIdx[0];
  pinEdit[1] = pinIdx[1];
  pinRow = 0;
  pinScreen = true;
  showHelp = false;
}

static void closePinMode(bool save) {
  if (save) {
    applyPins(pinEdit);
    savePins();
  }
  pinScreen = false;
}

static bool handlePinKey(char k) {
  switch (k) {
    case ';': pinRow = 0; break;
    case '.': pinRow = 1; break;
    case '1': pinRow = 0; break;
    case '2': pinRow = 1; break;
    case ',': stepPin(-1); break;
    case '/': stepPin(+1); break;
    case '`':
    case 'p': closePinMode(false); break;
    default: return false;
  }
  dirty = true;
  return false;
}

static bool handleWifiKey(char k) {
  switch (k) {
    case 'q': qrShowsUrl = !qrShowsUrl; break;
    case 'r': if (wifiOn) staConnect(); break;
    case 'm':
      // tryb "istniejaca siec" bez wybranej sieci -> najpierw wybor sieci
      if (netMode == NET_AP && staSsid.isEmpty()) startScan();
      else setNetMode(netMode == NET_AP ? NET_STA : NET_AP);
      break;
    case 'n': startScan(); break;
    case '`':
    case 'w': wifiScreen = false; break;
    default: return false;
  }
  dirty = true;
  return false;
}

// lista sieci (wpisywanie hasla obsluguje handleKeyboard)
static void handleNetListKey(char k) {
  if (netUi != NU_LIST) {
    if (k == '`') closeNetUi();
    return;
  }
  const int n = scanNets.size();
  switch (k) {
    case ';': if (n) scanSel = (scanSel + n - 1) % n; break;
    case '.': if (n) scanSel = (scanSel + 1) % n; break;
    case 'r': startScan(); break;
    case '`': closeNetUi(); break;
    default: return;
  }
  dirty = true;
}

static void handlePassInput(const Keyboard_Class::KeysState &st) {
  if (st.enter) {
    joinNetwork(scanNets[scanSel].ssid, passEdit);
    return;
  }
  if (st.del && passEdit.length()) passEdit.remove(passEdit.length() - 1);
  for (char c : st.word) {
    if (c == '`') {  // ESC - wroc do listy
      netUi = NU_LIST;
      break;
    }
    if (passEdit.length() < 63) passEdit += c;
  }
  dirty = true;
}

static bool handleKey(char k, bool repeat) {
  if (netUi != NU_NONE) {
    const bool nav = k == ';' || k == '.';  // przewijanie listy mozna przytrzymac
    if (!repeat || nav) handleNetListKey(k);
    return nav;
  }
  if (pinScreen) return repeat ? false : handlePinKey(k);
  if (wifiScreen) return repeat ? false : handleWifiKey(k);
  switch (k) {
    case ',': moveActive(-STEPS[stepIdx]); return true;
    case '/': moveActive(+STEPS[stepIdx]); return true;
    default: break;
  }
  if (repeat) return false;
  switch (k) {
    case '1': selected = 0; break;
    case '2': selected = 1; break;
    case ';': stepIdx = min(stepIdx + 1, NUM_STEPS - 1); break;
    case '.': stepIdx = max(stepIdx - 1, 0); break;
    case 'c': setAngle(0, 90); setAngle(1, 90); break;
    case '0': setActive(0); break;
    case '9': setActive(180); break;
    case 'l': linked = !linked; break;
    case 'd': demo = !demo; break;
    case 't': setServoType(selected, !cont[selected]); break;
    case '[': if (cont[selected]) setTrim(selected, trimUs[selected] - TRIM_STEP_US); break;
    case ']': if (cont[selected]) setTrim(selected, trimUs[selected] + TRIM_STEP_US); break;
    case 'h': showHelp = !showHelp; break;
    case 'p': openPinMode(); break;
    case 'w':
      wifiScreen = true;
      showHelp = false;
      break;
    case 'x':
      outputOn = !outputOn;
      servoWrite(0);
      servoWrite(1);
      break;
    default: return false;
  }
  dirty = true;
  return false;
}

static void handleKeyboard() {
  static uint32_t pressedAt = 0, lastRepeat = 0;
  static char heldKey = 0;

  auto &kb = M5Cardputer.Keyboard;
  const uint32_t now = millis();

  if (kb.isChange()) {
    heldKey = 0;
    if (kb.isPressed()) {
      auto st = kb.keysState();
      if (netUi == NU_PASS) {
        handlePassInput(st);
        return;
      }
      if (netUi == NU_LIST && st.enter) {
        pickNetwork();
        return;
      }
      if (pinScreen && st.enter) {
        closePinMode(true);
        dirty = true;
      } else if (wifiScreen && st.enter) {
        setWifi(!wifiOn);
        if (wifiOn && netMode == NET_STA && staSsid.isEmpty()) startScan();
      } else if (st.tab && !wifiScreen) {
        if (pinScreen) pinRow ^= 1;
        else selected ^= 1;
        dirty = true;
      }
      for (char c : st.word) {
        if (handleKey(c, false)) heldKey = c;
      }
      pressedAt = lastRepeat = now;
    }
    return;
  }

  if (heldKey && kb.isPressed() && now - pressedAt > REPEAT_DELAY_MS && now - lastRepeat > REPEAT_RATE_MS) {
    handleKey(heldKey, true);
    lastRepeat = now;
  }
}

// ---------- demo ----------
static void runDemo() {
  static uint32_t last = 0;
  const uint32_t now = millis();
  if (now - last < 15) return;
  last = now;

  int next = target[0] + demoDir * 2;
  if (next >= 180 || next <= 0) demoDir = -demoDir;
  setAngle(0, next);
  setAngle(1, 180 - target[0]);  // drugie serwo w lustrzanym odbiciu
}

// ---------- main ----------
void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  // nowsze M5GFX same ustawiaja ekran poziomo (240x135); obracamy tylko gdy jest pionowo
  if (M5Cardputer.Display.width() < M5Cardputer.Display.height()) M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.setBrightness(100);

  canvas.setColorDepth(16);
  canvas.createSprite(M5Cardputer.Display.width(), M5Cardputer.Display.height());
  initColors();

  loadPins();
  servoSetup();
  loadNet();
  if (wifiOn) wifiStart();
  draw();
}

void loop() {
  static uint32_t lastFrame = millis();

  M5Cardputer.update();
  handleKeyboard();
  netLoop();
  if (demo) runDemo();

  const uint32_t now = millis();
  if (now - lastFrame >= FRAME_MS) {
    updateSim((now - lastFrame) / 1000.0f);
    lastFrame = now;
    static uint32_t lastWifiDraw = 0;
    if (wifiScreen && now - lastWifiDraw > 1000) {
      lastWifiDraw = now;
      dirty = true;
    }
    if (dirty) draw();
  }
  delay(2);
}
