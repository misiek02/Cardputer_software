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
//   p           konfiguracja pinow
//   w           Wi-Fi: sterowanie z telefonu / komputera przez przegladarke
//   h           pomoc
//
// Sterowanie przez siec: Cardputer tworzy wlasna siec "ServoCtl-XXXX" (haslo na
// ekranie Wi-Fi, jest tez kod QR) i serwuje strone http://192.168.4.1/.
// Opcjonalnie laczy sie tez z siecia domowa (SSID/haslo ustawiasz na stronie).
// Strona: src/web_ui.h, komendy WebSocket: handleNetCmd().

#include <M5Cardputer.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <math.h>
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

// ekran konfiguracji pinow
static bool pinScreen = false;
static int pinEdit[2];
static int pinRow = 0;

static Preferences prefs;
static bool dirty = true;

// siec
enum StaState { STA_NONE, STA_CONNECTING, STA_OK, STA_FAIL };
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
static int angleToUs(int deg) { return PULSE_MIN_US + (PULSE_MAX_US - PULSE_MIN_US) * deg / 180; }

static uint32_t angleToDuty(int deg) {
  return (uint32_t)angleToUs(deg) * ((1u << PWM_RES) - 1) / (1000000u / PWM_FREQ_HZ);
}

static void servoWrite(int i) {
  const uint32_t duty = outputOn ? angleToDuty(target[i]) : 0;
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
    c.duty = angleToDuty(target[i]);
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
    c.duty = outputOn ? angleToDuty(target[i]) : 0;
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
  snprintf(buf, sizeof(buf), "{\"t\":[%d,%d],\"o\":%d,\"l\":%d,\"d\":%d,\"s\":%d,\"k\":%d,\"g\":[%d,%d]}",
           target[0], target[1], outputOn, linked, demo, selected, stepIdx, PIN_OPTS[pinIdx[0]].gpio,
           PIN_OPTS[pinIdx[1]].gpio);
  return buf;
}

// komendy z przegladarki: "a i deg", "b deg", "s i", "k n", "l 0|1", "d 0|1", "x 0|1", "p g1 g2"
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
             ",\"ip\":\"" + WiFi.localIP().toString() + "\",\"pins\":[";
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
    prefs.begin("servoctl", false);
    prefs.putString("sta_ssid", staSsid);
    prefs.putString("sta_pass", staPass);
    prefs.end();
    http.send(200, "text/plain", "ok");
    staConnect();
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
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);    // mniejsze opoznienia sterowania
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(apSsid.c_str(), apPass.c_str());
  dns.start(53, "*", AP_IP);
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

static void netLoop() {
  if (!wifiOn) return;
  dns.processNextRequest();
  http.handleClient();
  wsrv.loop();

  const uint32_t now = millis();
  const bool up = WiFi.status() == WL_CONNECTED;
  if (staState == STA_CONNECTING && up) {
    staState = STA_OK;
    dirty = true;
  } else if (staState == STA_CONNECTING && now - staStart > STA_TIMEOUT_MS) {
    staState = STA_FAIL;  // przestajemy probowac - ciagle skanowanie zakloca siec Cardputera
    WiFi.disconnect(false);
    dirty = true;
  } else if (staState == STA_OK && !up) {
    staState = STA_CONNECTING;
    staStart = now;
    dirty = true;
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
  canvas.drawString(String(outputOn ? angleToUs(target[i]) : 0) + "us", cx + 56, 18);

  // tor i wypelnienie lukiem (fillArc: 180 = lewo, 270 = gora, 360 = prawo)
  canvas.fillArc(cx, cy, R - 5, R, 180, 360, C_TRACK);
  if (simPos[i] > 0.5f) canvas.fillArc(cx, cy, R - 5, R, 180, 180 + simPos[i], acc);

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
  canvas.drawString("SG90", cx, cy + 10);

  // orczyk (ramie)
  float ex, ey, bx, by;
  polar(cx, cy, R - 11, simPos[i], ex, ey);
  polar(cx, cy, 9, simPos[i] + 180, bx, by);  // krotszy koniec z drugiej strony
  const uint16_t arm = outputOn ? C_TEXT : C_OFF;
  canvas.drawWideLine(bx, by, ex, ey, 3.0f, arm);
  canvas.fillCircle(ex, ey, 3, arm);
  canvas.fillCircle(cx, cy, 6, arm);
  canvas.fillCircle(cx, cy, 2, C_BODY);

  // kat
  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.setTextDatum(top_center);
  canvas.setTextColor(outputOn ? C_TEXT : C_OFF);
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
      {"0 9", "0 / 180 stopni"},      {"l", "link: oba razem"},
      {"d", "demo (sweep)"},          {"x", "PWM on/off"},
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

  if (!wifiOn) {
    line("Wi-Fi wylaczone.", C_TEXT, 16);
    line("Po wlaczeniu Cardputer tworzy", C_DIM);
    line("wlasna siec i strone WWW.", C_DIM);
    line("Serwami sterujesz wtedy z", C_DIM);
    line("telefonu, tabletu lub komputera", C_DIM);
    line("w przegladarce (dotyk, mysz,", C_DIM);
    line("klawiatura).", C_DIM, 16);
    line("ENTER - wlacz Wi-Fi", C_ACCENT);
    return;
  }

  line("Siec Cardputera", C_DIM);
  line(apSsid, C_TEXT);
  line("haslo " + apPass, C_TEXT);
  line(AP_IP.toString(), C_ACCENT, 14);
  line("Siec domowa", C_DIM);
  switch (staState) {
    case STA_NONE: line("ustaw na stronie", C_DIM); line("(Ustawienia)", C_DIM); break;
    case STA_CONNECTING: line(staSsid.substring(0, 21), C_TEXT); line("laczenie...", C_IDLE); break;
    case STA_OK: line(staSsid.substring(0, 21), C_TEXT); line(WiFi.localIP().toString(), C_OK); break;
    case STA_FAIL: line(staSsid.substring(0, 21), C_TEXT); line("brak polaczenia (r)", C_BAD); break;
  }
  y += 4;
  line("przegladarki: " + String(wsrv.connectedClients()), wsrv.connectedClients() ? C_OK : C_DIM);

  // QR: dolaczenie do sieci albo adres strony
  String qr;
  if (qrShowsUrl) {
    qr = "http://" + (staState == STA_OK ? WiFi.localIP().toString() : AP_IP.toString()) + "/";
  } else {
    qr = "WIFI:T:WPA;S:" + apSsid + ";P:" + apPass + ";;";
  }
  canvas.fillRoundRect(134, 16, 104, 104, 4, TFT_WHITE);
  canvas.qrcode(qr.c_str(), 137, 19, 98);
  canvas.setTextDatum(top_center);
  canvas.setTextColor(C_DIM);
  canvas.drawString(qrShowsUrl ? "QR: strona" : "QR: polacz z siecia", 186, 3);
}

static void draw() {
  canvas.fillSprite(C_BG);

  if (pinScreen) {
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(top_left);
    canvas.setTextColor(C_TEXT);
    canvas.drawString("KONFIGURACJA PINOW", 4, 3);
    canvas.drawFastHLine(0, 13, 240, C_TRACK);
    drawPinScreen();
    canvas.fillRect(0, 125, 240, 10, C_PANEL);
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(top_left);
    canvas.setTextColor(C_DIM);
    canvas.drawString(";. serwo  ,/ pin  ENT zapisz  ESC", 2, 126);
    canvas.pushSprite(0, 0);
    dirty = false;
    return;
  }

  if (wifiScreen) {
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(top_left);
    canvas.setTextColor(C_TEXT);
    canvas.drawString("WI-FI", 4, 3);
    canvas.setTextColor(wifiOn ? C_OK : C_BAD);
    canvas.drawString(wifiOn ? "ON" : "OFF", 38, 3);
    canvas.drawFastHLine(0, 13, 240, C_TRACK);
    drawWifiScreen();
    canvas.fillRect(0, 125, 240, 10, C_PANEL);
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(top_left);
    canvas.setTextColor(C_DIM);
    canvas.drawString(wifiOn ? "ENT wylacz  q QR  r ponow  ESC" : "ENT wlacz  ESC/w wroc", 2, 126);
    canvas.pushSprite(0, 0);
    dirty = false;
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
    case '`':
    case 'w': wifiScreen = false; break;
    default: return false;
  }
  dirty = true;
  return false;
}

static bool handleKey(char k, bool repeat) {
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
      if (pinScreen && st.enter) {
        closePinMode(true);
        dirty = true;
      } else if (wifiScreen && st.enter) {
        setWifi(!wifiOn);
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
