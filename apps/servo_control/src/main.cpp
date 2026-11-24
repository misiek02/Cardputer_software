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
//   h           pomoc

#include <M5Cardputer.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <math.h>

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
      {"p", "konfiguracja pinow"},    {"h", "zamknij pomoc"},
  };
  int y = 13;
  for (auto &l : lines) {
    canvas.setTextColor(C_ACCENT);
    canvas.drawString(l[0], 16, y);
    canvas.setTextColor(C_TEXT);
    canvas.drawString(l[1], 78, y);
    y += 11;
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

  // naglowek
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(C_TEXT);
  canvas.drawString("SERVO CONTROL", 4, 3);
  canvas.setTextDatum(top_right);
  canvas.setTextColor(outputOn ? C_OK : C_BAD);
  canvas.drawString(outputOn ? "PWM ON" : "PWM OFF", 236, 3);
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

static bool handleKey(char k, bool repeat) {
  if (pinScreen) return repeat ? false : handlePinKey(k);
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
      } else if (st.tab) {
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
  draw();
}

void loop() {
  static uint32_t lastFrame = millis();

  M5Cardputer.update();
  handleKeyboard();
  if (demo) runDemo();

  const uint32_t now = millis();
  if (now - lastFrame >= FRAME_MS) {
    updateSim((now - lastFrame) / 1000.0f);
    lastFrame = now;
    if (dirty) draw();
  }
  delay(2);
}
