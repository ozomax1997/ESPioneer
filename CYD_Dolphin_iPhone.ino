/*
  ESPioneer - standard CYD test sketch, plays .dlp clips from the microSD card (tiny flash use).
  Copy the "clips" folder (inside "sdcard") to the root of a FAT16/FAT32 microSD card, insert it, power on.
  Arduino IDE setup:
    Board:   "ESP32 Dev Module"   (Partition: Default 4MB with spiffs)
    Library: "GFX Library for Arduino" by moononournation (Library Manager)
  LIVE iPhone "now playing" over Bluetooth LE (Apple Media Service): track, artist, album, volume and
  play state of whatever the iPhone is playing (Spotify, Apple Music, podcasts...). Rear RGB LED follows
  the display colour. Audio is NOT affected - the iPhone keeps playing to its own speaker/headphones.
  FIRST TIME: iPhone Settings > Bluetooth > tap "ESPioneer" under Other Devices > Pair. A pairing code
  is shown on the display; it should match the one on the iPhone.
  If pairing gets stuck: "Forget This Device" on the iPhone, then HOLD the BOOT button for 3+ seconds
  (clears the ESP32's saved pairings and restarts), or send x over serial.
  Controls: tap screen left/middle/right = prev/pause/next. BOOT button: click = next clip,
            HOLD (0.6 s) = next colour theme (display + rear LED).
  Serial:   t / T = next / previous theme, 1-9 and 0 = jump to theme 1-10, l = LED brightness,
            x = forget paired phones, n / p = next / prev clip.   Theme and LED level are remembered.
  If colours look inverted set CYD_INVERT 1. If the screen is blank try CYD_ST7789 1.
*/
#include <Arduino_GFX_Library.h>
#include <SD.h>
#include <SPI.h>
#include <vector>
#include <algorithm>
#include <Preferences.h>
#include "ams.h"
#include "rows.h"

#define CYD_ST7789 0   // 1 for the 2-USB-port CYD variant with ST7789 panel
#define CYD_INVERT 0
#define SHUFFLE    0   // 1 = random order when auto-advancing
constexpr int MAX_CLIPS = 1500;

// ---- what is shown (filled from the iPhone; placeholders until it connects) ----
static int volumePct = 0;
static uint32_t pairCode = 0;
static AMS::Info lastInfo;
static bool havePlaying = false;

// ---- rear RGB LED (CYD: R=4 G=16 B=17, active LOW) ----
constexpr int LED_PIN[3] = {4, 16, 17};

constexpr int W = 256, H = 64, FRAME_BYTES = W * H / 4;
constexpr uint32_t MIN_CLIP_MS = 8000;   // play each clip at least this long, then advance
static int FRAMES = 60, DELAY_MS = 60;    // per clip, read from the .dlp header
static uint32_t clipStart = 0;
constexpr int OX = 32, OY = 80;                     // frame position on 320x240 landscape
constexpr int PIN_BL = 21, PIN_BTN = 0;
constexpr int T_CS = 33, T_IRQ = 36, T_CLK = 25, T_MOSI = 32, T_MISO = 39;

Arduino_DataBus *bus = new Arduino_ESP32SPI(2 /*DC*/, 15 /*CS*/, 14 /*SCK*/, 13 /*MOSI*/, 12 /*MISO*/, HSPI);
#if CYD_ST7789
Arduino_GFX *gfx = new Arduino_ST7789(bus, GFX_NOT_DEFINED, 1, false, 240, 320);
#else
Arduino_GFX *gfx = new Arduino_ILI9341(bus, GFX_NOT_DEFINED, 1 /*landscape*/);
#endif

static SPIClass sdSPI(VSPI);
static std::vector<String> names;
static File cf;
static uint8_t raw[FRAME_BYTES];
static int NUM_CLIPS = 0;
static String dir = "/";                   // "/clips/" if that folder exists, else the card root
static uint16_t buf[W * H];
static uint16_t pal[4];

// ---- colour themes: one base colour each; the display's 3 lit levels and the rear LED derive from it ----
struct Theme { const char *name; uint8_t r, g, b; };
static const Theme THEMES[] = {
  {"PIONEER BLUE",  0, 255, 255},   // exact OEL palette from the original clips
  {"PIONEER GREEN", 120, 255, 50},  // exact old head-unit green palette (LED pure green)
  {"AMBER",   255, 176,   0},  {"ORANGE",  255, 100,   0},  {"RED",      255,  30,  30},
  {"HOT PINK",255,  40, 140},  {"MAGENTA", 255,   0, 255},  {"PURPLE",  160,  60, 255},
  {"INDIGO",   90,  70, 255},  {"BLUE",     30,  90, 255},  {"ICE",     140, 200, 255},
  {"TEAL",      0, 255, 170},  {"LIME",    160, 255,   0},  {"YELLOW",  255, 230,   0},
  {"WARM WHITE",255,200, 140}, {"WHITE",   255, 255, 255},
};
constexpr int NUM_THEMES = sizeof(THEMES) / sizeof(THEMES[0]);
constexpr int LED_STEPS[] = {8, 20, 40, 90, 180};      // rear LED brightness levels (0..255)
static int ledStep = 2;
static Preferences prefs;
static uint16_t dimCol;                                // faint colour for unlit volume segments
static int theme = 0;
static int clip = 0, frameNo = 0, loops = 0;
static bool paused = false;
static uint32_t nextAt = 0;

// --- minimal XPT2046 read (bit-banged, no extra library) ---
static uint16_t xptRead(uint8_t cmd) {
  uint16_t v = 0;
  for (int i = 7; i >= 0; i--) { digitalWrite(T_MOSI, (cmd >> i) & 1); digitalWrite(T_CLK, HIGH); digitalWrite(T_CLK, LOW); }
  for (int i = 0; i < 16; i++) { digitalWrite(T_CLK, HIGH); digitalWrite(T_CLK, LOW); v = (v << 1) | digitalRead(T_MISO); }
  return v >> 4;
}
static int touchX() {                                // returns 0..319 or -1
  if (digitalRead(T_IRQ)) return -1;
  digitalWrite(T_CS, LOW);
  xptRead(0xD0);                                     // discard
  uint16_t x = xptRead(0xD0);
  digitalWrite(T_CS, HIGH);
  // CYD landscape: raw X (0xD0 = Y axis chip-side) maps along the long axis; calibrate roughly
  return constrain(map(x, 200, 3800, 0, 320), 0, 319);
}

// ---------- rear LED (PWM, active low; works on ESP32 Arduino core 2.x and 3.x) ----------
static void ledInit() {
  for (int i = 0; i < 3; i++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(LED_PIN[i], 5000, 8);
#else
    ledcSetup(i, 5000, 8); ledcAttachPin(LED_PIN[i], i);
#endif
  }
}
static void ledWrite(int i, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(LED_PIN[i], duty);
#else
  ledcWrite(i, duty);
#endif
}
static void ledSet(uint8_t r, uint8_t g, uint8_t b) {
  const uint8_t c[3] = {r, g, b};
  int lvl = LED_STEPS[ledStep];
  for (int i = 0; i < 3; i++) ledWrite(i, 255 - (c[i] * lvl) / 255);   // active low: 255 = off
}

// ---------- info bands (top 0..76, bottom 148..240) ----------
constexpr int MX = 32;                       // text margin: lines up with the animation's left edge
static void text(int x, int y, int size, uint16_t col, const char *t) {
  gfx->setTextSize(size); gfx->setTextColor(col); gfx->setCursor(x, y); gfx->print(t);
}
static void fit(char *dst, const char *src, int size) {   // truncate with ".." to fit between margins
  int maxc = (320 - 2 * MX) / (6 * size);
  char tmp[48]; strncpy(tmp, src, sizeof tmp - 1); tmp[sizeof tmp - 1] = 0;   // safe if dst == src
  if ((int)strlen(tmp) > maxc) { tmp[maxc - 2] = 0; strcat(tmp, ".."); }
  strcpy(dst, tmp);
}
static void drawVolume() {
  gfx->fillRect(0, 196, 320, 24, 0x0000);
  text(MX, 201, 2, pal[2], "VOL");
  constexpr int SEG = 16, X0 = 84, SW = 9, GAP = 2;
  int lit = (volumePct * SEG + 50) / 100;
  for (int i = 0; i < SEG; i++)
    gfx->fillRect(X0 + i * (SW + GAP), 198, SW, 18, i < lit ? pal[3] : dimCol);
  char v[8]; snprintf(v, sizeof v, "%d", volumePct);
  text(X0 + SEG * (SW + GAP) + 8, 201, 2, pal[3], v);
}
// Scrolling text row: long strings scroll one character at a time after a short pause.
static uint16_t colBright, colMid;
static Row rowTrack = {24, 2, &colBright}, rowArtist = {48, 2, &colMid}, rowAlbum = {170, 2, &colMid};
static void rowDraw(Row &r) {
  const int maxc = (320 - 2 * MX) / (6 * r.size);
  char b[40]; int n = strlen(r.text);
  if (n <= maxc) { strncpy(b, r.text, sizeof b - 1); b[sizeof b - 1] = 0; }
  else { strncpy(b, r.text + r.off, maxc); b[maxc] = 0; }
  gfx->fillRect(0, r.y, 320, 8 * r.size, 0x0000);
  text(MX, r.y, r.size, *r.col, b);
}
static void rowSet(Row &r, const char *t, bool draw = true) {
  if (strcmp(r.text, t) == 0) return;
  strncpy(r.text, t, sizeof r.text - 1); r.text[sizeof r.text - 1] = 0;
  r.off = 0; r.next = millis() + 1800;
  if (draw) rowDraw(r);
}
static void rowTick(Row &r) {
  const int maxc = (320 - 2 * MX) / (6 * r.size), n = strlen(r.text);
  if (n <= maxc || millis() < r.next) return;
  if (r.off + maxc >= n) { r.off = 0; r.next = millis() + 1800; }          // wrapped: pause, restart
  else { r.off++; r.next = millis() + (r.off + maxc >= n ? 1500 : 220); }  // pause at the end too
  rowDraw(r);
}
static void drawInfo() {
  colBright = pal[3]; colMid = pal[2];
  gfx->fillRect(0, 0, 320, 76, 0x0000);
  text(MX, 8, 1, pal[1], "NOW PLAYING");
  text(320 - MX - 9 * 6, 8, 1, pal[1], "BLUETOOTH");
  if (pairCode) {
    char b[40]; snprintf(b, sizeof b, "PAIR CODE %06lu", (unsigned long)pairCode);
    text(MX, 32, 2, pal[3], b);
    text(MX, 56, 1, pal[1], "Check it matches the iPhone, then tap Pair");
  } else {
    rowDraw(rowTrack); rowDraw(rowArtist);
  }
  gfx->fillRect(0, 148, 320, 48, 0x0000);
  text(MX, 156, 1, pal[1], "ALBUM");
  rowDraw(rowAlbum);
  drawVolume();
}
static void drawStatus() {                    // tiny footer: clip name + play state + theme
  gfx->fillRect(0, 224, 320, 16, 0x0000);
  const char *bt = lastInfo.link == 2 ? (lastInfo.playing ? "iPHONE PLAYING" : "iPHONE PAUSED") : lastInfo.link == 1 ? "PAIRING.." : (lastInfo.adv ? "ADVERTISING" : "BT STARTING");
  char b[72]; snprintf(b, sizeof b, "%s %d/%d  %s  %s", names.empty() ? "" : names[clip].c_str(), clip + 1, NUM_CLIPS, bt, THEMES[theme].name);
  text(MX, 228, 1, pal[1], b);
}

static void saveSettings() {
  prefs.putUChar("theme", theme); prefs.putUChar("led", ledStep); 
}
static void setTheme(int t) {
  theme = (t % NUM_THEMES + NUM_THEMES) % NUM_THEMES;
  const Theme &T = THEMES[theme];
  uint8_t lv[4][3];
  if (theme == 0) {                                   // original OEL cyan levels
    const uint8_t o[4][3] = {{0,0,0}, {7,158,175}, {5,199,220}, {0,255,255}}; memcpy(lv, o, sizeof lv);
  } else if (theme == 1) {                            // original old-Pioneer green levels
    const uint8_t o[4][3] = {{0,0,0}, {35,125,12}, {70,195,25}, {120,255,50}}; memcpy(lv, o, sizeof lv);
  } else {
    const int f[4] = {0, 60, 80, 100};                // dark / mid / bright / full, in percent
    for (int i = 0; i < 4; i++) { lv[i][0] = T.r * f[i] / 100; lv[i][1] = T.g * f[i] / 100; lv[i][2] = T.b * f[i] / 100; }
  }
  for (int i = 0; i < 4; i++) pal[i] = gfx->color565(lv[i][0], lv[i][1], lv[i][2]);
  dimCol = gfx->color565(lv[1][0] / 5, lv[1][1] / 5, lv[1][2] / 5);
  if (theme == 1) ledSet(0, 255, 0); else ledSet(T.r, T.g, T.b);
  gfx->drawRect(OX - 2, OY - 2, W + 4, H + 4, gfx->color565(40, 40, 40));
  drawInfo(); drawStatus();
}

static void setClip(int c) {
  clip = (c % NUM_CLIPS + NUM_CLIPS) % NUM_CLIPS; frameNo = 0; loops = 0;
  if (cf) cf.close();
  cf = SD.open(dir + names[clip] + ".dlp", FILE_READ);
  uint8_t h[16];
  if (cf && cf.read(h, 16) == 16 && !memcmp(h, "DLP1", 4)) {
    FRAMES = h[8] | (h[9] << 8);
    DELAY_MS = h[10] | (h[11] << 8);
    if (FRAMES < 1) FRAMES = 1;
  } else if (cf) { cf.close(); }
  clipStart = millis();
  drawStatus();
}

static void fatal(const char *m) {
  gfx->setTextColor(0xF800); gfx->setTextSize(2); gfx->setCursor(10, 100); gfx->print(m);
  while (true) delay(1000);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BL, OUTPUT); digitalWrite(PIN_BL, HIGH);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(T_CS, OUTPUT); digitalWrite(T_CS, HIGH);
  pinMode(T_CLK, OUTPUT); pinMode(T_MOSI, OUTPUT); pinMode(T_MISO, INPUT); pinMode(T_IRQ, INPUT);
  gfx->begin(40000000);
#if CYD_INVERT
  gfx->invertDisplay(true);
#endif
  gfx->fillScreen(0x0000);
  ledInit();
  prefs.begin("espioneer", false);
  ledStep = min((int)prefs.getUChar("led", 2), (int)(sizeof LED_STEPS / sizeof LED_STEPS[0]) - 1);
  setTheme(prefs.getUChar("theme", 0));
  rowSet(rowTrack, "Waiting for iPhone"); rowSet(rowArtist, "Bluetooth > ESPioneer"); rowSet(rowAlbum, "-");
  AMS::begin("ESPioneer");                       // Bluetooth first, so an SD problem cannot stop it advertising
  sdSPI.begin(18, 19, 23, 5);
  if (!SD.begin(5, sdSPI, 20000000)) fatal("No SD card found");
  dir = SD.exists("/clips") ? "/clips/" : "/";
  File root = SD.open(dir.length() > 1 ? "/clips" : "/");
  for (File e = root.openNextFile(); e; e = root.openNextFile()) {
    String n = e.name(); e.close();
    if (n.startsWith("._") || n.startsWith(".") || !n.endsWith(".dlp")) continue;   // skip macOS junk
    if ((int)names.size() >= MAX_CLIPS) break;
    n = n.substring(n.lastIndexOf('/') + 1); n.remove(n.length() - 4);
    names.push_back(n);
  }
  std::sort(names.begin(), names.end());
  NUM_CLIPS = names.size();
  if (!NUM_CLIPS) fatal("No .dlp files on SD");
  randomSeed(esp_random());
  setClip(SHUFFLE ? (int)random(NUM_CLIPS) : 0);
  drawInfo();
}

void loop() {
  static uint32_t lastIn = 0, downAt = 0;
  // BOOT button: click = next clip, hold = theme
  bool down = digitalRead(PIN_BTN) == LOW;
  if (down && !downAt) downAt = millis();
  if (down && downAt && millis() - downAt > 3000) { AMS::clearBonds(); gfx->fillScreen(0x0000); text(MX, 100, 2, pal[3], "Pairings cleared"); delay(800); ESP.restart(); }
  if (!down && downAt) { uint32_t d = millis() - downAt; downAt = 0; if (d > 600) { setTheme(theme + 1); saveSettings(); } else if (d > 30) setClip(clip + 1); }
  // touch: left / middle / right
  if (millis() - lastIn > 350) {
    int tx = touchX();
    if (tx >= 0) { lastIn = millis(); if (tx < 107) setClip(clip - 1); else if (tx > 213) setClip(clip + 1); else { paused = !paused; drawStatus(); } }
  }
  // serial
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 't': setTheme(theme + 1); saveSettings(); break;
      case 'T': setTheme(theme - 1); saveSettings(); break;
      case 'l': ledStep = (ledStep + 1) % (sizeof LED_STEPS / sizeof LED_STEPS[0]); setTheme(theme); saveSettings(); break;
      case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9':
        setTheme(c - '1'); saveSettings(); break;
      case '0': setTheme(9); saveSettings(); break;
      case 'x': AMS::clearBonds(); Serial.println("bonds cleared, restarting"); delay(200); ESP.restart(); break;
      case 'n': setClip(clip + 1); break;
      case 'p': setClip(clip - 1); break;
    }
  }
  // iPhone data (poll ~20x/s): redraw only what changed
  static uint32_t lastPoll = 0;
  if (millis() - lastPoll > 50) {
    lastPoll = millis();
    AMS::Info cur; AMS::snapshot(cur);
    if (cur.version != lastInfo.version) {
      bool full = cur.passkey != pairCode;
      pairCode = cur.passkey;
      const bool top = pairCode == 0;                 // top rows are hidden while a pairing code is shown
      if (cur.link == 2) {
        rowSet(rowTrack, cur.title[0] ? cur.title : "Nothing playing", top);
        rowSet(rowArtist, cur.artist, top);
        rowSet(rowAlbum, cur.album[0] ? cur.album : "-");
        int v = cur.volume < 0 ? 0 : (int)(cur.volume * 100 + 0.5f);
        if (v != volumePct) { volumePct = v; drawVolume(); }
      } else {
        rowSet(rowTrack, cur.link == 1 ? "Pairing with iPhone" : "Waiting for iPhone", top);
        rowSet(rowArtist, cur.link == 1 ? "Tap Pair on the iPhone" : "Bluetooth > ESPioneer", top);
        rowSet(rowAlbum, "-");
        if (volumePct) { volumePct = 0; drawVolume(); }
      }
      bool statusChanged = cur.link != lastInfo.link || cur.playing != lastInfo.playing || cur.adv != lastInfo.adv;
      lastInfo = cur;
      if (full) drawInfo();
      if (statusChanged) drawStatus();
    }
  }
  if (!pairCode) { rowTick(rowTrack); rowTick(rowArtist); }
  rowTick(rowAlbum);

  if (paused || (int32_t)(millis() - nextAt) < 0) return;
  nextAt = millis() + DELAY_MS;
  if (!cf) { setClip(clip + 1); return; }
  cf.seek(16 + (size_t)frameNo * FRAME_BYTES);
  if (cf.read(raw, FRAME_BYTES) != FRAME_BYTES) { setClip(clip + 1); return; }
  const uint8_t *f = raw;
  uint16_t *o = buf;
  for (int i = 0; i < FRAME_BYTES; i++) {
    uint8_t b = f[i];
    *o++ = pal[b >> 6]; *o++ = pal[(b >> 4) & 3]; *o++ = pal[(b >> 2) & 3]; *o++ = pal[b & 3];
  }
  gfx->draw16bitRGBBitmap(OX, OY, buf, W, H);
  if (++frameNo >= FRAMES) { frameNo = 0; if (millis() - clipStart >= MIN_CLIP_MS) setClip(SHUFFLE ? (int)random(NUM_CLIPS) : clip + 1); }
}
