// ams.h - Apple Media Service (AMS) client for ESP32, header-only.
//
// The iPhone connects to the ESP32 (which advertises "service solicitation" for AMS); the ESP32 then
// reads the phone's media service over that same link: title, artist, album, play state, volume.
// Pairing is a one-off: iPhone Settings > Bluetooth > ESPioneer > Pair.
//
// Design: all Bluetooth work runs in its own FreeRTOS task. The rest of the sketch only calls
// AMS::begin() once and AMS::snapshot() to read the latest data (copied under a lock).
//
// Ported from the working ESP32 Arduino BLE project err4o4/ctrl01 (MIT-style, itself based on
// James Hudson's ANCS client and the Adafruit/Moddable AMS examples), with these changes:
//   - non-blocking: runs in a task, never stalls the display
//   - long titles: iOS truncates notifications to the ATT MTU, so truncated fields are re-read in
//     full through the Entity Attribute characteristic
//   - UTF-8 -> plain ASCII (the built-in display font has no accents or curly quotes)
//   - pairing code is exposed so the display can show it
#pragma once
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLESecurity.h>
#include <esp_gap_ble_api.h>

namespace AMS {

#define AMSLOG(...) Serial.printf(__VA_ARGS__)   // progress messages for the Serial Monitor (115200)

struct Info {
  char title[64], artist[64], album[64];
  uint8_t playing;        // 1 = playing, 0 = paused/stopped
  float volume;           // 0..1, or -1 if the phone has not reported it yet
  uint8_t adv;            // 1 once the radio has really started advertising
  uint8_t link;           // 0 = advertising, 1 = connecting / pairing, 2 = receiving media info
  uint32_t passkey;       // non-zero while the iPhone is showing a pairing code to compare
  uint32_t version;       // increases whenever anything changes
};

// ---- Apple Media Service UUIDs and constants (Apple AMS specification) ----
#define AMS_SERVICE_UUID       "89D3502B-0F36-433A-8EF4-C502AD55F8DC"
#define AMS_REMOTE_CMD_UUID    "9B3C81D8-57B1-4A8A-B8DF-0E56F7CA51C2"
#define AMS_ENTITY_UPDATE_UUID "2F7CABCE-808D-411F-9A0C-BB92BA96C102"
#define AMS_ENTITY_ATTR_UUID   "C6B2F38C-23AB-46D8-A6AB-A3A870BBD5D7"
enum { ENT_PLAYER = 0, ENT_QUEUE = 1, ENT_TRACK = 2 };
enum { CMD_PLAY = 0, CMD_PAUSE = 1, CMD_TOGGLE = 2, CMD_NEXT = 3, CMD_PREV = 4, CMD_VOL_UP = 5, CMD_VOL_DOWN = 6 };

// ---- shared state ----
static portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
static Info gInfo;                                  // protected by gMux
static volatile bool gServerConnected = false;      // iPhone connected to us
static volatile bool gAuthDone = false;
static volatile uint32_t gPendingFull = 0;          // bit0 title, bit1 artist, bit2 album: need full re-read
static BLEServer *gServer = nullptr;
static BLEAddress *gPeer = nullptr;
static BLEClient *gClient = nullptr;
static BLERemoteCharacteristic *gChAttr = nullptr, *gChCmd = nullptr;
static uint8_t gPeerType = 0;                       // iPhone address type (public/random)
static bool gRunning = false;

// ---- security wrappers: the BLE API changed between ESP32 Arduino core 2.x and 3.x ----
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void secMode()  { BLESecurity::setAuthenticationMode(true, false, true); }  // bond + secure connections, no MITM; also enables security
static void secCap(uint8_t v)      { BLESecurity::setCapability(v); }
static void secInitKeys()          { BLESecurity::setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK); }
static void secRespKeys()          { BLESecurity::setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK); }
static void secReset()             { BLESecurity::resetSecurity(); }               // needed so a reconnect re-runs pairing/encryption
#else
static BLESecurity gSecurity;
static void secMode()  { gSecurity.setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND); BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT); }
static void secCap(uint8_t v)      { gSecurity.setCapability(v); }
static void secInitKeys()          { gSecurity.setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK); }
static void secRespKeys()          { gSecurity.setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK); }
static void secReset()             {}
#endif

// ---- helpers ----
static void setInfo(uint8_t link, bool clearText) {
  portENTER_CRITICAL(&gMux);
  gInfo.link = link;
  if (clearText) { gInfo.title[0] = gInfo.artist[0] = gInfo.album[0] = 0; gInfo.volume = -1; gInfo.playing = 0; }
  gInfo.version++;
  portEXIT_CRITICAL(&gMux);
}

// UTF-8 -> printable ASCII for the built-in 6x8 font.
static void sanitize(const uint8_t *s, size_t n, char *out, size_t cap) {
  static const char up[]  = "AAAAAAACEEEEIIIIDNOOOOO*OUUUUYTs";   // U+00C0..U+00DF
  static const char low[] = "aaaaaaaceeeeiiiidnooooo/ouuuuyty";   // U+00E0..U+00FF
  size_t o = 0;
  for (size_t i = 0; i < n && o + 1 < cap;) {
    uint8_t c = s[i];
    uint32_t cp; int len;
    if (c < 0x80) { cp = c; len = 1; }
    else if ((c >> 5) == 6 && i + 1 < n) { cp = ((c & 0x1F) << 6) | (s[i + 1] & 0x3F); len = 2; }
    else if ((c >> 4) == 14 && i + 2 < n) { cp = ((c & 0x0F) << 12) | ((s[i + 1] & 0x3F) << 6) | (s[i + 2] & 0x3F); len = 3; }
    else if ((c >> 3) == 30 && i + 3 < n) { cp = 0xFFFF; len = 4; }
    else { cp = '?'; len = 1; }
    i += len;
    char r;
    if (cp >= 0x20 && cp < 0x7F) r = (char)cp;
    else if (cp >= 0xC0 && cp <= 0xDF) r = up[cp - 0xC0];
    else if (cp >= 0xE0 && cp <= 0xFF) r = low[cp - 0xE0];
    else if (cp == 0x2018 || cp == 0x2019) r = '\'';
    else if (cp == 0x201C || cp == 0x201D) r = '"';
    else if (cp == 0x2013 || cp == 0x2014) r = '-';
    else if (cp == 0x2026) r = '.';
    else if (cp < 0x20) continue;
    else r = '?';
    out[o++] = r;
  }
  out[o] = 0;
}

// Spotify appends a badge like "<icon> Video Available" to the metadata of tracks that have a video.
// The icon is not in our font (it would show as "?"), and the badge isn't part of the title, so remove it.
static void stripNoise(char *t) {
  static const char pat[] = "video available";
  const int pl = sizeof pat - 1;
  for (int i = 0; t[i];) {
    int k = 0;
    while (k < pl && t[i + k] && (t[i + k] | 0x20) == pat[k]) k++;
    if (k == pl) memmove(t + i, t + i + pl, strlen(t + i + pl) + 1);   // cut the phrase out
    else i++;
  }
  // trim leftover separators / replaced icon characters at both ends
  int n = strlen(t);
  while (n > 0 && (t[n - 1] == ' ' || t[n - 1] == '?' || t[n - 1] == '-')) t[--n] = 0;
  int a = 0;
  while (t[a] == ' ' || t[a] == '?' || t[a] == '-') a++;
  if (a) memmove(t, t + a, strlen(t + a) + 1);
}

static void setField(char *dst, const char *src) {   // dst is one of gInfo's char arrays
  char t[64]; strncpy(t, src, sizeof t - 1); t[sizeof t - 1] = 0;
  stripNoise(t);
  portENTER_CRITICAL(&gMux);
  if (strcmp(dst, t) != 0) { strcpy(dst, t); gInfo.version++; }
  portEXIT_CRITICAL(&gMux);
}

// ---- notification from the iPhone's Entity Update characteristic (runs in the BLE task) ----
static void onEntityUpdate(BLERemoteCharacteristic *, uint8_t *d, size_t len, bool) {
  if (len < 3) return;
  uint8_t ent = d[0], att = d[1], flags = d[2];
  AMSLOG("[AMS] update ent=%u att=%u flags=%u len=%u\n", ent, att, flags, (unsigned)len);
  const uint8_t *v = d + 3; size_t vl = len - 3;
  char tmp[64];
  if (ent == ENT_PLAYER) {
    char b[48]; size_t m = vl < sizeof b - 1 ? vl : sizeof b - 1; memcpy(b, v, m); b[m] = 0;
    if (att == 1) {                                   // "state,rate,elapsed"
      uint8_t p = (atoi(b) == 1);
      portENTER_CRITICAL(&gMux); if (gInfo.playing != p) { gInfo.playing = p; gInfo.version++; } portEXIT_CRITICAL(&gMux);
    } else if (att == 2) {                            // volume "0.0".."1.0"
      float f = atof(b);
      portENTER_CRITICAL(&gMux); if (gInfo.volume != f) { gInfo.volume = f; gInfo.version++; } portEXIT_CRITICAL(&gMux);
    }
  } else if (ent == ENT_TRACK && att <= 2) {          // 0 artist, 1 album, 2 title
    sanitize(v, vl, tmp, sizeof tmp);
    char *dst = att == 0 ? gInfo.artist : att == 1 ? gInfo.album : gInfo.title;
    setField(dst, tmp);
    if (flags & 1) gPendingFull |= (att == 2 ? 1 : att == 0 ? 2 : 4);   // value was truncated
  }
}

// re-read one truncated track attribute via the Entity Attribute characteristic (task context only)
static void fetchFull(uint8_t att, char *dst) {
  if (!gChAttr) return;
  uint8_t req[2] = {ENT_TRACK, att};
  gChAttr->writeValue(req, 2, true);
  auto v = gChAttr->readValue();                      // std::string (core 2.x) or String (core 3.x)
  char tmp[64];
  sanitize((const uint8_t *)v.c_str(), v.length(), tmp, sizeof tmp);
  if (tmp[0]) setField(dst, tmp);
}

// ---- security callbacks ----
class SecCb : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() override { return 123456; }
  void onPassKeyNotify(uint32_t) override {}
  bool onSecurityRequest() override { return true; }
  bool onConfirmPIN(uint32_t pin) override {          // iPhone shows the same number; we auto-accept
    AMSLOG("[AMS] pairing confirm code %06lu\n", (unsigned long)pin);
    portENTER_CRITICAL(&gMux); gInfo.passkey = pin; gInfo.version++; portEXIT_CRITICAL(&gMux);
    return true;
  }
  void onAuthenticationComplete(esp_ble_auth_cmpl_t c) override {
    portENTER_CRITICAL(&gMux); gInfo.passkey = 0; gInfo.version++; portEXIT_CRITICAL(&gMux);
    AMSLOG("[AMS] pairing %s (reason %u)\n", c.success ? "OK" : "FAILED", c.fail_reason);
    if (c.success) gAuthDone = true;
  }
};

class SrvCb : public BLEServerCallbacks {
  void onConnect(BLEServer *, esp_ble_gatts_cb_param_t *param) override {
    if (gPeer) { delete gPeer; gPeer = nullptr; }
    gPeer = new BLEAddress(param->connect.remote_bda);
    gPeerType = (uint8_t)param->connect.ble_addr_type;
    gAuthDone = false;
    gServerConnected = true;
    AMSLOG("[AMS] iPhone connected: %s (addr type %u)\n", gPeer->toString().c_str(), gPeerType);
  }
  void onDisconnect(BLEServer *) override { gServerConnected = false; AMSLOG("[AMS] iPhone disconnected\n"); }
};

// raw advertising bytes: Flags + "List of 128-bit Service Solicitation UUIDs" (AD type 0x15) = AMS.
// This is what makes iOS list the device and connect to it. UUID is stored little-endian.
static void addRaw(BLEAdvertisementData &ad, const uint8_t *b, size_t n) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ad.addData((char *)b, n);
#else
  ad.addData(std::string((const char *)b, n));
#endif
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void gapLog(esp_gap_ble_cb_event_t ev, esp_ble_gap_cb_param_t *p) {   // called by the library after its own handling
  switch (ev) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT: AMSLOG("[AMS] adv data set, status=%d\n", p->adv_data_raw_cmpl.status); break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT: AMSLOG("[AMS] scan response set, status=%d\n", p->scan_rsp_data_raw_cmpl.status); break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
      AMSLOG("[AMS] advertising start, status=%d (0 = OK)\n", p->adv_start_cmpl.status);
      if (p->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
        portENTER_CRITICAL(&gMux); gInfo.adv = 1; gInfo.version++; portEXIT_CRITICAL(&gMux);
      }
      break;
    default: break;
  }
}
#endif

static void startAdvertising(const char *name) {
  BLEAdvertising *adv = gServer->getAdvertising();
  adv->setScanResponse(true);
  // Advertising packet: Flags + "128-bit Service Solicitation" (AD type 0x15) for AMS. This is what makes iOS list
  // the device in Settings > Bluetooth and connect to it. The UUID is stored little-endian.
  BLEAdvertisementData ad;
  ad.setFlags(0x06);                                   // LE General Discoverable, BR/EDR not supported
  static const uint8_t sol[18] = {17, 0x15,
    0xDC, 0xF8, 0x55, 0xAD, 0x02, 0xC5, 0xF4, 0x8E, 0x3A, 0x43, 0x36, 0x0F, 0x2B, 0x50, 0xD3, 0x89};
  addRaw(ad, sol, sizeof sol);
  // Scan response: the device name (does not fit in the 31-byte advertising packet next to the UUID).
  BLEAdvertisementData sr;
  sr.setName(name);
  // Both packets are supplied by us, so the library skips its own async config chain and simply starts.
  adv->setAdvertisementData(ad);
  adv->setScanResponseData(sr);
  secMode();
  secCap(ESP_IO_CAP_OUT);
  secInitKeys();
  adv->start();
}

// ---- connection handling (task context; blocking calls are fine here) ----
static bool startMediaService() {
  auto svc = gClient->getService(AMS_SERVICE_UUID);
  if (!svc) { AMSLOG("[AMS] Apple Media Service NOT found on the phone\n"); return false; }
  AMSLOG("[AMS] Apple Media Service found\n");
  gChCmd = svc->getCharacteristic(AMS_REMOTE_CMD_UUID);
  gChAttr = svc->getCharacteristic(AMS_ENTITY_ATTR_UUID);
  auto upd = svc->getCharacteristic(AMS_ENTITY_UPDATE_UUID);
  if (!upd) { AMSLOG("[AMS] Entity Update characteristic missing\n"); return false; }
  upd->registerForNotify(onEntityUpdate);
  // subscribe: player (name, playback info, volume), queue, track (artist, album, title, duration)
  uint8_t player[] = {0, 0, 1, 2};
  uint8_t queue[]  = {1, 0, 1, 2, 3};
  uint8_t track[]  = {2, 0, 1, 2, 3};
  upd->writeValue(player, sizeof player, true);
  upd->writeValue(queue, sizeof queue, true);
  upd->writeValue(track, sizeof track, true);
  return true;
}

static void teardownClient() {
  gRunning = false; gChAttr = gChCmd = nullptr;
  if (gClient) { if (gClient->isConnected()) gClient->disconnect(); delete gClient; gClient = nullptr; }
}

static void handleConnection() {
  if (!gPeer) return;
  setInfo(1, true);
  teardownClient();
  gClient = BLEDevice::createClient();
  secMode();
  secCap(ESP_IO_CAP_IO);
  secRespKeys();
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!gClient->connect(*gPeer, gPeerType)) { AMSLOG("[AMS] client connect to the phone FAILED\n"); teardownClient(); return; }
#else
  if (!gClient->connect(*gPeer)) { AMSLOG("[AMS] client connect to the phone FAILED\n"); teardownClient(); return; }
#endif
  AMSLOG("[AMS] client link up, waiting for pairing/encryption...\n");
  uint32_t t0 = millis();                              // wait for pairing / encryption (up to 90 s)
  while (!gAuthDone && gClient->isConnected() && gServerConnected && millis() - t0 < 90000) delay(100);
  delay(200);
  if (!gAuthDone) AMSLOG("[AMS] gave up waiting for pairing (auth=%d connected=%d)\n", (int)gAuthDone, (int)gClient->isConnected());
  if (!gAuthDone || !gClient->isConnected() || !startMediaService()) { teardownClient(); return; }
  AMSLOG("[AMS] RUNNING - subscribed to media updates\n");
  gRunning = true;
  setInfo(2, false);
}

static void amsTask(void *) {
  bool was = false;
  for (;;) {
    if (gServerConnected && !was) { was = true; handleConnection(); }
    if (!gServerConnected && was) {
      was = false;
      delay(500);
      teardownClient();
      secReset();
      gAuthDone = false;
      setInfo(0, true);
      portENTER_CRITICAL(&gMux); gInfo.adv = 0; portEXIT_CRITICAL(&gMux);
      gServer->getAdvertising()->start();
    }
    if (gRunning) {
      uint32_t p = gPendingFull; gPendingFull = 0;
      if (p & 1) fetchFull(2, gInfo.title);
      if (p & 2) fetchFull(0, gInfo.artist);
      if (p & 4) fetchFull(1, gInfo.album);
    }
    delay(100);
  }
}

// ---- public API ----
static void begin(const char *deviceName) {
  memset(&gInfo, 0, sizeof gInfo); gInfo.volume = -1;
  BLEDevice::init(deviceName);
  BLEDevice::setMTU(185);                              // more room per notification (fewer truncations)
  gServer = BLEDevice::createServer();
  gServer->setCallbacks(new SrvCb());
  BLEDevice::setSecurityCallbacks(new SecCb());
  secMode();
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  BLEDevice::setCustomGapHandler(gapLog);
#endif
  startAdvertising(deviceName);
  AMSLOG("[AMS] advertising as \"%s\" - on the iPhone: Settings > Bluetooth\n", deviceName);
  xTaskCreatePinnedToCore(amsTask, "ams", 8192, nullptr, 1, nullptr, 0);
}

static void snapshot(Info &out) {
  portENTER_CRITICAL(&gMux); out = gInfo; portEXIT_CRITICAL(&gMux);
}

static void command(uint8_t id) {                      // e.g. AMS::CMD_NEXT (not wired to the UI yet)
  if (gChCmd && gRunning) gChCmd->writeValue(&id, 1, true);
}

// forget every paired phone (use if pairing gets stuck; also tap "Forget This Device" on the iPhone)
static void clearBonds() {
  int n = esp_ble_get_bond_device_num();
  if (n <= 0) return;
  esp_ble_bond_dev_t *list = (esp_ble_bond_dev_t *)malloc(n * sizeof(esp_ble_bond_dev_t));
  if (!list) return;
  if (esp_ble_get_bond_device_list(&n, list) == ESP_OK)
    for (int i = 0; i < n; i++) esp_ble_remove_bond_device(list[i].bd_addr);
  free(list);
}

}  // namespace AMS
