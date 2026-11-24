// A-OK Blind Bridge: ESP32 + CC1101 firmware for 433.92 MHz A-OK blind motors (sold under several
// brands, including Motolux). https://github.com/olliemaller/aok-blind-bridge
//
// Based on pgnehm/ESP32-Orvibo-A-OK-HomeAssistant-Blind-Bridge (protocol research, encoder rules,
// CC1101 async OOK approach). This version adds: on-device learning from the physical remote,
// live remote listening (positions follow the hand remote), web setup with no recompiling,
// separate open/close travel times, position memory across reboots, favourite positions,
// group blinds, travel-time calibration, a remote-button event entity and a Wi-Fi setup portal.

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <Update.h>
#include <math.h>
#include "aok_sun.h"
#include "aok_drift.h"
#include "aok_hosts.h"
#include <WebSocketsServer.h>
#ifdef USE_ETH
#include <ETH.h>
#endif
#include <WebServer.h>
#include <WiFi.h>
#include <time.h>
#include <vector>
#include <stdarg.h>

#include "aok_protocol.h"
#include "somfy_rts.h"
#include "web_ui_gz.h"

#define FW_VERSION "1.9.0"
#define PROJECT_URL "https://github.com/olliemaller/aok-blind-bridge"

// HW-395 / ESP32 DevKit defaults (VSPI). Change in the web UI under Radio if wired differently.
#ifndef DEFAULT_PIN_SCK
#define DEFAULT_PIN_SCK 18
#endif
#ifndef DEFAULT_PIN_MISO
#define DEFAULT_PIN_MISO 19
#endif
#ifndef DEFAULT_PIN_MOSI
#define DEFAULT_PIN_MOSI 23
#endif
#ifndef DEFAULT_PIN_CS
#define DEFAULT_PIN_CS 5
#endif
#ifndef DEFAULT_PIN_GDO0
#define DEFAULT_PIN_GDO0 12
#endif
#ifndef DEFAULT_PIN_GDO2
#define DEFAULT_PIN_GDO2 27
#endif
#ifndef DEFAULT_PIN_LED
#define DEFAULT_PIN_LED 2
#endif

namespace {

constexpr size_t kMaxBlinds = 16;
constexpr size_t kMaxFrames = 12;
constexpr size_t kRingSize = 2048;  // power of two
constexpr uint32_t kDedupeMs = 900;
constexpr uint32_t kReversePauseMs = 700;  // stop → reverse gap that protects the motor

enum class Motion : uint8_t { Stopped, Opening, Closing };

struct Pins {
  int sck = DEFAULT_PIN_SCK, miso = DEFAULT_PIN_MISO, mosi = DEFAULT_PIN_MOSI, cs = DEFAULT_PIN_CS;
  int gdo0 = DEFAULT_PIN_GDO0, gdo2 = DEFAULT_PIN_GDO2, led = DEFAULT_PIN_LED;
  float mhz = 433.92f;
  int paDbm = 10;
  int rxBw = 0;  // receive bandwidth in kHz; 0 = automatic (wide enough for Somfy at 433.42 MHz too, when any are set up)
};

constexpr float kSomfyMhz = 433.42f;
constexpr size_t kSomfyRemotes = 3;

struct NetCfg {
  String ssid, pass, hostname = "aok-blinds", adminPass;
  String brand = "A-OK";            // shown in the page title, setup Wi-Fi name and Home Assistant
  String tz, tzName;                // POSIX time zone string and the IANA name it came from
  String mqttHost;
  uint16_t mqttPort = 1883;
  String mqttUser, mqttPass, base = "aok_blinds", discovery = "homeassistant";
  String extraHosts;                // other names the page answers to (aok-blinds.lan), comma-separated
  uint8_t resyncEvery = 10;         // part-way moves before a re-sync at an end stop (0 = off)
  bool resyncThrough = false;       // a middle target may go via an end stop to re-sync
};

struct RemoteCfg {
  bool have[3] = {false, false, false};
  uint32_t prefix = 0;
  uint8_t remoteId = 0;
  uint16_t base[3] = {0, 0, 0};
  aok::Timings timings;
  bool listen = true;
  uint16_t seen = 0;  // channels this remote has been heard on
  // Other remotes the bridge follows (a second hand remote, a wall switch): their
  // presses move the blinds' positions here too. Up to 3.
  uint32_t extraPrefix[3] = {0, 0, 0};
  uint8_t extraId[3] = {0, 0, 0};
  uint8_t extraCount = 0;
  // The programming (P) button, learned when it's pressed on the hand remote,
  // lets the bridge put a motor into pairing mode itself.
  bool haveProg = false;
  uint16_t progBase = 0;
  // The bridge's own remote ID (same prefix and codes, different ID). Channels in
  // ownMask have their motors paired to it, so they no longer need the hand remote.
  uint8_t ownId = 0;
  uint16_t ownMask = 0;
  bool repeat = false;  // re-send hand-remote presses, for motors the remote barely reaches
};

struct BlindCfg {
  String id, name, room, deviceClass = "shade";
  uint16_t mask = 1;
  uint32_t openMs = 30000, closeMs = 30000;
  uint16_t startMs = 0;  // motor start-up lag: time between the radio frame and the blind actually moving
  uint16_t tiltMs = 0;   // venetian blinds: time to turn the slats from closed to open (0 = no tilt)
  bool invert = false;
  String presetName[4];             // named positions ("Half", "Privacy"...)
  int presetPos[4] = {-1, -1, -1, -1};
  int favourite = -1;                // A-OK: favourite position. Somfy: the motor's "My" position, if known
  // Somfy RTS motors are addressed one by one (no channels): mask is 0 for them.
  bool somfy = false;
  uint32_t somfyAddr = 0;            // the bridge's own remote address for this motor
  uint32_t somfyRemote[kSomfyRemotes] = {0, 0, 0};  // hand remotes (Situo, Telis...) the bridge follows
  uint8_t somfyRemoteCount = 0;
};

struct BlindState {
  int position = -1;  // 0 closed .. 100 open, -1 unknown
  int startPos = -1;
  int target = -1;
  Motion motion = Motion::Stopped;
  uint32_t startMs = 0, durMs = 0;
  bool autoStop = false;
  int after = -1;              // after an endpoint run, go on to this position (reference run)
  int tilt = -1;               // slat angle estimate 0 (closed) .. 100 (open), -1 unknown
  bool locked = false;         // Home Assistant, scenes, schedules and the page leave it alone (Stop still works)
  bool interlock = false;      // window/door open (from Home Assistant): it won't close
  bool windLocked = false;     // an awning retracted for wind: locked until the wind drops
  int pending = -1;            // reverse requested while moving: go here once the motor has stopped
  uint32_t pendingAtMs = 0;
  uint32_t lastPublishMs = 0;
  String lastSource = "none";
  bool dirty = true;
  uint8_t partials = 0;        // runs that stopped part way since the last end stop (drift correction)
};

struct HeardFrame {
  uint32_t ms = 0;
  aok::Frame f;
  bool matched = false;
  uint16_t repeats = 1;
  int16_t rssi = 0;  // signal strength of the frame, dBm
  bool somfy = false;                // Somfy RTS frame: the fields below, not f (f.payload is the raw frame)
  somfy::Frame s;
};

// ---- automation: scenes and schedules (saved together as "auto")
constexpr size_t kMaxScenes = 8, kMaxSceneItems = 16, kMaxSchedules = 12;
struct SceneItem { String id; int position = 0; };
struct Scene { String id, name; SceneItem items[kMaxSceneItems]; uint8_t count = 0; };
enum class When : uint8_t { Time, Sunrise, Sunset };
struct Schedule {
  bool enabled = true;
  uint8_t days = 0x7F;         // bit 0 = Monday .. bit 6 = Sunday
  When when = When::Time;
  int16_t minutes = 0;         // time of day for Time; offset (+/-) for Sunrise/Sunset
  String target = "all";       // "all", a blind id, or "scene:<id>"
  String action = "OPEN";      // OPEN, CLOSE, STOP or a position 0-100
  int lastDay = -1;            // day of year it last ran, so it runs once a day
};
Scene scenes[kMaxScenes];
size_t sceneCount = 0;
Schedule schedules[kMaxSchedules];
size_t scheduleCount = 0;
float siteLat = NAN, siteLon = NAN;
bool windActive = false;       // set over MQTT by a wind sensor automation

Pins pins;
NetCfg net;
RemoteCfg remote;
BlindCfg cfg[kMaxBlinds];
BlindState st[kMaxBlinds];
size_t blindCount = 0;

Preferences prefs;
WebServer web(80);
DNSServer dns;
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

bool radioReady = false, apMode = false, otaReady = false, mdnsReady = false;
bool learnMode = false, rebootPending = false, discoveryPending = true;
bool learnLocked = false;            // a learn session sticks to the first remote it hears
bool learnExtra = false;             // this learn session adds a second remote instead
bool safeMode = false;               // crashed 3 times in a row: radio and MQTT stay off
RTC_NOINIT_ATTR uint32_t crashMagic;
RTC_NOINIT_ATTR uint32_t crashCount;
int16_t lastRssi = 0;                // signal strength of the last frame from our remotes
bool pressActive = false;            // a hand-remote button is being held
uint32_t pressStartMs = 0;
uint64_t repeatPayload = 0;          // last hand-remote frame, re-sent when the repeater is on
// Pairing a motor to the bridge's own ID: P, P, then Up from the new ID.
int pairChannel = 0;                 // 1-16 while a pairing runs
uint8_t pairStage = 0;
uint32_t pairAtMs = 0;
String pairStatus = "idle";
WebSocketsServer ws(81);
uint32_t lastWsPushMs = 0, lastWsRev = 0;
aok::Action pressAction = aok::Action::Unknown;
int lastScheduleMinute = -1;
int lastScheduleDay = -1;
uint32_t learnPrefix = 0;
uint8_t learnRemoteId = 0;
uint32_t stateRev = 1;               // bumps when frames, calibration history or config change (page refetches)
bool positionsDirty = false;         // positions are written to flash a few seconds after things settle
uint32_t positionsDirtyMs = 0;
uint32_t learnStartedMs = 0, rebootAtMs = 0, lastMqttTryMs = 0, lastDiagMs = 0, staStartMs = 0;
uint32_t lastTxMs = 0, lastRxFlashMs = 0;
uint32_t mqttRetryMs = 5000;           // grows to 60 s while the broker is unreachable
HeardFrame frames[kMaxFrames];
size_t frameCount = 0, frameHead = 0;
uint64_t lastPayload = 0;
uint32_t lastPayloadMs = 0;
String learnStatus = "idle";
String lastEventText = "none";

// calibration
int calIndex = -1;
Motion calMotion = Motion::Stopped;
uint32_t calStartMs = 0;

struct CalRecord { String name; bool opening = false; uint32_t ms = 0, previousMs = 0; time_t when = 0; };
constexpr size_t kCalHistory = 20;
CalRecord calHistory[kCalHistory];
size_t calHistoryCount = 0;

// in-memory log shown on the Logs page
constexpr size_t kLogLines = 120;
struct LogLine { uint32_t seq = 0, ms = 0; time_t when = 0; char level = 'I'; String text; };
LogLine logLines[kLogLines];
uint32_t logSeq = 0;
bool ntpStarted = false;

// RX ring buffer filled by the GDO2 edge interrupt
volatile uint32_t ring[kRingSize];
volatile uint16_t ringHead = 0, ringTail = 0;
volatile uint32_t lastEdgeUs = 0, ringOverflow = 0;
volatile int rxPin = -1;
aok::Decoder decoder;
somfy::Decoder somfyDecoder;
float rxMhzNow = 0, rxBwNow = 0;     // what the receiver is tuned to right now
int somfyLearn = -1;                 // blind listening for a Somfy hand remote
uint32_t somfyLearnMs = 0;
String somfyStatus = "idle";
uint32_t lastSomfyAddr = 0;          // de-duplicates the repeats of one press (same address and rolling code)
uint16_t lastSomfyCode = 0;
uint32_t lastSomfyMs = 0;

// ---------------------------------------------------------------- helpers
#ifdef USE_ETH
bool ethUp = false;  // WT32-ETH01 and similar: wired network, Wi-Fi optional
#endif
bool netUp() {
#ifdef USE_ETH
  if (ethUp) return true;
#endif
  return WiFi.status() == WL_CONNECTED;
}
IPAddress netIp() {
#ifdef USE_ETH
  if (ethUp) return ETH.localIP();
#endif
  return WiFi.localIP();
}
bool anyMoving();
void sendError(const String& msg, int code);
bool runScene(const String& key, const String& source, String& err);
void setLock(size_t i, bool locked, bool save = true);
bool channelCommand(const String& payload, const String& source, String& err);
void setWind(bool on);
bool remoteKnown(uint32_t prefix, uint8_t id);
int clampPos(int v) { return v < 0 ? 0 : (v > 100 ? 100 : v); }

String hex(uint64_t v, int digits) {
  char buf[20];
  if (digits > 8) snprintf(buf, sizeof(buf), "%0*llX", digits, static_cast<unsigned long long>(v));
  else snprintf(buf, sizeof(buf), "%0*lX", digits, static_cast<unsigned long>(v));
  return String(buf);
}

uint32_t parseHex(const String& s) { return static_cast<uint32_t>(strtoul(s.c_str(), nullptr, 16)); }

String maskText(uint16_t mask) {
  String out;
  for (int ch = 1; ch <= 16; ++ch)
    if (mask & (1u << (ch - 1))) { if (out.length()) out += ","; out += ch; }
  return out;
}

const char* motionName(Motion m) { return m == Motion::Opening ? "opening" : (m == Motion::Closing ? "closing" : "stopped"); }

String slug(const String& in) {
  String out;
  for (char c : in) {
    if (isalnum(static_cast<unsigned char>(c))) out += static_cast<char>(tolower(c));
    else if (out.length() && out[out.length() - 1] != '_') out += '_';
  }
  while (out.endsWith("_")) out.remove(out.length() - 1);
  return out.length() ? out : String("blind");
}

int findBlind(const String& id) {
  for (size_t i = 0; i < blindCount; ++i) if (cfg[i].id == id) return static_cast<int>(i);
  return -1;
}

bool remoteComplete() { return remote.have[0] && remote.have[1] && remote.have[2]; }

void ledSet(bool on) { if (pins.led >= 0) digitalWrite(pins.led, on ? HIGH : LOW); }

time_t wallClock() { const time_t t = time(nullptr); return t > 1700000000 ? t : 0; }

bool isSomfy(size_t i) { return cfg[i].somfy; }
bool anySomfy() { for (size_t i = 0; i < blindCount; ++i) if (cfg[i].somfy) return true; return false; }
// A-OK blind j moves with a frame sent on `mask` (all of its channels are in it). Somfy blinds never do.
bool inMask(size_t j, uint16_t mask) { return !cfg[j].somfy && (cfg[j].mask & ~mask) == 0; }
bool isGroup(size_t i) { return !cfg[i].somfy && __builtin_popcount(cfg[i].mask) > 1; }

void logf(char level, const char* fmt, ...) {
  char buf[220];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.printf("[%c] %s\n", level, buf);
  LogLine& l = logLines[logSeq % kLogLines];
  l.seq = ++logSeq; l.ms = millis(); l.when = wallClock(); l.level = level; l.text = buf;
}
#define LOGI(...) logf('I', __VA_ARGS__)
#define LOGW(...) logf('W', __VA_ARGS__)
#define LOGE(...) logf('E', __VA_ARGS__)

// what a received frame was, including buttons the bridge doesn't act on
const char* frameKind(const aok::Frame& f) {
  switch ((f.command >> 8) & 0xFF) {
    case 0xF4: return "up";
    case 0xBC: return "down";
    case 0xDC: return "stop";
    case 0xAC: return "program";
    case 0xDB: return "release";
    default: return "other";
  }
}

// ---------------------------------------------------------------- persistence
void saveNet() {
  JsonDocument d;
  d["ssid"] = net.ssid; d["pass"] = net.pass; d["host"] = net.hostname; d["admin"] = net.adminPass;
  d["mh"] = net.mqttHost; d["mp"] = net.mqttPort; d["mu"] = net.mqttUser; d["mw"] = net.mqttPass;
  d["base"] = net.base; d["disc"] = net.discovery;
  d["brand"] = net.brand; d["tz"] = net.tz; d["tzn"] = net.tzName;
  d["xh"] = net.extraHosts; d["rs"] = net.resyncEvery; d["rst"] = net.resyncThrough;
  String s; serializeJson(d, s); prefs.putString("net", s);
}

void loadNet() {
  JsonDocument d;
  const bool upgraded = prefs.isKey("net");
  if (deserializeJson(d, prefs.getString("net", "{}"))) return;
  net.ssid = d["ssid"] | ""; net.pass = d["pass"] | ""; net.hostname = d["host"] | "aok-blinds";
  net.adminPass = d["admin"] | ""; net.mqttHost = d["mh"] | ""; net.mqttPort = d["mp"] | 1883;
  net.mqttUser = d["mu"] | ""; net.mqttPass = d["mw"] | ""; net.base = d["base"] | "aok_blinds";
  net.discovery = d["disc"] | "homeassistant";
  net.brand = d["brand"] | "";
  net.tz = d["tz"] | ""; net.tzName = d["tzn"] | "";
  net.extraHosts = d["xh"] | ""; net.resyncEvery = d["rs"] | 10; net.resyncThrough = d["rst"] | false;
  // settings saved by 1.0.x/1.1.x: keep what those versions did
  if (!net.brand.length()) net.brand = net.hostname.indexOf("motolux") >= 0 ? "Motolux" : "A-OK";
  if (upgraded && !d["tz"].is<const char*>()) { net.tz = "AEST-10AEDT,M10.1.0,M4.1.0/3"; net.tzName = "Australia/Melbourne"; }
}

void savePins() {
  JsonDocument d;
  d["sck"] = pins.sck; d["miso"] = pins.miso; d["mosi"] = pins.mosi; d["cs"] = pins.cs;
  d["gdo0"] = pins.gdo0; d["gdo2"] = pins.gdo2; d["led"] = pins.led; d["mhz"] = pins.mhz; d["pa"] = pins.paDbm;
  d["rx_bw"] = pins.rxBw;
  String s; serializeJson(d, s); prefs.putString("pins", s);
}

void loadPins() {
  JsonDocument d;
  if (deserializeJson(d, prefs.getString("pins", "{}"))) return;
  pins.sck = d["sck"] | pins.sck; pins.miso = d["miso"] | pins.miso; pins.mosi = d["mosi"] | pins.mosi;
  pins.cs = d["cs"] | pins.cs; pins.gdo0 = d["gdo0"] | pins.gdo0; pins.gdo2 = d["gdo2"] | pins.gdo2;
  pins.led = d["led"] | pins.led; pins.mhz = d["mhz"] | pins.mhz; pins.paDbm = d["pa"] | pins.paDbm;
  pins.rxBw = d["rx_bw"] | pins.rxBw;
}

uint16_t parseChannels(const String& text);

void remoteToJson(JsonObject o) {
  o["prefix"] = hex(remote.prefix, 6);
  o["remote_id"] = hex(remote.remoteId, 2);
  o["open"] = remote.have[0] ? hex(remote.base[0], 4) : "";
  o["close"] = remote.have[1] ? hex(remote.base[1], 4) : "";
  o["stop"] = remote.have[2] ? hex(remote.base[2], 4) : "";
  o["listen"] = remote.listen;
  o["seen"] = maskText(remote.seen);
  o["prog"] = remote.haveProg ? hex(remote.progBase, 4) : "";
  o["own_id"] = remote.ownId ? hex(remote.ownId, 2) : "";
  o["own_channels"] = maskText(remote.ownMask);
  o["repeat"] = remote.repeat;
  JsonArray ex = o["extra"].to<JsonArray>();
  for (uint8_t k = 0; k < remote.extraCount; ++k) {
    JsonObject x = ex.add<JsonObject>();
    x["prefix"] = hex(remote.extraPrefix[k], 6); x["remote_id"] = hex(remote.extraId[k], 2);
  }
  JsonObject t = o["timings"].to<JsonObject>();
  t["short"] = remote.timings.shortUs; t["long"] = remote.timings.longUs;
  t["sync_high"] = remote.timings.syncHighUs; t["sync_low"] = remote.timings.syncLowUs;
  t["gap"] = remote.timings.gapUs; t["repeats"] = remote.timings.repeats;
}

void remoteFromJson(JsonObjectConst o) {
  if (o["prefix"].is<const char*>()) remote.prefix = parseHex(o["prefix"].as<String>()) & 0xFFFFFF;
  if (o["remote_id"].is<const char*>()) remote.remoteId = parseHex(o["remote_id"].as<String>()) & 0xFF;
  const char* keys[3] = {"open", "close", "stop"};
  for (int a = 0; a < 3; ++a) {
    if (!o[keys[a]].is<const char*>()) continue;
    String v = o[keys[a]].as<String>();
    remote.have[a] = v.length() > 0;
    remote.base[a] = remote.have[a] ? parseHex(v) : 0;
  }
  if (o["listen"].is<bool>()) remote.listen = o["listen"];
  if (o["seen"].is<const char*>()) remote.seen = parseChannels(o["seen"].as<String>());
  if (o["prog"].is<const char*>()) {
    const String v = o["prog"].as<String>();
    remote.haveProg = v.length() > 0;
    remote.progBase = remote.haveProg ? parseHex(v) : 0;
  }
  if (o["own_id"].is<const char*>()) remote.ownId = parseHex(o["own_id"].as<String>()) & 0xFF;
  if (o["own_channels"].is<const char*>()) remote.ownMask = parseChannels(o["own_channels"].as<String>());
  if (o["repeat"].is<bool>()) remote.repeat = o["repeat"];
  if (o["extra"].is<JsonArrayConst>()) {
    remote.extraCount = 0;
    for (JsonObjectConst x : o["extra"].as<JsonArrayConst>()) {
      if (remote.extraCount >= 3) break;
      remote.extraPrefix[remote.extraCount] = parseHex(x["prefix"] | "0") & 0xFFFFFF;
      remote.extraId[remote.extraCount] = parseHex(x["remote_id"] | "0") & 0xFF;
      remote.extraCount++;
    }
  }
  JsonObjectConst t = o["timings"];
  if (!t.isNull()) {
    remote.timings.shortUs = t["short"] | remote.timings.shortUs;
    remote.timings.longUs = t["long"] | remote.timings.longUs;
    remote.timings.syncHighUs = t["sync_high"] | remote.timings.syncHighUs;
    remote.timings.syncLowUs = t["sync_low"] | remote.timings.syncLowUs;
    remote.timings.gapUs = t["gap"] | remote.timings.gapUs;
    remote.timings.repeats = t["repeats"] | remote.timings.repeats;
  }
}

void saveRemote() {
  JsonDocument d; remoteToJson(d.to<JsonObject>());
  String s; serializeJson(d, s); prefs.putString("remote", s);
}

void loadRemote() {
  JsonDocument d;
  if (deserializeJson(d, prefs.getString("remote", "{}"))) return;
  remoteFromJson(d.as<JsonObjectConst>());
}

// Somfy rolling codes live in their own keys ("rc" + address), written before every send so a
// restart can never make the bridge reuse a code the motor has already seen.
String rcKey(uint32_t addr) { return String("rc") + hex(addr, 6); }
uint16_t somfyCode(uint32_t addr) { return prefs.getUShort(rcKey(addr).c_str(), 1); }
void setSomfyCode(uint32_t addr, uint16_t code) { prefs.putUShort(rcKey(addr).c_str(), code); }
// Only ever moves the code forward (restoring an old backup mustn't wind it back).
void raiseSomfyCode(uint32_t addr, uint16_t code) { if (code > somfyCode(addr)) setSomfyCode(addr, code); }

uint32_t parseHex24(const String& t) {
  if (!t.length() || t.length() > 6) return 0;
  for (char c : t) if (!isxdigit(static_cast<unsigned char>(c))) return 0;
  return strtoul(t.c_str(), nullptr, 16) & 0xFFFFFF;
}

// A fresh address for a new Somfy blind: from this ESP32's MAC, then a counter that only goes up,
// so a deleted blind's address (which its motor may still know) is never handed out again.
uint32_t newSomfyAddress(const BlindCfg* list, size_t n) {
  uint8_t mac[6]; WiFi.macAddress(mac);
  const uint32_t base = (static_cast<uint32_t>(mac[3]) << 16) | (static_cast<uint32_t>(mac[4]) << 8) | mac[5];
  uint32_t next = prefs.getUInt("somfynext", 1);
  for (;;) {
    const uint32_t a = (base + next * 0x10101u) & 0xFFFFFF;
    next++;
    bool used = a == 0;
    for (size_t k = 0; k < n && !used; ++k) {
      used |= list[k].somfyAddr == a;
      for (uint8_t r = 0; r < list[k].somfyRemoteCount; ++r) used |= list[k].somfyRemote[r] == a;
    }
    if (!used) { prefs.putUInt("somfynext", next); return a; }
  }
}

void blindToJson(size_t i, JsonObject o) {
  o["id"] = cfg[i].id; o["name"] = cfg[i].name; o["channels"] = maskText(cfg[i].mask);
  o["open_ms"] = cfg[i].openMs; o["close_ms"] = cfg[i].closeMs; o["invert"] = cfg[i].invert;
  o["start_ms"] = cfg[i].startMs;
  o["tilt_ms"] = cfg[i].tiltMs;
  JsonArray pr = o["presets"].to<JsonArray>();
  for (int k = 0; k < 4; ++k) {
    if (cfg[i].presetPos[k] < 0) continue;
    JsonObject x = pr.add<JsonObject>();
    x["name"] = cfg[i].presetName[k]; x["position"] = cfg[i].presetPos[k];
  }
  o["favourite"] = cfg[i].favourite; o["device_class"] = cfg[i].deviceClass; o["room"] = cfg[i].room;
  o["type"] = cfg[i].somfy ? "somfy" : "aok";
  if (cfg[i].somfy) {
    o["somfy_address"] = hex(cfg[i].somfyAddr, 6);
    o["somfy_code"] = somfyCode(cfg[i].somfyAddr);
    JsonArray r = o["somfy_remotes"].to<JsonArray>();
    for (uint8_t k = 0; k < cfg[i].somfyRemoteCount; ++k) r.add(hex(cfg[i].somfyRemote[k], 6));
  }
}

uint16_t parseChannels(const String& text) {
  uint16_t mask = 0; int value = 0; bool any = false;
  for (size_t k = 0; k <= text.length(); ++k) {
    char c = k < text.length() ? text[k] : ',';
    if (isdigit(static_cast<unsigned char>(c))) { value = value * 10 + (c - '0'); any = true; }
    else { if (any && value >= 1 && value <= 16) mask |= 1u << (value - 1); value = 0; any = false; }
  }
  return mask;
}

bool blindsFromJson(JsonArrayConst arr, String& err) {
  BlindCfg next[kMaxBlinds];
  size_t n = 0;
  for (JsonObjectConst o : arr) {
    if (n >= kMaxBlinds) { err = "maximum 16 blinds"; return false; }
    BlindCfg b;
    b.name = o["name"] | "";
    b.name.trim();
    if (!b.name.length()) { err = "every blind needs a name"; return false; }
    b.id = o["id"] | "";
    if (!b.id.length()) b.id = slug(b.name);
    b.id = slug(b.id);
    for (size_t k = 0; k < n; ++k) if (next[k].id == b.id) { b.id += "_"; b.id += n; }
    b.somfy = String(o["type"] | "aok") == "somfy";
    if (b.somfy) {
      b.mask = 0;
      b.somfyAddr = parseHex24(o["somfy_address"] | "");
      if (!b.somfyAddr) {  // keep the address of a blind that's being edited, or make a new one
        const int old = findBlind(b.id);
        if (old >= 0 && cfg[old].somfy) b.somfyAddr = cfg[old].somfyAddr;
      }
      for (JsonVariantConst v : o["somfy_remotes"].as<JsonArrayConst>()) {
        const uint32_t a = parseHex24(v.as<String>());
        if (a && b.somfyRemoteCount < kSomfyRemotes) b.somfyRemote[b.somfyRemoteCount++] = a;
      }
    } else {
      b.mask = o["channels"].is<const char*>() ? parseChannels(o["channels"].as<String>()) : (o["mask"] | 0);
      if (!b.mask) { err = b.name + ": choose at least one channel (1-16)"; return false; }
    }
    b.openMs = constrain(static_cast<uint32_t>(o["open_ms"] | 30000), 1000u, 300000u);
    b.closeMs = constrain(static_cast<uint32_t>(o["close_ms"] | 30000), 1000u, 300000u);
    b.invert = o["invert"] | false;
    b.startMs = constrain(static_cast<int>(o["start_ms"] | 0), 0, 3000);
    b.tiltMs = constrain(static_cast<int>(o["tilt_ms"] | 0), 0, 5000);
    int k = 0;
    for (JsonObjectConst x : o["presets"].as<JsonArrayConst>()) {
      if (k >= 4) break;
      const int pos = x["position"] | -1;
      if (pos < 0 || pos > 100) continue;
      b.presetName[k] = x["name"] | "";
      b.presetName[k].trim();
      if (!b.presetName[k].length()) b.presetName[k] = String(pos) + "%";
      b.presetPos[k++] = pos;
    }
    b.favourite = o["favourite"] | -1;
    if (b.favourite > 100) b.favourite = 100;
    b.deviceClass = o["device_class"] | "shade";
    b.room = o["room"] | "";
    b.room.trim();
    if (b.somfy) {
      b.tiltMs = 0;
      for (size_t k = 0; k < n; ++k)
        if (next[k].somfy && b.somfyAddr && next[k].somfyAddr == b.somfyAddr) { err = b.name + ": same Somfy address as " + next[k].name; return false; }
    }
    next[n++] = b;
  }
  for (size_t i = 0; i < n; ++i) {
    if (next[i].somfy && !next[i].somfyAddr) next[i].somfyAddr = newSomfyAddress(next, n);
  }
  // rolling codes from a backup (or the page) only ever move a code forward
  size_t idx = 0;
  for (JsonObjectConst o : arr) {
    if (idx < n && next[idx].somfy && o["somfy_code"].is<int>()) raiseSomfyCode(next[idx].somfyAddr, o["somfy_code"].as<uint16_t>());
    idx++;
  }
  // keep runtime state for blinds that still exist
  BlindState nextState[kMaxBlinds];
  for (size_t i = 0; i < n; ++i) {
    int old = findBlind(next[i].id);
    if (old >= 0) nextState[i] = st[old];
    nextState[i].dirty = true;
  }
  for (size_t i = 0; i < n; ++i) { cfg[i] = next[i]; st[i] = nextState[i]; }
  blindCount = n;
  return true;
}

void saveBlinds() {
  JsonDocument d; JsonArray a = d.to<JsonArray>();
  for (size_t i = 0; i < blindCount; ++i) blindToJson(i, a.add<JsonObject>());
  String s; serializeJson(d, s); prefs.putString("blinds", s);
}

void loadBlinds() {
  JsonDocument d; String err;
  if (deserializeJson(d, prefs.getString("blinds", "[]"))) return;
  blindsFromJson(d.as<JsonArrayConst>(), err);
}

void requestPositionSave() { positionsDirty = true; positionsDirtyMs = millis(); }

void savePositions() {
  positionsDirty = false;
  JsonDocument d;
  for (size_t i = 0; i < blindCount; ++i) {
    if (st[i].motion != Motion::Stopped) continue;
    d[cfg[i].id] = st[i].position;
    if (st[i].partials) d["#"][cfg[i].id] = st[i].partials;  // "#" can't be a blind id; older firmware ignores it
  }
  String s; serializeJson(d, s);
  if (s != prefs.getString("pos", "")) prefs.putString("pos", s);
}

void loadPositions() {
  JsonDocument d;
  if (deserializeJson(d, prefs.getString("pos", "{}"))) return;
  for (size_t i = 0; i < blindCount; ++i) {
    int p = d[cfg[i].id] | -1;
    st[i].position = st[i].startPos = st[i].target = p < 0 ? -1 : clampPos(p);
    st[i].lastSource = p < 0 ? "none" : "restored";
    st[i].partials = d["#"][cfg[i].id] | 0;
  }
}

void saveCalHistory() {
  JsonDocument d; JsonArray a = d.to<JsonArray>();
  for (size_t k = 0; k < calHistoryCount; ++k) {
    JsonObject o = a.add<JsonObject>();
    o["n"] = calHistory[k].name; o["o"] = calHistory[k].opening; o["ms"] = calHistory[k].ms;
    o["pm"] = calHistory[k].previousMs; o["t"] = static_cast<uint32_t>(calHistory[k].when);
  }
  String s; serializeJson(d, s); prefs.putString("calhist", s);
}

void loadCalHistory() {
  JsonDocument d;
  if (deserializeJson(d, prefs.getString("calhist", "[]"))) return;
  calHistoryCount = 0;
  for (JsonObjectConst o : d.as<JsonArrayConst>()) {
    if (calHistoryCount >= kCalHistory) break;
    CalRecord& r = calHistory[calHistoryCount++];
    r.name = o["n"] | ""; r.opening = o["o"] | false; r.ms = o["ms"] | 0; r.previousMs = o["pm"] | 0;
    r.when = static_cast<time_t>(o["t"] | 0);
  }
}

void addCalRecord(const String& name, bool opening, uint32_t ms, uint32_t previousMs) {
  if (calHistoryCount == kCalHistory) { for (size_t k = 1; k < kCalHistory; ++k) calHistory[k - 1] = calHistory[k]; calHistoryCount--; }
  CalRecord& r = calHistory[calHistoryCount++];
  r.name = name; r.opening = opening; r.ms = ms; r.previousMs = previousMs; r.when = wallClock();
  saveCalHistory();
  stateRev++;
}

// ---------------------------------------------------------------- radio
void IRAM_ATTR onEdge() {
  const uint32_t now = micros();
  const uint32_t dur = now - lastEdgeUs;
  lastEdgeUs = now;
  if (dur < 100) return;  // glitch filter
  const bool levelNow = digitalRead(rxPin);
  const uint32_t v = (dur > 0x7FFFFFFF ? 0x7FFFFFFF : dur) | (levelNow ? 0 : 0x80000000u);  // level before the edge
  const uint16_t next = (ringHead + 1) & (kRingSize - 1);
  if (next == ringTail) { ringOverflow++; return; }
  ring[ringHead] = v;
  ringHead = next;
}

// With Somfy blinds set up, one receiver hears both 433.42 and the A-OK frequency: it sits half
// way between them with a bandwidth wide enough for both. Otherwise it stays narrow on pins.mhz.
float rxMhz() { return anySomfy() ? (pins.mhz + kSomfyMhz) / 2.0f : pins.mhz; }
float rxBw() { return pins.rxBw > 0 ? static_cast<float>(pins.rxBw) : (anySomfy() ? 812.0f : 270.0f); }

void rxStart() {
  if (!radioReady) return;
  const float bw = rxBw();
  if (bw != rxBwNow) { ELECHOUSE_cc1101.setRxBW(bw); rxBwNow = bw; }
  rxMhzNow = rxMhz();
  ELECHOUSE_cc1101.SetRx(rxMhzNow);
  decoder.clear();
  somfyDecoder.clear();
  rxPin = pins.gdo2;
  pinMode(pins.gdo2, INPUT);
  lastEdgeUs = micros();
  attachInterrupt(digitalPinToInterrupt(pins.gdo2), onEdge, CHANGE);
}

void rxStop() {
  if (pins.gdo2 >= 0) detachInterrupt(digitalPinToInterrupt(pins.gdo2));
}

void setupRadio() {
  pinMode(pins.gdo0, OUTPUT);
  digitalWrite(pins.gdo0, LOW);
  ELECHOUSE_cc1101.setSpiPin(pins.sck, pins.miso, pins.mosi, pins.cs);
  ELECHOUSE_cc1101.Init();
  radioReady = ELECHOUSE_cc1101.getCC1101();
  if (!radioReady) { LOGE("CC1101 radio not detected: check the wiring (SCK %d, MISO %d, MOSI %d, CSN %d)", pins.sck, pins.miso, pins.mosi, pins.cs); return; }
  ELECHOUSE_cc1101.setCCMode(0);       // asynchronous serial mode (raw OOK on the GDO pins)
  ELECHOUSE_cc1101.setModulation(2);   // ASK/OOK
  ELECHOUSE_cc1101.setMHZ(pins.mhz);
  ELECHOUSE_cc1101.setPA(pins.paDbm);
  ELECHOUSE_cc1101.setDRate(4.8f);
  rxBwNow = 0;  // rxStart() sets the bandwidth
  ELECHOUSE_cc1101.setSyncMode(0);
  ELECHOUSE_cc1101.setCrc(false);
  ELECHOUSE_cc1101.setPktFormat(3);
  // GDO2 = received data out; GDO0 = high impedance in RX (it becomes the TX data input when transmitting)
  ELECHOUSE_cc1101.SpiWriteReg(0x00, 0x0D);  // IOCFG2
  ELECHOUSE_cc1101.SpiWriteReg(0x02, 0x2E);  // IOCFG0
  rxStart();
  if (anySomfy()) LOGI("CC1101 radio ready: listening at %.2f MHz (%.0f kHz wide) for A-OK on %.2f and Somfy on %.2f", rxMhzNow, rxBwNow, pins.mhz, kSomfyMhz);
  else LOGI("CC1101 radio ready at %.2f MHz", pins.mhz);
}

bool transmitPayload(uint64_t payload, String& err) {
  if (!radioReady) { err = "CC1101 radio not detected"; return false; }
  uint16_t pulses[aok::kFramePulses];
  const size_t n = aok::encodeFrame(payload, remote.timings, pulses, aok::kFramePulses);
  rxStop();
  ELECHOUSE_cc1101.SetTx(pins.mhz);
  pinMode(pins.gdo0, OUTPUT);
  ledSet(true);
  for (uint8_t r = 0; r < remote.timings.repeats; ++r) {
    bool level = true;
    for (size_t k = 0; k < n; ++k) {
      digitalWrite(pins.gdo0, level ? HIGH : LOW);
      delayMicroseconds(pulses[k]);
      level = !level;
    }
    digitalWrite(pins.gdo0, LOW);
    delayMicroseconds(remote.timings.gapUs);
  }
  ledSet(false);
  lastTxMs = millis();
  rxStart();
  return true;
}

bool sendAction(uint16_t mask, aok::Action action, String& err) {
  const int a = static_cast<int>(action);
  if (a > 2) { err = "unknown action"; return false; }
  if (!remote.have[a]) { err = String("remote not learned yet: ") + aok::actionName(action); return false; }
  const char* name = action == aok::Action::Open ? "Up" : action == aok::Action::Close ? "Down" : "Stop";
  const uint16_t own = remote.ownId ? (mask & remote.ownMask) : 0, copied = mask & ~own;
  if (copied) {
    LOGI("Sent %s on channel %s", name, maskText(copied).c_str());
    if (!transmitPayload(aok::makePayload(remote.prefix, remote.remoteId, copied, remote.base[a]), err)) return false;
  }
  if (own) {
    LOGI("Sent %s on channel %s (bridge's own ID)", name, maskText(own).c_str());
    if (!transmitPayload(aok::makePayload(remote.prefix, remote.ownId, own, remote.base[a]), err)) return false;
  }
  return true;
}

// Send one Somfy RTS command from blind i's own address. The rolling code goes up and is saved
// first, so a crash mid-send can't reuse it. `frames` counts the first frame and its repeats.
bool sendSomfy(size_t i, somfy::Command cmd, uint8_t frames, String& err) {
  if (!radioReady) { err = "CC1101 radio not detected"; return false; }
  const uint32_t addr = cfg[i].somfyAddr;
  const uint16_t code = somfyCode(addr);
  setSomfyCode(addr, static_cast<uint16_t>(code + 1));
  uint8_t frame[7];
  somfy::encode(cmd, code, addr, frame);
  uint32_t pulses[somfy::kMaxPulses];
  rxStop();
  ELECHOUSE_cc1101.SetTx(kSomfyMhz);
  pinMode(pins.gdo0, OUTPUT);
  ledSet(true);
  for (uint8_t r = 0; r < frames; ++r) {
    const size_t n = somfy::encodePulses(frame, r == 0, pulses, somfy::kMaxPulses);
    bool level = true;
    for (size_t k = 0; k < n; ++k) {
      digitalWrite(pins.gdo0, level ? HIGH : LOW);
      if (pulses[k] > 16000) delay(pulses[k] / 1000), delayMicroseconds(pulses[k] % 1000);
      else delayMicroseconds(pulses[k]);
      level = !level;
    }
    digitalWrite(pins.gdo0, LOW);
  }
  ledSet(false);
  lastTxMs = millis();
  rxStart();
  stateRev++;
  LOGI("%s: sent Somfy %s (address %s, code %u)", cfg[i].name.c_str(), somfy::commandName(cmd), hex(addr, 6).c_str(), code);
  return true;
}

somfy::Command somfyFor(aok::Action a) {
  return a == aok::Action::Open ? somfy::Command::Up : a == aok::Action::Close ? somfy::Command::Down : somfy::Command::My;
}

// Send an Up/Down/Stop for one blind, whichever kind it is. For Somfy, Stop is "My", which only
// stops a motor that is moving: callers make sure of that (a stopped motor would go to its My position).
bool blindSend(size_t i, aok::Action a, String& err) {
  if (cfg[i].somfy) return sendSomfy(i, somfyFor(a), 4, err);
  return sendAction(cfg[i].mask, a, err);
}

// ---------------------------------------------------------------- position model
uint32_t travelMs(size_t i, Motion m) { return max<uint32_t>(m == Motion::Closing ? cfg[i].closeMs : cfg[i].openMs, 1000); }

int estimate(size_t i) {
  const BlindState& s = st[i];
  if (s.motion == Motion::Stopped || s.startPos < 0) return s.position;
  uint32_t elapsed = millis() - s.startMs;
  elapsed = elapsed > cfg[i].startMs ? elapsed - cfg[i].startMs : 0;  // the motor hasn't moved yet
  int delta = static_cast<int>((static_cast<uint64_t>(elapsed) * 100) / travelMs(i, s.motion));
  if (delta > 100) delta = 100;
  return clampPos(s.startPos + (s.motion == Motion::Opening ? delta : -delta));
}

// physical direction the blind moves for an RF action (some motors are wired reversed)
Motion motionFor(size_t i, aok::Action rf) {
  if (rf == aok::Action::Open) return cfg[i].invert ? Motion::Closing : Motion::Opening;
  if (rf == aok::Action::Close) return cfg[i].invert ? Motion::Opening : Motion::Closing;
  return Motion::Stopped;
}

aok::Action rfFor(size_t i, Motion m) {
  if (m == Motion::Opening) return cfg[i].invert ? aok::Action::Close : aok::Action::Open;
  return cfg[i].invert ? aok::Action::Open : aok::Action::Close;
}

void beginMotion(size_t i, Motion m, int target, bool autoStop, const String& source) {
  BlindState& s = st[i];
  if (s.motion != Motion::Stopped && s.motion != m) s.partials = aok::afterRun(s.partials, false);  // reversed part way
  int cur = estimate(i);
  if (cur < 0) cur = (m == Motion::Opening) ? 0 : 100;  // unknown: assume the far end so the full run is covered
  s.position = cur;
  s.startPos = cur;
  s.target = clampPos(target);
  s.motion = m;
  s.startMs = millis();
  s.durMs = static_cast<uint32_t>(abs(s.target - cur)) * travelMs(i, m) / 100 + cfg[i].startMs;
  if (!autoStop) s.durMs += 1500;  // endpoint runs: motor stops itself at its limit
  s.autoStop = autoStop;
  s.lastSource = source;
  s.dirty = true;
}

// Stopping a run: counts as part way here; a run that reached its end stop resets the count after.
void haltMotion(size_t i, const String& source) {
  BlindState& s = st[i];
  if (s.motion != Motion::Stopped) s.partials = aok::afterRun(s.partials, false);
  const int cur = estimate(i);
  s.position = cur;
  s.startPos = cur;
  s.target = cur;
  s.motion = Motion::Stopped;
  s.autoStop = false;
  s.lastSource = source;
  s.dirty = true;
}

// Apply an RF action heard or sent on `mask` to every blind whose channels are all included.
void applyOne(size_t j, aok::Action action, const String& source, bool commander, int commanderTarget) {
  if (action == aok::Action::Stop) { haltMotion(j, source); return; }
  const Motion m = motionFor(j, action);
  if (commander && commanderTarget > 0 && commanderTarget < 100)
    beginMotion(j, m, commanderTarget, true, source);
  else
    beginMotion(j, m, m == Motion::Opening ? 100 : 0, false, source);
}

void applyAction(uint16_t mask, aok::Action action, const String& source, int commander, int commanderTarget) {
  for (size_t j = 0; j < blindCount; ++j) {
    if (!inMask(j, mask)) continue;
    applyOne(j, action, source, static_cast<int>(j) == commander, commanderTarget);
  }
  requestPositionSave();
}

// What a frame sent for blind i does to the position model: A-OK frames move every blind on
// those channels; a Somfy frame moves just that motor.
void blindApply(size_t i, aok::Action action, const String& source, int commanderTarget) {
  if (!cfg[i].somfy) { applyAction(cfg[i].mask, action, source, static_cast<int>(i), commanderTarget); return; }
  applyOne(i, action, source, true, commanderTarget);
  requestPositionSave();
}

// Blinds that can stop part way: A-OK needs the learned Stop button; Somfy uses My.
bool canStop(size_t i) { return cfg[i].somfy || remote.have[2]; }

bool groupPosition(size_t g, int& pos, Motion& motion);
bool stopBlind(size_t i, const String& source, String& err);

bool moveTo(size_t i, int target, const String& source, String& err) {
  target = clampPos(target);
  if (st[i].interlock && source != "wind") {
    const int now = estimate(i);
    if (now < 0 || target < now) { err = "the window or door is open: it won't close until it's shut"; return false; }
  }
  int gp; Motion gm;
  if (isGroup(i) && groupPosition(i, gp, gm) && gm == Motion::Stopped && st[i].motion == Motion::Stopped) {
    st[i].position = st[i].startPos = gp;  // a group starts from where its members actually are
  }
  int cur = estimate(i);
  st[i].after = -1;
  if (cur < 0 && target > 0 && target < 100) {
    // Position unknown: run to the nearer end first (a reference run), then on to the target.
    if (!canStop(i)) { err = "learn the Stop button first"; return false; }
    const int end = target >= 50 ? 100 : 0;
    if (!moveTo(i, end, source, err)) return false;
    st[i].after = target;
    LOGI("%s: position unknown, running to %s first", cfg[i].name.c_str(), end ? "open" : "closed");
    return true;
  }
  if (cur == target && st[i].motion == Motion::Stopped) { st[i].dirty = true; return true; }
  if (st[i].motion != Motion::Stopped && cur >= 0) {
    const Motion want = target > cur ? Motion::Opening : Motion::Closing;
    if (want != st[i].motion) {
      // Reversing a running tubular motor in one step strains it: stop, pause, then go.
      if (!stopBlind(i, source, err)) return false;
      st[i].pending = target;
      st[i].pendingAtMs = millis() + kReversePauseMs;
      st[i].lastSource = source;
      return true;
    }
  }
  st[i].pending = -1;
  // Drift correction: after enough part-way runs, re-sync at an end stop (see aok_drift.h).
  const aok::Resync plan = aok::resyncPlan(st[i].partials, net.resyncEvery, target, net.resyncThrough);
  if (plan == aok::Resync::ViaEnd && cur >= 0 && st[i].motion == Motion::Stopped && !st[i].interlock && canStop(i)) {
    const int end = aok::resyncEnd(cur, target);
    if (cur != end) {
      if (!moveTo(i, end, source, err)) return false;
      st[i].after = target;
      LOGI("%s: %u part-way moves, re-syncing at %s on the way", cfg[i].name.c_str(), static_cast<unsigned>(st[i].partials), end ? "open" : "closed");
      return true;
    }
  }
  if (cur < 0) cur = target == 100 ? 0 : 100;
  const Motion m = target > cur ? Motion::Opening : Motion::Closing;
  if (target > 0 && target < 100 && !canStop(i)) { err = "learn the Stop button first"; return false; }
  if (!blindSend(i, rfFor(i, m), err)) return false;
  blindApply(i, rfFor(i, m), source, target);
  if (plan == aok::Resync::AtEnd && !st[i].autoStop) {
    // The estimate may be off, so give the motor the whole travel time to reach its limit
    // before the position counts as the end (and before any move that follows).
    st[i].durMs = travelMs(i, st[i].motion) + cfg[i].startMs + 1500;
    LOGI("%s: %u part-way moves, re-syncing at the end stop", cfg[i].name.c_str(), static_cast<unsigned>(st[i].partials));
  }
  return true;
}

bool stopBlind(size_t i, const String& source, String& err) {
  st[i].pending = -1;
  st[i].after = -1;
  if (cfg[i].somfy && st[i].motion == Motion::Stopped) {
    // My on a motor that isn't moving sends it to its My position, so nothing is sent.
    haltMotion(i, source);
    return true;
  }
  if (!blindSend(i, aok::Action::Stop, err)) return false;
  blindApply(i, aok::Action::Stop, source, -1);
  return true;
}

// Somfy "My" position: the motor goes there by itself when it's stopped and gets My.
bool somfyFavourite(size_t i, const String& source, String& err) {
  if (st[i].motion != Motion::Stopped) return moveTo(i, cfg[i].favourite, source, err);
  if (!sendSomfy(i, somfy::Command::My, 4, err)) return false;
  int cur = estimate(i);
  if (cur == cfg[i].favourite) return true;
  if (cur < 0) cur = cfg[i].favourite >= 50 ? 0 : 100;
  st[i].position = st[i].startPos = cur;
  beginMotion(i, cfg[i].favourite > cur ? Motion::Opening : Motion::Closing, cfg[i].favourite, false, source);
  requestPositionSave();
  return true;
}

// members of a group = other blinds whose channels are a subset of the group's channels
bool groupPosition(size_t g, int& pos, Motion& motion) {
  int sum = 0, count = 0;
  bool opening = false, closing = false;
  for (size_t j = 0; j < blindCount; ++j) {
    if (j == g || !inMask(j, cfg[g].mask) || cfg[j].mask == cfg[g].mask) continue;
    const int p = estimate(j);
    if (p < 0) return false;
    sum += p; count++;
    opening |= st[j].motion == Motion::Opening;
    closing |= st[j].motion == Motion::Closing;
  }
  if (!count) return false;
  pos = (sum + count / 2) / count;
  motion = opening && !closing ? Motion::Opening : (closing && !opening ? Motion::Closing : (opening ? Motion::Opening : Motion::Stopped));
  return true;
}

void displayState(size_t i, int& pos, Motion& motion) {
  pos = estimate(i);
  motion = st[i].motion;
  int gp; Motion gm;
  if (isGroup(i) && groupPosition(i, gp, gm)) { pos = gp; motion = gm; }
}

// ---------------------------------------------------------------- MQTT
String topic(const String& suffix) { return net.base + "/" + suffix; }
String blindTopic(size_t i, const char* suffix) { return net.base + "/" + cfg[i].id + "/" + suffix; }
String nodeId() { return net.base; }

// Home Assistant device discovery: one message per blind (each blind is its own device, so it can
// sit in its own area) and one for the bridge, with the entities under "cmps". The unique_ids are
// the ones the per-entity messages of 1.8.0 and earlier used, so entities keep their names and history.
JsonObject deviceDoc(JsonDocument& d) {
  JsonObject dev = d["dev"].to<JsonObject>();
  dev["identifiers"].to<JsonArray>().add(nodeId());
  dev["name"] = net.brand + " Blind Bridge";
  dev["manufacturer"] = "aok-blind-bridge";
  dev["model"] = "ESP32 + CC1101";
  dev["sw_version"] = FW_VERSION;
  if (netUp()) dev["configuration_url"] = String("http://") + netIp().toString();
  JsonObject origin = d["o"].to<JsonObject>();
  origin["name"] = "aok-blind-bridge"; origin["sw_version"] = FW_VERSION; origin["support_url"] = PROJECT_URL;
  return d["cmps"].to<JsonObject>();
}

// One entity. Home Assistant refuses a component with both availability keys; the cover brings its own list.
JsonObject component(JsonObject cmps, const String& key, const char* platform, bool bridgeAvailability = true) {
  JsonObject c = cmps[key].to<JsonObject>();
  c["p"] = platform;
  if (bridgeAvailability) c["availability_topic"] = topic("status");
  return c;
}

// A component with only "p" removes that entity from Home Assistant.
void removedComponent(JsonObject cmps, const String& key, const char* platform) { cmps[key].to<JsonObject>()["p"] = platform; }

bool publishJson(const String& t, JsonDocument& d, bool retain = true) {
  String s; serializeJson(d, s);
  const bool ok = mqtt.publish(t.c_str(), s.c_str(), retain);
  if (!ok) LOGW("MQTT: couldn't publish %s (%u bytes)", t.c_str(), static_cast<unsigned>(s.length()));
  return ok;
}

// The per-entity discovery topics 1.8.0 and earlier published, for moving them to device discovery.
std::vector<String> legacyDiscoveryTopics() {
  const String disc = net.discovery, node = nodeId();
  std::vector<String> t;
  for (size_t i = 0; i < blindCount; ++i) {
    const String uid = node + "_" + cfg[i].id;
    t.push_back(disc + "/cover/" + uid + "/config");
    if (cfg[i].favourite >= 0) t.push_back(disc + "/button/" + uid + "_favourite/config");
    t.push_back(disc + "/button/" + uid + "_mark_open/config");
    t.push_back(disc + "/button/" + uid + "_mark_closed/config");
    t.push_back(disc + "/switch/" + uid + "_lock/config");
    for (int k = 0; k < 4; ++k) if (cfg[i].presetPos[k] >= 0) t.push_back(disc + "/button/" + uid + "_preset" + String(k + 1) + "/config");
  }
  for (const char* k : {"rssi", "uptime", "ip", "last_remote", "radio_rssi"}) t.push_back(disc + "/sensor/" + node + "_" + k + "/config");
  t.push_back(disc + "/binary_sensor/" + node + "_radio/config");
  t.push_back(disc + "/event/" + node + "_remote_event/config");
  t.push_back(disc + "/button/" + node + "_learn/config");
  for (size_t k = 0; k < sceneCount; ++k) t.push_back(disc + "/scene/" + node + "_scene_" + scenes[k].id + "/config");
  return t;
}

uint32_t migrateAtMs = 0;  // moving from per-entity discovery: when the device messages go out

void publishDiscovery() {
  const String disc = net.discovery;
  // Once, after upgrading from per-entity discovery: tell Home Assistant to keep the entities
  // (migrate_discovery), give it a moment to let go of them, publish the device messages, then
  // clear the old topics. Done in this order nothing is orphaned or created twice.
  const bool migrating = !prefs.getBool("devdisc", false) && prefs.isKey("pubids");
  if (migrating) {
    if (!migrateAtMs) {
      for (const String& t : legacyDiscoveryTopics()) mqtt.publish(t.c_str(), "{\"migrate_discovery\":true}", true);
      LOGI("Home Assistant: moving to device discovery");
      migrateAtMs = millis() + 3000;
      return;  // discoveryPending stays set; loop() comes back
    }
    if (static_cast<int32_t>(millis() - migrateAtMs) < 0) return;
  }
  JsonDocument ids;
  for (size_t i = 0; i < blindCount; ++i) {
    const String uid = nodeId() + "_" + cfg[i].id;
    JsonDocument d;
    JsonObject cmps = deviceDoc(d);
    JsonObject dev = d["dev"];
    dev["identifiers"][0] = uid;
    dev["name"] = cfg[i].name;
    dev["via_device"] = nodeId();
    dev["model"] = cfg[i].somfy ? String("Somfy RTS motor") : net.brand + " motor · channel " + maskText(cfg[i].mask);
    if (cfg[i].room.length()) dev["suggested_area"] = cfg[i].room;
    {
      JsonObject c = component(cmps, "cover", "cover", false);
      c["name"] = nullptr;  // entity takes the device name
      c["unique_id"] = uid;
      c["default_entity_id"] = String("cover.") + cfg[i].id;
      c["command_topic"] = blindTopic(i, "set");
      c["set_position_topic"] = blindTopic(i, "set_position");
      c["position_topic"] = blindTopic(i, "position");
      c["state_topic"] = blindTopic(i, "state");
      c["json_attributes_topic"] = blindTopic(i, "attributes");
      c["payload_open"] = "OPEN"; c["payload_close"] = "CLOSE"; c["payload_stop"] = "STOP";
      c["position_open"] = 100; c["position_closed"] = 0;
      c["optimistic"] = false;
      c["device_class"] = cfg[i].deviceClass;
      JsonArray av = c["availability"].to<JsonArray>();
      av.add<JsonObject>()["topic"] = topic("status");
      av.add<JsonObject>()["topic"] = blindTopic(i, "available");
      c["availability_mode"] = "all";
      if (cfg[i].tiltMs) {
        c["tilt_command_topic"] = blindTopic(i, "tilt/set");
        c["tilt_status_topic"] = blindTopic(i, "tilt");
        c["tilt_min"] = 0; c["tilt_max"] = 100;
      }
    }
    struct Btn { const char* key; const char* name; const char* payload; const char* icon; };
    const Btn btns[] = {
      {"favourite", "Favourite position", "FAVOURITE", "mdi:star"},
      {"mark_open", "Mark as open", "SYNC_OPEN", "mdi:arrow-collapse-up"},
      {"mark_closed", "Mark as closed", "SYNC_CLOSE", "mdi:arrow-collapse-down"},
    };
    for (const Btn& b : btns) {
      if (strcmp(b.key, "favourite") == 0 && cfg[i].favourite < 0) { removedComponent(cmps, b.key, "button"); continue; }
      JsonObject c = component(cmps, b.key, "button");
      c["name"] = b.name; c["unique_id"] = uid + "_" + b.key; c["command_topic"] = blindTopic(i, "set");
      c["payload_press"] = b.payload; c["icon"] = b.icon;
      if (strcmp(b.key, "favourite") != 0) c["entity_category"] = "config";
    }
    {  // lock switch: Home Assistant, scenes and schedules leave a locked blind alone
      JsonObject c = component(cmps, "lock", "switch");
      c["name"] = "Lock"; c["unique_id"] = uid + "_lock"; c["icon"] = "mdi:lock";
      c["command_topic"] = blindTopic(i, "lock/set"); c["state_topic"] = blindTopic(i, "lock");
      c["payload_on"] = "ON"; c["payload_off"] = "OFF"; c["entity_category"] = "config";
    }
    for (int k = 0; k < 4; ++k) {  // one button per named position
      const String key = String("preset") + String(k + 1);
      if (cfg[i].presetPos[k] < 0) { removedComponent(cmps, key, "button"); continue; }
      JsonObject c = component(cmps, key, "button");
      c["name"] = cfg[i].presetName[k]; c["unique_id"] = uid + "_" + key; c["icon"] = "mdi:blinds-horizontal";
      c["command_topic"] = blindTopic(i, "set"); c["payload_press"] = String("PRESET:") + String(k + 1);
    }
    publishJson(disc + "/device/" + uid + "/config", d);
    ids.add(cfg[i].id);
  }
  // remove blinds that were deleted: an empty device message removes the device and its entities
  JsonDocument old;
  if (!deserializeJson(old, prefs.getString("pubids", "[]"))) {
    for (JsonVariantConst v : old.as<JsonArrayConst>()) {
      const String id = v.as<String>();
      if (findBlind(id) >= 0) continue;
      const String uid = nodeId() + "_" + id;
      mqtt.publish((disc + "/device/" + uid + "/config").c_str(), "", true);
      // and the per-entity topics, in case the blind went before the move to device discovery
      mqtt.publish((disc + "/cover/" + uid + "/config").c_str(), "", true);
      for (const char* k : {"favourite", "mark_open", "mark_closed", "preset1", "preset2", "preset3", "preset4"})
        mqtt.publish((disc + "/button/" + uid + "_" + k + "/config").c_str(), "", true);
      mqtt.publish((disc + "/switch/" + uid + "_lock/config").c_str(), "", true);
      for (const char* s : {"state", "position", "attributes", "set", "set_position"})
        mqtt.publish((net.base + "/" + id + "/" + s).c_str(), "", true);
    }
  }
  String s; serializeJson(ids, s); prefs.putString("pubids", s);

  // bridge entities and scenes
  JsonDocument d;
  JsonObject cmps = deviceDoc(d);
  auto sensor = [&](const char* key, const char* name, const char* unit, const char* devClass, const char* icon, bool diag) {
    JsonObject c = component(cmps, key, "sensor");
    c["name"] = name; c["unique_id"] = nodeId() + "_" + key; c["state_topic"] = topic(String("diag/") + key);
    if (unit) c["unit_of_measurement"] = unit;
    if (devClass) c["device_class"] = devClass;
    if (icon) c["icon"] = icon;
    if (diag) c["entity_category"] = "diagnostic";
  };
  sensor("rssi", "Wi-Fi signal", "dBm", "signal_strength", nullptr, true);
  sensor("uptime", "Uptime", "s", "duration", nullptr, true);
  sensor("ip", "IP address", nullptr, nullptr, "mdi:ip-network", true);
  sensor("last_remote", "Last remote press", nullptr, nullptr, "mdi:remote", false);
  sensor("radio_rssi", "Remote signal", "dBm", "signal_strength", nullptr, true);
  {
    JsonObject c = component(cmps, "radio", "binary_sensor");
    c["name"] = "CC1101 radio"; c["unique_id"] = nodeId() + "_radio"; c["state_topic"] = topic("diag/radio");
    c["payload_on"] = "ON"; c["payload_off"] = "OFF"; c["device_class"] = "connectivity"; c["entity_category"] = "diagnostic";
  }
  {
    JsonObject c = component(cmps, "remote_event", "event");
    c["name"] = "Remote button"; c["unique_id"] = nodeId() + "_remote_event"; c["state_topic"] = topic("remote/event");
    JsonArray et = c["event_types"].to<JsonArray>();
    et.add("open"); et.add("close"); et.add("stop");
    et.add("open_long"); et.add("close_long"); et.add("stop_long");
    c["icon"] = "mdi:remote";
  }
  {
    JsonObject c = component(cmps, "learn", "button");
    c["name"] = "Learn remote"; c["unique_id"] = nodeId() + "_learn"; c["command_topic"] = topic("learn/set");
    c["payload_press"] = "START"; c["icon"] = "mdi:school"; c["entity_category"] = "config";
  }
  JsonDocument sceneIds;
  for (size_t k = 0; k < sceneCount; ++k) {  // one Home Assistant scene each
    JsonObject c = component(cmps, String("scene_") + scenes[k].id, "scene");
    c["name"] = scenes[k].name; c["unique_id"] = nodeId() + "_scene_" + scenes[k].id; c["icon"] = "mdi:blinds";
    c["command_topic"] = topic("scene/set"); c["payload_on"] = scenes[k].id;
    sceneIds.add(scenes[k].id);
  }
  JsonDocument oldScenes;
  if (!deserializeJson(oldScenes, prefs.getString("pubscenes", "[]"))) {
    for (JsonVariantConst v : oldScenes.as<JsonArrayConst>()) {
      bool still = false;
      for (size_t k = 0; k < sceneCount; ++k) still |= scenes[k].id == v.as<String>();
      if (still) continue;
      removedComponent(cmps, String("scene_") + v.as<String>(), "scene");
      mqtt.publish((disc + "/scene/" + nodeId() + "_scene_" + v.as<String>() + "/config").c_str(), "", true);
    }
  }
  publishJson(disc + "/device/" + nodeId() + "/config", d);
  { String ss; serializeJson(sceneIds, ss); prefs.putString("pubscenes", ss); }
  if (migrating) {
    for (const String& t : legacyDiscoveryTopics()) mqtt.publish(t.c_str(), "", true);
    LOGI("Home Assistant: now on device discovery");
  }
  if (!prefs.getBool("devdisc", false)) prefs.putBool("devdisc", true);
  migrateAtMs = 0;
  discoveryPending = false;
}

void publishBlind(size_t i) {
  if (!mqtt.connected()) return;
  int pos; Motion m;
  displayState(i, pos, m);
  const char* state = m == Motion::Opening ? "opening" : m == Motion::Closing ? "closing" : (pos < 0 ? "stopped" : (pos == 0 ? "closed" : "open"));
  mqtt.publish(blindTopic(i, "state").c_str(), state, true);
  if (pos >= 0) mqtt.publish(blindTopic(i, "position").c_str(), String(pos).c_str(), true);
  JsonDocument d;
  if (cfg[i].somfy) { d["protocol"] = "somfy_rts"; d["somfy_address"] = hex(cfg[i].somfyAddr, 6); }
  else d["channels"] = maskText(cfg[i].mask);
  d["position_known"] = pos >= 0;
  d["target_position"] = st[i].target;
  d["last_source"] = st[i].lastSource;
  d["open_time_s"] = cfg[i].openMs / 1000.0;
  d["close_time_s"] = cfg[i].closeMs / 1000.0;
  d["favourite_position"] = cfg[i].favourite;
  d["inverted"] = cfg[i].invert;
  d["locked"] = st[i].locked;
  d["window_open"] = st[i].interlock;
  d["wind_retracted"] = st[i].windLocked;
  d["partial_moves"] = st[i].partials;
  JsonArray pr = d["presets"].to<JsonArray>();
  for (int k = 0; k < 4; ++k) if (cfg[i].presetPos[k] >= 0) pr.add(cfg[i].presetName[k] + " " + String(cfg[i].presetPos[k]) + "%");
  mqtt.publish(blindTopic(i, "lock").c_str(), st[i].locked ? "ON" : "OFF", true);
  if (cfg[i].tiltMs && st[i].tilt >= 0) mqtt.publish(blindTopic(i, "tilt").c_str(), String(st[i].tilt).c_str(), true);
  mqtt.publish(blindTopic(i, "available").c_str(), (radioReady && (cfg[i].somfy || remoteComplete())) ? "online" : "offline", true);
  publishJson(blindTopic(i, "attributes"), d);
  st[i].lastPublishMs = millis();
  st[i].dirty = false;
}

void publishDiagnostics() {
  if (!mqtt.connected()) return;
  mqtt.publish(topic("diag/rssi").c_str(), String(WiFi.RSSI()).c_str(), true);
  mqtt.publish(topic("diag/uptime").c_str(), String(millis() / 1000).c_str(), true);
  mqtt.publish(topic("diag/ip").c_str(), netIp().toString().c_str(), true);
  mqtt.publish(topic("diag/radio").c_str(), radioReady ? "ON" : "OFF", true);
  mqtt.publish(topic("diag/last_remote").c_str(), lastEventText.c_str(), true);
  if (lastRssi) mqtt.publish(topic("diag/radio_rssi").c_str(), String(lastRssi).c_str(), true);
  lastDiagMs = millis();
}

bool handleBlindCommand(size_t i, String cmd, const String& source, String& err);

// Up, Down or Stop for every blind at once: one radio frame with all their channels.
bool allBlinds(String cmd, const String& source, String& err) {
  cmd.trim(); cmd.toUpperCase();
  const aok::Action a = cmd == "OPEN" ? aok::Action::Open : cmd == "CLOSE" ? aok::Action::Close : cmd == "STOP" ? aok::Action::Stop : aok::Action::Unknown;
  if (a == aok::Action::Unknown) { err = "all blinds accept OPEN, CLOSE or STOP"; return false; }
  // reversed blinds need the opposite command, so they go in a second frame
  uint16_t normal = 0, reversed = 0;
  for (size_t i = 0; i < blindCount; ++i) (cfg[i].invert ? reversed : normal) |= cfg[i].mask;
  reversed &= ~normal;
  // Locked blinds, awnings held for wind and (when closing) blinds with the window open sit this one out.
  uint16_t held = 0;
  if (a != aok::Action::Stop)
    for (size_t i = 0; i < blindCount; ++i)
      if ((st[i].locked && source != "wind") || st[i].windLocked || (a == aok::Action::Close && st[i].interlock)) held |= cfg[i].mask;
  normal &= ~held; reversed &= ~held;
  if (held) LOGI("All blinds: leaving out channel %s (locked, wind or window open)", maskText(held).c_str());
  // Somfy motors each get their own frame
  bool somfyMoved = false, somfyHeld = false;
  for (size_t i = 0; i < blindCount; ++i) {
    if (!cfg[i].somfy) continue;
    if (a != aok::Action::Stop && ((st[i].locked && source != "wind") || st[i].windLocked || (a == aok::Action::Close && st[i].interlock))) { somfyHeld = true; continue; }
    String e;
    const bool ok = a == aok::Action::Stop ? stopBlind(i, source, e) : moveTo(i, a == aok::Action::Open ? 100 : 0, source, e);
    if (ok) somfyMoved = true; else LOGW("%s: %s", cfg[i].name.c_str(), e.c_str());
  }
  if (!normal && !reversed) {
    if (somfyMoved) { LOGI("All blinds: %s (%s)", cmd.c_str(), source.c_str()); return true; }
    err = (held || somfyHeld) ? "every blind is locked or held" : "no blinds set up";
    return false;
  }
  const aok::Action flip = a == aok::Action::Open ? aok::Action::Close : a == aok::Action::Close ? aok::Action::Open : a;
  if (normal) { if (!sendAction(normal, a, err)) return false; applyAction(normal, a, source, -1, -1); }
  if (reversed) { if (!sendAction(reversed, flip, err)) return false; applyAction(reversed, flip, source, -1, -1); }
  LOGI("All blinds: %s (%s)", cmd.c_str(), source.c_str());
  return true;
}

void onMqtt(char* t, byte* payload, unsigned int len) {
  String tp(t), msg;
  msg.reserve(len);
  for (unsigned int k = 0; k < len; ++k) msg += static_cast<char>(payload[k]);
  if (!msg.length()) return;
  if (tp == topic("learn/set")) { learnMode = true; learnLocked = false; learnStartedMs = millis(); learnStatus = "waiting"; return; }
  if (tp == topic("all/set")) { String err; if (!allBlinds(msg, "home_assistant", err)) LOGW("All blinds: %s", err.c_str()); return; }
  if (tp == topic("scene/set")) { String err; if (!runScene(msg, "home_assistant", err)) LOGW("Scene %s: %s", msg.c_str(), err.c_str()); return; }
  if (tp == topic("channels/set")) { String err; if (!channelCommand(msg, "home_assistant", err)) LOGW("Channels %s: %s", msg.c_str(), err.c_str()); return; }
  if (tp == topic("wind")) { setWind(msg.equalsIgnoreCase("ON") || msg == "1" || msg.equalsIgnoreCase("true")); return; }
  for (size_t i = 0; i < blindCount; ++i) {
    if (tp == blindTopic(i, "tilt/set")) { String err; if (!handleBlindCommand(i, String("TILT:") + msg, "home_assistant", err)) LOGW("%s tilt: %s", cfg[i].name.c_str(), err.c_str()); return; }
    if (tp == blindTopic(i, "lock/set")) { setLock(i, msg.equalsIgnoreCase("ON") || msg.equalsIgnoreCase("LOCK")); return; }
    if (tp == blindTopic(i, "interlock")) {
      const bool open = msg.equalsIgnoreCase("ON") || msg.equalsIgnoreCase("open") || msg == "1";
      if (open != st[i].interlock) {
        st[i].interlock = open;
        st[i].dirty = true;
        LOGI("%s: window %s", cfg[i].name.c_str(), open ? "open, closing blocked" : "shut");
        String err;
        if (open && st[i].motion == Motion::Closing) stopBlind(i, "interlock", err);
      }
      return;
    }
  }
  for (size_t i = 0; i < blindCount; ++i) {
    if (tp == blindTopic(i, "set") || tp == blindTopic(i, "set_position")) {
      String err;
      LOGI("Home Assistant: %s → %s", cfg[i].name.c_str(), msg.c_str());
      if (!handleBlindCommand(i, msg, "home_assistant", err)) LOGW("%s: %s", cfg[i].name.c_str(), err.c_str());
      return;
    }
  }
}

void ensureMqtt() {
  if (safeMode) return;
  if (apMode && WiFi.status() != WL_CONNECTED) return;
  if (WiFi.status() != WL_CONNECTED || !net.mqttHost.length() || mqtt.connected()) return;
  if (lastMqttTryMs && millis() - lastMqttTryMs < mqttRetryMs) return;
  // Connecting can block for seconds when the broker is down; a blind waiting for its
  // timed Stop must never wait behind it.
  if (anyMoving()) return;
  lastMqttTryMs = millis();
  mqtt.setSocketTimeout(2);
  wifiClient.setTimeout(2);  // the TCP connect itself, not only the MQTT reply
  mqtt.setServer(net.mqttHost.c_str(), net.mqttPort);
  const String clientId = net.base + "_" + String(static_cast<uint32_t>(ESP.getEfuseMac()), HEX);
  const String will = topic("status");
  bool ok = net.mqttUser.length()
    ? mqtt.connect(clientId.c_str(), net.mqttUser.c_str(), net.mqttPass.c_str(), will.c_str(), 0, true, "offline")
    : mqtt.connect(clientId.c_str(), will.c_str(), 0, true, "offline");
  if (!ok) {
    const int st = mqtt.state();
    const char* why = st == 5 ? "username or password rejected" : st == 4 ? "bad username or password" : st == -2 ? "broker not reachable" : st == -4 ? "connection timed out" : "connection failed";
    LOGW("MQTT %s:%u: %s (code %d), next try in %u s", net.mqttHost.c_str(), net.mqttPort, why, st, mqttRetryMs / 1000);
    mqttRetryMs = min<uint32_t>(mqttRetryMs * 2, 60000);
    return;
  }
  mqttRetryMs = 5000;
  LOGI("MQTT connected to %s:%u", net.mqttHost.c_str(), net.mqttPort);
  mqtt.publish(will.c_str(), "online", true);
  mqtt.subscribe(topic("learn/set").c_str());
  mqtt.subscribe(topic("all/set").c_str());
  mqtt.subscribe(topic("scene/set").c_str());
  mqtt.subscribe(topic("channels/set").c_str());
  mqtt.subscribe(topic("wind").c_str());
  for (size_t i = 0; i < blindCount; ++i) {
    mqtt.subscribe(blindTopic(i, "set").c_str());
    mqtt.subscribe(blindTopic(i, "set_position").c_str());
    mqtt.subscribe(blindTopic(i, "lock/set").c_str());
    if (cfg[i].tiltMs) mqtt.subscribe(blindTopic(i, "tilt/set").c_str());
    mqtt.subscribe(blindTopic(i, "interlock").c_str());
  }
  publishDiscovery();
  for (size_t i = 0; i < blindCount; ++i) publishBlind(i);
  publishDiagnostics();
}

// ---------------------------------------------------------------- commands
bool handleBlindCommand(size_t i, String cmd, const String& source, String& err) {
  cmd.trim();
  String arg;
  if (cmd.indexOf(':') > 0) { arg = cmd.substring(cmd.indexOf(':') + 1); arg.trim(); cmd = cmd.substring(0, cmd.indexOf(':')); }
  cmd.toUpperCase();
  if (cmd == "LOCK" || cmd == "UNLOCK") { setLock(i, cmd == "LOCK"); return true; }
  const bool moving = cmd != "STOP" && cmd != "SYNC_OPEN" && cmd != "SYNC_CLOSE";
  if (moving && st[i].windLocked && source != "wind") { err = "retracted for wind: it moves again when the wind drops"; return false; }
  if (moving && st[i].locked && source != "wind") { err = "locked: unlock it first"; return false; }
  if (cmd == "TILT") {
    // Turn the slats: a short run in the right direction, timed from tilt_ms, then Stop.
    if (!cfg[i].tiltMs) { err = "this blind has no tilt time set"; return false; }
    const int want = clampPos(arg.toInt());
    const int cur = st[i].tilt < 0 ? (want >= 50 ? 0 : 100) : st[i].tilt;
    const int pos = estimate(i);
    if (pos < 0) { err = "position unknown: open or close it fully once"; return false; }
    const int dpos = static_cast<int>(lround((want - cur) * static_cast<double>(cfg[i].tiltMs) / travelMs(i, want > cur ? Motion::Opening : Motion::Closing)));
    st[i].tilt = want;
    st[i].dirty = true;
    if (!dpos) return true;
    return moveTo(i, clampPos(pos + dpos), source, err);
  }
  if (cmd == "PRESET") {
    for (int k = 0; k < 4; ++k) {
      if (cfg[i].presetPos[k] < 0) continue;
      if (arg == String(k + 1) || arg.equalsIgnoreCase(cfg[i].presetName[k])) return handleBlindCommand(i, String(cfg[i].presetPos[k]), source, err);
    }
    err = String("no preset called ") + arg;
    return false;
  }
  if (cmd == "OPEN") return moveTo(i, 100, source, err);
  if (cmd == "CLOSE") return moveTo(i, 0, source, err);
  if (cmd == "STOP") return stopBlind(i, source, err);
  if (cmd == "FAVOURITE" || cmd == "FAVORITE") {
    if (cfg[i].favourite < 0) { err = "no favourite position set"; return false; }
    if (cfg[i].somfy) return somfyFavourite(i, source, err);
    return moveTo(i, cfg[i].favourite, source, err);
  }
  if (cmd == "SYNC_OPEN" || cmd == "SYNC_CLOSE") {
    const int p = cmd == "SYNC_OPEN" ? 100 : 0;
    for (size_t j = 0; j < blindCount; ++j) {
      if (j != i && !inMask(j, cfg[i].mask)) continue;
      st[j].motion = Motion::Stopped;
      st[j].position = st[j].startPos = st[j].target = p;
      st[j].lastSource = "marked";
      st[j].partials = 0;
      st[j].dirty = true;
    }
    requestPositionSave();
    return true;
  }
  bool numeric = cmd.length() > 0;
  for (char c : cmd) numeric &= isdigit(static_cast<unsigned char>(c)) != 0;
  if (numeric) return moveTo(i, cmd.toInt(), source, err);
  err = String("unknown command: ") + cmd;
  return false;
}

bool anyMoving() {
  for (size_t i = 0; i < blindCount; ++i)
    if (st[i].motion != Motion::Stopped || st[i].pending >= 0) return true;
  return false;
}

void updateMotion() {
  const uint32_t now = millis();
  bool anyStopped = false;
  for (size_t i = 0; i < blindCount; ++i) {  // reversals waiting out their pause
    BlindState& s = st[i];
    if (s.pending < 0 || static_cast<int32_t>(now - s.pendingAtMs) < 0) continue;
    const int target = s.pending;
    s.pending = -1;
    String err;
    if (!moveTo(i, target, s.lastSource, err)) LOGW("%s: %s", cfg[i].name.c_str(), err.c_str());
  }
  for (size_t i = 0; i < blindCount; ++i) {
    BlindState& s = st[i];
    if (s.motion == Motion::Stopped) continue;
    if (now - s.lastPublishMs > 1000) s.dirty = true;
    if (now - s.startMs < s.durMs) continue;
    const int goal = s.target;
    if (s.autoStop) {
      String err;
      if (blindSend(i, aok::Action::Stop, err)) {
        blindApply(i, aok::Action::Stop, s.lastSource, -1);
        s.position = s.startPos = s.target = goal;
      } else {
        haltMotion(i, "auto_stop_failed");  // keep the time-based estimate
      }
    } else {
      haltMotion(i, s.lastSource);
      s.position = s.startPos = s.target = goal;  // endpoint reached: motor stops at its limit
      if (goal == 0 || goal == 100) s.partials = 0;  // re-synced (haltMotion counted it as part way)
      if (s.after >= 0) {  // reference run finished: now go where we were asked to
        const int next = s.after;
        s.after = -1;
        String err;
        if (!moveTo(i, next, s.lastSource, err)) LOGW("%s: %s", cfg[i].name.c_str(), err.c_str());
      }
    }
    s.dirty = true;
    anyStopped = true;
  }
  if (anyStopped) requestPositionSave();
  // groups follow their members
  for (size_t i = 0; i < blindCount; ++i) {
    if (!isGroup(i)) continue;
    for (size_t j = 0; j < blindCount; ++j)
      if (j != i && inMask(j, cfg[i].mask) && st[j].dirty) { st[i].dirty = true; break; }
  }
  for (size_t i = 0; i < blindCount; ++i) if (st[i].dirty) publishBlind(i);
}


// ---------------------------------------------------------------- locks, wind, scenes, schedules
bool remoteKnown(uint32_t prefix, uint8_t id) {
  for (uint8_t k = 0; k < remote.extraCount; ++k)
    if (remote.extraPrefix[k] == prefix && remote.extraId[k] == id) return true;
  return false;
}

void saveLocks() {
  JsonDocument d; JsonArray a = d.to<JsonArray>();
  for (size_t i = 0; i < blindCount; ++i) if (st[i].locked) a.add(cfg[i].id);
  String s; serializeJson(d, s); prefs.putString("locks", s);
}

void loadLocks() {
  JsonDocument d;
  if (deserializeJson(d, prefs.getString("locks", "[]"))) return;
  for (JsonVariantConst v : d.as<JsonArrayConst>()) { const int i = findBlind(v.as<String>()); if (i >= 0) st[i].locked = true; }
}

void setLock(size_t i, bool locked, bool save) {
  if (st[i].locked == locked) { st[i].dirty = true; return; }
  st[i].locked = locked;
  st[i].dirty = true;
  LOGI("%s %s", cfg[i].name.c_str(), locked ? "locked" : "unlocked");
  if (save) saveLocks();
}

// Wind on: retract every awning (close it) and keep it there until the wind drops.
void setWind(bool on) {
  if (on == windActive) return;
  windActive = on;
  LOGI("Wind %s", on ? "alarm: retracting awnings" : "has dropped: awnings released");
  for (size_t i = 0; i < blindCount; ++i) {
    if (cfg[i].deviceClass != "awning") continue;
    st[i].windLocked = on;
    st[i].dirty = true;
    if (on) { String err; if (!handleBlindCommand(i, "CLOSE", "wind", err)) LOGW("%s: %s", cfg[i].name.c_str(), err.c_str()); }
  }
  stateRev++;
}

// "1,3:OPEN", "2:STOP" or "1-4:CLOSE": any channels in one radio frame.
bool channelCommand(const String& payload, const String& source, String& err) {
  const int colon = payload.indexOf(':');
  if (colon <= 0) { err = "use channels:command, for example 1,3:OPEN"; return false; }
  String chans = payload.substring(0, colon);
  String cmd = payload.substring(colon + 1); cmd.trim(); cmd.toUpperCase();
  uint16_t mask = 0;
  const int dash = chans.indexOf('-');
  if (dash > 0) {
    const int a = chans.substring(0, dash).toInt(), b = chans.substring(dash + 1).toInt();
    for (int c = max(1, a); c <= min(16, b); ++c) mask |= 1u << (c - 1);
  } else {
    mask = parseChannels(chans);
  }
  if (!mask) { err = "no channels (1-16) given"; return false; }
  const aok::Action a = cmd == "OPEN" || cmd == "UP" ? aok::Action::Open : cmd == "CLOSE" || cmd == "DOWN" ? aok::Action::Close
                       : cmd == "STOP" ? aok::Action::Stop : aok::Action::Unknown;
  if (a == aok::Action::Unknown) { err = "command must be OPEN, CLOSE or STOP"; return false; }
  for (size_t i = 0; i < blindCount; ++i)
    if (inMask(i, mask) && a != aok::Action::Stop && (st[i].locked || st[i].windLocked)) {
      err = cfg[i].name + " is locked"; return false;
    }
  if (!sendAction(mask, a, err)) return false;
  applyAction(mask, a, source, -1, -1);
  return true;
}

int findScene(const String& key) {
  for (size_t k = 0; k < sceneCount; ++k)
    if (scenes[k].id == key || scenes[k].name.equalsIgnoreCase(key)) return static_cast<int>(k);
  return -1;
}

bool runScene(const String& key, const String& source, String& err) {
  String k = key; k.trim();
  const int s = findScene(k);
  if (s < 0) { err = String("no scene called ") + k; return false; }
  LOGI("Scene %s (%s)", scenes[s].name.c_str(), source.c_str());
  bool any = false;
  for (uint8_t n = 0; n < scenes[s].count; ++n) {
    const int i = findBlind(scenes[s].items[n].id);
    if (i < 0) continue;
    String e;
    if (handleBlindCommand(i, String(scenes[s].items[n].position), source, e)) any = true;
    else LOGW("Scene %s, %s: %s", scenes[s].name.c_str(), cfg[i].name.c_str(), e.c_str());
  }
  if (!any) { err = "no blind in the scene could move"; return false; }
  return true;
}

void automationToJson(JsonObject o) {
  if (!isnan(siteLat)) { o["latitude"] = siteLat; o["longitude"] = siteLon; }
  JsonArray sc = o["scenes"].to<JsonArray>();
  for (size_t k = 0; k < sceneCount; ++k) {
    JsonObject x = sc.add<JsonObject>();
    x["id"] = scenes[k].id; x["name"] = scenes[k].name;
    JsonArray it = x["blinds"].to<JsonArray>();
    for (uint8_t n = 0; n < scenes[k].count; ++n) {
      JsonObject y = it.add<JsonObject>(); y["id"] = scenes[k].items[n].id; y["position"] = scenes[k].items[n].position;
    }
  }
  JsonArray sh = o["schedules"].to<JsonArray>();
  for (size_t k = 0; k < scheduleCount; ++k) {
    const Schedule& x = schedules[k];
    JsonObject y = sh.add<JsonObject>();
    y["enabled"] = x.enabled; y["days"] = x.days;
    y["when"] = x.when == When::Sunrise ? "sunrise" : x.when == When::Sunset ? "sunset" : "time";
    y["minutes"] = x.minutes; y["target"] = x.target; y["action"] = x.action;
  }
}

bool automationFromJson(JsonObjectConst o, String& err) {
  Scene nextScenes[kMaxScenes]; size_t ns = 0;
  for (JsonObjectConst x : o["scenes"].as<JsonArrayConst>()) {
    if (ns >= kMaxScenes) { err = "up to 8 scenes"; return false; }
    Scene sc;
    sc.name = x["name"] | ""; sc.name.trim();
    if (!sc.name.length()) { err = "every scene needs a name"; return false; }
    sc.id = slug(x["id"] | sc.name);
    for (size_t k = 0; k < ns; ++k) if (nextScenes[k].id == sc.id) sc.id += String("_") + ns;
    for (JsonObjectConst y : x["blinds"].as<JsonArrayConst>()) {
      if (sc.count >= kMaxSceneItems) break;
      sc.items[sc.count].id = y["id"] | "";
      sc.items[sc.count].position = clampPos(y["position"] | 0);
      if (sc.items[sc.count].id.length()) sc.count++;
    }
    nextScenes[ns++] = sc;
  }
  Schedule nextSched[kMaxSchedules]; size_t nh = 0;
  for (JsonObjectConst x : o["schedules"].as<JsonArrayConst>()) {
    if (nh >= kMaxSchedules) { err = "up to 12 schedules"; return false; }
    Schedule s;
    s.enabled = x["enabled"] | true;
    s.days = static_cast<uint8_t>((x["days"] | 0x7F) & 0x7F);
    const String w = x["when"] | "time";
    s.when = w == "sunrise" ? When::Sunrise : w == "sunset" ? When::Sunset : When::Time;
    s.minutes = static_cast<int16_t>(x["minutes"] | 0);
    if (s.when == When::Time && (s.minutes < 0 || s.minutes >= 1440)) { err = "a schedule time must be 00:00-23:59"; return false; }
    if (s.when != When::Time && (s.minutes < -240 || s.minutes > 240)) { err = "sunrise/sunset offsets are up to 4 hours"; return false; }
    s.target = x["target"] | "all"; s.action = x["action"] | "OPEN"; s.action.toUpperCase();
    if (s.when != When::Time && (isnan(o["latitude"] | NAN))) { err = "set your location to use sunrise or sunset"; return false; }
    nextSched[nh++] = s;
  }
  for (size_t k = 0; k < ns; ++k) scenes[k] = nextScenes[k];
  sceneCount = ns;
  for (size_t k = 0; k < nh; ++k) schedules[k] = nextSched[k];
  scheduleCount = nh;
  siteLat = o["latitude"] | NAN; siteLon = o["longitude"] | NAN;
  return true;
}

void saveAutomation() {
  JsonDocument d; automationToJson(d.to<JsonObject>());
  String s; serializeJson(d, s); prefs.putString("auto", s);
}

void loadAutomation() {
  JsonDocument d; String err;
  if (deserializeJson(d, prefs.getString("auto", "{}"))) return;
  if (!automationFromJson(d.as<JsonObjectConst>(), err)) LOGW("Saved schedules not loaded: %s", err.c_str());
}

int utcOffsetMinutes() {
  const time_t now = time(nullptr);
  struct tm lt, gt;
  localtime_r(&now, &lt); gmtime_r(&now, &gt);
  gt.tm_isdst = lt.tm_isdst;
  return static_cast<int>((now - mktime(&gt)) / 60);
}

// Minute of the day a schedule runs today, or -1.
int scheduleMinute(const Schedule& s, const struct tm& lt) {
  if (s.when == When::Time) return s.minutes;
  if (isnan(siteLat)) return -1;
  const int sun = aok::sunMinutes(s.when == When::Sunrise, lt.tm_yday, siteLat, siteLon, utcOffsetMinutes());
  if (sun < 0) return -1;
  const int m = sun + s.minutes;
  return (m < 0 || m >= 1440) ? -1 : m;
}

void runSchedule(Schedule& s) {
  String err;
  bool ok;
  if (s.target.startsWith("scene:")) ok = runScene(s.target.substring(6), "schedule", err);
  else if (s.target == "all") {
    if (s.action == "OPEN" || s.action == "CLOSE" || s.action == "STOP") ok = allBlinds(s.action, "schedule", err);
    else {
      ok = false;
      for (size_t i = 0; i < blindCount; ++i) if (cfg[i].somfy || __builtin_popcount(cfg[i].mask) == 1) { String e; ok |= handleBlindCommand(i, s.action, "schedule", e); }
      if (!ok) err = "no blind could move";
    }
  } else {
    const int i = findBlind(s.target);
    ok = i >= 0 && handleBlindCommand(i, s.action, "schedule", err);
    if (i < 0) err = "blind not found";
  }
  if (ok) LOGI("Schedule: %s → %s", s.target.c_str(), s.action.c_str());
  else LOGW("Schedule %s → %s: %s", s.target.c_str(), s.action.c_str(), err.c_str());
}

void checkSchedules() {
  if (!scheduleCount || !wallClock()) return;
  const time_t now = time(nullptr);
  struct tm lt; localtime_r(&now, &lt);
  const int minute = lt.tm_hour * 60 + lt.tm_min;
  if (minute == lastScheduleMinute) return;
  // Minutes skipped since the last check today (the clocks going forward, or a long
  // firmware update) still count; after a restart only the current minute does.
  const int from = lastScheduleDay == lt.tm_yday && lastScheduleMinute >= 0 && minute > lastScheduleMinute
                       ? lastScheduleMinute + 1 : minute;
  lastScheduleMinute = minute;
  lastScheduleDay = lt.tm_yday;
  const int weekday = (lt.tm_wday + 6) % 7;  // 0 = Monday
  for (size_t k = 0; k < scheduleCount; ++k) {
    Schedule& s = schedules[k];
    if (!s.enabled || !(s.days & (1 << weekday)) || s.lastDay == lt.tm_yday) continue;
    const int due = scheduleMinute(s, lt);
    if (due < from || due > minute) continue;
    s.lastDay = lt.tm_yday;
    runSchedule(s);
  }
}

// A held hand-remote button: once frames stop, report a long press (1.5 s or more).
void finishPress() {
  if (!pressActive || millis() - lastPayloadMs < 500) return;
  pressActive = false;
  if (remote.repeat && repeatPayload && !learnMode) {
    // Repeater: send the same press again from the bridge, for a motor out of the remote's reach.
    String err;
    if (transmitPayload(repeatPayload, err)) LOGI("Repeated the remote's press");
    repeatPayload = 0;
  }
  if (lastPayloadMs - pressStartMs < 1500 || !mqtt.connected()) return;
  JsonDocument d;
  d["event_type"] = String(aok::actionName(pressAction)) + "_long";
  d["seconds"] = (lastPayloadMs - pressStartMs) / 1000.0f;
  publishJson(topic("remote/event"), d, false);
}

// ---------------------------------------------------------------- received frames
void recordFrame(const aok::Frame& f, bool matched) {
  if (frameCount && frames[(frameHead + kMaxFrames - 1) % kMaxFrames].f.payload == f.payload &&
      millis() - frames[(frameHead + kMaxFrames - 1) % kMaxFrames].ms < 3000) {
    HeardFrame& last = frames[(frameHead + kMaxFrames - 1) % kMaxFrames];
    last.repeats++;
    last.ms = millis();
    return;
  }
  stateRev++;
  frames[frameHead] = HeardFrame{millis(), f, matched, 1, lastRssi};
  frameHead = (frameHead + 1) % kMaxFrames;
  if (frameCount < kMaxFrames) frameCount++;
}

void onPayload(uint64_t payload) {
  const uint32_t now = millis();
  const aok::Frame f = aok::splitPayload(payload);
  const bool repeat = payload == lastPayload && now - lastPayloadMs < kDedupeMs;
  lastPayload = payload;
  lastPayloadMs = now;
  if (radioReady) lastRssi = static_cast<int16_t>(ELECHOUSE_cc1101.getRssi());
  const bool fromMain = (remote.have[0] || remote.have[1] || remote.have[2]) && f.prefix == remote.prefix && f.remoteId == remote.remoteId;
  const bool matched = fromMain || remoteKnown(f.prefix, f.remoteId);
  recordFrame(f, matched);
  if (repeat) return;
  lastRxFlashMs = now;
  if (fromMain && f.mask && ((f.command >> 8) & 0xFF) == 0xAC) {
    const uint16_t base = aok::deriveBase(f.remoteId, f.mask, f.command);
    if (!remote.haveProg || remote.progBase != base) {
      remote.progBase = base; remote.haveProg = true; saveRemote();
      LOGI("Learned the remote's programming (P) button");
    }
  }
  if (f.action == aok::Action::Unknown || !f.mask) return;
  if (matched && !learnMode) repeatPayload = payload;

  if (learnMode && learnExtra) {
    // Adding another remote: the first press from one the bridge doesn't know yet.
    if (fromMain || remoteKnown(f.prefix, f.remoteId)) { learnStatus = "that remote is already known: press a button on the new one"; return; }
    if (remote.extraCount >= 3) { learnStatus = "already following 3 extra remotes: remove one first"; learnMode = false; return; }
    remote.extraPrefix[remote.extraCount] = f.prefix;
    remote.extraId[remote.extraCount] = f.remoteId;
    remote.extraCount++;
    saveRemote();
    learnMode = false; learnExtra = false;
    learnStatus = String("added remote ") + hex(f.remoteId, 2);
    LOGI("Remote learning: %s", learnStatus.c_str());
    return;
  }
  if (learnMode) {
    if (learnLocked && (f.prefix != learnPrefix || f.remoteId != learnRemoteId)) {
      // Someone else's remote (a neighbour's?) mid-learning: ignore it, don't start over.
      learnStatus = String("ignored another remote (") + hex(f.remoteId, 2) + "), keep pressing yours";
      return;
    }
    if (!learnLocked) { learnLocked = true; learnPrefix = f.prefix; learnRemoteId = f.remoteId; }
    const bool sameRemote = f.prefix == remote.prefix && f.remoteId == remote.remoteId;
    if (!sameRemote) { for (bool& h : remote.have) h = false; remote.seen = 0; }
    remote.prefix = f.prefix;
    remote.remoteId = f.remoteId;
    const int a = static_cast<int>(f.action);
    remote.base[a] = aok::deriveBase(f.remoteId, f.mask, f.command);
    remote.have[a] = true;
    remote.seen |= f.mask;
    saveRemote();
    learnStatus = String("learned ") + aok::actionName(f.action) + " from remote " + hex(f.remoteId, 2) + " (channel " + maskText(f.mask) + ")";
    if (remoteComplete()) { learnMode = false; learnStatus = "complete"; }
    LOGI("Remote learning: %s", learnStatus.c_str());
    return;
  }

  if (!matched) return;
  pressActive = true; pressStartMs = now; pressAction = f.action;
  if (fromMain && (remote.seen | f.mask) != remote.seen) { remote.seen |= f.mask; saveRemote(); }
  LOGI("Remote: %s on channel %s", f.action == aok::Action::Open ? "Up" : f.action == aok::Action::Close ? "Down" : "Stop", maskText(f.mask).c_str());
  lastEventText = String(aok::actionName(f.action)) + " · channel " + maskText(f.mask);
  if (mqtt.connected()) {
    JsonDocument d;
    d["event_type"] = aok::actionName(f.action);
    d["channels"] = maskText(f.mask);
    publishJson(topic("remote/event"), d, false);
    mqtt.publish(topic("diag/last_remote").c_str(), lastEventText.c_str(), true);
  }
  if (remote.listen) applyAction(f.mask, f.action, "hand_remote", -1, -1);
}

void recordSomfy(const somfy::Frame& sf, uint64_t raw, bool matched) {
  HeardFrame& last = frames[(frameHead + kMaxFrames - 1) % kMaxFrames];
  if (frameCount && last.somfy && last.s.address == sf.address && last.s.code == sf.code && millis() - last.ms < 3000) {
    last.repeats++; last.ms = millis(); return;
  }
  stateRev++;
  HeardFrame h;
  h.ms = millis(); h.matched = matched; h.rssi = lastRssi; h.somfy = true; h.s = sf; h.f.payload = raw;
  frames[frameHead] = h;
  frameHead = (frameHead + 1) % kMaxFrames;
  if (frameCount < kMaxFrames) frameCount++;
}

bool somfyFollows(size_t i, uint32_t addr) {
  for (uint8_t k = 0; k < cfg[i].somfyRemoteCount; ++k) if (cfg[i].somfyRemote[k] == addr) return true;
  return false;
}

// A Somfy hand remote press: learn it, or move the positions of the motors it controls.
void onSomfy(const uint8_t raw[7]) {
  somfy::Frame sf;
  if (!somfy::decode(raw, sf)) return;
  const uint32_t now = millis();
  if (radioReady) lastRssi = static_cast<int16_t>(ELECHOUSE_cc1101.getRssi());
  bool ours = false, matched = false;
  for (size_t i = 0; i < blindCount; ++i) {
    if (!cfg[i].somfy) continue;
    ours |= cfg[i].somfyAddr == sf.address;
    matched |= somfyFollows(i, sf.address);
  }
  recordSomfy(sf, somfy::pack(raw), matched);
  // a held button repeats the same rolling code: act on it once
  const bool repeat = sf.address == lastSomfyAddr && sf.code == lastSomfyCode && now - lastSomfyMs < 3000;
  lastSomfyAddr = sf.address; lastSomfyCode = sf.code; lastSomfyMs = now;
  if (repeat || ours) return;
  lastRxFlashMs = now;

  if (somfyLearn >= 0 && somfyLearn < static_cast<int>(blindCount) && cfg[somfyLearn].somfy) {
    BlindCfg& b = cfg[somfyLearn];
    if (somfyFollows(somfyLearn, sf.address)) { somfyStatus = "that remote is already learned for " + b.name; return; }
    if (b.somfyRemoteCount >= kSomfyRemotes) { somfyStatus = "already following 3 remotes: forget them first"; somfyLearn = -1; return; }
    b.somfyRemote[b.somfyRemoteCount++] = sf.address;
    saveBlinds();
    somfyStatus = String("learned remote ") + hex(sf.address, 6) + " for " + b.name;
    LOGI("Somfy: %s", somfyStatus.c_str());
    somfyLearn = -1;
    stateRev++;
    return;
  }
  if (!matched) return;
  const char* name = somfy::commandName(sf.command);
  LOGI("Somfy remote %s: %s", hex(sf.address, 6).c_str(), name);
  lastEventText = String("Somfy ") + name + " · " + hex(sf.address, 6);
  if (mqtt.connected()) {
    JsonDocument d;
    d["event_type"] = sf.command == somfy::Command::Up ? "open" : sf.command == somfy::Command::Down ? "close" : "stop";
    d["somfy_remote"] = hex(sf.address, 6);
    d["somfy_command"] = name;
    publishJson(topic("remote/event"), d, false);
    mqtt.publish(topic("diag/last_remote").c_str(), lastEventText.c_str(), true);
  }
  if (!remote.listen) return;
  for (size_t i = 0; i < blindCount; ++i) {
    if (!cfg[i].somfy || !somfyFollows(i, sf.address)) continue;
    st[i].pending = -1; st[i].after = -1;
    switch (sf.command) {
      case somfy::Command::Up: applyOne(i, cfg[i].invert ? aok::Action::Close : aok::Action::Open, "hand_remote", false, -1); break;
      case somfy::Command::Down: applyOne(i, cfg[i].invert ? aok::Action::Open : aok::Action::Close, "hand_remote", false, -1); break;
      case somfy::Command::My:
        if (st[i].motion != Motion::Stopped) haltMotion(i, "hand_remote");
        else if (cfg[i].favourite >= 0) {  // stopped: the motor goes to its My position
          int cur = estimate(i);
          if (cur < 0) cur = cfg[i].favourite >= 50 ? 0 : 100;
          if (cur != cfg[i].favourite) {
            st[i].position = st[i].startPos = cur;
            beginMotion(i, cfg[i].favourite > cur ? Motion::Opening : Motion::Closing, cfg[i].favourite, false, "hand_remote");
          }
        }
        break;
      default: break;
    }
  }
  requestPositionSave();
}

void pollRadio() {
  uint64_t payload;
  uint8_t sraw[7];
  const bool listenSomfy = anySomfy();
  uint16_t budget = 512;
  while (ringTail != ringHead && budget--) {
    const uint32_t v = ring[ringTail];
    ringTail = (ringTail + 1) & (kRingSize - 1);
    const uint32_t us = v & 0x7FFFFFFF;
    const bool high = (v & 0x80000000u) != 0;
    if (decoder.feed(us, high, payload)) onPayload(payload);
    if (listenSomfy && somfyDecoder.feed(us, high, sraw)) onSomfy(sraw);
  }
  if (learnMode && millis() - learnStartedMs > 120000) { learnMode = false; learnExtra = false; learnStatus = remoteComplete() ? "complete" : "timed out"; }
  if (somfyLearn >= 0 && millis() - somfyLearnMs > 60000) { somfyLearn = -1; somfyStatus = "timed out: no Somfy remote heard"; stateRev++; }
}

// ---------------------------------------------------------------- web
// Requests that change something must come from the bridge's own page. A web page
// on another site can make the browser send a simple POST (a form, or text/plain
// JSON) to this address, but it can't add a custom header without a CORS
// preflight, which this server never approves. The browser also re-sends a saved
// Basic-auth login, so the password alone doesn't stop that.
bool fromOwnPage() {
  if (web.method() != HTTP_POST) return true;
  if (web.header("X-Bridge") == "1") return true;
  LOGW("Refused %s: not sent by the bridge's own page", web.uri().c_str());
  sendError("refused: requests must come from the bridge's own page", 403);
  return false;
}

// DNS rebinding: a web site can point its own name at this bridge's address, and the
// browser then treats the bridge as that site. The Host header still carries the other
// site's name, so only answer to the bridge's own IP addresses and name, plus the extra
// names an admin added (a router's DNS name such as aok-blinds.lan).
bool hostAllowed(String h) {
  if (h.startsWith("[")) return false;  // IPv6 literal: the bridge only serves on IPv4
  const int colon = h.indexOf(':');
  if (colon >= 0) h = h.substring(0, colon);
  h.toLowerCase();
  if (h.endsWith(".")) h.remove(h.length() - 1);
  String name = net.hostname; name.toLowerCase();
  if (h == name || h == name + ".local") return true;
  if (aok::hostInList(h.c_str(), net.extraHosts.c_str())) return true;
  if (netUp() && h == netIp().toString()) return true;
  if (WiFi.status() == WL_CONNECTED && h == WiFi.localIP().toString()) return true;  // Ethernet builds with Wi-Fi too
  return apMode && h == WiFi.softAPIP().toString();
}

bool hostOk() {
  if (hostAllowed(web.hostHeader())) return true;
  LOGW("Refused %s: sent to the unknown name '%s'", web.uri().c_str(), web.hostHeader().c_str());
  sendError("refused: open the bridge by its IP address or its .local name, or add this name under Settings, Device, Other names", 403);
  return false;
}

const char* const kNeedPassword =
  "set an admin password first: Settings, Device, Admin password. Until then this page only does Wi-Fi setup";

// Every API request: the right Host, the bridge's own page for changes, then the password.
// Until a password is set only `setup` requests (Wi-Fi, the password itself, status) work.
bool authed(bool setup = false) {
  if (!hostOk() || !fromOwnPage()) return false;
  if (!net.adminPass.length()) {
    if (setup) return true;
    sendError(kNeedPassword, 403);
    return false;
  }
  if (web.authenticate("admin", net.adminPass.c_str())) return true;
  web.requestAuthentication(BASIC_AUTH, "Blind Bridge");
  return false;
}

// The live WebSocket can't use the page's login, so the page passes this token in its URL.
// It comes from /api/state, which needs the login, and changes with the password.
String wsToken;
void newWsToken() {
  char b[25];
  snprintf(b, sizeof(b), "%08lx%08lx%08lx", static_cast<unsigned long>(esp_random()),
           static_cast<unsigned long>(esp_random()), static_cast<unsigned long>(esp_random()));
  wsToken = b;
}

void sendJson(JsonDocument& d, int code = 200) {
  String s; serializeJson(d, s);
  web.send(code, "application/json", s);
}

void sendError(const String& msg, int code = 400) {
  JsonDocument d; d["ok"] = false; d["error"] = msg; sendJson(d, code);
}

void sendOk() { JsonDocument d; d["ok"] = true; sendJson(d); }

bool bodyJson(JsonDocument& d) {
  if (!web.hasArg("plain") || deserializeJson(d, web.arg("plain"))) { sendError("invalid JSON body"); return false; }
  return true;
}

void buildState(JsonDocument& d, bool lite);

void handleState() {
  if (!authed(true)) return;
  const bool lite = web.hasArg("lite");  // frequent polls skip frames and calibration history
  JsonDocument d;
  buildState(d, lite);
  if (net.adminPass.length()) d["ws_token"] = wsToken;
  sendJson(d);
}

void buildState(JsonDocument& d, bool lite) {
  d["fw"] = FW_VERSION; d["uptime"] = millis() / 1000; d["heap"] = ESP.getFreeHeap(); d["heap_max_block"] = ESP.getMaxAllocHeap();
  d["admin_set"] = net.adminPass.length() > 0;
  d["rev"] = stateRev; d["brand"] = net.brand; d["lite"] = lite;
  d["safe_mode"] = safeMode; d["wind"] = windActive;
  JsonArray scl = d["scenes"].to<JsonArray>();
  for (size_t k = 0; k < sceneCount; ++k) { JsonObject x = scl.add<JsonObject>(); x["id"] = scenes[k].id; x["name"] = scenes[k].name; }
  d["reset_reason"] = static_cast<int>(esp_reset_reason());
  JsonObject w = d["wifi"].to<JsonObject>();
  w["connected"] = netUp(); w["ssid"] = net.ssid; w["ip"] = netIp().toString();
  w["rssi"] = WiFi.RSSI(); w["ap"] = apMode; w["hostname"] = net.hostname;
  JsonObject m = d["mqtt"].to<JsonObject>();
  m["configured"] = net.mqttHost.length() > 0; m["connected"] = mqtt.connected(); m["state"] = mqtt.state();
  JsonObject r = d["radio"].to<JsonObject>();
  r["ready"] = radioReady; r["overflow"] = static_cast<uint32_t>(ringOverflow); r["mhz"] = pins.mhz;
  if (lastRssi) r["last_rssi"] = lastRssi;
  r["last_tx_ago_s"] = lastTxMs ? (millis() - lastTxMs) / 1000 : -1;
  r["last_rx_ago_s"] = lastRxFlashMs ? (millis() - lastRxFlashMs) / 1000 : -1;
  JsonObject rm = d["remote"].to<JsonObject>();
  remoteToJson(rm);
  rm["complete"] = remoteComplete(); rm["learning"] = learnMode; rm["learn_status"] = learnStatus;
  JsonArray bl = d["blinds"].to<JsonArray>();
  for (size_t i = 0; i < blindCount; ++i) {
    JsonObject o = bl.add<JsonObject>();
    blindToJson(i, o);
    int pos; Motion mo; displayState(i, pos, mo);
    o["position"] = pos; o["motion"] = motionName(mo); o["target"] = st[i].target; o["source"] = st[i].lastSource;
    o["group"] = isGroup(i);
    o["locked"] = st[i].locked; o["window_open"] = st[i].interlock; o["wind"] = st[i].windLocked;
    if (mo != Motion::Stopped && pos >= 0) {
      // lets the page animate smoothly between polls: % per second and where the run ends
      o["rate"] = 100000.0f / static_cast<float>(travelMs(i, mo));
      o["goal"] = (st[i].autoStop && st[i].motion == mo) ? st[i].target : (mo == Motion::Opening ? 100 : 0);
    }
  }
  if (!lite) {
  JsonArray fr = d["frames"].to<JsonArray>();
  for (size_t k = 0; k < frameCount; ++k) {
    const HeardFrame& h = frames[(frameHead + kMaxFrames - 1 - k) % kMaxFrames];
    JsonObject o = fr.add<JsonObject>();
    if (h.somfy) {
      o["protocol"] = "somfy"; o["ago_s"] = (millis() - h.ms) / 1000; o["payload"] = hex(h.f.payload, 14);
      o["address"] = hex(h.s.address, 6); o["rolling_code"] = h.s.code; o["action"] = somfy::commandName(h.s.command);
      o["matched"] = h.matched; o["repeats"] = h.repeats; o["kind"] = somfy::commandName(h.s.command);
      if (h.rssi) o["rssi"] = h.rssi;
      if (wallClock()) o["when"] = static_cast<uint32_t>(wallClock() - (millis() - h.ms) / 1000);
      continue;
    }
    o["protocol"] = "aok";
    o["ago_s"] = (millis() - h.ms) / 1000; o["payload"] = hex(h.f.payload, 16); o["remote_id"] = hex(h.f.remoteId, 2);
    o["prefix"] = hex(h.f.prefix, 6); o["channels"] = maskText(h.f.mask); o["action"] = aok::actionName(h.f.action);
    o["matched"] = h.matched; o["repeats"] = h.repeats; o["kind"] = frameKind(h.f);
    if (h.rssi) o["rssi"] = h.rssi;
    o["code"] = hex((h.f.command >> 8) & 0xFF, 2);
    if (wallClock()) o["when"] = static_cast<uint32_t>(wallClock() - (millis() - h.ms) / 1000);
  }
  JsonArray ch = d["cal_history"].to<JsonArray>();
  for (size_t k = 0; k < calHistoryCount; ++k) {
    JsonObject o = ch.add<JsonObject>();
    o["name"] = calHistory[k].name; o["direction"] = calHistory[k].opening ? "open" : "close";
    o["ms"] = calHistory[k].ms; o["previous_ms"] = calHistory[k].previousMs;
    if (calHistory[k].when) o["when"] = static_cast<uint32_t>(calHistory[k].when);
  }
  }
  d["time"] = static_cast<uint32_t>(wallClock());
  d["log_seq"] = logSeq;
  JsonObject c = d["calibration"].to<JsonObject>();
  c["active"] = calIndex >= 0;
  if (calIndex >= 0) { c["id"] = cfg[calIndex].id; c["name"] = cfg[calIndex].name; c["direction"] = motionName(calMotion); c["elapsed_ms"] = millis() - calStartMs; }
  JsonObject pr = d["pairing"].to<JsonObject>();
  pr["channel"] = pairChannel; pr["status"] = pairStatus;
  JsonObject so = d["somfy"].to<JsonObject>();
  so["status"] = somfyStatus;
  so["learning"] = somfyLearn >= 0 ? cfg[somfyLearn].id : "";
  so["mhz"] = kSomfyMhz;
  r["rx_mhz"] = rxMhzNow; r["rx_bw"] = rxBwNow;
  d["live"] = true;  // this firmware pushes the state over a WebSocket on port 81
#ifdef USE_ETH
  d["build"] = "eth";
#else
  d["build"] = "wifi";
#endif
}

// Live updates: push the light state to open pages several times a second while
// anything moves, and on every change; a slow heartbeat otherwise.
void pushLive() {
  if (!ws.connectedClients()) return;
  const uint32_t now = millis();
  const bool busy = anyMoving() || calIndex >= 0 || learnMode || pairChannel;
  const uint32_t every = busy ? 250 : (stateRev != lastWsRev ? 0 : 5000);
  if (now - lastWsPushMs < every) return;
  lastWsPushMs = now;
  lastWsRev = stateRev;
  JsonDocument d;
  buildState(d, true);
  String out; serializeJson(d, out);
  ws.broadcastTXT(out);
}

void handleConfigGet() {
  if (!authed(true)) return;
  JsonDocument d;
  JsonObject n = d["network"].to<JsonObject>();
  n["ssid"] = net.ssid; n["pass_set"] = net.pass.length() > 0; n["hostname"] = net.hostname;
  n["admin_set"] = net.adminPass.length() > 0;
  n["mqtt_host"] = net.mqttHost; n["mqtt_port"] = net.mqttPort; n["mqtt_user"] = net.mqttUser;
  n["mqtt_pass_set"] = net.mqttPass.length() > 0; n["base_topic"] = net.base; n["discovery_prefix"] = net.discovery;
  n["brand"] = net.brand; n["tz"] = net.tz; n["tz_name"] = net.tzName;
  n["extra_hosts"] = net.extraHosts; n["resync_after"] = net.resyncEvery; n["resync_through"] = net.resyncThrough;
  JsonObject p = d["pins"].to<JsonObject>();
  p["sck"] = pins.sck; p["miso"] = pins.miso; p["mosi"] = pins.mosi; p["cs"] = pins.cs;
  p["gdo0"] = pins.gdo0; p["gdo2"] = pins.gdo2; p["led"] = pins.led; p["mhz"] = pins.mhz; p["pa_dbm"] = pins.paDbm; p["rx_bw"] = pins.rxBw;
  remoteToJson(d["remote"].to<JsonObject>());
  JsonArray b = d["blinds"].to<JsonArray>();
  for (size_t i = 0; i < blindCount; ++i) blindToJson(i, b.add<JsonObject>());
  sendJson(d);
}

void scheduleReboot(uint32_t ms = 1500) { rebootPending = true; rebootAtMs = millis() + ms; }

void handleNetworkPost() {
  if (!authed(true)) return;
  JsonDocument d; if (!bodyJson(d)) return;
  if (!net.adminPass.length()) {
    for (const char* k : {"mqtt_host", "mqtt_port", "mqtt_user", "mqtt_pass", "base_topic", "discovery_prefix", "extra_hosts", "resync_after", "resync_through"})
      if (!d[k].isNull()) { sendError(kNeedPassword, 403); return; }
  }
  const bool passChange = d["admin_pass"].is<const char*>();
  if (passChange && net.adminPass.length() && String(d["admin_current"] | "") != net.adminPass) {
    sendError("the current admin password is wrong", 403);
    return;
  }
  // Extra names widen what the DNS-rebinding check lets in, so only the admin sets them (checked above).
  std::string hosts;
  if (d["extra_hosts"].is<const char*>() && aok::parseHostList(d["extra_hosts"].as<const char*>(), hosts) < 0) {
    sendError("other names: up to 4 names such as aok-blinds.lan, separated by commas");
    return;
  }
  if (!d["resync_after"].isNull() && (!d["resync_after"].is<int>() || d["resync_after"].as<int>() < 0 || d["resync_after"].as<int>() > 100)) {
    sendError("re-sync after: 0 (off) to 100 moves");
    return;
  }
  bool reboot = false;
  if (d["ssid"].is<const char*>()) { String v = d["ssid"].as<String>(); if (v != net.ssid) { net.ssid = v; reboot = true; } }
  if (d["pass"].is<const char*>() && d["pass"].as<String>().length()) { net.pass = d["pass"].as<String>(); reboot = true; }
  if (d["hostname"].is<const char*>()) { String v = slug(d["hostname"].as<String>()); v.replace("_", "-"); if (v != net.hostname) { net.hostname = v; reboot = true; } }
  if (passChange) {
    net.adminPass = d["admin_pass"].as<String>();
    newWsToken();
    ws.disconnect();  // open pages reconnect with the new token
    LOGI(net.adminPass.length() ? "Admin password set" : "Admin password cleared: the page is back to Wi-Fi setup only");
  }
  if (d["mqtt_host"].is<const char*>()) net.mqttHost = d["mqtt_host"].as<String>();
  if (d["mqtt_port"].is<int>()) net.mqttPort = d["mqtt_port"];
  if (d["mqtt_user"].is<const char*>()) net.mqttUser = d["mqtt_user"].as<String>();
  if (d["mqtt_pass"].is<const char*>() && d["mqtt_pass"].as<String>().length()) net.mqttPass = d["mqtt_pass"].as<String>();
  if (d["base_topic"].is<const char*>() && d["base_topic"].as<String>().length()) net.base = slug(d["base_topic"].as<String>());
  if (d["discovery_prefix"].is<const char*>() && d["discovery_prefix"].as<String>().length()) net.discovery = d["discovery_prefix"].as<String>();
  if (d["brand"].is<const char*>()) { String v = d["brand"].as<String>(); v.trim(); if (v.length() && v.length() <= 24) net.brand = v; }
  if (d["extra_hosts"].is<const char*>() && net.extraHosts != hosts.c_str()) { net.extraHosts = hosts.c_str(); LOGI("Other names for the page: %s", hosts.length() ? hosts.c_str() : "none"); }
  if (d["resync_after"].is<int>()) net.resyncEvery = d["resync_after"].as<int>();
  if (d["resync_through"].is<bool>()) net.resyncThrough = d["resync_through"];
  if (d["tz"].is<const char*>()) {
    net.tz = d["tz"].as<String>(); net.tzName = d["tz_name"] | "";
    setenv("TZ", net.tz.length() ? net.tz.c_str() : "UTC0", 1); tzset();
  }
  saveNet();
  LOGI("Network settings saved%s", reboot ? ", restarting" : "");
  if (mqtt.connected()) mqtt.disconnect();
  lastMqttTryMs = 0;
  if (reboot) scheduleReboot(2000);
  JsonDocument r; r["ok"] = true; r["rebooting"] = reboot; sendJson(r);
}

void handlePinsPost() {
  if (!authed()) return;
  JsonDocument d; if (!bodyJson(d)) return;
  pins.sck = d["sck"] | pins.sck; pins.miso = d["miso"] | pins.miso; pins.mosi = d["mosi"] | pins.mosi;
  pins.cs = d["cs"] | pins.cs; pins.gdo0 = d["gdo0"] | pins.gdo0; pins.gdo2 = d["gdo2"] | pins.gdo2;
  pins.led = d["led"] | pins.led; pins.mhz = d["mhz"] | pins.mhz; pins.paDbm = d["pa_dbm"] | pins.paDbm;
  pins.rxBw = constrain(static_cast<int>(d["rx_bw"] | pins.rxBw), 0, 812);
  savePins();
  scheduleReboot(1500);
  JsonDocument r; r["ok"] = true; r["rebooting"] = true; sendJson(r);
}

void handleRemotePost() {
  if (!authed()) return;
  JsonDocument d; if (!bodyJson(d)) return;
  const String action = d["action"] | "";
  if (action == "learn") { learnMode = true; learnLocked = false; learnStartedMs = millis(); learnStatus = "waiting"; LOGI("Remote learning started"); }
  else if (action == "learn_extra") { learnMode = true; learnExtra = true; learnLocked = false; learnStartedMs = millis(); learnStatus = "press any button on the other remote"; LOGI("Adding another remote"); }
  else if (action == "forget_extra") { remote.extraCount = 0; saveRemote(); LOGI("Extra remotes forgotten"); }
  else if (action == "cancel") { learnMode = false; learnExtra = false; learnStatus = "cancelled"; }
  else if (action == "forget") { for (bool& h : remote.have) h = false; remote.prefix = 0; remote.remoteId = 0; remote.seen = 0; saveRemote(); learnStatus = "idle"; LOGI("Remote forgotten"); }
  else if (action == "save") { remoteFromJson(d["remote"].as<JsonObjectConst>()); saveRemote(); }
  else { sendError("unknown action"); return; }
  sendOk();
}

void handleBlindsPost() {
  if (!authed()) return;
  JsonDocument d; if (!bodyJson(d)) return;
  String err;
  const bool hadSomfy = anySomfy();
  if (!blindsFromJson(d["blinds"].as<JsonArrayConst>(), err)) { sendError(err); return; }
  saveBlinds();
  requestPositionSave();
  stateRev++;
  LOGI("Blinds saved (%u)", static_cast<unsigned>(blindCount));
  if (hadSomfy != anySomfy() && radioReady) { rxStop(); rxStart(); LOGI("Receiver retuned to %.2f MHz, %.0f kHz wide", rxMhzNow, rxBwNow); }
  if (mqtt.connected()) mqtt.disconnect();  // reconnect re-subscribes and republishes discovery
  lastMqttTryMs = 0;
  sendOk();
}

void handleCommandPost() {
  if (!authed()) return;
  JsonDocument d; if (!bodyJson(d)) return;
  if (String(d["id"] | "") == "all") {
    String err;
    if (!allBlinds(d["cmd"].as<String>(), "web", err)) { sendError(err); return; }
    sendOk(); return;
  }
  const int i = findBlind(d["id"] | "");
  if (i < 0) { sendError("unknown blind"); return; }
  String err;
  LOGI("Web page: %s → %s", cfg[i].name.c_str(), d["cmd"].as<String>().c_str());
  if (!handleBlindCommand(i, d["cmd"].as<String>(), "web", err)) { LOGW("%s: %s", cfg[i].name.c_str(), err.c_str()); sendError(err); return; }
  sendOk();
}

void handleTestPost() {  // send a raw action on any channel(s): handy before blinds are set up
  if (!authed()) return;
  JsonDocument d; if (!bodyJson(d)) return;
  const uint16_t mask = parseChannels(d["channels"] | "");
  const String a = d["action"] | "";
  const aok::Action act = a == "open" ? aok::Action::Open : a == "close" ? aok::Action::Close : a == "stop" ? aok::Action::Stop : aok::Action::Unknown;
  String err;
  if (!mask) { sendError("choose a channel"); return; }
  if (!sendAction(mask, act, err)) { sendError(err); return; }
  applyAction(mask, act, "test", -1, -1);
  sendOk();
}

void handleCalibratePost() {
  if (!authed()) return;
  JsonDocument d; if (!bodyJson(d)) return;
  const String action = d["action"] | "";
  String err;
  if (action == "start_close" || action == "start_open") {
    const int i = findBlind(d["id"] | "");
    if (i < 0) { sendError("unknown blind"); return; }
    calMotion = action == "start_open" ? Motion::Opening : Motion::Closing;
    if (!blindSend(i, rfFor(i, calMotion), err)) { sendError(err); return; }
    calIndex = i;
    calStartMs = millis();
    LOGI("Calibration started: %s %s", cfg[i].name.c_str(), calMotion == Motion::Opening ? "opening" : "closing");
    blindApply(i, rfFor(i, calMotion), "calibration", -1);
    sendOk();
  } else if (action == "done" && calIndex >= 0) {
    const uint32_t ms = millis() - calStartMs;
    if (ms < 1000) { sendError("that was under a second: press It's stopped when the blind stops moving"); return; }
    const bool opening = calMotion == Motion::Opening;
    const uint32_t previous = opening ? cfg[calIndex].openMs : cfg[calIndex].closeMs;
    if (opening) cfg[calIndex].openMs = ms; else cfg[calIndex].closeMs = ms;
    addCalRecord(cfg[calIndex].name, opening, ms, previous);
    LOGI("Calibrated %s: %s takes %.1f s (was %.1f s)", cfg[calIndex].name.c_str(), opening ? "opening" : "closing", ms / 1000.0, previous / 1000.0);
    const int p = calMotion == Motion::Opening ? 100 : 0;
    st[calIndex].motion = Motion::Stopped;
    st[calIndex].position = st[calIndex].startPos = st[calIndex].target = p;
    st[calIndex].partials = 0;
    st[calIndex].dirty = true;
    saveBlinds();
    requestPositionSave();
    JsonDocument r; r["ok"] = true; r["measured_ms"] = ms; sendJson(r);
    calIndex = -1;
  } else if (action == "clear_history") {
    calHistoryCount = 0; saveCalHistory(); stateRev++; sendOk();
  } else if (action == "cancel") {
    if (calIndex >= 0) { stopBlind(calIndex, "calibration", err); LOGI("Calibration cancelled"); }
    calIndex = -1;
    sendOk();
  } else sendError("unknown action");
}

// ---------------------------------------------------------------- MQTT broker discovery and test
bool portOpen(const IPAddress& ip, uint16_t port) {
  WiFiClient c;
  const bool ok = c.connect(ip, port, 700);
  c.stop();
  return ok;
}

// Looks for MQTT brokers on the network: services advertised as _mqtt._tcp, and Home Assistant
// (_home-assistant._tcp or homeassistant.local) with something listening on 1883.
void handleMqttDiscover() {
  if (!authed()) return;
  if (anyMoving()) { sendError("blinds are moving: search again when they've stopped", 409); return; }
  if (WiFi.status() != WL_CONNECTED) { sendError("join your Wi-Fi first, then search for the broker"); return; }
  if (!mdnsReady && MDNS.begin(net.hostname.c_str())) { MDNS.addService("http", "tcp", 80); mdnsReady = true; }
  JsonDocument d;
  JsonArray a = d["brokers"].to<JsonArray>();
  auto add = [&](const String& host, const IPAddress& ip, uint16_t port, const char* what) {
    if (ip == IPAddress(0, 0, 0, 0)) return;
    for (JsonObject o : a) if (o["ip"] == ip.toString() && o["port"] == port) return;
    if (!portOpen(ip, port)) return;
    JsonObject o = a.add<JsonObject>();
    o["host"] = host; o["ip"] = ip.toString(); o["port"] = port; o["what"] = what;
  };
  int n = MDNS.queryService("mqtt", "tcp");
  for (int i = 0; i < n; ++i) add(MDNS.hostname(i) + ".local", MDNS.IP(i), MDNS.port(i), "MQTT broker");
  n = MDNS.queryService("home-assistant", "tcp");
  for (int i = 0; i < n; ++i) add(MDNS.hostname(i) + ".local", MDNS.IP(i), 1883, "Home Assistant");
  if (a.size() == 0) add("homeassistant.local", MDNS.queryHost("homeassistant", 1500), 1883, "Home Assistant");
  LOGI("Broker search found %u", static_cast<unsigned>(a.size()));
  sendJson(d);
}

// Tries a login with the settings on the page (saved password used if the box is blank).
void handleMqttTest() {
  if (!authed()) return;
  if (anyMoving()) { sendError("blinds are moving: test again when they've stopped", 409); return; }
  JsonDocument d; if (!bodyJson(d)) return;
  if (WiFi.status() != WL_CONNECTED) { sendError("the bridge isn't on Wi-Fi yet"); return; }
  const String host = d["host"] | net.mqttHost;
  const uint16_t port = d["port"] | net.mqttPort;
  const String user = d["user"] | net.mqttUser;
  String pass = d["pass"] | "";
  if (!pass.length() && user == net.mqttUser) pass = net.mqttPass;
  if (!host.length()) { sendError("enter the broker address"); return; }
  WiFiClient wc;
  PubSubClient test(wc);
  test.setServer(host.c_str(), port);
  test.setSocketTimeout(4);
  const String id = net.base + "_test_" + String(static_cast<uint32_t>(esp_random()), HEX);
  const bool ok = user.length() ? test.connect(id.c_str(), user.c_str(), pass.c_str()) : test.connect(id.c_str());
  const int st = test.state();
  test.disconnect();
  JsonDocument r;
  r["ok"] = true; r["connected"] = ok; r["state"] = st;
  r["message"] = ok ? "Connected. These settings work."
    : st == 5 || st == 4 ? "The broker rejected the username or password."
    : st == -2 ? "Can't reach the broker: check the address and port."
    : st == -4 ? "The broker didn't answer in time."
    : "Couldn't connect.";
  LOGI("MQTT test %s:%u as %s: %s", host.c_str(), port, user.length() ? user.c_str() : "(no user)", ok ? "ok" : "failed");
  sendJson(r);
}

// Wi-Fi scan for the setup page: first call starts an async scan, later calls return results
void handleScan() {
  if (!authed(true)) return;
  const int16_t n = WiFi.scanComplete();
  JsonDocument d;
  if (n == WIFI_SCAN_RUNNING) { d["scanning"] = true; }
  else if (n < 0) {
    if (!(WiFi.getMode() & WIFI_STA)) WiFi.mode(apMode ? WIFI_AP_STA : WIFI_STA);
    WiFi.scanNetworks(true, false);
    d["scanning"] = true;
  } else {
    JsonArray a = d["networks"].to<JsonArray>();
    for (int i = 0; i < n; i++) {
      const String ssid = WiFi.SSID(i);
      if (!ssid.length()) continue;
      bool dup = false;
      for (JsonObject o : a) {
        if (o["ssid"] == ssid) { dup = true; if (WiFi.RSSI(i) > o["rssi"].as<int>()) o["rssi"] = WiFi.RSSI(i); break; }
      }
      if (dup) continue;
      JsonObject o = a.add<JsonObject>();
      o["ssid"] = ssid; o["rssi"] = WiFi.RSSI(i); o["secure"] = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
    }
    WiFi.scanDelete();
    d["scanning"] = false;
  }
  String out; serializeJson(d, out);
  web.send(200, "application/json", out);
}

// Settings backup: everything except passwords and the Wi-Fi network, so it can move to a new board.
void handleBackup() {
  if (!authed()) return;
  JsonDocument d;
  d["app"] = "aok-blind-bridge"; d["version"] = FW_VERSION; d["saved"] = static_cast<uint32_t>(wallClock());
  JsonObject n = d["network"].to<JsonObject>();
  n["hostname"] = net.hostname; n["brand"] = net.brand; n["tz"] = net.tz; n["tz_name"] = net.tzName;
  n["mqtt_host"] = net.mqttHost; n["mqtt_port"] = net.mqttPort; n["mqtt_user"] = net.mqttUser;
  n["base_topic"] = net.base; n["discovery_prefix"] = net.discovery;
  n["extra_hosts"] = net.extraHosts; n["resync_after"] = net.resyncEvery; n["resync_through"] = net.resyncThrough;
  JsonObject p = d["pins"].to<JsonObject>();
  p["sck"] = pins.sck; p["miso"] = pins.miso; p["mosi"] = pins.mosi; p["cs"] = pins.cs;
  p["gdo0"] = pins.gdo0; p["gdo2"] = pins.gdo2; p["led"] = pins.led; p["mhz"] = pins.mhz; p["pa_dbm"] = pins.paDbm; p["rx_bw"] = pins.rxBw;
  remoteToJson(d["remote"].to<JsonObject>());
  JsonArray b = d["blinds"].to<JsonArray>();
  for (size_t i = 0; i < blindCount; ++i) blindToJson(i, b.add<JsonObject>());
  automationToJson(d["automation"].to<JsonObject>());
  web.sendHeader("Content-Disposition", String("attachment; filename=") + net.hostname + "-backup.json");
  sendJson(d);
}

void handleRestore() {
  if (!authed()) return;
  JsonDocument d; if (!bodyJson(d)) return;
  if (String(d["app"] | "") != "aok-blind-bridge") { sendError("that isn't a blind bridge backup file"); return; }
  String err;
  if (d["blinds"].is<JsonArrayConst>() && !blindsFromJson(d["blinds"].as<JsonArrayConst>(), err)) { sendError(err); return; }
  JsonObjectConst n = d["network"];
  if (!n.isNull()) {
    net.hostname = n["hostname"] | net.hostname; net.brand = n["brand"] | net.brand;
    net.tz = n["tz"] | net.tz; net.tzName = n["tz_name"] | net.tzName;
    net.mqttHost = n["mqtt_host"] | net.mqttHost; net.mqttPort = n["mqtt_port"] | net.mqttPort;
    net.mqttUser = n["mqtt_user"] | net.mqttUser; net.base = n["base_topic"] | net.base;
    net.discovery = n["discovery_prefix"] | net.discovery;
    std::string hosts;
    if (n["extra_hosts"].is<const char*>() && aok::parseHostList(n["extra_hosts"].as<const char*>(), hosts) >= 0) net.extraHosts = hosts.c_str();
    net.resyncEvery = constrain(static_cast<int>(n["resync_after"] | net.resyncEvery), 0, 100);
    net.resyncThrough = n["resync_through"] | net.resyncThrough;
  }
  JsonObjectConst p = d["pins"];
  if (!p.isNull()) {
    pins.sck = p["sck"] | pins.sck; pins.miso = p["miso"] | pins.miso; pins.mosi = p["mosi"] | pins.mosi;
    pins.cs = p["cs"] | pins.cs; pins.gdo0 = p["gdo0"] | pins.gdo0; pins.gdo2 = p["gdo2"] | pins.gdo2;
    pins.led = p["led"] | pins.led; pins.mhz = p["mhz"] | pins.mhz; pins.paDbm = p["pa_dbm"] | pins.paDbm;
    pins.rxBw = p["rx_bw"] | pins.rxBw;
  }
  if (!d["remote"].isNull()) remoteFromJson(d["remote"].as<JsonObjectConst>());
  if (!d["automation"].isNull() && !automationFromJson(d["automation"].as<JsonObjectConst>(), err)) LOGW("Backup schedules skipped: %s", err.c_str());
  saveNet(); savePins(); saveRemote(); saveBlinds(); savePositions(); saveAutomation();
  LOGI("Settings restored from a backup (%u blinds), restarting", static_cast<unsigned>(blindCount));
  scheduleReboot(1500);
  JsonDocument r; r["ok"] = true; r["rebooting"] = true; sendJson(r);
}

void handleAutomationGet() {
  if (!authed()) return;
  JsonDocument d; automationToJson(d.to<JsonObject>());
  if (!isnan(siteLat) && wallClock()) {
    const time_t now = time(nullptr); struct tm lt; localtime_r(&now, &lt);
    d["sunrise"] = aok::sunMinutes(true, lt.tm_yday, siteLat, siteLon, utcOffsetMinutes());
    d["sunset"] = aok::sunMinutes(false, lt.tm_yday, siteLat, siteLon, utcOffsetMinutes());
  }
  d["wind"] = windActive;
  sendJson(d);
}

void handleAutomationPost() {
  if (!authed()) return;
  JsonDocument d; if (!bodyJson(d)) return;
  String err;
  if (!automationFromJson(d.as<JsonObjectConst>(), err)) { sendError(err); return; }
  saveAutomation();
  discoveryPending = true;
  stateRev++;
  LOGI("Scenes and schedules saved: %u scenes, %u schedules", static_cast<unsigned>(sceneCount), static_cast<unsigned>(scheduleCount));
  sendOk();
}

void handleScenePost() {
  if (!authed()) return;
  JsonDocument d; if (!bodyJson(d)) return;
  String err;
  if (!runScene(d["id"] | "", "web", err)) { sendError(err); return; }
  sendOk();
}

void handleLockPost() {
  if (!authed()) return;
  JsonDocument d; if (!bodyJson(d)) return;
  const int i = findBlind(d["id"] | "");
  if (i < 0) { sendError("unknown blind"); return; }
  setLock(i, d["locked"] | false);
  sendOk();
}

// Pairing a channel's motor to the bridge's own ID. With the P button learned, the
// bridge does it all: P, P (the motor jogs each time), then Up from its own ID.
// Without it, press P twice on the hand remote, then "Pair now" sends the Up.
void stepPairing() {
  if (!pairChannel || static_cast<int32_t>(millis() - pairAtMs) < 0) return;
  const uint16_t mask = 1u << (pairChannel - 1);
  String err;
  if (pairStage == 0 || pairStage == 1) {
    if (!transmitPayload(aok::makePayload(remote.prefix, remote.remoteId, mask, remote.progBase), err)) { pairStatus = "failed: " + err; pairChannel = 0; return; }
    pairStatus = pairStage == 0 ? "sent P once: the motor should jog" : "sent P twice: the motor should jog again";
    pairStage++;
    pairAtMs = millis() + 2500;
    return;
  }
  if (!transmitPayload(aok::makePayload(remote.prefix, remote.ownId, mask, remote.base[0]), err)) { pairStatus = "failed: " + err; pairChannel = 0; return; }
  remote.ownMask |= mask;
  saveRemote();
  pairStatus = String("sent Up from the bridge's ID on channel ") + pairChannel + ": if the motor jogged, it's paired";
  LOGI("Pairing: %s", pairStatus.c_str());
  pairChannel = 0;
  stateRev++;
}

void handlePairPost() {
  if (!authed()) return;
  JsonDocument d; if (!bodyJson(d)) return;
  const String action = d["action"] | "";
  const int ch = d["channel"] | 0;
  if (action == "forget") {
    if (ch >= 1 && ch <= 16) remote.ownMask &= ~(1u << (ch - 1)); else remote.ownMask = 0;
    saveRemote(); stateRev++; LOGI("Pairing: channel %s back to the hand remote's ID", ch ? String(ch).c_str() : "all");
    sendOk(); return;
  }
  if (ch < 1 || ch > 16) { sendError("choose a channel 1-16"); return; }
  if (!remoteComplete()) { sendError("learn the remote first"); return; }
  if (anyMoving()) { sendError("wait until the blinds stop", 409); return; }
  if (!remote.ownId) {
    remote.ownId = static_cast<uint8_t>((remote.remoteId + 0x5A) & 0xFF);
    while (remote.ownId == remote.remoteId || remote.ownId == 0 || remoteKnown(remote.prefix, remote.ownId)) remote.ownId++;
    saveRemote();
  }
  pairChannel = ch;
  pairAtMs = millis();
  if (action == "auto") {
    if (!remote.haveProg) { pairChannel = 0; sendError("press the P button on the hand remote once first, so the bridge learns it"); return; }
    pairStage = 0; pairStatus = "starting";
  } else if (action == "now") {
    pairStage = 2; pairStatus = "sending Up from the bridge's ID";
  } else { pairChannel = 0; sendError("unknown action"); return; }
  LOGI("Pairing channel %d to the bridge's ID %s (%s)", ch, hex(remote.ownId, 2).c_str(), action.c_str());
  sendOk();
}

// Somfy RTS: pair a motor with the bridge, follow a hand remote, skip the rolling code ahead.
void handleSomfyPost() {
  if (!authed()) return;
  JsonDocument d; if (!bodyJson(d)) return;
  const String action = d["action"] | "";
  if (action == "cancel") { somfyLearn = -1; somfyStatus = "cancelled"; stateRev++; sendOk(); return; }
  const int i = findBlind(d["id"] | "");
  if (i < 0 || !cfg[i].somfy) { sendError("choose a Somfy blind"); return; }
  String err;
  if (action == "pair") {
    // The motor must be in programming mode: hold PROG on its remote until it jogs, then press Pair.
    if (anyMoving()) { sendError("wait until the blinds stop", 409); return; }
    if (!sendSomfy(i, somfy::Command::Prog, 8, err)) { sendError(err); return; }
    somfyStatus = "sent PROG for " + cfg[i].name + ": if the motor jogged, it's paired (do the same again to unpair)";
  } else if (action == "learn") {
    somfyLearn = i; somfyLearnMs = millis();
    somfyStatus = "press any button on the Somfy remote for " + cfg[i].name;
  } else if (action == "forget_remotes") {
    cfg[i].somfyRemoteCount = 0; saveBlinds();
    somfyStatus = "forgot the remotes for " + cfg[i].name;
  } else if (action == "resync") {
    // The motor ignores codes it has already seen. After restoring an old backup onto a new
    // board, skip ahead (the motor accepts a jump of up to about 100).
    const int by = constrain(static_cast<int>(d["by"] | 20), 1, 100);
    setSomfyCode(cfg[i].somfyAddr, static_cast<uint16_t>(somfyCode(cfg[i].somfyAddr) + by));
    somfyStatus = String("rolling code for ") + cfg[i].name + " moved on by " + by + " to " + somfyCode(cfg[i].somfyAddr);
  } else if (action == "send") {
    // raw button, for testing: up, down, my, prog
    const String b = d["button"] | "";
    const somfy::Command c = b == "up" ? somfy::Command::Up : b == "down" ? somfy::Command::Down : b == "my" ? somfy::Command::My
                           : b == "prog" ? somfy::Command::Prog : somfy::Command::Unknown;
    if (c == somfy::Command::Unknown) { sendError("button must be up, down, my or prog"); return; }
    if (!sendSomfy(i, c, c == somfy::Command::Prog ? 8 : 4, err)) { sendError(err); return; }
    if (c == somfy::Command::Up || c == somfy::Command::Down) blindApply(i, c == somfy::Command::Up ? aok::Action::Open : aok::Action::Close, "test", -1);
    else if (c == somfy::Command::My && st[i].motion != Motion::Stopped) blindApply(i, aok::Action::Stop, "test", -1);
    somfyStatus = String("sent ") + b + " to " + cfg[i].name;
  } else { sendError("unknown action"); return; }
  LOGI("Somfy: %s", somfyStatus.c_str());
  stateRev++;
  sendOk();
}

// Frequency scan: while someone holds a button on the hand remote, measure the
// signal across 433.42-434.42 MHz and report where it's strongest. One 180 ms step
// at a time from loop(), so the page, MQTT and blind timing keep running meanwhile.
constexpr int kScanSteps = 21;  // 50 kHz apart
int scanStep = -1;              // -1 idle, 0..kScanSteps-1 measuring, kScanSteps finished
int scanRssi[kScanSteps];
uint32_t scanStepMs = 0, scanDoneMs = 0, scanTxMs = 0;
String scanErr;
float scanMhz(int k) { return 433.42f + 0.05f * k; }

void finishFreqScan(const char* err) {
  scanErr = err ? err : "";
  scanStep = kScanSteps;
  scanDoneMs = millis();
  rxBwNow = 0;  // rxStart() puts the bandwidth and frequency back
  rxStart();
}

void stepFreqScan() {
  if (scanStep < 0 || scanStep >= kScanSteps) return;
  // a command sent mid-scan retunes the radio, so the rest of the readings would be wrong
  if (lastTxMs != scanTxMs) { finishFreqScan("a blind was sent a command during the scan: scan again"); return; }
  scanRssi[scanStep] = max(scanRssi[scanStep], ELECHOUSE_cc1101.getRssi());
  if (millis() - scanStepMs < 180) return;
  if (++scanStep == kScanSteps) { finishFreqScan(nullptr); return; }
  ELECHOUSE_cc1101.setMHZ(scanMhz(scanStep));
  ELECHOUSE_cc1101.SetRx(scanMhz(scanStep));
  scanRssi[scanStep] = -200;
  scanStepMs = millis();
}

// The page calls this until `scanning` is false: the first call starts, the last one gets the result.
void handleFreqScan() {
  if (!authed()) return;
  JsonDocument d;
  if (scanStep >= 0 && scanStep < kScanSteps) { d["scanning"] = true; sendJson(d); return; }
  if (scanStep == kScanSteps && millis() - scanDoneMs < 30000) {
    scanStep = -1;
    if (scanErr.length()) { sendError(scanErr, 409); return; }
    JsonArray pts = d["points"].to<JsonArray>();
    float best = pins.mhz; int bestRssi = -200;
    for (int k = 0; k < kScanSteps; ++k) {
      JsonObject o = pts.add<JsonObject>(); o["mhz"] = scanMhz(k); o["rssi"] = scanRssi[k];
      if (scanRssi[k] > bestRssi) { bestRssi = scanRssi[k]; best = scanMhz(k); }
    }
    d["scanning"] = false; d["best_mhz"] = best; d["best_rssi"] = bestRssi; d["current_mhz"] = pins.mhz;
    LOGI("Frequency scan: strongest at %.2f MHz (%d dBm)", best, bestRssi);
    sendJson(d);
    return;
  }
  if (!radioReady) { sendError("the radio isn't working"); return; }
  if (anyMoving()) { sendError("wait until the blinds stop", 409); return; }
  rxStop();
  ELECHOUSE_cc1101.setRxBW(270.0f);  // narrow, so the peak shows where the remote really is
  ELECHOUSE_cc1101.setMHZ(scanMhz(0));
  ELECHOUSE_cc1101.SetRx(scanMhz(0));
  scanStep = 0; scanRssi[0] = -200; scanStepMs = millis(); scanTxMs = lastTxMs;
  d["scanning"] = true;
  sendJson(d);
}

void handleReboot() { if (!authed(true)) return; LOGI("Restart requested from the web page"); sendOk(); scheduleReboot(800); }

void handleLog() {
  if (!authed(true)) return;
  const uint32_t after = web.hasArg("after") ? strtoul(web.arg("after").c_str(), nullptr, 10) : 0;
  const bool text = web.hasArg("format") && web.arg("format") == "text";
  const uint32_t first = logSeq > kLogLines ? logSeq - kLogLines + 1 : 1;
  if (text) {
    String out;
    for (uint32_t s = first; s <= logSeq; ++s) {
      const LogLine& l = logLines[(s - 1) % kLogLines];
      char head[40];
      if (l.when) { struct tm tmv; localtime_r(&l.when, &tmv); strftime(head, sizeof(head), "%Y-%m-%d %H:%M:%S", &tmv); }
      else snprintf(head, sizeof(head), "+%lu.%03lus", static_cast<unsigned long>(l.ms / 1000), static_cast<unsigned long>(l.ms % 1000));
      out += head; out += ' '; out += l.level; out += ' '; out += l.text; out += '\n';
    }
    web.sendHeader("Content-Disposition", "attachment; filename=blind-bridge-log.txt");
    web.send(200, "text/plain", out);
    return;
  }
  JsonDocument d;
  d["seq"] = logSeq; d["uptime_ms"] = millis(); d["time"] = static_cast<uint32_t>(wallClock());
  JsonArray a = d["lines"].to<JsonArray>();
  for (uint32_t s = max(first, after + 1); s <= logSeq; ++s) {
    const LogLine& l = logLines[(s - 1) % kLogLines];
    JsonObject o = a.add<JsonObject>();
    o["seq"] = l.seq; o["ms"] = l.ms; o["level"] = String(l.level); o["text"] = l.text;
    if (l.when) o["when"] = static_cast<uint32_t>(l.when);
  }
  sendJson(d);
}

bool updateAllowed = false;  // set at the start of an upload once it has passed the checks

void handleUpdateDone() {
  if (!authed()) return;
  const bool ok = updateAllowed && !Update.hasError();
  updateAllowed = false;
  if (ok) LOGI("Firmware updated, restarting"); else LOGE("Firmware update failed: %s", Update.errorString());
  web.send(ok ? 200 : 500, "text/plain", ok ? "Update OK, restarting" : "Update failed");
  if (ok) scheduleReboot(1000);
}

void handleUpdateUpload() {
  HTTPUpload& up = web.upload();
  if (up.status == UPLOAD_FILE_START) {
    updateAllowed = web.header("X-Bridge") == "1" && hostAllowed(web.hostHeader()) &&
                    net.adminPass.length() && web.authenticate("admin", net.adminPass.c_str());
    if (!updateAllowed) { LOGW("Refused a firmware upload: it needs the admin password, set from the bridge's own page"); return; }
    LOGI("Firmware upload started: %s", up.filename.c_str());
    rxStop();
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      LOGE("Firmware update can't start: %s", Update.errorString());
      updateAllowed = false;
      rxStart();
    }
    return;
  }
  if (!updateAllowed) return;
  if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.progress() == 0 && up.currentSize && up.buf[0] != 0xE9) {  // ESP32 app images start with 0xE9
      LOGE("That file isn't ESP32 firmware (use the .bin from the release, not the -full or -factory image)");
      Update.abort();
      updateAllowed = false;
      rxStart();
      return;
    }
    if (Update.write(up.buf, up.currentSize) != up.currentSize) {
      LOGE("Firmware write failed: %s", Update.errorString());
      Update.abort();
      updateAllowed = false;
      rxStart();
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (!Update.end(true)) { LOGE("Firmware check failed: %s", Update.errorString()); rxStart(); }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    updateAllowed = false;
    rxStart();
  }
}

void handleRoot() {
  // the page only changes with the firmware, so browsers can reuse their copy
  const String etag = String("\"") + FW_VERSION + "-" + WEB_UI_GZ_LEN + "\"";
  web.sendHeader("Cache-Control", "no-cache");
  web.sendHeader("ETag", etag);
  if (web.header("If-None-Match") == etag) { web.send(304); return; }
  web.sendHeader("Content-Encoding", "gzip");
  web.send_P(200, "text/html", reinterpret_cast<const char*>(WEB_UI_GZ), WEB_UI_GZ_LEN);
}

void handleNotFound() {
  if (apMode) { web.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true); web.send(302, "text/plain", ""); return; }
  web.send(404, "text/plain", "not found");
}

void setupWeb() {
  static const char* kHeaders[] = {"If-None-Match", "X-Bridge"};
  web.collectHeaders(kHeaders, 2);
  web.on("/", HTTP_GET, handleRoot);
  web.on("/api/backup", HTTP_GET, handleBackup);
  web.on("/api/restore", HTTP_POST, handleRestore);
  web.on("/api/state", HTTP_GET, handleState);
  web.on("/api/config", HTTP_GET, handleConfigGet);
  web.on("/api/network", HTTP_POST, handleNetworkPost);
  web.on("/api/pins", HTTP_POST, handlePinsPost);
  web.on("/api/remote", HTTP_POST, handleRemotePost);
  web.on("/api/blinds", HTTP_POST, handleBlindsPost);
  web.on("/api/command", HTTP_POST, handleCommandPost);
  web.on("/api/test", HTTP_POST, handleTestPost);
  web.on("/api/calibrate", HTTP_POST, handleCalibratePost);
  web.on("/api/reboot", HTTP_POST, handleReboot);
  web.on("/api/scan", HTTP_GET, handleScan);
  web.on("/api/mqtt/discover", HTTP_GET, handleMqttDiscover);
  web.on("/api/mqtt/test", HTTP_POST, handleMqttTest);
  web.on("/api/log", HTTP_GET, handleLog);
  web.on("/api/automation", HTTP_GET, handleAutomationGet);
  web.on("/api/automation", HTTP_POST, handleAutomationPost);
  web.on("/api/scene", HTTP_POST, handleScenePost);
  web.on("/api/lock", HTTP_POST, handleLockPost);
  web.on("/api/pair", HTTP_POST, handlePairPost);
  web.on("/api/somfy", HTTP_POST, handleSomfyPost);
  web.on("/api/freqscan", HTTP_POST, handleFreqScan);
  web.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  // Phones and Windows probe these when they join the setup Wi-Fi; answering them
  // directly opens the setup page and stops "request handler not found" log spam.
  for (const char* probe : {"/generate_204", "/gen_204", "/hotspot-detect.html", "/library/test/success.html",
                            "/connecttest.txt", "/ncsi.txt", "/redirect", "/fwlink", "/canonical.html",
                            "/success.txt", "/favicon.ico"}) {
    web.on(probe, HTTP_ANY, handleNotFound);
  }
  web.onNotFound(handleNotFound);
  web.begin();
  newWsToken();
  static const char* kWsHeaders[] = {"Host"};
  ws.onValidateHttpHeader([](String name, String value) { return !name.equalsIgnoreCase("Host") || hostAllowed(value); },
                          kWsHeaders, 1);
  ws.begin();
  ws.onEvent([](uint8_t num, WStype_t type, uint8_t* url, size_t len) {
    if (type != WStype_CONNECTED) return;
    // with a password set, only pages that passed the login (and so have the token) get the state
    if (net.adminPass.length() && String(reinterpret_cast<const char*>(url), len).indexOf("t=" + wsToken) < 0) {
      ws.disconnect(num);
      return;
    }
    lastWsRev = 0;
  });
}

// ---------------------------------------------------------------- Improv Wi-Fi (serial)
// https://www.improv-wifi.com/serial/: lets the web installer ask for Wi-Fi straight after flashing.
namespace improv {
enum : uint8_t { TypeState = 1, TypeError = 2, TypeRpc = 3, TypeRpcResult = 4 };
enum : uint8_t { StateReady = 2, StateProvisioning = 3, StateProvisioned = 4 };
enum : uint8_t { ErrNone = 0, ErrInvalid = 1, ErrUnknownCmd = 2, ErrConnect = 3 };
enum : uint8_t { CmdWifi = 1, CmdState = 2, CmdInfo = 3, CmdScan = 4 };
uint8_t buf[256];
size_t len = 0;
bool provisioning = false;
uint32_t provisionStartMs = 0;

void send(uint8_t type, const uint8_t* data, size_t n) {
  uint8_t pkt[268];
  size_t k = 0;
  memcpy(pkt, "IMPROV", 6); k = 6;
  pkt[k++] = 1; pkt[k++] = type; pkt[k++] = static_cast<uint8_t>(n);
  memcpy(pkt + k, data, n); k += n;
  uint8_t sum = 0;
  for (size_t i = 0; i < k; ++i) sum += pkt[i];
  pkt[k++] = sum; pkt[k++] = '\n';
  Serial.write(pkt, k);
}
void state(uint8_t s) { send(TypeState, &s, 1); }
void error(uint8_t e) { send(TypeError, &e, 1); }
void result(uint8_t cmd, std::initializer_list<String> strings) {
  uint8_t d[250];
  size_t k = 2;
  for (const String& s : strings) {
    const size_t l = min<size_t>(s.length(), sizeof(d) - k - 1);
    d[k++] = static_cast<uint8_t>(l); memcpy(d + k, s.c_str(), l); k += l;
  }
  d[0] = cmd; d[1] = static_cast<uint8_t>(k - 2);
  send(TypeRpcResult, d, k);
}
String url() { return String("http://") + netIp().toString(); }

void handle(const uint8_t* d, size_t n) {
  if (n < 2 || static_cast<size_t>(d[1]) + 2 > n) { error(ErrInvalid); return; }
  switch (d[0]) {
    case CmdState:
      if (netUp()) { state(StateProvisioned); result(CmdState, {url()}); }
      else state(StateReady);
      break;
    case CmdInfo:
      result(CmdInfo, {"AOK Blind Bridge", FW_VERSION, "ESP32", net.hostname});
      break;
    case CmdScan: {
      const int count = WiFi.scanNetworks(false, false);
      for (int i = 0; i < count; ++i)
        result(CmdScan, {WiFi.SSID(i), String(WiFi.RSSI(i)), WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "NO" : "YES"});
      result(CmdScan, {});
      WiFi.scanDelete();
      break;
    }
    case CmdWifi: {
      const uint8_t* p = d + 2;
      const size_t sl = p[0];
      if (sl + 2 > d[1]) { error(ErrInvalid); return; }
      const String ssid(reinterpret_cast<const char*>(p + 1), sl);
      const size_t pl = p[1 + sl];
      if (2 + sl + pl > d[1]) { error(ErrInvalid); return; }
      const String pass(reinterpret_cast<const char*>(p + 2 + sl), pl);
      net.ssid = ssid; net.pass = pass; saveNet();
      LOGI("Wi-Fi set over USB (Improv): %s", ssid.c_str());
      error(ErrNone);
      state(StateProvisioning);
      provisioning = true; provisionStartMs = millis();
      WiFi.mode(apMode ? WIFI_AP_STA : WIFI_STA);
      WiFi.begin(net.ssid.c_str(), net.pass.c_str());
      staStartMs = millis();
      break;
    }
    default: error(ErrUnknownCmd);
  }
}

void poll() {
  while (Serial.available()) {
    const uint8_t b = Serial.read();
    if (len < 6) {
      if (b == static_cast<uint8_t>("IMPROV"[len])) buf[len++] = b;
      else len = b == 'I' ? (buf[0] = b, 1) : 0;
      continue;
    }
    buf[len++] = b;
    if (len < 9) continue;
    const size_t total = 9 + buf[8] + 1;
    if (len < total && len < sizeof(buf)) continue;
    if (len == total) {
      uint8_t sum = 0;
      for (size_t i = 0; i + 1 < total; ++i) sum += buf[i];
      if (sum == buf[total - 1] && buf[6] == 1 && buf[7] == TypeRpc) handle(buf + 9, buf[8]);
    }
    len = 0;
  }
  if (provisioning) {
    if (netUp()) { state(StateProvisioned); result(CmdWifi, {url()}); provisioning = false; }
    else if (millis() - provisionStartMs > 20000) { error(ErrConnect); state(StateReady); provisioning = false; }
  }
}
}  // namespace improv

// ---------------------------------------------------------------- Wi-Fi
void startAp() {
  if (apMode) return;
  String brandSlug;
  for (char c : net.brand) if (isalnum(static_cast<unsigned char>(c))) brandSlug += c;
  if (!brandSlug.length()) brandSlug = "AOK";
  const String name = brandSlug + "-Blinds-" + String(static_cast<uint32_t>(ESP.getEfuseMac() >> 24) & 0xFFFF, HEX);
  WiFi.mode(WIFI_AP_STA);  // STA side stays up so the setup page can scan for networks
  WiFi.softAP(name.c_str());
  dns.start(53, "*", WiFi.softAPIP());
  apMode = true;
  LOGI("Setup Wi-Fi '%s' open at http://%s", name.c_str(), WiFi.softAPIP().toString().c_str());
}

void stopAp() {
  if (!apMode) return;
  dns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  apMode = false;
}

#ifdef USE_ETH
void onEthEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_ETH_START) ETH.setHostname(net.hostname.c_str());
  else if (event == ARDUINO_EVENT_ETH_GOT_IP) { ethUp = true; LOGI("Ethernet up, IP %s", ETH.localIP().toString().c_str()); }
  else if (event == ARDUINO_EVENT_ETH_DISCONNECTED || event == ARDUINO_EVENT_ETH_STOP) { if (ethUp) LOGW("Ethernet down"); ethUp = false; }
}
#endif

void setupWifi() {
  WiFi.persistent(false);
  WiFi.setHostname(net.hostname.c_str());
#ifdef USE_ETH
  WiFi.onEvent(onEthEvent);
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_TYPE, ETH_CLK_MODE);
  if (!net.ssid.length()) { staStartMs = millis(); return; }  // the setup Wi-Fi opens only if the cable has no network
#endif
  if (!net.ssid.length()) { startAp(); return; }
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(net.ssid.c_str(), net.pass.c_str());
  staStartMs = millis();
}

void maintainWifi() {
  const bool up = netUp();
  static bool wasUp = false;
  if (up != wasUp) {
    wasUp = up;
    if (up && WiFi.status() == WL_CONNECTED) WiFi.setSleep(false);  // modem sleep holds incoming MQTT commands until the next wake-up
    if (up) LOGI("Wi-Fi connected to %s, IP %s, signal %d dBm", net.ssid.c_str(), netIp().toString().c_str(), WiFi.RSSI());
    else if (net.ssid.length()) LOGW("Wi-Fi disconnected from %s", net.ssid.c_str());
  }
  if (up) {
    if (!ntpStarted) { configTzTime(net.tz.length() ? net.tz.c_str() : "UTC0", "pool.ntp.org", "time.google.com"); ntpStarted = true; }
    if (apMode && millis() - staStartMs > 5000) stopAp();
    if (!mdnsReady && MDNS.begin(net.hostname.c_str())) { MDNS.addService("http", "tcp", 80); mdnsReady = true; }
    if (!otaReady && net.adminPass.length()) {  // PlatformIO/espota updates need the admin password
      ArduinoOTA.setHostname(net.hostname.c_str());
      ArduinoOTA.setPassword(net.adminPass.c_str());
      ArduinoOTA.onStart([]() { rxStop(); });
      ArduinoOTA.begin();
      otaReady = true;
    }
#ifdef USE_ETH
  } else if (!net.ssid.length() && !apMode && millis() - staStartMs > 30000) {
    LOGW("No network on the Ethernet cable after 30 s: opening the setup Wi-Fi");
    startAp();
#endif
  } else if (net.ssid.length() && !apMode && millis() - staStartMs > 45000) {
    LOGW("Can't join %s after 45 s: opening the setup Wi-Fi as well", net.ssid.c_str());
    startAp();  // can't join: open the setup portal but keep trying the home network
  }
  if (apMode) dns.processNextRequest();
}

void updateLed() {
  if (pins.led < 0) return;
  const uint32_t now = millis();
  bool on;
  if (now - lastRxFlashMs < 120) on = true;
  else if (apMode) on = (now / 150) % 2;                         // fast blink: setup portal
  else if (WiFi.status() != WL_CONNECTED) on = (now / 600) % 2;   // slow blink: joining Wi-Fi
  else if (!radioReady) on = (now / 1000) % 4 == 0;               // blip: radio missing
  else on = net.mqttHost.length() && !mqtt.connected() ? (now / 1000) % 2 : true;
  ledSet(on);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println();
  prefs.begin("aok", false);
  loadNet();
  loadPins();
  loadRemote();
  loadBlinds();
  loadPositions();
  loadCalHistory();
  loadAutomation();
  loadLocks();
  {
    // Safe mode: after 3 crashes in a row, start without the radio and MQTT so the
    // page and updates still work and a bad setting can be fixed.
    const esp_reset_reason_t rr = esp_reset_reason();
    if (crashMagic != 0xB11DB00Bu) { crashMagic = 0xB11DB00Bu; crashCount = 0; }
    const bool crashed = rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT;
    crashCount = crashed ? crashCount + 1 : 0;
    safeMode = crashCount >= 3;
    if (safeMode) LOGE("Safe mode: the bridge crashed %u times in a row. Radio and MQTT are off; fix the settings or update the firmware, then restart.", static_cast<unsigned>(crashCount));
  }
  {
    const esp_reset_reason_t rr = esp_reset_reason();
    const char* why = rr == ESP_RST_POWERON ? "power on" : rr == ESP_RST_SW ? "restart" : rr == ESP_RST_BROWNOUT ? "low voltage"
      : (rr == ESP_RST_PANIC ? "crash" : (rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT) ? "watchdog" : "other");
    LOGI("%s Blind Bridge %s started after %s, %u blinds", net.brand.c_str(), FW_VERSION, why, static_cast<unsigned>(blindCount));
  }
  if (pins.led >= 0) { pinMode(pins.led, OUTPUT); ledSet(true); }
  if (!safeMode) setupRadio();
  setupWifi();
  setupWeb();
  mqtt.setBufferSize(6144);  // a blind or the bridge in one device discovery message
  mqtt.setCallback(onMqtt);
}

void loop() {
  pollRadio();
  improv::poll();
  maintainWifi();
  web.handleClient();
  if (otaReady) ArduinoOTA.handle();
  ensureMqtt();
  if (mqtt.connected()) mqtt.loop();
  if (mqtt.connected() && discoveryPending) publishDiscovery();  // scene changes, and the move to device discovery
  pollRadio();
  updateMotion();
  finishPress();
  checkSchedules();
  stepPairing();
  stepFreqScan();
  ws.loop();
  pushLive();
  if (crashCount && millis() > 120000) crashCount = 0;  // ran for 2 minutes: the crash streak is over
  if (mqtt.connected() && millis() - lastDiagMs > 30000) publishDiagnostics();
  if (calIndex >= 0 && millis() - calStartMs > 300000) calIndex = -1;
  updateLed();
  if (positionsDirty && millis() - positionsDirtyMs > 3000) {
    bool still = true;
    for (size_t i = 0; i < blindCount; ++i) still &= st[i].motion == Motion::Stopped;
    if (still) savePositions();
  }
  if (rebootPending && static_cast<int32_t>(millis() - rebootAtMs) >= 0) { savePositions(); ESP.restart(); }
}
