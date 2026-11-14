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
#include <WebServer.h>
#include <WiFi.h>
#include <time.h>
#include <stdarg.h>

#include "aok_protocol.h"
#include "web_ui_gz.h"

#define FW_VERSION "1.2.0"
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

enum class Motion : uint8_t { Stopped, Opening, Closing };

struct Pins {
  int sck = DEFAULT_PIN_SCK, miso = DEFAULT_PIN_MISO, mosi = DEFAULT_PIN_MOSI, cs = DEFAULT_PIN_CS;
  int gdo0 = DEFAULT_PIN_GDO0, gdo2 = DEFAULT_PIN_GDO2, led = DEFAULT_PIN_LED;
  float mhz = 433.92f;
  int paDbm = 10;
};

struct NetCfg {
  String ssid, pass, hostname = "aok-blinds", adminPass;
  String brand = "A-OK";            // shown in the page title, setup Wi-Fi name and Home Assistant
  String tz, tzName;                // POSIX time zone string and the IANA name it came from
  String mqttHost;
  uint16_t mqttPort = 1883;
  String mqttUser, mqttPass, base = "aok_blinds", discovery = "homeassistant";
};

struct RemoteCfg {
  bool have[3] = {false, false, false};
  uint32_t prefix = 0;
  uint8_t remoteId = 0;
  uint16_t base[3] = {0, 0, 0};
  aok::Timings timings;
  bool listen = true;
  uint16_t seen = 0;  // channels this remote has been heard on
};

struct BlindCfg {
  String id, name, room, deviceClass = "shade";
  uint16_t mask = 1;
  uint32_t openMs = 30000, closeMs = 30000;
  bool invert = false;
  int favourite = -1;
};

struct BlindState {
  int position = -1;  // 0 closed .. 100 open, -1 unknown
  int startPos = -1;
  int target = -1;
  Motion motion = Motion::Stopped;
  uint32_t startMs = 0, durMs = 0;
  bool autoStop = false;
  uint32_t lastPublishMs = 0;
  String lastSource = "none";
  bool dirty = true;
};

struct HeardFrame {
  uint32_t ms = 0;
  aok::Frame f;
  bool matched = false;
  uint16_t repeats = 1;
};

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
uint32_t stateRev = 1;               // bumps when frames, calibration history or config change (page refetches)
bool positionsDirty = false;         // positions are written to flash a few seconds after things settle
uint32_t positionsDirtyMs = 0;
uint32_t learnStartedMs = 0, rebootAtMs = 0, lastMqttTryMs = 0, lastDiagMs = 0, staStartMs = 0;
uint32_t lastTxMs = 0, lastRxFlashMs = 0;
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

// ---------------------------------------------------------------- helpers
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
  // settings saved by 1.0.x/1.1.x: keep what those versions did
  if (!net.brand.length()) net.brand = net.hostname.indexOf("motolux") >= 0 ? "Motolux" : "A-OK";
  if (upgraded && !d["tz"].is<const char*>()) { net.tz = "AEST-10AEDT,M10.1.0,M4.1.0/3"; net.tzName = "Australia/Melbourne"; }
}

void savePins() {
  JsonDocument d;
  d["sck"] = pins.sck; d["miso"] = pins.miso; d["mosi"] = pins.mosi; d["cs"] = pins.cs;
  d["gdo0"] = pins.gdo0; d["gdo2"] = pins.gdo2; d["led"] = pins.led; d["mhz"] = pins.mhz; d["pa"] = pins.paDbm;
  String s; serializeJson(d, s); prefs.putString("pins", s);
}

void loadPins() {
  JsonDocument d;
  if (deserializeJson(d, prefs.getString("pins", "{}"))) return;
  pins.sck = d["sck"] | pins.sck; pins.miso = d["miso"] | pins.miso; pins.mosi = d["mosi"] | pins.mosi;
  pins.cs = d["cs"] | pins.cs; pins.gdo0 = d["gdo0"] | pins.gdo0; pins.gdo2 = d["gdo2"] | pins.gdo2;
  pins.led = d["led"] | pins.led; pins.mhz = d["mhz"] | pins.mhz; pins.paDbm = d["pa"] | pins.paDbm;
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

void blindToJson(size_t i, JsonObject o) {
  o["id"] = cfg[i].id; o["name"] = cfg[i].name; o["channels"] = maskText(cfg[i].mask);
  o["open_ms"] = cfg[i].openMs; o["close_ms"] = cfg[i].closeMs; o["invert"] = cfg[i].invert;
  o["favourite"] = cfg[i].favourite; o["device_class"] = cfg[i].deviceClass; o["room"] = cfg[i].room;
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
    b.mask = o["channels"].is<const char*>() ? parseChannels(o["channels"].as<String>()) : (o["mask"] | 0);
    if (!b.mask) { err = b.name + ": choose at least one channel (1-16)"; return false; }
    b.openMs = constrain(static_cast<uint32_t>(o["open_ms"] | 30000), 1000u, 300000u);
    b.closeMs = constrain(static_cast<uint32_t>(o["close_ms"] | 30000), 1000u, 300000u);
    b.invert = o["invert"] | false;
    b.favourite = o["favourite"] | -1;
    if (b.favourite > 100) b.favourite = 100;
    b.deviceClass = o["device_class"] | "shade";
    b.room = o["room"] | "";
    b.room.trim();
    next[n++] = b;
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
  for (size_t i = 0; i < blindCount; ++i) if (st[i].motion == Motion::Stopped) d[cfg[i].id] = st[i].position;
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

void rxStart() {
  if (!radioReady) return;
  ELECHOUSE_cc1101.SetRx(pins.mhz);
  decoder.clear();
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
  ELECHOUSE_cc1101.setRxBW(270.0f);
  ELECHOUSE_cc1101.setDRate(4.8f);
  ELECHOUSE_cc1101.setSyncMode(0);
  ELECHOUSE_cc1101.setCrc(false);
  ELECHOUSE_cc1101.setPktFormat(3);
  // GDO2 = received data out; GDO0 = high impedance in RX (it becomes the TX data input when transmitting)
  ELECHOUSE_cc1101.SpiWriteReg(0x00, 0x0D);  // IOCFG2
  ELECHOUSE_cc1101.SpiWriteReg(0x02, 0x2E);  // IOCFG0
  LOGI("CC1101 radio ready at %.2f MHz", pins.mhz);
  rxStart();
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
  const uint64_t payload = aok::makePayload(remote.prefix, remote.remoteId, mask, remote.base[a]);
  LOGI("Sent %s on channel %s", action == aok::Action::Open ? "Up" : action == aok::Action::Close ? "Down" : "Stop", maskText(mask).c_str());
  return transmitPayload(payload, err);
}

// ---------------------------------------------------------------- position model
uint32_t travelMs(size_t i, Motion m) { return max<uint32_t>(m == Motion::Closing ? cfg[i].closeMs : cfg[i].openMs, 1000); }

int estimate(size_t i) {
  const BlindState& s = st[i];
  if (s.motion == Motion::Stopped || s.startPos < 0) return s.position;
  const uint32_t elapsed = millis() - s.startMs;
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
  int cur = estimate(i);
  if (cur < 0) cur = (m == Motion::Opening) ? 0 : 100;  // unknown: assume the far end so the full run is covered
  s.position = cur;
  s.startPos = cur;
  s.target = clampPos(target);
  s.motion = m;
  s.startMs = millis();
  s.durMs = static_cast<uint32_t>(abs(s.target - cur)) * travelMs(i, m) / 100;
  if (!autoStop) s.durMs += 1500;  // endpoint runs: motor stops itself at its limit
  s.autoStop = autoStop;
  s.lastSource = source;
  s.dirty = true;
}

void haltMotion(size_t i, const String& source) {
  BlindState& s = st[i];
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
void applyAction(uint16_t mask, aok::Action action, const String& source, int commander, int commanderTarget) {
  for (size_t j = 0; j < blindCount; ++j) {
    if ((cfg[j].mask & ~mask) != 0) continue;
    if (action == aok::Action::Stop) { haltMotion(j, source); continue; }
    const Motion m = motionFor(j, action);
    if (static_cast<int>(j) == commander && commanderTarget > 0 && commanderTarget < 100)
      beginMotion(j, m, commanderTarget, true, source);
    else
      beginMotion(j, m, m == Motion::Opening ? 100 : 0, false, source);
  }
  requestPositionSave();
}

bool groupPosition(size_t g, int& pos, Motion& motion);

bool moveTo(size_t i, int target, const String& source, String& err) {
  target = clampPos(target);
  int gp; Motion gm;
  if (__builtin_popcount(cfg[i].mask) > 1 && groupPosition(i, gp, gm) && gm == Motion::Stopped && st[i].motion == Motion::Stopped) {
    st[i].position = st[i].startPos = gp;  // a group starts from where its members actually are
  }
  int cur = estimate(i);
  if (cur < 0 && target > 0 && target < 100) {
    err = "position unknown — open or close fully once (or use Mark open/closed)";
    return false;
  }
  if (cur == target && st[i].motion == Motion::Stopped) { st[i].dirty = true; return true; }
  if (cur < 0) cur = target == 100 ? 0 : 100;
  const Motion m = target > cur ? Motion::Opening : Motion::Closing;
  if (target > 0 && target < 100 && !remote.have[2]) { err = "learn the Stop button first"; return false; }
  if (!sendAction(cfg[i].mask, rfFor(i, m), err)) return false;
  applyAction(cfg[i].mask, rfFor(i, m), source, static_cast<int>(i), target);
  return true;
}

bool stopBlind(size_t i, const String& source, String& err) {
  if (!sendAction(cfg[i].mask, aok::Action::Stop, err)) return false;
  applyAction(cfg[i].mask, aok::Action::Stop, source, static_cast<int>(i), -1);
  return true;
}

// members of a group = other blinds whose channels are a subset of the group's channels
bool groupPosition(size_t g, int& pos, Motion& motion) {
  int sum = 0, count = 0;
  bool opening = false, closing = false;
  for (size_t j = 0; j < blindCount; ++j) {
    if (j == g || (cfg[j].mask & ~cfg[g].mask) != 0 || cfg[j].mask == cfg[g].mask) continue;
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
  if (__builtin_popcount(cfg[i].mask) > 1 && groupPosition(i, gp, gm)) { pos = gp; motion = gm; }
}

// ---------------------------------------------------------------- MQTT
String topic(const String& suffix) { return net.base + "/" + suffix; }
String blindTopic(size_t i, const char* suffix) { return net.base + "/" + cfg[i].id + "/" + suffix; }
String nodeId() { return net.base; }

void deviceJson(JsonObject o) {
  JsonObject dev = o["device"].to<JsonObject>();
  dev["identifiers"].to<JsonArray>().add(nodeId());
  dev["name"] = net.brand + " Blind Bridge";
  dev["manufacturer"] = "aok-blind-bridge";
  dev["model"] = "ESP32 + CC1101";
  dev["sw_version"] = FW_VERSION;
  if (WiFi.status() == WL_CONNECTED) dev["configuration_url"] = String("http://") + WiFi.localIP().toString();
  o["availability_topic"] = topic("status");
  JsonObject origin = o["origin"].to<JsonObject>();
  origin["name"] = "aok-blind-bridge"; origin["sw_version"] = FW_VERSION; origin["support_url"] = PROJECT_URL;
}

void publishJson(const String& t, JsonDocument& d, bool retain = true) {
  String s; serializeJson(d, s);
  mqtt.publish(t.c_str(), s.c_str(), retain);
}

void publishDiscovery() {
  const String disc = net.discovery;
  JsonDocument ids;
  for (size_t i = 0; i < blindCount; ++i) {
    const String uid = nodeId() + "_" + cfg[i].id;
    {
      JsonDocument d;
      d["name"] = nullptr;  // entity takes the device name below
      d["unique_id"] = uid;
      d["default_entity_id"] = String("cover.") + cfg[i].id;
      d["command_topic"] = blindTopic(i, "set");
      d["set_position_topic"] = blindTopic(i, "set_position");
      d["position_topic"] = blindTopic(i, "position");
      d["state_topic"] = blindTopic(i, "state");
      d["json_attributes_topic"] = blindTopic(i, "attributes");
      d["payload_open"] = "OPEN"; d["payload_close"] = "CLOSE"; d["payload_stop"] = "STOP";
      d["position_open"] = 100; d["position_closed"] = 0;
      d["optimistic"] = false;
      d["device_class"] = cfg[i].deviceClass;
      deviceJson(d.as<JsonObject>());
      // each blind is its own device so it can sit in its own area
      JsonObject dev = d["device"];
      dev["identifiers"][0] = uid;
      dev["name"] = cfg[i].name;
      dev["via_device"] = nodeId();
      dev["model"] = net.brand + " motor · channel " + maskText(cfg[i].mask);
      if (cfg[i].room.length()) dev["suggested_area"] = cfg[i].room;
      publishJson(disc + "/cover/" + uid + "/config", d);
    }
    struct Btn { const char* key; const char* name; const char* payload; const char* icon; };
    const Btn btns[] = {
      {"favourite", "Favourite position", "FAVOURITE", "mdi:star"},
      {"mark_open", "Mark as open", "SYNC_OPEN", "mdi:arrow-collapse-up"},
      {"mark_closed", "Mark as closed", "SYNC_CLOSE", "mdi:arrow-collapse-down"},
    };
    for (const Btn& b : btns) {
      const String buid = uid + "_" + b.key;
      const String t = disc + "/button/" + buid + "/config";
      if (strcmp(b.key, "favourite") == 0 && cfg[i].favourite < 0) { mqtt.publish(t.c_str(), "", true); continue; }
      JsonDocument d;
      d["name"] = b.name; d["unique_id"] = buid; d["command_topic"] = blindTopic(i, "set");
      d["payload_press"] = b.payload; d["icon"] = b.icon;
      if (strcmp(b.key, "favourite") != 0) d["entity_category"] = "config";
      deviceJson(d.as<JsonObject>());
      JsonObject dev = d["device"];
      dev["identifiers"][0] = uid; dev["name"] = cfg[i].name; dev["via_device"] = nodeId();
      publishJson(t, d);
    }
    ids.add(cfg[i].id);
  }
  // remove entities for blinds that were deleted
  JsonDocument old;
  if (!deserializeJson(old, prefs.getString("pubids", "[]"))) {
    for (JsonVariantConst v : old.as<JsonArrayConst>()) {
      const String id = v.as<String>();
      if (findBlind(id) >= 0) continue;
      const String uid = nodeId() + "_" + id;
      mqtt.publish((disc + "/cover/" + uid + "/config").c_str(), "", true);
      for (const char* k : {"favourite", "mark_open", "mark_closed"})
        mqtt.publish((disc + "/button/" + uid + "_" + k + "/config").c_str(), "", true);
      for (const char* s : {"state", "position", "attributes", "set", "set_position"})
        mqtt.publish((net.base + "/" + id + "/" + s).c_str(), "", true);
    }
  }
  String s; serializeJson(ids, s); prefs.putString("pubids", s);

  // bridge entities
  auto sensor = [&](const char* key, const char* name, const char* unit, const char* devClass, const char* icon, bool diag) {
    JsonDocument d;
    d["name"] = name; d["unique_id"] = nodeId() + "_" + key; d["state_topic"] = topic(String("diag/") + key);
    if (unit) d["unit_of_measurement"] = unit;
    if (devClass) d["device_class"] = devClass;
    if (icon) d["icon"] = icon;
    if (diag) d["entity_category"] = "diagnostic";
    deviceJson(d.as<JsonObject>());
    publishJson(disc + "/sensor/" + nodeId() + "_" + key + "/config", d);
  };
  sensor("rssi", "Wi-Fi signal", "dBm", "signal_strength", nullptr, true);
  sensor("uptime", "Uptime", "s", "duration", nullptr, true);
  sensor("ip", "IP address", nullptr, nullptr, "mdi:ip-network", true);
  sensor("last_remote", "Last remote press", nullptr, nullptr, "mdi:remote", false);
  {
    JsonDocument d;
    d["name"] = "CC1101 radio"; d["unique_id"] = nodeId() + "_radio"; d["state_topic"] = topic("diag/radio");
    d["payload_on"] = "ON"; d["payload_off"] = "OFF"; d["device_class"] = "connectivity"; d["entity_category"] = "diagnostic";
    deviceJson(d.as<JsonObject>());
    publishJson(disc + "/binary_sensor/" + nodeId() + "_radio/config", d);
  }
  {
    JsonDocument d;
    d["name"] = "Remote button"; d["unique_id"] = nodeId() + "_remote_event"; d["state_topic"] = topic("remote/event");
    JsonArray et = d["event_types"].to<JsonArray>();
    et.add("open"); et.add("close"); et.add("stop");
    d["icon"] = "mdi:remote";
    deviceJson(d.as<JsonObject>());
    publishJson(disc + "/event/" + nodeId() + "_remote_event/config", d);
  }
  {
    JsonDocument d;
    d["name"] = "Learn remote"; d["unique_id"] = nodeId() + "_learn"; d["command_topic"] = topic("learn/set");
    d["payload_press"] = "START"; d["icon"] = "mdi:school"; d["entity_category"] = "config";
    deviceJson(d.as<JsonObject>());
    publishJson(disc + "/button/" + nodeId() + "_learn/config", d);
  }
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
  d["channels"] = maskText(cfg[i].mask);
  d["position_known"] = pos >= 0;
  d["target_position"] = st[i].target;
  d["last_source"] = st[i].lastSource;
  d["open_time_s"] = cfg[i].openMs / 1000.0;
  d["close_time_s"] = cfg[i].closeMs / 1000.0;
  d["favourite_position"] = cfg[i].favourite;
  d["inverted"] = cfg[i].invert;
  publishJson(blindTopic(i, "attributes"), d);
  st[i].lastPublishMs = millis();
  st[i].dirty = false;
}

void publishDiagnostics() {
  if (!mqtt.connected()) return;
  mqtt.publish(topic("diag/rssi").c_str(), String(WiFi.RSSI()).c_str(), true);
  mqtt.publish(topic("diag/uptime").c_str(), String(millis() / 1000).c_str(), true);
  mqtt.publish(topic("diag/ip").c_str(), WiFi.localIP().toString().c_str(), true);
  mqtt.publish(topic("diag/radio").c_str(), radioReady ? "ON" : "OFF", true);
  mqtt.publish(topic("diag/last_remote").c_str(), lastEventText.c_str(), true);
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
  if (!normal && !reversed) { err = "no blinds set up"; return false; }
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
  if (tp == topic("learn/set")) { learnMode = true; learnStartedMs = millis(); learnStatus = "waiting"; return; }
  if (tp == topic("all/set")) { String err; if (!allBlinds(msg, "home_assistant", err)) LOGW("All blinds: %s", err.c_str()); return; }
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
  if (apMode && WiFi.status() != WL_CONNECTED) return;
  if (WiFi.status() != WL_CONNECTED || !net.mqttHost.length() || mqtt.connected()) return;
  if (millis() - lastMqttTryMs < 5000 && lastMqttTryMs) return;
  lastMqttTryMs = millis();
  mqtt.setServer(net.mqttHost.c_str(), net.mqttPort);
  const String clientId = net.base + "_" + String(static_cast<uint32_t>(ESP.getEfuseMac()), HEX);
  const String will = topic("status");
  bool ok = net.mqttUser.length()
    ? mqtt.connect(clientId.c_str(), net.mqttUser.c_str(), net.mqttPass.c_str(), will.c_str(), 0, true, "offline")
    : mqtt.connect(clientId.c_str(), will.c_str(), 0, true, "offline");
  if (!ok) {
    const int st = mqtt.state();
    const char* why = st == 5 ? "username or password rejected" : st == 4 ? "bad username or password" : st == -2 ? "broker not reachable" : st == -4 ? "connection timed out" : "connection failed";
    LOGW("MQTT %s:%u: %s (code %d)", net.mqttHost.c_str(), net.mqttPort, why, st);
    return;
  }
  LOGI("MQTT connected to %s:%u", net.mqttHost.c_str(), net.mqttPort);
  mqtt.publish(will.c_str(), "online", true);
  mqtt.subscribe(topic("learn/set").c_str());
  mqtt.subscribe(topic("all/set").c_str());
  for (size_t i = 0; i < blindCount; ++i) {
    mqtt.subscribe(blindTopic(i, "set").c_str());
    mqtt.subscribe(blindTopic(i, "set_position").c_str());
  }
  publishDiscovery();
  for (size_t i = 0; i < blindCount; ++i) publishBlind(i);
  publishDiagnostics();
}

// ---------------------------------------------------------------- commands
bool handleBlindCommand(size_t i, String cmd, const String& source, String& err) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd == "OPEN") return moveTo(i, 100, source, err);
  if (cmd == "CLOSE") return moveTo(i, 0, source, err);
  if (cmd == "STOP") return stopBlind(i, source, err);
  if (cmd == "FAVOURITE" || cmd == "FAVORITE") {
    if (cfg[i].favourite < 0) { err = "no favourite position set"; return false; }
    return moveTo(i, cfg[i].favourite, source, err);
  }
  if (cmd == "SYNC_OPEN" || cmd == "SYNC_CLOSE") {
    const int p = cmd == "SYNC_OPEN" ? 100 : 0;
    for (size_t j = 0; j < blindCount; ++j) {
      if ((cfg[j].mask & ~cfg[i].mask) != 0) continue;
      st[j].motion = Motion::Stopped;
      st[j].position = st[j].startPos = st[j].target = p;
      st[j].lastSource = "marked";
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

void updateMotion() {
  const uint32_t now = millis();
  bool anyStopped = false;
  for (size_t i = 0; i < blindCount; ++i) {
    BlindState& s = st[i];
    if (s.motion == Motion::Stopped) continue;
    if (now - s.lastPublishMs > 1000) s.dirty = true;
    if (now - s.startMs < s.durMs) continue;
    const int goal = s.target;
    if (s.autoStop) {
      String err;
      if (sendAction(cfg[i].mask, aok::Action::Stop, err)) {
        applyAction(cfg[i].mask, aok::Action::Stop, s.lastSource, static_cast<int>(i), -1);
        s.position = s.startPos = s.target = goal;
      } else {
        haltMotion(i, "auto_stop_failed");  // keep the time-based estimate
      }
    } else {
      haltMotion(i, s.lastSource);
      s.position = s.startPos = s.target = goal;  // endpoint reached: motor stops at its limit
    }
    s.dirty = true;
    anyStopped = true;
  }
  if (anyStopped) requestPositionSave();
  // groups follow their members
  for (size_t i = 0; i < blindCount; ++i) {
    if (__builtin_popcount(cfg[i].mask) <= 1) continue;
    for (size_t j = 0; j < blindCount; ++j)
      if (j != i && (cfg[j].mask & ~cfg[i].mask) == 0 && st[j].dirty) { st[i].dirty = true; break; }
  }
  for (size_t i = 0; i < blindCount; ++i) if (st[i].dirty) publishBlind(i);
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
  frames[frameHead] = HeardFrame{millis(), f, matched, 1};
  frameHead = (frameHead + 1) % kMaxFrames;
  if (frameCount < kMaxFrames) frameCount++;
}

void onPayload(uint64_t payload) {
  const uint32_t now = millis();
  const aok::Frame f = aok::splitPayload(payload);
  const bool repeat = payload == lastPayload && now - lastPayloadMs < kDedupeMs;
  lastPayload = payload;
  lastPayloadMs = now;
  const bool matched = (remote.have[0] || remote.have[1] || remote.have[2]) && f.prefix == remote.prefix && f.remoteId == remote.remoteId;
  recordFrame(f, matched);
  if (repeat) return;
  lastRxFlashMs = now;
  if (f.action == aok::Action::Unknown || !f.mask) return;

  if (learnMode) {
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
  if ((remote.seen | f.mask) != remote.seen) { remote.seen |= f.mask; saveRemote(); }
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

void pollRadio() {
  uint64_t payload;
  uint16_t budget = 512;
  while (ringTail != ringHead && budget--) {
    const uint32_t v = ring[ringTail];
    ringTail = (ringTail + 1) & (kRingSize - 1);
    if (decoder.feed(v & 0x7FFFFFFF, (v & 0x80000000u) != 0, payload)) onPayload(payload);
  }
  if (learnMode && millis() - learnStartedMs > 120000) { learnMode = false; learnStatus = remoteComplete() ? "complete" : "timed out"; }
}

// ---------------------------------------------------------------- web
bool authed() {
  if (!net.adminPass.length()) return true;
  if (web.authenticate("admin", net.adminPass.c_str())) return true;
  web.requestAuthentication(BASIC_AUTH, "Blind Bridge");
  return false;
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

void handleState() {
  const bool lite = web.hasArg("lite");  // frequent polls skip frames and calibration history
  JsonDocument d;
  d["fw"] = FW_VERSION; d["uptime"] = millis() / 1000; d["heap"] = ESP.getFreeHeap();
  d["rev"] = stateRev; d["brand"] = net.brand; d["lite"] = lite;
  d["reset_reason"] = static_cast<int>(esp_reset_reason());
  JsonObject w = d["wifi"].to<JsonObject>();
  w["connected"] = WiFi.status() == WL_CONNECTED; w["ssid"] = net.ssid; w["ip"] = WiFi.localIP().toString();
  w["rssi"] = WiFi.RSSI(); w["ap"] = apMode; w["hostname"] = net.hostname;
  JsonObject m = d["mqtt"].to<JsonObject>();
  m["configured"] = net.mqttHost.length() > 0; m["connected"] = mqtt.connected(); m["state"] = mqtt.state();
  JsonObject r = d["radio"].to<JsonObject>();
  r["ready"] = radioReady; r["overflow"] = static_cast<uint32_t>(ringOverflow); r["mhz"] = pins.mhz;
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
    o["group"] = __builtin_popcount(cfg[i].mask) > 1;
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
    o["ago_s"] = (millis() - h.ms) / 1000; o["payload"] = hex(h.f.payload, 16); o["remote_id"] = hex(h.f.remoteId, 2);
    o["prefix"] = hex(h.f.prefix, 6); o["channels"] = maskText(h.f.mask); o["action"] = aok::actionName(h.f.action);
    o["matched"] = h.matched; o["repeats"] = h.repeats; o["kind"] = frameKind(h.f);
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
  sendJson(d);
}

void handleConfigGet() {
  if (!authed()) return;
  JsonDocument d;
  JsonObject n = d["network"].to<JsonObject>();
  n["ssid"] = net.ssid; n["pass_set"] = net.pass.length() > 0; n["hostname"] = net.hostname;
  n["admin_set"] = net.adminPass.length() > 0;
  n["mqtt_host"] = net.mqttHost; n["mqtt_port"] = net.mqttPort; n["mqtt_user"] = net.mqttUser;
  n["mqtt_pass_set"] = net.mqttPass.length() > 0; n["base_topic"] = net.base; n["discovery_prefix"] = net.discovery;
  n["brand"] = net.brand; n["tz"] = net.tz; n["tz_name"] = net.tzName;
  JsonObject p = d["pins"].to<JsonObject>();
  p["sck"] = pins.sck; p["miso"] = pins.miso; p["mosi"] = pins.mosi; p["cs"] = pins.cs;
  p["gdo0"] = pins.gdo0; p["gdo2"] = pins.gdo2; p["led"] = pins.led; p["mhz"] = pins.mhz; p["pa_dbm"] = pins.paDbm;
  remoteToJson(d["remote"].to<JsonObject>());
  JsonArray b = d["blinds"].to<JsonArray>();
  for (size_t i = 0; i < blindCount; ++i) blindToJson(i, b.add<JsonObject>());
  sendJson(d);
}

void scheduleReboot(uint32_t ms = 1500) { rebootPending = true; rebootAtMs = millis() + ms; }

void handleNetworkPost() {
  if (!authed()) return;
  JsonDocument d; if (!bodyJson(d)) return;
  bool reboot = false;
  if (d["ssid"].is<const char*>()) { String v = d["ssid"].as<String>(); if (v != net.ssid) { net.ssid = v; reboot = true; } }
  if (d["pass"].is<const char*>() && d["pass"].as<String>().length()) { net.pass = d["pass"].as<String>(); reboot = true; }
  if (d["hostname"].is<const char*>()) { String v = slug(d["hostname"].as<String>()); v.replace("_", "-"); if (v != net.hostname) { net.hostname = v; reboot = true; } }
  if (d["admin_pass"].is<const char*>()) net.adminPass = d["admin_pass"].as<String>();
  if (d["mqtt_host"].is<const char*>()) net.mqttHost = d["mqtt_host"].as<String>();
  if (d["mqtt_port"].is<int>()) net.mqttPort = d["mqtt_port"];
  if (d["mqtt_user"].is<const char*>()) net.mqttUser = d["mqtt_user"].as<String>();
  if (d["mqtt_pass"].is<const char*>() && d["mqtt_pass"].as<String>().length()) net.mqttPass = d["mqtt_pass"].as<String>();
  if (d["base_topic"].is<const char*>() && d["base_topic"].as<String>().length()) net.base = slug(d["base_topic"].as<String>());
  if (d["discovery_prefix"].is<const char*>() && d["discovery_prefix"].as<String>().length()) net.discovery = d["discovery_prefix"].as<String>();
  if (d["brand"].is<const char*>()) { String v = d["brand"].as<String>(); v.trim(); if (v.length() && v.length() <= 24) net.brand = v; }
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
  savePins();
  scheduleReboot(1500);
  JsonDocument r; r["ok"] = true; r["rebooting"] = true; sendJson(r);
}

void handleRemotePost() {
  if (!authed()) return;
  JsonDocument d; if (!bodyJson(d)) return;
  const String action = d["action"] | "";
  if (action == "learn") { learnMode = true; learnStartedMs = millis(); learnStatus = "waiting"; LOGI("Remote learning started"); }
  else if (action == "cancel") { learnMode = false; learnStatus = "cancelled"; }
  else if (action == "forget") { for (bool& h : remote.have) h = false; remote.prefix = 0; remote.remoteId = 0; remote.seen = 0; saveRemote(); learnStatus = "idle"; LOGI("Remote forgotten"); }
  else if (action == "save") { remoteFromJson(d["remote"].as<JsonObjectConst>()); saveRemote(); }
  else { sendError("unknown action"); return; }
  sendOk();
}

void handleBlindsPost() {
  if (!authed()) return;
  JsonDocument d; if (!bodyJson(d)) return;
  String err;
  if (!blindsFromJson(d["blinds"].as<JsonArrayConst>(), err)) { sendError(err); return; }
  saveBlinds();
  requestPositionSave();
  stateRev++;
  LOGI("Blinds saved (%u)", static_cast<unsigned>(blindCount));
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

void handleTestPost() {  // send a raw action on any channel(s) — handy before blinds are set up
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
    if (!sendAction(cfg[i].mask, rfFor(i, calMotion), err)) { sendError(err); return; }
    calIndex = i;
    calStartMs = millis();
    LOGI("Calibration started: %s %s", cfg[i].name.c_str(), calMotion == Motion::Opening ? "opening" : "closing");
    applyAction(cfg[i].mask, rfFor(i, calMotion), "calibration", -1, -1);
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

// Wi-Fi scan for the setup page: first call starts an async scan, later calls return results
void handleScan() {
  if (!authed()) return;
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
  JsonObject p = d["pins"].to<JsonObject>();
  p["sck"] = pins.sck; p["miso"] = pins.miso; p["mosi"] = pins.mosi; p["cs"] = pins.cs;
  p["gdo0"] = pins.gdo0; p["gdo2"] = pins.gdo2; p["led"] = pins.led; p["mhz"] = pins.mhz; p["pa_dbm"] = pins.paDbm;
  remoteToJson(d["remote"].to<JsonObject>());
  JsonArray b = d["blinds"].to<JsonArray>();
  for (size_t i = 0; i < blindCount; ++i) blindToJson(i, b.add<JsonObject>());
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
  }
  JsonObjectConst p = d["pins"];
  if (!p.isNull()) {
    pins.sck = p["sck"] | pins.sck; pins.miso = p["miso"] | pins.miso; pins.mosi = p["mosi"] | pins.mosi;
    pins.cs = p["cs"] | pins.cs; pins.gdo0 = p["gdo0"] | pins.gdo0; pins.gdo2 = p["gdo2"] | pins.gdo2;
    pins.led = p["led"] | pins.led; pins.mhz = p["mhz"] | pins.mhz; pins.paDbm = p["pa_dbm"] | pins.paDbm;
  }
  if (!d["remote"].isNull()) remoteFromJson(d["remote"].as<JsonObjectConst>());
  saveNet(); savePins(); saveRemote(); saveBlinds(); savePositions();
  LOGI("Settings restored from a backup (%u blinds), restarting", static_cast<unsigned>(blindCount));
  scheduleReboot(1500);
  JsonDocument r; r["ok"] = true; r["rebooting"] = true; sendJson(r);
}

void handleReboot() { if (!authed()) return; LOGI("Restart requested from the web page"); sendOk(); scheduleReboot(800); }

void handleLog() {
  if (!authed()) return;
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

void handleUpdateDone() {
  if (!authed()) return;
  const bool ok = !Update.hasError();
  if (ok) LOGI("Firmware updated, restarting"); else LOGE("Firmware update failed: %s", Update.errorString());
  web.send(ok ? 200 : 500, "text/plain", ok ? "Update OK — rebooting" : "Update failed");
  if (ok) scheduleReboot(1000);
}

void handleUpdateUpload() {
  if (net.adminPass.length() && !web.authenticate("admin", net.adminPass.c_str())) return;
  HTTPUpload& up = web.upload();
  if (up.status == UPLOAD_FILE_START) { LOGI("Firmware upload started: %s", up.filename.c_str()); rxStop(); Update.begin(UPDATE_SIZE_UNKNOWN); }
  else if (up.status == UPLOAD_FILE_WRITE) Update.write(up.buf, up.currentSize);
  else if (up.status == UPLOAD_FILE_END) Update.end(true);
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
  static const char* kHeaders[] = {"If-None-Match"};
  web.collectHeaders(kHeaders, 1);
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
  web.on("/api/log", HTTP_GET, handleLog);
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
}

// ---------------------------------------------------------------- Improv Wi-Fi (serial)
// https://www.improv-wifi.com/serial/ — lets the web installer ask for Wi-Fi straight after flashing.
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
String url() { return String("http://") + WiFi.localIP().toString(); }

void handle(const uint8_t* d, size_t n) {
  if (n < 2 || static_cast<size_t>(d[1]) + 2 > n) { error(ErrInvalid); return; }
  switch (d[0]) {
    case CmdState:
      if (WiFi.status() == WL_CONNECTED) { state(StateProvisioned); result(CmdState, {url()}); }
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
    if (WiFi.status() == WL_CONNECTED) { state(StateProvisioned); result(CmdWifi, {url()}); provisioning = false; }
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

void setupWifi() {
  WiFi.persistent(false);
  WiFi.setHostname(net.hostname.c_str());
  if (!net.ssid.length()) { startAp(); return; }
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(net.ssid.c_str(), net.pass.c_str());
  staStartMs = millis();
}

void maintainWifi() {
  const bool up = WiFi.status() == WL_CONNECTED;
  static bool wasUp = false;
  if (up != wasUp) {
    wasUp = up;
    if (up) LOGI("Wi-Fi connected to %s, IP %s, signal %d dBm", net.ssid.c_str(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
    else if (net.ssid.length()) LOGW("Wi-Fi disconnected from %s", net.ssid.c_str());
  }
  if (up) {
    if (!ntpStarted) { configTzTime(net.tz.length() ? net.tz.c_str() : "UTC0", "pool.ntp.org", "time.google.com"); ntpStarted = true; }
    if (apMode && millis() - staStartMs > 5000) stopAp();
    if (!mdnsReady && MDNS.begin(net.hostname.c_str())) { MDNS.addService("http", "tcp", 80); mdnsReady = true; }
    if (!otaReady) {
      ArduinoOTA.setHostname(net.hostname.c_str());
      if (net.adminPass.length()) ArduinoOTA.setPassword(net.adminPass.c_str());
      ArduinoOTA.onStart([]() { rxStop(); });
      ArduinoOTA.begin();
      otaReady = true;
    }
  } else if (net.ssid.length() && !apMode && millis() - staStartMs > 45000) {
    LOGW("Can't join %s after 45 s: opening the setup Wi-Fi as well", net.ssid.c_str());
    startAp();  // can't join — open the setup portal but keep trying the home network
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
  {
    const esp_reset_reason_t rr = esp_reset_reason();
    const char* why = rr == ESP_RST_POWERON ? "power on" : rr == ESP_RST_SW ? "restart" : rr == ESP_RST_BROWNOUT ? "low voltage"
      : (rr == ESP_RST_PANIC ? "crash" : (rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT) ? "watchdog" : "other");
    LOGI("%s Blind Bridge %s started after %s, %u blinds", net.brand.c_str(), FW_VERSION, why, static_cast<unsigned>(blindCount));
  }
  if (pins.led >= 0) { pinMode(pins.led, OUTPUT); ledSet(true); }
  setupRadio();
  setupWifi();
  setupWeb();
  mqtt.setBufferSize(2048);
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
  pollRadio();
  updateMotion();
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
