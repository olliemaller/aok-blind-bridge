#include "aok_rf.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <Arduino.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

namespace esphome {
namespace aok_rf {

static const char *const TAG = "aok_rf";

// Edge timings captured by the GDO2 interrupt, decoded in loop().
static constexpr uint16_t RING_SIZE = 1024;  // power of two
static volatile uint32_t ring[RING_SIZE];
static volatile uint16_t ring_head = 0, ring_tail = 0;
static volatile uint32_t last_edge_us = 0;
static volatile int rx_pin = -1;

static void IRAM_ATTR on_edge() {
  const uint32_t now = micros();
  const uint32_t dur = now - last_edge_us;
  last_edge_us = now;
  if (dur < 100) return;
  const bool level_now = digitalRead(rx_pin);
  const uint32_t v = (dur > 0x7FFFFFFF ? 0x7FFFFFFF : dur) | (level_now ? 0 : 0x80000000u);  // level before the edge
  const uint16_t next = (ring_head + 1) & (RING_SIZE - 1);
  if (next == ring_tail) return;
  ring[ring_head] = v;
  ring_head = next;
}

static std::string mask_text(uint16_t mask) {
  std::string out;
  for (int ch = 1; ch <= 16; ++ch)
    if (mask & (1u << (ch - 1))) {
      if (!out.empty()) out += ",";
      out += std::to_string(ch);
    }
  return out;
}

static const char *kind_name(uint16_t command) {
  switch ((command >> 8) & 0xFF) {
    case 0xF4: return "up";
    case 0xBC: return "down";
    case 0xDC: return "stop";
    case 0xAC: return "program";
    case 0xDB: return "release";
    default: return "other";
  }
}

void AokRf::setup() {
  pinMode(gdo0_, OUTPUT);
  digitalWrite(gdo0_, LOW);
  ELECHOUSE_cc1101.setSpiPin(sck_, miso_, mosi_, cs_);
  ELECHOUSE_cc1101.Init();
  if (!ELECHOUSE_cc1101.getCC1101()) {
    ESP_LOGE(TAG, "CC1101 not found: check VCC (3V3), SCK, MISO, MOSI and CSN");
    this->mark_failed();
    return;
  }
  ELECHOUSE_cc1101.setCCMode(0);      // asynchronous serial: raw OOK on the GDO pins
  ELECHOUSE_cc1101.setModulation(2);  // ASK/OOK
  ELECHOUSE_cc1101.setMHZ(mhz_);
  ELECHOUSE_cc1101.setPA(tx_power_);
  ELECHOUSE_cc1101.setRxBW(270.0f);
  ELECHOUSE_cc1101.setDRate(4.8f);
  ELECHOUSE_cc1101.setSyncMode(0);
  ELECHOUSE_cc1101.setCrc(false);
  ELECHOUSE_cc1101.setPktFormat(3);
  ELECHOUSE_cc1101.SpiWriteReg(0x00, 0x0D);  // IOCFG2: GDO2 = received data
  ELECHOUSE_cc1101.SpiWriteReg(0x02, 0x2E);  // IOCFG0: GDO0 high impedance until we transmit
  ready_ = true;
  this->start_rx_();
}

void AokRf::start_rx_() {
  ELECHOUSE_cc1101.SetRx(mhz_);
  decoder_.clear();
  rx_pin = gdo2_;
  pinMode(gdo2_, INPUT);
  last_edge_us = micros();
  attachInterrupt(digitalPinToInterrupt(gdo2_), on_edge, CHANGE);
}

void AokRf::stop_rx_() { detachInterrupt(digitalPinToInterrupt(gdo2_)); }

bool AokRf::known_(uint32_t prefix, uint8_t id) const {
  for (const auto &e : extra_)
    if (e.prefix == prefix && e.id == id) return true;
  return false;
}

bool AokRf::send(uint16_t mask, uint8_t action) {
  if (!ready_) { ESP_LOGW(TAG, "Radio not ready"); return false; }
  if (!have_remote_ || action > 2) {
    ESP_LOGW(TAG, "No remote set: add a remote: block (prefix, remote_id, up, down, stop) from the log");
    return false;
  }
  const char *name = action == 0 ? "up" : action == 1 ? "down" : "stop";
  const uint16_t own = own_id_ ? (mask & own_mask_) : 0, copied = mask & ~own;
  bool ok = true;
  if (copied) {
    ESP_LOGI(TAG, "Sending %s on channel %s", name, mask_text(copied).c_str());
    ok &= this->transmit_(aok::makePayload(prefix_, remote_id_, copied, base_[action]));
  }
  if (own) {
    ESP_LOGI(TAG, "Sending %s on channel %s (own ID)", name, mask_text(own).c_str());
    ok &= this->transmit_(aok::makePayload(prefix_, own_id_, own, base_[action]));
  }
  return ok;
}

void AokRf::pair(uint16_t mask, bool pressed_p_already) {
  if (!ready_ || !have_remote_) { ESP_LOGW(TAG, "Pairing needs the radio and the remote: block"); return; }
  if (!own_id_) { ESP_LOGW(TAG, "Pairing needs own_id: in the aok_rf: block"); return; }
  if (!pressed_p_already && !have_program_) {
    ESP_LOGW(TAG, "Pairing needs program: in remote: (press P on the hand remote and copy the code from the log), "
                  "or press P twice on the hand remote and use pressed_p: true");
    return;
  }
  pair_mask_ = mask;
  pair_stage_ = pressed_p_already ? 2 : 0;
  pair_at_ms_ = millis();
  ESP_LOGI(TAG, "Pairing channel %s to own ID %02X", mask_text(mask).c_str(), own_id_);
}

void AokRf::frequency_scan() {
  if (!ready_) return;
  this->stop_rx_();
  float best = mhz_;
  int best_rssi = -200;
  for (float f = 433.42f; f <= 434.425f; f += 0.05f) {
    ELECHOUSE_cc1101.setMHZ(f);
    ELECHOUSE_cc1101.SetRx(f);
    int peak = -200;
    const uint32_t t0 = millis();
    while (millis() - t0 < 180) { peak = std::max(peak, ELECHOUSE_cc1101.getRssi()); delay(2); }
    ESP_LOGD(TAG, "  %.2f MHz: %d dBm", f, peak);
    if (peak > best_rssi) { best_rssi = peak; best = f; }
  }
  ELECHOUSE_cc1101.setMHZ(mhz_);
  this->start_rx_();
  ESP_LOGI(TAG, "Frequency scan: strongest at %.2f MHz (%d dBm); set frequency: %.2fMHz to use it", best, best_rssi, best);
}

bool AokRf::transmit_(uint64_t payload) {
  uint16_t pulses[aok::kFramePulses];
  const size_t n = aok::encodeFrame(payload, timings_, pulses, aok::kFramePulses);
  this->stop_rx_();
  ELECHOUSE_cc1101.SetTx(mhz_);
  pinMode(gdo0_, OUTPUT);
  for (uint8_t r = 0; r < timings_.repeats; ++r) {
    bool level = true;
    for (size_t k = 0; k < n; ++k) {
      digitalWrite(gdo0_, level ? HIGH : LOW);
      delayMicroseconds(pulses[k]);
      level = !level;
    }
    digitalWrite(gdo0_, LOW);
    delayMicroseconds(timings_.gapUs);
  }
  this->start_rx_();
  return true;
}

void AokRf::loop() {
  if (!ready_) return;
  const uint32_t now = millis();
  // a held button ended: long press event, and the repeater
  if (press_active_ && now - last_payload_ms_ > 500) {
    press_active_ = false;
    const uint32_t held = last_payload_ms_ - press_start_ms_;
    if (held >= 1500) long_callbacks_.call(press_kind_, press_mask_, held / 1000.0f);
    if (repeat_ && press_payload_) { this->transmit_(press_payload_); ESP_LOGD(TAG, "Repeated the remote's press"); }
  }
  // pairing: P, P (2.5 s apart), then Up from the own ID
  if (pair_mask_ && static_cast<int32_t>(now - pair_at_ms_) >= 0) {
    if (pair_stage_ < 2) {
      this->transmit_(aok::makePayload(prefix_, remote_id_, pair_mask_, program_));
      ESP_LOGI(TAG, "Pairing: sent P (%u of 2): the motor should jog", pair_stage_ + 1);
      pair_stage_++;
      pair_at_ms_ = now + 2500;
    } else {
      this->transmit_(aok::makePayload(prefix_, own_id_, pair_mask_, base_[0]));
      ESP_LOGI(TAG, "Pairing: sent Up from own ID %02X on channel %s. If the motor jogged, add it to own_channels:",
               own_id_, mask_text(pair_mask_).c_str());
      pair_mask_ = 0;
    }
  }
  uint64_t payload;
  uint16_t budget = 512;
  while (ring_tail != ring_head && budget--) {
    const uint32_t v = ring[ring_tail];
    ring_tail = (ring_tail + 1) & (RING_SIZE - 1);
    if (decoder_.feed(v & 0x7FFFFFFF, (v & 0x80000000u) != 0, payload)) this->on_payload_(payload);
  }
}

void AokRf::on_payload_(uint64_t payload) {
  const uint32_t now = millis();
  if (payload == last_payload_ && now - last_payload_ms_ < 900) { last_payload_ms_ = now; return; }  // remote repeats
  last_payload_ = payload;
  last_payload_ms_ = now;
  const aok::Frame f = aok::splitPayload(payload);
  if (!f.mask) return;
  const bool from_main = have_remote_ && f.prefix == prefix_ && f.remoteId == remote_id_;
  const bool mine = from_main || this->known_(f.prefix, f.remoteId);
  const char *kind = kind_name(f.command);
#ifdef USE_SENSOR
  if (mine && rssi_sensor_ != nullptr) rssi_sensor_->publish_state(ELECHOUSE_cc1101.getRssi());
#endif
  if (from_main && ((f.command >> 8) & 0xFF) == 0xAC && !have_program_)
    ESP_LOGI(TAG, "P button heard -> remote: program: 0x%04X", aok::deriveBase(f.remoteId, f.mask, f.command));
  if (mine && f.action != aok::Action::Unknown) {
    press_active_ = true; press_start_ms_ = now; press_kind_ = kind; press_mask_ = f.mask; press_payload_ = payload;
  }
  if (mine) {
    ESP_LOGI(TAG, "Your remote: %s on channel %s", kind, mask_text(f.mask).c_str());
  } else if (f.action != aok::Action::Unknown) {
    // Everything needed for the remote: block. Press Up, Down and Stop once each.
    ESP_LOGI(TAG, "Remote heard: %s on channel %s -> remote: prefix: 0x%06X, remote_id: 0x%02X, %s: 0x%04X",
             kind, mask_text(f.mask).c_str(), (unsigned) f.prefix, f.remoteId,
             f.action == aok::Action::Open ? "up" : f.action == aok::Action::Close ? "down" : "stop",
             aok::deriveBase(f.remoteId, f.mask, f.command));
  } else {
    ESP_LOGD(TAG, "Frame %s (code %02X) on channel %s from remote %02X", kind, (f.command >> 8) & 0xFF,
             mask_text(f.mask).c_str(), f.remoteId);
  }
  callbacks_.call(kind, f.mask, mine);
}

void AokRf::dump_config() {
  ESP_LOGCONFIG(TAG, "A-OK RF (CC1101):");
  ESP_LOGCONFIG(TAG, "  Frequency: %.2f MHz, power %d dBm, %u repeats", mhz_, tx_power_, timings_.repeats);
  ESP_LOGCONFIG(TAG, "  Pins: SCK %d, MISO %d, MOSI %d, CSN %d, GDO0 %d, GDO2 %d", sck_, miso_, mosi_, cs_, gdo0_, gdo2_);
  if (have_remote_)
    ESP_LOGCONFIG(TAG, "  Remote: %06X / %02X", (unsigned) prefix_, remote_id_);
  else
    ESP_LOGCONFIG(TAG, "  Remote: not set. Press Up, Down and Stop on the remote and copy the codes from the log.");
  if (this->is_failed()) ESP_LOGE(TAG, "  CC1101 not found");
}

}  // namespace aok_rf
}  // namespace esphome
