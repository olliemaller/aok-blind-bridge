#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif

#include <vector>

#include <string>

#include "aok_protocol.h"

namespace esphome {
namespace aok_rf {

class AokRf : public Component {
 public:
  void set_pins(int sck, int miso, int mosi, int cs, int gdo0, int gdo2) {
    sck_ = sck; miso_ = miso; mosi_ = mosi; cs_ = cs; gdo0_ = gdo0; gdo2_ = gdo2;
  }
  void set_frequency(float mhz) { mhz_ = mhz; }
  void set_tx_power(int dbm) { tx_power_ = dbm; }
  void set_repeats(int n) { timings_.repeats = static_cast<uint8_t>(n); }
  void set_remote(uint32_t prefix, uint8_t remote_id, uint16_t up, uint16_t down, uint16_t stop) {
    prefix_ = prefix; remote_id_ = remote_id; base_[0] = up; base_[1] = down; base_[2] = stop; have_remote_ = true;
  }
  // The P button's code (from the log, like up/down/stop): lets aok_rf.pair do the whole pairing.
  void set_program(uint16_t program) { program_ = program; have_program_ = true; }
  // Other remotes to follow (a second hand remote, a wall switch): my_remote is true for them too.
  void add_extra_remote(uint32_t prefix, uint8_t remote_id) { extra_.push_back({prefix, remote_id}); }
  // Channels whose motors are paired to the component's own ID (see aok_rf.pair).
  void set_own_id(uint8_t id) { own_id_ = id; }
  void set_own_channels(uint16_t mask) { own_mask_ = mask; }
  void set_repeat_remote(bool on) { repeat_ = on; }
#ifdef USE_SENSOR
  void set_rssi_sensor(sensor::Sensor *s) { rssi_sensor_ = s; }
#endif

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // action: 0 up, 1 down, 2 stop. mask: bit n-1 = channel n.
  bool send(uint16_t mask, uint8_t action);
  // Pair channel(s) to the own ID: P, P from the remote's ID (needs program:), then Up from own_id.
  // With pressed_p_already, only the Up is sent (press P twice on the hand remote first).
  void pair(uint16_t mask, bool pressed_p_already);
  // Measure 433.42-434.42 MHz while a remote button is held; logs where it's strongest.
  void frequency_scan();

  void add_on_long_press_callback(std::function<void(std::string, uint16_t, float)> &&cb) { long_callbacks_.add(std::move(cb)); }

  void add_on_frame_callback(std::function<void(std::string, uint16_t, bool)> &&cb) { callbacks_.add(std::move(cb)); }

 protected:
  void start_rx_();
  void stop_rx_();
  void on_payload_(uint64_t payload);
  bool transmit_(uint64_t payload);
  bool known_(uint32_t prefix, uint8_t id) const;

  int sck_{18}, miso_{19}, mosi_{23}, cs_{5}, gdo0_{12}, gdo2_{27};
  float mhz_{433.92f};
  int tx_power_{10};
  bool ready_{false};
  bool have_remote_{false};
  uint32_t prefix_{0};
  uint8_t remote_id_{0};
  uint16_t base_[3]{0, 0, 0};
  aok::Timings timings_;
  aok::Decoder decoder_;
  uint64_t last_payload_{0};
  uint32_t last_payload_ms_{0};
  bool have_program_{false};
  uint16_t program_{0};
  struct Extra { uint32_t prefix; uint8_t id; };
  std::vector<Extra> extra_;
  uint8_t own_id_{0};
  uint16_t own_mask_{0};
  bool repeat_{false};
  // a held button: start time, action and the payload to repeat
  bool press_active_{false};
  uint32_t press_start_ms_{0};
  std::string press_kind_;
  uint16_t press_mask_{0};
  uint64_t press_payload_{0};
  // pairing steps run from loop()
  uint16_t pair_mask_{0};
  uint8_t pair_stage_{0};
  uint32_t pair_at_ms_{0};
#ifdef USE_SENSOR
  sensor::Sensor *rssi_sensor_{nullptr};
#endif
  CallbackManager<void(std::string, uint16_t, bool)> callbacks_;
  CallbackManager<void(std::string, uint16_t, float)> long_callbacks_;
};

class FrameTrigger : public Trigger<std::string, uint16_t, bool> {
 public:
  explicit FrameTrigger(AokRf *parent) {
    parent->add_on_frame_callback([this](std::string action, uint16_t channels, bool mine) {
      this->trigger(action, channels, mine);
    });
  }
};

class LongPressTrigger : public Trigger<std::string, uint16_t, float> {
 public:
  explicit LongPressTrigger(AokRf *parent) {
    parent->add_on_long_press_callback([this](std::string action, uint16_t channels, float seconds) {
      this->trigger(action, channels, seconds);
    });
  }
};

template<typename... Ts> class PairAction : public Action<Ts...>, public Parented<AokRf> {
 public:
  void set_mask(uint16_t mask) { mask_ = mask; }
  void set_pressed(bool pressed) { pressed_ = pressed; }
  void play(Ts... x) override { this->parent_->pair(mask_, pressed_); }

 protected:
  uint16_t mask_{1};
  bool pressed_{false};
};

template<typename... Ts> class ScanAction : public Action<Ts...>, public Parented<AokRf> {
 public:
  void play(Ts... x) override { this->parent_->frequency_scan(); }
};

template<typename... Ts> class SendAction : public Action<Ts...>, public Parented<AokRf> {
 public:
  void set_mask(uint16_t mask) { mask_ = mask; }
  void set_action(uint8_t action) { action_ = action; }
  void play(Ts... x) override { this->parent_->send(mask_, action_); }

 protected:
  uint16_t mask_{1};
  uint8_t action_{2};
};

}  // namespace aok_rf
}  // namespace esphome
