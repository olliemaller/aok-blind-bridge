#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"

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

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // action: 0 up, 1 down, 2 stop. mask: bit n-1 = channel n.
  bool send(uint16_t mask, uint8_t action);

  void add_on_frame_callback(std::function<void(std::string, uint16_t, bool)> &&cb) { callbacks_.add(std::move(cb)); }

 protected:
  void start_rx_();
  void stop_rx_();
  void on_payload_(uint64_t payload);

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
  CallbackManager<void(std::string, uint16_t, bool)> callbacks_;
};

class FrameTrigger : public Trigger<std::string, uint16_t, bool> {
 public:
  explicit FrameTrigger(AokRf *parent) {
    parent->add_on_frame_callback([this](std::string action, uint16_t channels, bool mine) {
      this->trigger(action, channels, mine);
    });
  }
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
