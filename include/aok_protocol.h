#pragma once
// A-OK / Zemismart 433.92 MHz blind remote protocol ("new PWM" format, 64-bit payload).
// Encoding rules follow tools/aok_protocol.py from pgnehm/ESP32-Orvibo-A-OK-HomeAssistant-Blind-Bridge.
// Header-only and free of Arduino types so it can be unit-tested on the host (see test/).

#include <stddef.h>
#include <stdint.h>

namespace aok {

enum class Action : uint8_t { Open = 0, Close = 1, Stop = 2, Unknown = 255 };

struct Timings {
  uint16_t shortUs = 263;
  uint16_t longUs = 582;
  uint16_t syncHighUs = 4960;
  uint16_t syncLowUs = 670;
  uint16_t gapUs = 9790;
  uint8_t repeats = 8;
};

// Channel mask: bit (n-1) set = channel n (1..16).
inline uint16_t channelField(uint16_t mask) {
  uint16_t field = 0xFFFF;
  for (uint8_t ch = 1; ch <= 16; ++ch) {
    if (mask & (1u << (ch - 1))) field ^= static_cast<uint16_t>(1u << ((ch + 7) % 16));
  }
  return field;
}

inline uint16_t maskFromField(uint16_t field) {
  uint16_t mask = 0;
  for (uint8_t ch = 1; ch <= 16; ++ch) {
    if ((field & (1u << ((ch + 7) % 16))) == 0) mask |= static_cast<uint16_t>(1u << (ch - 1));
  }
  return mask;
}

inline int signed8(int value) { return ((value + 128) % 256 + 256) % 256 - 128; }

inline int channelOffset(uint16_t mask) {
  int sum = 2;
  for (uint8_t ch = 1; ch <= 16; ++ch) {
    if (mask & (1u << (ch - 1))) sum += 1 << ((ch - 1) % 8);
  }
  return signed8(sum);
}

inline uint16_t deriveBase(uint8_t remoteId, uint16_t mask, uint16_t capturedCommand) {
  const int offset = channelOffset(mask);
  return static_cast<uint16_t>((capturedCommand & 0xFF00) | ((capturedCommand - remoteId + offset) & 0xFF));
}

inline uint16_t makeCommand(uint8_t remoteId, uint16_t mask, uint16_t base) {
  const int offset = channelOffset(mask);
  return static_cast<uint16_t>((base & 0xFF00) | ((base + remoteId - offset) & 0xFF));
}

inline uint64_t makePayload(uint32_t prefix24, uint8_t remoteId, uint16_t mask, uint16_t base) {
  return (static_cast<uint64_t>(prefix24 & 0xFFFFFF) << 40) | (static_cast<uint64_t>(remoteId) << 32) |
         (static_cast<uint64_t>(channelField(mask)) << 16) | makeCommand(remoteId, mask, base);
}

inline Action actionFromCommand(uint16_t command) {
  switch ((command >> 8) & 0xFF) {
    case 0xF4: return Action::Open;
    case 0xBC: return Action::Close;
    case 0xDC: return Action::Stop;
    default: return Action::Unknown;
  }
}

inline const char* actionName(Action a) {
  switch (a) {
    case Action::Open: return "open";
    case Action::Close: return "close";
    case Action::Stop: return "stop";
    default: return "unknown";
  }
}

struct Frame {
  uint64_t payload = 0;
  uint32_t prefix = 0;
  uint8_t remoteId = 0;
  uint16_t field = 0;
  uint16_t mask = 0;
  uint16_t command = 0;
  Action action = Action::Unknown;
};

inline Frame splitPayload(uint64_t payload) {
  Frame f;
  f.payload = payload;
  f.prefix = static_cast<uint32_t>((payload >> 40) & 0xFFFFFF);
  f.remoteId = static_cast<uint8_t>((payload >> 32) & 0xFF);
  f.field = static_cast<uint16_t>((payload >> 16) & 0xFFFF);
  f.mask = maskFromField(f.field);
  f.command = static_cast<uint16_t>(payload & 0xFFFF);
  f.action = actionFromCommand(f.command);
  return f;
}

// Pulse train for one frame, starting HIGH and alternating. Returns the pulse count written
// (134 = sync pair + 66 bit pairs). The inter-frame gap is not included; wait gapUs between repeats.
constexpr size_t kFramePulses = 2 + 66 * 2;
inline size_t encodeFrame(uint64_t payload, const Timings& t, uint16_t* out, size_t cap) {
  if (cap < kFramePulses) return 0;
  size_t n = 0;
  out[n++] = t.syncHighUs;
  out[n++] = t.syncLowUs;
  for (int i = 0; i < 66; ++i) {
    int bit;
    if (i < 64) bit = static_cast<int>((payload >> (63 - i)) & 1u);
    else bit = (i == 64) ? 1 : 0;  // trailer bits [1, 0]
    if (bit == 0) { out[n++] = t.longUs; out[n++] = t.shortUs; }
    else { out[n++] = t.shortUs; out[n++] = t.longUs; }
  }
  return n;
}

// Streaming decoder fed with pulse durations and the level they were at.
class Decoder {
 public:
  // Returns true when a complete 64-bit payload has been read into `payload`.
  bool feed(uint32_t durationUs, bool high, uint64_t& payload) {
    switch (state_) {
      case State::Seek:
        if (high && inRange(durationUs, 3800, 6500)) state_ = State::SyncLow;
        return false;
      case State::SyncLow:
        if (!high && inRange(durationUs, 420, 1000)) { state_ = State::BitHigh; bits_ = 0; value_ = 0; }
        else reset(durationUs, high);
        return false;
      case State::BitHigh:
        if (high) { highUs_ = durationUs; state_ = State::BitLow; }
        else reset(durationUs, high);
        return false;
      case State::BitLow: {
        if (high) { reset(durationUs, high); return false; }
        int bit = classify(highUs_, durationUs);
        if (bit < 0) {
          // The last low of a frame merges into the gap; accept once 64 bits are in.
          reset(durationUs, high);
          return false;
        }
        value_ = (value_ << 1) | static_cast<uint64_t>(bit);
        if (++bits_ == 64) { payload = value_; state_ = State::Seek; return true; }
        state_ = State::BitHigh;
        return false;
      }
    }
    return false;
  }
  void clear() { state_ = State::Seek; }

 private:
  enum class State : uint8_t { Seek, SyncLow, BitHigh, BitLow };
  static bool inRange(uint32_t v, uint32_t lo, uint32_t hi) { return v >= lo && v <= hi; }
  static int classify(uint32_t hi, uint32_t lo) {
    const bool hiShort = inRange(hi, 140, 420), hiLong = inRange(hi, 421, 950);
    const bool loShort = inRange(lo, 140, 420), loLong = inRange(lo, 421, 950);
    if (hiLong && loShort) return 0;
    if (hiShort && loLong) return 1;
    return -1;
  }
  void reset(uint32_t durationUs, bool high) {
    state_ = State::Seek;
    if (high && inRange(durationUs, 3800, 6500)) state_ = State::SyncLow;
  }
  State state_ = State::Seek;
  uint32_t highUs_ = 0;
  uint64_t value_ = 0;
  uint8_t bits_ = 0;
};

}  // namespace aok
