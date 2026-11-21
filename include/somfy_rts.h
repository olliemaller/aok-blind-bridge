#pragma once
// Somfy RTS (433.42 MHz) frame builder, pulse encoder and decoder.
// The protocol is documented at https://pushstack.wordpress.com/somfy-rts-protocol/ and used by
// Nickduino/Somfy_Remote and rstrouse/ESPSomfy-RTS; this is an independent implementation.
// Header-only and free of Arduino types so it can be unit-tested on the host (see test/).
//
// Frame (7 bytes, before obfuscation):
//   0: 0xA0 | key nibble   1: command << 4 | checksum   2-3: rolling code (big endian)   4-6: address
// The checksum makes the XOR of all 14 nibbles zero. Obfuscation XORs each byte with the one
// before it (after obfuscation). Bits are sent most significant first, Manchester coded:
// a 1 is low then high, a 0 is high then low, each half 640 us.

#include <stddef.h>
#include <stdint.h>

namespace somfy {

enum class Command : uint8_t {
  My = 0x1, Up = 0x2, MyUp = 0x3, Down = 0x4, MyDown = 0x5, UpDown = 0x6, Prog = 0x8, SunFlag = 0x9, Flag = 0xA,
  Unknown = 0xFF,
};

inline const char* commandName(Command c) {
  switch (c) {
    case Command::My: return "my";
    case Command::Up: return "up";
    case Command::MyUp: return "my_up";
    case Command::Down: return "down";
    case Command::MyDown: return "my_down";
    case Command::UpDown: return "up_down";
    case Command::Prog: return "prog";
    case Command::SunFlag: return "sun_flag";
    case Command::Flag: return "flag";
    default: return "unknown";
  }
}

constexpr uint16_t kHalfUs = 640;
constexpr uint16_t kWakeHighUs = 9415;
constexpr uint32_t kWakeLowUs = 89565;
constexpr uint16_t kHwSyncUs = 2560;
constexpr uint16_t kSwSyncHighUs = 4550;
constexpr uint32_t kGapUs = 30415;

struct Frame {
  Command command = Command::Unknown;
  uint16_t code = 0;     // rolling code
  uint32_t address = 0;  // 24-bit remote address
  uint8_t key = 0;
};

inline uint8_t checksum(const uint8_t f[7]) {
  uint8_t c = 0;
  for (int i = 0; i < 7; ++i) c ^= static_cast<uint8_t>(f[i] ^ (f[i] >> 4));
  return c & 0x0F;
}

// Plain (not yet obfuscated) frame bytes.
inline void build(Command cmd, uint16_t code, uint32_t address, uint8_t out[7]) {
  out[0] = static_cast<uint8_t>(0xA0 | (code & 0x0F));
  out[1] = static_cast<uint8_t>(static_cast<uint8_t>(cmd) << 4);
  out[2] = static_cast<uint8_t>(code >> 8);
  out[3] = static_cast<uint8_t>(code);
  out[4] = static_cast<uint8_t>(address >> 16);
  out[5] = static_cast<uint8_t>(address >> 8);
  out[6] = static_cast<uint8_t>(address);
  out[1] |= checksum(out);
}

inline void obfuscate(uint8_t f[7]) {
  for (int i = 1; i < 7; ++i) f[i] ^= f[i - 1];
}

inline void deobfuscate(uint8_t f[7]) {
  for (int i = 6; i >= 1; --i) f[i] ^= f[i - 1];
}

// Obfuscated bytes ready to send.
inline void encode(Command cmd, uint16_t code, uint32_t address, uint8_t out[7]) {
  build(cmd, code, address, out);
  obfuscate(out);
}

// Received (obfuscated) bytes → frame. False when the checksum or key nibble is wrong.
inline bool decode(const uint8_t raw[7], Frame& fr) {
  uint8_t f[7];
  for (int i = 0; i < 7; ++i) f[i] = raw[i];
  deobfuscate(f);
  if ((f[0] & 0xF0) != 0xA0) return false;
  if (checksum(f) != 0) return false;
  fr.key = f[0];
  const uint8_t c = f[1] >> 4;
  fr.command = (c == 0x1 || c == 0x2 || c == 0x3 || c == 0x4 || c == 0x5 || c == 0x6 || c == 0x8 || c == 0x9 || c == 0xA)
                   ? static_cast<Command>(c) : Command::Unknown;
  fr.code = static_cast<uint16_t>((f[2] << 8) | f[3]);
  fr.address = (static_cast<uint32_t>(f[4]) << 16) | (static_cast<uint32_t>(f[5]) << 8) | f[6];
  return true;
}

inline uint64_t pack(const uint8_t f[7]) {
  uint64_t v = 0;
  for (int i = 0; i < 7; ++i) v = (v << 8) | f[i];
  return v;
}

// One frame as alternating levels, starting HIGH. `first` adds the wake-up pulse and uses
// 2 hardware syncs; repeats use 7. Returns the number of durations written (max kMaxPulses).
constexpr size_t kMaxPulses = 2 + 14 + 2 + 112 + 1;
inline size_t encodePulses(const uint8_t frame[7], bool first, uint32_t* out, size_t cap) {
  size_t n = 0;
  // Entry k is HIGH when k is even. Halves of the same level as the last entry merge into it.
  auto add = [&](bool high, uint32_t us) {
    if (n && ((n - 1) % 2 == 0) == high) { out[n - 1] += us; return; }
    if (n >= cap) return;
    if ((n % 2 == 0) != high) return;  // can't happen: levels always alternate
    out[n++] = us;
  };
  if (first) { add(true, kWakeHighUs); add(false, kWakeLowUs); }
  const int syncs = first ? 2 : 7;
  for (int s = 0; s < syncs; ++s) { add(true, kHwSyncUs); add(false, kHwSyncUs); }
  add(true, kSwSyncHighUs);
  add(false, kHalfUs);
  for (int i = 0; i < 56; ++i) {
    const bool bit = (frame[i / 8] >> (7 - (i % 8))) & 1;
    if (bit) { add(false, kHalfUs); add(true, kHalfUs); }
    else { add(true, kHalfUs); add(false, kHalfUs); }
  }
  add(false, kGapUs);
  return n;
}

// Edge-stream decoder: feed each pulse (duration, level). Returns true with the 7 received
// (obfuscated) bytes in `out` when a frame with a good checksum has been read. Frames longer
// than 56 bits (some newer remotes send 80) are read from their first 56 bits.
class Decoder {
 public:
  bool feed(uint32_t us, bool high, uint8_t out[7]) {
    switch (state_) {
      case State::Seek:
        if (high && in(us, 1800, 3300)) { syncs_ = 1; state_ = State::HwSync; }
        return false;
      case State::HwSync:
        if (in(us, 1800, 3300)) { if (high) syncs_++; return false; }
        if (high && syncs_ >= 2 && in(us, 3400, 6000)) { state_ = State::Data; halves_ = 0; skip_ = true; return false; }
        restart(us, high);
        return false;
      case State::Data: {
        int n = 0;
        if (in(us, 380, 960)) n = 1;
        else if (in(us, 961, 1650)) n = 2;
        else {
          // End of frame: the last low half of a 0 bit merges into the gap.
          bool done = false;
          if (!high && halves_ >= 111) {
            if (halves_ < 112) push(false);
            done = finish(out);
          }
          restart(us, high);
          return done;
        }
        for (int k = 0; k < n; ++k) {
          if (skip_) { skip_ = false; if (high) { restart(us, high); return false; } continue; }  // the low after the sync
          push(high);
        }
        return false;
      }
    }
    return false;
  }
  void clear() { state_ = State::Seek; }

 private:
  enum class State : uint8_t { Seek, HwSync, Data };
  static bool in(uint32_t v, uint32_t lo, uint32_t hi) { return v >= lo && v <= hi; }
  void push(bool level) {
    if (halves_ < 112) levels_[halves_] = level;
    if (halves_ < 255) halves_++;
  }
  bool finish(uint8_t out[7]) {
    for (int i = 0; i < 7; ++i) out[i] = 0;
    for (int b = 0; b < 56; ++b) {
      const bool a = levels_[b * 2], c = levels_[b * 2 + 1];
      if (a == c) return false;  // not Manchester
      if (!a && c) out[b / 8] |= static_cast<uint8_t>(1u << (7 - (b % 8)));
    }
    Frame f;
    return decode(out, f);
  }
  void restart(uint32_t us, bool high) {
    state_ = State::Seek;
    if (high && in(us, 1800, 3300)) { syncs_ = 1; state_ = State::HwSync; }
  }
  State state_ = State::Seek;
  uint8_t syncs_ = 0;
  uint16_t halves_ = 0;
  bool skip_ = false;
  bool levels_[112] = {};
};

}  // namespace somfy
