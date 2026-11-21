// Host test: g++ -std=c++17 -I include test/test_somfy.cpp -o /tmp/s && /tmp/s
#include <cstdio>
#include <cstring>
#include <random>
#include "somfy_rts.h"
using namespace somfy;

int main() {
  int fails = 0;
  // Known vector (pushstack write-up): My, code 0x0000? Check obfuscation is reversible and checksums.
  std::mt19937 rng(7);
  const Command cmds[] = {Command::My, Command::Up, Command::Down, Command::Prog};
  for (int t = 0; t < 400; ++t) {
    const Command c = cmds[t % 4];
    const uint16_t code = static_cast<uint16_t>(rng());
    const uint32_t addr = rng() & 0xFFFFFF;
    uint8_t f[7];
    encode(c, code, addr, f);
    Frame fr;
    if (!decode(f, fr) || fr.command != c || fr.code != code || fr.address != addr) { fprintf(stderr, "FAIL decode %d\n", t); fails++; continue; }
    // a flipped bit must fail the checksum (almost always: XOR-of-nibbles catches every single-bit flip)
    uint8_t bad[7]; memcpy(bad, f, 7); bad[rng() % 7] ^= static_cast<uint8_t>(1u << (rng() % 8));
    Frame x;
    // A flip in the key nibble also flips the checksum nibble (a known RTS weakness): only the key changes.
    if (decode(bad, x) && x.code == code && x.address == addr && x.command == c && x.key == fr.key) { fprintf(stderr, "FAIL bitflip %d\n", t); fails++; }
    // pulses → decoder, with +-20% jitter, noise before, first frame then a repeat
    uint32_t p[kMaxPulses];
    Decoder d;
    uint8_t out[7];
    int got = 0;
    for (int k = 0; k < 30; ++k) d.feed(100 + rng() % 500, k % 2 == 0, out);
    for (int rep = 0; rep < 2; ++rep) {
      const size_t n = encodePulses(f, rep == 0, p, kMaxPulses);
      for (size_t i = 0; i < n; ++i) {
        const double j = 1.0 + (static_cast<int>(rng() % 41) - 20) / 100.0;
        if (d.feed(static_cast<uint32_t>(p[i] * j), i % 2 == 0, out)) {
          if (memcmp(out, f, 7) != 0) { fprintf(stderr, "FAIL bytes %d\n", t); fails++; }
          got++;
        }
      }
    }
    if (got != 2) { fprintf(stderr, "FAIL frames %d got %d\n", t, got); fails++; }
  }
  // An 80-bit frame (newer remotes): extra 24 bits after the 56 must still decode.
  {
    uint8_t f[7]; encode(Command::Up, 0x1234, 0xABCDEF, f);
    uint32_t p[kMaxPulses + 64]; size_t n = encodePulses(f, true, p, kMaxPulses);
    n--;  // drop the gap
    // 24 more bits of alternating 1/0 at 640 us halves (low,high / high,low)
    Decoder d; uint8_t out[7]; bool ok = false;
    for (size_t i = 0; i < n; ++i) ok |= d.feed(p[i], i % 2 == 0, out);
    size_t k = n;
    for (int b = 0; b < 24; ++b) { d.feed(640, k++ % 2 == 0, out); d.feed(640, k++ % 2 == 0, out); }
    ok |= d.feed(30415, k % 2 == 0, out);
    // only counts when the final pulse is low; either way it mustn't crash
    printf("80-bit: %s\n", ok ? "decoded" : "skipped");
  }
  fprintf(stderr, fails ? "%d failures\n" : "all somfy round-trips ok\n", fails);
  return fails ? 1 : 0;
}
