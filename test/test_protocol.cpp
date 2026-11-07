// Host test: g++ -std=c++17 -I include test/test_protocol.cpp -o /tmp/t && /tmp/t
#include <cstdio>
#include <cstdlib>
#include <random>
#include "aok_protocol.h"
using namespace aok;
int main() {
  const uint32_t prefix = 0xA1B2C3; const uint8_t rid = 0x42;
  const uint16_t bases[3] = {0xF42A, 0xBCF2, 0xDC12};
  const uint16_t masks[] = {0x0001, 0x0002, 0x0004, 0x0007, 0x0020, 0x003F, 0x8000, 0x0101};
  std::mt19937 rng(1);
  int fails = 0;
  for (uint16_t m : masks) for (int a = 0; a < 3; ++a) {
    uint64_t p = makePayload(prefix, rid, m, bases[a]);
    uint16_t pulses[kFramePulses]; size_t n = encodeFrame(p, Timings{}, pulses, kFramePulses);
    printf("%04X %d %016llX", m, a, (unsigned long long)p);
    for (size_t i = 0; i < n; ++i) printf(" %u", pulses[i]);
    printf("\n");
    // round trip with +-15% jitter, preceded by noise and followed by gap
    Decoder d; uint64_t out = 0; bool got = false;
    for (int k = 0; k < 40; ++k) d.feed(50 + rng() % 400, k % 2 == 0, out);
    for (size_t i = 0; i < n && !got; ++i) {
      double j = 1.0 + ((int)(rng() % 31) - 15) / 100.0;
      got = d.feed((uint32_t)(pulses[i] * j), i % 2 == 0, out);
    }
    Frame f = splitPayload(out);
    bool ok = got && out == p && f.mask == m && (int)f.action == a && deriveBase(rid, m, f.command) == bases[a];
    if (!ok) { fprintf(stderr, "FAIL mask %04X action %d got=%d\n", m, a, got); fails++; }
  }
  fprintf(stderr, fails ? "%d failures\n" : "all round-trips ok\n", fails);
  return fails ? 1 : 0;
}
