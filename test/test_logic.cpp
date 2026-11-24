// Host test for drift correction and the extra host names: g++ -std=c++17 -I include test/test_logic.cpp
#include <cstdio>
#include <string>

#include "aok_drift.h"
#include "aok_hosts.h"

static int failures = 0;
#define CHECK(c) do { if (!(c)) { std::printf("FAIL line %d: %s\n", __LINE__, #c); ++failures; } } while (0)

int main() {
  using aok::Resync;
  // off, or not enough part-way moves yet
  CHECK(aok::resyncPlan(50, 0, 0, true) == Resync::None);
  CHECK(aok::resyncPlan(9, 10, 0, true) == Resync::None);
  // due: a move to an end waits the full time; a middle target only goes via an end when enabled
  CHECK(aok::resyncPlan(10, 10, 0, false) == Resync::AtEnd);
  CHECK(aok::resyncPlan(12, 10, 100, false) == Resync::AtEnd);
  CHECK(aok::resyncPlan(10, 10, 40, false) == Resync::None);
  CHECK(aok::resyncPlan(10, 10, 40, true) == Resync::ViaEnd);
  CHECK(aok::resyncPlan(1, 1, 99, true) == Resync::ViaEnd);
  // the shorter trip: 30 -> 0 -> 40 is 70; 30 -> 100 -> 40 is 130
  CHECK(aok::resyncEnd(30, 40) == 0);
  CHECK(aok::resyncEnd(70, 60) == 100);
  CHECK(aok::resyncEnd(50, 50) == 0);
  // counting
  uint8_t n = 0;
  for (int k = 0; k < 10; ++k) n = aok::afterRun(n, false);
  CHECK(n == 10);
  CHECK(aok::afterRun(n, true) == 0);
  CHECK(aok::afterRun(255, false) == 255);
  // the whole loop: ten part-way moves, then a middle target goes via the nearer end, then counting starts again
  n = 0;
  for (int k = 0; k < 10; ++k) { CHECK(aok::resyncPlan(n, 10, 30, true) == Resync::None); n = aok::afterRun(n, false); }
  CHECK(aok::resyncPlan(n, 10, 30, true) == Resync::ViaEnd);
  n = aok::afterRun(n, true);   // the run to the end stop
  n = aok::afterRun(n, false);  // then on to 30
  CHECK(n == 1 && aok::resyncPlan(n, 10, 30, true) == Resync::None);

  std::string out;
  CHECK(aok::parseHostList("", out) == 0 && out.empty());
  CHECK(aok::parseHostList("AOK-Blinds.lan, blinds.home.arpa.", out) == 2 && out == "aok-blinds.lan,blinds.home.arpa");
  CHECK(aok::parseHostList("a.lan b.lan,a.lan", out) == 2 && out == "a.lan,b.lan");
  CHECK(aok::parseHostList("a,b,c,d", out) == 4);
  CHECK(aok::parseHostList("a,b,c,d,e", out) == -1);
  CHECK(aok::parseHostList("-bad.lan", out) == -1);
  CHECK(aok::parseHostList("bad-.lan", out) == -1);
  CHECK(aok::parseHostList("a..lan", out) == -1);
  CHECK(aok::parseHostList("evil.com/x", out) == -1);
  CHECK(aok::parseHostList("under_score.lan", out) == -1);
  CHECK(aok::parseHostList(std::string(64, 'a') + ".lan", out) == -1);
  CHECK(aok::parseHostList(std::string(63, 'a') + ".lan", out) == 1);
  CHECK(aok::hostInList("aok-blinds.lan", "aok-blinds.lan,blinds.home.arpa"));
  CHECK(aok::hostInList("blinds.home.arpa", "aok-blinds.lan,blinds.home.arpa"));
  CHECK(!aok::hostInList("blinds.lan", "aok-blinds.lan"));
  CHECK(!aok::hostInList("", "aok-blinds.lan"));
  CHECK(!aok::hostInList("a.lan", ""));
  if (failures) return 1;
  std::printf("drift and host names ok\n");
  return 0;
}
