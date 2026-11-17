// Host test for the sunrise/sunset maths: g++ -std=c++17 -I include test/test_sun.cpp
#include <cstdio>
#include <cstdlib>

#include "aok_sun.h"

static int failures = 0;

static void near(const char* what, int got, int want, int tolerance = 4) {
  if (std::abs(got - want) > tolerance) {
    std::printf("FAIL %s: got %02d:%02d, want %02d:%02d\n", what, got / 60, got % 60, want / 60, want % 60);
    ++failures;
  }
}

int main() {
  // Melbourne (-37.81, 144.96). 17 Nov is day 320 (yday 320 in a non-leap year), AEDT = UTC+11.
  // Published times: sunrise 06:03, sunset 20:15.
  near("Melbourne sunrise 17 Nov", aok::sunMinutes(true, 320, -37.81, 144.96, 660), 6 * 60 + 3);
  near("Melbourne sunset 17 Nov", aok::sunMinutes(false, 320, -37.81, 144.96, 660), 20 * 60 + 15);
  // Midwinter, 21 Jun (yday 171), AEST = UTC+10: sunrise 07:35, sunset 17:08.
  near("Melbourne sunrise 21 Jun", aok::sunMinutes(true, 171, -37.81, 144.96, 600), 7 * 60 + 35);
  near("Melbourne sunset 21 Jun", aok::sunMinutes(false, 171, -37.81, 144.96, 600), 17 * 60 + 8);
  // London (51.51, -0.13), 21 Jun, BST = UTC+1: sunrise 04:43, sunset 21:21.
  near("London sunrise 21 Jun", aok::sunMinutes(true, 171, 51.51, -0.13, 60), 4 * 60 + 43);
  near("London sunset 21 Jun", aok::sunMinutes(false, 171, 51.51, -0.13, 60), 21 * 60 + 21);
  // Polar night: Tromsø (69.65) in late December has no sunrise.
  if (aok::sunMinutes(true, 355, 69.65, 18.96, 60) != -1) { std::printf("FAIL polar night\n"); ++failures; }
  if (failures) return 1;
  std::printf("sun times ok\n");
  return 0;
}
