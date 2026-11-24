// Drift correction for the time-based position model. Positions are worked out from
// run times, so every move that stops part way adds a little error; a run that ends
// at an end stop puts it right. Header-only and free of Arduino so the host tests can check it.
#pragma once

#include <cstdint>

namespace aok {

enum class Resync : uint8_t {
  None,    // move as asked
  AtEnd,   // the move goes to an end stop: wait the full travel time so the end is certain
  ViaEnd,  // a middle target: run to an end stop first, then on to the target
};

// What the next move does, given `partials` part-way moves since the last end stop,
// the setting `every` (0 = off), the target (0-100) and the "re-sync through an end
// stop" option. A middle target only goes via an end when that option is on, since
// people see the blind take the long way round.
inline Resync resyncPlan(int partials, int every, int target, bool through) {
  if (every <= 0 || partials < every) return Resync::None;
  if (target <= 0 || target >= 100) return Resync::AtEnd;
  return through ? Resync::ViaEnd : Resync::None;
}

// The end stop to pass on the way from cur to target: the one that makes the shorter trip.
inline int resyncEnd(int cur, int target) { return cur + target <= 100 ? 0 : 100; }

// The part-way count after a run ends: a run that finished at an end stop starts it again.
inline uint8_t afterRun(uint8_t partials, bool reachedEnd) {
  if (reachedEnd) return 0;
  return partials < 255 ? static_cast<uint8_t>(partials + 1) : partials;
}

}  // namespace aok
