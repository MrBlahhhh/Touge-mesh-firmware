#include "relaypref.h"

#include "kvline.h"

namespace touge {

void RelayPrefs::clear() { *this = RelayPrefs(); }

RelayPrefs::Verdict RelayPrefs::consider(const ReachEntry& e, uint32_t reporter, uint32_t self, uint8_t selfByte,
                                         bool weHearIt, uint32_t maxSinceS, uint32_t holdMs, uint32_t nowMs) {
  // Our own positions, our own summary, or an origin reporting on itself.
  if (e.origin == 0 || e.origin == self || reporter == self || reporter == e.origin) return Verdict::NONE;
  // Heard directly, or first through somebody else: nothing about our relaying.
  if (e.hops == 0 || e.hops == REACH_HOPS_UNKNOWN || e.relay != selfByte) return Verdict::NONE;

  Pref* held = nullptr;
  for (size_t i = 0; i < PREFER_SLOTS && held == nullptr; i++) {
    if (live(prefs_[i], nowMs) && prefs_[i].origin == e.origin) held = &prefs_[i];
  }
  // An unknown age (no fix time on either end) is no evidence of delivery.
  const uint32_t ageMs = reachAgeMs(e.ageQ);
  const bool good = weHearIt && ageMs <= PREFER_MAX_AGE_MS && e.sinceS <= maxSinceS;
  if (!good) {
    if (held == nullptr) return Verdict::NONE;
    *held = Pref();
    withdrawn_++;
    return Verdict::WITHDRAWN;
  }
  if (held != nullptr) {
    held->untilMs = nowMs + holdMs;
    return Verdict::RENEWED;
  }
  Pref* place = nullptr;
  for (size_t i = 0; i < PREFER_SLOTS && place == nullptr; i++) {
    if (!live(prefs_[i], nowMs)) place = &prefs_[i];
  }
  if (place == nullptr) {
    place = &prefs_[0];
    for (size_t i = 1; i < PREFER_SLOTS; i++) {
      if ((int32_t)(prefs_[i].untilMs - place->untilMs) < 0) place = &prefs_[i];
    }
  }
  place->origin = e.origin;
  place->untilMs = nowMs + holdMs;
  granted_++;
  return Verdict::GRANTED;
}

bool RelayPrefs::preferred(uint32_t origin, uint32_t nowMs) const {
  for (size_t i = 0; i < PREFER_SLOTS; i++) {
    if (prefs_[i].origin == origin && live(prefs_[i], nowMs)) return true;
  }
  return false;
}

size_t RelayPrefs::count(uint32_t nowMs) const {
  size_t n = 0;
  for (size_t i = 0; i < PREFER_SLOTS; i++) n += live(prefs_[i], nowMs) ? 1 : 0;
  return n;
}

size_t formatLoraReach(const Reach& reach, const RelayPrefs& prefs, bool json, char* out, size_t cap) {
  KvLine line(out, cap, "le", json);
  line.add("st", reach.summariesSent());
  line.add("sh", reach.summariesHeard());
  line.add("pg", prefs.granted());
  line.add("pw", prefs.withdrawn());
  return line.finish();
}

}  // namespace touge
