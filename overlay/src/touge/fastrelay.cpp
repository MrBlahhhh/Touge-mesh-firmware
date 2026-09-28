#include "fastrelay.h"

namespace touge {
namespace {

// prevGap_ steps: fine enough against FAST_RELAY_FRESH_MS, and a byte reaches
// past it.
const uint32_t GAP_STEP_MS = 16;
const uint8_t GAP_NONE = 0xFF;
static_assert(GAP_NONE * GAP_STEP_MS > FAST_RELAY_FRESH_MS, "a gap too long to be fresh must fit below GAP_NONE");

// Since `atMs`, for the relay's own stamps. Unsigned: seats are never aged out,
// so a seat left for weeks has to read as long gone, not as just heard. A
// stamp up to a second ahead of nowMs reads as now: a map fed on its receive
// time and judged on a pass that began a moment before.
uint32_t msSince(uint32_t atMs, uint32_t nowMs) {
  const uint32_t age = nowMs - atMs;
  return age > 0u - SCHEDULE_MS ? 0 : age;
}

uint8_t gapSteps(uint32_t gapMs) {
  const uint32_t steps = (gapMs + GAP_STEP_MS - 1) / GAP_STEP_MS;
  return steps < GAP_NONE ? (uint8_t)steps : GAP_NONE;
}

// On the ride as Schedule counts it: heard within a lease. An id equal to ours
// is a node number collision, never us. A stamp later than nowMs is on it.
bool onRide(const Rider& r, uint32_t selfId, uint32_t nowMs) {
  return r.used && r.id != 0 && r.id != selfId && riderAgeMs(r, nowMs) < LEASE_MS;
}

const Rider* findOnRide(uint32_t id, uint32_t selfId, const Rider* riders, size_t maxRiders, uint32_t nowMs) {
  for (size_t i = 0; i < maxRiders; i++) {
    if (riders[i].id == id && onRide(riders[i], selfId, nowMs)) return &riders[i];
  }
  return nullptr;
}

// The one car on the ride holding `slot`, or nullptr when none does or two
// claim it: in a clash the maps show whichever got through, and a shared tag
// would pass one for the other.
const Rider* holderOf(uint8_t slot, uint32_t selfId, const Rider* riders, size_t maxRiders, uint32_t nowMs) {
  if (slot >= MAX_SLOTS) return nullptr;
  const Rider* holder = nullptr;
  for (size_t i = 0; i < maxRiders; i++) {
    if (!onRide(riders[i], selfId, nowMs) || riders[i].pos.slot != slot) continue;
    if (holder != nullptr) return nullptr;
    holder = &riders[i];
  }
  return holder;
}

// The one car on the ride with `tag`, or nullptr when none has it or two do.
const Rider* taggedOnRide(uint8_t tag, uint32_t selfId, const Rider* riders, size_t maxRiders, uint32_t nowMs) {
  if (tag == 0) return nullptr;
  const Rider* found = nullptr;
  for (size_t i = 0; i < maxRiders; i++) {
    if (!onRide(riders[i], selfId, nowMs) || slotTag(riders[i].id) != tag) continue;
    if (found != nullptr) return nullptr;
    found = &riders[i];
  }
  return found;
}

// Mesh::note turns a newcomer away only when every seat is taken and none has
// been quiet for RIDER_STALE_MS, by the same riderAgeMs. Then a car we have
// never heard may be on the ride, and nobody can be shown not to need a forward.
bool rosterMayMissCars(const Rider* riders, size_t maxRiders, uint32_t nowMs) {
  for (size_t i = 0; i < maxRiders; i++) {
    if (!riders[i].used || riderAgeMs(riders[i], nowMs) > RIDER_STALE_MS) return false;
  }
  return true;
}

}  // namespace

bool mapCoversTwoBeacons(uint8_t reporterSlot, uint8_t originSlot) {
  if (reporterSlot >= MAX_SLOTS || originSlot >= MAX_SLOTS) return true;
  // How long before the reporter's slot the origin's opens, round the second.
  const uint32_t leadMs = (slotStartMs(reporterSlot) + SCHEDULE_MS - slotStartMs(originSlot)) % SCHEDULE_MS;
  // The older of two beacons is still inside the map's window while the lead
  // is under half a second (HEARD_WINDOW_MS less the second between them).
  const uint32_t twoUnderMs = HEARD_WINDOW_MS - SCHEDULE_MS;
  const bool clearlyOne = leadMs >= twoUnderMs + FAST_RELAY_OLDER_MARGIN_MS &&
                          leadMs + FAST_RELAY_NEXT_MARGIN_MS <= SCHEDULE_MS;
  return !clearlyOne;
}

void FastRelay::reset() { *this = FastRelay(); }

size_t FastRelay::seatOf(uint32_t id) const {
  if (id == 0) return FAST_RELAY_CARS;
  for (size_t i = 0; i < FAST_RELAY_CARS; i++) {
    if (carId_[i] == id) return i;
  }
  return FAST_RELAY_CARS;
}

size_t FastRelay::takeSeat(uint32_t id, uint32_t nowMs) {
  size_t seat = FAST_RELAY_CARS;
  for (size_t i = 0; i < FAST_RELAY_CARS && seat == FAST_RELAY_CARS; i++) {
    if (carId_[i] == 0) seat = i;
  }
  if (seat == FAST_RELAY_CARS) {
    seat = 0;
    for (size_t i = 1; i < FAST_RELAY_CARS; i++) {
      if ((uint32_t)(nowMs - mapAtMs_[i]) > (uint32_t)(nowMs - mapAtMs_[seat])) seat = i;
    }
    // Every seat holds a car on the ride, so more cars are about than the 27
    // this counts on, and one may now be on no list at all.
    if (msSince(mapAtMs_[seat], nowMs) < LEASE_MS) {
      crowded_ = true;
      crowdedAtMs_ = nowMs;
    }
  }
  // Nothing said by or about the car that had it carries over.
  clearAsReporter(seat);
  clearAsOrigin(seat);
  carId_[seat] = id;
  carSlot_[seat] = SLOT_NONE;
  mapAtMs_[seat] = nowMs;
  prevGap_[seat] = GAP_NONE;
  return seat;
}

void FastRelay::clearAsReporter(size_t seat) {
  for (size_t t = 0; t < FAST_RELAY_CARS; t++) shown_[seat][t] = 0;
}

void FastRelay::clearAsOrigin(size_t seat) {
  for (size_t n = 0; n < FAST_RELAY_CARS; n++) shown_[n][seat] = 0;
}

void FastRelay::heardMap(uint32_t senderId, const Position& p, uint32_t nowMs) {
  if (senderId == 0 || p.extra) return;
  size_t n = seatOf(senderId);
  if (n == FAST_RELAY_CARS) {
    n = takeSeat(senderId, nowMs);
  } else if (carSlot_[n] != p.slot) {
    // A new lease changes the phase between this car's maps and everyone
    // else's beacons, which sets k, and where its own column is read. Counted
    // however soon it comes, so the column is never read at the old slot.
    clearAsReporter(n);
    clearAsOrigin(n);
    prevGap_[n] = GAP_NONE;
  } else {
    const uint32_t gapMs = msSince(mapAtMs_[n], nowMs);
    // Nothing moves, freshness included: refreshed by maps that do not count,
    // a car beaconing four times a second would keep a frozen record fresh.
    if (gapMs < FAST_RELAY_MAP_SPACING_MS) return;
    if (gapMs >= FAST_RELAY_GAP_MS) clearAsReporter(n);
    prevGap_[n] = gapSteps(gapMs);
  }
  carSlot_[n] = p.slot;
  mapAtMs_[n] = nowMs;

  for (size_t t = 0; t < FAST_RELAY_CARS; t++) {
    if (t == n || carId_[t] == 0) continue;
    // Car t's own slot with car t's own tag, and nothing else in the map.
    const uint8_t s = carSlot_[t];
    const bool shown = s < MAX_SLOTS && p.slotMap[s] == slotTag(carId_[t]);
    shown_[n][t] = (uint8_t)((shown_[n][t] << 1) | (shown ? 1 : 0));
  }
}

bool FastRelay::onRideHere(size_t seat, uint32_t selfId, uint32_t nowMs) const {
  return carId_[seat] != 0 && carId_[seat] != selfId && msSince(mapAtMs_[seat], nowMs) < LEASE_MS;
}

// Unsigned, so a map stamped later than nowMs reads as stale.
bool FastRelay::fresh(size_t seat, uint32_t nowMs) const {
  return (uint32_t)(nowMs - mapAtMs_[seat]) < FAST_RELAY_FRESH_MS;
}

// Unsigned as fresh() is, and the gap rounded up, so either way errs old.
bool FastRelay::previousMapFresh(size_t seat, uint32_t nowMs) const {
  if (prevGap_[seat] == GAP_NONE) return false;
  const uint32_t latestMs = nowMs - mapAtMs_[seat];
  return latestMs < FAST_RELAY_FRESH_MS && latestMs + prevGap_[seat] * GAP_STEP_MS < FAST_RELAY_FRESH_MS;
}

bool FastRelay::hearsSteadily(size_t reporter, size_t origin, uint32_t nowMs) const {
  const uint8_t shown = shown_[reporter][origin];
  const bool twoBeacons = mapCoversTwoBeacons(carSlot_[reporter], carSlot_[origin]);
  const int need = twoBeacons ? FAST_RELAY_SHOWN_TWO : FAST_RELAY_SHOWN_ONE;
  if (__builtin_popcount(shown) < need) return false;
  // The origin got through lately: the latest map shows it (it covers the last
  // two beacons when a map covers two), or the map before does and is fresh.
  if ((shown & 0x01) != 0) return true;
  return !twoBeacons && (shown & 0x02) != 0 && previousMapFresh(reporter, nowMs);
}

size_t FastRelay::placeRelayer(uint8_t tag, uint32_t selfId, const Rider* riders, size_t maxRiders,
                               uint32_t nowMs) const {
  const Rider* relayer = taggedOnRide(tag, selfId, riders, maxRiders, nowMs);
  if (relayer == nullptr) return FAST_RELAY_CARS;
  // A car on no roster that sends us maps may have the tag too.
  for (size_t i = 0; i < FAST_RELAY_CARS; i++) {
    if (onRideHere(i, selfId, nowMs) && carId_[i] != relayer->id && slotTag(carId_[i]) == tag)
      return FAST_RELAY_CARS;
  }
  // Its column is read at the lease its maps came with, so that has to be the
  // lease the roster has for it.
  const size_t r = seatOf(relayer->id);
  if (r == FAST_RELAY_CARS || carSlot_[r] != relayer->pos.slot) return FAST_RELAY_CARS;
  return r;
}

FastRelayVerdict FastRelay::standing(size_t k, size_t origin, size_t relayer, uint32_t nowMs) const {
  if (k == FAST_RELAY_CARS) return FastRelayVerdict::NO_EVIDENCE;
  if (!fresh(k, nowMs)) return FastRelayVerdict::STALE;
  if (hearsSteadily(k, origin, nowMs)) return FastRelayVerdict::SKIP;
  if (relayer != FAST_RELAY_CARS && hearsSteadily(k, relayer, nowMs)) return FastRelayVerdict::SKIP;
  return FastRelayVerdict::NEEDED;
}

FastRelayVerdict FastRelay::judge(uint32_t selfId, uint32_t origin, bool relayed, uint8_t relayerTag,
                                  const Rider* riders, size_t maxRiders, uint32_t nowMs,
                                  bool* relayerUnplaced) const {
  if (relayerUnplaced != nullptr) *relayerUnplaced = false;
  if (rosterMayMissCars(riders, maxRiders, nowMs)) return FastRelayVerdict::NO_EVIDENCE;
  if (crowded_ && msSince(crowdedAtMs_, nowMs) < LEASE_MS) return FastRelayVerdict::NO_EVIDENCE;

  // The origin has to hold a lease of its own for any map to show it.
  const Rider* originRider = findOnRide(origin, selfId, riders, maxRiders, nowMs);
  if (originRider == nullptr) return FastRelayVerdict::NO_EVIDENCE;
  if (holderOf(originRider->pos.slot, selfId, riders, maxRiders, nowMs) != originRider)
    return FastRelayVerdict::NO_EVIDENCE;
  const size_t o = seatOf(origin);
  if (o == FAST_RELAY_CARS || carSlot_[o] != originRider->pos.slot) return FastRelayVerdict::NO_EVIDENCE;

  // A relayer we cannot place leaves the origin alone to judge by.
  size_t r = FAST_RELAY_CARS;
  uint32_t relayerId = 0;
  if (relayed) {
    r = placeRelayer(relayerTag, selfId, riders, maxRiders, nowMs);
    if (r != FAST_RELAY_CARS) {
      relayerId = carId_[r];
    } else if (relayerUnplaced != nullptr) {
      *relayerUnplaced = true;
    }
  }

  FastRelayVerdict verdict = FastRelayVerdict::SKIP;
  for (size_t i = 0; i < maxRiders; i++) {
    const Rider& car = riders[i];
    // The relayer has the frame: it just sent it.
    if (!onRide(car, selfId, nowMs) || car.id == origin || car.id == relayerId) continue;
    const FastRelayVerdict v = standing(seatOf(car.id), o, r, nowMs);
    if (v == FastRelayVerdict::NEEDED) return v;
    if (v > verdict) verdict = v;
  }
  // Cars sending us maps that are on no roster: Mesh::note turned them away
  // while every seat was live, and a seat going stale since does not put them
  // back until their next position gets through.
  for (size_t k = 0; k < FAST_RELAY_CARS; k++) {
    if (!onRideHere(k, selfId, nowMs) || carId_[k] == origin || carId_[k] == relayerId) continue;
    if (findOnRide(carId_[k], selfId, riders, maxRiders, nowMs) != nullptr) continue;
    const FastRelayVerdict v = standing(k, o, r, nowMs);
    if (v == FastRelayVerdict::NEEDED) return v;
    if (v > verdict) verdict = v;
  }
  return verdict;
}

void FastRelaySkips::note(FastRelayVerdict v, bool relayerUnplaced) {
  if (relayerUnplaced) noRelayer++;
  switch (v) {
    case FastRelayVerdict::SKIP:
      skipped++;
      break;
    case FastRelayVerdict::NO_EVIDENCE:
      noEvidence++;
      break;
    case FastRelayVerdict::STALE:
      stale++;
      break;
    case FastRelayVerdict::NEEDED:
    default:
      needed++;
      break;
  }
}

}  // namespace touge
