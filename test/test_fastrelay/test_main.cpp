// Host tests for the 2.4 GHz relay rule read from slot maps (fastrelay.h): a
// forward is skipped only when every other car on the ride hears the origin
// steadily, by the evidence of that car's own slot maps.
//
// The rule's edges are tested on maps written out by hand, with a real Mesh as
// the roster so its turn-aways and evictions are the firmware's own. Links with
// loss are tested on maps from a real Schedule per listening car (heardBeacon,
// then fillSlotMap as its beacon goes out), so the 1.5 s map window and its
// phase against the origin's slot are the firmware's own.

#include <unity.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <vector>
#include "fastrelay.h"

using namespace touge;

void setUp() {}
void tearDown() {}

static const uint32_t SELF = 0x5E1F0001;

static uint32_t rng = 12345;
static uint32_t rand32() {
  rng = rng * 1664525u + 1013904223u;
  return rng >> 8;
}
// True `pct` times in a hundred.
static bool chance(int pct) { return (int)(rand32() % 100) < pct; }

// ---- A ride whose maps are written by hand ----------------------------------

// Cars on slots, each hearing every other car unless told otherwise. Every
// second each car beacons in its slot with a map of the cars it hears, and we
// hear each beacon; the verdict is asked at the end of the second.
struct Ride {
  struct Car {
    uint32_t id;
    uint8_t slot;
  };
  std::vector<Car> cars;
  std::vector<std::vector<bool>> deaf;  // deaf[a][b]: car a's maps never show car b
  std::vector<bool> onlyExtras;         // only car a's extras reach us: on the ride, no maps
  std::vector<bool> quiet;              // car a sends nothing at all
  std::vector<bool> unheard;            // car a's frames never reach us, though the others hear it
  std::vector<bool> offRoster;          // car a's maps reach us, its positions never the roster
  std::vector<uint8_t> junkTag;         // written in every slot of car a's map that no car holds
  Mesh mesh;
  FastRelay relay;
  uint32_t nowMs = 100000;

  Ride() { mesh.reset(); }

  size_t add(uint32_t id, uint8_t slot) {
    cars.push_back({id, slot});
    for (auto& row : deaf) row.push_back(false);
    deaf.push_back(std::vector<bool>(cars.size(), false));
    onlyExtras.push_back(false);
    quiet.push_back(false);
    unheard.push_back(false);
    offRoster.push_back(false);
    junkTag.push_back(0);
    return cars.size() - 1;
  }

  const Rider* riders() const { return mesh.riders(); }
  const Rider* rider(size_t a) const { return mesh.find(cars[a].id); }

  // Car a's beacon as we hear it at `atMs`: noted on the roster (which may turn
  // it away) and, unless it is an extra, fed to the relay record.
  void beacon(size_t a, uint32_t atMs) {
    Position p;
    p.slot = cars[a].slot;
    for (uint8_t s = 0; s < MAX_SLOTS; s++) p.slotMap[s] = junkTag[a];
    for (size_t b = 0; b < cars.size(); b++) {
      if (cars[b].slot >= MAX_SLOTS) continue;
      const bool shows = b != a && !deaf[a][b] && !quiet[b];
      // Two cars on one slot: the map shows whichever it heard last.
      if (shows) p.slotMap[cars[b].slot] = slotTag(cars[b].id);
      else if (p.slotMap[cars[b].slot] == junkTag[a]) p.slotMap[cars[b].slot] = 0;
    }
    p.extra = onlyExtras[a];
    if (!offRoster[a]) mesh.note(cars[a].id, p, HEARD_FAST, -50, 0, atMs, 1);
    relay.heardMap(cars[a].id, p, atMs);
  }

  // When car a beacons in the second starting at `base`: in its slot, or in the
  // shared window after block 0 when unleased.
  uint32_t beaconAt(size_t a, uint32_t base) const {
    return base + (cars[a].slot < MAX_SLOTS ? slotStartMs(cars[a].slot) : SHARED_OFFSET_MS) + 2;
  }

  void second() {
    const uint32_t base = nowMs - nowMs % SCHEDULE_MS + SCHEDULE_MS;
    std::vector<size_t> order;
    for (size_t a = 0; a < cars.size(); a++) {
      if (!quiet[a] && !unheard[a]) order.push_back(a);
    }
    // In slot order, and cars added first first within a slot.
    std::stable_sort(order.begin(), order.end(),
                     [&](size_t x, size_t y) { return beaconAt(x, base) < beaconAt(y, base); });
    for (size_t a : order) beacon(a, beaconAt(a, base));
    nowMs = base + SCHEDULE_MS - 1;
  }

  void seconds(int n) {
    for (int i = 0; i < n; i++) second();
  }

  FastRelayVerdict judge(size_t origin, bool relayed = false, uint8_t relayerTag = 0,
                         bool* relayerUnplaced = nullptr) const {
    return relay.judge(SELF, cars[origin].id, relayed, relayerTag, riders(), MAX_RIDERS, nowMs, relayerUnplaced);
  }

  uint8_t tagOf(size_t a) const { return slotTag(cars[a].id); }
};

static const char* verdictName(FastRelayVerdict v) {
  switch (v) {
    case FastRelayVerdict::SKIP:
      return "SKIP";
    case FastRelayVerdict::NO_EVIDENCE:
      return "NO_EVIDENCE";
    case FastRelayVerdict::STALE:
      return "STALE";
    case FastRelayVerdict::NEEDED:
      return "NEEDED";
  }
  return "?";
}

#define ASSERT_VERDICT(want, got) TEST_ASSERT_EQUAL_STRING(verdictName(want), verdictName(got))

// Five cars over three blocks, so the park has pairs of both phases.
static void carPark(Ride& ride) {
  ride.add(0xA0000001, 0);
  ride.add(0xA0000002, 4);
  ride.add(0xA0000003, 1);
  ride.add(0xA0000004, 3);
  ride.add(0xA0000005, 9);
}

void test_a_car_park_where_everyone_hears_the_origin_skips() {
  Ride ride;
  carPark(ride);
  // Nothing heard yet: nobody on the ride, the origin included.
  ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE, ride.judge(0));
  // One map each is too little to call anybody steady.
  ride.second();
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(0));
  ride.seconds(FAST_RELAY_MAPS);
  for (size_t origin = 0; origin < ride.cars.size(); origin++) ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(origin));
}

void test_one_car_that_never_shows_the_origin_needs_the_forward() {
  Ride ride;
  carPark(ride);
  ride.deaf[3][0] = true;
  ride.seconds(20);
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(0));
  // Per origin: car 3 hears the others, and car 0 hears car 3.
  for (size_t origin = 1; origin < ride.cars.size(); origin++) ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(origin));
}

void test_a_two_car_ride_skips_every_forward() {
  // The only car to hear our forward would be the one that sent it (1C.2).
  Ride ride;
  ride.add(0xA0000001, 0);
  ride.second();
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));
}

// Car k's maps of car t over the coming seconds, oldest first: '1' shows t.
static void mapsOf(Ride& ride, size_t k, size_t t, const char* pattern) {
  for (const char* c = pattern; *c; c++) {
    ride.deaf[k][t] = *c != '1';
    ride.second();
  }
}

void test_steady_is_k_of_the_last_eight_maps_and_one_of_the_last_two_beacons() {
  // One beacon a map: car 1's slot opens 750 ms after the origin's.
  {
    TEST_ASSERT_FALSE(mapCoversTwoBeacons(3, 0));
    Ride ride;
    ride.add(0xB0000001, 0);
    ride.add(0xB0000002, 3);
    // The first second seats both cars; car 1's record of car 0 starts after.
    ride.second();
    mapsOf(ride, 1, 0, "11110001");
    ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));  // 5 of 8, the latest
    mapsOf(ride, 1, 0, "00000000");
    mapsOf(ride, 1, 0, "01111100");
    ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(0));  // 5 of 8, neither of the latest two
    mapsOf(ride, 1, 0, "1");
    ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));  // 1111100 1: 6 of 8
    mapsOf(ride, 1, 0, "00000000");
    mapsOf(ride, 1, 0, "00011110");
    ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(0));  // 4 of 8
    mapsOf(ride, 1, 0, "00000000");
    mapsOf(ride, 1, 0, "11111110");
    ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));  // 7 of 8, the map before the latest, a second back
  }
  // Two beacons a map: car 1's slot opens 250 ms after the origin's.
  {
    TEST_ASSERT_TRUE(mapCoversTwoBeacons(1, 0));
    Ride ride;
    ride.add(0xB0000001, 0);
    ride.add(0xB0000002, 1);
    ride.second();
    mapsOf(ride, 1, 0, "11101111");
    ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));  // 7 of 8
    mapsOf(ride, 1, 0, "00000000");
    mapsOf(ride, 1, 0, "11011011");
    ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(0));  // 6 of 8
    mapsOf(ride, 1, 0, "00000000");
    mapsOf(ride, 1, 0, "11111110");
    ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(0));  // 7 of 8, but the latest map covers the last two beacons
  }
}

// ---- One origin and one listener, beacon by beacon --------------------------

// T on slot 0 and K on slot 3, one beacon a map, fed by hand so the time of
// each map is ours to choose.
struct Pair {
  static const uint32_t T = 0x7C000001;
  static const uint32_t K = 0x7D000002;
  FastRelay relay;
  Rider riders[MAX_RIDERS];
  Position fromT, fromK;

  Pair(uint8_t kSlot = 3) {
    fromT.slot = 0;
    fromK.slot = kSlot;
  }
  void t(uint32_t atMs) {
    riders[0].used = true;
    riders[0].id = T;
    riders[0].pos = fromT;
    riders[0].atMs = atMs;
    relay.heardMap(T, fromT, atMs);
  }
  // A map from K, showing T or not.
  void k(uint32_t atMs, bool showsT) {
    fromK.slotMap[0] = showsT ? slotTag(T) : 0;
    riders[1].used = true;
    riders[1].id = K;
    riders[1].pos = fromK;
    riders[1].atMs = atMs;
    relay.heardMap(K, fromK, atMs);
  }
  FastRelayVerdict judge(uint32_t atMs) const { return relay.judge(SELF, T, false, 0, riders, MAX_RIDERS, atMs); }
};

void test_the_map_before_the_latest_counts_only_while_recent() {
  TEST_ASSERT_FALSE(mapCoversTwoBeacons(3, 0));
  const uint32_t base = 100000;
  {
    Pair pair;
    for (uint32_t s = 0; s < 10; s++) {
      pair.t(base + s * 1000 + 2);
      pair.k(base + s * 1000 + 752, true);
    }
    // The next map misses T: 7 of 8, the one before a second back.
    pair.t(base + 10002);
    pair.k(base + 10752, false);
    pair.t(base + 11002);
    ASSERT_VERDICT(FastRelayVerdict::SKIP, pair.judge(base + 11002));
    // The same record once the map before is no longer fresh, the latest still
    // is.
    ASSERT_VERDICT(FastRelayVerdict::SKIP, pair.judge(base + 12700));
    ASSERT_VERDICT(FastRelayVerdict::NEEDED, pair.judge(base + 12800));
  }
  {
    // K's maps stop reaching us for five seconds, and the next misses T.
    // The one before it, 9 s back, is no evidence T gets through now.
    Pair pair;
    for (uint32_t s = 0; s < 10; s++) {
      pair.t(base + s * 1000 + 2);
      pair.k(base + s * 1000 + 752, true);
    }
    for (uint32_t s = 10; s < 19; s++) pair.t(base + s * 1000 + 2);
    pair.k(base + 15752, false);
    ASSERT_VERDICT(FastRelayVerdict::NEEDED, pair.judge(base + 18652));
    // Had it been the latest map, fresh, it would count.
    Pair other;
    for (uint32_t s = 0; s < 16; s++) {
      other.t(base + s * 1000 + 2);
      if (s < 10 || s == 15) other.k(base + s * 1000 + 752, true);
    }
    ASSERT_VERDICT(FastRelayVerdict::SKIP, other.judge(base + 15900));
  }
}

void test_maps_faster_than_one_a_second_count_once_a_second() {
  const uint32_t base = 100000;
  {
    // K is unleased and moving, so it beacons five times a second. Only the
    // first each second comes a second after the last one counted, and that
    // one misses T. Counted every time, K would read 7 of 8 and steady.
    Pair pair(SLOT_NONE);
    for (uint32_t s = 0; s < 12; s++) {
      pair.t(base + s * 1000 + 2);
      pair.k(base + s * 1000 + 100, false);
      for (uint32_t i = 1; i < 5; i++) pair.k(base + s * 1000 + 100 + i * 180, true);
    }
    ASSERT_VERDICT(FastRelayVerdict::NEEDED, pair.judge(base + 11900));
  }
  {
    // A lease beacon sent late in its slot and the next sent early both count:
    // one map that misses T, then five that show it, so k 5 needs every one.
    Pair pair;
    pair.t(base + 2);
    pair.k(base + 750, false);
    for (uint32_t s = 1; s <= 5; s++) {
      pair.t(base + s * 1000 + 2);
      pair.k(base + s * 1000 + 750 + (s % 2 ? SLOT_MS - SLOT_GUARD_MS : 0), true);
    }
    ASSERT_VERDICT(FastRelayVerdict::SKIP, pair.judge(base + 5800));
  }
  {
    // A new lease counts however soon it comes: K moves to slot 1 and beacons
    // there half a second after its last. What it heard at the old phase is
    // gone at once.
    Pair pair;
    for (uint32_t s = 0; s < 10; s++) {
      pair.t(base + s * 1000 + 2);
      pair.k(base + s * 1000 + 750, true);
    }
    ASSERT_VERDICT(FastRelayVerdict::SKIP, pair.judge(base + 9800));
    pair.fromK.slot = 1;
    pair.k(base + 10250, true);
    ASSERT_VERDICT(FastRelayVerdict::NEEDED, pair.judge(base + 10300));
  }
}

void test_a_map_that_comes_too_soon_refreshes_nothing() {
  // K, unleased, is steady on T at a map a second. Then it moves and beacons
  // every 250 ms, and T drops out of its maps. The maps that come too soon to
  // count must not keep its record alive, nor keep it fresh.
  const uint32_t base = 100000;
  Pair pair(SLOT_NONE);
  for (uint32_t s = 0; s < 10; s++) {
    pair.t(base + s * 1000 + 2);
    pair.k(base + s * 1000 + 100, true);
  }
  ASSERT_VERDICT(FastRelayVerdict::SKIP, pair.judge(base + 9200));
  for (uint32_t ms = 350; ms < 1000; ms += 250) pair.k(base + 9000 + ms, false);
  for (uint32_t s = 10; s < 17; s++) {
    pair.t(base + s * 1000 + 2);
    if (s == 10) ASSERT_VERDICT(FastRelayVerdict::SKIP, pair.judge(base + 10050));
    // Four maps a second until 13.35 s, the last not counted.
    for (uint32_t ms = 100; ms < 1000 && s * 1000 + ms <= 13350; ms += 250) {
      pair.k(base + s * 1000 + ms, false);
      if (s == 10 && ms == 100) ASSERT_VERDICT(FastRelayVerdict::NEEDED, pair.judge(base + 10200));
    }
  }
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, pair.judge(base + 16050));
  // Its maps stop. Three seconds after the last one counted it is stale, though
  // one came since.
  ASSERT_VERDICT(FastRelayVerdict::STALE, pair.judge(base + 16100));
}

void test_a_seat_left_for_24_days_reads_as_gone() {
  // Seats here are never aged out, and a radio may stay powered for weeks. A
  // seat or a crowding left for 2^31 ms reads as long gone, not as just heard.
  const uint32_t base = 100000;
  const uint32_t later = base + 0x80000000u + 5000;
  {
    // A rides with T and K for ten seconds, then leaves: Mesh drops it from
    // the roster, and its seat here stays.
    Pair pair;
    const uint32_t A = 0x7E000003;
    Position fromA;
    fromA.slot = 9;
    fromA.slotMap[0] = slotTag(Pair::T);
    auto a = [&](uint32_t atMs) {
      pair.riders[2].used = true;
      pair.riders[2].id = A;
      pair.riders[2].pos = fromA;
      pair.riders[2].atMs = atMs;
      pair.relay.heardMap(A, fromA, atMs);
    };
    for (uint32_t s = 0; s < 10; s++) {
      pair.t(base + s * 1000 + 2);
      a(base + s * 1000 + 306);
      pair.k(base + s * 1000 + 752, true);
    }
    pair.riders[2] = Rider();
    for (uint32_t s = 0; s < 10; s++) {
      pair.t(later + s * 1000 + 2);
      pair.k(later + s * 1000 + 752, true);
    }
    ASSERT_VERDICT(FastRelayVerdict::SKIP, pair.judge(later + 9800));
    // A comes back: its maps count again.
    for (uint32_t s = 10; s < 20; s++) {
      pair.t(later + s * 1000 + 2);
      a(later + s * 1000 + 306);
      pair.k(later + s * 1000 + 752, true);
    }
    ASSERT_VERDICT(FastRelayVerdict::SKIP, pair.judge(later + 19800));
  }
  {
    // 27 cars fill every seat, and a 28th takes one from a car still on the
    // ride. Then only T and K ride on.
    Pair pair;
    Position filler;
    for (uint32_t i = 0; i < FAST_RELAY_CARS - 2; i++) {
      filler.slot = (uint8_t)(4 + i);
      pair.relay.heardMap(0x7F000000 + i, filler, base + i);
    }
    pair.t(base + 100);
    pair.k(base + 752, true);
    filler.slot = 30;
    pair.relay.heardMap(0x7F100000, filler, base + 800);
    ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE, pair.judge(base + 900));
    for (uint32_t s = 0; s < 10; s++) {
      pair.t(later + s * 1000 + 2);
      pair.k(later + s * 1000 + 752, true);
    }
    ASSERT_VERDICT(FastRelayVerdict::SKIP, pair.judge(later + 9800));
  }
}

void test_a_stamp_later_than_the_verdict_reads_as_now() {
  // The module may judge on an older time than it noted a car at: rx.rxMs
  // against the pass time, or a fresh millis() on another path.
  const uint32_t base = 100000;
  Pair deafK;
  for (uint32_t s = 0; s < 10; s++) {
    deafK.t(base + s * 1000 + 2);
    deafK.k(base + s * 1000 + 752, false);
  }
  const uint32_t at = base + 9800;
  deafK.riders[1].atMs = at + 1;
  // On the ride, not a car gone 49 days.
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, deafK.judge(at));
  // Its latest map stamped after the verdict reads as stale: the forward goes.
  ASSERT_VERDICT(FastRelayVerdict::STALE, deafK.judge(base + 9700));

  // X, on no roster, never hears T. Its latest map is stamped on its receive
  // time, a moment after the pass the verdict is judged on began: X is still
  // a car on the ride, not one gone 49 days.
  Pair offRoster;
  const uint32_t X = 0x7E200000;
  Position fromX;
  fromX.slot = 9;
  for (uint32_t s = 0; s < 10; s++) {
    offRoster.t(base + s * 1000 + 2);
    offRoster.k(base + s * 1000 + 752, true);
    offRoster.relay.heardMap(X, fromX, base + s * 1000 + 800);
  }
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, offRoster.judge(base + 9900));
  ASSERT_VERDICT(FastRelayVerdict::STALE, offRoster.judge(base + 9795));

  // A full roster, a stamp later than now among it: nobody is stale, so a car
  // may have been turned away.
  Pair hearsT;
  for (uint32_t s = 0; s < 10; s++) {
    hearsT.t(base + s * 1000 + 2);
    hearsT.k(base + s * 1000 + 752, true);
  }
  ASSERT_VERDICT(FastRelayVerdict::SKIP, hearsT.judge(at));
  // A car on the roster stamped after the verdict, with no map here yet.
  Rider& late = hearsT.riders[2];
  late.used = true;
  late.id = 0x7E100000;
  late.atMs = at + 1;
  ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE, hearsT.judge(at));
  for (size_t i = 2; i < MAX_RIDERS; i++) {
    Rider& r = hearsT.riders[i];
    r.used = true;
    r.id = 0x7E000000 + (uint32_t)i;
    // Off the ride, not yet stale.
    r.atMs = at - LEASE_MS - 1000;
  }
  hearsT.riders[1].atMs = at + 1;
  ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE, hearsT.judge(at));
}

// A roster of 28: T and K, whose maps the relay has, 25 cars off the ride but
// not stale, and one more seat stamped at `oddAtMs`.
static void fullRoster(Mesh& mesh, FastRelay& relay, uint32_t oddAtMs, uint32_t nowMs) {
  mesh.reset();
  Position fromT;
  fromT.slot = 0;
  Position fromK;
  fromK.slot = 3;
  fromK.slotMap[0] = slotTag(Pair::T);
  for (uint32_t s = 10; s >= 1; s--) {
    const uint32_t at = nowMs - s * SCHEDULE_MS;
    mesh.note(Pair::T, fromT, HEARD_FAST, -50, 0, at, 1);
    relay.heardMap(Pair::T, fromT, at);
    mesh.note(Pair::K, fromK, HEARD_FAST, -50, 0, at + 750, 1);
    relay.heardMap(Pair::K, fromK, at + 750);
  }
  Position other;
  for (uint32_t i = 0; i < MAX_RIDERS - 3; i++)
    mesh.note(0x7F000000 + i, other, HEARD_FAST, -50, 0, nowMs - LEASE_MS - SCHEDULE_MS, 1);
  mesh.note(0x7F100000, other, HEARD_FAST, -50, 0, oddAtMs, 1);
}

void test_the_roster_and_the_relay_agree_on_which_seat_is_stale() {
  Position newcomer;
  {
    // A radio on for 49.7 days passes the millis() wrap. The seat heard just
    // before it is stale by age, though its stamp is numerically ahead of now.
    const uint32_t beforeWrap = 0xFFFFF000u;
    const uint32_t now = beforeWrap + RIDER_STALE_MS + 1;
    TEST_ASSERT_TRUE(now < beforeWrap);
    Mesh mesh;
    FastRelay relay;
    fullRoster(mesh, relay, beforeWrap, now);
    TEST_ASSERT_EQUAL_UINT32(MAX_RIDERS, mesh.count());
    // The relay does not count the roster full, so the roster must not be.
    ASSERT_VERDICT(FastRelayVerdict::SKIP,
                   relay.judge(SELF, Pair::T, false, 0, mesh.riders(), MAX_RIDERS, now));
    TEST_ASSERT_NOT_NULL(mesh.note(0x7F200000, newcomer, HEARD_FAST, -50, 0, now, 1));
  }
  {
    // A seat stamped later than now is a car just heard, on both sides.
    const uint32_t now = 1000000;
    Mesh mesh;
    FastRelay relay;
    fullRoster(mesh, relay, now + 5, now);
    ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE,
                   relay.judge(SELF, Pair::T, false, 0, mesh.riders(), MAX_RIDERS, now));
    TEST_ASSERT_NULL(mesh.note(0x7F200000, newcomer, HEARD_FAST, -50, 0, now, 1));
  }
}

// ---- Rides built second by second -------------------------------------------

void test_an_extras_map_moves_nothing() {
  Ride ride;
  ride.add(0xB0000001, 0);
  ride.add(0xB0000002, 3);
  ride.seconds(10);
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));
  // Four maps a second over one window would count one hearing four times.
  Position extra = ride.rider(1)->pos;
  extra.extra = true;
  extra.slotMap[0] = 0;
  for (int i = 0; i < 8; i++) ride.relay.heardMap(ride.cars[1].id, extra, ride.nowMs - 500 + i * 60);
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));
}

void test_an_origin_without_a_lease_or_off_the_ride_goes() {
  Ride ride;
  carPark(ride);
  const size_t unleased = ride.add(0xA0000006, SLOT_NONE);
  ride.seconds(10);
  // An unleased car is in nobody's map.
  ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE, ride.judge(unleased));
  // Its own maps count for the others: it hears them all.
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));
  // Not on the roster at all.
  ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE,
                 ride.relay.judge(SELF, 0xDEAD0001, false, 0, ride.riders(), MAX_RIDERS, ride.nowMs));

  // Quiet for a whole lease: off the ride, though still on the roster.
  ride.quiet[4] = true;
  ride.nowMs += LEASE_MS;
  ride.second();
  ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE, ride.judge(4));
}

void test_two_cars_on_one_slot_go_whichever_the_roster_holds_first() {
  // The maps show whichever car got through last, here the one added last. A
  // rule that took the last car on the slot for its holder would pass the maps
  // for that one; so each order, and both cars, are asked.
  for (int clasherFirst = 0; clasherFirst < 2; clasherFirst++) {
    Ride clash;
    size_t clasher = 0;
    if (clasherFirst) clasher = clash.add(0xA0000007, 0);
    carPark(clash);
    if (!clasherFirst) clasher = clash.add(0xA0000007, 0);
    clash.seconds(10);
    const size_t origin = clasherFirst ? 1 : 0;
    TEST_ASSERT_EQUAL_UINT32(0, clash.cars[origin].slot);
    TEST_ASSERT_TRUE(clash.rider(clasher) != nullptr);
    ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE, clash.judge(origin));
    ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE, clash.judge(clasher));
  }
}

void test_a_full_roster_may_hide_a_car_and_never_skips() {
  Ride ride;
  for (size_t i = 0; i < MAX_RIDERS; i++) ride.add(0xC0000001 + (uint32_t)i, (uint8_t)i);
  ride.seconds(10);
  ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE, ride.judge(0));
  // One car goes quiet. Off the ride after a lease, but its seat is not
  // stale, so Mesh::note would still turn a newcomer away.
  ride.quiet[MAX_RIDERS - 1] = true;
  ride.seconds((int)(LEASE_MS / SCHEDULE_MS) + 1);
  ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE, ride.judge(0));
  // Stale: a newcomer would take its seat, so nobody can be missing.
  ride.seconds((int)((RIDER_STALE_MS - LEASE_MS) / SCHEDULE_MS) + 1);
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));
}

// T, and 25 listeners that hear it, on slots 0-25.
static void bigRide(Ride& ride) {
  for (uint32_t i = 0; i < 26; i++) ride.add(0xC1000000 + i, (uint8_t)i);
}

void test_a_car_the_roster_turned_away_still_needs_the_forward() {
  Ride ride;
  bigRide(ride);
  // Two more fill the roster by their extras alone, then go quiet: off the
  // ride after a lease, but not stale for two minutes.
  const size_t q1 = ride.add(0xC1000100, 26);
  const size_t q2 = ride.add(0xC1000101, 27);
  ride.onlyExtras[q1] = ride.onlyExtras[q2] = true;
  ride.seconds(10);
  ride.quiet[q1] = ride.quiet[q2] = true;
  ride.seconds((int)(LEASE_MS / SCHEDULE_MS) + 2);
  // X arrives. The roster has no stale seat and turns it away, but X's maps
  // count here, and X never hears T.
  const size_t x = ride.add(0xC1000200, 28);
  ride.deaf[x][0] = true;
  const uint32_t quietSince = ride.rider(q1)->atMs;
  while ((uint32_t)(ride.nowMs + SCHEDULE_MS - quietSince) <= RIDER_STALE_MS) ride.second();
  TEST_ASSERT_NULL(ride.rider(x));
  ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE, ride.judge(0));
  // The quiet seats go stale: the roster no longer looks full, and X is still
  // on none until its next position gets through.
  ride.nowMs = quietSince + RIDER_STALE_MS + 1;
  TEST_ASSERT_NULL(ride.rider(x));
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(0));
  // Once X is on the roster it counts as any car.
  ride.second();
  TEST_ASSERT_NOT_NULL(ride.rider(x));
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(0));
  // And a car on no roster that does hear T does not stop the skip.
  Ride hears;
  bigRide(hears);
  const size_t h1 = hears.add(0xC1000100, 26);
  const size_t h2 = hears.add(0xC1000101, 27);
  hears.onlyExtras[h1] = hears.onlyExtras[h2] = true;
  hears.seconds(10);
  hears.quiet[h1] = hears.quiet[h2] = true;
  hears.seconds((int)(LEASE_MS / SCHEDULE_MS) + 2);
  const size_t y = hears.add(0xC1000200, 28);
  const uint32_t since = hears.rider(h1)->atMs;
  while ((uint32_t)(hears.nowMs + SCHEDULE_MS - since) <= RIDER_STALE_MS) hears.second();
  hears.nowMs = since + RIDER_STALE_MS + 1;
  TEST_ASSERT_NULL(hears.rider(y));
  ASSERT_VERDICT(FastRelayVerdict::SKIP, hears.judge(0));
}

void test_more_cars_than_seats_here_never_skips() {
  // T and 25 listeners send maps; two more are on the roster by their extras
  // alone, then go quiet. X, deaf to T, arrives while those two are not yet
  // stale: the roster turns it away, and only its seat here counts it.
  Ride ride;
  bigRide(ride);
  const size_t q1 = ride.add(0xC1000100, 26);
  const size_t q2 = ride.add(0xC1000101, 27);
  ride.onlyExtras[q1] = ride.onlyExtras[q2] = true;
  ride.seconds(10);
  ride.quiet[q1] = ride.quiet[q2] = true;
  const uint32_t quietSince = ride.rider(q1)->atMs;
  while (ride.nowMs - quietSince < RIDER_STALE_MS - 15 * SCHEDULE_MS) ride.second();
  const size_t x = ride.add(0xC1000200, 28);
  ride.deaf[x][0] = true;
  ride.seconds(3);
  TEST_ASSERT_NULL(ride.rider(x));
  ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE, ride.judge(0));
  // X's frames stop reaching us. The quiet seats go stale, and Y takes one on
  // the roster and X's here, the oldest, while every seat here is a car on
  // the ride.
  ride.unheard[x] = true;
  while (ride.nowMs - quietSince <= RIDER_STALE_MS) ride.second();
  const size_t y = ride.add(0xC1000300, 29);
  ride.seconds(FAST_RELAY_MAPS + 1);
  TEST_ASSERT_NOT_NULL(ride.rider(y));
  TEST_ASSERT_NULL(ride.rider(x));
  // X, heard well within a lease, is on neither list.
  ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE, ride.judge(0));
  // A lease after it lost its seat, X is off the ride however it is counted.
  ride.seconds((int)(LEASE_MS / SCHEDULE_MS));
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));
}

void test_a_seat_taken_over_carries_nothing_of_the_car_before() {
  // FAST_RELAY_CARS cars that all hear each other. Then X's frames stop
  // reaching us, though the others still hear it, until the roster has it
  // stale. Y, the next car, takes X's seat here: everyone heard X, nobody
  // hears Y, and Y hears everyone but has sent one map.
  Ride ride;
  for (size_t i = 0; i < FAST_RELAY_CARS; i++) ride.add(0xC2000001 + (uint32_t)i, (uint8_t)i);
  ride.seconds(10);
  const size_t x = FAST_RELAY_CARS - 1;
  ride.unheard[x] = true;
  ride.seconds((int)(RIDER_STALE_MS / SCHEDULE_MS) + 2);
  const size_t y = ride.add(0xC2000100, 30);
  for (size_t a = 0; a < ride.cars.size(); a++) ride.deaf[a][y] = true;
  ride.nowMs = ride.beaconAt(y, ride.nowMs - ride.nowMs % SCHEDULE_MS + SCHEDULE_MS);
  ride.beacon(y, ride.nowMs);
  TEST_ASSERT_NOT_NULL(ride.rider(y));
  // Nobody has shown Y yet.
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(y));
  // Y has shown car 0 once, not five times.
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(0));
  ride.seconds(FAST_RELAY_MAPS);
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(y));
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));
}

void test_stale_or_missing_evidence_goes() {
  Ride ride;
  carPark(ride);
  ride.seconds(10);
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));

  // Car 2's maps stop reaching us; its extras keep it on the ride.
  ride.onlyExtras[2] = true;
  ride.seconds(2);
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));
  ride.second();
  ASSERT_VERDICT(FastRelayVerdict::STALE, ride.judge(0));
  // A car that lacks it outranks a stale one.
  ride.deaf[3][0] = true;
  ride.seconds(3);
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(0));
  ride.deaf[3][0] = false;
  ride.seconds(FAST_RELAY_MAPS);
  ASSERT_VERDICT(FastRelayVerdict::STALE, ride.judge(0));

  // Back after a gap of FAST_RELAY_GAP_MS: what it heard before counts for
  // nothing, and its record starts again.
  ride.onlyExtras[2] = false;
  ride.second();
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(0));
  ride.seconds(FAST_RELAY_SHOWN_TWO);
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));

  // A car on the ride that has never sent us a map.
  const size_t newcomer = ride.add(0xA0000006, 13);
  ride.onlyExtras[newcomer] = true;
  ride.seconds(10);
  ASSERT_VERDICT(FastRelayVerdict::NO_EVIDENCE, ride.judge(0));
}

void test_a_tag_collision_cannot_skip() {
  Ride ride;
  const size_t origin = ride.add(0xD0000001, 0);
  // A car with the origin's tag, as one in 255 are.
  uint32_t twin = 0xD0000002;
  while (slotTag(twin) != slotTag(ride.cars[origin].id)) twin++;
  const size_t twinCar = ride.add(twin, 8);
  const size_t listener = ride.add(0xD0100001, 3);
  ride.add(0xD0100002, 1);
  // The listener hears the twin in its slot, and its map carries the origin's
  // tag in every slot nobody holds, but never in the origin's slot.
  ride.deaf[listener][origin] = true;
  ride.junkTag[listener] = slotTag(ride.cars[origin].id);
  ride.seconds(20);
  TEST_ASSERT_EQUAL_HEX8(slotTag(ride.cars[origin].id), ride.rider(listener)->pos.slotMap[ride.cars[twinCar].slot]);
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(origin));
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(twinCar));
}

void test_a_relayed_copy_skips_only_for_cars_that_hear_the_origin_or_its_relayer() {
  Ride ride;
  const size_t origin = ride.add(0xE0000001, 0);
  const size_t relayer = ride.add(0xE0000002, 4);
  const size_t other = ride.add(0xE0000003, 1);
  const size_t far = ride.add(0xE0000004, 8);
  // The far car hears the relayer and nothing else. The relayer does not hear
  // the origin either: it had the frame from somewhere, and has it.
  ride.deaf[far][origin] = true;
  ride.deaf[far][other] = true;
  ride.deaf[relayer][origin] = true;
  ride.seconds(20);
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(origin));
  bool unplaced = false;
  // Everyone hears the origin or the relayer.
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(origin, true, ride.tagOf(relayer), &unplaced));
  TEST_ASSERT_FALSE(unplaced);
  // Relayed by a car the far one does not hear.
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(origin, true, ride.tagOf(other), &unplaced));
  TEST_ASSERT_FALSE(unplaced);
  // The relayer's own record counts from maps like anyone's.
  ride.deaf[far][relayer] = true;
  ride.seconds(FAST_RELAY_MAPS);
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(origin, true, ride.tagOf(relayer), &unplaced));
}

void test_a_relayer_is_placed_only_by_a_tag_one_car_on_the_ride_has() {
  Ride ride;
  const size_t origin = ride.add(0xE1000001, 0);
  const size_t relayer = ride.add(0xE1000002, 9);
  const size_t far = ride.add(0xE1000003, 8);
  ride.deaf[far][origin] = true;
  ride.seconds(20);
  bool unplaced = false;
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(origin, true, ride.tagOf(relayer), &unplaced));
  TEST_ASSERT_FALSE(unplaced);
  // R2 shares the relayer's slot, unheard by us: its beacons lose to R's here.
  // Its forward reaches us all the same, and names R2, not R's slot.
  uint32_t r2 = 0xE1000100;
  while (slotTag(r2) == ride.tagOf(relayer) || slotTag(r2) == 0) r2++;
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(origin, true, slotTag(r2), &unplaced));
  TEST_ASSERT_TRUE(unplaced);
  // A frame that names nobody.
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(origin, true, 0, &unplaced));
  TEST_ASSERT_TRUE(unplaced);
  // Two cars on the ride with the relayer's tag: either could have sent it.
  uint32_t twin = 0xE1000200;
  while (slotTag(twin) != ride.tagOf(relayer)) twin++;
  const size_t twinCar = ride.add(twin, 17);
  ride.seconds(FAST_RELAY_MAPS);
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(origin, true, ride.tagOf(relayer), &unplaced));
  TEST_ASSERT_TRUE(unplaced);
  // A lease after the twin left, the tag is the relayer's alone again.
  ride.quiet[twinCar] = true;
  ride.seconds((int)(LEASE_MS / SCHEDULE_MS) + 1);
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(origin, true, ride.tagOf(relayer), &unplaced));
  TEST_ASSERT_FALSE(unplaced);
  // The same when the twin is on no roster, only sending us maps.
  Ride hidden;
  const size_t o = hidden.add(0xE1000001, 0);
  const size_t r = hidden.add(0xE1000002, 9);
  const size_t f = hidden.add(0xE1000003, 8);
  hidden.deaf[f][o] = true;
  uint32_t w = 0xE1000300;
  while (slotTag(w) != hidden.tagOf(r)) w++;
  hidden.offRoster[hidden.add(w, 17)] = true;
  hidden.seconds(20);
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, hidden.judge(o, true, hidden.tagOf(r), &unplaced));
  TEST_ASSERT_TRUE(unplaced);
  // And when the twin is on the roster ahead of the relayer, heard by its
  // extras alone, so it has no seat here.
  Ride extrasTwin;
  const size_t o3 = extrasTwin.add(0xE1000001, 0);
  const size_t r3 = extrasTwin.add(0xE1000002, 9);
  const size_t f3 = extrasTwin.add(0xE1000003, 8);
  extrasTwin.deaf[f3][o3] = true;
  const size_t t3 = extrasTwin.add(w, 5);
  extrasTwin.onlyExtras[t3] = true;
  extrasTwin.seconds(20);
  TEST_ASSERT_TRUE(extrasTwin.rider(t3) < extrasTwin.rider(r3));
  extrasTwin.judge(o3, true, extrasTwin.tagOf(r3), &unplaced);
  TEST_ASSERT_TRUE(unplaced);

  // A relayer whose new lease the roster has from an extra, before any map
  // here has come with it: its column was read at the old slot.
  Ride moved;
  const size_t o4 = moved.add(0xE1000001, 0);
  const size_t r4 = moved.add(0xE1000002, 9);
  const size_t f4 = moved.add(0xE1000003, 8);
  moved.deaf[f4][o4] = true;
  moved.seconds(20);
  ASSERT_VERDICT(FastRelayVerdict::SKIP, moved.judge(o4, true, moved.tagOf(r4), &unplaced));
  moved.cars[r4].slot = 13;
  moved.onlyExtras[r4] = true;
  moved.second();
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, moved.judge(o4, true, moved.tagOf(r4), &unplaced));
  TEST_ASSERT_TRUE(unplaced);
}

void test_a_relayed_copy_whose_relayer_cannot_be_placed_is_judged_on_the_origin() {
  // Every other car, the unknown relayer included, hears the origin: our copy
  // reaches nobody who lacks it.
  Ride ride;
  carPark(ride);
  ride.seconds(10);
  bool unplaced = false;
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0, true, 0, &unplaced));
  TEST_ASSERT_TRUE(unplaced);
  // And when one does not, as for a copy straight from the origin.
  ride.deaf[3][0] = true;
  ride.seconds(3);
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(0, true, 0, &unplaced));
  TEST_ASSERT_TRUE(unplaced);
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(0, false, 0, &unplaced));
  TEST_ASSERT_FALSE(unplaced);
}

void test_a_new_lease_starts_the_record_again() {
  Ride ride;
  carPark(ride);
  ride.seconds(10);
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));
  // The origin moves slot: the maps showed it where it was. Car 1's maps stop
  // reaching us as it does, so every record read after the move was built
  // before it and has to have been started again.
  ride.cars[0].slot = 12;
  ride.onlyExtras[1] = true;
  ride.second();
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(0));
  ride.onlyExtras[1] = false;
  ride.seconds(FAST_RELAY_MAPS);
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));
  // A listener moves: its maps now land at another phase.
  ride.cars[2].slot = 17;
  ride.second();
  ASSERT_VERDICT(FastRelayVerdict::NEEDED, ride.judge(0));
  ride.seconds(FAST_RELAY_MAPS);
  ASSERT_VERDICT(FastRelayVerdict::SKIP, ride.judge(0));
}

// Send offsets within a slot and clock skew, as either margin budgets them.
static const int SKEW_MS = 60;
// Airtime and the drain pass, as the older-beacon margin budgets them.
static const uint32_t DRAIN_LAG_MS = FRAME_AIRTIME_MS + 40;

void test_a_map_read_as_covering_one_beacon_never_covers_two() {
  // Every pair of slots, with the listener's map going out up to 60 ms either
  // side of where the slots alone put it against the origin's beacon (each
  // leaves in the first 17 ms of its slot, and the clocks differ by a few ms a
  // sync hop), and the listener stamping the origin's beacon up to
  // DRAIN_LAG_MS after it went out. A real Schedule hears only the older of
  // the origin's last two beacons before the map; if the map still shows the
  // origin, it covered two, and the rule must say so.
  uint32_t pairs = 0;
  uint32_t readAsOne = 0;
  for (uint8_t originSlot = 0; originSlot < MAX_SLOTS; originSlot++) {
    for (uint8_t listenerSlot = 0; listenerSlot < MAX_SLOTS; listenerSlot++) {
      if (listenerSlot == originSlot) continue;
      pairs++;
      const bool two = mapCoversTwoBeacons(listenerSlot, originSlot);
      if (!two) readAsOne++;
      Schedule listener;
      Position fromOrigin;
      fromOrigin.slot = originSlot;
      for (int skew = -SKEW_MS; skew <= SKEW_MS; skew++) {
        const uint32_t base = 100000;
        const uint32_t mapAt = base + 2 * SCHEDULE_MS + slotStartMs(listenerSlot) + 60 + skew;
        // The origin's beacons once a second from `base`: the latest by the
        // time the map goes out, and the one before it.
        uint32_t latest = base + slotStartMs(originSlot) + 60;
        while (latest + SCHEDULE_MS <= mapAt) latest += SCHEDULE_MS;
        const uint32_t older = latest - SCHEDULE_MS;
        for (uint32_t lag = 0; lag <= DRAIN_LAG_MS; lag++) {
          // Each call stamps the slot afresh; the map reads only that stamp.
          listener.heardBeacon(0xF0000001, fromOrigin, older + lag);
          uint8_t map[SLOT_MAP_LEN];
          listener.fillSlotMap(mapAt, map);
          if (map[originSlot] == slotTag(0xF0000001) && !two) {
            char line[112];
            snprintf(line, sizeof(line), "listener %u origin %u skew %d lag %u: two beacons read as one",
                     (unsigned)listenerSlot, (unsigned)originSlot, skew, (unsigned)lag);
            TEST_FAIL_MESSAGE(line);
          }
        }
      }
    }
  }
  // The margin leaves a real share of pairs on the looser k.
  char line[64];
  snprintf(line, sizeof(line), "%u of %u slot pairs read as one beacon a map", (unsigned)readAsOne, (unsigned)pairs);
  TEST_MESSAGE(line);
  TEST_ASSERT_TRUE(readAsOne * 4 >= pairs);
  TEST_ASSERT_TRUE(mapCoversTwoBeacons(2, 0));          // exactly half a second
  TEST_ASSERT_TRUE(mapCoversTwoBeacons(0, 4));          // the origin one slot after
  TEST_ASSERT_FALSE(mapCoversTwoBeacons(0, 1));         // the origin 250 ms after
  TEST_ASSERT_TRUE(mapCoversTwoBeacons(14, 0));         // 581 ms: the older beacon may still be stamped in the window
  TEST_ASSERT_FALSE(mapCoversTwoBeacons(18, 0));        // 608 ms
  TEST_ASSERT_TRUE(mapCoversTwoBeacons(SLOT_NONE, 1));  // an unleased listener, anywhere
}

// ---- A listening car with loss, maps from its own Schedule ------------------

// The origin T, one listening car K, and us. T's beacons reach K with lossTk
// percent loss and K's maps reach us with lossKus; T's frames reach us. Each
// second we judge T's position as it arrives, and a voice frame at a random
// moment.
struct LossyRun {
  uint32_t judged = 0;
  uint32_t skipped = 0;
  // Position forwards skipped when K had missed that very position.
  uint32_t positionsJudged = 0;
  uint32_t skippedWhileMissed = 0;
  double skipShare() const { return judged ? (double)skipped / judged : 0; }
};

struct LossyLink {
  static const uint32_t T = 0x7A000001;
  static const uint32_t K = 0x7B000002;
  uint8_t slotT, slotK;
  Schedule listener;  // K's
  FastRelay relay;
  Rider riders[MAX_RIDERS];
  Position fromT, fromK;
  uint32_t second = 0;

  LossyLink(uint8_t tSlot, uint8_t kSlot) : slotT(tSlot), slotK(kSlot) {
    fromT.slot = slotT;
    fromK.slot = slotK;
    riders[0].id = T;
    riders[1].id = K;
  }

  bool skips(uint32_t atMs) const {
    return relay.judge(SELF, T, false, 0, riders, MAX_RIDERS, atMs) == FastRelayVerdict::SKIP;
  }

  // One second. Beacons leave a couple of ms into the slot, as the module's
  // 5 ms pass puts them.
  void run(int lossTk, int lossKus, LossyRun* out) {
    const uint32_t base = 1000000 + second++ * SCHEDULE_MS;
    const uint32_t atT = base + slotStartMs(slotT) + 2;
    const uint32_t atK = base + slotStartMs(slotK) + 2;
    const uint32_t voiceAt = base + rand32() % SCHEDULE_MS;
    const uint32_t times[3] = {atT, atK, voiceAt};
    int order[3] = {0, 1, 2};
    std::sort(order, order + 3, [&](int a, int b) { return times[a] < times[b]; });
    for (int what : order) {
      if (what == 0) {
        const bool kGotIt = !chance(lossTk);
        if (kGotIt) listener.heardBeacon(T, fromT, atT);
        riders[0].used = true;
        riders[0].pos = fromT;
        riders[0].atMs = atT;
        relay.heardMap(T, fromT, atT);
        if (out) {
          const bool skip = skips(atT);
          out->judged++;
          out->positionsJudged++;
          out->skipped += skip;
          out->skippedWhileMissed += skip && !kGotIt;
        }
      } else if (what == 1) {
        listener.fillSlotMap(atK, fromK.slotMap);
        if (chance(lossKus)) continue;
        riders[1].used = true;
        riders[1].pos = fromK;
        riders[1].atMs = atK;
        relay.heardMap(K, fromK, atK);
      } else if (out) {
        const bool skip = skips(voiceAt);
        out->judged++;
        out->skipped += skip;
      }
    }
  }
};

static LossyRun lossyRun(uint8_t slotT, uint8_t slotK, int lossTk, int lossKus, uint32_t seconds, uint32_t seed) {
  rng = seed;
  LossyLink link(slotT, slotK);
  for (int i = 0; i < 30; i++) link.run(lossTk, lossKus, nullptr);
  LossyRun out;
  for (uint32_t i = 0; i < seconds; i++) link.run(lossTk, lossKus, &out);
  return out;
}

// One beacon a map: K's slot opens 750 ms after T's. Two: 250 ms after.
static const uint8_t T_SLOT = 0;
static const uint8_t K_ONE = 3;
static const uint8_t K_TWO = 1;
static const uint32_t RUN_S = 20000;

void test_a_flaky_neighbour_is_judged_by_how_often_its_maps_show_the_origin() {
  TEST_ASSERT_FALSE(mapCoversTwoBeacons(K_ONE, T_SLOT));
  TEST_ASSERT_TRUE(mapCoversTwoBeacons(K_TWO, T_SLOT));
  const int losses[] = {20, 40, 50, 75, 100};
  double share[2][5];
  double missedShare[2][5];
  for (int phase = 0; phase < 2; phase++) {
    for (int i = 0; i < 5; i++) {
      const LossyRun r = lossyRun(T_SLOT, phase == 0 ? K_ONE : K_TWO, losses[i], 40, RUN_S, 1000 + i);
      share[phase][i] = r.skipShare();
      missedShare[phase][i] = (double)r.skippedWhileMissed / r.positionsJudged;
      char line[160];
      snprintf(line, sizeof(line),
               "%s beacon a map, T to K %d %% loss, K's maps 40 %%: judged steady %.1f %%, "
               "a position skipped that K missed %.1f %%",
               phase == 0 ? "one" : "two", losses[i], 100 * share[phase][i], 100 * missedShare[phase][i]);
      TEST_MESSAGE(line);
    }
  }
  for (int phase = 0; phase < 2; phase++) {
    // A good link most of the time, a working one at bench loss often.
    TEST_ASSERT_TRUE(share[phase][0] > 0.80);
    TEST_ASSERT_TRUE(share[phase][1] > 0.40 && share[phase][1] < 0.70);
    TEST_ASSERT_TRUE(share[phase][2] > 0.20 && share[phase][2] < 0.50);
    // The false skips: a car that hears one beacon in four is almost never
    // taken for one that hears the origin, and a deaf one never.
    TEST_ASSERT_TRUE(share[phase][3] < 0.05);
    TEST_ASSERT_TRUE(missedShare[phase][3] < 0.04);
    TEST_ASSERT_TRUE(share[phase][4] == 0);
  }
}

void test_a_link_that_dies_stops_counting_within_seconds() {
  // A working link at bench loss goes dead at the end of a second whose record
  // read steady. Seconds until the end of a second reads otherwise, over many
  // deaths.
  for (int phase = 0; phase < 2; phase++) {
    std::vector<uint32_t> waits;
    rng = 77 + phase;
    for (int trial = 0; trial < 400; trial++) {
      LossyLink link(T_SLOT, phase == 0 ? K_ONE : K_TWO);
      for (int i = 0; i < 40; i++) link.run(40, 40, nullptr);
      auto endOfSecond = [&]() { return 1000000 + link.second * SCHEDULE_MS - 1; };
      int guard = 0;
      while (!link.skips(endOfSecond()) && guard++ < 200) link.run(40, 40, nullptr);
      if (guard >= 200) continue;
      uint32_t wait = 0;
      do {
        link.run(100, 40, nullptr);
        wait++;
      } while (link.skips(endOfSecond()) && wait < 30);
      waits.push_back(wait);
    }
    std::sort(waits.begin(), waits.end());
    const uint32_t p95 = waits[waits.size() * 95 / 100];
    uint64_t sum = 0;
    for (uint32_t w : waits) sum += w;
    char line[128];
    snprintf(line, sizeof(line),
             "%s beacon a map: a dead link stops counting after %.1f s on average, %u s at p95, %u s at most",
             phase == 0 ? "one" : "two", (double)sum / waits.size(), (unsigned)p95, (unsigned)waits.back());
    TEST_MESSAGE(line);
    TEST_ASSERT_TRUE(waits.size() > 300);
    TEST_ASSERT_TRUE(p95 <= 4);
    // By time, not by maps received: a window's worth of maps that still show
    // T, then FAST_RELAY_FRESH_MS.
    TEST_ASSERT_TRUE(waits.back() <= (HEARD_WINDOW_MS + FAST_RELAY_FRESH_MS) / SCHEDULE_MS + 1);
  }
}

// ---- Counters and size -------------------------------------------------------

void test_the_counters_tally_each_verdict() {
  FastRelaySkips skips;
  skips.note(FastRelayVerdict::SKIP);
  skips.note(FastRelayVerdict::SKIP, true);
  skips.note(FastRelayVerdict::NO_EVIDENCE);
  skips.note(FastRelayVerdict::STALE);
  skips.note(FastRelayVerdict::NEEDED);
  skips.note(FastRelayVerdict::NEEDED, true);
  skips.note(FastRelayVerdict::NEEDED);
  TEST_ASSERT_EQUAL_UINT32(2, skips.skipped);
  TEST_ASSERT_EQUAL_UINT32(1, skips.noEvidence);
  TEST_ASSERT_EQUAL_UINT32(1, skips.stale);
  TEST_ASSERT_EQUAL_UINT32(3, skips.needed);
  // On top of their verdicts.
  TEST_ASSERT_EQUAL_UINT32(2, skips.noRelayer);
}

void test_the_record_fits_a_v3() {
  char line[64];
  snprintf(line, sizeof(line), "sizeof(FastRelay) = %u bytes", (unsigned)sizeof(FastRelay));
  TEST_MESSAGE(line);
  TEST_ASSERT_TRUE(sizeof(FastRelay) <= 1024);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_a_car_park_where_everyone_hears_the_origin_skips);
  RUN_TEST(test_one_car_that_never_shows_the_origin_needs_the_forward);
  RUN_TEST(test_a_two_car_ride_skips_every_forward);
  RUN_TEST(test_steady_is_k_of_the_last_eight_maps_and_one_of_the_last_two_beacons);
  RUN_TEST(test_the_map_before_the_latest_counts_only_while_recent);
  RUN_TEST(test_maps_faster_than_one_a_second_count_once_a_second);
  RUN_TEST(test_a_map_that_comes_too_soon_refreshes_nothing);
  RUN_TEST(test_a_seat_left_for_24_days_reads_as_gone);
  RUN_TEST(test_a_stamp_later_than_the_verdict_reads_as_now);
  RUN_TEST(test_the_roster_and_the_relay_agree_on_which_seat_is_stale);
  RUN_TEST(test_an_extras_map_moves_nothing);
  RUN_TEST(test_an_origin_without_a_lease_or_off_the_ride_goes);
  RUN_TEST(test_two_cars_on_one_slot_go_whichever_the_roster_holds_first);
  RUN_TEST(test_a_full_roster_may_hide_a_car_and_never_skips);
  RUN_TEST(test_a_car_the_roster_turned_away_still_needs_the_forward);
  RUN_TEST(test_more_cars_than_seats_here_never_skips);
  RUN_TEST(test_a_seat_taken_over_carries_nothing_of_the_car_before);
  RUN_TEST(test_stale_or_missing_evidence_goes);
  RUN_TEST(test_a_tag_collision_cannot_skip);
  RUN_TEST(test_a_relayed_copy_skips_only_for_cars_that_hear_the_origin_or_its_relayer);
  RUN_TEST(test_a_relayer_is_placed_only_by_a_tag_one_car_on_the_ride_has);
  RUN_TEST(test_a_relayed_copy_whose_relayer_cannot_be_placed_is_judged_on_the_origin);
  RUN_TEST(test_a_new_lease_starts_the_record_again);
  RUN_TEST(test_a_map_read_as_covering_one_beacon_never_covers_two);
  RUN_TEST(test_a_flaky_neighbour_is_judged_by_how_often_its_maps_show_the_origin);
  RUN_TEST(test_a_link_that_dies_stops_counting_within_seconds);
  RUN_TEST(test_the_counters_tally_each_verdict);
  RUN_TEST(test_the_record_fits_a_v3);
  return UNITY_END();
}
