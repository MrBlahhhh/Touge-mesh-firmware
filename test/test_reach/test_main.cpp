// Host tests for end-to-end delivery summaries (SCALE-PLAN 5e) and relays
// chosen on their evidence (5f).
//
// The convoy tests fly a small Meshtastic flood on the host. Cars sit on a
// line and hear the cars within one step. A broadcast is relayed by every car
// that hears it with hops left, after the delay Meshtastic draws, and a car
// drops its queued relay on hearing another car's copy first (managed flooding;
// channel sensing makes it wait out a transmission already under way). A
// preferred relay draws from the ROUTER's early window (core-patches/0012). No
// collisions and no loss: what is tested is who carries whom, not the RF.

#include <unity.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "reach.h"
#include "relaypref.h"

using namespace touge;

void setUp() {}
void tearDown() {}

// ---- The summary on the wire -------------------------------------------------

static ReachEntry entry(uint32_t origin, uint16_t seq, uint32_t ageMs, uint8_t sinceS, uint8_t hops, uint8_t relay) {
  ReachEntry e;
  e.origin = origin;
  e.seq = seq;
  e.ageQ = reachAgeQ(ageMs);
  e.sinceS = sinceS;
  e.hops = hops;
  e.relay = relay;
  return e;
}

void test_a_summary_round_trips() {
  const ReachEntry sent[3] = {entry(0xA000A1B2, 1234, 3400, 12, 1, 0x5E), entry(0xA000C3D4, 65535, 250, 0, 0, 0xD4),
                              entry(0x0000000F, 7, UINT32_MAX, 255, REACH_HOPS_UNKNOWN, 0x33)};
  uint8_t buf[64];
  const size_t len = encodeReach(sent, 3, REACH_MORE, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_UINT32(REACH_HEADER + 3 * REACH_ENTRY, len);
  TEST_ASSERT_EQUAL_HEX8(0xC3, buf[0]);
  size_t entries = 0;
  uint8_t flags = 0;
  TEST_ASSERT_TRUE(decodeReachHeader(buf, len, entries, flags));
  TEST_ASSERT_EQUAL_UINT32(3, entries);
  TEST_ASSERT_EQUAL_HEX8(REACH_MORE, flags);
  for (size_t i = 0; i < 3; i++) {
    ReachEntry got;
    TEST_ASSERT_TRUE(decodeReachEntry(buf, len, i, got));
    TEST_ASSERT_EQUAL_HEX32(sent[i].origin, got.origin);
    TEST_ASSERT_EQUAL_UINT16(sent[i].seq, got.seq);
    TEST_ASSERT_EQUAL_UINT8(sent[i].ageQ, got.ageQ);
    TEST_ASSERT_EQUAL_UINT8(sent[i].sinceS, got.sinceS);
    TEST_ASSERT_EQUAL_UINT8(sent[i].hops, got.hops);
    TEST_ASSERT_EQUAL_HEX8(sent[i].relay, got.relay);
  }
  // The app's LoraReachTest pins the same bytes, so the two ends cannot drift
  // apart unnoticed.
  const uint8_t pinned[] = {0xC3, 0x01, 0x03, 0x01,                                      //
                            0xA0, 0x00, 0xA1, 0xB2, 0x04, 0xD2, 0x0D, 0x0C, 0x01, 0x5E,  //
                            0xA0, 0x00, 0xC3, 0xD4, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0xD4,  //
                            0x00, 0x00, 0x00, 0x0F, 0x00, 0x07, 0xFF, 0xFF, 0x0F, 0x33};
  TEST_ASSERT_EQUAL_UINT32(sizeof(pinned), len);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(pinned, buf, sizeof(pinned));
  ReachEntry past;
  TEST_ASSERT_FALSE(decodeReachEntry(buf, len, 3, past));
  // Cut short, another version, or not a summary at all.
  TEST_ASSERT_FALSE(decodeReachHeader(buf, len - 1, entries, flags));
  buf[1] = 2;
  TEST_ASSERT_FALSE(decodeReachHeader(buf, len, entries, flags));
  const uint8_t json[] = "{\"fl\":{}}";
  TEST_ASSERT_FALSE(decodeReachHeader(json, sizeof(json), entries, flags));
  // A full summary fits one Meshtastic payload.
  ReachEntry many[REACH_PER_SUMMARY];
  uint8_t full[233];
  TEST_ASSERT_TRUE(encodeReach(many, REACH_PER_SUMMARY, 0, full, sizeof(full)) > 0);
}

void test_ages_go_in_quarter_seconds() {
  TEST_ASSERT_EQUAL_UINT8(0, reachAgeQ(0));
  TEST_ASSERT_EQUAL_UINT8(0, reachAgeQ(249));
  TEST_ASSERT_EQUAL_UINT8(13, reachAgeQ(3400));
  TEST_ASSERT_EQUAL_UINT8(253, reachAgeQ(63499));
  TEST_ASSERT_EQUAL_UINT8(REACH_AGE_OVER, reachAgeQ(63500));
  TEST_ASSERT_EQUAL_UINT8(REACH_AGE_OVER, reachAgeQ(3600000));
  TEST_ASSERT_EQUAL_UINT8(REACH_AGE_UNKNOWN, reachAgeQ(UINT32_MAX));
  TEST_ASSERT_EQUAL_UINT32(3250, reachAgeMs(13));
  TEST_ASSERT_EQUAL_UINT32(63500, reachAgeMs(REACH_AGE_OVER));
  TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, reachAgeMs(REACH_AGE_UNKNOWN));
}

void test_an_entry_reads_as_the_app_shows_cars() {
  char out[40];
  TEST_ASSERT_TRUE(formatReachEntry(entry(0xA000A1B2, 1234, 3400, 12, 1, 0x5E), out, sizeof(out)) > 0);
  TEST_ASSERT_EQUAL_STRING("a1b2#1234 3.2s 1h/5e -12s", out);
  TEST_ASSERT_TRUE(formatReachEntry(entry(0xA000C3D4, 9, 250, 0, 0, 0xD4), out, sizeof(out)) > 0);
  TEST_ASSERT_EQUAL_STRING("c3d4#9 0.2s 0h -0s", out);
  TEST_ASSERT_TRUE(formatReachEntry(entry(0x0000000F, 7, UINT32_MAX, 255, REACH_HOPS_UNKNOWN, 0x33), out, sizeof(out)) > 0);
  TEST_ASSERT_EQUAL_STRING("000f#7 ? ?h/33 -255s", out);
}

// ---- One car's table ---------------------------------------------------------

static FixId fixId(uint16_t session, uint32_t seq) {
  FixId f;
  f.session = session;
  f.seq = seq;
  f.fixSec = 1790000000 + seq;
  return f;
}

// The one entry for [origin] in [reach]'s next summary.
static bool summaryEntry(Reach& reach, uint32_t origin, uint32_t nowMs, ReachEntry& out) {
  uint8_t buf[233];
  const size_t len = reach.takeSummary(nowMs, buf, sizeof(buf));
  for (size_t i = 0; decodeReachEntry(buf, len, i, out); i++) {
    if (out.origin == origin) return true;
  }
  return false;
}

void test_the_newest_fix_per_origin_is_kept() {
  Reach reach;
  reach.clear();
  const uint32_t CAR = 0xA000B2B2;
  reach.heard(CAR, fixId(7, 10), 1200, 1, 0x44, 1000);
  // A late copy of an older fix changes nothing.
  reach.heard(CAR, fixId(7, 9), 800, 0, 0xB2, 1500);
  ReachEntry e;
  TEST_ASSERT_TRUE(summaryEntry(reach, CAR, 3000, e));
  TEST_ASSERT_EQUAL_UINT16(10, e.seq);
  TEST_ASSERT_EQUAL_UINT8(1, e.hops);
  TEST_ASSERT_EQUAL_HEX8(0x44, e.relay);
  TEST_ASSERT_EQUAL_UINT8(2, e.sinceS);
  // The same fix again (a parked phone's repeat) is still the car heard.
  reach.heard(CAR, fixId(7, 10), 6000, 0, 0xB2, 5000);
  TEST_ASSERT_TRUE(summaryEntry(reach, CAR, 5000, e));
  TEST_ASSERT_EQUAL_UINT8(0, e.sinceS);
  TEST_ASSERT_EQUAL_UINT8(0, e.hops);
  // A rebooted car starts its sequence again under a new session.
  reach.heard(CAR, fixId(9, 1), 900, 0, 0xB2, 6000);
  TEST_ASSERT_TRUE(summaryEntry(reach, CAR, 6000, e));
  TEST_ASSERT_EQUAL_UINT16(1, e.seq);
  // The low sixteen bits carry on across their wrap.
  reach.heard(CAR, fixId(9, 30000), 900, 0, 0xB2, 6500);
  reach.heard(CAR, fixId(9, 60000), 900, 0, 0xB2, 7000);
  reach.heard(CAR, fixId(9, 65537), 900, 0, 0xB2, 8000);
  TEST_ASSERT_TRUE(summaryEntry(reach, CAR, 8000, e));
  TEST_ASSERT_EQUAL_UINT16(1, e.seq);
  TEST_ASSERT_EQUAL_UINT8(0, e.sinceS);
  TEST_ASSERT_TRUE(reach.heardWithin(CAR, 1000, 8500));
  TEST_ASSERT_FALSE(reach.heardWithin(CAR, 1000, 9500));
}

void test_a_long_list_goes_on_in_the_next_summary() {
  Reach reach;
  reach.clear();
  for (uint32_t i = 1; i <= 20; i++) reach.heard(0xB0000000 + i, fixId((uint16_t)i, 1), 1000, 0, (uint8_t)i, 1000);
  uint8_t buf[233];
  size_t entries = 0;
  uint8_t flags = 0;
  size_t len = reach.takeSummary(2000, buf, sizeof(buf));
  TEST_ASSERT_TRUE(decodeReachHeader(buf, len, entries, flags));
  TEST_ASSERT_EQUAL_UINT32(REACH_PER_SUMMARY, entries);
  TEST_ASSERT_EQUAL_HEX8(REACH_MORE, flags);
  len = reach.takeSummary(3000, buf, sizeof(buf));
  TEST_ASSERT_TRUE(decodeReachHeader(buf, len, entries, flags));
  TEST_ASSERT_EQUAL_UINT32(4, entries);
  TEST_ASSERT_EQUAL_HEX8(0, flags);
  // And round again.
  len = reach.takeSummary(4000, buf, sizeof(buf));
  TEST_ASSERT_TRUE(decodeReachHeader(buf, len, entries, flags));
  TEST_ASSERT_EQUAL_UINT32(REACH_PER_SUMMARY, entries);
  TEST_ASSERT_EQUAL_UINT32(3, reach.summariesSent());
  TEST_ASSERT_EQUAL_UINT32(20, reach.count(4000));
}

void test_an_origin_not_heard_for_4_minutes_is_forgotten() {
  Reach reach;
  reach.clear();
  reach.heard(0xB0000001, fixId(1, 1), 1000, 0, 0x01, 1000);
  reach.heard(0xB0000002, fixId(2, 1), 1000, 0, 0x02, 200000);
  TEST_ASSERT_EQUAL_UINT32(2, reach.count(200000));
  TEST_ASSERT_EQUAL_UINT32(1, reach.count(1000 + REACH_KEEP_MS));
  ReachEntry e;
  TEST_ASSERT_FALSE(summaryEntry(reach, 0xB0000001, 1000 + REACH_KEEP_MS, e));
  // Nobody at all: no summary.
  uint8_t buf[233];
  TEST_ASSERT_EQUAL_UINT32(0, reach.takeSummary(200000 + REACH_KEEP_MS, buf, sizeof(buf)));
}

void test_a_full_table_forgets_the_origin_heard_longest_ago() {
  Reach reach;
  reach.clear();
  for (uint32_t i = 0; i < REACH_SLOTS; i++) reach.heard(0xC0000000 + i, fixId(1, 1), 0, 0, 1, 1000 + i * 10);
  reach.heard(0xCFFFFFFF, fixId(1, 1), 0, 0, 1, 5000);
  TEST_ASSERT_FALSE(reach.heardWithin(0xC0000000, REACH_KEEP_MS, 5000));
  TEST_ASSERT_TRUE(reach.heardWithin(0xC0000001, REACH_KEEP_MS, 5000));
  TEST_ASSERT_TRUE(reach.heardWithin(0xCFFFFFFF, REACH_KEEP_MS, 5000));
  TEST_ASSERT_EQUAL_UINT32(REACH_SLOTS, reach.count(5000));
}

// ---- The relay decision --------------------------------------------------------

static const uint32_t SELF = 0xA00000B1;
static const uint8_t SELF_BYTE = 0xB1;
static const uint32_t FRONT = 0xA000000A;
static const uint32_t REAR = 0xA000000C;
static const uint32_t HOLD_MS = 150000;

void test_a_summary_that_got_the_origin_through_us_grants_early_relaying() {
  RelayPrefs prefs;
  prefs.clear();
  const ReachEntry viaUs = entry(REAR, 5, 1800, 2, 1, SELF_BYTE);
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::GRANTED, prefs.consider(viaUs, FRONT, SELF, SELF_BYTE, true, 15, HOLD_MS, 1000));
  TEST_ASSERT_TRUE(prefs.preferred(REAR, 1000));
  TEST_ASSERT_FALSE(prefs.preferred(FRONT, 1000));
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::RENEWED, prefs.consider(viaUs, FRONT, SELF, SELF_BYTE, true, 15, HOLD_MS, 90000));
  // Renewed at 90 s: good until 240 s, not 151.
  TEST_ASSERT_TRUE(prefs.preferred(REAR, 200000));
  TEST_ASSERT_FALSE(prefs.preferred(REAR, 90000 + HOLD_MS));
  TEST_ASSERT_EQUAL_UINT32(1, prefs.granted());
}

void test_only_evidence_of_delivery_through_us_counts() {
  RelayPrefs prefs;
  prefs.clear();
  // First through another car.
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::NONE,
                    prefs.consider(entry(REAR, 5, 1800, 2, 1, 0x77), FRONT, SELF, SELF_BYTE, true, 15, HOLD_MS, 1000));
  // Heard directly: nobody relayed it, which says nothing about us.
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::NONE,
                    prefs.consider(entry(REAR, 5, 1800, 2, 0, SELF_BYTE), FRONT, SELF, SELF_BYTE, true, 15, HOLD_MS, 1000));
  // Hops unknown (an old sender's firmware).
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::NONE,
                    prefs.consider(entry(REAR, 5, 1800, 2, REACH_HOPS_UNKNOWN, SELF_BYTE), FRONT, SELF, SELF_BYTE, true,
                                   15, HOLD_MS, 1000));
  // Our own positions, our own summary, a car reporting itself.
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::NONE,
                    prefs.consider(entry(SELF, 5, 1800, 2, 1, SELF_BYTE), FRONT, SELF, SELF_BYTE, true, 15, HOLD_MS, 1000));
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::NONE,
                    prefs.consider(entry(REAR, 5, 1800, 2, 1, SELF_BYTE), SELF, SELF, SELF_BYTE, true, 15, HOLD_MS, 1000));
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::NONE,
                    prefs.consider(entry(REAR, 5, 1800, 2, 1, SELF_BYTE), REAR, SELF, SELF_BYTE, true, 15, HOLD_MS, 1000));
  // Through us, but we no longer hear the origin ourselves.
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::NONE,
                    prefs.consider(entry(REAR, 5, 1800, 2, 1, SELF_BYTE), FRONT, SELF, SELF_BYTE, false, 15, HOLD_MS, 1000));
  TEST_ASSERT_EQUAL_UINT32(0, prefs.count(1000));
}

void test_worse_delivery_through_us_ends_it_at_once() {
  RelayPrefs prefs;
  prefs.clear();
  const ReachEntry good = entry(REAR, 5, 1800, 2, 1, SELF_BYTE);
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::GRANTED, prefs.consider(good, FRONT, SELF, SELF_BYTE, true, 15, HOLD_MS, 1000));
  // The far car had it twenty seconds old.
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::WITHDRAWN,
                    prefs.consider(entry(REAR, 9, 20000, 2, 1, SELF_BYTE), FRONT, SELF, SELF_BYTE, true, 15, HOLD_MS, 30000));
  TEST_ASSERT_FALSE(prefs.preferred(REAR, 30000));
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::GRANTED, prefs.consider(good, FRONT, SELF, SELF_BYTE, true, 15, HOLD_MS, 60000));
  // The far car has gone without it longer than a few intervals.
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::WITHDRAWN,
                    prefs.consider(entry(REAR, 9, 1800, 40, 1, SELF_BYTE), FRONT, SELF, SELF_BYTE, true, 15, HOLD_MS, 90000));
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::GRANTED, prefs.consider(good, FRONT, SELF, SELF_BYTE, true, 15, HOLD_MS, 100000));
  // We stopped hearing the origin.
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::WITHDRAWN,
                    prefs.consider(good, FRONT, SELF, SELF_BYTE, false, 15, HOLD_MS, 110000));
  // An unknown age is no evidence either way until a grant exists.
  TEST_ASSERT_EQUAL(RelayPrefs::Verdict::NONE,
                    prefs.consider(entry(REAR, 9, UINT32_MAX, 2, 1, SELF_BYTE), FRONT, SELF, SELF_BYTE, true, 15, HOLD_MS,
                                   120000));
  TEST_ASSERT_EQUAL_UINT32(3, prefs.granted());
  TEST_ASSERT_EQUAL_UINT32(3, prefs.withdrawn());
}

void test_a_full_set_of_grants_gives_up_the_one_lapsing_soonest() {
  RelayPrefs prefs;
  prefs.clear();
  for (uint32_t i = 0; i < PREFER_SLOTS; i++) {
    prefs.consider(entry(0xD0000000 + i, 1, 1000, 1, 1, SELF_BYTE), FRONT, SELF, SELF_BYTE, true, 15, HOLD_MS, 1000 + i);
  }
  TEST_ASSERT_EQUAL_UINT32(PREFER_SLOTS, prefs.count(2000));
  prefs.consider(entry(0xDFFFFFFF, 1, 1000, 1, 1, SELF_BYTE), FRONT, SELF, SELF_BYTE, true, 15, HOLD_MS, 3000);
  TEST_ASSERT_FALSE(prefs.preferred(0xD0000000, 3000));
  TEST_ASSERT_TRUE(prefs.preferred(0xD0000001, 3000));
  TEST_ASSERT_TRUE(prefs.preferred(0xDFFFFFFF, 3000));
}

// ---- A convoy ------------------------------------------------------------------

// SHORT_FAST-like timing: 58 ms a packet and 10 ms contention slots. An early
// relay goes within 16 slots; an ordinary one waits those 16 and up to 64 more
// (RadioInterface::getTxDelayMsecWeighted: 2 x CWmax slots, then 2^CW).
static const uint32_t AIR_MS = 58;
static const uint32_t SLOT_MS = 10;
static const uint8_t HOP_LIMIT = 3;  // Meshtastic's default
static const uint32_t INTERVAL_MS = 5000;
// The module's figures at a 5 s interval (noteReachSummary).
static const uint32_t WE_HEAR_MS = 3 * INTERVAL_MS;
static const uint32_t MAX_SINCE_S = 3 * INTERVAL_MS / 1000;
static const uint32_t CONVOY_HOLD_MS = REACH_EVERY_POSITIONS * INTERVAL_MS * 5 / 2;

struct Car {
  uint32_t id = 0;
  int x = 0;
  bool up = true;
  uint32_t seq = 0;
  Reach reach;
  RelayPrefs prefs;
};

struct Copy {
  bool got = false;
  uint32_t atMs = 0;
  uint8_t hops = 0;
  uint8_t relay = 0;
};

struct Convoy {
  std::vector<Car> cars;
  uint32_t nowMs = 1000;
  uint32_t rng = 7;
  // Per origin, the last position flood: who got it, and who relayed it.
  std::vector<std::vector<Copy>> lastGot;
  std::vector<std::vector<uint32_t>> relayed;
  std::vector<uint32_t> sentAt;

  explicit Convoy(const std::vector<int>& xs) {
    for (size_t i = 0; i < xs.size(); i++) {
      Car c;
      // Distinct last bytes: Meshtastic's relay byte is the node number's last.
      c.id = 0xA0000000u | (uint32_t)(0x11 * (i + 1));
      c.x = xs[i];
      c.reach.clear();
      c.prefs.clear();
      cars.push_back(c);
    }
    lastGot.assign(cars.size(), std::vector<Copy>(cars.size()));
    relayed.assign(cars.size(), std::vector<uint32_t>(cars.size(), 0));
    sentAt.assign(cars.size(), 0);
  }

  uint32_t random() {
    rng = rng * 1103515245u + 12345u;
    return rng >> 8;
  }
  bool hears(size_t a, size_t b) const { return a != b && cars[a].up && cars[b].up && abs(cars[a].x - cars[b].x) <= 1; }
  static uint8_t relayByte(uint32_t id) { return (uint8_t)(id & 0xFF); }

  // One broadcast from [origin] at nowMs, flooded. [relays] counts who relayed it.
  std::vector<Copy> flood(size_t origin, bool position, std::vector<uint32_t>& relays) {
    const size_t n = cars.size();
    std::vector<Copy> got(n);
    std::vector<uint32_t> relayAt(n, UINT32_MAX);
    std::vector<uint8_t> relayHops(n, 0);
    got[origin].got = true;
    size_t sender = origin;
    uint32_t txAt = nowMs;
    uint8_t hopLimit = HOP_LIMIT;
    for (;;) {
      for (size_t r = 0; r < n; r++) {
        if (r == origin || !hears(sender, r)) continue;
        if (!got[r].got) {
          got[r].got = true;
          got[r].atMs = txAt + AIR_MS;
          got[r].hops = (uint8_t)(HOP_LIMIT - hopLimit);
          got[r].relay = relayByte(cars[sender].id);
          if (hopLimit > 0) {
            const bool early = position && cars[r].prefs.preferred(cars[origin].id, nowMs);
            const uint32_t delay = early ? (random() % 16) * SLOT_MS : 16 * SLOT_MS + (random() % 64) * SLOT_MS;
            relayAt[r] = txAt + AIR_MS + delay;
            relayHops[r] = (uint8_t)(hopLimit - 1);
          }
        } else if (relayAt[r] != UINT32_MAX && relayAt[r] >= txAt) {
          // Another car's copy before ours went: Meshtastic drops the queued one.
          relayAt[r] = UINT32_MAX;
        }
      }
      size_t next = n;
      for (size_t r = 0; r < n; r++) {
        if (relayAt[r] != UINT32_MAX && (next == n || relayAt[r] < relayAt[next])) next = r;
      }
      if (next == n) break;
      sender = next;
      txAt = relayAt[next];
      hopLimit = relayHops[next];
      relayAt[next] = UINT32_MAX;
      relays[next]++;
    }
    return got;
  }

  // Every car sends its position, at least a second apart, and every car that
  // gets one notes it as the module does.
  void positions() {
    for (size_t i = 0; i < cars.size(); i++) {
      if (cars[i].up) {
        FixId fix;
        fix.session = (uint16_t)(i + 1);
        fix.seq = ++cars[i].seq;
        const uint64_t utcMs = 1790000000000ull + nowMs;
        fix.fixSec = (uint32_t)(utcMs / 1000);
        fix.fixMs = (uint16_t)(utcMs % 1000);
        sentAt[i] = nowMs;
        lastGot[i] = flood(i, true, relayed[i]);
        uint32_t lastArrival = nowMs;
        for (size_t r = 0; r < cars.size(); r++) {
          const Copy& c = lastGot[i][r];
          if (r == i || !c.got) continue;
          cars[r].reach.heard(cars[i].id, fix, c.atMs - nowMs, c.hops, c.relay, c.atMs);
          if (c.atMs > lastArrival) lastArrival = c.atMs;
        }
        // The next car sends once this flood is over: no car hears anything
        // stamped later than now.
        nowMs = nowMs + 1000 > lastArrival + 100 ? nowMs + 1000 : lastArrival + 100;
      } else {
        nowMs += 1000;
      }
    }
  }

  // Every car sends its reach summary, and every car that gets one weighs it.
  void summaries() {
    std::vector<uint32_t> ignored(cars.size(), 0);
    for (size_t i = 0; i < cars.size(); i++) {
      if (!cars[i].up) continue;
      uint8_t buf[233];
      const size_t len = cars[i].reach.takeSummary(nowMs, buf, sizeof(buf));
      if (len == 0) continue;
      const std::vector<Copy> got = flood(i, false, ignored);
      for (size_t r = 0; r < cars.size(); r++) {
        if (r == i || !got[r].got) continue;
        weigh(r, cars[i].id, buf, len);
      }
      nowMs += 500;
    }
  }

  void weigh(size_t r, uint32_t reporter, const uint8_t* buf, size_t len) {
    Car& car = cars[r];
    car.reach.noteSummaryHeard();
    ReachEntry e;
    for (size_t k = 0; decodeReachEntry(buf, len, k, e); k++) {
      const bool weHearIt = car.reach.heardWithin(e.origin, WE_HEAR_MS, nowMs);
      car.prefs.consider(e, reporter, car.id, relayByte(car.id), weHearIt, MAX_SINCE_S, CONVOY_HOLD_MS, nowMs);
    }
  }
};

static bool findEntry(const uint8_t* buf, size_t len, uint32_t origin, ReachEntry& out) {
  for (size_t i = 0; decodeReachEntry(buf, len, i, out); i++) {
    if (out.origin == origin) return true;
  }
  return false;
}

// SCALE-PLAN 5e: a strung-out convoy reports which origins reach the far end
// and how old they are when they get there. Six cars in a line, each hearing
// only its neighbours, three hops: the sixth never reaches the front.
void test_a_strung_out_convoy_reports_which_origins_reach_the_far_end() {
  Convoy convoy({0, 1, 2, 3, 4, 5});
  convoy.positions();
  convoy.positions();

  uint8_t buf[233];
  const size_t len = convoy.cars[0].reach.takeSummary(convoy.nowMs, buf, sizeof(buf));
  TEST_ASSERT_TRUE(len > 0);
  uint32_t lastAgeMs = 0;
  for (size_t i = 1; i <= 4; i++) {
    ReachEntry e;
    TEST_ASSERT_TRUE(findEntry(buf, len, convoy.cars[i].id, e));
    TEST_ASSERT_EQUAL_UINT16(2, e.seq);
    TEST_ASSERT_EQUAL_UINT8(i - 1, e.hops);
    // The last relay is always the car next to the front (for its own
    // position, its own byte).
    TEST_ASSERT_EQUAL_HEX8(Convoy::relayByte(convoy.cars[1].id), e.relay);
    // How old it was when it got there: each hop adds a relay's wait and airtime.
    const uint32_t ageMs = convoy.lastGot[i][0].atMs - convoy.sentAt[i];
    TEST_ASSERT_EQUAL_UINT8(reachAgeQ(ageMs), e.ageQ);
    TEST_ASSERT_TRUE(ageMs > lastAgeMs);
    TEST_ASSERT_TRUE(ageMs >= AIR_MS + (i - 1) * (AIR_MS + 16 * SLOT_MS));
    lastAgeMs = ageMs;
  }
  ReachEntry beyond;
  TEST_ASSERT_FALSE(findEntry(buf, len, convoy.cars[5].id, beyond));

  // The front's summary travels back down the convoy as far as the hop limit
  // takes it, and every car it reaches learns the same.
  std::vector<uint32_t> relays(convoy.cars.size(), 0);
  const std::vector<Copy> got = convoy.flood(0, false, relays);
  for (size_t r = 1; r <= 4; r++) {
    TEST_ASSERT_TRUE(got[r].got);
    ReachEntry e;
    TEST_ASSERT_TRUE(findEntry(buf, len, convoy.cars[4].id, e));
    TEST_ASSERT_EQUAL_UINT8(3, e.hops);
  }
  TEST_ASSERT_FALSE(got[5].got);
}

// A line of four: each car that first carried an origin's position on to a car
// further along is preferred for that origin, from that car's summary.
void test_the_cars_that_carry_an_origin_along_the_convoy_are_preferred() {
  Convoy convoy({0, 1, 2, 3});
  const uint32_t A = convoy.cars[0].id, B = convoy.cars[1].id, C = convoy.cars[2].id, D = convoy.cars[3].id;
  convoy.positions();
  convoy.summaries();
  const uint32_t now = convoy.nowMs;
  // B carries C and D to A, and A to C; C carries D to B, and A and B to D.
  TEST_ASSERT_TRUE(convoy.cars[1].prefs.preferred(C, now));
  TEST_ASSERT_TRUE(convoy.cars[1].prefs.preferred(D, now));
  TEST_ASSERT_TRUE(convoy.cars[1].prefs.preferred(A, now));
  TEST_ASSERT_TRUE(convoy.cars[2].prefs.preferred(D, now));
  TEST_ASSERT_TRUE(convoy.cars[2].prefs.preferred(A, now));
  TEST_ASSERT_TRUE(convoy.cars[2].prefs.preferred(B, now));
  // The ends carry nobody anywhere new.
  TEST_ASSERT_EQUAL_UINT32(0, convoy.cars[0].prefs.count(now));
  TEST_ASSERT_EQUAL_UINT32(0, convoy.cars[3].prefs.count(now));
  // Everyone heard everyone else's summary: three hops cover the line.
  for (size_t i = 0; i < 4; i++) TEST_ASSERT_EQUAL_UINT32(3, convoy.cars[i].reach.summariesHeard());
}

// SCALE-PLAN 5f: the rear car keeps arriving at the front when the preferred
// relay drops out. Two cars side by side in the middle can both carry the rear
// car to the front; one is preferred, the other is the backup.
void test_the_rear_car_keeps_arriving_at_the_front_when_the_preferred_relay_drops_out() {
  Convoy convoy({0, 1, 1, 2});
  const size_t FRONT_CAR = 0, MID1 = 1, MID2 = 2, REAR_CAR = 3;
  const uint32_t rear = convoy.cars[REAR_CAR].id;
  convoy.positions();
  convoy.summaries();
  const bool firstIsPreferred = convoy.cars[MID1].prefs.preferred(rear, convoy.nowMs);
  const size_t preferred = firstIsPreferred ? MID1 : MID2;
  const size_t backup = firstIsPreferred ? MID2 : MID1;
  TEST_ASSERT_TRUE(convoy.cars[preferred].prefs.preferred(rear, convoy.nowMs));
  TEST_ASSERT_FALSE(convoy.cars[backup].prefs.preferred(rear, convoy.nowMs));

  // Preferred, it goes first every time and the backup drops its copy: one
  // relay per position, not two.
  const uint32_t preferredBefore = convoy.relayed[REAR_CAR][preferred];
  const uint32_t backupBefore = convoy.relayed[REAR_CAR][backup];
  for (int round = 0; round < 5; round++) {
    convoy.positions();
    convoy.summaries();
    TEST_ASSERT_TRUE(convoy.lastGot[REAR_CAR][FRONT_CAR].got);
  }
  TEST_ASSERT_EQUAL_UINT32(5, convoy.relayed[REAR_CAR][preferred] - preferredBefore);
  TEST_ASSERT_EQUAL_UINT32(0, convoy.relayed[REAR_CAR][backup] - backupBefore);
  TEST_ASSERT_FALSE(convoy.cars[backup].prefs.preferred(rear, convoy.nowMs));

  // The preferred car turns off down a side road.
  convoy.cars[preferred].up = false;
  for (int round = 0; round < 5; round++) {
    convoy.positions();
    // Still at the front, a hop away, now through the backup.
    TEST_ASSERT_TRUE(convoy.lastGot[REAR_CAR][FRONT_CAR].got);
    TEST_ASSERT_EQUAL_UINT8(1, convoy.lastGot[REAR_CAR][FRONT_CAR].hops);
    TEST_ASSERT_EQUAL_HEX8(Convoy::relayByte(convoy.cars[backup].id), convoy.lastGot[REAR_CAR][FRONT_CAR].relay);
    convoy.summaries();
  }
  // The front's summaries now name the backup, so it is the preferred relay.
  TEST_ASSERT_TRUE(convoy.cars[backup].prefs.preferred(rear, convoy.nowMs));
}

// SCALE-PLAN 5f: stale evidence returns every car to ordinary forwarding.
void test_stale_evidence_returns_every_car_to_ordinary_forwarding() {
  Convoy convoy({0, 1, 2, 3});
  convoy.positions();
  convoy.summaries();
  size_t preferredSomewhere = 0;
  for (size_t i = 0; i < convoy.cars.size(); i++) preferredSomewhere += convoy.cars[i].prefs.count(convoy.nowMs);
  TEST_ASSERT_TRUE(preferredSomewhere >= 6);

  // No summaries get through for longer than a grant holds.
  const uint32_t lapseAt = convoy.nowMs + CONVOY_HOLD_MS;
  while ((int32_t)(convoy.nowMs - lapseAt) <= 0) {
    convoy.positions();
    // Positions still cross the whole line meanwhile.
    TEST_ASSERT_TRUE(convoy.lastGot[3][0].got);
    TEST_ASSERT_TRUE(convoy.lastGot[0][3].got);
  }
  for (size_t i = 0; i < convoy.cars.size(); i++) TEST_ASSERT_EQUAL_UINT32(0, convoy.cars[i].prefs.count(convoy.nowMs));
  // And with no grants, every relay draws the ordinary delay: the rear car's
  // positions reach the front all the same.
  convoy.positions();
  TEST_ASSERT_TRUE(convoy.lastGot[3][0].got);
  TEST_ASSERT_EQUAL_UINT8(2, convoy.lastGot[3][0].hops);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_a_summary_round_trips);
  RUN_TEST(test_ages_go_in_quarter_seconds);
  RUN_TEST(test_an_entry_reads_as_the_app_shows_cars);
  RUN_TEST(test_the_newest_fix_per_origin_is_kept);
  RUN_TEST(test_a_long_list_goes_on_in_the_next_summary);
  RUN_TEST(test_an_origin_not_heard_for_4_minutes_is_forgotten);
  RUN_TEST(test_a_full_table_forgets_the_origin_heard_longest_ago);
  RUN_TEST(test_a_summary_that_got_the_origin_through_us_grants_early_relaying);
  RUN_TEST(test_only_evidence_of_delivery_through_us_counts);
  RUN_TEST(test_worse_delivery_through_us_ends_it_at_once);
  RUN_TEST(test_a_full_set_of_grants_gives_up_the_one_lapsing_soonest);
  RUN_TEST(test_a_strung_out_convoy_reports_which_origins_reach_the_far_end);
  RUN_TEST(test_the_cars_that_carry_an_origin_along_the_convoy_are_preferred);
  RUN_TEST(test_the_rear_car_keeps_arriving_at_the_front_when_the_preferred_relay_drops_out);
  RUN_TEST(test_stale_evidence_returns_every_car_to_ordinary_forwarding);
  return UNITY_END();
}
