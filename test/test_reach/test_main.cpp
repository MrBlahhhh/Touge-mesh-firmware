// Host tests for end-to-end delivery summaries (SCALE-PLAN 5e), relays chosen
// on their evidence (5f), and from build 43 relays nobody needs, skipped on the
// claims summaries carry, with early summaries when a car loses an origin.
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

// Our interval, which the table takes as every origin's, and how long a car's
// summary counts at it (summaryHoldMs in the module): 150 s.
static const uint32_t INTERVAL_MS = 5000;
static const uint32_t HOLD_MS = REACH_EVERY_POSITIONS * INTERVAL_MS * 5 / 2;

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
    TEST_ASSERT_FALSE(got.steady);
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

// Build 43: the header says the entries carry the steady flag, and whether the
// summary is an early one; an entry says it in its hops byte.
void test_the_steady_and_early_flags_round_trip() {
  ReachEntry sent[3] = {entry(0xA000A1B2, 1234, 3400, 12, 1, 0x5E), entry(0xA000C3D4, 65535, 250, 0, 0, 0xD4),
                        entry(0x0000000F, 7, UINT32_MAX, 255, REACH_HOPS_UNKNOWN, 0x33)};
  sent[1].steady = true;
  uint8_t buf[64];
  const size_t len = encodeReach(sent, 3, REACH_EARLY | REACH_STEADY, buf, sizeof(buf));
  // LoraReachTest pins these too.
  const uint8_t pinned[] = {0xC3, 0x01, 0x03, 0x06,                                      //
                            0xA0, 0x00, 0xA1, 0xB2, 0x04, 0xD2, 0x0D, 0x0C, 0x01, 0x5E,  //
                            0xA0, 0x00, 0xC3, 0xD4, 0xFF, 0xFF, 0x01, 0x00, 0x10, 0xD4,  //
                            0x00, 0x00, 0x00, 0x0F, 0x00, 0x07, 0xFF, 0xFF, 0x0F, 0x33};
  TEST_ASSERT_EQUAL_UINT32(sizeof(pinned), len);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(pinned, buf, sizeof(pinned));
  ReachEntry got;
  TEST_ASSERT_TRUE(decodeReachEntry(buf, len, 1, got));
  TEST_ASSERT_TRUE(got.steady);
  TEST_ASSERT_EQUAL_UINT8(0, got.hops);
  TEST_ASSERT_TRUE(decodeReachEntry(buf, len, 0, got));
  TEST_ASSERT_FALSE(got.steady);
  TEST_ASSERT_EQUAL_UINT8(1, got.hops);
  // Without the header's flag the bit means nothing.
  buf[3] = REACH_EARLY;
  TEST_ASSERT_TRUE(decodeReachEntry(buf, len, 1, got));
  TEST_ASSERT_FALSE(got.steady);
  TEST_ASSERT_EQUAL_UINT8(0, got.hops);
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
  ReachEntry steadyDirect = entry(0xA000C3D4, 9, 250, 0, 0, 0xD4);
  steadyDirect.steady = true;
  TEST_ASSERT_TRUE(formatReachEntry(steadyDirect, out, sizeof(out)) > 0);
  TEST_ASSERT_EQUAL_STRING("c3d4#9 0.2s 0h steady -0s", out);
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
  const size_t len = reach.takeSummary(nowMs, INTERVAL_MS, buf, sizeof(buf));
  for (size_t i = 0; decodeReachEntry(buf, len, i, out); i++) {
    if (out.origin == origin) return true;
  }
  return false;
}

void test_the_newest_fix_per_origin_is_kept() {
  Reach reach;
  reach.clear();
  const uint32_t CAR = 0xA000B2B2;
  reach.heard(CAR, fixId(7, 10), 1200, 1, 0x44, INTERVAL_MS, 1000);
  // A late copy of an older fix changes nothing.
  reach.heard(CAR, fixId(7, 9), 800, 0, 0xB2, INTERVAL_MS, 1500);
  ReachEntry e;
  TEST_ASSERT_TRUE(summaryEntry(reach, CAR, 3000, e));
  TEST_ASSERT_EQUAL_UINT16(10, e.seq);
  TEST_ASSERT_EQUAL_UINT8(1, e.hops);
  TEST_ASSERT_EQUAL_HEX8(0x44, e.relay);
  TEST_ASSERT_EQUAL_UINT8(2, e.sinceS);
  // The same fix again (a parked phone's repeat) is still the car heard.
  reach.heard(CAR, fixId(7, 10), 6000, 0, 0xB2, INTERVAL_MS, 5000);
  TEST_ASSERT_TRUE(summaryEntry(reach, CAR, 5000, e));
  TEST_ASSERT_EQUAL_UINT8(0, e.sinceS);
  TEST_ASSERT_EQUAL_UINT8(0, e.hops);
  // A rebooted car starts its sequence again under a new session.
  reach.heard(CAR, fixId(9, 1), 900, 0, 0xB2, INTERVAL_MS, 6000);
  TEST_ASSERT_TRUE(summaryEntry(reach, CAR, 6000, e));
  TEST_ASSERT_EQUAL_UINT16(1, e.seq);
  // The low sixteen bits carry on across their wrap.
  reach.heard(CAR, fixId(9, 30000), 900, 0, 0xB2, INTERVAL_MS, 6500);
  reach.heard(CAR, fixId(9, 60000), 900, 0, 0xB2, INTERVAL_MS, 7000);
  reach.heard(CAR, fixId(9, 65537), 900, 0, 0xB2, INTERVAL_MS, 8000);
  TEST_ASSERT_TRUE(summaryEntry(reach, CAR, 8000, e));
  TEST_ASSERT_EQUAL_UINT16(1, e.seq);
  TEST_ASSERT_EQUAL_UINT8(0, e.sinceS);
  TEST_ASSERT_TRUE(reach.heardWithin(CAR, 1000, 8500));
  TEST_ASSERT_FALSE(reach.heardWithin(CAR, 1000, 9500));
}

void test_a_long_list_goes_on_in_the_next_summary() {
  Reach reach;
  reach.clear();
  for (uint32_t i = 1; i <= 20; i++) {
    reach.heard(0xB0000000 + i, fixId((uint16_t)i, 1), 1000, 0, (uint8_t)i, INTERVAL_MS, 1000);
  }
  uint8_t buf[233];
  size_t entries = 0;
  uint8_t flags = 0;
  size_t len = reach.takeSummary(2000, INTERVAL_MS, buf, sizeof(buf));
  TEST_ASSERT_TRUE(decodeReachHeader(buf, len, entries, flags));
  TEST_ASSERT_EQUAL_UINT32(REACH_PER_SUMMARY, entries);
  TEST_ASSERT_EQUAL_HEX8(REACH_MORE | REACH_STEADY, flags);
  len = reach.takeSummary(3000, INTERVAL_MS, buf, sizeof(buf));
  TEST_ASSERT_TRUE(decodeReachHeader(buf, len, entries, flags));
  TEST_ASSERT_EQUAL_UINT32(4, entries);
  TEST_ASSERT_EQUAL_HEX8(REACH_STEADY, flags);
  // And round again.
  len = reach.takeSummary(4000, INTERVAL_MS, buf, sizeof(buf));
  TEST_ASSERT_TRUE(decodeReachHeader(buf, len, entries, flags));
  TEST_ASSERT_EQUAL_UINT32(REACH_PER_SUMMARY, entries);
  TEST_ASSERT_EQUAL_UINT32(3, reach.summariesSent());
  TEST_ASSERT_EQUAL_UINT32(20, reach.count(4000));
}

void test_an_origin_not_heard_for_4_minutes_is_forgotten() {
  Reach reach;
  reach.clear();
  reach.heard(0xB0000001, fixId(1, 1), 1000, 0, 0x01, INTERVAL_MS, 1000);
  reach.heard(0xB0000002, fixId(2, 1), 1000, 0, 0x02, INTERVAL_MS, 200000);
  TEST_ASSERT_EQUAL_UINT32(2, reach.count(200000));
  TEST_ASSERT_EQUAL_UINT32(1, reach.count(1000 + REACH_KEEP_MS));
  ReachEntry e;
  TEST_ASSERT_FALSE(summaryEntry(reach, 0xB0000001, 1000 + REACH_KEEP_MS, e));
  // Nobody at all: no summary.
  uint8_t buf[233];
  TEST_ASSERT_EQUAL_UINT32(0, reach.takeSummary(200000 + REACH_KEEP_MS, INTERVAL_MS, buf, sizeof(buf)));
}

void test_a_full_table_forgets_the_origin_heard_longest_ago() {
  Reach reach;
  reach.clear();
  for (uint32_t i = 0; i < REACH_SLOTS; i++) reach.heard(0xC0000000 + i, fixId(1, 1), 0, 0, 1, INTERVAL_MS, 1000 + i * 10);
  reach.heard(0xCFFFFFFF, fixId(1, 1), 0, 0, 1, INTERVAL_MS, 5000);
  TEST_ASSERT_FALSE(reach.heardWithin(0xC0000000, REACH_KEEP_MS, 5000));
  TEST_ASSERT_TRUE(reach.heardWithin(0xC0000001, REACH_KEEP_MS, 5000));
  TEST_ASSERT_TRUE(reach.heardWithin(0xCFFFFFFF, REACH_KEEP_MS, 5000));
  TEST_ASSERT_EQUAL_UINT32(REACH_SLOTS, reach.count(5000));
}

// 30 origins, one more than a 30-car ride shows any car, each heard direct once
// an interval in rank order, each summary claiming every other car steady. With
// 28 slots (MAX_RIDERS, to build 46) each arrival evicted the car due next,
// wiping its claims, so one car was always UNPROVEN and no relay was skipped.
void test_30_origins_keep_their_slots_and_claims() {
  static const uint32_t OTHERS = 30;
  TEST_ASSERT_TRUE(REACH_SLOTS >= OTHERS);
  Reach reach;
  reach.clear();
  uint32_t cars[OTHERS];
  for (uint32_t k = 0; k < OTHERS; k++) cars[k] = 0xD0000001 + k;
  uint32_t seq[OTHERS] = {0};
  uint32_t nowMs = 0;
  for (uint32_t round = 0; round < 4; round++) {
    for (uint32_t k = 0; k < OTHERS; k++) {
      nowMs = 1000 + round * INTERVAL_MS + k * (INTERVAL_MS / OTHERS);
      reach.heard(cars[k], fixId(1, ++seq[k]), 300, 0, (uint8_t)cars[k], INTERVAL_MS, nowMs);
      ReachEntry claims[OTHERS];
      size_t n = 0;
      for (uint32_t o = 0; o < OTHERS; o++) {
        if (o == k) continue;
        claims[n] = entry(cars[o], 9, 300, 1, 0, (uint8_t)cars[o]);
        claims[n++].steady = true;
      }
      uint8_t buf[REACH_HEADER + OTHERS * REACH_ENTRY];
      const size_t len = encodeReach(claims, n, REACH_STEADY, buf, sizeof(buf));
      TEST_ASSERT_TRUE(reach.noteSummary(cars[k], buf, len, nowMs));
    }
  }

  TEST_ASSERT_EQUAL_UINT32(OTHERS, reach.count(nowMs));
  for (uint32_t o = 0; o < OTHERS; o++) {
    TEST_ASSERT_TRUE(reach.heardWithin(cars[o], INTERVAL_MS * 2, nowMs));
    uint32_t known[OTHERS];
    size_t n = 0;
    for (uint32_t c = 0; c < OTHERS; c++) {
      if (c == o) continue;
      TEST_ASSERT_EQUAL(DirectClaim::STEADY, reach.claim(cars[c], cars[o], HOLD_MS, nowMs));
      known[n++] = cars[c];
    }
    TEST_ASSERT_EQUAL(RelayVerdict::SKIP, judgeRelay(reach, cars[o], known, n, HOLD_MS, nowMs));
  }

  // Our own summaries still list all 30, sixteen and then the rest.
  bool listed[OTHERS] = {false};
  uint8_t buf[233];
  for (int part = 0; part < 2; part++) {
    const size_t len = reach.takeSummary(nowMs, INTERVAL_MS, buf, sizeof(buf));
    ReachEntry e;
    for (size_t i = 0; decodeReachEntry(buf, len, i, e); i++) {
      TEST_ASSERT_TRUE(e.origin >= cars[0] && e.origin <= cars[OTHERS - 1]);
      listed[e.origin - cars[0]] = true;
    }
  }
  for (uint32_t o = 0; o < OTHERS; o++) TEST_ASSERT_TRUE(listed[o]);
}

// ---- Heard steadily direct (build 43) ------------------------------------------

static const uint32_t ORIGIN = 0xA00000C1;
static const uint32_t REPORTER = 0xA00000D4;

// [n] positions from [origin], an interval apart from [fromMs], first heard
// [hops] out, each fix [ageMs] old on arrival. Returns when the last came in.
static uint32_t hearPositions(Reach& reach, uint32_t origin, uint32_t n, uint32_t fromMs, uint8_t hops, uint32_t& seq,
                              uint32_t ageMs = 300) {
  uint32_t atMs = fromMs;
  for (uint32_t i = 0; i < n; i++) {
    atMs = fromMs + i * INTERVAL_MS;
    reach.heard(origin, fixId(3, ++seq), ageMs, hops, hops == 0 ? (uint8_t)origin : 0x77, INTERVAL_MS, atMs);
  }
  return atMs;
}

void test_an_origin_is_claimed_steady_after_four_direct_positions_in_a_row() {
  Reach reach;
  reach.clear();
  uint32_t seq = 0;
  uint32_t atMs = hearPositions(reach, ORIGIN, 3, 1000, 0, seq);
  ReachEntry e;
  TEST_ASSERT_TRUE(summaryEntry(reach, ORIGIN, atMs + 100, e));
  TEST_ASSERT_FALSE(e.steady);
  atMs = hearPositions(reach, ORIGIN, 1, atMs + INTERVAL_MS, 0, seq);
  TEST_ASSERT_TRUE(summaryEntry(reach, ORIGIN, atMs + 100, e));
  TEST_ASSERT_TRUE(e.steady);
  // Not once the latest is more than an interval and a half old.
  TEST_ASSERT_TRUE(summaryEntry(reach, ORIGIN, atMs + INTERVAL_MS * 3 / 2 + 1, e));
  TEST_ASSERT_FALSE(e.steady);
}

void test_a_relayed_copy_or_a_missed_position_starts_the_count_again() {
  Reach reach;
  reach.clear();
  uint32_t seq = 0;
  uint32_t atMs = hearPositions(reach, ORIGIN, 5, 1000, 0, seq);
  ReachEntry e;
  TEST_ASSERT_TRUE(summaryEntry(reach, ORIGIN, atMs, e));
  TEST_ASSERT_TRUE(e.steady);
  // First through a relay: not direct, and the count starts again.
  atMs = hearPositions(reach, ORIGIN, 1, atMs + INTERVAL_MS, 1, seq);
  TEST_ASSERT_TRUE(summaryEntry(reach, ORIGIN, atMs, e));
  TEST_ASSERT_FALSE(e.steady);
  atMs = hearPositions(reach, ORIGIN, 4, atMs + INTERVAL_MS, 0, seq);
  TEST_ASSERT_TRUE(summaryEntry(reach, ORIGIN, atMs, e));
  TEST_ASSERT_TRUE(e.steady);
  // A position missed: two intervals between two of them.
  atMs = hearPositions(reach, ORIGIN, 1, atMs + 2 * INTERVAL_MS, 0, seq);
  TEST_ASSERT_TRUE(summaryEntry(reach, ORIGIN, atMs, e));
  TEST_ASSERT_FALSE(e.steady);
  atMs = hearPositions(reach, ORIGIN, 3, atMs + INTERVAL_MS, 0, seq);
  TEST_ASSERT_TRUE(summaryEntry(reach, ORIGIN, atMs, e));
  TEST_ASSERT_TRUE(e.steady);
  // Direct and regular with a fix twelve seconds old on arrival is still
  // steady: the age is the origin's GPS, which a relay would carry unchanged
  // (build 44; indoors every fix arrived 20-30 s old and no relay was skipped).
  Reach late;
  late.clear();
  uint32_t lateSeq = 0;
  atMs = hearPositions(late, ORIGIN, 5, 1000, 0, lateSeq, 12000);
  TEST_ASSERT_TRUE(summaryEntry(late, ORIGIN, atMs, e));
  TEST_ASSERT_TRUE(e.steady);
}

// A one-entry summary about [origin] as a car would send it.
static size_t summaryAbout(uint32_t origin, bool steady, uint8_t flags, uint8_t* buf, size_t cap) {
  ReachEntry e = entry(origin, 9, 300, 1, steady ? 0 : 1, 0x5E);
  e.steady = steady;
  return encodeReach(&e, 1, flags, buf, cap);
}

void test_a_cars_claims_come_from_its_summaries_and_age_out() {
  Reach reach;
  reach.clear();
  uint32_t seq = 0, reporterSeq = 0;
  hearPositions(reach, ORIGIN, 1, 1000, 0, seq);
  hearPositions(reach, REPORTER, 1, 1000, 0, reporterSeq);
  // Nothing from the reporter yet.
  TEST_ASSERT_EQUAL(DirectClaim::UNPROVEN, reach.claim(REPORTER, ORIGIN, HOLD_MS, 2000));
  uint8_t buf[64];
  size_t len = summaryAbout(ORIGIN, true, REACH_STEADY, buf, sizeof(buf));
  TEST_ASSERT_TRUE(reach.noteSummary(REPORTER, buf, len, 2000));
  TEST_ASSERT_EQUAL(DirectClaim::STEADY, reach.claim(REPORTER, ORIGIN, HOLD_MS, 2000));
  // Good for two and a half summary periods, then stale.
  TEST_ASSERT_EQUAL(DirectClaim::STEADY, reach.claim(REPORTER, ORIGIN, HOLD_MS, 2000 + HOLD_MS - 10000));
  TEST_ASSERT_EQUAL(DirectClaim::STALE, reach.claim(REPORTER, ORIGIN, HOLD_MS, 2000 + HOLD_MS + 5000));
  // A fresh summary saying otherwise withdraws it at once.
  len = summaryAbout(ORIGIN, false, REACH_STEADY, buf, sizeof(buf));
  TEST_ASSERT_TRUE(reach.noteSummary(REPORTER, buf, len, 3000));
  TEST_ASSERT_EQUAL(DirectClaim::NOT_STEADY, reach.claim(REPORTER, ORIGIN, HOLD_MS, 3000));
  // An origin it never listed, it never claimed.
  hearPositions(reach, 0xA00000EE, 1, 3000, 0, seq);
  TEST_ASSERT_EQUAL(DirectClaim::NOT_STEADY, reach.claim(REPORTER, 0xA00000EE, HOLD_MS, 3000));
  // A car gone quiet for four minutes is forgotten with its claims, and back
  // again it has claimed nothing yet.
  hearPositions(reach, ORIGIN, 1, 200000, 0, seq);
  TEST_ASSERT_EQUAL(DirectClaim::UNPROVEN, reach.claim(REPORTER, ORIGIN, HOLD_MS, 1000 + REACH_KEEP_MS));
  hearPositions(reach, REPORTER, 1, 1000 + REACH_KEEP_MS, 0, reporterSeq);
  TEST_ASSERT_EQUAL(DirectClaim::UNPROVEN, reach.claim(REPORTER, ORIGIN, HOLD_MS, 1000 + REACH_KEEP_MS));
}

void test_only_a_build_43_summary_from_a_car_we_hear_makes_a_claim() {
  uint8_t buf[64];
  uint32_t seq = 0, reporterSeq = 0;
  // A build 41 summary: no steady flag in its header, so no claim either way.
  Reach reach;
  reach.clear();
  hearPositions(reach, ORIGIN, 1, 1000, 0, seq);
  hearPositions(reach, REPORTER, 1, 1000, 0, reporterSeq);
  size_t len = summaryAbout(ORIGIN, true, 0, buf, sizeof(buf));
  TEST_ASSERT_TRUE(reach.noteSummary(REPORTER, buf, len, 2000));
  TEST_ASSERT_EQUAL_UINT32(1, reach.summariesHeard());
  TEST_ASSERT_EQUAL(DirectClaim::UNPROVEN, reach.claim(REPORTER, ORIGIN, HOLD_MS, 2000));
  // A reporter whose positions we have not heard over LoRa has no slot to keep claims in.
  Reach unheard;
  unheard.clear();
  hearPositions(unheard, ORIGIN, 1, 1000, 0, seq);
  len = summaryAbout(ORIGIN, true, REACH_STEADY, buf, sizeof(buf));
  TEST_ASSERT_TRUE(unheard.noteSummary(REPORTER, buf, len, 2000));
  TEST_ASSERT_EQUAL(DirectClaim::UNPROVEN, unheard.claim(REPORTER, ORIGIN, HOLD_MS, 2000));
  // Not a summary at all.
  const uint8_t json[] = "{\"fl\":{}}";
  TEST_ASSERT_FALSE(reach.noteSummary(REPORTER, json, sizeof(json), 2000));
}

// An early summary changes the claims it lists; the rest still age from the
// car's last full summary.
void test_an_early_summary_changes_what_it_lists_and_leaves_the_clock() {
  Reach reach;
  reach.clear();
  uint32_t seq = 0, reporterSeq = 0;
  const uint32_t OTHER = 0xA00000E1;
  hearPositions(reach, ORIGIN, 1, 1000, 0, seq);
  hearPositions(reach, OTHER, 1, 1000, 0, seq);
  hearPositions(reach, REPORTER, 1, 1000, 0, reporterSeq);
  ReachEntry both[2] = {entry(ORIGIN, 9, 300, 1, 0, 0xC1), entry(OTHER, 9, 300, 1, 0, 0xE1)};
  both[0].steady = both[1].steady = true;
  uint8_t buf[64];
  size_t len = encodeReach(both, 2, REACH_STEADY, buf, sizeof(buf));
  reach.noteSummary(REPORTER, buf, len, 2000);
  len = summaryAbout(ORIGIN, false, REACH_EARLY | REACH_STEADY, buf, sizeof(buf));
  reach.noteSummary(REPORTER, buf, len, 60000);
  TEST_ASSERT_EQUAL(DirectClaim::NOT_STEADY, reach.claim(REPORTER, ORIGIN, HOLD_MS, 60000));
  TEST_ASSERT_EQUAL(DirectClaim::STEADY, reach.claim(REPORTER, OTHER, HOLD_MS, 60000));
  TEST_ASSERT_EQUAL(DirectClaim::STALE, reach.claim(REPORTER, OTHER, HOLD_MS, 2000 + HOLD_MS + 5000));
}

// A car that stays in range but stops sending summaries (a radio back in stock
// mode) goes stale, then has claimed nothing, and its old claims never come
// back as the clock the stamps run on goes round (4.7 hours).
void test_claims_from_a_car_that_stopped_summarising_never_come_back() {
  Reach reach;
  reach.clear();
  uint32_t seq = 0, reporterSeq = 0;
  hearPositions(reach, ORIGIN, 1, 1000, 0, seq);
  hearPositions(reach, REPORTER, 1, 1000, 0, reporterSeq);
  uint8_t buf[64];
  const size_t len = summaryAbout(ORIGIN, true, REACH_STEADY, buf, sizeof(buf));
  reach.noteSummary(REPORTER, buf, len, 1000);
  bool staleSeen = false;
  uint32_t atMs = 1000;
  for (uint32_t i = 0; i < 5 * 3600000 / INTERVAL_MS; i++) {
    atMs += INTERVAL_MS;
    reach.heard(ORIGIN, fixId(3, ++seq), 300, 0, 0xC1, INTERVAL_MS, atMs);
    reach.heard(REPORTER, fixId(4, ++reporterSeq), 300, 0, 0xD4, INTERVAL_MS, atMs);
    const DirectClaim c = reach.claim(REPORTER, ORIGIN, HOLD_MS, atMs);
    if (atMs - 1000 > HOLD_MS + 1000) {
      TEST_ASSERT_TRUE(c != DirectClaim::STEADY);
    }
    if (c == DirectClaim::STALE) staleSeen = true;
    if (atMs - 1000 > 16 * 60000) {
      TEST_ASSERT_EQUAL(DirectClaim::UNPROVEN, c);
    }
  }
  TEST_ASSERT_TRUE(staleSeen);
}

// ---- Which relays go (build 43) -------------------------------------------------

void test_a_relay_is_skipped_only_when_every_known_car_claims_the_origin() {
  Reach reach;
  reach.clear();
  const uint32_t CAR_1 = 0xA0000011, CAR_2 = 0xA0000022, CAR_3 = 0xA0000033, CAR_4 = 0xA0000044;
  uint32_t seq = 0;
  hearPositions(reach, ORIGIN, 1, 1000, 0, seq);
  hearPositions(reach, CAR_1, 1, 1000, 0, seq);
  hearPositions(reach, CAR_2, 1, 1000, 0, seq);
  hearPositions(reach, CAR_3, 1, 1000, 0, seq);
  uint8_t buf[64];
  size_t len = summaryAbout(ORIGIN, true, REACH_STEADY, buf, sizeof(buf));
  reach.noteSummary(CAR_1, buf, len, 2000);
  reach.noteSummary(CAR_2, buf, len, 2000);
  len = summaryAbout(ORIGIN, false, REACH_STEADY, buf, sizeof(buf));
  reach.noteSummary(CAR_3, buf, len, 2000);

  const uint32_t steadyPair[] = {CAR_1, CAR_2};
  TEST_ASSERT_EQUAL(RelayVerdict::SKIP, judgeRelay(reach, ORIGIN, steadyPair, 2, HOLD_MS, 3000));
  // One car that does not hear it directly is reason enough.
  const uint32_t withNeedy[] = {CAR_1, CAR_3, CAR_2};
  TEST_ASSERT_EQUAL(RelayVerdict::NEEDED, judgeRelay(reach, ORIGIN, withNeedy, 3, HOLD_MS, 3000));
  // So is one that has claimed nothing: a stock node, or a car just heard.
  const uint32_t withSilent[] = {CAR_1, CAR_4};
  TEST_ASSERT_EQUAL(RelayVerdict::NO_EVIDENCE, judgeRelay(reach, ORIGIN, withSilent, 2, HOLD_MS, 3000));
  // Nobody else known is nobody to relay for.
  TEST_ASSERT_EQUAL(RelayVerdict::SKIP, judgeRelay(reach, ORIGIN, steadyPair, 0, HOLD_MS, 3000));
  // Stale claims count for nothing, and the strongest reason is the one given:
  // needed, then stale, then no evidence.
  const uint32_t lateMs = 2000 + HOLD_MS + 5000;
  TEST_ASSERT_EQUAL(RelayVerdict::STALE, judgeRelay(reach, ORIGIN, steadyPair, 2, HOLD_MS, lateMs));
  const uint32_t silentAndStale[] = {CAR_4, CAR_1};
  TEST_ASSERT_EQUAL(RelayVerdict::STALE, judgeRelay(reach, ORIGIN, silentAndStale, 2, HOLD_MS, lateMs));
  reach.noteSummary(CAR_3, buf, len, lateMs - 1000);
  const uint32_t all[] = {CAR_4, CAR_1, CAR_3};
  TEST_ASSERT_EQUAL(RelayVerdict::NEEDED, judgeRelay(reach, ORIGIN, all, 3, HOLD_MS, lateMs));
}

static uint32_t knownAsked = 0;

static bool oneSilentCar(uint32_t origin, uint32_t* cars, size_t cap, size_t& n, void* ctx) {
  (void)origin;
  (void)ctx;
  knownAsked++;
  n = 0;
  if (cap == 0) return false;
  cars[n++] = 0xA00000FE;
  return true;
}

static bool tooManyCars(uint32_t origin, uint32_t* cars, size_t cap, size_t& n, void* ctx) {
  (void)origin;
  (void)cars;
  (void)cap;
  (void)ctx;
  knownAsked++;
  n = 0;
  return false;
}

static bool neverQueued(uint32_t, uint32_t, void*) { return false; }

void test_the_relay_veto_never_touches_anything_but_a_touge_position() {
  Reach reach;
  reach.clear();
  TxPositions noted;
  noted.clear();
  noted.note(ORIGIN, 100, fixId(3, 1), 1000, &neverQueued, nullptr);
  knownAsked = 0;
  // Text, NodeInfo, control, a summary: never noted, so stock, without even a
  // look at who is around.
  TEST_ASSERT_EQUAL(RelayVerdict::STOCK, relayVerdict(noted, ORIGIN, 101, reach, &oneSilentCar, nullptr, HOLD_MS, 1000));
  // A stock position carries no identity and is never noted either.
  noted.note(ORIGIN, 102, fixId(0, 5), 1000, &neverQueued, nullptr);
  TEST_ASSERT_EQUAL(RelayVerdict::STOCK, relayVerdict(noted, ORIGIN, 102, reach, &oneSilentCar, nullptr, HOLD_MS, 1000));
  TEST_ASSERT_EQUAL_UINT32(0, knownAsked);
  // The Touge position is judged.
  TEST_ASSERT_EQUAL(RelayVerdict::NO_EVIDENCE,
                    relayVerdict(noted, ORIGIN, 100, reach, &oneSilentCar, nullptr, HOLD_MS, 1000));
  TEST_ASSERT_EQUAL_UINT32(1, knownAsked);
  // More cars around than a decision reads: nobody can be shown not to need it.
  TEST_ASSERT_EQUAL(RelayVerdict::NO_EVIDENCE,
                    relayVerdict(noted, ORIGIN, 100, reach, &tooManyCars, nullptr, HOLD_MS, 1000));
  // The counts "le" reports: sk, rn, rs, rd.
  RelaySkips skips;
  skips.note(RelayVerdict::STOCK);
  skips.note(RelayVerdict::SKIP);
  skips.note(RelayVerdict::NO_EVIDENCE);
  skips.note(RelayVerdict::NO_EVIDENCE);
  skips.note(RelayVerdict::STALE);
  skips.note(RelayVerdict::NEEDED);
  TEST_ASSERT_EQUAL_UINT32(1, skips.skipped);
  TEST_ASSERT_EQUAL_UINT32(2, skips.noEvidence);
  TEST_ASSERT_EQUAL_UINT32(1, skips.stale);
  TEST_ASSERT_EQUAL_UINT32(1, skips.needed);
}

// ---- Early summaries (build 43) --------------------------------------------------

void test_an_origin_claimed_steady_and_quiet_two_and_a_half_intervals_goes_early() {
  Reach reach;
  reach.clear();
  uint32_t seq = 0;
  const uint32_t lastMs = hearPositions(reach, ORIGIN, 5, 1000, 0, seq);
  uint8_t buf[233];
  // Our summary claims it steady.
  TEST_ASSERT_TRUE(reach.takeSummary(lastMs + 100, INTERVAL_MS, buf, sizeof(buf)) > 0);
  // Two positions late is not yet two missed.
  TEST_ASSERT_FALSE(reach.earlyDue(lastMs + INTERVAL_MS * 5 / 2, INTERVAL_MS, 0));
  // Past that it is due, here at once (a random part of a quarter interval: 0).
  const uint32_t quietMs = lastMs + INTERVAL_MS * 5 / 2 + 1;
  TEST_ASSERT_TRUE(reach.earlyDue(quietMs, INTERVAL_MS, 0));
  const size_t len = reach.takeEarlySummary(quietMs, INTERVAL_MS, buf, sizeof(buf));
  size_t entries = 0;
  uint8_t flags = 0;
  TEST_ASSERT_TRUE(decodeReachHeader(buf, len, entries, flags));
  TEST_ASSERT_EQUAL_UINT32(1, entries);
  TEST_ASSERT_EQUAL_HEX8(REACH_EARLY | REACH_STEADY, flags);
  ReachEntry e;
  TEST_ASSERT_TRUE(decodeReachEntry(buf, len, 0, e));
  TEST_ASSERT_EQUAL_HEX32(ORIGIN, e.origin);
  TEST_ASSERT_FALSE(e.steady);
  // Said, it is not said again, bar one repeat two intervals on if the origin
  // stays quiet, in case the first was lost.
  TEST_ASSERT_FALSE(reach.earlyDue(quietMs + INTERVAL_MS, INTERVAL_MS, 0));
  TEST_ASSERT_TRUE(reach.earlyDue(quietMs + 2 * INTERVAL_MS, INTERVAL_MS, 0));
  TEST_ASSERT_TRUE(reach.takeEarlySummary(quietMs + 2 * INTERVAL_MS, INTERVAL_MS, buf, sizeof(buf)) > 0);
  TEST_ASSERT_FALSE(reach.earlyDue(quietMs + 10 * INTERVAL_MS, INTERVAL_MS, 0));
  TEST_ASSERT_EQUAL_UINT32(2, reach.earlySent());
  TEST_ASSERT_EQUAL_UINT32(3, reach.summariesSent());
}

void test_early_summaries_keep_an_interval_apart_and_stop_once_the_origin_is_heard() {
  Reach reach;
  reach.clear();
  const uint32_t ORIGIN_B = ORIGIN + 1;
  uint32_t seq = 0, seqB = 0;
  // The first every 5 s from 1 s, the last at 21 s; the second from 5 s, the
  // last at 25 s. In time order: the table's clock only goes forward.
  for (uint32_t i = 0; i < 5; i++) {
    hearPositions(reach, ORIGIN, 1, 1000 + i * INTERVAL_MS, 0, seq);
    hearPositions(reach, ORIGIN_B, 1, 5000 + i * INTERVAL_MS, 0, seqB);
  }
  uint8_t buf[233];
  TEST_ASSERT_TRUE(reach.takeSummary(25100, INTERVAL_MS, buf, sizeof(buf)) > 0);
  // The first goes quiet: due a random part of a quarter interval later, a
  // draw that holds once made.
  TEST_ASSERT_FALSE(reach.earlyDue(33501, INTERVAL_MS, 1000));
  TEST_ASSERT_FALSE(reach.earlyDue(34500, INTERVAL_MS, 7));
  TEST_ASSERT_TRUE(reach.earlyDue(34501, INTERVAL_MS, 7));
  size_t len = reach.takeEarlySummary(34501, INTERVAL_MS, buf, sizeof(buf));
  size_t entries = 0;
  uint8_t flags = 0;
  ReachEntry e;
  TEST_ASSERT_TRUE(decodeReachHeader(buf, len, entries, flags));
  TEST_ASSERT_EQUAL_UINT32(1, entries);
  TEST_ASSERT_TRUE(decodeReachEntry(buf, len, 0, e));
  TEST_ASSERT_EQUAL_HEX32(ORIGIN, e.origin);
  // The second goes quiet at 37.5 s, but waits out an interval from the first.
  TEST_ASSERT_FALSE(reach.earlyDue(37501, INTERVAL_MS, 0));
  TEST_ASSERT_FALSE(reach.earlyDue(39500, INTERVAL_MS, 0));
  TEST_ASSERT_TRUE(reach.earlyDue(39501, INTERVAL_MS, 0));
  len = reach.takeEarlySummary(39501, INTERVAL_MS, buf, sizeof(buf));
  TEST_ASSERT_TRUE(decodeReachHeader(buf, len, entries, flags));
  TEST_ASSERT_EQUAL_UINT32(1, entries);
  TEST_ASSERT_TRUE(decodeReachEntry(buf, len, 0, e));
  TEST_ASSERT_EQUAL_HEX32(ORIGIN_B, e.origin);
  // The first comes back through a relay: its early summary worked, so no
  // repeat for it. The second, still quiet, gets its one repeat.
  reach.heard(ORIGIN, fixId(3, ++seq), 300, 1, 0x77, INTERVAL_MS, 40000);
  TEST_ASSERT_FALSE(reach.earlyDue(49500, INTERVAL_MS, 0));
  TEST_ASSERT_TRUE(reach.earlyDue(49501, INTERVAL_MS, 0));
  len = reach.takeEarlySummary(49501, INTERVAL_MS, buf, sizeof(buf));
  TEST_ASSERT_TRUE(decodeReachHeader(buf, len, entries, flags));
  TEST_ASSERT_EQUAL_UINT32(1, entries);
  TEST_ASSERT_TRUE(decodeReachEntry(buf, len, 0, e));
  TEST_ASSERT_EQUAL_HEX32(ORIGIN_B, e.origin);
  TEST_ASSERT_EQUAL_UINT32(3, reach.earlySent());
}

void test_an_origin_claimed_steady_now_coming_through_relays_goes_early() {
  Reach reach;
  reach.clear();
  uint32_t seq = 0;
  uint32_t atMs = hearPositions(reach, ORIGIN, 5, 1000, 0, seq);
  uint8_t buf[233];
  TEST_ASSERT_TRUE(reach.takeSummary(atMs + 100, INTERVAL_MS, buf, sizeof(buf)) > 0);
  // One copy first through a relay can be a lost direct copy.
  atMs = hearPositions(reach, ORIGIN, 1, atMs + INTERVAL_MS, 1, seq);
  TEST_ASSERT_FALSE(reach.earlyDue(atMs + 100, INTERVAL_MS, 0));
  // Two in a row is the direct link gone.
  atMs = hearPositions(reach, ORIGIN, 1, atMs + INTERVAL_MS, 1, seq);
  TEST_ASSERT_TRUE(reach.earlyDue(atMs + 100, INTERVAL_MS, 0));
  // Heard directly again before it went: nothing to say after all.
  atMs = hearPositions(reach, ORIGIN, 1, atMs + INTERVAL_MS, 0, seq);
  TEST_ASSERT_FALSE(reach.earlyDue(atMs + 100, INTERVAL_MS, 0));
  // Lost again; this time our regular summary goes first and says it, so the
  // early one is dropped.
  atMs = hearPositions(reach, ORIGIN, 2, atMs + INTERVAL_MS, 1, seq);
  TEST_ASSERT_TRUE(reach.earlyDue(atMs + 100, INTERVAL_MS, 0));
  ReachEntry e;
  TEST_ASSERT_TRUE(summaryEntry(reach, ORIGIN, atMs + 150, e));
  TEST_ASSERT_FALSE(e.steady);
  TEST_ASSERT_FALSE(reach.earlyDue(atMs + 200, INTERVAL_MS, 0));
  TEST_ASSERT_EQUAL_UINT32(0, reach.earlySent());
}

// ---- The "le" report -----------------------------------------------------------

void test_the_le_report_carries_the_relays_skipped_and_why_the_rest_went() {
  Reach reach;
  reach.clear();
  RelayPrefs prefs;
  prefs.clear();
  RelaySkips skips;
  skips.skipped = 6;
  skips.noEvidence = 7;
  skips.stale = 8;
  skips.needed = 9;
  char out[233];
  TEST_ASSERT_TRUE(formatLoraReach(reach, prefs, skips, false, out, sizeof(out)) > 0);
  TEST_ASSERT_EQUAL_STRING("le st=0 sh=0 pg=0 pw=0 se=0 sk=6 rn=7 rs=8 rd=9", out);
  TEST_ASSERT_TRUE(formatLoraReach(reach, prefs, skips, true, out, sizeof(out)) > 0);
  TEST_ASSERT_EQUAL_STRING("{\"le\":{\"st\":0,\"sh\":0,\"pg\":0,\"pw\":0,\"se\":0,\"sk\":6,\"rn\":7,\"rs\":8,\"rd\":9}}", out);
  skips.skipped = skips.noEvidence = skips.stale = skips.needed = 0xFFFFFFFF;
  TEST_ASSERT_TRUE(formatLoraReach(reach, prefs, skips, true, out, sizeof(out)) > 0);
}

// ---- The relay decision --------------------------------------------------------

static const uint32_t SELF = 0xA00000B1;
static const uint8_t SELF_BYTE = 0xB1;
static const uint32_t FRONT = 0xA000000A;
static const uint32_t REAR = 0xA000000C;

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

// SHORT_FAST-like timing: 66 ms a packet (an unsigned position) and 10 ms
// contention slots. An early relay goes within 16 slots; an ordinary one waits
// those 16 and up to 64 more (RadioInterface::getTxDelayMsecWeighted: 2 x CWmax
// slots, then 2^CW).
static const uint32_t AIR_MS = 66;
static const uint32_t SLOT_MS = 10;
static const uint8_t HOP_LIMIT = 3;  // Meshtastic's default
// The module's figures at a 5 s interval (noteReachSummary).
static const uint32_t WE_HEAR_MS = 3 * INTERVAL_MS;
static const uint32_t MAX_SINCE_S = 3 * INTERVAL_MS / 1000;

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
          cars[r].reach.heard(cars[i].id, fix, c.atMs - nowMs, c.hops, c.relay, INTERVAL_MS, c.atMs);
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
      const size_t len = cars[i].reach.takeSummary(nowMs, INTERVAL_MS, buf, sizeof(buf));
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
    car.reach.noteSummary(reporter, buf, len, nowMs);
    ReachEntry e;
    for (size_t k = 0; decodeReachEntry(buf, len, k, e); k++) {
      const bool weHearIt = car.reach.heardWithin(e.origin, WE_HEAR_MS, nowMs);
      car.prefs.consider(e, reporter, car.id, relayByte(car.id), weHearIt, MAX_SINCE_S, HOLD_MS, nowMs);
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
  const size_t len = convoy.cars[0].reach.takeSummary(convoy.nowMs, INTERVAL_MS, buf, sizeof(buf));
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
  const uint32_t lapseAt = convoy.nowMs + HOLD_MS;
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

// ---- A ride with relays nobody needs skipped (build 43) --------------------------
//
// The same flood with the module's build 43 rules on every Touge car: each
// notes a position it may relay as Meshtastic hands it over (the module runs
// before RoutingModule), and relays it only if relayVerdict says a car it knows
// needs it. Every car keeps NodeDB's last heard per car, which is the cars it
// knows; sends its summary when told; and sends an early one when Reach says
// it is due, checked every 100 ms as the module checks every pass. The ride's
// clock stands still during a flood: cars send a share of the interval apart,
// and the ages summaries carry are the flood's own.

struct SimRider {
  uint32_t id = 0;
  int x = 0;
  bool up = true;
  bool touge = true;  // false: a stock node, positions with no identity and no summaries
  uint32_t seq = 0;
  Reach reach;
  TxPositions noted;
  RelaySkips skips;
  std::vector<uint32_t> lastHeardMs;  // NodeDB's last_heard for each rider; 0 never
  uint32_t lastEarlyMs = 0;
};

struct Ride;
struct KnownBy {
  const Ride* ride;
  size_t self;
};
static bool knownOnRide(uint32_t origin, uint32_t* cars, size_t cap, size_t& n, void* ctx);

struct Ride {
  std::vector<SimRider> riders;
  uint32_t nowMs = 1000;
  uint32_t rng = 11;
  uint32_t nextPacketId = 1;
  uint32_t positionRelays = 0;
  uint32_t summaryRelays = 0;
  // Per origin, who got its last position, and how.
  std::vector<std::vector<Copy>> lastGot;

  explicit Ride(const std::vector<int>& xs) {
    for (size_t i = 0; i < xs.size(); i++) {
      SimRider r;
      r.id = 0xA0000000u | (uint32_t)(0x11 * (i + 1));
      r.x = xs[i];
      r.reach.clear();
      r.noted.clear();
      r.lastHeardMs.assign(xs.size(), 0);
      riders.push_back(r);
    }
    lastGot.assign(riders.size(), std::vector<Copy>(riders.size()));
  }

  uint32_t random() {
    rng = rng * 1103515245u + 12345u;
    return rng >> 8;
  }
  bool hears(size_t a, size_t b) const {
    return a != b && riders[a].up && riders[b].up && abs(riders[a].x - riders[b].x) <= 1;
  }

  // One broadcast from [origin], flooded: a position when [fix] is set, a
  // summary of [len] bytes otherwise.
  std::vector<Copy> flood(size_t origin, const FixId* fix, const uint8_t* summary, size_t len) {
    const size_t n = riders.size();
    const uint32_t originId = riders[origin].id;
    const uint32_t packetId = nextPacketId++;
    std::vector<Copy> got(n);
    std::vector<uint32_t> relayAt(n, UINT32_MAX);
    std::vector<uint8_t> relayHops(n, 0);
    got[origin].got = true;
    size_t sender = origin;
    uint32_t txAt = 0;
    uint8_t hopLimit = HOP_LIMIT;
    for (;;) {
      for (size_t r = 0; r < n; r++) {
        if (r == origin || !hears(sender, r)) continue;
        if (got[r].got) {
          // Another car's copy before ours went: Meshtastic drops the queued one.
          if (relayAt[r] != UINT32_MAX && relayAt[r] >= txAt) relayAt[r] = UINT32_MAX;
          continue;
        }
        SimRider& rider = riders[r];
        got[r].got = true;
        got[r].atMs = txAt + AIR_MS;
        got[r].hops = (uint8_t)(HOP_LIMIT - hopLimit);
        got[r].relay = Convoy::relayByte(riders[sender].id);
        rider.lastHeardMs[origin] = nowMs;
        if (fix != nullptr) {
          rider.reach.heard(originId, *fix, got[r].atMs, got[r].hops, got[r].relay, INTERVAL_MS, nowMs);
        } else if (rider.touge) {
          rider.reach.noteSummary(originId, summary, len, nowMs);
        }
        if (hopLimit == 0) continue;
        bool relay = true;
        if (fix != nullptr && rider.touge) {
          rider.noted.note(originId, packetId, *fix, nowMs, &neverQueued, nullptr);
          KnownBy known = {this, r};
          const RelayVerdict verdict =
              relayVerdict(rider.noted, originId, packetId, rider.reach, &knownOnRide, &known, HOLD_MS, nowMs);
          rider.skips.note(verdict);
          relay = verdict != RelayVerdict::SKIP;
        }
        if (relay) {
          relayAt[r] = txAt + AIR_MS + 16 * SLOT_MS + (random() % 64) * SLOT_MS;
          relayHops[r] = (uint8_t)(hopLimit - 1);
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
      if (fix != nullptr) {
        positionRelays++;
      } else {
        summaryRelays++;
      }
    }
    return got;
  }

  void sendPosition(size_t i) {
    SimRider& rider = riders[i];
    FixId fix;
    fix.session = rider.touge ? (uint16_t)(i + 1) : 0;
    fix.seq = ++rider.seq;
    const uint64_t utcMs = 1790000000000ull + nowMs;
    fix.fixSec = (uint32_t)(utcMs / 1000);
    fix.fixMs = (uint16_t)(utcMs % 1000);
    lastGot[i] = flood(i, &fix, nullptr, 0);
  }

  void sendSummary(size_t i, bool early) {
    uint8_t buf[233];
    Reach& reach = riders[i].reach;
    const size_t len = early ? reach.takeEarlySummary(nowMs, INTERVAL_MS, buf, sizeof(buf))
                             : reach.takeSummary(nowMs, INTERVAL_MS, buf, sizeof(buf));
    if (len == 0) return;
    if (early) riders[i].lastEarlyMs = nowMs;
    flood(i, nullptr, buf, len);
  }

  // Every Touge car's regular summary.
  void summaries() {
    for (size_t i = 0; i < riders.size(); i++) {
      if (riders[i].up && riders[i].touge) sendSummary(i, false);
    }
  }

  void advanceTo(uint32_t atMs) {
    while ((int32_t)(atMs - nowMs) > 0) {
      nowMs += atMs - nowMs < 100 ? atMs - nowMs : 100;
      for (size_t i = 0; i < riders.size(); i++) {
        SimRider& rider = riders[i];
        if (rider.up && rider.touge && rider.reach.earlyDue(nowMs, INTERVAL_MS, random())) sendSummary(i, true);
      }
    }
  }

  // One interval: every car sends its position, a share of it apart.
  void round() {
    const uint32_t start = nowMs;
    const uint32_t share = INTERVAL_MS / (uint32_t)riders.size();
    for (size_t i = 0; i < riders.size(); i++) {
      advanceTo(start + (uint32_t)i * share);
      if (riders[i].up) sendPosition(i);
    }
    advanceTo(start + INTERVAL_MS);
  }

  bool everyoneGotEveryone() const {
    for (size_t o = 0; o < riders.size(); o++) {
      for (size_t r = 0; r < riders.size(); r++) {
        if (r == o || !riders[o].up || !riders[r].up) continue;
        if (!lastGot[o][r].got) return false;
      }
    }
    return true;
  }

  uint32_t skipped() const {
    uint32_t n = 0;
    for (size_t i = 0; i < riders.size(); i++) n += riders[i].skips.skipped;
    return n;
  }
};

// The cars [ctx]'s rider knows: every other rider heard within RIDER_DROP_MS,
// as the module reads NodeDB.
static bool knownOnRide(uint32_t origin, uint32_t* cars, size_t cap, size_t& n, void* ctx) {
  const KnownBy* by = (const KnownBy*)ctx;
  const SimRider& me = by->ride->riders[by->self];
  n = 0;
  for (size_t i = 0; i < by->ride->riders.size(); i++) {
    const SimRider& other = by->ride->riders[i];
    if (i == by->self || other.id == origin || me.lastHeardMs[i] == 0) continue;
    if ((uint32_t)(by->ride->nowMs - me.lastHeardMs[i]) >= RIDER_DROP_MS) continue;
    if (n == cap) return false;
    cars[n++] = other.id;
  }
  return true;
}

// The bench: three cars that all hear each other. Until their claims are in,
// managed flooding relays every position once (bar the very first, which
// nobody else was known to need); after the first summaries, never, and every
// car still gets every position.
void test_on_the_bench_position_relays_stop_once_the_claims_are_in() {
  Ride ride({0, 0, 0});
  for (int i = 0; i < 4; i++) ride.round();
  TEST_ASSERT_TRUE(ride.positionRelays >= 10);
  ride.summaries();
  const uint32_t relaysBefore = ride.positionRelays;
  for (int i = 0; i < 20; i++) {
    ride.round();
    TEST_ASSERT_TRUE(ride.everyoneGotEveryone());
  }
  TEST_ASSERT_EQUAL_UINT32(relaysBefore, ride.positionRelays);
  // The two cars that got each position both skipped their relay.
  TEST_ASSERT_TRUE(ride.skipped() >= 20 * 3 * 2);
  // The summaries themselves relayed as ever.
  TEST_ASSERT_TRUE(ride.summaryRelays >= 3);
}

// Missing or stale evidence keeps stock relaying: with no summaries every
// position is relayed; claims older than two and a half summary periods count
// for nothing; and a node that never claims anything, a stock one here, keeps
// every relay going.
void test_missing_or_stale_claims_keep_stock_relaying() {
  Ride none({0, 0, 0});
  for (int i = 0; i < 10; i++) none.round();
  TEST_ASSERT_TRUE(none.positionRelays >= 28);
  for (size_t i = 0; i < 3; i++) TEST_ASSERT_TRUE(none.riders[i].skips.noEvidence >= 18);

  Ride stale({0, 0, 0});
  for (int i = 0; i < 4; i++) stale.round();
  stale.summaries();
  uint32_t before = stale.positionRelays;
  for (int i = 0; i < 26; i++) stale.round();
  TEST_ASSERT_EQUAL_UINT32(before, stale.positionRelays);
  for (int i = 0; i < 8; i++) stale.round();
  before = stale.positionRelays;
  for (int i = 0; i < 5; i++) stale.round();
  TEST_ASSERT_EQUAL_UINT32(before + 5 * 3, stale.positionRelays);
  TEST_ASSERT_TRUE(stale.riders[0].skips.stale > 0);

  Ride mixed({0, 0, 0, 0});
  mixed.riders[3].touge = false;
  for (int i = 0; i < 4; i++) mixed.round();
  mixed.summaries();
  before = mixed.positionRelays;
  const uint32_t skippedBefore = mixed.skipped();
  for (int i = 0; i < 10; i++) mixed.round();
  TEST_ASSERT_EQUAL_UINT32(before + 10 * 4, mixed.positionRelays);
  TEST_ASSERT_EQUAL_UINT32(skippedBefore, mixed.skipped());
}

// A strung-out convoy: the front car, two cars side by side, the rear car. The
// middle pair hear everyone and the ends hear only the middle. Once the claims
// are in the middle cars' positions go unrelayed, since everyone hears them;
// the ends' still go through one of the pair, once each; and the far car still
// gets every car.
void test_a_strung_out_convoy_still_carries_every_origin_to_the_far_car() {
  Ride ride({0, 1, 1, 2});
  for (int i = 0; i < 4; i++) ride.round();
  ride.summaries();
  const uint32_t before = ride.positionRelays;
  for (int i = 0; i < 10; i++) {
    ride.round();
    TEST_ASSERT_TRUE(ride.everyoneGotEveryone());
    TEST_ASSERT_EQUAL_UINT8(1, ride.lastGot[0][3].hops);
    TEST_ASSERT_EQUAL_UINT8(1, ride.lastGot[3][0].hops);
  }
  TEST_ASSERT_EQUAL_UINT32(before + 10 * 2, ride.positionRelays);
}

// Three cars that all hear each other, with position relays stopped; the
// middle car drifts out of the front car's range while the third still hears
// both. The middle car goes two and a half intervals without the front, says
// so in an early summary, and the third car relays the front's very next
// position to it: back within an interval of the early summary, two positions
// missed, and no regular summary needed. The front, which lost the middle car
// too, does the same the other way.
void test_when_the_middle_car_drifts_out_of_the_fronts_range_relays_come_back_within_an_interval() {
  Ride ride({0, 1, 1});
  const size_t FRONT_CAR = 0, MIDDLE = 1, THIRD = 2;
  for (int i = 0; i < 4; i++) ride.round();
  ride.summaries();
  for (int i = 0; i < 3; i++) ride.round();
  uint32_t regularSent[3];
  for (size_t i = 0; i < 3; i++) regularSent[i] = ride.riders[i].reach.summariesSent() - ride.riders[i].reach.earlySent();

  ride.riders[MIDDLE].x = 2;
  uint32_t missed = 0;
  bool back = false;
  for (int round = 0; round < 4 && !back; round++) {
    const uint32_t frontSentMs = ride.nowMs;
    ride.round();
    const Copy& got = ride.lastGot[FRONT_CAR][MIDDLE];
    if (!got.got) {
      missed++;
      continue;
    }
    back = true;
    TEST_ASSERT_EQUAL_UINT8(1, got.hops);
    TEST_ASSERT_EQUAL_HEX8(Convoy::relayByte(ride.riders[THIRD].id), got.relay);
    TEST_ASSERT_EQUAL_UINT32(1, ride.riders[MIDDLE].reach.earlySent());
    TEST_ASSERT_TRUE(frontSentMs - ride.riders[MIDDLE].lastEarlyMs <= INTERVAL_MS);
  }
  TEST_ASSERT_TRUE(back);
  TEST_ASSERT_EQUAL_UINT32(2, missed);
  TEST_ASSERT_TRUE(ride.lastGot[MIDDLE][FRONT_CAR].got);
  TEST_ASSERT_EQUAL_UINT8(1, ride.lastGot[MIDDLE][FRONT_CAR].hops);
  TEST_ASSERT_EQUAL_UINT32(1, ride.riders[FRONT_CAR].reach.earlySent());
  for (size_t i = 0; i < 3; i++) {
    TEST_ASSERT_EQUAL_UINT32(regularSent[i], ride.riders[i].reach.summariesSent() - ride.riders[i].reach.earlySent());
  }
  // From here the two hear each other through the third car: no repeats, and
  // nothing more to say early.
  for (int i = 0; i < 5; i++) {
    ride.round();
    TEST_ASSERT_TRUE(ride.everyoneGotEveryone());
  }
  TEST_ASSERT_EQUAL_UINT32(1, ride.riders[MIDDLE].reach.earlySent());
  TEST_ASSERT_EQUAL_UINT32(1, ride.riders[FRONT_CAR].reach.earlySent());

  // Back in range: once the next summaries claim it again, the relays stop.
  ride.riders[MIDDLE].x = 1;
  for (int i = 0; i < 5; i++) ride.round();
  ride.summaries();
  const uint32_t before = ride.positionRelays;
  for (int i = 0; i < 5; i++) ride.round();
  TEST_ASSERT_EQUAL_UINT32(before, ride.positionRelays);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_a_summary_round_trips);
  RUN_TEST(test_the_steady_and_early_flags_round_trip);
  RUN_TEST(test_ages_go_in_quarter_seconds);
  RUN_TEST(test_an_entry_reads_as_the_app_shows_cars);
  RUN_TEST(test_the_newest_fix_per_origin_is_kept);
  RUN_TEST(test_a_long_list_goes_on_in_the_next_summary);
  RUN_TEST(test_an_origin_not_heard_for_4_minutes_is_forgotten);
  RUN_TEST(test_a_full_table_forgets_the_origin_heard_longest_ago);
  RUN_TEST(test_30_origins_keep_their_slots_and_claims);
  RUN_TEST(test_an_origin_is_claimed_steady_after_four_direct_positions_in_a_row);
  RUN_TEST(test_a_relayed_copy_or_a_missed_position_starts_the_count_again);
  RUN_TEST(test_a_cars_claims_come_from_its_summaries_and_age_out);
  RUN_TEST(test_only_a_build_43_summary_from_a_car_we_hear_makes_a_claim);
  RUN_TEST(test_an_early_summary_changes_what_it_lists_and_leaves_the_clock);
  RUN_TEST(test_claims_from_a_car_that_stopped_summarising_never_come_back);
  RUN_TEST(test_a_relay_is_skipped_only_when_every_known_car_claims_the_origin);
  RUN_TEST(test_the_relay_veto_never_touches_anything_but_a_touge_position);
  RUN_TEST(test_an_origin_claimed_steady_and_quiet_two_and_a_half_intervals_goes_early);
  RUN_TEST(test_early_summaries_keep_an_interval_apart_and_stop_once_the_origin_is_heard);
  RUN_TEST(test_an_origin_claimed_steady_now_coming_through_relays_goes_early);
  RUN_TEST(test_the_le_report_carries_the_relays_skipped_and_why_the_rest_went);
  RUN_TEST(test_a_summary_that_got_the_origin_through_us_grants_early_relaying);
  RUN_TEST(test_only_evidence_of_delivery_through_us_counts);
  RUN_TEST(test_worse_delivery_through_us_ends_it_at_once);
  RUN_TEST(test_a_full_set_of_grants_gives_up_the_one_lapsing_soonest);
  RUN_TEST(test_a_strung_out_convoy_reports_which_origins_reach_the_far_end);
  RUN_TEST(test_the_cars_that_carry_an_origin_along_the_convoy_are_preferred);
  RUN_TEST(test_the_rear_car_keeps_arriving_at_the_front_when_the_preferred_relay_drops_out);
  RUN_TEST(test_stale_evidence_returns_every_car_to_ordinary_forwarding);
  RUN_TEST(test_on_the_bench_position_relays_stop_once_the_claims_are_in);
  RUN_TEST(test_missing_or_stale_claims_keep_stock_relaying);
  RUN_TEST(test_a_strung_out_convoy_still_carries_every_origin_to_the_far_car);
  RUN_TEST(test_when_the_middle_car_drifts_out_of_the_fronts_range_relays_come_back_within_an_interval);
  return UNITY_END();
}
