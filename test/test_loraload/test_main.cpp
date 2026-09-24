// Host tests for the LoRa lane's measured load (SCALE-PLAN 5d): the interval
// the busy share allows, the send grid, and the counts the radio reports.
//
// The ride's channel is modelled as positions and their relays: each car's
// position and [relays] relayed copies of it, [airtimeMs] each, every interval.
// The rule only ever sees the busy share that produces, as it would on the air.

#include <unity.h>
#include <string.h>
#include <vector>
#include "loraload.h"

using namespace touge;

void setUp() {}
void tearDown() {}

// A moving car's unsigned position, 67 bytes on the air from build 43: 66 ms on
// SHORT_FAST, 764 on LONG_FAST (MeshProto.airtimeMs). Signed it was 114 and
// 1255.
static const uint32_t SHORT_FAST_MS = 66;
static const uint32_t LONG_FAST_MS = 764;

static uint32_t busyPermille(uint32_t cars, uint32_t relays, uint32_t airtimeMs, uint32_t intervalMs) {
  const uint64_t busy = (uint64_t)cars * (1 + relays) * airtimeMs * 1000 / intervalMs;
  return busy > 1000 ? 1000 : (uint32_t)busy;
}

// Judges once every LORA_JUDGE_MS for [steps] steps against the model above,
// [nowMs] running on.
static void ride(LoraLoad& load, uint32_t& nowMs, uint32_t cars, uint32_t relays, uint32_t airtimeMs, uint32_t steps) {
  for (uint32_t i = 0; i < steps; i++) {
    nowMs += LORA_JUDGE_MS;
    load.judge(busyPermille(cars, relays, airtimeMs, load.intervalMs()), nowMs);
  }
}

// ---- The interval -------------------------------------------------------------

void test_the_interval_holds_5_s_while_the_air_allows() {
  LoraLoad load;
  load.reset();
  TEST_ASSERT_EQUAL_UINT32(LORA_TARGET_MS, load.intervalMs());
  // Three cars, every position relayed twice: about a tenth of the channel.
  uint32_t nowMs = 0;
  ride(load, nowMs, 3, 2, SHORT_FAST_MS, 20);
  TEST_ASSERT_EQUAL_UINT32(LORA_TARGET_MS, load.intervalMs());
  TEST_ASSERT_FALSE(load.saturated());
  // At the target is still 5 s.
  load.judge(LORA_BUSY_TARGET_PERMILLE, nowMs + LORA_JUDGE_MS);
  TEST_ASSERT_EQUAL_UINT32(LORA_TARGET_MS, load.intervalMs());
}

void test_a_busy_channel_stretches_the_interval_only_as_far_as_it_needs() {
  LoraLoad load;
  load.reset();
  // Half again over the target at 5 s: 7.5 s would bring it back, whole seconds up.
  load.judge(LORA_BUSY_TARGET_PERMILLE * 3 / 2, 1000);
  TEST_ASSERT_EQUAL_UINT32(8000, load.intervalMs());
  // At the target: stays.
  load.judge(LORA_BUSY_TARGET_PERMILLE, 1000 + LORA_JUDGE_MS);
  TEST_ASSERT_EQUAL_UINT32(8000, load.intervalMs());

  // Twenty-five cars on SHORT_FAST, every position relayed twice, settle where
  // the air sits under the target: 20 s, 24.7 % busy.
  LoraLoad ride25;
  ride25.reset();
  uint32_t nowMs = 0;
  ride(ride25, nowMs, 25, 2, SHORT_FAST_MS, 10);
  TEST_ASSERT_EQUAL_UINT32(20000, ride25.intervalMs());
  TEST_ASSERT_TRUE(busyPermille(25, 2, SHORT_FAST_MS, ride25.intervalMs()) <= LORA_BUSY_TARGET_PERMILLE);
  TEST_ASSERT_FALSE(ride25.saturated());
}

void test_a_step_at_most_doubles_the_interval() {
  LoraLoad load;
  load.reset();
  load.judge(1000, 1000);
  TEST_ASSERT_EQUAL_UINT32(10000, load.intervalMs());
  load.judge(1000, 1000 + LORA_JUDGE_MS);
  TEST_ASSERT_EQUAL_UINT32(LORA_MAX_MS, load.intervalMs());
}

void test_the_interval_is_judged_no_more_than_every_30_s() {
  LoraLoad load;
  load.reset();
  load.judge(LORA_BUSY_TARGET_PERMILLE * 2, 1000);
  TEST_ASSERT_EQUAL_UINT32(10000, load.intervalMs());
  load.judge(LORA_BUSY_TARGET_PERMILLE * 2, 1000 + LORA_JUDGE_MS - 1);
  TEST_ASSERT_EQUAL_UINT32(10000, load.intervalMs());
  load.judge(LORA_BUSY_TARGET_PERMILLE * 2, 1000 + LORA_JUDGE_MS);
  TEST_ASSERT_EQUAL_UINT32(LORA_MAX_MS, load.intervalMs());
}

void test_it_comes_back_down_a_quarter_a_step_once_the_air_clears() {
  LoraLoad load;
  load.reset();
  load.judge(1000, 1000);
  load.judge(1000, 1000 + LORA_JUDGE_MS);
  TEST_ASSERT_EQUAL_UINT32(LORA_MAX_MS, load.intervalMs());
  const uint32_t expected[] = {15000, 12000, 9000, 7000, 6000, 5000, 5000};
  for (size_t i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
    load.judge(50, 1000 + (2 + i) * LORA_JUDGE_MS);
    TEST_ASSERT_EQUAL_UINT32(expected[i], load.intervalMs());
  }
  // Between three quarters of the target and the target it holds, so it does
  // not hunt around the line.
  load.judge(LORA_BUSY_TARGET_PERMILLE * 3 / 4 + 10, 1000 + 20 * LORA_JUDGE_MS);
  TEST_ASSERT_EQUAL_UINT32(5000, load.intervalMs());
}

// SCALE-PLAN 5d: an overloaded channel is reported as such, not silently overrun.
void test_an_overloaded_channel_is_reported_not_overrun() {
  // Thirty cars on SHORT_FAST: even 20 s leaves the air over the target.
  LoraLoad load;
  load.reset();
  uint32_t nowMs = 0;
  ride(load, nowMs, 30, 2, SHORT_FAST_MS, 2);
  TEST_ASSERT_EQUAL_UINT32(LORA_MAX_MS, load.intervalMs());
  // Not yet measured at the cap.
  TEST_ASSERT_FALSE(load.saturated());
  ride(load, nowMs, 30, 2, SHORT_FAST_MS, 3);
  TEST_ASSERT_TRUE(load.saturated());
  const LoraWindow w = load.takeWindow(true, busyPermille(30, 2, SHORT_FAST_MS, LORA_MAX_MS), 12);
  TEST_ASSERT_EQUAL_UINT32(LORA_MAX_MS, w.intervalMs);
  TEST_ASSERT_TRUE((w.overloaded & LORA_OVER_CHANNEL) != 0);
  TEST_ASSERT_TRUE(w.busyPermille > LORA_BUSY_TARGET_PERMILLE);

  // Three cars on LONG_FAST are the same story.
  LoraLoad slow;
  slow.reset();
  uint32_t slowMs = 0;
  ride(slow, slowMs, 3, 2, LONG_FAST_MS, 6);
  TEST_ASSERT_TRUE(slow.saturated());

  // And once the ride thins out, it says so no longer.
  ride(load, nowMs, 10, 2, SHORT_FAST_MS, 1);
  TEST_ASSERT_FALSE(load.saturated());
}

// ---- What went out -----------------------------------------------------------

// SCALE-PLAN 5d: the reported interval matches what goes out on the air. A car
// sends on the grid, each position waits a while in the TX queue, and la is the
// mean of the last four gaps between them on the air.
void test_the_reported_interval_matches_what_goes_out() {
  LoraLoad load;
  load.reset();
  const uint32_t cars = 5, rank = 2;
  const uint64_t utcStart = 1790000000000ull;
  uint32_t rng = 12345;
  auto next = [&rng]() {
    rng = rng * 1103515245u + 12345u;
    return rng >> 8;
  };

  std::vector<uint64_t> onAir;
  uint64_t dueAt = 0;
  bool first = true;
  uint64_t pendingAt = 0;
  bool pending = false;
  uint32_t checked = 0;
  for (uint32_t ms = 0; ms < 180000; ms += 10) {
    const uint64_t utc = utcStart + 1234 + ms;
    if (first || utc >= dueAt) {
      const uint32_t interval = load.intervalMs();
      dueAt = nextLoraSendAt(utc + interval / 2, interval, rank, cars, next());
      first = false;
      load.ownQueued(false);
      pending = true;
      pendingAt = utc + 20 + next() % 380;
    }
    if (pending && utc >= pendingAt) {
      load.sent(LoraTx::OWN_POSITION, SHORT_FAST_MS, (uint32_t)(utc % 400), false, (uint32_t)utc);
      onAir.push_back(utc);
      pending = false;
    }
    if (ms % 5000 == 0) {
      const LoraWindow w = load.takeWindow(true, 100, 5);
      TEST_ASSERT_EQUAL_UINT32(LORA_TARGET_MS, w.intervalMs);
      if (onAir.size() >= 5) {
        const size_t n = onAir.size();
        TEST_ASSERT_EQUAL_UINT32((uint32_t)((onAir[n - 1] - onAir[n - 5]) / 4), w.gapMeanMs);
        // What goes out is the interval, give or take the jitter and the queue.
        TEST_ASSERT_TRUE(w.gapMeanMs + 1000 >= w.intervalMs && w.gapMeanMs <= w.intervalMs + 1000);
        checked++;
      }
      TEST_ASSERT_EQUAL(0, w.overloaded);
    }
  }
  TEST_ASSERT_TRUE(checked > 25);
  TEST_ASSERT_EQUAL_UINT32(onAir.size(), load.counts().ownTx);
  TEST_ASSERT_EQUAL_UINT32(onAir.size() * SHORT_FAST_MS, load.counts().ownAirMs);
}

void test_the_longest_gap_and_waits_are_per_window() {
  LoraLoad load;
  load.reset();
  load.sent(LoraTx::OWN_POSITION, 58, 40, false, 1000);
  load.sent(LoraTx::OWN_POSITION, 58, 300, false, 6000);
  load.sent(LoraTx::RELAY, 58, 900, false, 6500);
  load.sent(LoraTx::RELAY, 58, LORA_WAIT_UNKNOWN, false, 6600);
  load.queueDepth(3);
  load.queueDepth(1);
  LoraWindow w = load.takeWindow(true, 0, 0);
  TEST_ASSERT_EQUAL_UINT32(5000, w.gapMeanMs);
  TEST_ASSERT_EQUAL_UINT32(5000, w.gapMaxMs);
  TEST_ASSERT_EQUAL_UINT32(300, w.ownWaitMaxMs);
  TEST_ASSERT_EQUAL_UINT32(900, w.relayWaitMaxMs);
  TEST_ASSERT_EQUAL_UINT32(3, w.depthMax);
  // A window with nothing sent keeps the running mean and reports no gap.
  w = load.takeWindow(true, 0, 0);
  TEST_ASSERT_EQUAL_UINT32(5000, w.gapMeanMs);
  TEST_ASSERT_EQUAL_UINT32(0, w.gapMaxMs);
  TEST_ASSERT_EQUAL_UINT32(0, w.ownWaitMaxMs);
  TEST_ASSERT_EQUAL_UINT32(0, w.relayWaitMaxMs);
  TEST_ASSERT_EQUAL_UINT32(0, w.depthMax);
}

void test_a_pause_in_sending_is_not_a_gap() {
  LoraLoad load;
  load.reset();
  load.sent(LoraTx::OWN_POSITION, 58, 0, false, 1000);
  load.sent(LoraTx::OWN_POSITION, 58, 0, false, 6000);
  // The phone went away for ten minutes: the radio sent nothing.
  LoraWindow w = load.takeWindow(false, 0, 0);
  TEST_ASSERT_EQUAL_UINT32(0, w.intervalMs);
  load.sent(LoraTx::OWN_POSITION, 58, 0, false, 606000);
  w = load.takeWindow(true, 0, 0);
  TEST_ASSERT_EQUAL_UINT32(0, w.gapMeanMs);
  TEST_ASSERT_EQUAL_UINT32(0, w.gapMaxMs);
}

// Our position still queued when the next is due: counted and flagged for the
// window it happened in. Ahead of relays from build 43, it should not happen.
void test_a_late_own_position_is_counted_and_flagged() {
  LoraLoad load;
  load.reset();
  load.ownQueued(false);
  load.ownQueued(true);
  LoraWindow w = load.takeWindow(true, 100, 5);
  TEST_ASSERT_EQUAL_UINT32(1, load.counts().ownLate);
  TEST_ASSERT_EQUAL(LORA_OVER_OWN_LATE, w.overloaded);
  w = load.takeWindow(true, 100, 5);
  TEST_ASSERT_EQUAL(0, w.overloaded);
}

void test_airtime_is_counted_by_kind() {
  LoraLoad load;
  load.reset();
  load.sent(LoraTx::OWN_POSITION, 58, 0, false, 100);
  load.sent(LoraTx::OWN_SUMMARY, 150, 0, false, 200);
  load.sent(LoraTx::OWN_OTHER, 70, 0, false, 300);
  load.sent(LoraTx::RELAY, 58, 0, true, 400);
  load.sent(LoraTx::RELAY, 60, 0, false, 500);
  const LoraTxCounts& c = load.counts();
  TEST_ASSERT_EQUAL_UINT32(1, c.ownTx);
  TEST_ASSERT_EQUAL_UINT32(58, c.ownAirMs);
  TEST_ASSERT_EQUAL_UINT32(150, c.summaryAirMs);
  TEST_ASSERT_EQUAL_UINT32(70, c.otherAirMs);
  TEST_ASSERT_EQUAL_UINT32(2, c.relayTx);
  TEST_ASSERT_EQUAL_UINT32(118, c.relayAirMs);
  TEST_ASSERT_EQUAL_UINT32(1, c.relayEarly);
}

// ---- The send grid -----------------------------------------------------------

void test_the_grid_spreads_cars_by_rank() {
  const uint32_t interval = 5000, cars = 5;
  const uint64_t after = 1790000000123ull;
  std::vector<uint32_t> phases;
  for (uint32_t rank = 0; rank < cars; rank++) {
    const uint64_t at = nextLoraSendAt(after, interval, rank, cars, 0);
    TEST_ASSERT_TRUE(at > after && at - after <= interval);
    phases.push_back((uint32_t)(at % interval));
  }
  for (uint32_t rank = 0; rank < cars; rank++) TEST_ASSERT_EQUAL_UINT32(rank * 1000, phases[rank]);
}

void test_jitter_stays_inside_the_cars_own_share() {
  const uint32_t interval = 12000, cars = 7, share = interval / cars;
  uint32_t rng = 99;
  for (uint32_t i = 0; i < 2000; i++) {
    rng = rng * 1103515245u + 12345u;
    const uint32_t rank = i % cars;
    const uint64_t after = 1790000000000ull + (uint64_t)i * 3517;
    const uint64_t at = nextLoraSendAt(after, interval, rank, cars, rng);
    TEST_ASSERT_TRUE(at > after && at - after <= interval);
    const uint32_t phase = (uint32_t)(at % interval);
    TEST_ASSERT_TRUE(phase >= rank * share && phase < rank * share + share / 2);
  }
  // One car alone, or a clock with nothing to go on, still gets a time.
  TEST_ASSERT_TRUE(nextLoraSendAt(10, 5000, 0, 1, 12345) > 10);
  TEST_ASSERT_TRUE(nextLoraSendAt(10, 5000, 3, 0, 0) > 10);
}

// ---- The reports ---------------------------------------------------------------

void test_the_reports_use_the_same_keys_on_serial_and_to_the_phone() {
  LoraWindow w;
  w.intervalMs = 5000;
  w.gapMeanMs = 5012;
  w.gapMaxMs = 5400;
  w.busyPermille = 125;
  w.txPermille = 31;
  w.overloaded = 0;
  w.ownWaitMaxMs = 40;
  w.relayWaitMaxMs = 900;
  w.depthMax = 3;
  w.originsHeard = 4;
  w.preferredFor = 1;
  char out[240];
  TEST_ASSERT_TRUE(formatLoraWindow(w, false, out, sizeof(out)) > 0);
  TEST_ASSERT_EQUAL_STRING("ll li=5000 la=5012 lx=5400 cu=125 tu=31 ov=0 ow=40 rw=900 qm=3 oh=4 pf=1", out);
  TEST_ASSERT_TRUE(formatLoraWindow(w, true, out, sizeof(out)) > 0);
  TEST_ASSERT_EQUAL_STRING(
      "{\"ll\":{\"li\":5000,\"la\":5012,\"lx\":5400,\"cu\":125,\"tu\":31,\"ov\":0,\"ow\":40,\"rw\":900,\"qm\":3,\"oh\":4,"
      "\"pf\":1}}",
      out);

  LoraTxCounts c;
  c.ownTx = 1;
  c.ownAirMs = 2;
  c.relayTx = 3;
  c.relayAirMs = 4;
  c.relayEarly = 5;
  c.summaryAirMs = 6;
  c.otherAirMs = 7;
  c.dropped = 8;
  c.cancelled = 9;
  c.replaced = 10;
  c.refused = 11;
  c.ownLate = 12;
  TEST_ASSERT_TRUE(formatLoraTx(c, false, out, sizeof(out)) > 0);
  TEST_ASSERT_EQUAL_STRING("lt ot=1 oa=2 rt=3 ra=4 re=5 sa=6 xa=7 dr=8 cn=9 rp=10 rf=11 os=12", out);
}

// Every counter at its largest still fits one Meshtastic payload (233 bytes).
void test_the_reports_fit_one_payload_at_their_largest() {
  LoraWindow w;
  w.intervalMs = w.gapMeanMs = w.gapMaxMs = w.busyPermille = w.txPermille = 0xFFFFFFFF;
  w.ownWaitMaxMs = w.relayWaitMaxMs = w.depthMax = w.originsHeard = w.preferredFor = 0xFFFFFFFF;
  w.overloaded = 0xFF;
  LoraTxCounts c;
  c.ownTx = c.ownAirMs = c.relayTx = c.relayAirMs = c.relayEarly = c.summaryAirMs = 0xFFFFFFFF;
  c.otherAirMs = c.dropped = c.cancelled = c.replaced = c.refused = c.ownLate = 0xFFFFFFFF;
  char out[233];
  TEST_ASSERT_TRUE(formatLoraWindow(w, true, out, sizeof(out)) > 0);
  TEST_ASSERT_TRUE(formatLoraTx(c, true, out, sizeof(out)) > 0);
  // Too small a buffer says so rather than cutting a report short.
  char tiny[20];
  TEST_ASSERT_EQUAL_UINT32(0, formatLoraTx(c, true, tiny, sizeof(tiny)));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_the_interval_holds_5_s_while_the_air_allows);
  RUN_TEST(test_a_busy_channel_stretches_the_interval_only_as_far_as_it_needs);
  RUN_TEST(test_a_step_at_most_doubles_the_interval);
  RUN_TEST(test_the_interval_is_judged_no_more_than_every_30_s);
  RUN_TEST(test_it_comes_back_down_a_quarter_a_step_once_the_air_clears);
  RUN_TEST(test_an_overloaded_channel_is_reported_not_overrun);
  RUN_TEST(test_the_reported_interval_matches_what_goes_out);
  RUN_TEST(test_the_longest_gap_and_waits_are_per_window);
  RUN_TEST(test_a_pause_in_sending_is_not_a_gap);
  RUN_TEST(test_a_late_own_position_is_counted_and_flagged);
  RUN_TEST(test_airtime_is_counted_by_kind);
  RUN_TEST(test_the_grid_spreads_cars_by_rank);
  RUN_TEST(test_jitter_stays_inside_the_cars_own_share);
  RUN_TEST(test_the_reports_use_the_same_keys_on_serial_and_to_the_phone);
  RUN_TEST(test_the_reports_fit_one_payload_at_their_largest);
  return UNITY_END();
}
