// The realistic ride simulator (plan 2B): cars running the firmware's own
// modules over a modelled 2.4 GHz channel. See ridesim.h for the model and
// ridesim_policy.h for switching a build's forwarding rules.
//
// The default run is short: the hardware facts the sim must reproduce (three
// bench radios put every voice frame on the air three times; positions keep
// flowing), the broad shape of the build 50 baseline, what build 51's defaults
// do against 50, and checks that the sim runs forward.h's code and reads what
// it relies on. Built with -D TOUGE_RIDESIM_REPORT=1 it also prints the tables
// behind plan 2B's Results (ridesim_report.h; run the test verbose to see them).

#include <unity.h>
#include <chrono>
#include <stdio.h>
#include <string.h>
#include <vector>
#include "ridesim.h"

using namespace ridesim;

void setUp() {}
void tearDown() {}

// ---- Scenarios -----------------------------------------------------------------

static Config scenario(Layout layout, double p, bool talker, uint32_t measureMs, uint32_t seed) {
  Config c;
  c.layout = layout;
  c.cars = layout == Layout::BENCH ? 3 : 25;
  c.p = p;
  c.talker = talker;
  c.measureMs = measureMs;
  c.seed = seed;
  // The least warm-up; runAll then waits for one clock (ridesim.h, SYNC_HOLD_MS).
  c.warmupMs = layout == Layout::BENCH ? 8000 : 15000;
  return c;
}

static Result timed(const Config& c) {
  const auto t0 = std::chrono::steady_clock::now();
  Result r = simulate(c);
  r.wallMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return r;
}

static double onTimePct(const Result& r, size_t car) {
  const CarStats& c = r.cars[car];
  return c.expected ? 100.0 * c.onTime / c.expected : 0;
}

// On-time voice at the listeners within `maxHops` of the talker: the worst and the median.
static void voiceWithin(const Result& r, uint8_t maxHops, double& worst, double& median) {
  std::vector<double> pct;
  for (size_t j = 0; j < r.cars.size(); j++) {
    const CarStats& c = r.cars[j];
    if (j == r.talker || c.fromTalker == 0 || c.fromTalker > maxHops || c.expected == 0) continue;
    pct.push_back(onTimePct(r, j));
  }
  worst = pct.empty() ? 0 : *std::min_element(pct.begin(), pct.end());
  median = percentile(pct, 0.5);
}

// Mean latency of the frames delivered, over the listeners within 3 hops.
static double voiceLatencyMs(const Result& r) {
  double sum = 0;
  uint32_t cars = 0;
  for (size_t j = 0; j < r.cars.size(); j++) {
    const CarStats& c = r.cars[j];
    if (j == r.talker || c.fromTalker == 0 || c.fromTalker > 3 || c.delivered == 0) continue;
    sum += c.latSumMs / c.delivered;
    cars++;
  }
  return cars ? sum / cars : 0;
}

static double posPct(const Result& r, size_t bucket) {
  return r.posSent[bucket] > 0 ? 100.0 * r.posGot[bucket] / r.posSent[bucket] : 0;
}

static double onTimeOver(const Result& r, size_t from, size_t to) {
  double sum = 0;
  for (size_t j = from; j <= to && j < r.cars.size(); j++) sum += onTimePct(r, j);
  return sum / (to - from + 1);
}

static void report(const char* what, const Result& r) {
  char line[240];
  snprintf(line, sizeof(line),
           "%s: pos %.0f/%.0f/%.0f %% (1/2/3 hops), age %.0f ms, lease copies %.2f, voice copies %.2f, air %.0f %%, "
           "one clock at %.1f s, %.2f s wall",
           what, posPct(r, 0), posPct(r, 1), posPct(r, 2), r.ageN ? r.ageSumMs / r.ageN : 0, r.leaseCopies,
           r.voiceCopies, r.airPctTotal(), r.syncedAtMs / 1000.0, r.wallMs / 1000);
  TEST_MESSAGE(line);
}

// ---- The model's own numbers ------------------------------------------------------

void test_airtime_is_what_the_plan_assumes() {
  Config c = scenario(Layout::BENCH, 0, false, 1000, 1);
  World w(c);
  // Plan 2A: a position about 4.2 ms at LR 250k, a full voice frame with prev
  // about 9 ms on the air.
  TEST_ASSERT_UINT32_WITHIN(300, 4400, w.airtimeUs(89));
  TEST_ASSERT_UINT32_WITHIN(400, 9400, w.airtimeUs(249));
  c.phy = rate1M();
  World fast(c);
  TEST_ASSERT_UINT32_WITHIN(100, 1250, fast.airtimeUs(89));
}

void test_the_default_policy_is_build_50() {
  const Policy p;
  TEST_ASSERT_TRUE(p.build == Build::B50);
  TEST_ASSERT_EQUAL_UINT8(2, adapt::originHops(p, FRAME_POSITION));
  TEST_ASSERT_EQUAL_UINT8(2, adapt::originHops(p, FRAME_VOICE));
  TEST_ASSERT_EQUAL_UINT8(3, p.voiceSuppressAfter);
  // mesh.cpp's forwardDelayMs as build 50 shipped it: -95 to -40 dBm onto the
  // spread, weakest first, and a 0-3 ms tie.
  TEST_ASSERT_EQUAL_UINT32(0, b50::forwardDelayMs(-95, 120, 0));
  TEST_ASSERT_EQUAL_UINT32(3, b50::forwardDelayMs(-100, 120, 7));
  TEST_ASSERT_EQUAL_UINT32(31, b50::forwardDelayMs(-30, 30, 5));
  TEST_ASSERT_EQUAL_UINT32(58, b50::forwardDelayMs(-67, 110, 2));
  // From the pass's time, three copies for every type, and never a skip.
  Frame f;
  f.type = FRAME_POSITION;
  f.hops = 2;
  f.len = 60;
  FastRelay relay;
  relay.reset();
  Rider riders[MAX_RIDERS];
  const ForwardPlan plan = adapt::planForward(p, f, relay, 1, riders, 1000, 1004, -95, 3, 0);
  TEST_ASSERT_FALSE(plan.skip);
  TEST_ASSERT_EQUAL_UINT32(1004, plan.dueMs);
  TEST_ASSERT_EQUAL_UINT8(3, plan.suppressAfter);
}

// ---- Hardware facts ---------------------------------------------------------------

// The plan's bench observation: three radios a foot apart put every voice frame
// on the air three times. At SUPPRESS_AFTER 3 that holds whatever the timing: a
// listener counts at most the original and the other listener's forward before
// its own goes.
void test_three_bench_radios_put_every_voice_frame_on_the_air_three_times() {
  const Result r = timed(scenario(Layout::BENCH, 0, true, 10000, 3));
  report("bench p=0", r);
  TEST_ASSERT_GREATER_THAN_UINT32(150, r.voiceAttempted);
  TEST_ASSERT_EQUAL_UINT32(0, r.voiceRefused);
  TEST_ASSERT_TRUE_MESSAGE(r.voiceCopies > 2.9 && r.voiceCopies <= 3.0, "three copies of every voice frame");
  // A frame now and then meets a lease beacon ending its backoff in the same slot.
  for (size_t j = 0; j < r.cars.size(); j++)
    if (j != r.talker) TEST_ASSERT_TRUE(onTimePct(r, j) >= 98);
}

// With loss a listener that missed the original still forwards the other's
// copy, so it stays near three.
void test_with_loss_it_stays_near_three() {
  const Result r = timed(scenario(Layout::BENCH, 0.2, true, 10000, 4));
  report("bench p=0.2", r);
  TEST_ASSERT_TRUE_MESSAGE(r.voiceCopies > 2.6 && r.voiceCopies <= 3.0, "copies at p = 0.2");
}

// Two cars: on 50 the listener forwards every frame for nobody. 51 skips a
// forward whose origin is the only other car on the ride (1C.2's two-car rule).
void test_two_cars_forward_every_voice_frame_on_50_and_none_on_51() {
  Config c = scenario(Layout::BENCH, 0, true, 8000, 5);
  c.cars = 2;
  const Result r50 = timed(c);
  c.policy = build51();
  const Result r51 = timed(c);
  TEST_ASSERT_TRUE(r50.voiceCopies > 1.95 && r50.voiceCopies <= 2.0);
  TEST_ASSERT_TRUE(r51.voiceCopies >= 1.0 && r51.voiceCopies < 1.05);
  TEST_ASSERT_TRUE(onTimePct(r51, 1 - r51.talker) >= 98);
}

void test_positions_keep_flowing_on_a_lossy_bench_with_no_talker() {
  const Result r = timed(scenario(Layout::BENCH, 0.4, false, 30000, 6));
  report("bench p=0.4 no talker", r);
  TEST_ASSERT_EQUAL(3, r.leased);
  TEST_ASSERT_EQUAL(1, r.references);
  // 60 % direct, and a listener that missed it gets a forward about a third of
  // the time: 74 % over a long run.
  TEST_ASSERT_TRUE(posPct(r, 0) > 60);
  TEST_ASSERT_TRUE(r.ageSumMs / r.ageN < 2000);
  TEST_ASSERT_EQUAL_UINT32(0, r.leaseClash);
}

// ---- The build 50 baseline's shape --------------------------------------------------

// At LR 250 kbit/s the 24 hearers' forwards saturate a car park: the weakest
// hearers' waits fall within an airtime of each other, each commits its copy to
// the radio before it hears another's, and colliding copies go uncounted. The
// talker is heard, badly, everywhere.
void test_a_car_park_saturates_on_the_build_50_flood() {
  const Result r = timed(scenario(Layout::PARK, 0.2, true, 15000, 7));
  report("park p=0.2", r);
  double worst = 0, median = 0;
  voiceWithin(r, 3, worst, median);
  TEST_ASSERT_TRUE(r.voiceCopies >= 6 && r.leaseCopies >= 6);
  TEST_ASSERT_TRUE(r.airPctTotal() > 150);
  TEST_ASSERT_TRUE(worst > 5 && median < 50);
  TEST_ASSERT_TRUE(posPct(r, 0) > 30 && posPct(r, 0) < 75);
  TEST_ASSERT_TRUE(r.syncedAtMs > 0);
}

// How much of the talker the saturated park gets through is the MAC's doing, so
// this pins it. A waiting car's backoff is frozen at the slots it has not
// counted, as 802.11 DCF does. Taking 15 us more off it every busy period served
// the oldest frame first: 46 % median on time and 148 ms. 24 seeds of 20 s read
// 27.1 % (SD 3.9) and 256 ms (SD 28); the bounds are three SDs of a mean of
// three either side.
void test_the_mac_holds_park_voice_where_dcf_puts_it() {
  double median = 0, latency = 0;
  for (uint32_t seed = 30; seed < 33; seed++) {
    Config c = scenario(Layout::PARK, 0.2, true, 20000, seed);
    const Result r = simulate(c);
    double worst = 0, m = 0;
    voiceWithin(r, 3, worst, m);
    median += m / 3;
    latency += voiceLatencyMs(r) / 3;
  }
  char line[120];
  snprintf(line, sizeof(line), "park p=0.2 on 50: voice median %.1f %% on time, latency %.0f ms", median, latency);
  TEST_MESSAGE(line);
  TEST_ASSERT_TRUE(median > 20 && median < 34);
  TEST_ASSERT_TRUE(latency > 205 && latency < 305);
}

// The same park at 1 Mbit/s: a quarter of the airtime, and the flood fits.
void test_the_same_park_at_1_mbit_carries_the_talker() {
  Config c = scenario(Layout::PARK, 0.2, true, 15000, 7);
  c.phy = rate1M();
  const Result r = timed(c);
  report("park p=0.2 1M", r);
  double worst = 0, median = 0;
  voiceWithin(r, 3, worst, median);
  TEST_ASSERT_TRUE(worst > 95);
  TEST_ASSERT_TRUE(r.voiceCopies >= 4);
  TEST_ASSERT_TRUE(posPct(r, 0) > 95);
}

void test_a_line_carries_the_talker_three_transmissions_and_no_further() {
  const Result r = timed(scenario(Layout::LINE, 0.2, true, 15000, 8));
  report("line p=0.2", r);
  // The talker's direct range hears most of it.
  for (size_t j = 1; j <= 3; j++) TEST_ASSERT_TRUE(onTimePct(r, j) > 50);
  // FAST_HOPS 2 is three transmissions of about four cars each: the far part of
  // a 25-car line hears next to nothing (the plan's case for VOICE_HOPS).
  uint32_t far = 0;
  for (size_t j = 17; j < 25; j++) far += r.cars[j].delivered;
  TEST_ASSERT_TRUE(far <= 10);
  TEST_ASSERT_TRUE(r.voiceCopies > 3 && r.voiceCopies < 7);
  TEST_ASSERT_TRUE(posPct(r, 0) > 80);
  TEST_ASSERT_TRUE(posPct(r, 1) > 40 && posPct(r, 1) < 90);
  TEST_ASSERT_EQUAL(25, r.leased);
}

// ---- The sim itself -----------------------------------------------------------------

void test_the_same_seed_gives_the_same_ride() {
  const Config c = scenario(Layout::BENCH, 0.4, true, 5000, 9);
  const Result a = simulate(c);
  const Result b = simulate(c);
  TEST_ASSERT_EQUAL_UINT32(a.txCount[TX_FWD_VOICE], b.txCount[TX_FWD_VOICE]);
  TEST_ASSERT_EQUAL_UINT32(a.randomLoss, b.randomLoss);
  TEST_ASSERT_TRUE(a.ageSumMs == b.ageSumMs);
  for (size_t j = 0; j < a.cars.size(); j++) TEST_ASSERT_EQUAL_UINT32(a.cars[j].onTime, b.cars[j].onTime);
}

// ---- Build 51: the module's forward code -----------------------------------------------

// build51() is the firmware's constants, and the sweeps' stand-ins for them
// agree with the firmware's own functions wherever a sweep leaves a value alone.
void test_build_51_runs_the_firmware_forward_code() {
  const Policy p = build51();
  TEST_ASSERT_TRUE(p.build == Build::B51);
  TEST_ASSERT_EQUAL_UINT8(FAST_HOPS, adapt::originHops(p, FRAME_POSITION));
  TEST_ASSERT_EQUAL_UINT8(VOICE_HOPS, adapt::originHops(p, FRAME_VOICE));
  TEST_ASSERT_EQUAL_UINT8(VOICE_SUPPRESS_AFTER, p.voiceSuppressAfter);
  TEST_ASSERT_EQUAL_UINT32(FORWARD_TIE_STEPS, p.tieSteps);
  TEST_ASSERT_TRUE(adapt::firmwareSkipRule(p));

  // The sweep's forward delay, worked out on its own, is mesh.cpp's at the
  // firmware's tie count, and mesh.cpp's with no tie at one step.
  const size_t lens[] = {89, 129, FRAME_MAX};
  const uint32_t spreads[] = {0, 30, 80, 120};
  for (int16_t rssi = -100; rssi <= -30; rssi += 7)
    for (uint32_t spread : spreads)
      for (size_t len : lens)
        for (uint32_t tie = 0; tie <= 8; tie++) {
          TEST_ASSERT_EQUAL_UINT32(forwardDelayMs(rssi, spread, tie, len),
                                   adapt::stepDelayMs(rssi, spread, tie, len, FORWARD_TIE_STEPS));
          TEST_ASSERT_EQUAL_UINT32(forwardDelayMs(rssi, spread, 0, len), adapt::stepDelayMs(rssi, spread, tie, len, 1));
        }

  // The sweep's skip rule is fastRelayDropsForward's at the firmware's floor.
  const FastRelayVerdict verdicts[] = {FastRelayVerdict::SKIP, FastRelayVerdict::NO_EVIDENCE, FastRelayVerdict::STALE,
                                       FastRelayVerdict::NEEDED};
  for (FastRelayVerdict v : verdicts)
    for (uint8_t type : {FRAME_POSITION, FRAME_VOICE})
      for (size_t cars = 0; cars <= MAX_RIDERS; cars++)
        TEST_ASSERT_EQUAL(fastRelayDropsForward(v, type, cars), adapt::dropsForward(p, v, type, cars));

  // And the swept plan, taken through at the firmware's values, is planForward's:
  // due times from the receive time and after a stall, and each type's copies.
  FastRelay relay;
  relay.reset();
  Rider riders[MAX_RIDERS];
  for (uint8_t type : {FRAME_POSITION, FRAME_VOICE})
    for (uint8_t hops = 1; hops <= VOICE_HOPS; hops++)
      for (uint32_t late : {0u, 3u, 9u, 11u, 130u})
        for (int16_t rssi : {-99, -80, -60, -35})
          for (size_t neighbours : {1u, 6u, 14u})
            for (uint32_t tie = 0; tie < 6; tie++) {
              Frame f;
              f.type = type;
              f.hops = hops;
              f.len = type == FRAME_VOICE ? 123 : 83;
              const uint32_t rxMs = 0xFFFFFFF0u;  // across the millis() wrap
              const ForwardPlan want = planForward(f, relay, 7, riders, MAX_RIDERS, rxMs, rxMs + late, rssi, neighbours, tie);
              const ForwardPlan got = adapt::sweptPlan(p, f, relay, 7, riders, rxMs, rxMs + late, rssi, neighbours, tie);
              TEST_ASSERT_EQUAL(want.skip, got.skip);
              TEST_ASSERT_EQUAL_UINT32(want.dueMs, got.dueMs);
              TEST_ASSERT_EQUAL_UINT8(want.suppressAfter, got.suppressAfter);
            }
}

// With every link clean, fastrelay finds every car hearing every other and
// skips nearly every position forward: about 10.8 copies of a lease position
// on 50, 1.8 on 51 over a minute (plan 2B, Results). The maps take a while to
// settle, so the first 20 s after one clock read 1.4-7.8 over 24 seeds and a
// minute 1.1-3.3; this seed's minute reads 1.85.
void test_build_51_sends_a_clean_car_park_each_position_about_once() {
  Config c = scenario(Layout::PARK, 0, false, 60000, 21);
  const Result r50 = timed(c);
  c.policy = build51();
  const Result r51 = timed(c);
  report("park p=0 quiet, 50", r50);
  report("park p=0 quiet, 51", r51);
  TEST_ASSERT_TRUE(r50.leaseCopies > 8);
  TEST_ASSERT_TRUE(r51.leaseCopies < 2.5);
  TEST_ASSERT_TRUE(posPct(r51, 0) >= posPct(r50, 0));
}

// A lossy line with the talker at car 0, two seeds of 50 and 51 and one of each
// hop count, run once for the tests below.
struct LineRuns {
  Result b50[2], b51[2], oneHop, threeHops, noRelayer;
};
static const LineRuns& lineRuns() {
  static LineRuns runs;
  static bool done = false;
  if (!done) {
    for (int k = 0; k < 2; k++) {
      Config c = scenario(Layout::LINE, 0.2, true, 15000, 22 + k);
      runs.b50[k] = timed(c);
      c.policy = build51();
      runs.b51[k] = timed(c);
      if (k > 0) continue;
      c.policy.voiceHops = 1;
      c.policy.voiceSuppressAfter = 2;
      runs.oneHop = timed(c);
      c.policy.voiceHops = 3;
      runs.threeHops = timed(c);
    }
    done = true;
  }
  return runs;
}

// Level: 51 minus 50 over 192 seeds of a minute is within 0.1 points of 1- and
// 2-hop positions, and car by car within a point of voice (plan 2B, Results). A
// 15 s run of one seed varies by 1.4, 3.0 and 1.9 points, so the bounds are
// three times that over two seeds.
void test_down_a_lossy_line_51_keeps_positions_and_voice_level_with_50() {
  const LineRuns& l = lineRuns();
  report("line p=0.2, 50", l.b50[0]);
  report("line p=0.2, 51", l.b51[0]);
  double pos1 = 0, pos2 = 0, voice = 0;
  for (int k = 0; k < 2; k++) {
    pos1 += (posPct(l.b51[k], 0) - posPct(l.b50[k], 0)) / 2;
    pos2 += (posPct(l.b51[k], 1) - posPct(l.b50[k], 1)) / 2;
    voice += (onTimeOver(l.b51[k], 1, 12) - onTimeOver(l.b50[k], 1, 12)) / 2;
  }
  char line[160];
  snprintf(line, sizeof(line), "51 minus 50: 1-hop positions %+.1f, 2-hop %+.1f, voice at cars 1-12 %+.1f points", pos1,
           pos2, voice);
  TEST_MESSAGE(line);
  TEST_ASSERT_TRUE(pos1 > -3);
  TEST_ASSERT_TRUE(pos2 > -6);
  TEST_ASSERT_TRUE(voice > -4);
}

// Why VOICE_HOPS is not 1 or 3: one hop (plan 1C.2) gives out around car 8,
// and three reach cars 8-16 about 12 points more often at p = 0.2 (plan 2B,
// Results, for what that costs).
void test_voice_hops_trade_reach_down_a_line() {
  const LineRuns& l = lineRuns();
  const double at51 = onTimeOver(l.b51[0], 8, 16);
  char line[160];
  snprintf(line, sizeof(line), "cars 8-16 on time: 1 hop %.1f %%, 51 %.1f %%, 3 hops %.1f %%", onTimeOver(l.oneHop, 8, 16),
           at51, onTimeOver(l.threeHops, 8, 16));
  TEST_MESSAGE(line);
  TEST_ASSERT_TRUE(onTimeOver(l.oneHop, 8, 16) < at51 / 2);
  TEST_ASSERT_TRUE(onTimeOver(l.threeHops, 8, 16) > at51 + 4);
}

// sendDeferred's relayer byte reaches judge: down a line nearly every relayed
// copy's relayer is placed, all but 4 % of the verdicts. Without the byte every
// relayed copy that gets that far is judged on its origin alone, 45 % of them.
void test_the_relayer_byte_places_the_relayer() {
  const LineRuns& l = lineRuns();
  uint32_t judged = 0, unplaced = 0;
  for (int k = 0; k < 2; k++) {
    const FastRelaySkips& s = l.b51[k].relayPositions;
    judged += s.skipped + s.noEvidence + s.stale + s.needed;
    unplaced += s.noRelayer;
  }
  char line[120];
  snprintf(line, sizeof(line), "line p=0.2 on 51: %u position verdicts, %u with the relayer unplaced", (unsigned)judged,
           (unsigned)unplaced);
  TEST_MESSAGE(line);
  TEST_ASSERT_GREATER_THAN_UINT32(1000, judged);
  TEST_ASSERT_TRUE(unplaced * 10 < judged);
}

// A lease beacon polled mid-drain carries an rx.rxMs after the pass's nowMs,
// as onRecv on the Wi-Fi task allows. Schedule reads that stamp as now
// (stampAgeMs), so the map a beacon sends on the same pass still shows it. At
// 150 us a frame a stamp lands ahead about once in 1500 frames; a drain of 1 ms
// a frame makes it about 150 times in 10 s of a busy park.
void test_a_beacon_stamped_after_the_pass_began_is_in_that_pass_map() {
  Config c = scenario(Layout::PARK, 0.2, false, 10000, 31);
  c.policy = build51();
  c.model.drainCostUs = 1000;
  const Result r = simulate(c);
  char line[120];
  snprintf(line, sizeof(line), "park p=0.2 on 51, 1 ms a frame: %u lease beacons stamped after their pass began",
           (unsigned)r.stampsAhead);
  TEST_MESSAGE(line);
  TEST_ASSERT_GREATER_THAN_UINT32(50, r.stampsAhead);
  TEST_ASSERT_EQUAL_UINT32(0, r.stampsAheadUnmapped);
}

// What the tie rule decides, on three bench radios at two copies a frame
// (suppress-after 2; three copies is structural, see above). 50's 0-3 ms tie
// leaves both listeners inside one pass and both forward; 51's whole steps put
// one a step behind, and it hears the other's copy and drops its own, except
// when the tie draws the same step, one time in FORWARD_TIE_STEPS: 2 + 1/2 at
// two steps. Six seeds of 10 s: 2.94 and 2.45.
void test_on_a_bench_the_tie_steps_cut_copies_at_two_copies_a_frame() {
  double at50 = 0, at51 = 0;
  const int seeds = 6;
  for (int k = 0; k < seeds; k++) {
    Config c = scenario(Layout::BENCH, 0, true, 10000, 40 + k);
    c.policy = build50();
    c.policy.voiceSuppressAfter = 2;
    at50 += simulate(c).voiceCopies / seeds;
    c.policy = build51();
    c.policy.voiceSuppressAfter = 2;
    const Result r = simulate(c);
    at51 += r.voiceCopies / seeds;
    for (size_t j = 0; j < r.cars.size(); j++)
      if (j != r.talker) TEST_ASSERT_TRUE(onTimePct(r, j) >= 98);
  }
  char line[120];
  snprintf(line, sizeof(line), "bench voice copies at two copies a frame: 50's tie %.2f, 51's steps %.2f", at50, at51);
  TEST_MESSAGE(line);
  TEST_ASSERT_TRUE(at50 > 2.8);
  TEST_ASSERT_TRUE(at51 < 2.6);
}

// A skip on a small ride leaves the car that missed a position without the
// third car's forward: skipping at any size costs a 3-car ride 9 points at
// p = 0.2. 51 skips only from FAST_RELAY_MIN_CARS up, which keeps it on 50's
// flood. A minute of one seed varies by about 2.4 points, so four seeds.
void test_the_ride_size_floor_keeps_a_three_car_ride_on_the_flood() {
  double at50 = 0, at51 = 0, noFloor = 0;
  for (uint32_t seed = 24; seed < 28; seed++) {
    Config c = scenario(Layout::BENCH, 0.2, false, 60000, seed);
    at50 += posPct(simulate(c), 0) / 4;
    c.policy = build51();
    at51 += posPct(simulate(c), 0) / 4;
    c.policy.fastRelayMinCars = 1;
    noFloor += posPct(simulate(c), 0) / 4;
  }
  char line[160];
  snprintf(line, sizeof(line), "3-car ride at p=0.2, 1-hop positions: 50 %.1f %%, 51 %.1f %%, 51 skipping at any size %.1f %%",
           at50, at51, noFloor);
  TEST_MESSAGE(line);
  TEST_ASSERT_TRUE(at51 > at50 - 3);
  TEST_ASSERT_TRUE(noFloor < at50 - 4);
}

#if TOUGE_RIDESIM_REPORT
#include "ridesim_report.h"
#endif

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_airtime_is_what_the_plan_assumes);
  RUN_TEST(test_the_default_policy_is_build_50);
  RUN_TEST(test_three_bench_radios_put_every_voice_frame_on_the_air_three_times);
  RUN_TEST(test_with_loss_it_stays_near_three);
  RUN_TEST(test_two_cars_forward_every_voice_frame_on_50_and_none_on_51);
  RUN_TEST(test_positions_keep_flowing_on_a_lossy_bench_with_no_talker);
  RUN_TEST(test_a_car_park_saturates_on_the_build_50_flood);
  RUN_TEST(test_the_mac_holds_park_voice_where_dcf_puts_it);
  RUN_TEST(test_the_same_park_at_1_mbit_carries_the_talker);
  RUN_TEST(test_a_line_carries_the_talker_three_transmissions_and_no_further);
  RUN_TEST(test_the_same_seed_gives_the_same_ride);
  RUN_TEST(test_build_51_runs_the_firmware_forward_code);
  RUN_TEST(test_build_51_sends_a_clean_car_park_each_position_about_once);
  RUN_TEST(test_down_a_lossy_line_51_keeps_positions_and_voice_level_with_50);
  RUN_TEST(test_voice_hops_trade_reach_down_a_line);
  RUN_TEST(test_the_relayer_byte_places_the_relayer);
  RUN_TEST(test_a_beacon_stamped_after_the_pass_began_is_in_that_pass_map);
  RUN_TEST(test_on_a_bench_the_tie_steps_cut_copies_at_two_copies_a_frame);
  RUN_TEST(test_the_ride_size_floor_keeps_a_three_car_ride_on_the_flood);
#if TOUGE_RIDESIM_REPORT
  RUN_TEST(test_report_the_build_50_baseline);
  RUN_TEST(test_report_build_51_against_50);
#endif
  return UNITY_END();
}
