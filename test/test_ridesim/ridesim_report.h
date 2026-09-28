#pragma once
//
// The tables behind plan 2B's Results, printed when test_ridesim is built with
// -D TOUGE_RIDESIM_REPORT=1 (run the test verbose to see them). Each env prints
// its own: native-lean with the V3's tables, native with the V4's.
// -D TOUGE_RIDESIM_SEEDS=n sets the seeds per row of the 51-against-50 tables
// and -D TOUGE_RIDESIM_CHECK_SEEDS=n those of the fresh-seed check after them, 4
// each by default. The plan's numbers used 192 and 512 on the V3's tables, 47
// minutes at -O2 on 16 cores. Every run is independent and seeded, so they go
// out over every core.
//
// Included by test_main.cpp after its helpers (scenario, timed, posPct,
// voiceWithin, voiceLatencyMs, onTimePct, onTimeOver).

#include <math.h>
#include <atomic>
#include <string>
#include <thread>
#include <vector>

#ifndef TOUGE_RIDESIM_SEEDS
#define TOUGE_RIDESIM_SEEDS 4
#endif

// Every config through simulate() on every core, each result through `measure`.
template <class Out, class Measure>
static std::vector<Out> runEach(const std::vector<Config>& configs, Measure measure) {
  std::vector<Out> out(configs.size());
  std::atomic<size_t> next{0};
  const unsigned cores = std::max(1u, std::thread::hardware_concurrency());
  std::vector<std::thread> workers;
  for (unsigned t = 0; t < cores; t++) {
    workers.emplace_back([&] {
      for (size_t i = next++; i < configs.size(); i = next++) out[i] = measure(timed(configs[i]));
    });
  }
  for (std::thread& w : workers) w.join();
  return out;
}

static const char* tablesName() { return TOUGE_LEAN_RAM ? "lean (V3)" : "roomy (V4)"; }

// ---- Build 50 alone: one seed a scenario, every column --------------------------

static double meanMs(const std::vector<int32_t>& v) {
  if (v.empty()) return 0;
  double s = 0;
  for (int32_t x : v) s += x;
  return s / v.size() / 1000.0;
}

static void label(const Result& r, char* out, size_t cap) {
  const Config& c = r.cfg;
  snprintf(out, cap, "%s p=%.1f %s%s%s%s%s",
           c.layout == Layout::PARK ? "park" : (c.layout == Layout::LINE ? "line" : "bench"), c.p,
           c.talker ? (c.talkerBytes > TEST_VOICE_DEFAULT_BYTES ? "talk203" : "talk107") : "quiet",
           c.phy.bitsPerSec != 250000 ? " 1M" : "", c.stallEveryMs ? " stalls" : "", c.moving ? " moving" : "",
           c.label[0] ? c.label : "");
}

static void printPositions(const std::vector<Result>& rs) {
  printf("\nPositions: lease and unleased beacons delivered, by graph hops from the origin; age of each\n"
         "car's position on every map within 3 hops, sampled every 100 ms; air copies per lease beacon.\n");
  printf("%-34s %6s %6s %6s %6s | %7s %7s %7s | %6s %7s %7s\n", "scenario", "1hop%", "2hop%", "3hop%", "4+hop%",
         "age ms", "p95", "p99", "copies", "lease/s", "unlsd/s");
  for (const Result& r : rs) {
    char l[64];
    label(r, l, sizeof(l));
    printf("%-34s %6.1f %6.1f %6.1f %6.1f | %7.0f %7.0f %7.0f | %6.2f %7.1f %7.1f\n", l, posPct(r, 0), posPct(r, 1),
           posPct(r, 2), posPct(r, 3), r.ageN ? r.ageSumMs / r.ageN : 0, r.ageP(0.95), r.ageP(0.99), r.leaseCopies,
           r.txCount[TX_LEASE] / r.seconds(), r.txCount[TX_UNLEASED] / r.seconds());
  }
}

static void printVoice(const std::vector<Result>& rs) {
  printf("\nVoice: on time = first copy drained within 250 ms of the talker's send. Over the listeners\n"
         "within 3 graph hops: the worst and median on-time %%, then means. Holes are runs of frames\n"
         "not on time, per listener. 'repr' also counts a frame the next one's prev covers (203 B).\n");
  printf("%-34s %5s %4s | %6s %6s %6s | %6s %6s %6s | %6s %5s | %6s %5s | %5s\n", "scenario", "sent", "refd", "worst%",
         "med%", "repr%", "lat ms", "p95", "jitter", "holes", "long", "copies", "max", "air%");
  for (const Result& r : rs) {
    if (!r.cfg.talker) continue;
    char l[64];
    label(r, l, sizeof(l));
    double worst = 0, median = 0;
    voiceWithin(r, 3, worst, median);
    double lat = 0, p95 = 0, jit = 0, holes = 0, longest = 0, repaired = 0;
    uint32_t cars = 0;
    for (size_t j = 0; j < r.cars.size(); j++) {
      const CarStats& c = r.cars[j];
      if (j == r.talker || c.fromTalker == 0 || c.fromTalker > 3 || c.expected == 0) continue;
      cars++;
      lat += c.delivered ? c.latSumMs / c.delivered : 0;
      p95 += c.latP95Ms;
      jit += c.jitterN ? c.jitterSumMs / c.jitterN : 0;
      holes += c.holeRuns;
      longest = std::max(longest, (double)c.longestHole);
      repaired += 100.0 * c.repaired / c.expected;
    }
    if (cars == 0) cars = 1;
    printf("%-34s %5u %4u | %6.1f %6.1f %6.1f | %6.1f %6.1f %6.1f | %6.1f %5.0f | %6.2f %5u | %5.1f\n", l,
           (unsigned)r.voiceAttempted, (unsigned)r.voiceRefused, worst, median, repaired / cars, lat / cars, p95 / cars,
           jit / cars, holes / cars, longest, r.voiceCopies, (unsigned)r.voiceCopiesMax,
           r.airPct(TX_VOICE) + r.airPct(TX_FWD_VOICE));
  }
}

static void printAir(const std::vector<Result>& rs) {
  printf("\nAir and schedule: airtime as a share of the window, summed over transmitters (a line reuses\n"
         "the channel, so it can pass 100); each car's busy share (sensed or sending), median and max.\n"
         "Beacon lateness = send start minus slot start, mean/p95 ms: on the reference's clock by hops\n"
         "to it, and on the car's own clock for all. ovr = still on the air when the slot closed, on\n"
         "the car's own clock and the reference's.\n");
  printf("%-34s %5s %5s %5s %5s | %5s %5s | %-11s %-11s %-11s %-11s %-11s | %4s %4s\n", "scenario", "pos%", "fwdP%",
         "voic%", "fwdV%", "busy", "max", "ride hop0", "ride hop1", "ride hop2", "ride hop3+", "own", "ovrO", "ovrR");
  for (const Result& r : rs) {
    char l[64];
    label(r, l, sizeof(l));
    std::vector<double> busy;
    for (const CarStats& c : r.cars) busy.push_back(c.busyPct);
    char hop[5][24];
    uint32_t overOwn = 0, overRide = 0, beacons = 0, rideBeacons = 0;
    std::vector<int32_t> deep, ownAll;
    for (int h = 0; h < 7; h++) {
      overOwn += r.late[h].overranOwn;
      overRide += r.late[h].overranRide;
      beacons += (uint32_t)r.late[h].ownUs.size();
      rideBeacons += (uint32_t)r.late[h].rideUs.size();
      ownAll.insert(ownAll.end(), r.late[h].ownUs.begin(), r.late[h].ownUs.end());
      if (h >= 3 && h <= 5) deep.insert(deep.end(), r.late[h].rideUs.begin(), r.late[h].rideUs.end());
    }
    for (int h = 0; h < 4; h++) {
      const std::vector<int32_t>& v = h < 3 ? r.late[h].rideUs : deep;
      if (v.empty()) snprintf(hop[h], sizeof(hop[h]), "-");
      else snprintf(hop[h], sizeof(hop[h]), "%.1f/%.1f", meanMs(v), percentileI(v, 0.95) / 1000);
    }
    snprintf(hop[4], sizeof(hop[4]), "%.1f/%.1f", meanMs(ownAll), percentileI(ownAll, 0.95) / 1000);
    printf("%-34s %5.1f %5.1f %5.1f %5.1f | %5.1f %5.1f | %-11s %-11s %-11s %-11s %-11s | %4.1f %4.1f\n", l,
           r.airPct(TX_LEASE) + r.airPct(TX_UNLEASED) + r.airPct(TX_EXTRA), r.airPct(TX_FWD_POS), r.airPct(TX_VOICE),
           r.airPct(TX_FWD_VOICE), percentile(busy, 0.5), percentile(busy, 1.0), hop[0], hop[1], hop[2], hop[3],
           hop[4], beacons ? 100.0 * overOwn / beacons : 0, rideBeacons ? 100.0 * overRide / rideBeacons : 0);
  }
}

static void printLosses(const std::vector<Result>& rs) {
  printf("\nLosses and the schedule: receptions lost to p and the edge ('loss'), because the receiver was\n"
         "sending ('hdplx'), and to collisions: a lease beacon to another lease beacon ('clash'), to an\n"
         "unslotted frame ('hit'), any other frame ('other'). Leases given up, 100 ms samples with two\n"
         "cars that could collide on one slot, samples not on one clock, and when the ride first kept\n"
         "one clock after power-on (s). Cars leased and references named at the end.\n");
  printf("%-34s %7s %6s | %5s %6s %7s | %4s %6s %6s %6s | %4s %4s\n", "scenario", "loss", "hdplx", "clash", "hit",
         "other", "slot", "shared", "unsync", "clock", "lsd", "refs");
  for (const Result& r : rs) {
    char l[64], clock[16];
    label(r, l, sizeof(l));
    if (r.syncedAtMs == 0) snprintf(clock, sizeof(clock), "never");
    else snprintf(clock, sizeof(clock), "%.1f", r.syncedAtMs / 1000.0);
    printf("%-34s %7u %6u | %5u %6u %7u | %4u %6u %6u %6s | %4u %4u\n", l, (unsigned)r.randomLoss,
           (unsigned)r.halfDuplex, (unsigned)r.leaseClash, (unsigned)r.leaseHit, (unsigned)r.otherCollisions,
           (unsigned)r.slotLosses, (unsigned)r.sharedSlotSamples, (unsigned)r.unsyncedSamples, clock,
           (unsigned)r.leased, (unsigned)r.references);
  }
}

static void printQueues(const std::vector<Result>& rs) {
  printf("\nQueues: esp_now_send refused on a full TX queue, by kind, and lease beacons retried in the\n"
         "slot; frames the RX queue dropped; Mesh::defer refused (forward slots full); forwards dropped\n"
         "at nextDue on copies; the longest gap between passes; wall time for the scenario.\n");
  printf("%-34s %6s %6s %6s %6s %6s | %6s %6s %6s %6s | %5s | %6s\n", "scenario", "lease", "unlsd", "fwdP", "voice",
         "fwdV", "retry", "rxdrop", "defer", "supp", "gapms", "wall s");
  for (const Result& r : rs) {
    char l[64];
    label(r, l, sizeof(l));
    printf("%-34s %6u %6u %6u %6u %6u | %6u %6u %6u %6u | %5u | %6.2f\n", l, (unsigned)r.txRefused[TX_LEASE],
           (unsigned)r.txRefused[TX_UNLEASED], (unsigned)r.txRefused[TX_FWD_POS], (unsigned)r.txRefused[TX_VOICE],
           (unsigned)r.txRefused[TX_FWD_VOICE], (unsigned)r.leaseRetries, (unsigned)r.rxDrops,
           (unsigned)r.deferRefused, (unsigned)r.suppressed, (unsigned)r.maxPassGapMs, r.wallMs / 1000);
  }
}

static void printCars(const Result& r) {
  char l[64];
  label(r, l, sizeof(l));
  printf("\nPer car, %s (talker car %u): graph hops from the talker; positions from cars within 3\n"
         "hops and their mean age; voice on time, mean hops of the first copy, latency mean and p95,\n"
         "jitter (mean change between consecutive on-time frames), holes and the longest; the\n"
         "VoiceMeter's own delay reading ({\"vt\"} dm, -1 none); busy share.\n",
         l, (unsigned)r.talker);
  printf("%3s %4s | %6s %7s | %6s %5s %6s %6s %6s %5s %5s %6s | %5s\n", "car", "hops", "pos%", "age ms", "voice%",
         "hops", "lat", "p95", "jitter", "holes", "long", "vt dm", "busy%");
  for (size_t j = 0; j < r.cars.size(); j++) {
    const CarStats& c = r.cars[j];
    printf("%3u %4u | %6.1f %7.0f | %6.1f %5.2f %6.1f %6.1f %6.1f %5u %5u %6.1f | %5.1f%s\n", (unsigned)j,
           (unsigned)c.fromTalker, c.posSent ? 100.0 * c.posGot / c.posSent : 0, c.ageN ? c.ageSumMs / c.ageN : 0,
           onTimePct(r, j), c.delivered ? c.firstHopsSum / c.delivered : 0, c.delivered ? c.latSumMs / c.delivered : 0,
           c.latP95Ms, c.jitterN ? c.jitterSumMs / c.jitterN : 0, (unsigned)c.holeRuns, (unsigned)c.longestHole,
           c.meterMeanMs, c.busyPct, j == r.talker ? "  talker" : "");
  }
}

static void printAll(const char* title, const std::vector<Result>& rs) {
  printf("\n==== %s ====\n", title);
  printPositions(rs);
  printVoice(rs);
  printAir(rs);
  printLosses(rs);
  printQueues(rs);
}

void test_report_the_build_50_baseline() {
  printf("\nride sim: %s tables (RX %u deep, %u full-size forward slots), policy build 50\n", tablesName(),
         (unsigned)Model().rxDepth, (unsigned)FORWARD_FULL_SLOTS);
  std::vector<Config> base;
  uint32_t seed = 100;
  for (Layout layout : {Layout::PARK, Layout::LINE}) {
    for (double p : {0.0, 0.2, 0.4}) {
      for (bool talker : {false, true}) base.push_back(scenario(layout, p, talker, 60000, seed++));
    }
  }
  std::vector<Config> more;
  seed = 200;
  for (Layout layout : {Layout::PARK, Layout::LINE}) {
    for (double p : {0.2, 0.4}) {
      Config c = scenario(layout, p, true, 60000, seed++);
      c.talkerBytes = 203;
      more.push_back(c);
    }
    Config fast = scenario(layout, 0.2, true, 60000, seed++);
    fast.phy = rate1M();
    more.push_back(fast);
    Config stalls = scenario(layout, 0.2, true, 60000, seed++);
    stalls.stallEveryMs = 5000;
    more.push_back(stalls);
    Config moving = scenario(layout, 0.2, true, 60000, seed++);
    moving.moving = true;
    more.push_back(moving);
  }
  // How far the park and line results lean on the model's guesses.
  std::vector<Config> sens;
  seed = 300;
  for (Layout layout : {Layout::PARK, Layout::LINE}) {
    const Config base1 = scenario(layout, 0.2, true, 60000, seed++);
    Config c = base1;
    c.label = " (as baseline)";
    sens.push_back(c);
    c = base1;
    c.label = " CW 31";
    c.model.cwSlots = 31;
    sens.push_back(c);
    c = base1;
    c.label = " capture 6 dB";
    c.model.captureDb = 6;
    sens.push_back(c);
    c = base1;
    c.label = " loss flat by length";
    c.model.lossScalesWithAirtime = false;
    sens.push_back(c);
    c = base1;
    c.label = " no fading";
    c.model.fadeSigmaDb = 0;
    sens.push_back(c);
    c = base1;
    c.label = " TX 16 buffers";
    c.model.txBuffers = 16;
    sens.push_back(c);
    c = base1;
    c.label = " drain 1 ms a frame";
    c.model.drainCostUs = 1000;
    sens.push_back(c);
    if (layout == Layout::PARK) {
      c = base1;
      c.label = " n=2.5 (stronger, closer)";
      c.model.exponent = 2.5;
      sens.push_back(c);
      c = base1;
      c.label = " shadowing 6 dB";
      c.model.shadowSigmaDb = 6;
      sens.push_back(c);
    } else {
      c = base1;
      c.label = " 22 m apart (~5 each way)";
      c.model.lineSpacingM = 22;
      sens.push_back(c);
      c = base1;
      c.label = " 34 m apart (~3 each way)";
      c.model.lineSpacingM = 34;
      sens.push_back(c);
    }
  }
  const auto whole = [](const Result& r) { return r; };
  const std::vector<Result> baseRuns = runEach<Result>(base, whole);
  const std::vector<Result> moreRuns = runEach<Result>(more, whole);
  const std::vector<Result> sensRuns = runEach<Result>(sens, whole);

  printAll("Build 50 baseline: 25 cars, 60 s after one clock, LR 250 kbit/s", baseRuns);
  for (const Result& r : baseRuns)
    if (r.cfg.talker && r.cfg.layout == Layout::LINE) printCars(r);
  for (const Result& r : baseRuns)
    if (r.cfg.talker && r.cfg.layout == Layout::PARK && r.cfg.p == 0.4) printCars(r);
  printAll("Build 50 variants: 203 B voice (prev), 1 Mbit/s, V4 stalls every ~5 s, moving", moreRuns);
  for (const Result& r : moreRuns)
    if (r.cfg.layout == Layout::LINE && r.cfg.talkerBytes > TEST_VOICE_DEFAULT_BYTES && r.cfg.p == 0.4) printCars(r);
  printAll("Sensitivity: p = 0.2 with the talker, one assumption changed at a time", sensRuns);

  double wall = 0;
  for (const std::vector<Result>* rs : {&baseRuns, &moreRuns, &sensRuns})
    for (const Result& r : *rs) wall += r.wallMs;
  printf("\nsimulated in %.1f s of CPU for %u scenarios\n", wall / 1000,
         (unsigned)(baseRuns.size() + moreRuns.size() + sensRuns.size()));
  fflush(stdout);
}

// ---- Build 51 against 50, every variant, over seeds ------------------------------
//
// Every variant runs on the same seeds as 50, so a difference is taken seed by
// seed and its standard error is that of the paired differences, printed as
// difference/SE. A position cell counts as below 50 when it is more than 3
// standard errors down: a variant is held to about 60 cells, and at 2 one of
// them would fail by chance.
static const double BELOW_SE = 3;
// And by more than this, in points. A difference smaller than this counts as
// none however many seeds resolve it, in the floor and in far-end voice: every
// 51 variant loses 0.1-0.2 points on a clean 4-car ride from the step rule
// itself, which 96 seeds resolve.
static const double LEVEL_POINTS = 0.5;

// What the tables read from one run. NAN where a run has nothing to say.
struct Metrics {
  double pos[3] = {NAN, NAN, NAN};  // % delivered by graph hops from the origin
  double ageMs = 0, leaseCopies = 0, airPct = 0, wallMs = 0;
  // voice, over listeners within 3 graph hops
  double voiceWorst = NAN, voiceMedian = NAN, voiceMean = NAN, latMs = NAN, latP95Ms = NAN, holes = NAN;
  double voiceCopies = NAN;
  // down a line: on time at cars 8-16, the same after prev repair, and cars 1-4 after repair
  double far = NAN, farRepaired = NAN, nearRepaired = NAN;
  double carOnTime[25];
  // lease lateness on the reference's clock, hop 0 / 1 / 2 / 3+; overran on the car's own
  double late[4] = {NAN, NAN, NAN, NAN}, overranPct = NAN;
  double slotLosses = 0, deferRefused = 0;
  Metrics() {
    for (double& x : carOnTime) x = NAN;
  }
};

static double meanOrNan(const std::vector<int32_t>& v) { return v.empty() ? NAN : meanMs(v); }

static Metrics measure(const Result& r) {
  Metrics m;
  for (size_t b = 0; b < 3; b++)
    if (r.posSent[b] > 0) m.pos[b] = 100.0 * r.posGot[b] / r.posSent[b];
  m.ageMs = r.ageN ? r.ageSumMs / r.ageN : 0;
  m.leaseCopies = r.leaseCopies;
  m.airPct = r.airPctTotal();
  m.wallMs = r.wallMs;
  if (r.cfg.talker) {
    voiceWithin(r, 3, m.voiceWorst, m.voiceMedian);
    double onTime = 0, lat = 0, p95 = 0, holes = 0;
    uint32_t cars = 0;
    for (size_t j = 0; j < r.cars.size(); j++) {
      const CarStats& c = r.cars[j];
      if (j == r.talker || c.fromTalker == 0 || c.fromTalker > 3 || c.expected == 0) continue;
      cars++;
      onTime += onTimePct(r, j);
      lat += c.delivered ? c.latSumMs / c.delivered : 0;
      p95 += c.latP95Ms;
      holes += c.holeRuns;
    }
    if (cars > 0) {
      m.voiceMean = onTime / cars;
      m.latMs = lat / cars;
      m.latP95Ms = p95 / cars;
      m.holes = holes / cars;
    }
    m.voiceCopies = r.voiceCopies;
    if (r.cfg.layout == Layout::LINE) {
      m.far = onTimeOver(r, 8, 16);
      auto repairedOver = [&](size_t from, size_t to) {
        double sum = 0;
        for (size_t j = from; j <= to; j++) sum += r.cars[j].expected ? 100.0 * r.cars[j].repaired / r.cars[j].expected : 0;
        return sum / (to - from + 1);
      };
      m.farRepaired = repairedOver(8, 16);
      m.nearRepaired = repairedOver(1, 4);
      for (size_t j = 0; j < r.cars.size() && j < 25; j++) m.carOnTime[j] = onTimePct(r, j);
    }
  }
  std::vector<int32_t> deep;
  uint32_t overran = 0, beacons = 0;
  for (int h = 0; h < 7; h++) {
    if (h >= 3 && h <= 5) deep.insert(deep.end(), r.late[h].rideUs.begin(), r.late[h].rideUs.end());
    overran += r.late[h].overranOwn;
    beacons += (uint32_t)r.late[h].ownUs.size();
  }
  for (int h = 0; h < 3; h++) m.late[h] = meanOrNan(r.late[h].rideUs);
  m.late[3] = meanOrNan(deep);
  if (beacons > 0) m.overranPct = 100.0 * overran / beacons;
  m.slotLosses = r.slotLosses * 60000.0 / r.cfg.measureMs;
  m.deferRefused = r.deferRefused * 60000.0 / r.cfg.measureMs;
  return m;
}

struct Scen {
  Layout layout;
  size_t cars;
  double p;
  bool talker;
  uint8_t bytes;
  bool big() const { return layout != Layout::BENCH; }
  std::string name() const {
    char out[48];
    if (big())
      snprintf(out, sizeof(out), "%s, p = %.1f, %s", layout == Layout::PARK ? "park" : "line", p,
               talker ? (bytes > TEST_VOICE_DEFAULT_BYTES ? "203 B talker" : "talker") : "quiet");
    else
      snprintf(out, sizeof(out), "%u cars, p = %.1f, %s", (unsigned)cars, p, talker ? "talker" : "quiet");
    return out;
  }
};

// BASE: 50 and 51. RELAY: 51 with another skip rule. GRID: 51 with other voice
// hops, voice copies and tie steps, every combination the selection weighs.
enum class Family : uint8_t { BASE, RELAY, GRID };

struct Variant {
  std::string name;
  Policy policy;
  Family family;
  // A variant that runs the same without a talker, whose quiet runs stand in
  // for this one's; SIZE_MAX for none.
  size_t quietTwin;
  // Printed row by row: 50, 51, the skip rules, and the grid one constant away
  // from 51. The rest of the grid shows in the selection only.
  bool detail;
};

static const uint8_t GRID_HOPS[] = {1, 2, 3, 5};
static const uint8_t GRID_COPIES[] = {2, 3};
static const uint32_t GRID_TIES[] = {1, 2, 3, 4, 6, 8};

static std::string gridName(uint8_t hops, uint8_t copies, uint32_t ties) {
  const bool pair = hops != VOICE_HOPS || copies != VOICE_SUPPRESS_AFTER;
  const bool tie = ties != FORWARD_TIE_STEPS;
  char out[64];
  if (pair && tie)
    snprintf(out, sizeof(out), "voice %u hop%s, %u copies, tie %u", (unsigned)hops, hops == 1 ? "" : "s", (unsigned)copies,
             (unsigned)ties);
  else if (pair)
    snprintf(out, sizeof(out), "voice %u hop%s, %u copies", (unsigned)hops, hops == 1 ? "" : "s", (unsigned)copies);
  else
    snprintf(out, sizeof(out), "tie %u step%s", (unsigned)ties, ties == 1 ? "" : "s");
  return out;
}

static std::vector<Variant> variants() {
  std::vector<Variant> v;
  v.push_back({"50", build50(), Family::BASE, SIZE_MAX, true});
  v.push_back({"51", build51(), Family::BASE, SIZE_MAX, true});
  Policy p = build51();
  p.fastRelaySkips = false;
  v.push_back({"51, fastrelay off", p, Family::RELAY, SIZE_MAX, true});
  p = build51();
  p.fastRelayMinCars = 1;
  v.push_back({"51, no ride-size floor", p, Family::RELAY, SIZE_MAX, true});
  p = build51();
  p.fastRelaySkipsVoice = true;
  v.push_back({"51, voice skips above the floor", p, Family::RELAY, 1, true});
  for (uint32_t ties : GRID_TIES) {
    // Voice changes nothing without a talker, so every pair at one tie count
    // shares the quiet runs of the firmware's pair at it.
    size_t twin = 1;
    if (ties != FORWARD_TIE_STEPS) {
      p = build51();
      p.tieSteps = ties;
      twin = v.size();
      v.push_back({gridName(VOICE_HOPS, VOICE_SUPPRESS_AFTER, ties), p, Family::GRID, SIZE_MAX, true});
    }
    for (uint8_t hops : GRID_HOPS)
      for (uint8_t copies : GRID_COPIES) {
        if (hops == VOICE_HOPS && copies == VOICE_SUPPRESS_AFTER) continue;
        p = build51();
        p.voiceHops = hops;
        p.voiceSuppressAfter = copies;
        p.tieSteps = ties;
        v.push_back({gridName(hops, copies, ties), p, Family::GRID, twin, ties == FORWARD_TIE_STEPS});
      }
  }
  return v;
}

static std::vector<Scen> scenarios() {
  std::vector<Scen> s;
  for (Layout layout : {Layout::PARK, Layout::LINE})
    for (bool talker : {false, true})
      for (double p : {0.0, 0.2, 0.4}) s.push_back({layout, 25, p, talker, TEST_VOICE_DEFAULT_BYTES});
  s.push_back({Layout::LINE, 25, 0.4, true, 203});
  for (size_t cars : {2, 3, 4, 5, 6, 8})
    for (bool talker : {false, true})
      for (double p : {0.0, 0.2, 0.4}) s.push_back({Layout::BENCH, cars, p, talker, TEST_VOICE_DEFAULT_BYTES});
  return s;
}

// The seeds of every (scenario, variant), paired across variants.
struct Runs {
  std::vector<Scen> scens;
  std::vector<Variant> vars;
  int seeds = 0;
  uint32_t firstSeed = 5000;
  std::vector<std::vector<std::vector<Metrics>>> at;  // [scenario][variant][seed]

  const std::vector<Metrics>& of(size_t s, size_t v) const {
    if (vars[v].quietTwin != SIZE_MAX && !scens[s].talker) return at[s][vars[v].quietTwin];
    return at[s][v];
  }
  size_t find(const char* name) const {
    for (size_t v = 0; v < vars.size(); v++)
      if (vars[v].name == name) return v;
    return SIZE_MAX;
  }
  size_t scen(Layout layout, double p, bool talker, uint8_t bytes = TEST_VOICE_DEFAULT_BYTES, size_t cars = 25) const {
    for (size_t s = 0; s < scens.size(); s++) {
      const Scen& x = scens[s];
      if (x.layout == layout && x.cars == cars && fabs(x.p - p) < 1e-9 && x.talker == talker && x.bytes == bytes) return s;
    }
    return SIZE_MAX;
  }
};

static Runs runVariants(const std::vector<Scen>& scens, const std::vector<Variant>& vars, int seeds,
                        uint32_t firstSeed = 5000) {
  Runs runs;
  runs.scens = scens;
  runs.vars = vars;
  runs.seeds = seeds;
  runs.firstSeed = firstSeed;
  std::vector<Config> configs;
  std::vector<size_t> where;  // (s * vars + v) * seeds + k for each config
  for (size_t s = 0; s < runs.scens.size(); s++) {
    const Scen& sc = runs.scens[s];
    for (size_t v = 0; v < runs.vars.size(); v++) {
      if (runs.vars[v].quietTwin != SIZE_MAX && !sc.talker) continue;
      for (int k = 0; k < runs.seeds; k++) {
        Config c = scenario(sc.layout, sc.p, sc.talker, 60000, firstSeed + k);
        c.cars = sc.cars;
        c.talkerBytes = sc.bytes;
        c.policy = runs.vars[v].policy;
        configs.push_back(c);
        where.push_back((s * runs.vars.size() + v) * runs.seeds + k);
      }
    }
  }
  const std::vector<Metrics> out = runEach<Metrics>(configs, measure);
  runs.at.assign(runs.scens.size(), std::vector<std::vector<Metrics>>(runs.vars.size()));
  for (size_t i = 0; i < out.size(); i++) {
    const size_t k = where[i] % runs.seeds, sv = where[i] / runs.seeds;
    std::vector<Metrics>& seeds = runs.at[sv / runs.vars.size()][sv % runs.vars.size()];
    if (seeds.size() < (size_t)runs.seeds) seeds.resize(runs.seeds);
    seeds[k] = out[i];
  }
  return runs;
}

// A column's mean over seeds, and its difference from another variant's, seed by seed.
struct Cell {
  double mean = NAN, diff = NAN, se = NAN;
};

template <class Get>
static Cell cell(const Runs& runs, size_t s, size_t v, Get get, size_t against = 0) {
  Cell c;
  const std::vector<Metrics>& a = runs.of(s, v);
  const std::vector<Metrics>& b = runs.of(s, against);
  double sum = 0, dsum = 0, dsq = 0;
  int n = 0, nd = 0;
  for (size_t k = 0; k < a.size(); k++) {
    const double x = get(a[k]);
    if (isnan(x)) continue;
    sum += x;
    n++;
    const double y = get(b[k]);
    if (isnan(y)) continue;
    dsum += x - y;
    dsq += (x - y) * (x - y);
    nd++;
  }
  if (n * 2 < (int)a.size()) return c;  // mostly nothing to say
  c.mean = sum / n;
  if (nd >= 2) {
    c.diff = dsum / nd;
    const double var = (dsq - nd * c.diff * c.diff) / (nd - 1);
    c.se = sqrt(var > 0 ? var : 0) / sqrt((double)nd);
  }
  return c;
}

static std::string num(double x, const char* fmt = "%.1f") {
  if (isnan(x)) return "-";
  char out[32];
  snprintf(out, sizeof(out), fmt, x);
  return out;
}

static std::string diff(const Cell& c) {
  if (isnan(c.diff)) return "-";
  char out[32];
  snprintf(out, sizeof(out), "%+.1f/%.1f", c.diff, c.se);
  return out;
}

static std::string withDiff(const Cell& c) {
  if (isnan(c.mean)) return "-";
  return num(c.mean) + " (" + diff(c) + ")";
}

static const auto POS1 = [](const Metrics& m) { return m.pos[0]; };
static const auto POS2 = [](const Metrics& m) { return m.pos[1]; };
static const auto POS3 = [](const Metrics& m) { return m.pos[2]; };
static const auto FAR = [](const Metrics& m) { return m.far; };

// Every position cell of a variant against 50: each ride, each hop bucket it
// has. Below: under 50 by more than BELOW_SE standard errors and LEVEL_POINTS.
struct Below {
  int cells = 0, below = 0;
  std::string worst;  // the cell furthest below 50 of those beyond BELOW_SE
  double worstDiff = 0;
};

static Below positionsAgainst50(const Runs& runs, size_t v) {
  Below out;
  for (size_t s = 0; s < runs.scens.size(); s++) {
    for (int b = 0; b < 3; b++) {
      const Cell c = cell(runs, s, v, [b](const Metrics& m) { return m.pos[b]; });
      if (isnan(c.diff)) continue;
      out.cells++;
      if (!(c.diff < -BELOW_SE * c.se)) continue;
      if (c.diff < -LEVEL_POINTS) out.below++;
      if (c.diff < out.worstDiff) {
        out.worstDiff = c.diff;
        char at[96];
        snprintf(at, sizeof(at), "%s, %d-hop %s", runs.scens[s].name().c_str(), b + 1, diff(c).c_str());
        out.worst = at;
      }
    }
  }
  return out;
}

// Far-end voice down the line: cars 8-16, the mean over p = 0, 0.2 and 0.4.
static Cell farEnd(const Runs& runs, size_t v, size_t against) {
  Cell out;
  const size_t lines[] = {runs.scen(Layout::LINE, 0, true), runs.scen(Layout::LINE, 0.2, true),
                          runs.scen(Layout::LINE, 0.4, true)};
  std::vector<double> d(runs.seeds, 0);
  double mean = 0;
  for (size_t s : lines) {
    const std::vector<Metrics>& a = runs.of(s, v);
    const std::vector<Metrics>& b = runs.of(s, against);
    for (int k = 0; k < runs.seeds; k++) {
      mean += a[k].far / (3 * runs.seeds);
      d[k] += (a[k].far - b[k].far) / 3;
    }
  }
  out.mean = mean;
  double sum = 0, sq = 0;
  for (double x : d) sum += x;
  out.diff = sum / runs.seeds;
  for (double x : d) sq += (x - out.diff) * (x - out.diff);
  out.se = runs.seeds > 1 ? sqrt(sq / (runs.seeds - 1) / runs.seeds) : NAN;
  return out;
}

static void printHeader(const char* title, const Runs& runs) {
  printf("\n==== %s (%s tables, %d seeds from %u, 60 s each) ====\n", title, tablesName(), runs.seeds,
         (unsigned)runs.firstSeed);
}

static void printRideTables(const Runs& runs) {
  const size_t b50 = 0, b51 = 1;
  printHeader("25 cars without a talker: 51 against 50", runs);
  printf("Positions delivered by graph hops from the origin, %% (51: the difference from 50 seed by seed, difference/SE);\n"
         "age of each car's position on every map within 3 hops; air copies per lease beacon; summed air %%;\n"
         "lease lateness on the reference's clock by hops to it, ms; leases lost a minute.\n");
  printf("%-27s %-5s | %-16s %-16s %-16s | %6s %6s %5s | %-25s | %5s\n", "ride", "build", "1 hop %", "2 hops %",
         "3 hops %", "age ms", "copies", "air %", "late 0 / 1 / 2 / 3+ ms", "lost");
  for (size_t s = 0; s < runs.scens.size(); s++) {
    const Scen& sc = runs.scens[s];
    if (!sc.big() || sc.talker) continue;
    for (size_t v : {b50, b51}) {
      auto show = [&](const Cell& c) { return v == b50 ? num(c.mean) : withDiff(c); };
      const Cell late0 = cell(runs, s, v, [](const Metrics& m) { return m.late[0]; });
      const Cell late1 = cell(runs, s, v, [](const Metrics& m) { return m.late[1]; });
      const Cell late2 = cell(runs, s, v, [](const Metrics& m) { return m.late[2]; });
      const Cell late3 = cell(runs, s, v, [](const Metrics& m) { return m.late[3]; });
      const std::string late = num(late0.mean) + " / " + num(late1.mean) + " / " + num(late2.mean) + " / " + num(late3.mean);
      printf("%-27s %-5s | %-16s %-16s %-16s | %6s %6s %5s | %-25s | %5s\n", v == b50 ? sc.name().c_str() : "",
             runs.vars[v].name.c_str(), show(cell(runs, s, v, POS1)).c_str(), show(cell(runs, s, v, POS2)).c_str(),
             show(cell(runs, s, v, POS3)).c_str(), num(cell(runs, s, v, [](const Metrics& m) { return m.ageMs; }).mean, "%.0f").c_str(),
             num(cell(runs, s, v, [](const Metrics& m) { return m.leaseCopies; }).mean, "%.2f").c_str(),
             num(cell(runs, s, v, [](const Metrics& m) { return m.airPct; }).mean, "%.0f").c_str(), late.c_str(),
             num(cell(runs, s, v, [](const Metrics& m) { return m.slotLosses; }).mean).c_str());
    }
  }

  printHeader("25 cars with the talker: 51 against 50", runs);
  printf("Voice over listeners within 3 graph hops of the talker: on time = drained within 250 ms of the send;\n"
         "worst and median listener; cars 8-16 of the line; latency mean and p95 ms; hole runs per listener a\n"
         "minute; air copies per lease beacon and per voice frame.\n");
  printf("%-27s %-5s | %-16s %-16s %6s | %-11s %6s | %-9s %6s | %-11s %5s\n", "ride", "build", "1 hop %", "2 hops %",
         "age ms", "worst / med", "8-16 %", "lat / p95", "holes", "lease / vce", "air %");
  for (size_t s = 0; s < runs.scens.size(); s++) {
    const Scen& sc = runs.scens[s];
    if (!sc.big() || !sc.talker) continue;
    for (size_t v : {b50, b51}) {
      auto show = [&](const Cell& c) { return v == b50 ? num(c.mean) : withDiff(c); };
      auto mean = [&](double Metrics::*field, const char* fmt) {
        return num(cell(runs, s, v, [field](const Metrics& m) { return m.*field; }).mean, fmt);
      };
      printf("%-27s %-5s | %-16s %-16s %6s | %-11s %6s | %-9s %6s | %-11s %5s\n", v == b50 ? sc.name().c_str() : "",
             runs.vars[v].name.c_str(), show(cell(runs, s, v, POS1)).c_str(), show(cell(runs, s, v, POS2)).c_str(),
             mean(&Metrics::ageMs, "%.0f").c_str(),
             (mean(&Metrics::voiceWorst, "%.1f") + " / " + mean(&Metrics::voiceMedian, "%.1f")).c_str(),
             mean(&Metrics::far, "%.1f").c_str(),
             (mean(&Metrics::latMs, "%.0f") + " / " + mean(&Metrics::latP95Ms, "%.0f")).c_str(),
             mean(&Metrics::holes, "%.0f").c_str(),
             (mean(&Metrics::leaseCopies, "%.1f") + " / " + mean(&Metrics::voiceCopies, "%.1f")).c_str(),
             mean(&Metrics::airPct, "%.0f").c_str());
    }
  }

  printHeader("Lease lateness with the talker", runs);
  printf("Lateness on the reference's clock by hops to it, ms; overran = still on the air when its slot\n"
         "closed, %% of lease beacons on the car's own clock; leases lost and forwards the forward slots\n"
         "refused, a minute.\n");
  printf("%-27s %-5s | %6s %6s %6s %6s | %8s | %5s %8s\n", "ride", "build", "hop 0", "hop 1", "hop 2", "hop 3+",
         "overran", "lost", "refused");
  for (size_t s = 0; s < runs.scens.size(); s++) {
    const Scen& sc = runs.scens[s];
    if (!sc.big() || !sc.talker) continue;
    for (size_t v : {b50, b51}) {
      auto late = [&](int h) { return num(cell(runs, s, v, [h](const Metrics& m) { return m.late[h]; }).mean); };
      const std::string l0 = late(0), l1 = late(1), l2 = late(2), l3 = late(3);
      printf("%-27s %-5s | %6s %6s %6s %6s | %8s | %5s %8s\n", v == b50 ? sc.name().c_str() : "",
             runs.vars[v].name.c_str(), l0.c_str(), l1.c_str(), l2.c_str(), l3.c_str(),
             num(cell(runs, s, v, [](const Metrics& m) { return m.overranPct; }).mean).c_str(),
             num(cell(runs, s, v, [](const Metrics& m) { return m.slotLosses; }).mean).c_str(),
             num(cell(runs, s, v, [](const Metrics& m) { return m.deferRefused; }).mean, "%.0f").c_str());
    }
  }
}

static void printVariantTables(const Runs& runs) {
  const size_t b51 = 1;
  printHeader("Every variant against 50: the position floor", runs);
  printf("Position cells: every ride above (25 cars and 2-8 cars, quiet and talking) and every hop bucket it\n"
         "has. Below: cells under 50 by more than %.0f SE and %.1f points. Worst: the furthest under of the cells\n"
         "beyond %.0f SE, whatever the size.\n", BELOW_SE, LEVEL_POINTS, BELOW_SE);
  printf("%-30s %5s %5s | %s\n", "variant", "cells", "below", "worst");
  for (size_t v = 1; v < runs.vars.size(); v++) {
    if (!runs.vars[v].detail) continue;
    const Below b = positionsAgainst50(runs, v);
    printf("%-30s %5d %5d | %s\n", runs.vars[v].name.c_str(), b.cells, b.below, b.worst.c_str());
  }

  printHeader("Fastrelay in the 25-car park with the talker", runs);
  printf("1-hop positions against 50 (difference/SE), voice median on time %% and voice copies, at p = 0 / 0.2 / 0.4.\n");
  printf("%-30s | %-10s %-10s %-10s | %-18s | %-18s\n", "variant", "1h p=0", "1h .2", "1h .4", "voice median", "voice copies");
  for (size_t v = 0; v < runs.vars.size(); v++) {
    if (runs.vars[v].family != Family::BASE && runs.vars[v].family != Family::RELAY) continue;
    std::string median, copies;
    printf("%-30s |", runs.vars[v].name.c_str());
    for (double p : {0.0, 0.2, 0.4}) {
      const size_t s = runs.scen(Layout::PARK, p, true);
      printf(" %-10s", diff(cell(runs, s, v, POS1)).c_str());
      median += (median.empty() ? "" : " / ") + num(cell(runs, s, v, [](const Metrics& m) { return m.voiceMedian; }).mean);
      copies += (copies.empty() ? "" : " / ") + num(cell(runs, s, v, [](const Metrics& m) { return m.voiceCopies; }).mean);
    }
    printf(" | %-18s | %-18s\n", median.c_str(), copies.c_str());
  }

  const size_t park2 = runs.scen(Layout::PARK, 0.2, true), line2 = runs.scen(Layout::LINE, 0.2, true);
  const size_t line4 = runs.scen(Layout::LINE, 0.4, true), line0 = runs.scen(Layout::LINE, 0, true);
  const size_t parkQ2 = runs.scen(Layout::PARK, 0.2, false);
  printHeader("Every variant, 25 cars with the talker", runs);
  printf("Park and line at p = 0.2 unless marked. Voice median over listeners within 3 hops; cars 8-16 of\n"
         "the line on time at p = 0 / 0.2 / 0.4, and their mean against 51 seed by seed (difference/SE); the line's\n"
         "2-hop positions at p = 0.4 and the park's 1-hop positions without a talker, against 50.\n");
  printf("%-30s | %6s %6s %6s %5s | %6s %-14s %-11s %4s %5s | %-10s %-10s\n", "variant", "pk 1h%", "vmed%", "vcopy",
         "air%", "ln 2h%", "8-16 %", "8-16 vs 51", "lat", "vcopy", "ln 2h p.4", "pk q 1h");
  for (size_t v = 0; v < runs.vars.size(); v++) {
    if (!runs.vars[v].detail) continue;
    auto mean = [&](size_t s, double Metrics::*field, const char* fmt) {
      return num(cell(runs, s, v, [field](const Metrics& m) { return m.*field; }).mean, fmt);
    };
    const std::string far = num(cell(runs, line0, v, FAR).mean, "%.0f") + " / " + num(cell(runs, line2, v, FAR).mean, "%.0f") +
                            " / " + num(cell(runs, line4, v, FAR).mean, "%.0f");
    const Cell farVs51 = farEnd(runs, v, b51);
    printf("%-30s | %6s %6s %6s %5s | %6s %-14s %-11s %4s %5s | %-10s %-10s\n", runs.vars[v].name.c_str(),
           num(cell(runs, park2, v, POS1).mean).c_str(), mean(park2, &Metrics::voiceMedian, "%.1f").c_str(),
           mean(park2, &Metrics::voiceCopies, "%.1f").c_str(), mean(park2, &Metrics::airPct, "%.0f").c_str(),
           num(cell(runs, line2, v, POS2).mean).c_str(), far.c_str(), (num(farVs51.mean) + " " + diff(farVs51)).c_str(),
           mean(line2, &Metrics::latMs, "%.0f").c_str(), mean(line2, &Metrics::voiceCopies, "%.1f").c_str(),
           diff(cell(runs, line4, v, POS2)).c_str(), diff(cell(runs, parkQ2, v, POS1)).c_str());
  }

  printHeader("The line car by car, p = 0.2 with the talker at car 0, on time %", runs);
  const size_t shown[] = {1, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 24};
  printf("%-30s |", "variant");
  for (size_t j : shown) printf(" %4u", (unsigned)j);
  printf("\n");
  for (size_t v = 0; v < runs.vars.size(); v++) {
    if (!runs.vars[v].detail) continue;
    printf("%-30s |", runs.vars[v].name.c_str());
    for (size_t j : shown) printf(" %4s", num(cell(runs, line2, v, [j](const Metrics& m) { return m.carOnTime[j]; }).mean, "%.0f").c_str());
    printf("\n");
  }

  const size_t line203 = runs.scen(Layout::LINE, 0.4, true, 203);
  printHeader("The line at p = 0.4 with a 203 B talker (prev)", runs);
  printf("On time, and with prev repair: a frame the next one's prev carries in time counts.\n");
  printf("%-30s | %8s %9s %9s\n", "variant", "8-16 %", "8-16 rep", "1-4 rep");
  for (size_t v = 0; v < runs.vars.size(); v++) {
    if (!runs.vars[v].detail) continue;
    printf("%-30s | %8s %9s %9s\n", runs.vars[v].name.c_str(), num(cell(runs, line203, v, FAR).mean).c_str(),
           num(cell(runs, line203, v, [](const Metrics& m) { return m.farRepaired; }).mean).c_str(),
           num(cell(runs, line203, v, [](const Metrics& m) { return m.nearRepaired; }).mean).c_str());
  }

  printHeader("25 cars without a talker, every variant against 50", runs);
  printf("Park 1-hop and line 2-hop positions, points against 50 (difference/SE).\n");
  printf("%-30s | %-10s %-10s %-10s | %-10s %-10s %-10s\n", "variant", "park p=0", "park .2", "park .4", "line p=0",
         "line .2", "line .4");
  for (size_t v = 1; v < runs.vars.size(); v++) {
    if (runs.vars[v].quietTwin != SIZE_MAX || !runs.vars[v].detail) continue;
    printf("%-30s |", runs.vars[v].name.c_str());
    for (double p : {0.0, 0.2, 0.4}) printf(" %-10s", diff(cell(runs, runs.scen(Layout::PARK, p, false), v, POS1)).c_str());
    printf(" |");
    for (double p : {0.0, 0.2, 0.4}) printf(" %-10s", diff(cell(runs, runs.scen(Layout::LINE, p, false), v, POS2)).c_str());
    printf("\n");
  }
}

static void printSmallRides(const Runs& runs) {
  std::vector<size_t> shown;
  for (const char* name : {"50", "51", "51, fastrelay off", "51, no ride-size floor", "51, voice skips above the floor"})
    shown.push_back(runs.find(name));
  printHeader("Small rides, 1 m apart, every link -40 dBm or stronger", runs);
  printf("Columns: 50, then 51, 51 with fastrelay off, 51 with no ride-size floor, 51 with voice skipped\n"
         "above the floor, each against 50 (difference/SE). 1-hop positions %%; voice on time, mean over the listeners %%;\n"
         "air copies per voice frame and per lease beacon.\n");
  printf("%-23s | %5s %-10s %-10s %-10s %-10s | %5s %-10s %-10s %-10s %-10s | %-24s | %-24s\n", "ride", "1h 50", "51",
         "off", "no floor", "voice 8", "vc 50", "51", "off", "no floor", "voice 8", "voice copies", "lease copies");
  for (size_t s = 0; s < runs.scens.size(); s++) {
    const Scen& sc = runs.scens[s];
    if (sc.big()) continue;
    printf("%-23s |", sc.name().c_str());
    for (size_t i = 0; i < shown.size(); i++) {
      const Cell c = cell(runs, s, shown[i], POS1);
      printf(i == 0 ? " %5s" : " %-10s", (i == 0 ? num(c.mean) : diff(c)).c_str());
    }
    printf(" |");
    for (size_t i = 0; i < shown.size(); i++) {
      const Cell c = cell(runs, s, shown[i], [](const Metrics& m) { return m.voiceMean; });
      printf(i == 0 ? " %5s" : " %-10s", (i == 0 ? num(c.mean) : diff(c)).c_str());
    }
    std::string copies, lease;
    for (size_t i = 0; i < shown.size(); i++) {
      copies += (i ? " " : "") + num(cell(runs, s, shown[i], [](const Metrics& m) { return m.voiceCopies; }).mean, "%.2f");
      lease += (i ? " " : "") + num(cell(runs, s, shown[i], [](const Metrics& m) { return m.leaseCopies; }).mean, "%.2f");
    }
    printf(" | %-24s | %-24s\n", copies.c_str(), lease.c_str());
  }
}

// "Materially worse" in step (b) below: more than this share above the least.
static const double AIR_MARGIN = 0.10;

static std::string names(const Runs& runs, const std::vector<size_t>& vs) {
  std::string out;
  for (size_t v : vs) out += (out.empty() ? "" : ", ") + runs.vars[v].name;
  return out.empty() ? "none" : out;
}

// The selection rule of plan 2B, Results, over 51 and the whole grid. Each step
// works on what the one before it kept:
// (a) no position cell below 50 (positionsAgainst50);
// (b) the park's lease copies and summed air without a talker, at p = 0 and at
//     p = 0.2, each within AIR_MARGIN of the least among them, since air is what
//     2C's voice windows will need;
// (c) the most on time at cars 8-16 of the line, and every variant level with
//     it (within LEVEL_POINTS or 2 SE);
// (d) the fewest voice copies in the park at p = 0.2 with the talker.
static void printSelection(const Runs& runs) {
  printHeader("Selection over the grid", runs);
  printf("Voice hops x copies x tie steps, taken through the rule in order: (a) below = position cells under 50\n"
         "by more than %.0f SE and %.1f points, none allowed; (b) the park without a talker, lease copies and air %%\n"
         "at p = 0 and 0.2, each no more than %.0f %% above the least of (a)'s; (c) far = cars 8-16 of the line on\n"
         "time, the mean over p = 0 / 0.2 / 0.4, against 51 (difference/SE), the most and those within %.1f points\n"
         "or 2 SE of it; (d) the fewest voice copies in the park at p = 0.2 with the talker.\n",
         BELOW_SE, LEVEL_POINTS, 100 * AIR_MARGIN, LEVEL_POINTS);
  printf("%-38s %5s | %-44s | %-11s %-12s | %5s %-10s | %6s\n", "variant", "below", "worst beyond 3 SE", "p=0 cp/air",
         "p=.2 cp/air", "far", "vs 51", "vcopy");
  const size_t parkQ0 = runs.scen(Layout::PARK, 0, false), parkQ2 = runs.scen(Layout::PARK, 0.2, false);
  const size_t park2 = runs.scen(Layout::PARK, 0.2, true);
  auto voiceCopies = [&](size_t v) { return cell(runs, park2, v, [](const Metrics& m) { return m.voiceCopies; }).mean; };
  // The four air numbers of (b): lease copies and air at p = 0, then at p = 0.2.
  auto air = [&](size_t v, int i) {
    const size_t s = i < 2 ? parkQ0 : parkQ2;
    return i % 2 == 0 ? cell(runs, s, v, [](const Metrics& m) { return m.leaseCopies; }).mean
                      : cell(runs, s, v, [](const Metrics& m) { return m.airPct; }).mean;
  };
  std::vector<size_t> keptA;
  for (size_t v = 1; v < runs.vars.size(); v++) {
    if (v != 1 && runs.vars[v].family != Family::GRID) continue;
    const Below b = positionsAgainst50(runs, v);
    const Cell far = farEnd(runs, v, 1);
    printf("%-38s %5d | %-44s | %-11s %-12s | %5s %-10s | %6s\n", runs.vars[v].name.c_str(), b.below, b.worst.c_str(),
           (num(air(v, 0), "%.2f") + " / " + num(air(v, 1), "%.0f")).c_str(),
           (num(air(v, 2), "%.2f") + " / " + num(air(v, 3), "%.0f")).c_str(), num(far.mean).c_str(), diff(far).c_str(),
           num(voiceCopies(v), "%.2f").c_str());
    if (b.below == 0) keptA.push_back(v);
  }
  printf("  (a) keeps: %s\n", names(runs, keptA).c_str());
  if (keptA.empty()) return;

  std::vector<size_t> keptB;
  double least[4];
  for (int i = 0; i < 4; i++) {
    least[i] = air(keptA[0], i);
    for (size_t v : keptA) least[i] = std::min(least[i], air(v, i));
  }
  for (size_t v : keptA) {
    bool within = true;
    for (int i = 0; i < 4; i++) within = within && air(v, i) <= least[i] * (1 + AIR_MARGIN);
    if (within) keptB.push_back(v);
  }
  printf("  (b) least of (a)'s: p = 0 %.2f copies, %.0f %% air; p = 0.2 %.2f copies, %.0f %% air; keeps: %s\n", least[0],
         least[1], least[2], least[3], names(runs, keptB).c_str());

  size_t most = keptB[0];
  for (size_t v : keptB)
    if (farEnd(runs, v, 1).mean > farEnd(runs, most, 1).mean) most = v;
  std::vector<size_t> keptC;
  for (size_t v : keptB) {
    const Cell gap = farEnd(runs, v, most);
    if (v == most || !(gap.diff <= -2 * gap.se && gap.diff <= -LEVEL_POINTS)) keptC.push_back(v);
  }
  printf("  (c) most at the far end: %s; level with it: %s\n", runs.vars[most].name.c_str(), names(runs, keptC).c_str());

  size_t chosen = keptC[0];
  for (size_t v : keptC)
    if (voiceCopies(v) < voiceCopies(chosen)) chosen = v;
  printf("  (d) chosen: %s\n", runs.vars[chosen].name.c_str());
}

// ---- The selection again, on fresh seeds ------------------------------------------
//
// The cells the rule turns on sit a few SE from its margins, so its nearest
// rivals run again on seeds from 6000, which none of the choices were made on:
// the tie counts either side of the firmware's, and two voice copies at its hops
// and at one hop more. -D TOUGE_RIDESIM_CHECK_SEEDS=n, 4 by default; the plan's
// numbers used 512.
#ifndef TOUGE_RIDESIM_CHECK_SEEDS
#define TOUGE_RIDESIM_CHECK_SEEDS 4
#endif

static std::vector<Variant> checkVariants() {
  std::vector<Variant> v;
  v.push_back({"50", build50(), Family::BASE, SIZE_MAX, true});
  v.push_back({"51", build51(), Family::BASE, SIZE_MAX, true});
  for (uint32_t ties : {FORWARD_TIE_STEPS - 1, FORWARD_TIE_STEPS + 1}) {
    Policy p = build51();
    p.tieSteps = ties;
    v.push_back({gridName(VOICE_HOPS, VOICE_SUPPRESS_AFTER, ties), p, Family::GRID, SIZE_MAX, true});
  }
  for (uint8_t hops : {VOICE_HOPS, (uint8_t)(VOICE_HOPS + 1)}) {
    Policy p = build51();
    p.voiceHops = hops;
    p.voiceSuppressAfter = 2;
    v.push_back({gridName(hops, 2, FORWARD_TIE_STEPS), p, Family::GRID, 1, true});
  }
  return v;
}

// Every position cell more than BELOW_SE under 50, '*' where it also passes LEVEL_POINTS.
static void printCellsBelow(const Runs& runs) {
  printHeader("Position cells beyond 3 SE under 50", runs);
  for (size_t v = 1; v < runs.vars.size(); v++) {
    printf("%s:", runs.vars[v].name.c_str());
    int shown = 0;
    for (size_t s = 0; s < runs.scens.size(); s++) {
      for (int b = 0; b < 3; b++) {
        const Cell c = cell(runs, s, v, [b](const Metrics& m) { return m.pos[b]; });
        if (isnan(c.diff) || !(c.diff < -BELOW_SE * c.se)) continue;
        printf("%s %s, %d-hop %s%s", shown++ ? ";" : "", runs.scens[s].name().c_str(), b + 1, diff(c).c_str(),
               c.diff < -LEVEL_POINTS ? " *" : "");
      }
    }
    printf("%s\n", shown ? "" : " none");
  }
}

void test_report_build_51_against_50() {
  const auto t0 = std::chrono::steady_clock::now();
  const Runs runs = runVariants(scenarios(), variants(), TOUGE_RIDESIM_SEEDS);
  printf("\nride sim: build 51 against 50 and the variants plan 2B compares, %s tables, %d seeds.\n", tablesName(),
         runs.seeds);
  printRideTables(runs);
  printVariantTables(runs);
  printSmallRides(runs);
  printSelection(runs);
  const Runs check = runVariants(scenarios(), checkVariants(), TOUGE_RIDESIM_CHECK_SEEDS, 6000);
  printCellsBelow(check);
  printSelection(check);
  const double totalS = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  double cpu = 0;
  size_t n = 0;
  for (const Runs* r : {&runs, &check})
    for (const auto& byVariant : r->at)
      for (const auto& seeds : byVariant)
        for (const Metrics& m : seeds) {
          cpu += m.wallMs;
          n++;
        }
  printf("\n%u runs, %.0f s of CPU, %.0f s on %u cores\n", (unsigned)n, cpu / 1000, totalS,
         std::max(1u, std::thread::hardware_concurrency()));
  fflush(stdout);
}
