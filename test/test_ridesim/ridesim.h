#pragma once
//
// A ride on the 2.4 GHz lane, car by car and frame by frame: the realistic
// simulator of docs/plans/2026-09-27-voice-coding-plan.md, 2B.
//
// Each car runs the firmware's platform-free modules (Mesh, Schedule, OwnFix,
// frame and position encoding, forward.h's planForward and markRelayer,
// voicetest's frames and meter, FastRelay) through Node, which follows
// TougeFastModule's pass line by line: runOnce, drainRadio, beacon and
// sendExtraBeacon, transmit, sendDeferred, sendTestVoice. ridesim_policy.h
// swaps in build 50's forwarding. Frames are not sealed (cipher.cpp is
// board-only): transmit appends TAG_LEN zero bytes where the tag goes and
// drainRadio strips them, so every frame is its real length on the air and in
// Mesh's forward slots.
//
// Around the cars: where they are, path loss, airtime by length, the 802.11
// MAC's carrier sense and backoff, half duplex, collisions with capture, and
// each radio's receive and transmit queues. Events run in microseconds.

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <deque>
#include <queue>
#include <unordered_map>
#include <vector>
#include "fastrelay.h"
#include "frame.h"
#include "hmac.h"
#include "hop.h"
#include "mesh.h"
#include "ownfix.h"
#include "ram.h"
#include "ridesim_policy.h"
#include "schedule.h"
#include "voicetest.h"

namespace ridesim {

using namespace touge;
typedef uint64_t Us;

// ---- The physical model: every assumption, with its default ------------------
//
// Geometry
//   Car park: 25 cars on a 5 x 5 grid at 10 m, each moved up to 2 m either way,
//     so every pair is within about 57 m. The talker is the middle car.
//   Line: 25 cars along a straight road 27 m apart. The 3rd neighbour is at
//     -87 dBm, the 4th at -91 (a third of its frames lost at the edge) and the
//     5th at -94 (four in five lost), so a car hears about 4 each way. The
//     talker is car 0, an end.
//   Bench: 3 cars 1 m apart, every link -40 dBm or stronger, as the bench boards
//     were: all clamp to forwardDelayMs's near end.
// Signal
//   Mean RSSI = 20 dBm TX (espnow.cpp asks for 80 quarter-dBm) - 10 dB for PCB
//   antennas inside cars (a guess) - 40 dB at 1 m (free space, 2.44 GHz)
//   - 10 n log10(d) with n = 3 (ground level, bodies in the way; a guess):
//   -30 dBm at 1 m, -60 at 10 m, -83 at 57 m. Each link gets a fixed shadowing
//   draw, N(0, 3 dB), the same both ways. Each frame at each receiver gets
//   N(0, 2 dB) of fading on top, and that is the RSSI the firmware reads
//   (forwardDelayMs orders on it).
//   Floor -95 dBm (FORWARD_FAR_DBM): weaker is neither received nor sensed.
// Loss
//   Every receiver draws every frame on its own: it survives with probability
//   ((1 - p)(1 - e))^(airtime / airtime of an 89 B position). p is the base
//   loss (BLE sharing the radio: 35-50 % each way on the bench). e is edge
//   loss, 0 at 6 dB over the floor and above, rising linearly to 1 at the
//   floor, from the link's mean RSSI. Both scale with airtime, because a BLE
//   slot or a bit error anywhere in a frame loses it: p = 0.4 on a position is
//   0.49 on a 129 B test voice frame and 0.64 on a 225 B one.
// Airtime
//   192 us of preamble and PLCP header (802.11b long preamble; Espressif does
//   not publish LR's) + (wire bytes + 43 B of action frame, vendor element and
//   FCS) x 8 at the PHY rate. LR 250 kbit/s by default: a position (89 B) 4.4
//   ms, test voice 5.7 ms (129 B) and 8.8 ms (225 B). 1 Mbit/s: 1.2, 1.6, 2.3.
// MAC (an ESP-NOW broadcast is 802.11 DCF with no ACK and no retry)
//   The frame at the head of the TX queue waits for DIFS (50 us) of idle, then
//   0-15 slots of 20 us, the count frozen while the medium is busy. A slot
//   counts only once it has passed idle, so the one a frame interrupts is
//   counted again after it. The medium is busy while any frame at or above the
//   floor is on the air; a car senses a frame 15 us after it starts, so two
//   backoffs ending that close both go.
//   Half duplex: a car receives nothing while it transmits. At a receiver a
//   frame survives an overlap only 10 dB or more above the sum of every other
//   frame overlapping it there (capture), whichever started first.
// Radio queues
//   TX: 8 buffers counting the frame on the air (espnow.cpp, dynamic_tx_buf_num
//   on the V3); esp_now_send is refused when all are taken. RX: 16 on the lean
//   build (V3) and 32 roomy (V4), espnow.cpp's RX_DEPTH; the receive callback
//   drops a frame when full. rx.rxMs is the receiver's millis() as the frame
//   ends.
// The pass
//   TougeFastModule::runOnce starts TICK_MS after the last one finished, plus
//   0-1 ms of main-loop jitter. drainRadio polls a frame at a time, each costing
//   150 us (HMAC, AES, decode, note, rebuild on an ESP32-S3; a guess), so a
//   frame that lands mid-drain is polled in the same pass with an rx.rxMs after
//   the pass's nowMs. The pass costs 50 us more, so what runs after drainRadio
//   reads a later millis(). Optional stalls of 95-140 ms (a V4 showed them) at
//   random. Clocks run +-40 ppm from a random offset.
// Traffic
//   Parked: the phone feeds a fix every second, same place, 100 ms after it is
//   measured, so lease beacons at 1 Hz and no extras. Moving: a fix every
//   250 ms, 5 m on, so the 20 m gate and extras run. Names every 30 s.
//   The test talker sends voicetest frames of 107 B (or 203 B) every 60 ms.
struct Model {
  double txDbm = 20;
  double antennaDb = -10;
  double lossAt1mDb = 40;
  double exponent = 3.0;
  double shadowSigmaDb = 3;
  double fadeSigmaDb = 2;
  double floorDbm = FORWARD_FAR_DBM;
  double edgeDb = 6;
  bool lossScalesWithAirtime = true;
  double captureDb = 10;
  double parkPitchM = 10;
  double parkJitterM = 2;
  double lineSpacingM = 27;
  double benchSpacingM = 1;
  uint32_t difsUs = 50;
  uint32_t slotUs = 20;
  uint32_t cwSlots = 15;
  uint32_t ccaDelayUs = 15;
  uint32_t preambleUs = 192;
  uint32_t macOverheadBytes = 43;
  size_t txBuffers = 8;
#if TOUGE_LEAN_RAM
  size_t rxDepth = 16;
#else
  size_t rxDepth = 32;
#endif
  uint32_t passJitterUs = 1000;
  uint32_t passBaseUs = 50;
  uint32_t drainCostUs = 150;
  uint32_t stallMinMs = 95;
  uint32_t stallMaxMs = 140;
  int32_t driftPpm = 40;
  uint32_t parkedFixMs = 1000;
  uint32_t movingFixMs = 250;
  double movingStepM = 5;
  uint32_t phoneDelayMs = 100;
  uint32_t voiceDeadlineMs = 250;
  // Pairs this strong are one hop apart in the graph the tables bucket by.
  double graphMarginDb = 3;
};

enum class Layout : uint8_t { BENCH, PARK, LINE };

struct Phy {
  const char* name;
  uint32_t bitsPerSec;
};
inline Phy lr250k() { return {"LR250k", 250000}; }
inline Phy rate1M() { return {"1M", 1000000}; }

struct Config {
  const char* label = "";
  Layout layout = Layout::PARK;
  size_t cars = 25;
  double p = 0;
  bool talker = false;
  uint8_t talkerBytes = TEST_VOICE_DEFAULT_BYTES;
  int talkerCar = -1;  // -1: the middle of the park, car 0 of the line
  Phy phy = lr250k();
  uint32_t seed = 1;
  uint32_t bootSpreadMs = 3000;
  uint32_t warmupMs = 15000;
  uint32_t measureMs = 60000;
  bool moving = false;
  uint32_t stallEveryMs = 0;  // mean gap between stalls, 0 for none
  Policy policy;
  Model model;
};

// ---- Deterministic randomness ------------------------------------------------

struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed = 1) : s(seed) {}
  uint64_t next() {
    uint64_t z = (s += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }
  uint32_t u32() { return (uint32_t)(next() >> 32); }
  double unit() { return (double)(next() >> 11) * (1.0 / 9007199254740992.0); }
  uint32_t below(uint32_t n) { return n == 0 ? 0 : (uint32_t)(unit() * n); }
  double gauss() {
    double u = unit();
    if (u < 1e-12) u = 1e-12;
    return sqrt(-2.0 * log(u)) * cos(6.283185307179586 * unit());
  }
};

// ---- What goes on the air ------------------------------------------------------

enum TxKind : uint8_t { TX_LEASE, TX_UNLEASED, TX_EXTRA, TX_FWD_POS, TX_VOICE, TX_FWD_VOICE, TX_KINDS };
inline bool original(uint8_t kind) { return kind != TX_FWD_POS && kind != TX_FWD_VOICE; }

struct TxMeta {
  uint8_t kind = TX_LEASE;
  uint8_t hop = REF_UNREACHABLE;  // hopsToReference as a lease beacon was queued
  uint8_t slot = SLOT_NONE;
};

struct TxJob {
  uint8_t wire[FRAME_MAX];
  uint16_t len = 0;
  TxMeta meta;
};

struct Air {
  uint64_t serial = 0;
  size_t car = 0;
  Us start = 0, end = 0;
  TxJob job;
};

// FastRx as the receive callback fills it.
struct RxFrame {
  uint8_t data[FRAME_MAX];
  uint16_t len = 0;
  int8_t rssi = 0;
  uint32_t rxMs = 0;
  uint8_t chan = 0;
};

struct Radio {
  bool on = false;
  std::deque<TxJob> txq;  // the head is on the air, or next
  std::deque<RxFrame> rxq;
  bool onAir = false;
  enum : uint8_t { MAC_IDLE, MAC_WAIT, MAC_COUNT } mac = MAC_IDLE;
  Us backoffUs = 0, countFrom = 0, attemptAt = 0, lastIdle = 0;
  uint64_t token = 0;
  uint32_t busyDepth = 0;
  Us busySince = 0, busyTotal = 0;
};

// ---- Results -----------------------------------------------------------------

// Graph distance buckets: 1, 2, 3 and 4+ hops over links at floor + graphMarginDb.
static const size_t BUCKETS = 4;
inline size_t bucketOf(uint8_t d) { return d == 0 ? 0 : (d > BUCKETS ? BUCKETS - 1 : d - 1); }

struct CarStats {
  uint8_t fromTalker = 0;  // graph hops from the talker
  // positions from origins within 3 graph hops
  uint32_t posSent = 0, posGot = 0;
  double ageSumMs = 0;
  uint32_t ageN = 0;
  // voice
  uint32_t expected = 0, delivered = 0, onTime = 0, repaired = 0;
  double latSumMs = 0, latP95Ms = 0;
  double jitterSumMs = 0;
  uint32_t jitterN = 0;
  uint32_t holeRuns = 0, longestHole = 0;
  double firstHopsSum = 0;
  double meterMeanMs = -1;  // VoiceMeter's dm, what {"vt"} would report
  double busyPct = 0;
};

struct Late {
  std::vector<int32_t> rideUs, ownUs;
  // Still on the air when the slot closed, by the reference's clock and by the car's own.
  uint32_t overranRide = 0, overranOwn = 0;
};

struct Result {
  Config cfg;
  size_t talker = SIZE_MAX;
  std::vector<CarStats> cars;
  // positions
  double posSent[BUCKETS] = {}, posGot[BUCKETS] = {};
  std::vector<uint32_t> ageHist;  // 10 ms bins, pairs within 3 hops, sampled every 100 ms
  double ageSumMs = 0;
  uint32_t ageN = 0;
  double leaseCopies = 0;
  // voice
  uint32_t voiceAttempted = 0, voiceRefused = 0;
  double voiceCopies = 0;
  uint32_t voiceCopiesMax = 0;
  // air
  double airUs[TX_KINDS] = {};
  uint32_t txCount[TX_KINDS] = {};
  uint32_t txRefused[TX_KINDS] = {};
  // schedule
  Late late[7];  // by hops to the reference, 6 for unreachable
  uint32_t slotLosses = 0, leaseClash = 0, leaseHit = 0, otherCollisions = 0, halfDuplex = 0, randomLoss = 0;
  uint32_t sharedSlotSamples = 0;
  size_t leased = 0, references = 0;
  // Power-on to one clock: when one reference and every epoch within SYNCED_MS
  // of its had held for SYNC_HOLD_MS. 0: never, within the warm-up cap.
  uint32_t syncedAtMs = 0;
  uint32_t unsyncedSamples = 0;  // 100 ms samples in the window that were not
  // queues
  uint32_t leaseRetries = 0, rxDrops = 0, deferRefused = 0, suppressed = 0, maxPassGapMs = 0;
  // Lease beacons polled mid-drain with rx.rxMs after the pass's nowMs, and of
  // those, how many the slot map for that same pass left out (stampAgeMs).
  uint32_t stampsAhead = 0, stampsAheadUnmapped = 0;
  FastRelaySkips relayPositions, relayVoice;  // FastRelay verdicts, when the policy judges
  double wallMs = 0;

  double seconds() const { return cfg.measureMs / 1000.0; }
  double airPct(uint8_t k) const { return airUs[k] / (cfg.measureMs * 10.0); }
  double airPctTotal() const {
    double t = 0;
    for (uint8_t k = 0; k < TX_KINDS; k++) t += airPct(k);
    return t;
  }
  double ageP(double q) const;
};

inline double Result::ageP(double q) const {
  uint64_t total = 0;
  for (uint32_t c : ageHist) total += c;
  if (total == 0) return 0;
  uint64_t seen = 0;
  for (size_t i = 0; i < ageHist.size(); i++) {
    seen += ageHist[i];
    if (seen >= q * total) return i * 10.0 + 5;
  }
  return ageHist.size() * 10.0;
}

inline double percentile(std::vector<double> v, double q) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  size_t i = (size_t)(q * (v.size() - 1) + 0.5);
  return v[i < v.size() ? i : v.size() - 1];
}

inline double percentileI(std::vector<int32_t> v, double q) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  size_t i = (size_t)(q * (v.size() - 1) + 0.5);
  return v[i < v.size() ? i : v.size() - 1];
}

// ---- One car's firmware ------------------------------------------------------

struct World;

// The module's stats_ the tables read. Radio refusals are counted by the world.
struct Counters {
  uint32_t leaseRefused = 0;     // a lease beacon the radio refused, retried in the slot
  uint32_t forwardsRefused = 0;  // Mesh::defer had no slot
};

// TougeFastModule's lane, member for member where the pass reads it. Methods
// carry the module's names and follow its order; comments say where it differs.
struct Node {
  World* w = nullptr;
  size_t car = 0;
  Policy pol;
  uint32_t nodeId_ = 0;
  uint8_t chanByte_ = 0xA7;                    // FastNet::chanByte, one ride
  static constexpr uint8_t RADIO_CHANNEL = 1;  // FAST_HOME_INDEX: channel 1, never hops
  char name_[16] = {0};

  Mesh mesh_;
  Schedule schedule_;
  FastRelay fastRelay_;
  FastRelaySkips positionSkips_, voiceSkips_;
  VoiceMeter voiceMeter_;
  OwnFix ownFix_;

  uint32_t lastBeaconMs_ = 0, nextBeaconMs_ = 0, lastNameMs_ = 0, lastHeardMs_ = 0;
  uint32_t lastSyncSrc_ = 0, lastSyncId_ = 0;
  bool wantBeacon_ = false;
  uint8_t announcedSlot_ = SLOT_NONE;
  int32_t sentLat_ = 0, sentLon_ = 0;
  bool sentOnce_ = false;
  uint32_t lastPassMs_ = 0, passGapMaxMs_ = 0;
  int drained_ = 0;  // frames drainRadio has polled this pass

  bool talkerOn_ = false;
  uint8_t talkerBytes_ = TEST_VOICE_DEFAULT_BYTES;
  uint8_t testSession_ = 0;
  uint32_t testSeq_ = 0, nextTestMs_ = 0, testTalkerAtMs_ = 0;

  Counters c;
  uint32_t nowMs_ = 0;
  int64_t fixFed_ = -1;  // the last phone fix fed, by index

  void boot();
  void passBegin();
  bool drainNext();
  void drainFrame(const RxFrame& rx, uint32_t nowMs);
  void passEnd();
  size_t fastNeighbours(uint32_t nowMs) const;
  bool beacon(uint32_t nowMs);
  bool sendExtraBeacon(uint32_t nowMs);
  void fillBeacon(Position& p, uint32_t nowMs);
  bool transmit(uint8_t type, const uint8_t* body, size_t len, uint8_t hops, TxMeta meta);
  void sendDeferred(uint32_t nowMs);
  void sendTestVoice(uint32_t nowMs);
  void startTalker(uint8_t bytes);
  uint32_t millis() const;  // a fresh millis() inside the pass
  void feedPhone();
};

// ---- The world ---------------------------------------------------------------

// A frame as its origin queued it, and what became of it.
struct Original {
  size_t car = 0;
  uint8_t kind = 0;
  uint16_t copies = 0;  // transmissions, the origin's own included
  uint32_t heardMask = 0;
  bool counted = false;  // queued inside the measured window
};

enum EvKind : uint8_t { EV_BOOT, EV_PASS, EV_DRAIN, EV_PASS_END, EV_MAC, EV_TX_END, EV_SAMPLE };

struct Ev {
  Us t;
  uint64_t seq;
  uint8_t kind;
  size_t car;
  uint64_t arg;
};
struct EvLater {
  bool operator()(const Ev& a, const Ev& b) const { return a.t != b.t ? a.t > b.t : a.seq > b.seq; }
};

// Epoch seconds at sim time 0, for fix times.
static const uint32_t EPOCH_SEC = 1790000000u;
static const Us SAMPLE_US = 100000;
static const Us AIR_KEEP_US = 20000;
// One clock: every car names the same reference and runs its epoch within this
// of the reference's (a hop adds 5-8 ms, review B5). Measuring starts once that
// has held for SYNC_HOLD_MS, or at the cap: at 40 % loss the head of a line
// took 40 s to join.
static const int64_t SYNCED_MS = 60;
static const uint32_t SYNC_HOLD_MS = 3000;
static const uint32_t WARMUP_CAP_MS = 120000;

struct World {
  Config cfg;
  Model m;
  size_t n = 0;
  std::vector<Node> nodes;
  std::vector<Radio> radios;
  std::vector<double> x, y;
  std::vector<std::vector<double>> rssi, mw, surviveBase;
  std::vector<std::vector<uint8_t>> senses, hopDist;
  Rng rngWorld, rngChan, rngMac, rngFw, rngPass;
  std::priority_queue<Ev, std::vector<Ev>, EvLater> events;
  uint64_t evSeq = 0;
  Us now = 0;
  std::vector<Air> air;
  uint64_t airSerial = 0;
  uint32_t airPosUs = 1;

  std::vector<Us> bootAt, fixPhaseUs;
  std::vector<int32_t> ppm;
  std::vector<uint32_t> localBaseMs;
  std::vector<Us> nextStallUs;
  std::unordered_map<uint32_t, size_t> carOf;
  size_t talker = SIZE_MAX;

  bool measuring = false;
  Us markUs = 0, endUs = 0;
  std::vector<Original> originals;
  std::unordered_map<uint64_t, size_t> originalAt;
  std::vector<Us> voiceTriedUs;                 // by seq, when sendTestVoice ran; 0 outside the window
  std::vector<uint8_t> voiceSent;               // by seq: the radio took it
  std::vector<std::vector<Us>> voiceFirstUs;    // [car][seq]
  std::vector<std::vector<uint8_t>> voiceHops;  // [car][seq], first copy
  std::vector<std::vector<Us>> newestFixUs;     // [receiver][origin]
  std::vector<Counters> countersAtMark;
  std::vector<uint32_t> lossesAtMark, suppressedAtMark;
  Result result;

  explicit World(const Config& c);
  void runAll();

  // clocks
  int64_t localUs(size_t car, Us t) const;
  uint32_t millisOf(size_t car, Us t) const { return (uint32_t)(localUs(car, t) / 1000); }
  bool phaseUs(size_t car, Us t, int64_t& out) const;
  uint32_t espRandom() { return rngFw.u32(); }
  uint32_t airtimeUs(size_t wireLen) const {
    return m.preambleUs + (uint32_t)(((uint64_t)(wireLen + m.macOverheadBytes) * 8 * 1000000) / cfg.phy.bitsPerSec);
  }
  // the phone's fix
  Us fixMeasuredUs(size_t car, int64_t k) const { return fixPhaseUs[car] + (Us)k * fixEveryUs(); }
  Us fixEveryUs() const { return (Us)(cfg.moving ? m.movingFixMs : m.parkedFixMs) * 1000; }
  Fix fixAt(size_t car, int64_t k) const;

  void push(Us t, uint8_t kind, size_t car, uint64_t arg = 0) { events.push(Ev{t, evSeq++, kind, car, arg}); }

  // radio
  bool radioSend(size_t car, const uint8_t* wire, size_t len, const TxMeta& meta);
  bool poll(size_t car, RxFrame& out);
  void drainStep(size_t car);
  bool busyAt(size_t car, Us t) const;
  void startContention(size_t car);
  void tryCount(size_t car);
  void startTx(size_t car);
  void endTx(uint64_t serial);
  void sample();

  // metrics hooks
  void noteFirstSight(size_t receiver, const Frame& f, const uint8_t* body, size_t bodyLen, uint8_t hopsAway);
  void noteVoiceAttempt(size_t car, uint32_t seq, bool sent);
  void noteLeaseHeard(size_t car, uint32_t src, uint8_t slot, uint32_t rxMs, uint32_t nowMs);
  void mark();
  void finish();
  bool oneClock() const;

  void build();
  void runUntil(Us t);
};

// ---- Node ----------------------------------------------------------------------

inline uint32_t Node::millis() const { return w->millisOf(car, w->now); }

inline void Node::boot() {
  // syncChannel, on a key that is already the ride's.
  mesh_.reset();
  schedule_.reset();
  fastRelay_.reset();
  voiceMeter_.reset();
  positionSkips_ = FastRelaySkips();
  voiceSkips_ = FastRelaySkips();
  mesh_.seedIds(w->rngWorld.u32());
  feedPhone();
  const uint32_t t = millis();
  schedule_.rebuild(nodeId_, false, mesh_.riders(), MAX_RIDERS, t, ownFix_.fresh(t));
  wantBeacon_ = false;
  sentOnce_ = false;
  announcedSlot_ = SLOT_NONE;
  nextBeaconMs_ = 0;
  lastHeardMs_ = t;
  snprintf(name_, sizeof(name_), "rider-%02u", (unsigned)car);
}

// notePhoneFix: the phone's newest fix, once it has reached the radio.
inline void Node::feedPhone() {
  const Us firstArrives = w->fixPhaseUs[car] + (Us)w->m.phoneDelayMs * 1000;
  if (w->now < firstArrives) return;
  const int64_t k = (int64_t)((w->now - firstArrives) / w->fixEveryUs());
  if (k == fixFed_) return;
  fixFed_ = k;
  ownFix_.fromPhone(w->fixAt(car, k), millis(), w->espRandom());
}

inline void Node::startTalker(uint8_t bytes) {
  talkerOn_ = true;
  talkerBytes_ = bytes;
  testSession_ = (uint8_t)(testSession_ + 1 + w->espRandom() % 255);
  testSeq_ = 1;
  nextTestMs_ = 0;
  testTalkerAtMs_ = millis();
}

// runOnce up to drainRadio, which drainNext then runs a frame at a time. The
// rest runs in passEnd, after what the drain cost.
inline void Node::passBegin() {
  nowMs_ = millis();
  feedPhone();
  if (lastPassMs_ != 0 && (uint32_t)(nowMs_ - lastPassMs_) > passGapMaxMs_) passGapMaxMs_ = nowMs_ - lastPassMs_;
  lastPassMs_ = nowMs_;
  drained_ = 0;
}

inline void Node::passEnd() {
  const uint32_t now = nowMs_;
  const bool lost = (uint32_t)(now - lastHeardMs_) >= fw::LOST_MS;
  const bool beaconRetrying = beacon(now);
  if (!lost && !beaconRetrying) {
    sendDeferred(now);
    sendTestVoice(now);
  }
  mesh_.age(now);
}

inline size_t Node::fastNeighbours(uint32_t nowMs) const {
  size_t count = 0;
  const Rider* r = mesh_.riders();
  for (size_t i = 0; i < MAX_RIDERS; i++) {
    if (!r[i].used || r[i].via != HEARD_FAST) continue;
    if ((uint32_t)(nowMs - r[i].atMs) >= fw::FAST_PRECEDENCE_MS) continue;
    if (r[i].chan != RADIO_CHANNEL) continue;
    count++;
  }
  return count;
}

// One turn of drainRadio's loop: false once the budget is spent or nothing is
// queued. The world calls it a frame at a time, drainCostUs apart.
inline bool Node::drainNext() {
  RxFrame rx;
  if (drained_ >= fw::DRAIN_BUDGET || !w->poll(car, rx)) return false;
  drained_++;
  drainFrame(rx, nowMs_);
  return true;
}

// drainRadio's loop body, for one frame.
inline void Node::drainFrame(const RxFrame& rx, uint32_t nowMs) {
  Frame f;
  if (!decodeFrame(rx.data, rx.len, f)) return;
  if (f.chan != chanByte_) return;
  if (f.src == nodeId_) return;
  if (f.len > FRAME_MAX_PAYLOAD) return;

  uint8_t sealed[FRAME_MAX_PAYLOAD];
  memcpy(sealed, f.payload, f.len);
  // unseal() stands in: the tag's bytes off the end.
  if (f.len <= TAG_LEN) return;
  const uint8_t* body = sealed;
  const size_t bodyLen = f.len - TAG_LEN;
  lastHeardMs_ = nowMs;

  Position syncBeacon;
  if (schedule_.takesClockFrom(f.src) && adapt::originPosition(pol, f) &&
      !(f.src == lastSyncSrc_ && f.id == lastSyncId_) && decodePosition(body, bodyLen, syncBeacon) &&
      !syncBeacon.extra) {
    lastSyncSrc_ = f.src;
    lastSyncId_ = f.id;
    schedule_.syncTo(rx.rxMs, syncBeacon.slot, fw::SYNC_BIAS_MS, syncBeacon.layout);
  }

  if (!mesh_.firstSight(f.src, f.id, nowMs)) return;
  w->noteFirstSight(car, f, body, bodyLen, adapt::hopsAway(pol, f));

  if (f.type == FRAME_POSITION) {
    Position p;
    if (decodePosition(body, bodyLen, p)) {
      const uint8_t hopsAway = p.extra ? 0 : adapt::hopsAway(pol, f);
      mesh_.note(f.src, p, HEARD_FAST, rx.rssi, hopsAway, nowMs, rx.chan);
      if (adapt::originPosition(pol, f) && !p.extra) {
        schedule_.heardBeacon(f.src, p, adapt::beaconStampMs(pol, nowMs, rx.rxMs));
        w->noteLeaseHeard(car, f.src, p.slot, rx.rxMs, nowMs);
      }
      adapt::feedMap(pol, fastRelay_, f.src, p, rx.rxMs);
      schedule_.rebuild(nodeId_, false, mesh_.riders(), MAX_RIDERS, nowMs, ownFix_.fresh(nowMs));
    }
  }

  uint8_t testSession = 0;
  uint32_t testSeq = 0;
  uint16_t sentPhase = 0;
  if (f.type == FRAME_VOICE && decodeTestVoice(body, bodyLen, testSession, testSeq, sentPhase)) {
    uint32_t phase = 0;
    const uint16_t heardPhase = schedule_.phaseAt(rx.rxMs, phase) ? (uint16_t)phase : TEST_VOICE_NO_PHASE;
    voiceMeter_.heard(f.src, testSession, testSeq, adapt::hopsAway(pol, f), sentPhase, heardPhase, nowMs);
  }
  // inject() has no cost here beyond the drain's.

  if (f.hops == 0) return;
  const ForwardPlan plan = adapt::planForward(pol, f, fastRelay_, nodeId_, mesh_.riders(), rx.rxMs, nowMs, rx.rssi,
                                              fastNeighbours(nowMs), w->espRandom());
  // Build 50 had no verdicts to count.
  if (adapt::b51(pol)) (f.type == FRAME_VOICE ? voiceSkips_ : positionSkips_).note(plan.verdict, plan.relayerUnplaced);
  if (plan.skip) return;
  Frame fwd = f;
  fwd.hops = f.hops - 1;
  fwd.payload = sealed;
  uint8_t wire[FRAME_MAX];
  const size_t n = encodeFrame(fwd, wire, sizeof(wire));
  if (n == 0) return;
  if (!mesh_.defer(wire, n, f.src, f.id, plan.dueMs, plan.suppressAfter)) c.forwardsRefused++;
}

inline bool Node::transmit(uint8_t type, const uint8_t* body, size_t len, uint8_t hops, TxMeta meta) {
  if (len > FRAME_MAX_BODY) return false;
  uint8_t payload[FRAME_MAX_PAYLOAD];
  memcpy(payload, body, len);
  Frame f;
  f.type = type;
  f.src = nodeId_;
  f.id = mesh_.nextId();
  f.hops = hops;
  f.chan = chanByte_;
  // seal() stands in: a tag's worth of bytes on the end, so the length is real.
  memset(payload + len, 0, TAG_LEN);
  f.payload = payload;
  f.len = (uint16_t)(len + TAG_LEN);
  uint8_t wire[FRAME_MAX];
  const size_t n = encodeFrame(f, wire, sizeof(wire));
  if (n == 0) return false;
  if (!w->radioSend(car, wire, n, meta)) return false;
  mesh_.firstSight(f.src, f.id, millis());
  return true;
}

inline void Node::fillBeacon(Position& p, uint32_t nowMs) {
  const Fix& fix = ownFix_.fix();
  p.lat = fix.lat;
  p.lon = fix.lon;
  p.fix = ownFix_.id();
  p.headingDeg = (uint16_t)(fix.trackE5 / 100000);
  p.speedMph = 0;
  p.hasFix = true;
  p.phoneAttached = fix.external;
  p.clockLocked = false;  // no GPS pulse on these radios
  p.slot = schedule_.slot();
  p.leaseGen = schedule_.leaseGeneration();
  p.schedGen = schedule_.generation();
  p.layout = SCHEDULE_LAYOUT;
  schedule_.fillSlotMap(nowMs, p.slotMap);
  p.hop = hopPack(0, 0);
  p.refId = schedule_.referenceId();
  p.refHops = schedule_.hopsToReference();
  p.refLocked = schedule_.referenceLocked();
  p.fitToKeepTime = schedule_.fitToKeepTime();
  p.refFit = schedule_.referenceFit();
  p.batteryPct = 255;
}

inline bool Node::sendExtraBeacon(uint32_t nowMs) {
  if (!sentOnce_) return false;
  const Fix& fix = ownFix_.fix();
  if (fix.lat == sentLat_ && fix.lon == sentLon_) return false;
  if ((uint32_t)(nowMs - lastBeaconMs_) < fw::EXTRA_MIN_GAP_MS) return false;
  if (!schedule_.inExtraSlot(millis())) return false;

  Position p;
  fillBeacon(p, nowMs);
  p.extra = true;
  uint8_t body[POSITION_MIN];
  const size_t n = encodePosition(p, body, sizeof(body));
  TxMeta meta;
  meta.kind = TX_EXTRA;
  if (n > 0 && transmit(FRAME_POSITION, body, n, 0, meta)) {
    lastBeaconMs_ = nowMs;
    sentLat_ = fix.lat;
    sentLon_ = fix.lon;
    schedule_.announceFit(p.fitToKeepTime);
  }
  return true;
}

inline bool Node::beacon(uint32_t nowMs) {
  if (!ownFix_.fresh(nowMs)) return false;
  const Fix& fix = ownFix_.fix();
  if (sendExtraBeacon(nowMs)) return false;

  if (!wantBeacon_) {
    const uint32_t moved = sentOnce_ ? distanceM(sentLat_, sentLon_, fix.lat, fix.lon) : fw::GATE_METRES;
    const bool timeDue = !sentOnce_ || (int32_t)(nowMs - nextBeaconMs_) >= 0;
    if (timeDue || moved >= fw::GATE_METRES) {
      wantBeacon_ = true;
      schedule_.rebuild(nodeId_, false, mesh_.riders(), MAX_RIDERS, nowMs, true);
      schedule_.drawSharedTurn(w->espRandom());
    }
  }
  if (schedule_.claimed() && schedule_.slot() != announcedSlot_) wantBeacon_ = true;
  if (!wantBeacon_) return false;

  if (schedule_.weAreReference()) schedule_.startEpoch(nowMs);
  if (!schedule_.inSlot(millis())) return false;

  Position p;
  fillBeacon(p, nowMs);
  const bool named = (uint32_t)(nowMs - lastNameMs_) >= fw::NAME_EVERY_MS;
  if (named) {
    strncpy(p.name, name_, sizeof(p.name) - 1);
    p.name[sizeof(p.name) - 1] = 0;
  }
  uint8_t body[POSITION_MIN + sizeof(p.name)];
  const size_t n = encodePosition(p, body, sizeof(body));
  TxMeta meta;
  meta.kind = schedule_.claimed() ? TX_LEASE : TX_UNLEASED;
  meta.hop = schedule_.hopsToReference();
  meta.slot = schedule_.slot();
  if (n == 0 || !transmit(FRAME_POSITION, body, n, adapt::originHops(pol, FRAME_POSITION), meta)) {
    if (!schedule_.claimed()) return false;
    c.leaseRefused++;
    return true;
  }

  wantBeacon_ = false;
  lastBeaconMs_ = nowMs;
  announcedSlot_ = schedule_.slot();
  schedule_.announceFit(p.fitToKeepTime);
  if (named) lastNameMs_ = nowMs;
  if (nextBeaconMs_ == 0) nextBeaconMs_ = nowMs;
  nextBeaconMs_ = nextOnGrid(nextBeaconMs_, fw::GATE_IDLE_MS, nowMs);
  sentLat_ = fix.lat;
  sentLon_ = fix.lon;
  sentOnce_ = true;
  return false;
}

inline void Node::sendDeferred(uint32_t nowMs) {
  Forward f;
  for (int budget = 0; budget < fw::DEFERRED_BUDGET && mesh_.nextDue(nowMs, f); budget++) {
    adapt::markRelayer(pol, f.wire, f.len, nodeId_);
    TxMeta meta;
    meta.kind = (f.len > 1 && (f.wire[1] & 0x0F) == FRAME_VOICE) ? TX_FWD_VOICE : TX_FWD_POS;
    w->radioSend(car, f.wire, f.len, meta);
  }
}

inline void Node::sendTestVoice(uint32_t nowMs) {
  if (!talkerOn_) return;
  if ((int32_t)(nowMs - testTalkerAtMs_) >= (int32_t)TEST_TALKER_LEASE_MS) {
    talkerOn_ = false;
    return;
  }
  if (nextTestMs_ != 0 && (int32_t)(nowMs - nextTestMs_) < 0) return;
  nextTestMs_ = nextOnGrid(nextTestMs_ != 0 ? nextTestMs_ : nowMs, TEST_VOICE_PERIOD_MS, nowMs);

  uint32_t phase = 0;
  const uint16_t phaseMs = schedule_.phaseAt(millis(), phase) ? (uint16_t)phase : TEST_VOICE_NO_PHASE;
  const uint8_t bytes = talkerBytes_ > FRAME_MAX_BODY ? (uint8_t)FRAME_MAX_BODY : talkerBytes_;
  uint8_t body[FRAME_MAX_BODY];
  const uint32_t seq = testSeq_++;
  const size_t n = encodeTestVoice(testSession_, seq, phaseMs, bytes, body, sizeof(body));
  TxMeta meta;
  meta.kind = TX_VOICE;
  const bool sent = n > 0 && transmit(FRAME_VOICE, body, n, adapt::originHops(pol, FRAME_VOICE), meta);
  w->noteVoiceAttempt(car, seq, sent);
}

// ---- World ---------------------------------------------------------------------

inline World::World(const Config& c)
    : cfg(c),
      m(c.model),
      rngWorld(c.seed * 0x100000001B3ull + 1),
      rngChan(c.seed * 0x100000001B3ull + 2),
      rngMac(c.seed * 0x100000001B3ull + 3),
      rngFw(c.seed * 0x100000001B3ull + 4),
      rngPass(c.seed * 0x100000001B3ull + 5) {
  build();
}

inline int64_t World::localUs(size_t car, Us t) const {
  const int64_t since = t > bootAt[car] ? (int64_t)(t - bootAt[car]) : 0;
  return (int64_t)localBaseMs[car] * 1000 + since + since * ppm[car] / 1000000;
}

inline bool World::phaseUs(size_t car, Us t, int64_t& out) const {
  const int64_t lus = localUs(car, t);
  uint32_t ph = 0;
  if (!nodes[car].schedule_.phaseAt((uint32_t)(lus / 1000), ph)) return false;
  out = (int64_t)ph * 1000 + lus % 1000;
  return true;
}

inline Fix World::fixAt(size_t car, int64_t k) const {
  Fix f;
  const double north = cfg.moving ? (double)k * m.movingStepM : 0;
  const double lat = 35.0 + (y[car] + north) / 111320.0;
  const double lon = -83.0 + x[car] / (111320.0 * cos(35.0 * 3.14159265358979 / 180.0));
  f.lat = (int32_t)llround(lat * 1e7);
  f.lon = (int32_t)llround(lon * 1e7);
  const Us at = fixMeasuredUs(car, k);
  f.fixSec = EPOCH_SEC + (uint32_t)(at / 1000000);
  f.fixMs = (uint16_t)((at / 1000) % 1000);
  f.external = true;
  return f;
}

inline void World::build() {
  n = cfg.cars;
  if (n > 32) n = 32;  // Original::heardMask
  nodes.resize(n);
  radios.resize(n);
  x.assign(n, 0);
  y.assign(n, 0);
  // Where the cars are.
  for (size_t i = 0; i < n; i++) {
    switch (cfg.layout) {
      case Layout::BENCH:
        x[i] = m.benchSpacingM * i;
        break;
      case Layout::LINE:
        x[i] = m.lineSpacingM * i;
        break;
      case Layout::PARK: {
        const size_t side = (size_t)ceil(sqrt((double)n));
        x[i] = m.parkPitchM * (i % side) + (rngWorld.unit() * 2 - 1) * m.parkJitterM;
        y[i] = m.parkPitchM * (i / side) + (rngWorld.unit() * 2 - 1) * m.parkJitterM;
        break;
      }
    }
  }
  if (cfg.talker) {
    talker = cfg.talkerCar >= 0 ? (size_t)cfg.talkerCar : (cfg.layout == Layout::PARK ? n / 2 : 0);
  }
  // The links.
  rssi.assign(n, std::vector<double>(n, -200));
  mw.assign(n, std::vector<double>(n, 0));
  surviveBase.assign(n, std::vector<double>(n, 0));
  senses.assign(n, std::vector<uint8_t>(n, 0));
  for (size_t i = 0; i < n; i++) {
    for (size_t j = i + 1; j < n; j++) {
      double d = hypot(x[i] - x[j], y[i] - y[j]);
      if (d < 0.1) d = 0.1;
      const double mean = m.txDbm + m.antennaDb - m.lossAt1mDb - 10 * m.exponent * log10(d) +
                          rngWorld.gauss() * m.shadowSigmaDb;
      rssi[i][j] = rssi[j][i] = mean;
    }
  }
  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j < n; j++) {
      if (i == j) continue;
      mw[i][j] = pow(10.0, rssi[i][j] / 10.0);
      senses[i][j] = rssi[i][j] >= m.floorDbm;
      double edge = (m.floorDbm + m.edgeDb - rssi[i][j]) / m.edgeDb;
      edge = edge < 0 ? 0 : (edge > 1 ? 1 : edge);
      surviveBase[i][j] = (1 - cfg.p) * (1 - edge);
    }
  }
  // Graph distance over the links that carry most frames.
  hopDist.assign(n, std::vector<uint8_t>(n, 255));
  for (size_t s = 0; s < n; s++) {
    std::vector<size_t> q{s};
    hopDist[s][s] = 0;
    for (size_t h = 0; h < q.size(); h++) {
      const size_t a = q[h];
      for (size_t b = 0; b < n; b++) {
        if (hopDist[s][b] != 255 || rssi[a][b] < m.floorDbm + m.graphMarginDb) continue;
        hopDist[s][b] = (uint8_t)(hopDist[s][a] + 1);
        q.push_back(b);
      }
    }
  }
  airPosUs = airtimeUs(FRAME_HEADER + POSITION_MIN + TAG_LEN);

  bootAt.assign(n, 0);
  fixPhaseUs.assign(n, 0);
  ppm.assign(n, 0);
  localBaseMs.assign(n, 0);
  nextStallUs.assign(n, 0);
  for (size_t i = 0; i < n; i++) {
    Node& nd = nodes[i];
    nd.w = this;
    nd.car = i;
    nd.pol = cfg.policy;
    bool unique;
    do {
      nd.nodeId_ = 0x10000000u + (rngWorld.u32() & 0x0FFFFFFFu);
      unique = true;
      for (size_t j = 0; j < i; j++) unique = unique && nodes[j].nodeId_ != nd.nodeId_;
    } while (!unique);
    carOf[nd.nodeId_] = i;
    ppm[i] = (int32_t)rngWorld.below(2 * m.driftPpm + 1) - m.driftPpm;
    localBaseMs[i] = 10000 + rngWorld.below(100000);
    bootAt[i] = (Us)rngWorld.below(cfg.bootSpreadMs) * 1000 + rngWorld.below(1000);
    fixPhaseUs[i] = (Us)rngWorld.below((uint32_t)fixEveryUs());
    push(bootAt[i], EV_BOOT, i);
  }
  result.cfg = cfg;
  result.talker = talker;
  result.cars.assign(n, CarStats());
  result.ageHist.assign(1000, 0);
  voiceFirstUs.assign(n, std::vector<Us>());
  voiceHops.assign(n, std::vector<uint8_t>());
  newestFixUs.assign(n, std::vector<Us>(n, 0));
}

inline bool World::radioSend(size_t car, const uint8_t* wire, size_t len, const TxMeta& meta) {
  Radio& radio = radios[car];
  if (!radio.on || len == 0 || len > FRAME_MAX) return false;
  if (radio.txq.size() >= m.txBuffers) {
    if (measuring) result.txRefused[meta.kind]++;
    return false;
  }
  TxJob job;
  memcpy(job.wire, wire, len);
  job.len = (uint16_t)len;
  job.meta = meta;
  radio.txq.push_back(job);
  if (original(meta.kind)) {
    Frame f;
    if (decodeFrame(wire, len, f)) {
      Original record;
      record.car = car;
      record.kind = meta.kind;
      record.counted = measuring;
      originalAt[((uint64_t)f.src << 32) | f.id] = originals.size();
      originals.push_back(record);
    }
  }
  if (!radio.onAir && radio.mac == Radio::MAC_IDLE && radio.txq.size() == 1) startContention(car);
  return true;
}

inline bool World::poll(size_t car, RxFrame& out) {
  Radio& radio = radios[car];
  if (radio.rxq.empty()) return false;
  out = radio.rxq.front();
  radio.rxq.pop_front();
  return true;
}

// drainRadio's next frame, drainCostUs after the last, or the rest of the pass
// once it stops. A frame that ends meanwhile is polled in this pass.
inline void World::drainStep(size_t car) {
  if (nodes[car].drainNext()) push(now + m.drainCostUs, EV_DRAIN, car);
  else push(now + m.passBaseUs, EV_PASS_END, car);
}

// Busy as this car will see it: any frame it can sense on the air. One that
// started under ccaDelayUs ago counts, since DIFS is longer than that.
inline bool World::busyAt(size_t car, Us t) const {
  for (const Air& frame : air) {
    if (frame.car != car && frame.start <= t && frame.end > t && senses[frame.car][car]) return true;
  }
  return false;
}

inline void World::startContention(size_t car) {
  Radio& radio = radios[car];
  radio.backoffUs = (Us)rngMac.below(m.cwSlots + 1) * m.slotUs;
  tryCount(car);
}

inline void World::tryCount(size_t car) {
  Radio& radio = radios[car];
  if (busyAt(car, now)) {
    radio.mac = Radio::MAC_WAIT;
    return;
  }
  radio.countFrom = std::max(now, radio.lastIdle + m.difsUs);
  radio.attemptAt = radio.countFrom + radio.backoffUs;
  radio.mac = Radio::MAC_COUNT;
  radio.token++;
  push(radio.attemptAt, EV_MAC, car, radio.token);
}

inline void World::startTx(size_t car) {
  Radio& radio = radios[car];
  radio.mac = Radio::MAC_IDLE;
  radio.onAir = true;
  Air frame;
  frame.serial = ++airSerial;
  frame.car = car;
  frame.start = now;
  frame.job = radio.txq.front();
  frame.end = now + airtimeUs(frame.job.len);
  const uint8_t kind = frame.job.meta.kind;

  // Everyone who can sense it freezes, unless its own backoff ends before it
  // could. It keeps the slots it has not counted, the one this frame cuts into
  // among them, since DCF counts a slot only once it has passed idle.
  const Us senseAt = now + m.ccaDelayUs;
  for (size_t c = 0; c < n; c++) {
    if (c == car || !senses[car][c] || !radios[c].on) continue;
    Radio& other = radios[c];
    if (other.mac == Radio::MAC_COUNT && other.attemptAt > senseAt) {
      if (now > other.countFrom) other.backoffUs = (other.attemptAt - now + m.slotUs - 1) / m.slotUs * m.slotUs;
      other.mac = Radio::MAC_WAIT;
      other.token++;
    }
    if (other.busyDepth++ == 0) other.busySince = now;
  }
  if (radio.busyDepth++ == 0) radio.busySince = now;

  // What went on the air.
  const uint32_t srcId = ((uint32_t)frame.job.wire[2] << 24) | ((uint32_t)frame.job.wire[3] << 16) |
                         ((uint32_t)frame.job.wire[4] << 8) | frame.job.wire[5];
  const uint32_t id = ((uint32_t)frame.job.wire[6] << 24) | ((uint32_t)frame.job.wire[7] << 16) |
                      ((uint32_t)frame.job.wire[8] << 8) | frame.job.wire[9];
  auto it = originalAt.find(((uint64_t)srcId << 32) | id);
  if (it != originalAt.end()) originals[it->second].copies++;
  if (measuring) {
    result.airUs[kind] += (double)(frame.end - frame.start);
    result.txCount[kind]++;
    if (kind == TX_LEASE && frame.job.meta.slot < MAX_SLOTS) {
      const uint8_t hop = frame.job.meta.hop > 5 ? 6 : frame.job.meta.hop;
      const int64_t slotUs = (int64_t)slotStartMs(frame.job.meta.slot) * 1000;
      auto wrap = [](int64_t v) { return (int32_t)(((v % 1000000) + 1000000 + 500000) % 1000000 - 500000); };
      const int64_t slotEndUs = (int64_t)SLOT_MS * 1000 - (int64_t)(frame.end - frame.start);
      int64_t own = 0;
      if (phaseUs(car, now, own)) {
        const int32_t late = wrap(own - slotUs);
        result.late[hop].ownUs.push_back(late);
        if (late > slotEndUs) result.late[hop].overranOwn++;
      }
      auto ref = carOf.find(nodes[car].schedule_.referenceId());
      int64_t ride = 0;
      if (ref != carOf.end() && phaseUs(ref->second, now, ride)) {
        const int32_t late = wrap(ride - slotUs);
        result.late[hop].rideUs.push_back(late);
        if (late > slotEndUs) result.late[hop].overranRide++;
      }
    }
  }
  air.push_back(frame);
  push(frame.end, EV_TX_END, car, frame.serial);
}

inline void World::endTx(uint64_t serial) {
  size_t ai = 0;
  while (ai < air.size() && air[ai].serial != serial) ai++;
  if (ai == air.size()) return;
  const Air tx = air[ai];
  const size_t src = tx.car;
  const bool lease = tx.job.meta.kind == TX_LEASE;
  const double airRatio = m.lossScalesWithAirtime ? (double)(tx.end - tx.start) / airPosUs : 1.0;

  // Every receiver's copy.
  for (size_t j = 0; j < n; j++) {
    if (j == src || !radios[j].on) continue;
    const double mean = rssi[src][j];
    if (mean < m.floorDbm - 4 * m.fadeSigmaDb) continue;
    const double heard = mean + rngChan.gauss() * m.fadeSigmaDb;
    if (heard < m.floorDbm) continue;
    bool deaf = false, leaseOverlap = false;
    double interMw = 0;
    for (const Air& other : air) {
      if (other.serial == tx.serial || other.start >= tx.end || other.end <= tx.start) continue;
      if (other.car == j) {
        deaf = true;
        break;
      }
      interMw += mw[other.car][j];
      if (other.job.meta.kind == TX_LEASE) leaseOverlap = true;
    }
    if (deaf) {
      if (measuring) result.halfDuplex++;
      continue;
    }
    if (rngChan.unit() >= pow(surviveBase[src][j], airRatio)) {
      if (measuring) result.randomLoss++;
      continue;
    }
    if (interMw > 0 && heard - 10 * log10(interMw) < m.captureDb) {
      if (measuring) {
        if (lease && leaseOverlap) result.leaseClash++;
        else if (lease) result.leaseHit++;
        else result.otherCollisions++;
      }
      continue;
    }
    Radio& radio = radios[j];
    if (radio.rxq.size() >= m.rxDepth) {
      if (measuring) result.rxDrops++;
      continue;
    }
    RxFrame rx;
    memcpy(rx.data, tx.job.wire, tx.job.len);
    rx.len = tx.job.len;
    const long rounded = lround(heard);
    rx.rssi = (int8_t)(rounded < -128 ? -128 : (rounded > 0 ? 0 : rounded));
    rx.rxMs = millisOf(j, now);
    rx.chan = Node::RADIO_CHANNEL;
    radio.rxq.push_back(rx);
  }

  // The medium as each car now senses it.
  for (size_t c = 0; c < n; c++) {
    if (c == src || !senses[src][c] || !radios[c].on) continue;
    Radio& other = radios[c];
    if (other.busyDepth > 0 && --other.busyDepth == 0 && measuring) other.busyTotal += now - std::max(other.busySince, markUs);
    if (!busyAt(c, now)) {
      other.lastIdle = now;
      if (other.mac == Radio::MAC_WAIT) tryCount(c);
    }
  }
  Radio& radio = radios[src];
  if (radio.busyDepth > 0 && --radio.busyDepth == 0 && measuring) radio.busyTotal += now - std::max(radio.busySince, markUs);
  radio.onAir = false;
  radio.lastIdle = std::max(radio.lastIdle, now);
  radio.txq.pop_front();
  if (!radio.txq.empty()) startContention(src);

  air.erase(std::remove_if(air.begin(), air.end(), [this](const Air& frame) { return frame.end + AIR_KEEP_US < now; }),
            air.end());
}

inline void World::noteFirstSight(size_t receiver, const Frame& f, const uint8_t* body, size_t bodyLen,
                                  uint8_t hopsAway) {
  auto it = originalAt.find(((uint64_t)f.src << 32) | f.id);
  if (it != originalAt.end() && receiver < 32) originals[it->second].heardMask |= 1u << receiver;
  auto from = carOf.find(f.src);
  if (from == carOf.end()) return;
  if (f.type == FRAME_POSITION) {
    Position p;
    if (!decodePosition(body, bodyLen, p) || p.fix.fixSec < EPOCH_SEC) return;
    const Us fixUs = ((Us)(p.fix.fixSec - EPOCH_SEC) * 1000 + p.fix.fixMs) * 1000;
    Us& newest = newestFixUs[receiver][from->second];
    if (fixUs > newest) newest = fixUs;
    return;
  }
  uint8_t session = 0;
  uint32_t seq = 0;
  uint16_t phase = 0;
  if (f.type == FRAME_VOICE && decodeTestVoice(body, bodyLen, session, seq, phase)) {
    std::vector<Us>& first = voiceFirstUs[receiver];
    if (seq >= first.size()) {
      first.resize(seq + 64, 0);
      voiceHops[receiver].resize(seq + 64, 0);
    }
    if (first[seq] == 0) {
      first[seq] = now;
      voiceHops[receiver][seq] = hopsAway;
    }
  }
}

inline void World::noteVoiceAttempt(size_t car, uint32_t seq, bool sent) {
  (void)car;
  if (!measuring) return;
  if (seq >= voiceTriedUs.size()) {
    voiceTriedUs.resize(seq + 64, 0);
    voiceSent.resize(seq + 64, 0);
  }
  voiceTriedUs[seq] = now;
  voiceSent[seq] = sent ? 1 : 0;
  result.voiceAttempted++;
  if (!sent) result.voiceRefused++;
}

// A lease beacon just given to heardBeacon. One stamped after the pass began
// must still show in the map a beacon sent on this pass carries.
inline void World::noteLeaseHeard(size_t car, uint32_t src, uint8_t slot, uint32_t rxMs, uint32_t nowMs) {
  if (!measuring || (int32_t)(rxMs - nowMs) <= 0 || slot >= MAX_SLOTS) return;
  result.stampsAhead++;
  uint8_t map[SLOT_MAP_LEN];
  nodes[car].schedule_.fillSlotMap(nowMs, map);
  if (map[slot] != slotTag(src)) result.stampsAheadUnmapped++;
}

inline bool World::oneClock() const {
  const uint32_t ref = nodes[0].schedule_.referenceId();
  auto at = carOf.find(ref);
  if (at == carOf.end()) return false;
  int64_t refPhase = 0;
  if (!radios[at->second].on || !phaseUs(at->second, now, refPhase)) return false;
  for (size_t i = 0; i < n; i++) {
    int64_t ph = 0;
    if (!radios[i].on || nodes[i].schedule_.referenceId() != ref || !phaseUs(i, now, ph)) return false;
    const int64_t off = ((ph - refPhase) % 1000000 + 1500000) % 1000000 - 500000;
    if (off > SYNCED_MS * 1000 || off < -SYNCED_MS * 1000) return false;
  }
  return true;
}

inline void World::sample() {
  if (!measuring) return;
  if (!oneClock()) result.unsyncedSamples++;
  // How old each car's position is on every map within three hops.
  for (size_t j = 0; j < n; j++) {
    for (size_t i = 0; i < n; i++) {
      if (i == j || hopDist[i][j] > 3) continue;
      const Us newest = newestFixUs[j][i];
      const double ageMs = newest == 0 ? (double)(now - markUs) / 1000 : (double)(now - newest) / 1000;
      const size_t bin = (size_t)(ageMs / 10);
      result.ageHist[bin < result.ageHist.size() ? bin : result.ageHist.size() - 1]++;
      result.ageSumMs += ageMs;
      result.ageN++;
      result.cars[j].ageSumMs += ageMs;
      result.cars[j].ageN++;
    }
  }
  // Two cars on one slot that a third car hears well (floor + graphMarginDb), or
  // that hear each other: their beacons collide where it matters. Edge links
  // lose most frames anyway, and the slot maps cannot see a clash there either.
  const double good = m.floorDbm + m.graphMarginDb;
  for (size_t a = 0; a < n; a++) {
    if (!nodes[a].schedule_.claimed()) continue;
    for (size_t b = a + 1; b < n; b++) {
      if (nodes[b].schedule_.slot() != nodes[a].schedule_.slot()) continue;
      bool interferes = rssi[a][b] >= good;
      for (size_t k = 0; k < n && !interferes; k++) interferes = k != a && k != b && rssi[a][k] >= good && rssi[b][k] >= good;
      if (interferes) result.sharedSlotSamples++;
    }
  }
  // The app renews the talker every 30 s.
  if (talker < n && ((now - markUs) / SAMPLE_US) % 300 == 0) nodes[talker].testTalkerAtMs_ = millisOf(talker, now);
}

inline void World::mark() {
  measuring = true;
  markUs = now;
  countersAtMark.clear();
  lossesAtMark.clear();
  suppressedAtMark.clear();
  for (size_t i = 0; i < n; i++) {
    countersAtMark.push_back(nodes[i].c);
    lossesAtMark.push_back(nodes[i].schedule_.losses());
    suppressedAtMark.push_back(nodes[i].mesh_.suppressed());
    nodes[i].passGapMaxMs_ = 0;
    nodes[i].positionSkips_ = FastRelaySkips();
    nodes[i].voiceSkips_ = FastRelaySkips();
    radios[i].busyTotal = 0;
    if (radios[i].busyDepth > 0) radios[i].busySince = now;
  }
  if (talker < n) nodes[talker].startTalker(cfg.talkerBytes);
}

inline void World::runUntil(Us t) {
  while (!events.empty() && events.top().t <= t) {
    const Ev e = events.top();
    events.pop();
    now = e.t;
    switch (e.kind) {
      case EV_BOOT: {
        radios[e.car].on = true;
        radios[e.car].lastIdle = now;
        nodes[e.car].boot();
        push(now + rngPass.below(fw::TICK_MS * 1000), EV_PASS, e.car);
        if (cfg.stallEveryMs > 0) nextStallUs[e.car] = now + (Us)(rngPass.unit() * 2 * cfg.stallEveryMs * 1000);
        break;
      }
      case EV_PASS:
        nodes[e.car].passBegin();
        drainStep(e.car);
        break;
      case EV_DRAIN:
        drainStep(e.car);
        break;
      case EV_PASS_END: {
        nodes[e.car].passEnd();
        Us next = now + fw::TICK_MS * 1000 + rngPass.below(m.passJitterUs + 1);
        if (cfg.stallEveryMs > 0 && now >= nextStallUs[e.car]) {
          next += (Us)(m.stallMinMs + rngPass.below(m.stallMaxMs - m.stallMinMs + 1)) * 1000;
          nextStallUs[e.car] = now + (Us)(rngPass.unit() * 2 * cfg.stallEveryMs * 1000);
        }
        push(next, EV_PASS, e.car);
        break;
      }
      case EV_MAC: {
        Radio& radio = radios[e.car];
        if (radio.mac == Radio::MAC_COUNT && radio.token == e.arg && !radio.txq.empty()) startTx(e.car);
        break;
      }
      case EV_TX_END:
        endTx(e.arg);
        break;
      case EV_SAMPLE:
        sample();
        if (now + SAMPLE_US <= endUs) push(now + SAMPLE_US, EV_SAMPLE, 0);
        break;
    }
  }
  now = t;
}

inline void World::runAll() {
  // At least the warm-up, then until the ride has kept one clock for SYNC_HOLD_MS.
  runUntil((Us)cfg.warmupMs * 1000);
  Us heldSince = oneClock() ? now : 0;
  while (now < (Us)WARMUP_CAP_MS * 1000 && (heldSince == 0 || now - heldSince < (Us)SYNC_HOLD_MS * 1000)) {
    runUntil(now + SAMPLE_US);
    if (!oneClock()) heldSince = 0;
    else if (heldSince == 0) heldSince = now;
  }
  if (heldSince != 0 && now - heldSince >= (Us)SYNC_HOLD_MS * 1000) result.syncedAtMs = (uint32_t)(heldSince / 1000);
  mark();
  endUs = now + (Us)cfg.measureMs * 1000;
  push(now + SAMPLE_US, EV_SAMPLE, 0);
  runUntil(endUs);
  finish();
}

inline void World::finish() {
  measuring = false;
  const Us window = endUs - markUs;
  // Radios still busy at the end.
  for (size_t i = 0; i < n; i++) {
    if (radios[i].busyDepth > 0) radios[i].busyTotal += endUs - std::max(radios[i].busySince, markUs);
    result.cars[i].busyPct = 100.0 * radios[i].busyTotal / window;
  }
  // Positions: lease and unleased beacons sent in the window, by graph distance.
  double leaseCopies = 0;
  uint32_t leaseN = 0;
  double voiceCopies = 0;
  uint32_t voiceN = 0;
  for (const Original& frame : originals) {
    if (!frame.counted) continue;
    if (frame.kind == TX_LEASE || frame.kind == TX_UNLEASED) {
      if (frame.kind == TX_LEASE) {
        leaseCopies += frame.copies;
        leaseN++;
      }
      for (size_t j = 0; j < n; j++) {
        if (j == frame.car) continue;
        const uint8_t d = hopDist[frame.car][j];
        const bool got = (frame.heardMask >> j) & 1u;
        const size_t b = bucketOf(d == 255 ? BUCKETS : d);
        result.posSent[b]++;
        if (got) result.posGot[b]++;
        if (d <= 3) {
          result.cars[j].posSent++;
          if (got) result.cars[j].posGot++;
        }
      }
    } else if (frame.kind == TX_VOICE && frame.copies > 0) {
      voiceCopies += frame.copies;
      voiceN++;
      if (frame.copies > result.voiceCopiesMax) result.voiceCopiesMax = frame.copies;
    }
  }
  result.leaseCopies = leaseN ? leaseCopies / leaseN : 0;
  result.voiceCopies = voiceN ? voiceCopies / voiceN : 0;

  // Voice, per listener: every seq the talker tried in the window, refused ones
  // included, up to a deadline and a packet before the end so the last ones
  // had their chance.
  if (talker < n) {
    const Us deadline = (Us)m.voiceDeadlineMs * 1000;
    // Anything over today's packet is taken to carry the one before (VoicePacket prev).
    const bool withPrev = cfg.talkerBytes > TEST_VOICE_DEFAULT_BYTES;
    uint32_t firstSeq = 0, lastSeq = 0;
    for (uint32_t s = 1; s < voiceTriedUs.size(); s++) {
      if (voiceTriedUs[s] == 0 || voiceTriedUs[s] + deadline + TEST_VOICE_PERIOD_MS * 1000 > endUs) continue;
      if (firstSeq == 0) firstSeq = s;
      lastSeq = s;
    }
    for (size_t j = 0; j < n; j++) {
      CarStats& stats = result.cars[j];
      stats.fromTalker = hopDist[talker][j];
      if (j == talker || firstSeq == 0) continue;
      const std::vector<Us>& first = voiceFirstUs[j];
      std::vector<double> lats;
      bool prevOk = false;
      double prevLat = 0;
      uint32_t run = 0;
      for (uint32_t s = firstSeq; s <= lastSeq; s++) {
        stats.expected++;
        const Us sent = voiceSent[s] ? voiceTriedUs[s] : 0;
        const Us got = s < first.size() ? first[s] : 0;
        bool ok = false;
        if (sent != 0 && got != 0) {
          stats.delivered++;
          const double lat = (double)(got - sent) / 1000;
          lats.push_back(lat);
          stats.latSumMs += lat;
          stats.firstHopsSum += voiceHops[j][s];
          ok = got - sent <= deadline;
        }
        if (ok) {
          stats.onTime++;
          const double lat = (double)(got - sent) / 1000;
          if (prevOk) {
            stats.jitterSumMs += fabs(lat - prevLat);
            stats.jitterN++;
          }
          prevLat = lat;
        }
        prevOk = ok;
        // The next packet's copy of this one (VoicePacket prev), in time to play.
        bool repaired = ok;
        if (!ok && withPrev && sent != 0 && s + 1 < first.size() && first[s + 1] != 0 &&
            first[s + 1] <= sent + deadline)
          repaired = true;
        if (repaired) stats.repaired++;
        if (!ok) {
          run++;
        } else if (run > 0) {
          stats.holeRuns++;
          stats.longestHole = std::max(stats.longestHole, run);
          run = 0;
        }
      }
      if (run > 0) {
        stats.holeRuns++;
        stats.longestHole = std::max(stats.longestHole, run);
      }
      stats.latP95Ms = percentile(lats, 0.95);
      for (size_t t = 0; t < VoiceMeter::TALKERS; t++) {
        const VoiceMeter::Talker& vt = nodes[j].voiceMeter_.talkers()[t];
        if (vt.used && vt.src == nodes[talker].nodeId_ && vt.delayed > 0)
          stats.meterMeanMs = (double)vt.delaySumMs / vt.delayed;
      }
    }
  }

  // Schedule, queues and counters.
  for (size_t i = 0; i < n; i++) {
    const Node& nd = nodes[i];
    const Counters& atMark = countersAtMark[i];
    result.slotLosses += nd.schedule_.losses() - lossesAtMark[i];
    result.suppressed += nd.mesh_.suppressed() - suppressedAtMark[i];
    result.leaseRetries += nd.c.leaseRefused - atMark.leaseRefused;
    result.deferRefused += nd.c.forwardsRefused - atMark.forwardsRefused;
    result.maxPassGapMs = std::max(result.maxPassGapMs, nd.passGapMaxMs_);
    if (nd.schedule_.claimed()) result.leased++;
    // Zeroed at mark(), so these are the window's.
    auto add = [](FastRelaySkips& to, const FastRelaySkips& from) {
      to.skipped += from.skipped;
      to.noEvidence += from.noEvidence;
      to.stale += from.stale;
      to.needed += from.needed;
      to.noRelayer += from.noRelayer;
    };
    add(result.relayPositions, nd.positionSkips_);
    add(result.relayVoice, nd.voiceSkips_);
  }
  std::vector<uint32_t> refs;
  for (const Node& nd : nodes) {
    if (std::find(refs.begin(), refs.end(), nd.schedule_.referenceId()) == refs.end())
      refs.push_back(nd.schedule_.referenceId());
  }
  result.references = refs.size();
}

// One scenario, start to finish.
inline Result simulate(const Config& cfg) {
  World w(cfg);
  w.runAll();
  return w.result;
}

}  // namespace ridesim
