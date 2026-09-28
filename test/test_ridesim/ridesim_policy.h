#pragma once
//
// Which build's forwarding rules the simulated cars run.
//
// Build 51 is the module's own code: Node::drainFrame asks touge::planForward
// (forward.h) what to do with each frame and Node::sendDeferred calls
// touge::markRelayer, as TougeFastModule does. Build 50 (4143455) is kept here,
// since 51 rewrote its forward delay in place: signal order plus a 0-3 ms tie
// from the pass's time, two hops and three copies for every frame type, no
// fastrelay, no relayer byte, and heardBeacon stamped on the pass's time.
//
// The sweeps run 51 with other constants: voice hops and copies, tie steps, and
// which SKIP verdicts drop a forward. At build51()'s values planForward's answer
// is used as it stands; a field set to another value adjusts that answer the
// way the constant would.

#include <stddef.h>
#include <stdint.h>
#include "fastrelay.h"
#include "forward.h"
#include "frame.h"
#include "mesh.h"
#include "schedule.h"

namespace ridesim {

// ---- TougeFastModule.cpp's file-scope constants, mirrored --------------------
// The module is board-only, so the sim restates what its pass reads.
namespace fw {
const uint32_t TICK_MS = 5;
static_assert(TICK_MS == touge::FORWARD_PASS_MS, "the module asserts the same");
const uint32_t GATE_METRES = 20;
const uint32_t GATE_IDLE_MS = 1000;
const uint32_t FAST_PRECEDENCE_MS = 3000;
const uint32_t NAME_EVERY_MS = 30000;
const uint32_t LOST_MS = 6000;
const uint32_t EXTRA_MIN_GAP_MS = 100;
const uint32_t SYNC_BIAS_MS = TICK_MS / 2 + 1;
const int DRAIN_BUDGET = 8;     // drainRadio
const int DEFERRED_BUDGET = 4;  // sendDeferred
}  // namespace fw

// ---- Build 50's forwarding, kept as shipped ----------------------------------
namespace b50 {
// mesh.cpp at 4143455: signal order plus a 0-3 ms tie.
const uint32_t FORWARD_TIE_MS = 4;
const uint8_t HOPS = 2;            // every frame type
const uint8_t SUPPRESS_AFTER = 3;  // every frame type

inline uint32_t forwardDelayMs(int16_t rssi, uint32_t spreadMs, uint32_t tieBreak) {
  const uint32_t tie = tieBreak % FORWARD_TIE_MS;
  if (spreadMs == 0) return tie;
  int32_t r = rssi;
  if (r < touge::FORWARD_FAR_DBM) r = touge::FORWARD_FAR_DBM;
  if (r > touge::FORWARD_NEAR_DBM) r = touge::FORWARD_NEAR_DBM;
  const int32_t span = (int32_t)touge::FORWARD_NEAR_DBM - (int32_t)touge::FORWARD_FAR_DBM;
  return (uint32_t)(((int64_t)(r - touge::FORWARD_FAR_DBM) * (int64_t)spreadMs) / span) + tie;
}
}  // namespace b50

enum class Build : uint8_t { B50, B51 };

struct Policy {
  Build build = Build::B50;
  // What a voice frame leaves with and the copies that drop a held forward of
  // it. build50() and build51() set each build's own; a sweep sets others.
  uint8_t voiceHops = b50::HOPS;
  uint8_t voiceSuppressAfter = b50::SUPPRESS_AFTER;
  // Build 51 only. build51() sets the firmware's constants.
  uint32_t tieSteps = touge::FORWARD_TIE_STEPS;
  bool fastRelaySkips = true;  // false: verdicts are counted, never taken
  size_t fastRelayMinCars = touge::FAST_RELAY_MIN_CARS;
  bool fastRelaySkipsVoice = touge::FAST_RELAY_SKIPS_VOICE;
};

inline Policy build50() { return Policy(); }

inline Policy build51() {
  Policy p;
  p.build = Build::B51;
  p.voiceHops = touge::VOICE_HOPS;
  p.voiceSuppressAfter = touge::VOICE_SUPPRESS_AFTER;
  return p;
}

// ---- The adapter: each call the glue makes that differs by build -------------
namespace adapt {

inline bool b51(const Policy& pol) { return pol.build == Build::B51; }

// TougeFastModule::transmit's hops: FAST_HOPS for a position, VOICE_HOPS for voice.
inline uint8_t originHops(const Policy& pol, uint8_t type) {
  if (type == touge::FRAME_VOICE) return pol.voiceHops;
  return b51(pol) ? touge::FAST_HOPS : b50::HOPS;
}

// touge::hopsTravelled against the policy's hops: 0 straight from the origin.
inline uint8_t hopsAway(const Policy& pol, const touge::Frame& f) {
  if (b51(pol) && pol.voiceHops == touge::VOICE_HOPS) return touge::hopsTravelled(f.type, f.hops);
  const uint8_t start = originHops(pol, f.type);
  return f.hops < start ? (uint8_t)(start - f.hops) : 0;
}

// The origin's own copy of a position: the only one drainRadio syncs to, and
// the only one heardBeacon takes.
inline bool originPosition(const Policy& pol, const touge::Frame& f) {
  return f.type == touge::FRAME_POSITION && f.hops == originHops(pol, touge::FRAME_POSITION);
}

// drainRadio: 51 stamps heardBeacon with rx.rxMs, 50 with the pass's nowMs.
inline uint32_t beaconStampMs(const Policy& pol, uint32_t nowMs, uint32_t rxMs) { return b51(pol) ? rxMs : nowMs; }

// drainRadio, 51: every first-sight position's map, any hops, on its receive time.
inline void feedMap(const Policy& pol, touge::FastRelay& relay, uint32_t src, const touge::Position& p, uint32_t rxMs) {
  if (b51(pol)) relay.heardMap(src, p, rxMs);
}

// sendDeferred, 51: the relayer byte, just before fastRadio.send.
inline void markRelayer(const Policy& pol, uint8_t* wire, size_t len, uint32_t selfId) {
  if (b51(pol)) touge::markRelayer(wire, len, selfId);
}

// forwardDelayMs with `tieSteps` in place of FORWARD_TIE_STEPS, written out on
// its own so a test can hold it to mesh.cpp's at the firmware's count.
inline uint32_t stepDelayMs(int16_t rssi, uint32_t spreadMs, uint32_t tieBreak, size_t wireLen, uint32_t tieSteps) {
  const uint32_t step = touge::forwardStepMs(wireLen);
  const uint32_t tie = (tieBreak % (tieSteps > 0 ? tieSteps : 1)) * step;
  if (spreadMs == 0) return tie;
  int32_t r = rssi;
  if (r < touge::FORWARD_FAR_DBM) r = touge::FORWARD_FAR_DBM;
  if (r > touge::FORWARD_NEAR_DBM) r = touge::FORWARD_NEAR_DBM;
  const int32_t span = (int32_t)touge::FORWARD_NEAR_DBM - (int32_t)touge::FORWARD_FAR_DBM;
  const uint32_t bySignal = (uint32_t)(((int64_t)(r - touge::FORWARD_FAR_DBM) * (int64_t)spreadMs) / span);
  return bySignal - bySignal % step + tie;
}

// fastRelayDropsForward with the policy's floor and voice switch.
inline bool dropsForward(const Policy& pol, touge::FastRelayVerdict verdict, uint8_t frameType, size_t carsOnRide) {
  if (!pol.fastRelaySkips || verdict != touge::FastRelayVerdict::SKIP) return false;
  if (carsOnRide == 1) return true;
  if (carsOnRide < pol.fastRelayMinCars) return false;
  return frameType != touge::FRAME_VOICE || pol.fastRelaySkipsVoice;
}

inline bool firmwareSkipRule(const Policy& pol) {
  return pol.fastRelaySkips && pol.fastRelayMinCars == touge::FAST_RELAY_MIN_CARS &&
         pol.fastRelaySkipsVoice == touge::FAST_RELAY_SKIPS_VOICE;
}

// Build 51 with the policy's constants: planForward's answer, adjusted where
// the policy differs from the firmware. Taken for any sweep, so a test holds it
// to planForward at build51().
inline touge::ForwardPlan sweptPlan(const Policy& pol, const touge::Frame& heard, const touge::FastRelay& relay,
                                    uint32_t selfId, const touge::Rider* riders, uint32_t rxMs, uint32_t nowMs,
                                    int16_t rssi, size_t neighbours, uint32_t tieBreak) {
  const bool voice = heard.type == touge::FRAME_VOICE;
  // planForward reads "relayed" against VOICE_HOPS, so a copy relayed under the
  // sweep's hops is shown to it as one a hop down from VOICE_HOPS.
  touge::Frame judged = heard;
  if (voice) judged.hops = heard.hops < pol.voiceHops ? touge::VOICE_HOPS - 1 : touge::VOICE_HOPS;
  touge::ForwardPlan plan = touge::planForward(judged, relay, selfId, riders, touge::MAX_RIDERS, rxMs, nowMs, rssi,
                                               neighbours, tieBreak);
  const size_t cars = touge::countCarsOnRide(selfId, riders, touge::MAX_RIDERS, rxMs);
  plan.skip = dropsForward(pol, plan.verdict, heard.type, cars);
  if (plan.skip) return plan;
  // planForward stops at a skip it takes, so the due time is worked out here
  // for one the policy does not take. Another tie count keeps the start
  // forwardDueMs chose (rx.rxMs, or the pass after a stall) and runs its own tie.
  const size_t wireLen = touge::FRAME_HEADER + heard.len;
  const uint32_t firmwareDue = touge::forwardDueMs(rxMs, nowMs, rssi, neighbours, tieBreak, wireLen);
  plan.dueMs = firmwareDue;
  if (pol.tieSteps != touge::FORWARD_TIE_STEPS) {
    const uint32_t spreadMs = touge::forwardSpreadMs(neighbours);
    const uint32_t waitFrom = firmwareDue - touge::forwardDelayMs(rssi, spreadMs, tieBreak, wireLen);
    plan.dueMs = waitFrom + stepDelayMs(rssi, spreadMs, tieBreak, wireLen, pol.tieSteps);
  }
  plan.suppressAfter = voice ? pol.voiceSuppressAfter : touge::SUPPRESS_AFTER;
  return plan;
}

// drainRadio's forward decision, after `f.hops > 0` and before encodeFrame.
inline touge::ForwardPlan planForward(const Policy& pol, const touge::Frame& heard, const touge::FastRelay& relay,
                                      uint32_t selfId, const touge::Rider* riders, uint32_t rxMs, uint32_t nowMs,
                                      int16_t rssi, size_t neighbours, uint32_t tieBreak) {
  if (!b51(pol)) {
    touge::ForwardPlan plan;
    plan.dueMs = nowMs + b50::forwardDelayMs(rssi, touge::forwardSpreadMs(neighbours), tieBreak);
    plan.suppressAfter = heard.type == touge::FRAME_VOICE ? pol.voiceSuppressAfter : b50::SUPPRESS_AFTER;
    return plan;
  }
  const bool firmware = pol.voiceHops == touge::VOICE_HOPS && pol.voiceSuppressAfter == touge::VOICE_SUPPRESS_AFTER &&
                        pol.tieSteps == touge::FORWARD_TIE_STEPS && firmwareSkipRule(pol);
  if (firmware)
    return touge::planForward(heard, relay, selfId, riders, touge::MAX_RIDERS, rxMs, nowMs, rssi, neighbours, tieBreak);
  return sweptPlan(pol, heard, relay, selfId, riders, rxMs, nowMs, rssi, neighbours, tieBreak);
}

}  // namespace adapt
}  // namespace ridesim
