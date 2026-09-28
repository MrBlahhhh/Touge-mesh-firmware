#pragma once
//
// What drainRadio does with a frame it could forward, and the byte sendDeferred
// writes into each forward. Out of TougeFastModule so the host tests and the
// ride sim (test_ridesim) run the module's own decision rather than a copy.
//
// Platform-free: time and the random tie come in as arguments.

#include <stddef.h>
#include <stdint.h>
#include "fastrelay.h"
#include "frame.h"
#include "mesh.h"

namespace touge {

// A frame received this long before the pass that drains it sat in the RX
// queue through a stall (95-140 ms on the tablet's V4), not one pass gap.
static const uint32_t FORWARD_STALL_MS = 2 * FORWARD_PASS_MS;

/**
 * When a forward of a `wireLen`-byte frame received at `rxMs` and drained on
 * the pass at `nowMs` is due: forwardDelayMs over forwardSpreadMs(neighbours),
 * from the receive time, so every hearer's wait starts at the same instant and
 * the steps between them hold. After a stall, from the pass instead.
 */
uint32_t forwardDueMs(uint32_t rxMs, uint32_t nowMs, int16_t rssi, size_t neighbours, uint32_t tieBreak,
                      size_t wireLen);

struct ForwardPlan {
  FastRelayVerdict verdict = FastRelayVerdict::NO_EVIDENCE;
  bool relayerUnplaced = false;  // with the verdict, for FastRelaySkips::note
  bool skip = false;             // the verdict drops the forward: hold nothing
  uint32_t dueMs = 0;            // otherwise Mesh::defer's due time
  uint8_t suppressAfter = SUPPRESS_AFTER;  // and its copy count
};

/**
 * What to do with `heard`, a frame with hops left seen for the first time.
 * Judged by `relay` against the roster at its receive time `rxMs`, skipped
 * when fastRelayDropsForward says so, and otherwise held until forwardDueMs
 * with its type's suppressAfterFor. `neighbours` is fastNeighbours() and
 * `tieBreak` any random number. Ask before Mesh::defer, so a skipped frame
 * never holds a forward slot.
 */
ForwardPlan planForward(const Frame& heard, const FastRelay& relay, uint32_t selfId, const Rider* riders,
                        size_t maxRiders, uint32_t rxMs, uint32_t nowMs, int16_t rssi, size_t neighbours,
                        uint32_t tieBreak);

// We are putting this copy of a frame on the air: sendDeferred, just before
// the send. The byte is outside the tag like hops, so the sealed payload goes
// out unchanged.
void markRelayer(uint8_t* wire, size_t len, uint32_t selfId);

}  // namespace touge
