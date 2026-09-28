#include "forward.h"
#include "schedule.h"

namespace touge {

uint32_t forwardDueMs(uint32_t rxMs, uint32_t nowMs, int16_t rssi, size_t neighbours, uint32_t tieBreak,
                      size_t wireLen) {
  const uint32_t delayMs = forwardDelayMs(rssi, forwardSpreadMs(neighbours), tieBreak, wireLen);
  // Timed from the receive time, a frame that sat through a stall is due on
  // this pass and goes before the copies queued behind it are counted.
  const bool stalled = (int32_t)(nowMs - rxMs) > (int32_t)FORWARD_STALL_MS;
  return (stalled ? nowMs : rxMs) + delayMs;
}

ForwardPlan planForward(const Frame& heard, const FastRelay& relay, uint32_t selfId, const Rider* riders,
                        size_t maxRiders, uint32_t rxMs, uint32_t nowMs, int16_t rssi, size_t neighbours,
                        uint32_t tieBreak) {
  ForwardPlan plan;
  // Relayed by the hops this frame type starts with, not FAST_HOPS.
  const bool relayed = hopsTravelled(heard.type, heard.hops) > 0;
  plan.verdict =
      relay.judge(selfId, heard.src, relayed, heard.relayer, riders, maxRiders, rxMs, &plan.relayerUnplaced);
  plan.skip = fastRelayDropsForward(plan.verdict, heard.type, countCarsOnRide(selfId, riders, maxRiders, rxMs));
  if (plan.skip) return plan;
  // The forward is this frame a hop down: the same length on the wire.
  plan.dueMs = forwardDueMs(rxMs, nowMs, rssi, neighbours, tieBreak, FRAME_HEADER + heard.len);
  plan.suppressAfter = suppressAfterFor(heard.type);
  return plan;
}

void markRelayer(uint8_t* wire, size_t len, uint32_t selfId) {
  if (wire != nullptr && len > FRAME_RELAYER_AT) wire[FRAME_RELAYER_AT] = slotTag(selfId);
}

}  // namespace touge
