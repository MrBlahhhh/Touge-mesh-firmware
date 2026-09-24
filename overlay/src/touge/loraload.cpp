#include "loraload.h"

#include "kvline.h"

namespace touge {

void LoraLoad::reset() { *this = LoraLoad(); }

void LoraLoad::judge(uint32_t busyPermille, uint32_t nowMs) {
  if (judged_ && (uint32_t)(nowMs - judgedMs_) < LORA_JUDGE_MS) return;
  judged_ = true;
  judgedMs_ = nowMs;
  const uint32_t target = LORA_BUSY_TARGET_PERMILLE;
  // Measured, not predicted: the minute just gone ran at the longest interval
  // and the air was still over the target.
  saturated_ = intervalMs_ >= LORA_MAX_MS && busyPermille > target;

  // The interval at which the same traffic would sit at the target, taking the
  // load as inversely proportional to it. On a ride's channel it mostly is:
  // positions and their relays. Where it is not (another group, noise), the
  // interval runs to the cap and saturated() says so.
  const uint64_t wanted = (uint64_t)intervalMs_ * busyPermille / target;
  uint64_t next = intervalMs_;
  if (busyPermille > target) {
    const uint64_t doubled = 2ull * intervalMs_;
    next = wanted < doubled ? wanted : doubled;
  } else if (busyPermille * 4 < target * 3) {
    const uint64_t stepDown = (uint64_t)intervalMs_ * 3 / 4;
    next = wanted > stepDown ? wanted : stepDown;
  }
  if (next < LORA_TARGET_MS) next = LORA_TARGET_MS;
  if (next > LORA_MAX_MS) next = LORA_MAX_MS;
  intervalMs_ = (uint32_t)((next + 999) / 1000 * 1000);
}

void LoraLoad::ownQueued(bool previousLate) {
  if (!previousLate) return;
  counts_.ownLate++;
  lateInWindow_ = true;
}

void LoraLoad::sent(LoraTx kind, uint32_t airtimeMs, uint32_t waitedMs, bool early, uint32_t nowMs) {
  const bool waitKnown = waitedMs != LORA_WAIT_UNKNOWN;
  switch (kind) {
    case LoraTx::OWN_POSITION:
      counts_.ownTx++;
      counts_.ownAirMs += airtimeMs;
      if (hasLastOwnTx_) {
        const uint32_t gap = nowMs - lastOwnTxMs_;
        gaps_[gapNext_] = gap;
        gapNext_ = (uint8_t)((gapNext_ + 1) % GAPS_KEPT);
        if (gapsHeld_ < GAPS_KEPT) gapsHeld_++;
        if (gap > gapMaxMs_) gapMaxMs_ = gap;
      }
      lastOwnTxMs_ = nowMs;
      hasLastOwnTx_ = true;
      if (waitKnown && waitedMs > ownWaitMaxMs_) ownWaitMaxMs_ = waitedMs;
      break;
    case LoraTx::OWN_SUMMARY:
      counts_.summaryAirMs += airtimeMs;
      break;
    case LoraTx::OWN_OTHER:
      counts_.otherAirMs += airtimeMs;
      break;
    case LoraTx::RELAY:
      counts_.relayTx++;
      counts_.relayAirMs += airtimeMs;
      if (early) counts_.relayEarly++;
      if (waitKnown && waitedMs > relayWaitMaxMs_) relayWaitMaxMs_ = waitedMs;
      break;
  }
}

void LoraLoad::queueDepth(uint32_t depth) {
  if (depth > depthMax_) depthMax_ = depth;
}

LoraWindow LoraLoad::takeWindow(bool sending, uint32_t busyPermille, uint32_t txPermille) {
  LoraWindow w;
  w.intervalMs = sending ? intervalMs_ : 0;
  uint64_t gapSum = 0;
  for (uint8_t i = 0; i < gapsHeld_; i++) gapSum += gaps_[i];
  w.gapMeanMs = gapsHeld_ >= 1 ? (uint32_t)(gapSum / gapsHeld_) : 0;
  w.gapMaxMs = gapMaxMs_;
  w.busyPermille = busyPermille;
  w.txPermille = txPermille;
  w.overloaded = (uint8_t)((saturated_ ? LORA_OVER_CHANNEL : 0) | (lateInWindow_ ? LORA_OVER_OWN_LATE : 0));
  w.ownWaitMaxMs = ownWaitMaxMs_;
  w.relayWaitMaxMs = relayWaitMaxMs_;
  w.depthMax = depthMax_;

  gapMaxMs_ = 0;
  ownWaitMaxMs_ = 0;
  relayWaitMaxMs_ = 0;
  depthMax_ = 0;
  lateInWindow_ = false;
  // A pause in sending (no fix, or no app) is not a gap between positions.
  if (!sending) {
    hasLastOwnTx_ = false;
    gapsHeld_ = 0;
    gapNext_ = 0;
  }
  return w;
}

uint64_t nextLoraSendAt(uint64_t afterMs, uint32_t intervalMs, uint32_t rank, uint32_t cars, uint32_t jitter) {
  if (intervalMs == 0) return afterMs;
  if (cars == 0) cars = 1;
  const uint32_t share = intervalMs / cars;
  const uint32_t jitterSpan = share / 2;
  // Under one interval: at most (cars - 1) shares plus half of one.
  const uint32_t offset = (rank % cars) * share + (jitterSpan > 0 ? jitter % jitterSpan : 0);
  uint64_t at = afterMs / intervalMs * intervalMs + offset;
  if (at <= afterMs) at += intervalMs;
  return at;
}

size_t formatLoraWindow(const LoraWindow& w, bool json, char* out, size_t cap) {
  KvLine line(out, cap, "ll", json);
  line.add("li", w.intervalMs);
  line.add("la", w.gapMeanMs);
  line.add("lx", w.gapMaxMs);
  line.add("cu", w.busyPermille);
  line.add("tu", w.txPermille);
  line.add("ov", w.overloaded);
  line.add("ow", w.ownWaitMaxMs);
  line.add("rw", w.relayWaitMaxMs);
  line.add("qm", w.depthMax);
  line.add("oh", w.originsHeard);
  line.add("pf", w.preferredFor);
  return line.finish();
}

size_t formatLoraTx(const LoraTxCounts& c, bool json, char* out, size_t cap) {
  KvLine line(out, cap, "lt", json);
  line.add("ot", c.ownTx);
  line.add("oa", c.ownAirMs);
  line.add("rt", c.relayTx);
  line.add("ra", c.relayAirMs);
  line.add("re", c.relayEarly);
  line.add("sa", c.summaryAirMs);
  line.add("xa", c.otherAirMs);
  line.add("dr", c.dropped);
  line.add("cn", c.cancelled);
  line.add("rp", c.replaced);
  line.add("rf", c.refused);
  line.add("os", c.ownLate);
  return line.finish();
}

}  // namespace touge
