#pragma once
//
// The LoRa lane's load, measured, and the interval it allows this car's
// position (SCALE-PLAN 5d).
//
// Build 39 stretched the 5 s position by an estimate, cars x cars packets a
// round, which cannot tell an efficient relay path from too much forwarding.
// The interval now follows what the radio measures: the share of the last
// minute the channel was busy, Meshtastic's airtime figure over everything
// heard and sent. Alongside it the module counts what went on the air, own and
// relayed, how long it waited in the TX queue, and what the queue dropped.
//
// Two reports, every status cycle, with the same keys on serial ("touge: lora
// ll ...") and to the phone ({"ll":{...}}), see formatLoraWindow/formatLoraTx.
//
// Platform-free: the module reads Meshtastic's counters and hands over numbers.

#include <stddef.h>
#include <stdint.h>

namespace touge {

// The target, and the longest the load may stretch it to. Twenty seconds is the
// most the app's dead reckoning can still place a car across.
static const uint32_t LORA_TARGET_MS = 5000;
static const uint32_t LORA_MAX_MS = 20000;

// The busy share the interval stretches to stay under: Meshtastic's "polite"
// limit (AirTime::isTxAllowedChannelUtil), above which its own telemetry and
// nodeinfo hold back.
static const uint32_t LORA_BUSY_TARGET_PERMILLE = 250;

// How often the interval is re-judged. The busy share covers the last minute,
// so a change shows in it gradually; half a minute between steps keeps the rule
// from chasing its own lag. A step at most doubles the interval, and comes down
// by at most a quarter once the air is under three quarters of the target.
static const uint32_t LORA_JUDGE_MS = 30000;

// Wait unknown: a packet the module did not note on its way to the queue.
static const uint32_t LORA_WAIT_UNKNOWN = 0xFFFFFFFF;

enum class LoraTx : uint8_t {
  OWN_POSITION,
  OWN_SUMMARY,  // our reach summary (reach.h)
  OWN_OTHER,    // text, nodeinfo, telemetry
  RELAY,
};

// Since boot: "lt".
struct LoraTxCounts {
  uint32_t ownTx = 0;         // ot: our positions on the air
  uint32_t ownAirMs = 0;      // oa
  uint32_t relayTx = 0;       // rt: other cars' packets we relayed
  uint32_t relayAirMs = 0;    // ra
  uint32_t relayEarly = 0;    // re: of those, sent early as a preferred relay (relaypref.h)
  uint32_t summaryAirMs = 0;  // sa
  uint32_t otherAirMs = 0;    // xa: our text, nodeinfo, telemetry
  uint32_t dropped = 0;       // dr: Meshtastic's txDrop, the TX queue full
  uint32_t cancelled = 0;     // cn: Meshtastic's txRelayCanceled, another car's copy heard first
  uint32_t replaced = 0;      // rp: a newer position took an older one's place (5c)
  uint32_t refused = 0;       // rf: an older position turned away (5c)
  uint32_t ownLate = 0;       // os: our position still queued when the next was due
};

// ov bits.
static const uint8_t LORA_OVER_CHANNEL = 0x01;  // busier than the target even at LORA_MAX_MS
static const uint8_t LORA_OVER_OWN_LATE = 0x02;  // our position missed its turn this window

// One report's window: "ll".
struct LoraWindow {
  uint32_t intervalMs = 0;      // li: what the rule set; 0 while this radio sends none
  uint32_t gapMeanMs = 0;       // la: mean of the last four gaps between our positions on the air; 0 before one
  uint32_t gapMaxMs = 0;        // lx: longest gap that ended in this window; 0 none
  uint32_t busyPermille = 0;    // cu: channel busy, last minute
  uint32_t txPermille = 0;      // tu: our own transmitting, last hour
  uint8_t overloaded = 0;       // ov
  uint32_t ownWaitMaxMs = 0;    // ow: longest our position sat in the TX queue
  uint32_t relayWaitMaxMs = 0;  // rw: the same for relayed positions
  uint32_t depthMax = 0;        // qm: TX queue high-water mark
  uint32_t originsHeard = 0;    // oh: cars heard over LoRa lately (reach.h)
  uint32_t preferredFor = 0;    // pf: origins we are a preferred relay for (relaypref.h)
};

class LoraLoad {
 public:
  void reset();

  uint32_t intervalMs() const { return intervalMs_; }
  // Re-judges the interval from how busy the channel was, [busyPermille] of
  // the last minute; no more often than LORA_JUDGE_MS. Quantised to whole
  // seconds, so cars reading nearly the same air land on the same interval
  // and their send grids (nextLoraSendAt) line up.
  void judge(uint32_t busyPermille, uint32_t nowMs);
  // Busier than the target with the interval already at LORA_MAX_MS.
  bool saturated() const { return saturated_; }

  // Our position went to the TX queue, [previousLate] if the one before it was
  // still there.
  void ownQueued(bool previousLate);
  // A packet went on the air after [waitedMs] in the TX queue.
  void sent(LoraTx kind, uint32_t airtimeMs, uint32_t waitedMs, bool early, uint32_t nowMs);
  void queueDepth(uint32_t depth);

  // The module fills the counters Meshtastic keeps (dropped, cancelled) and the
  // 5c ones (replaced, refused).
  LoraTxCounts& counts() { return counts_; }
  const LoraTxCounts& counts() const { return counts_; }

  // The window since the last call, and a new one. [sending]: this radio sends
  // its car's LoRa position, so li means something. The busy shares are
  // Meshtastic's, as they stand now.
  LoraWindow takeWindow(bool sending, uint32_t busyPermille, uint32_t txPermille);

 private:
  LoraTxCounts counts_;
  uint32_t intervalMs_ = LORA_TARGET_MS;
  uint32_t judgedMs_ = 0;
  bool judged_ = false;
  bool saturated_ = false;
  // Gaps between our positions on the air: the last few, for a steady la.
  static const uint8_t GAPS_KEPT = 4;
  uint32_t gaps_[GAPS_KEPT] = {0};
  uint8_t gapsHeld_ = 0;
  uint8_t gapNext_ = 0;
  bool hasLastOwnTx_ = false;
  uint32_t lastOwnTxMs_ = 0;
  // The window.
  uint32_t gapMaxMs_ = 0;
  uint32_t ownWaitMaxMs_ = 0;
  uint32_t relayWaitMaxMs_ = 0;
  uint32_t depthMax_ = 0;
  bool lateInWindow_ = false;
};

// When this car's next LoRa position is due: the first point after [afterMs]
// on a grid every car shares, the interval's multiples on a clock they all have
// (UTC from their fixes), [rank] of [cars] shares in, plus [jitter] (reduced
// under half a share). Cars that chanced into step used to stay there; spread
// by rank, a round's first transmissions go out a share apart and each car's
// relays have that share to finish in. Meshtastic's contention delay and
// channel sensing still run on top.
uint64_t nextLoraSendAt(uint64_t afterMs, uint32_t intervalMs, uint32_t rank, uint32_t cars, uint32_t jitter);

// "ll" and "lt": the serial line when [json] is false ("ll li=5000 ..."), the
// phone's JSON when true. Every counter at 2^32 - 1 still fits one Meshtastic
// payload. 0 if it does not fit [cap].
size_t formatLoraWindow(const LoraWindow& w, bool json, char* out, size_t cap);
size_t formatLoraTx(const LoraTxCounts& c, bool json, char* out, size_t cap);

}  // namespace touge
