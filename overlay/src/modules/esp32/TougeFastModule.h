#pragma once
//
// The 2.4 GHz fast lane, as a Meshtastic module.
//
// Meshtastic keeps doing everything it already does on LoRa. This adds a
// second path over the board's own 2.4 GHz radio for the traffic that is
// wasted on LoRa: position at 1 Hz, and push-to-talk audio, neither of which
// a convoy in line of sight has any reason to pay SF7 airtime for.
//
// Nothing here replaces the LoRa side. A car that drops behind a ridge falls
// back to Meshtastic on its own, because its positions keep arriving over LoRa
// and NodeDB does not care which radio they came in on.
//
// ## How the phone talks to it
//
// Audio is captured and played on the phone, because the board has neither a
// microphone nor a speaker. The app sends a voice packet as an ordinary
// Meshtastic data packet on PRIVATE_APP addressed **to the local radio's own
// node number**, not to broadcast. Router::sendLocal delivers a packet aimed
// at this node straight to the modules and never puts it on the air, so that
// addressing is what keeps audio off LoRa. Sending it to broadcast instead
// would flood 12 kbps of speech across the mesh, which would be a bad day.
//
// Inbound audio goes the other way: a voice frame heard over ESP-NOW is
// injected as though it had arrived over the radio, so the phone receives it
// on the BLE link it already has open with no app changes.

#include "configuration.h"

#if defined(ARCH_ESP32) && !defined(MESHTASTIC_EXCLUDE_TOUGE_FAST)

#include "MeshPacketQueue.h"
#include "RadioTxHook.h"
#include "SinglePortModule.h"
#include "concurrency/OSThread.h"
#include "touge/espnow.h"
#include "touge/frame.h"
#include "touge/gnssfix.h"
#include "touge/mesh.h"
#include "touge/ownfix.h"
#include "touge/phonebatch.h"
#include "touge/ride.h"
#include "touge/hop.h"
#include "touge/loraload.h"
#include "touge/lorapos.h"
#include "touge/reach.h"
#include "touge/relaypref.h"
#include "touge/rideclock.h"
#include "touge/schedule.h"

class TougeFastModule : public SinglePortModule, private concurrency::OSThread {
  public:
    TougeFastModule();

    // For the screen: how many cars are close enough to be on 2.4 GHz.
    size_t fastNeighbours(uint32_t nowMs) const;

  protected:
    virtual int32_t runOnce() override;
    virtual bool wantPacket(const meshtastic_MeshPacket *p) override;
    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

  private:
    // Re-derives the fast network when the primary channel's key changes, and
    // starts or retunes the radio. Cheap when nothing moved.
    void syncChannel();

    void drainRadio(uint32_t nowMs);
    void sendDeferred(uint32_t nowMs);

    // Follows the ride onto another channel when this one is unusable, and
    // goes looking for it when we have lost track of where it went.
    void hopKeeping(uint32_t nowMs);

    // One periodic line saying what the fast lane thinks is going on. None of
    // it is visible anywhere else.
    void status(uint32_t nowMs);
    // Free internal RAM now, its low-water mark and the largest block, to serial.
    void logHeap();
    void beacon(uint32_t nowMs);
    // A new fix in one of our extra slots (Schedule::extraSlots). True if sent.
    bool sendExtraBeacon(uint32_t nowMs);
    // Everything a beacon carries except the name, lease beacon or extra.
    void fillBeacon(touge::Position &p, uint32_t nowMs);

    // Our own fix and the identity both lanes send it under (SCALE-PLAN 5a):
    // the phone's from its writes, the receiver's read every pass while the
    // phone is quiet. See touge/ownfix.h.
    void notePhoneFix(const meshtastic_MeshPacket &mp);
    void noteGnssFix(uint32_t nowMs);
    static touge::Fix fixOf(const meshtastic_Position &pos);
    static bool readPosition(const meshtastic_MeshPacket &mp, meshtastic_Position &pos);
    meshtastic_Position ownLoraPosition() const;
    touge::OwnFix ownFix_;

    // ---- Our LoRa position, and the TX queue's one per car (SCALE-PLAN 5b, 5c)
    //
    // This radio sends its car's LoRa position from ownFix_, at the interval
    // the measured load allows (loraLoad_), while loraOwned(); PositionModule's
    // own broadcasts stand down meanwhile (core-patches/0011). See
    // touge/lorapos.h and touge/loraload.h.
    void sendLoraPosition(uint32_t nowMs);
    // A Touge app has said hello since boot, the primary channel is a ride, and
    // there is a fix, fresh or not.
    bool loraOwned() const;
    // This car and every node heard on either radio within RIDER_DROP_MS, and
    // how many of them have a lower node number: our share of the send grid.
    void loraRoster(uint32_t &cars, uint32_t &rank) const;
    // A position from another car that we may relay, noted for the TX queue.
    void noteRelayedPosition(const meshtastic_MeshPacket &mp, const meshtastic_Position &pos, uint32_t nowMs);
    // The hooks in core-patches/0010, 0011 and 0012.
    static bool ownsPositionBroadcast();
    static TougeTxPlace placeTxPacket(const std::vector<meshtastic_MeshPacket *> &queue, const meshtastic_MeshPacket *p,
                                      size_t &at);
    static bool inTxQueue(uint32_t from, uint32_t id, void *ctx);
    static bool relayEarly(const meshtastic_MeshPacket *p);
    touge::TxPositions txPositions_;
    uint32_t lastLoraMs_ = 0;
    uint32_t nextLoraMs_ = 0;
    // Our last LoRa position's packet id, to see whether it is still queued
    // when the next is due.
    uint32_t lastOwnPositionId_ = 0;
    bool rideAppSeen_ = false;

    // ---- The LoRa lane measured, and relays chosen on evidence (SCALE-PLAN 5d-5f)
    //
    // What went on the air and how long it waited, from every packet leaving the
    // TX queue (Meshtastic's RadioTxHook); the reach summaries this car sends
    // and hears; which origins it relays early. Reported as "ll", "lt" and "le".
    class LoraTxWatch : public RadioTxHook
    {
      public:
        void packetReleased(RadioInterface *iface, const meshtastic_MeshPacket *p) override;
    };
    void noteLoraReleased(RadioInterface *iface, const meshtastic_MeshPacket *p);
    // Meshtastic's busy share of the channel over the last minute, permille.
    static uint32_t loraBusyPermille();
    void noteLoraReach(const meshtastic_MeshPacket &mp, const meshtastic_Position &pos, uint32_t nowMs);
    void noteReachSummary(const meshtastic_MeshPacket &mp);
    void sendReachSummary(uint32_t nowMs);
    // A summary to serial, a few entries a line: [what] "sent" or "from".
    void logReach(const char *what, uint32_t reporter, const uint8_t *payload, size_t len);
    // The three LoRa reports to serial, and to the phone when one is connected.
    void reportLora(uint32_t nowMs);
    touge::LoraLoad loraLoad_;
    touge::Reach reach_;
    touge::RelayPrefs relayPrefs_;
    LoraTxWatch loraTxWatch_;
    // Meshtastic's count of completed transmissions, last seen: it moves before
    // a sent packet is released and not for a cancelled or dropped one.
    uint32_t txGoodSeen_ = 0;
    uint32_t lorasSent_ = 0;
    uint32_t lastReachId_ = 0;
    uint32_t reachQueuedMs_ = 0;

    // The receiver's own fix to the phone, about 1 Hz, so a tablet with no GPS
    // can navigate on it. See touge/gnssfix.h.
    void forwardGnssFix(uint32_t nowMs);
    touge::GnssForward gnssForward_;

    // Hands a frame heard on 2.4 GHz to the rest of Meshtastic as though it
    // had come off the LoRa radio, so NodeDB, the OLED and the phone all see
    // it without any of them needing to know this module exists.
    void inject(const touge::Frame &f, const uint8_t *body, size_t len, int8_t rssi);

    bool transmit(uint8_t type, const uint8_t *body, size_t len, uint8_t hops);

    // Our wire position as Meshtastic's, for NodeDB and for the phone.
    static meshtastic_Position asMeshPosition(const touge::Position &p);

    // Puts a car's 2.4 GHz position back after a LoRa copy has overwritten it.
    // See handleReceived for why the LoRa copy is now allowed through at all.
    void reassertFastPositions();

    // Packet ids are half the AES-CTR nonce, so they must never repeat under
    // one channel key. NVS holds a value safely ahead of anything already
    // sent, and it is re-armed in blocks rather than written every packet.
    void loadIdCounter();
    void saveIdCounter();

    touge::Mesh mesh_;
    touge::FastNet net_;
    touge::Schedule schedule_;
    touge::Hop hop_;

    uint8_t keySeen_[touge::PSK_LEN] = {0};
    uint8_t keySeenLen_ = 0;
    bool started_ = false;
    // The lane left BLE under LANE_HEAP_FLOOR and was taken down for this boot.
    bool heapRefused_ = false;

    // Cars whose NodeDB row a stale LoRa position is about to overwrite.
    //
    // Small and fixed: this only fills with cars heard on both radios inside
    // three seconds, and one entry each is enough because the correction runs
    // within five milliseconds. A full ring drops the oldest, which costs that
    // car a single stale row until its next fast frame.
    static const size_t RESTORE_SLOTS = 8;
    uint32_t restore_[RESTORE_SLOTS] = {0};
    size_t restoreCount_ = 0;

    uint32_t nodeId_ = 0;
    uint32_t idCeiling_ = 0;
    uint32_t lastBeaconMs_ = 0;
    uint32_t nextBeaconMs_ = 0;
    uint32_t lastNameMs_ = 0;
    uint32_t lastSyncMs_ = 0;
    uint32_t lastHeardMs_ = 0;
    uint32_t lastScanMs_ = 0;
    uint32_t lastHopCheckMs_ = 0;
    uint32_t lastStatusMs_ = 0;
    /** Rate limit for saying why this board is not beaconing. */
    uint32_t lastMuteLogMs_ = 0;
    uint32_t heardInWindow_ = 0;
    // The last reference beacon the clock was stepped to, so a repeat of
    // the same frame cannot step it a second time.
    uint32_t lastSyncSrc_ = 0;
    uint32_t lastSyncId_ = 0;
    // The gate has opened and we are waiting for our slot.
    bool wantBeacon_ = false;
    // The lease slot our last lease beacon went out in. A new lease is
    // announced in its first slot rather than at the next 1 s deadline.
    uint8_t announcedSlot_ = touge::SLOT_NONE;
    // Where we were when we last transmitted, for the distance gate.
    int32_t sentLat_ = 0;
    int32_t sentLon_ = 0;
    bool sentOnce_ = false;

    // ---- Positions to the phone in batches, and the link counters ----------
    // SCALE-PLAN steps 1 and 3; the pure parts are in touge/phonebatch.h.

    touge::PhoneRecord phoneRecordFor(const touge::Frame &f, const touge::Position &p, int8_t rssi) const;
    void offerToPhone(const touge::PhoneRecord &r);
    void flushPhoneBatch(uint32_t nowMs);
    // Hands one encoded batch toward the phone: pre-encoded for the next read
    // when allowed, the ordinary phone queue otherwise. False if neither took it;
    // otherwise [packetId] is the MeshPacket it went as and [preloaded] which way.
    bool handPhoneBatch(const uint8_t *payload, size_t len, uint32_t &packetId, bool &preloaded);
    // Once a tick: queue depth, a phone that went away, batches read or discarded.
    void trackPhoneLink(uint32_t nowMs);
    // A batch the phone read: [records] from its in-flight entry, [expired] of
    // them held too long to deliver as positions.
    void notePhoneRead(uint8_t records, uint32_t expired);
    // Sends the phone the ids of its writes the radio dropped (core-patches/0007).
    void reportDroppedWrites();
    // The lane report while the lane is not running: build and reason, every
    // STATUS_EVERY_MS, so the phone can tell this radio from stock Meshtastic.
    void reportLaneDown(uint32_t nowMs, touge::LaneDown why);
    void reportLinkStats(uint32_t nowMs, uint32_t windowMs);
    void queueJsonToPhone(const char *json, size_t len);
    // Not const: a batch is restamped on its way out (core-patches/0006).
    void notePhoneDelivered(meshtastic_MeshPacket &p);
    static void onPhoneDelivered(meshtastic_MeshPacket &p);
    static bool stillInPhoneQueue(uint32_t packetId, void *ctx);

    touge::PhoneStore phoneStore_;
    touge::BatchesInFlight batchesInFlight_;
    touge::LinkStats stats_;
    touge::LinkStats statsAtLastReport_;
    touge::PhoneHello hello_;
    bool helloSeen_ = false;
    // Why the lane is down while !started_; reported by reportLaneDown.
    touge::LaneDown laneDown_ = touge::LaneDown::NO_KEY;
    uint16_t batchSeq_ = 0;
    // NimBLE counters last seen, to notice the pre-encoded batch being read or lost.
    uint32_t preloadReadSeen_ = 0;
    uint32_t preloadLostSeen_ = 0;
    uint32_t preloadExpiredSeen_ = 0;
    uint32_t storeExpiredSeen_ = 0;
};

extern TougeFastModule *tougeFastModule;

#endif
