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

#include "SinglePortModule.h"
#include "concurrency/OSThread.h"
#include "touge/espnow.h"
#include "touge/frame.h"
#include "touge/mesh.h"
#include "touge/ride.h"
#include "touge/schedule.h"

class TougeFastModule : public SinglePortModule, private concurrency::OSThread {
  public:
    TougeFastModule();

    // For the screen: how many cars are close enough to be on 2.4 GHz.
    size_t fastNeighbours() const;

  protected:
    virtual int32_t runOnce() override;
    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

  private:
    // Re-derives the fast network when the primary channel's key changes, and
    // starts or retunes the radio. Cheap when nothing moved.
    void syncChannel();

    void drainRadio(uint32_t nowMs);
    void sendDeferred(uint32_t nowMs);
    void beacon(uint32_t nowMs);

    // Hands a frame heard on 2.4 GHz to the rest of Meshtastic as though it
    // had come off the LoRa radio, so NodeDB, the OLED and the phone all see
    // it without any of them needing to know this module exists.
    void inject(const touge::Frame &f, const uint8_t *body, size_t len, int8_t rssi);

    bool transmit(uint8_t type, const uint8_t *body, size_t len, uint8_t hops);

    // Packet ids are half the AES-CTR nonce, so they must never repeat under
    // one channel key. NVS holds a value safely ahead of anything already
    // sent, and it is re-armed in blocks rather than written every packet.
    void loadIdCounter();
    void saveIdCounter();

    touge::Mesh mesh_;
    touge::FastNet net_;
    touge::Schedule schedule_;

    uint8_t keySeen_[touge::PSK_LEN] = {0};
    uint8_t keySeenLen_ = 0;
    bool started_ = false;

    uint32_t nodeId_ = 0;
    uint32_t idCeiling_ = 0;
    uint32_t lastBeaconMs_ = 0;
    uint32_t lastNameMs_ = 0;
    uint32_t lastSyncMs_ = 0;
    // The gate has opened and we are waiting for our slot.
    bool wantBeacon_ = false;
    // Where we were when we last transmitted, for the distance gate.
    int32_t sentLat_ = 0;
    int32_t sentLon_ = 0;
    bool sentOnce_ = false;
};

extern TougeFastModule *tougeFastModule;

#endif
