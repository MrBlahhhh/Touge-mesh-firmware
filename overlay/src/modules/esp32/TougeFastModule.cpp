#include "TougeFastModule.h"

#if defined(ARCH_ESP32) && !defined(MESHTASTIC_EXCLUDE_TOUGE_FAST)

#include "Channels.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include "main.h"
#include "touge/cipher.h"
#include <Preferences.h>
#include <string.h>

using namespace touge;

TougeFastModule *tougeFastModule = nullptr;

namespace {

// 1 Hz. On LoRa this would be indefensible; here a position costs 26 bytes on
// a radio nothing else is using, and the difference between a car icon that
// moves and one that teleports is entirely in this number.
const uint32_t BEACON_MS = 1000;

// The name rides along every half minute rather than on every ping. Everyone
// who can hear you has it after one, and after that it is just bytes.
const uint32_t NAME_EVERY_MS = 30000;

// Two hops. ESP-NOW reaches roughly as far as you can see, so the case this
// covers is a convoy strung out far enough that the front and back cannot hear
// each other directly but the middle can hear both. Three would mostly buy
// duplicate transmissions.
const uint8_t FAST_HOPS = 2;

// How far ahead of anything already sent the id counter jumps at boot. Large
// enough that a board would have to send this many packets in one power cycle
// to catch up with it, small enough that the 32-bit space is not a concern.
const uint32_t ID_BLOCK = 65536;

// How often the primary channel is re-read. Slow, because it only moves when
// somebody reconfigures the ride.
const uint32_t SYNC_EVERY_MS = 2000;

const char *NVS_NAMESPACE = "tougefast";
const char *NVS_ID_KEY = "idceil";

uint8_t speedToMph(float metresPerSecond) {
    float mph = metresPerSecond * 2.23694f;
    if (mph < 0) return 0;
    if (mph > 255) return 255;
    return (uint8_t)(mph + 0.5f);
}

} // namespace

TougeFastModule::TougeFastModule()
    : SinglePortModule("tougefast", meshtastic_PortNum_PRIVATE_APP), OSThread("tougefast")
{
    mesh_.reset();
    loadIdCounter();
}

size_t TougeFastModule::fastNeighbours() const
{
    size_t n = 0;
    const Rider *r = mesh_.riders();
    for (size_t i = 0; i < MAX_RIDERS; i++)
        if (r[i].used && r[i].via == HEARD_FAST) n++;
    return n;
}

void TougeFastModule::loadIdCounter()
{
    Preferences p;
    if (!p.begin(NVS_NAMESPACE, false)) {
        // No NVS means no way to guarantee a fresh nonce range, and reusing one
        // leaks the XOR of two payloads. Start from a value derived from the
        // node number instead of zero so at least two boards do not collide,
        // and carry on: losing the fast lane entirely would be worse.
        LOG_WARN("touge: no NVS, packet ids are not reboot-safe");
        mesh_.seedIds(nodeDB->getNodeNum());
        idCeiling_ = 0;
        return;
    }
    uint32_t ceiling = p.getUInt(NVS_ID_KEY, 0);
    mesh_.seedIds(ceiling);
    idCeiling_ = ceiling + ID_BLOCK;
    // Written now, before a single packet goes out, so a power cut mid-ride
    // cannot bring the board back up inside a range it has already used.
    p.putUInt(NVS_ID_KEY, idCeiling_);
    p.end();
}

void TougeFastModule::saveIdCounter()
{
    if (idCeiling_ == 0) return; // no NVS, nothing to keep
    if (mesh_.lastId() + (ID_BLOCK / 8) < idCeiling_) return;

    Preferences p;
    if (!p.begin(NVS_NAMESPACE, false)) return;
    idCeiling_ = mesh_.lastId() + ID_BLOCK;
    p.putUInt(NVS_ID_KEY, idCeiling_);
    p.end();
}

void TougeFastModule::syncChannel()
{
    CryptoKey key = channels.getKey(channels.getPrimaryIndex());
    // length of -1 means the channel has no usable key. Nothing to derive from
    // and nothing worth broadcasting in clear, so the fast lane stays down.
    if (key.length < 0) {
        if (started_) {
            LOG_INFO("touge: primary channel lost its key, fast lane down");
            fastRadio.end();
            started_ = false;
        }
        return;
    }

    uint8_t len = (uint8_t)key.length;
    if (len > PSK_LEN) len = PSK_LEN;
    if (started_ && len == keySeenLen_ && memcmp(key.bytes, keySeen_, len) == 0) return;

    if (!deriveFast(key.bytes, len, net_)) return;
    memcpy(keySeen_, key.bytes, len);
    keySeenLen_ = len;

    // The roster and the dedupe table are keyed to the old ride. Keeping them
    // across a channel change would leave cars from the last ride on the map.
    mesh_.reset();
    mesh_.seedIds(idCeiling_ ? idCeiling_ - ID_BLOCK : nodeDB->getNodeNum());

    if (!fastRadio.begin(net_)) {
        LOG_WARN("touge: ESP-NOW would not start, LoRa only");
        started_ = false;
        return;
    }
    started_ = true;
    LOG_INFO("touge: fast lane up on wifi channel %u", (unsigned)fastRadio.channel());
}

bool TougeFastModule::transmit(uint8_t type, const uint8_t *body, size_t len, uint8_t hops)
{
    if (!started_ || len > FRAME_MAX_PAYLOAD) return false;

    uint8_t payload[FRAME_MAX_PAYLOAD];
    memcpy(payload, body, len);

    Frame f;
    f.type = type;
    f.src = nodeId_;
    f.id = mesh_.nextId();
    f.hops = hops;
    f.chan = net_.chanByte;
    f.payload = payload;
    f.len = (uint16_t)len;

    cipherApply(net_.key, f.src, f.id, f.type, payload, len);

    uint8_t wire[FRAME_MAX];
    size_t n = encodeFrame(f, wire, sizeof(wire));
    if (n == 0) return false;

    // Our own traffic goes in the dedupe table too. Without it a frame we sent
    // and a neighbour rebroadcast comes back and we forward our own packet.
    mesh_.firstSight(f.src, f.id, millis());
    saveIdCounter();
    return fastRadio.send(wire, n);
}

void TougeFastModule::beacon(uint32_t nowMs)
{
    if (!started_) return;
    if ((uint32_t)(nowMs - lastBeaconMs_) < BEACON_MS) return;
    lastBeaconMs_ = nowMs;

    // No fix means nothing worth sending. The other cars keep the last one they
    // heard and show it as ageing, which is more useful than a zero.
    if (!localPosition.has_latitude_i || !localPosition.has_longitude_i) return;
    if (localPosition.latitude_i == 0 && localPosition.longitude_i == 0) return;

    Position p;
    p.lat = localPosition.latitude_i;
    p.lon = localPosition.longitude_i;
    p.headingDeg = (uint16_t)(localPosition.ground_track / 1e5);
    p.speedMph = speedToMph((float)localPosition.ground_speed);
    p.hasFix = true;
    p.phoneAttached = false;

    uint8_t battery = powerStatus ? (uint8_t)powerStatus->getBatteryChargePercent() : 255;
    p.batteryPct = battery;

    if ((uint32_t)(nowMs - lastNameMs_) >= NAME_EVERY_MS) {
        strncpy(p.name, owner.short_name, sizeof(p.name) - 1);
        lastNameMs_ = nowMs;
    }

    uint8_t body[POSITION_MIN + sizeof(p.name)];
    size_t n = encodePosition(p, body, sizeof(body));
    if (n > 0) transmit(FRAME_POSITION, body, n, FAST_HOPS);
}

void TougeFastModule::inject(const Frame &f, const uint8_t *body, size_t len, int8_t rssi)
{
    if (f.type == FRAME_POSITION) {
        Position p;
        if (!decodePosition(body, len, p)) return;

        // Straight into NodeDB. This is the whole reason for forking rather
        // than writing a firmware: the OLED, the phone app and every other
        // module read positions from here, and none of them need to know a
        // second radio exists.
        meshtastic_Position mp = meshtastic_Position_init_default;
        mp.latitude_i = p.lat;
        mp.longitude_i = p.lon;
        mp.has_latitude_i = true;
        mp.has_longitude_i = true;
        mp.ground_track = (uint32_t)p.headingDeg * 100000;
        mp.ground_speed = (uint32_t)(p.speedMph / 2.23694f);
        mp.location_source = meshtastic_Position_LocSource_LOC_EXTERNAL;
        mp.time = getValidTime(RTCQualityFromNet);

        nodeDB->updatePosition(f.src, mp, RX_SRC_RADIO);

        if (p.name[0] != 0) {
            meshtastic_NodeInfoLite *n = nodeDB->getMeshNode(f.src);
            // Only fill in a name for a node we have never heard from over
            // LoRa. A NodeInfo that arrived the normal way is signed and
            // carries a long name; overwriting it from an unauthenticated
            // 2.4 GHz frame would be a downgrade.
            if (n && !n->has_user) {
                meshtastic_User u = meshtastic_User_init_default;
                strncpy(u.short_name, p.name, sizeof(u.short_name) - 1);
                strncpy(u.long_name, p.name, sizeof(u.long_name) - 1);
                nodeDB->updateUser(f.src, u, channels.getPrimaryIndex());
            }
        }
        return;
    }

    // Everything else, voice included, goes to the phone as an ordinary packet
    // on our port. The app already has a BLE link open and already knows this
    // port, so nothing app-side has to change to receive it.
    if (len > meshtastic_Constants_DATA_PAYLOAD_LEN) return;

    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p) return;
    p->from = f.src;
    p->to = NODENUM_BROADCAST;
    p->id = f.id;
    p->channel = channels.getPrimaryIndex();
    p->hop_limit = 0;
    p->hop_start = 0;
    p->rx_rssi = rssi;
    p->rx_time = getValidTime(RTCQualityFromNet);
    p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    p->decoded.portnum = meshtastic_PortNum_PRIVATE_APP;
    memcpy(p->decoded.payload.bytes, body, len);
    p->decoded.payload.size = (uint16_t)len;
    service->sendToPhone(p);
}

void TougeFastModule::drainRadio(uint32_t nowMs)
{
    FastRx rx;
    // Bounded. A burst of speech from three cars at once could otherwise keep
    // this loop running past the point where the rest of Meshtastic gets a
    // turn, and a stalled LoRa thread is a worse failure than late audio.
    for (int budget = 0; budget < 8 && fastRadio.poll(rx); budget++) {
        Frame f;
        if (!decodeFrame(rx.data, rx.len, f)) continue;
        if (f.chan != net_.chanByte) continue;      // another group on this channel
        if (f.src == nodeId_) continue;             // our own frame, echoed back
        if (!mesh_.firstSight(f.src, f.id, nowMs)) continue;

        if (f.len > FRAME_MAX_PAYLOAD) continue;

        // The ciphertext is kept as it arrived, because forwarding has to put
        // the same bytes back on the air. Re-encrypting would work, CTR being
        // symmetric, but only for as long as nobody changes the nonce recipe.
        uint8_t sealed[FRAME_MAX_PAYLOAD];
        memcpy(sealed, f.payload, f.len);

        uint8_t body[FRAME_MAX_PAYLOAD];
        memcpy(body, sealed, f.len);
        cipherApply(net_.key, f.src, f.id, f.type, body, f.len);

        if (f.type == FRAME_POSITION) {
            Position p;
            if (decodePosition(body, f.len, p))
                mesh_.note(f.src, p, HEARD_FAST, rx.rssi, FAST_HOPS - f.hops, nowMs);
        }

        inject(f, body, f.len, rx.rssi);

        // Forward for anyone who cannot hear the sender directly. The frame
        // keeps the original sender and id so every copy in flight is the same
        // packet; giving it a new id here would defeat dedupe at the next node
        // and turn a convoy into an echo chamber.
        if (f.hops > 0) {
            Frame fwd = f;
            fwd.hops = f.hops - 1;
            fwd.payload = sealed;
            uint8_t wire[FRAME_MAX];
            size_t n = encodeFrame(fwd, wire, sizeof(wire));
            if (n > 0) fastRadio.send(wire, n);
        }
    }
}

int32_t TougeFastModule::runOnce()
{
    if (nodeId_ == 0) nodeId_ = nodeDB->getNodeNum();

    uint32_t now = millis();
    // The channel only changes when somebody reconfigures the ride, so this is
    // checked on a slow clock. Running it every pass would compare and rederive
    // keys fifty times a second for no reason.
    if (!started_ || (uint32_t)(now - lastSyncMs_) >= SYNC_EVERY_MS) {
        lastSyncMs_ = now;
        syncChannel();
    }
    if (!started_) return 5000; // nothing to do until there is a channel

    drainRadio(now);
    beacon(now);
    mesh_.age(now);

    // Fast enough that a 20 ms audio frame is never sitting in the queue long,
    // and slow enough that an idle board is not spinning.
    return 20;
}

ProcessMessage TougeFastModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    // Only packets the phone aimed at this radio in particular. Router::sendLocal
    // delivers those to modules without transmitting, which is what keeps
    // push-to-talk audio off the LoRa side entirely.
    if (mp.which_payload_variant != meshtastic_MeshPacket_decoded_tag) return ProcessMessage::CONTINUE;
    if (mp.from != nodeDB->getNodeNum()) return ProcessMessage::CONTINUE;
    if (mp.to != nodeDB->getNodeNum()) return ProcessMessage::CONTINUE;

    if (!started_) {
        // Swallowed rather than passed on. Letting it fall through would put
        // speech on LoRa, and 12 kbps of audio would take the mesh down for
        // everyone on it.
        LOG_DEBUG("touge: fast lane down, dropping a %u byte payload", (unsigned)mp.decoded.payload.size);
        return ProcessMessage::STOP;
    }

    transmit(FRAME_VOICE, mp.decoded.payload.bytes, mp.decoded.payload.size, FAST_HOPS);
    return ProcessMessage::STOP;
}

#endif
