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
#include <esp_random.h>
#include <esp_timer.h>
#include <string.h>

using namespace touge;

TougeFastModule *tougeFastModule = nullptr;

namespace {

// One TDMA cycle. Our slot comes round this often, which is both the fastest
// we can beacon and the longest a ready beacon ever waits for its turn.
//
// A hop over ESP-NOW is two or three milliseconds, so the transport was never
// what made a position stale: the interval was. Nine slots across 250 ms puts
// each one at 27 ms, which is ten times the length of a frame and leaves room
// for the clock to be a couple of milliseconds out.
const uint32_t CYCLE_MS = 250;

// The gate, borrowed from the Army's Blue Force Tracker, which reports every
// 30 seconds or every 50 metres of travel and lets whichever comes first win.
//
// The point of a distance trigger is that it bounds the thing that actually
// matters, which is not how old a position is but how wrong it is. Twenty
// metres is about four car lengths: nobody's icon is ever further than that
// from where the car really is, whether it is doing 70 or sitting at a
// junction. A pure time trigger gives you the opposite, spending the most
// airtime on the car that has not moved.
const uint32_t GATE_METRES = 20;

// The heartbeat for a car that is parked. Long, because a stationary car needs
// to prove it is alive and nothing more.
const uint32_t GATE_IDLE_MS = 3000;

// How long a car stays "on 2.4 GHz" for the purpose of outranking its own LoRa
// positions. Short on purpose: when the fast link drops, LoRa has to take over
// without a gap, and a two-second-old position beats none at all.
const uint32_t FAST_PRECEDENCE_MS = 2000;

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

// Silence long enough to mean something is wrong rather than that the road is
// quiet. Every car beacons at least once per idle heartbeat, so four seconds
// is more than one missed beacon from everybody at once.
const uint32_t LOST_MS = 4000;

// A second on each candidate while searching. Long enough to catch a beacon
// from a car on its idle heartbeat, short enough that three channels is three
// seconds rather than a minute.
const uint32_t SCAN_DWELL_MS = 1000;

// How long the reference watches before deciding the channel is unusable, and
// the share of expected beacons below which it moves the ride. Generous: a hop
// costs everyone a few seconds of scanning, so it has to be worth it.
const uint32_t HOP_WINDOW_MS = 10000;
const uint32_t HOP_KEEP_PCT = 40;

// How often the state of the fast lane is printed. Often enough to watch a
// bench of boards find each other, rare enough not to drown the log.
const uint32_t STATUS_EVERY_MS = 5000;

const char *NVS_NAMESPACE = "tougefast";
const char *NVS_ID_KEY = "idceil";

// The clock, at file scope because an interrupt handler cannot be handed a
// context pointer and there is one 2.4 GHz radio anyway.
RideClock rideClock;

#ifdef PIN_GPS_PPS
// IRAM, because the handler must not fault while flash is busy. Meshtastic
// sets this pin to INPUT and never attaches anything, so it is ours to take.
void IRAM_ATTR onGpsPulse()
{
    rideClock.onPulse((uint64_t)esp_timer_get_time());
}
#endif

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
    schedule_.reset();
    rideClock.reset();

    // The whole scheme rests on a second being a whole number of cycles: that
    // is what makes the pulse a cycle boundary and a missed pulse harmless.
    static_assert(1000 % CYCLE_MS == 0, "the cycle must divide a second exactly");

#ifdef PIN_GPS_PPS
    pinMode(PIN_GPS_PPS, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), onGpsPulse, RISING);
#endif

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
    schedule_.reset();
    schedule_.rebuild(nodeDB->getNodeNum(), rideClock.locked((uint64_t)esp_timer_get_time()),
                      mesh_.riders(), MAX_RIDERS);
    wantBeacon_ = false;
    sentOnce_ = false;
    mesh_.seedIds(idCeiling_ ? idCeiling_ - ID_BLOCK : nodeDB->getNodeNum());

    // Which of the three non-overlapping channels this ride starts on. Derived,
    // so two groups in the same car park usually begin apart.
    hop_.begin(net_.chanByte);
    lastHeardMs_ = millis();
    lastScanMs_ = lastHeardMs_;
    lastHopCheckMs_ = lastHeardMs_;
    heardInWindow_ = 0;

    if (!fastRadio.begin(net_)) {
        LOG_WARN("touge: ESP-NOW would not start, LoRa only");
        started_ = false;
        return;
    }
    fastRadio.retuneTo(hop_.channel());
    started_ = true;
    LOG_INFO("touge: fast lane up on wifi channel %u", (unsigned)fastRadio.channel());
}

bool TougeFastModule::transmit(uint8_t type, const uint8_t *body, size_t len, uint8_t hops)
{
    if (!started_ || len > FRAME_MAX_BODY) return false;

    uint8_t payload[FRAME_MAX_PAYLOAD];
    memcpy(payload, body, len);

    Frame f;
    f.type = type;
    f.src = nodeId_;
    f.id = mesh_.nextId();
    f.hops = hops;
    f.chan = net_.chanByte;

    // Encrypted, then tagged over the ciphertext and the header fields that
    // name it. The id has to be settled first, because it is both half the
    // encryption nonce and part of what the tag covers.
    size_t sealed = seal(net_.key, f.src, f.id, f.type, f.chan, payload, len, sizeof(payload));
    if (sealed == 0) return false;
    f.payload = payload;
    f.len = (uint16_t)sealed;

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

    // No fix means nothing worth sending. The other cars keep the last one they
    // heard and show it as ageing, which is more useful than a zero.
    if (!localPosition.has_latitude_i || !localPosition.has_longitude_i) return;
    if (localPosition.latitude_i == 0 && localPosition.longitude_i == 0) return;

    // Two separate questions, deliberately. First: is there anything worth
    // saying? Then: is it our turn to say it? Collapsing them would either
    // give up the slot discipline or let a parked car hold one open.
    if (!wantBeacon_) {
        uint32_t since = (uint32_t)(nowMs - lastBeaconMs_);
        uint32_t moved = sentOnce_ ? distanceM(sentLat_, sentLon_, localPosition.latitude_i,
                                               localPosition.longitude_i)
                                   : GATE_METRES;
        if (!sentOnce_ || moved >= GATE_METRES || since >= GATE_IDLE_MS) wantBeacon_ = true;
    }
    if (!wantBeacon_) return;

    // Our turn comes round once a cycle, so the wait is bounded by that and
    // the gate above decides everything else.
    //
    // GPS first. A pulse-disciplined cycle needs no reference car, so nobody's
    // departure costs the ride its clock, and two cars meeting for the first
    // time are already in step. When the receiver has no fix this falls back
    // to the cycle recovered from the reference car's beacons rather than
    // going quiet.
    uint32_t phase = 0;
    bool mine;
    if (rideClock.phaseMs((uint64_t)esp_timer_get_time(), CYCLE_MS, phase)) {
        mine = schedule_.inSlotAtPhase(phase, CYCLE_MS);
    } else {
        mine = schedule_.inSlot(nowMs, CYCLE_MS);
    }
    if (!mine) return;

    wantBeacon_ = false;
    lastBeaconMs_ = nowMs;
    sentLat_ = localPosition.latitude_i;
    sentLon_ = localPosition.longitude_i;
    sentOnce_ = true;

    Position p;
    p.lat = localPosition.latitude_i;
    p.lon = localPosition.longitude_i;
    p.headingDeg = (uint16_t)(localPosition.ground_track / 1e5);
    p.speedMph = speedToMph((float)localPosition.ground_speed);
    p.hasFix = true;
    p.phoneAttached = false;
    // Tells everyone else whether we are fit to be the reference car.
    p.clockLocked = rideClock.locked((uint64_t)esp_timer_get_time());
    // And which slot we are holding, which is how anyone else knows to stay
    // off it. Until this has gone out once we are invisible to their claim,
    // which is why an unclaimed car free-runs rather than waiting its turn.
    p.slot = schedule_.slot();
    // And which channel we think the ride is on. Every car carries this, so a
    // car that missed a hop learns it from whoever it hears next rather than
    // from an announcement it had one chance at.
    p.hop = hopPack(hop_.index(), hop_.generation());

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
            //
            // An empty long_name is the test, because NodeInfoLite has no
            // nested User to ask about: this firmware flattens long_name and
            // short_name onto the node itself, so there is no has_user to
            // read. No name stored means there is nothing to downgrade.
            if (n && n->long_name[0] == 0) {
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
        if (f.len > FRAME_MAX_PAYLOAD) continue;

        // The ciphertext is kept as it arrived, because forwarding has to put
        // the same bytes back on the air. Re-encrypting would work, CTR being
        // symmetric, but only for as long as nobody changes the nonce recipe.
        uint8_t sealed[FRAME_MAX_PAYLOAD];
        memcpy(sealed, f.payload, f.len);

        uint8_t body[FRAME_MAX_PAYLOAD];
        memcpy(body, sealed, f.len);
        size_t bodyLen = unseal(net_.key, f.src, f.id, f.type, f.chan, body, f.len);
        // Forged, corrupted, or from a ride we are not on. Dropped before it
        // reaches the dedupe table on purpose: the sender and the packet id are
        // in clear on the wire, so anyone can read them off a real frame and
        // replay the header with a payload of their own. Recording that first
        // would let them silence the genuine frame behind it.
        if (bodyLen == 0) continue;

        // Authenticated, so it is genuinely one of ours. That makes it evidence
        // the channel works, which is what the hop decision is measuring.
        lastHeardMs_ = nowMs;
        heardInWindow_++;

        if (!mesh_.firstSight(f.src, f.id, nowMs)) continue;

        if (f.type == FRAME_POSITION) {
            Position p;
            if (decodePosition(body, bodyLen, p)) {
                mesh_.note(f.src, p, HEARD_FAST, rx.rssi, FAST_HOPS - f.hops, nowMs);
                // A newer belief about the channel wins, wherever it comes
                // from. Only acted on after the tag has already passed, so a
                // stranger cannot walk the ride off its channel.
                if (hop_.observe(p.hop)) {
                    LOG_INFO("touge: following a hop to channel %u", (unsigned)hop_.channel());
                    fastRadio.retuneTo(hop_.channel());
                }
                // Every position, not only the ones that change the head count.
                // A car can keep its seat on the roster and still move slot, or
                // gain a GPS fix and become the right car to keep time by, and
                // either of those has to reach the schedule when it happens
                // rather than when somebody else next turns up. Nine slots
                // across eight riders is not work worth conserving.
                schedule_.rebuild(nodeId_, rideClock.locked((uint64_t)esp_timer_get_time()),
                                  mesh_.riders(), MAX_RIDERS);
            }
        }

        // The lowest node number on the ride holds slot zero, so its frame
        // landing is the start of a cycle. Only a frame heard directly is any
        // use for this: one that came via a neighbour carries that neighbour's
        // forwarding jitter and would drag the whole schedule sideways.
        if (f.src == schedule_.referenceId() && f.hops == FAST_HOPS) schedule_.syncTo(rx.rxMs, CYCLE_MS);

        inject(f, body, bodyLen, rx.rssi);

        // Forward for anyone who cannot hear the sender directly. The frame
        // keeps the original sender and id so every copy in flight is the same
        // packet; giving it a new id here would defeat dedupe at the next node
        // and turn a convoy into an echo chamber.
        //
        // Held, not sent. Every car that heard this frame is about to reach
        // this line at the same instant, and if they all transmit together the
        // forward is lost to a collision and helps nobody. Waiting a random
        // slice also gives the others time to go first, and whoever loses the
        // race drops their copy instead of adding to the noise.
        if (f.hops > 0) {
            Frame fwd = f;
            fwd.hops = f.hops - 1;
            fwd.payload = sealed;
            uint8_t wire[FRAME_MAX];
            size_t n = encodeFrame(fwd, wire, sizeof(wire));
            if (n > 0) mesh_.defer(wire, n, f.src, f.id, nowMs + (esp_random() % FORWARD_JITTER_MS));
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
    sendDeferred(now);
    beacon(now);
    mesh_.age(now);
    hopKeeping(now);
    status(now);

    // Fast enough that a 20 ms audio frame is never sitting in the queue long,
    // and slow enough that an idle board is not spinning. It also has to be
    // well under FORWARD_JITTER_MS: at a 20 ms pass every held frame would
    // come due in the same sweep and the jitter would buy nothing.
    return 5;
}

void TougeFastModule::status(uint32_t nowMs)
{
    // One line that says whether any of this is working.
    //
    // Everything interesting here is invisible from outside: which slot a car
    // claimed, who it decided keeps time, whether its clock came from a GPS
    // pulse or from somebody's beacons, how many forwards it did not bother
    // making. None of it shows on the OLED and none of it reaches the phone,
    // so bringing up a bench of boards without this is watching three LEDs
    // and guessing.
    if (!started_) return;
    if ((uint32_t)(nowMs - lastStatusMs_) < STATUS_EVERY_MS) return;
    lastStatusMs_ = nowMs;

    const char *clock = rideClock.locked((uint64_t)esp_timer_get_time()) ? "gps"
                        : schedule_.synced()                            ? "beacon"
                                                                        : "free";
    // A plain buffer rather than String(n).c_str(). The temporary would live
    // just long enough to be correct, which is not a property worth relying on
    // inside a log call somebody will reformat later.
    char slotText[8];
    if (schedule_.claimed()) {
        snprintf(slotText, sizeof(slotText), "%u", (unsigned)schedule_.slot());
    } else {
        strncpy(slotText, "none", sizeof(slotText) - 1);
        slotText[sizeof(slotText) - 1] = 0;
    }

    LOG_INFO("touge: ch=%u slot=%s/%u known=%u ref=%08x%s clock=%s fast=%u suppressed=%u dropped=%u",
             (unsigned)fastRadio.channel(), slotText, (unsigned)MAX_SLOTS,
             (unsigned)schedule_.known(), (unsigned)schedule_.referenceId(),
             schedule_.weAreReference() ? " (us)" : "", clock, (unsigned)fastNeighbours(),
             (unsigned)mesh_.suppressed(), (unsigned)fastRadio.dropped());

    // Signal strength per car, which is the number that settles an argument
    // about antennas.
    //
    // The board offers three ways to wire 2.4 GHz - the on-board trace, an
    // IPEX after lifting an inductor, or both bridged together - and the only
    // honest way to choose is to build two boards differently and read what
    // they hear. Every position is already stamped with the RSSI it arrived
    // at; it simply had nowhere to go.
    const Rider *rs = mesh_.riders();
    for (size_t i = 0; i < MAX_RIDERS; i++) {
        if (!rs[i].used || rs[i].via != HEARD_FAST) continue;
        LOG_INFO("touge:   %08x rssi=%d hops=%u age=%ums %s", (unsigned)rs[i].id, (int)rs[i].rssi,
                 (unsigned)rs[i].hopsAway, (unsigned)(nowMs - rs[i].atMs),
                 rs[i].pos.name[0] ? rs[i].pos.name : "");
    }
}

void TougeFastModule::hopKeeping(uint32_t nowMs)
{
    if (!started_) return;

    uint32_t quiet = (uint32_t)(nowMs - lastHeardMs_);

    // The scan, and the reason any of the rest of this is safe to do.
    //
    // A car that has heard nothing for a while is either alone, or on a
    // channel the ride has left. It cannot tell which, and it must not assume
    // the first: that is the failure where a car sits deaf on an abandoned
    // channel forever, with the news it needs carried only on the channel it
    // is no longer listening to.
    //
    // So it goes and looks. Three candidates, a second on each, and it is back
    // with the group within a few seconds however it came to be lost - a
    // missed hop, switched off during one, or simply joining late.
    if (quiet >= LOST_MS) {
        if ((uint32_t)(nowMs - lastScanMs_) >= SCAN_DWELL_MS) {
            lastScanMs_ = nowMs;
            uint8_t ch = hop_.scanNext();
            LOG_DEBUG("touge: quiet for %ums, listening on channel %u", (unsigned)quiet, (unsigned)ch);
            fastRadio.retuneTo(ch);
        }
        return;
    }

    // Only the car keeping time decides to move, so the ride does not have
    // several cars hopping it in different directions at once.
    if (!schedule_.weAreReference()) return;
    if ((uint32_t)(nowMs - lastHopCheckMs_) < HOP_WINDOW_MS) return;
    lastHopCheckMs_ = nowMs;

    // Expected against received over the window. Every car beacons at least
    // once per idle heartbeat, so a roster that is present but barely audible
    // is interference rather than an empty road.
    size_t cars = mesh_.count();
    uint32_t expect = (uint32_t)cars * (HOP_WINDOW_MS / GATE_IDLE_MS);
    uint32_t got = heardInWindow_;
    heardInWindow_ = 0;
    if (cars == 0 || expect == 0) return;

    if (got * 100 / expect < HOP_KEEP_PCT) {
        hop_.advance();
        LOG_INFO("touge: %u of %u expected beacons, hopping to channel %u", (unsigned)got,
                 (unsigned)expect, (unsigned)hop_.channel());
        fastRadio.retuneTo(hop_.channel());
        // Everyone else is still on the old channel and will not hear the next
        // beacon. They go quiet, they scan, they find us. That is the design
        // rather than a shortcoming of it.
    }
}

void TougeFastModule::sendDeferred(uint32_t nowMs)
{
    Forward f;
    // Bounded for the same reason drainRadio is. Everything left behind comes
    // due again five milliseconds from now.
    for (int budget = 0; budget < 4 && mesh_.nextDue(nowMs, f); budget++) {
        fastRadio.send(f.wire, f.len);
    }
}

bool TougeFastModule::wantPacket(const meshtastic_MeshPacket *p)
{
    if (!p) return false;
    if (p->which_payload_variant != meshtastic_MeshPacket_decoded_tag) return false;
    // Our own port for voice, and positions so that a LoRa one cannot overwrite
    // a fresher 2.4 GHz one. See handleReceived.
    return p->decoded.portnum == ourPortNum || p->decoded.portnum == meshtastic_PortNum_POSITION_APP;
}

ProcessMessage TougeFastModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    // 2.4 GHz beats LoRa, and this is where that is enforced.
    //
    // Both radios feed the same NodeDB row and it keeps whatever arrived last,
    // so a LoRa position sampled two seconds ago can land after a 2.4 GHz one
    // sampled a quarter of a second ago and replace it with the older truth.
    // Meshtastic's own timestamps are in whole seconds, which is far too
    // coarse to sort a 250 ms beacon out from a slow one, so the comparison is
    // made here instead: if we have heard this car on 2.4 GHz recently, its
    // LoRa position is dropped before PositionModule can write it.
    //
    // Recently is deliberately short. The moment 2.4 GHz stops carrying a car,
    // LoRa has to take over without a gap, and a stale position beats none.
    if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag &&
        mp.decoded.portnum == meshtastic_PortNum_POSITION_APP) {
        const Rider *r = mesh_.find(mp.from);
        if (r && r->via == HEARD_FAST && (uint32_t)(millis() - r->atMs) < FAST_PRECEDENCE_MS) {
            return ProcessMessage::STOP;
        }
        return ProcessMessage::CONTINUE;
    }

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
