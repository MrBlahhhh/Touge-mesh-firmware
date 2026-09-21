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

// The heartbeat for a car that is parked.
//
// One a second. Three was chosen when this was only about proving a parked car
// was alive, but the interval is also the floor on how fast anything else can
// notice a car has gone: every other timer here is a multiple of it, so three
// seconds made the whole lane sluggish to state a fact it already knew. A
// stationary car at 1 Hz is a few bytes a second against a 250 ms cycle.
const uint32_t GATE_IDLE_MS = 1000;

// How long a car stays "on 2.4 GHz" for the purpose of outranking its own LoRa
// positions, and for being counted as being on the lane at all.
//
// This has to be longer than the gap between one car's beacons or a healthy
// car falls off the lane between them. It was two seconds against a three
// second idle heartbeat, so a parked car was absent for one second in every
// three: the live count flapped 2, 1, 2, 0, its LoRa position took over each
// time the fast one aged out, and the hop logic read the resulting gaps as a
// bad channel and moved the whole ride off it. Voice would have been declared
// dead between every pair of frames.
//
// Three heartbeats, so a car has to miss three beacons outright before
// anything treats it as gone, and LoRa fills within a few seconds rather than
// most of a minute. At the old three second heartbeat this had to be seven to
// clear two beacons; at 1 Hz the same safety costs three.
const uint32_t FAST_PRECEDENCE_MS = 3000;

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
// quiet.
//
// Everybody has to go quiet for six heartbeats before this board concludes it
// is the one that is lost. Four seconds was barely one heartbeat plus jitter,
// so a board went hunting for a ride that was still there and stopped sending
// held traffic while it hunted.
const uint32_t LOST_MS = 6000;

// How long to listen on each candidate while searching.
//
// Must outlast two beacons, or the search can sweep straight past the ride.
// This was one second against a heartbeat of the same length, so hearing the
// channel you were standing on was a coin toss - and against a board still on
// the old three second heartbeat, roughly one chance in three. Watched on a
// bench: a board cycling 6, 11, 6, 1, 11 with heard=0 the whole time, while
// another sat healthy on channel 6 the entire sweep.
//
// The cost of a longer dwell is a slower sweep, which is the cheaper mistake:
// three channels at two and a half seconds is still under eight seconds to
// find a ride, and a sweep that misses does not converge at all.
const uint32_t SCAN_DWELL_MS = 2500;

// How long the reference watches before deciding the channel is unusable, and
// the share of expected beacons below which it moves the ride.
//
// Both were far too eager. The expectation is cars x (window / heartbeat),
// which at ten seconds was exactly three beacons per car with no margin at
// all, so ordinary jitter read as a failing channel and the board moved the
// whole ride to another one - splitting it, because the others had no reason
// to follow. Seen on a bench with three boards a foot apart and a perfect
// channel.
//
// Thirty seconds averages over ten heartbeats per car, and a quarter of that
// is a channel genuinely carrying almost nothing rather than one having a bad
// second. A hop costs everyone seconds of scanning and risks splitting the
// ride, so it has to be the last explanation left, not the first.
const uint32_t HOP_WINDOW_MS = 30000;
const uint32_t HOP_KEEP_PCT = 25;

// How often the state of the fast lane is printed. Often enough to watch a
// bench of boards find each other, rare enough not to drown the log.
const uint32_t STATUS_EVERY_MS = 5000;

// How old the local fix may be before this board stops beaconing it.
//
// The phone feeds a position every few seconds, so ten is several missed
// updates - long enough to ride out a hiccup, short enough that a radio whose
// phone has gone stops claiming to know where its car is.
const uint32_t POSITION_STALE_S = 10;

// The shortest PSK worth deriving a 2.4 GHz key from.
//
// Meshtastic uses one byte to mean "the default channel, key number N", which
// is public knowledge rather than a secret. A Touge ride always writes a full
// 32 byte PSK, so anything this short is a channel nobody has secured.
const int MIN_PSK_BYTES = 16;

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
    // A car must not age off the fast lane between its own beacons.
    static_assert(FAST_PRECEDENCE_MS > 2 * GATE_IDLE_MS,
                  "the fast-lane window must outlast two idle heartbeats");
    // A search that listens for less than two beacons can miss a live channel.
    static_assert(SCAN_DWELL_MS > 2 * GATE_IDLE_MS,
                  "each scan dwell must outlast two idle heartbeats");

#ifdef PIN_GPS_PPS
    pinMode(PIN_GPS_PPS, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), onGpsPulse, RISING);
#endif

    loadIdCounter();
}

size_t TougeFastModule::fastNeighbours(uint32_t nowMs) const
{
    size_t n = 0;
    const Rider *r = mesh_.riders();
    for (size_t i = 0; i < MAX_RIDERS; i++) {
        if (!r[i].used || r[i].via != HEARD_FAST) continue;
        // Heard on the fast lane recently, not ever.
        //
        // via is only ever set to HEARD_FAST, because that is the only way a
        // position reaches this module, and a rider keeps its seat for ten
        // minutes. So without a window this counted every car heard on
        // 2.4 GHz since the ride started, long after they had dropped back to
        // LoRa, and reported it as the live fast count. That number is what
        // an antenna change gets judged on, so it has to mean what it says.
        if ((uint32_t)(nowMs - r[i].atMs) >= FAST_PRECEDENCE_MS) continue;
        n++;
    }
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
    // No secret, no fast lane.
    //
    // -1 is "no usable key", 0 is "encryption off", and 1 is Meshtastic's
    // default channel: a single byte naming one of the well known keys that
    // ships with the firmware and is on the public internet. None of the three
    // is a secret, and a key derived from a value everybody has is the same key
    // on every board on earth - so every radio on a default channel would share
    // one 2.4 GHz secret, and the tag checks would all pass and make it look
    // authenticated.
    //
    // The first version of this guard tested `<= 0` and let the one byte case
    // straight through, which is the same hole with a longer name. A ride key
    // derives a 32 byte PSK, so anything shorter than a real key is not one.
    if (key.length < MIN_PSK_BYTES) {
        if (started_) {
            LOG_INFO("touge: primary channel has no real key, fast lane down");
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
                      mesh_.riders(), MAX_RIDERS, millis());
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
    if (!fastRadio.retuneTo(hop_.channel())) {
        // Up, but not where we meant to be. Worth one line at boot rather than
        // a silent mismatch that only shows up as a ride nobody can hear.
        LOG_WARN("touge: wanted channel %u, radio stayed on %u (WiFi or MQTT associated?)",
                 (unsigned)hop_.channel(), (unsigned)fastRadio.channel());
    }
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
    // Nothing to say if nobody has told us anything lately.
    //
    // A board keeps beaconing whatever localPosition last held, so a radio
    // left switched on after its phone walked away broadcast a frozen fix at
    // 1 Hz indefinitely - and because a fast position outranks the LoRa one,
    // everyone else pinned that car to a place it had left. Seen tonight: a
    // rider who had closed the app, left the ride and turned her LoRa off was
    // still on the map, because her board was still talking.
    //
    // The position carries the time it was measured. If that has stopped
    // advancing, we have nothing new to say and should say nothing, which also
    // lets the far end fall back to LoRa after FAST_PRECEDENCE_MS rather than
    // preferring our stale copy forever.
    uint32_t nowSec = getValidTime(RTCQualityFromNet);
    if (nowSec > 0 && localPosition.time > 0 &&
        nowSec - localPosition.time > POSITION_STALE_S) {
        wantBeacon_ = false;
        return;
    }

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
    // Whose fix this is, honestly.
    //
    // Hardcoded false, while inject() on the far end stamped every arriving
    // 2.4 GHz position as LOC_EXTERNAL - so a board beaconing its own GPS, or
    // a position frozen because the phone stopped feeding it, arrived
    // everywhere claiming to be a phone fix. The app ranks a phone fix above a
    // radio's own, so the worse position won and held for the whole precedence
    // window with the LoRa copy suppressed behind it.
    p.phoneAttached =
        localPosition.location_source == meshtastic_Position_LocSource_LOC_EXTERNAL;
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
        // The long name, because the frame has room for it.
        //
        // This sent short_name, which is four characters, and the receiving
        // end wrote whatever arrived into long_name as well. So a car called
        // "mattpixel" reached every other car on the fast lane as "matt", and
        // the same node showed one name on the phone holding its own radio
        // and a different one on everybody else's. The field is sixteen bytes
        // and a long name is fifteen at most here, so the short one was never
        // buying anything.
        strncpy(p.name, owner.long_name[0] ? owner.long_name : owner.short_name,
                sizeof(p.name) - 1);
        p.name[sizeof(p.name) - 1] = 0;
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
        // What the sender said it was, not what we wish it were.
        mp.location_source = p.phoneAttached ? meshtastic_Position_LocSource_LOC_EXTERNAL
                                             : meshtastic_Position_LocSource_LOC_INTERNAL;
        mp.time = getValidTime(RTCQualityFromNet);

        nodeDB->updatePosition(f.src, mp, RX_SRC_RADIO);

        // Stamp when we heard them.
        //
        // updatePosition deliberately does not touch last_heard; updateFrom
        // does, off a LoRa packet's rx_time, and a car carried only by the
        // fast lane never goes through it. Without this a car whose position
        // is a quarter of a second old reads as never heard from at all.
        meshtastic_NodeInfoLite *heard = nodeDB->getMeshNode(f.src);
        if (heard) heard->last_heard = mp.time;

        // And hand the phone a packet, which is the entire point of the
        // exercise.
        //
        // NodeDB is not a route to the app. updatePosition ends in
        // notifyObservers, and the things observing that are the screen and
        // other modules, never PhoneAPI: the phone learns positions from
        // packets and only sees the node database on a config dump. Worse,
        // handleReceived below returns STOP for the LoRa copy of any car on
        // the fast lane, and that STOP breaks the module loop before
        // RoutingModule, which owns the only live handleFromRadio call. So
        // the app went blind to precisely the cars the fast lane was working
        // for, while the OLED two feet away looked perfect.
        meshtastic_MeshPacket *pp = router->allocForSending();
        if (pp) {
            pp->from = f.src;
            pp->to = NODENUM_BROADCAST;
            pp->id = f.id;
            pp->channel = channels.getPrimaryIndex();
            pp->hop_limit = 0;
            pp->hop_start = 0;
            pp->rx_rssi = rssi;
            pp->rx_time = mp.time;
            pp->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
            pp->decoded.portnum = meshtastic_PortNum_POSITION_APP;
            size_t n = pb_encode_to_bytes(pp->decoded.payload.bytes, sizeof(pp->decoded.payload.bytes),
                                          &meshtastic_Position_msg, &mp);
            if (n > 0) {
                pp->decoded.payload.size = (uint16_t)n;
                service->sendToPhone(pp);
            } else {
                // An oversized encode is silent and returns 0. Shipping the
                // empty packet would look like a position of nowhere.
                packetPool.release(pp);
            }
        }

        if (p.name[0] != 0) {
            meshtastic_NodeInfoLite *n = nodeDB->getMeshNode(f.src);
            // Fill in a name we do not have, and finish one we cut short.
            //
            // This accepted a name only when long_name was empty, so that a
            // frame could not overwrite a name that arrived the ordinary way.
            // The caution was misplaced twice over. These frames carry an HMAC
            // over the ride key, so they are not unauthenticated; and an
            // earlier build sent short_name over the air, which the receiver
            // wrote into long_name. That left "Jack" where "Jackie" belonged
            // and nothing could ever replace it, because "Jack" is not empty.
            // A wrong name was frozen permanently by a rule meant to protect
            // a right one.
            //
            // So: take a name when there is none, and take one that merely
            // extends what is already stored. "Jack" gives way to "Jackie";
            // "Jackie" does not give way to "Bob". Un-truncating is the only
            // rewrite allowed, which fixes the boards already carrying a
            // clipped name without opening the door the old rule was guarding.
            const bool haveNone = n && n->long_name[0] == 0;
            const bool extends = n && !haveNone &&
                                 strncmp(n->long_name, p.name, strlen(n->long_name)) == 0 &&
                                 strlen(p.name) > strlen(n->long_name);
            if (n && (haveNone || extends)) {
                meshtastic_User u = meshtastic_User_init_default;
                // The frame carries the long name, so the short one is the
                // first few characters of it rather than a copy. Copying the
                // whole thing into a five byte field just truncates it in a
                // second place.
                strncpy(u.long_name, p.name, sizeof(u.long_name) - 1);
                strncpy(u.short_name, p.name, sizeof(u.short_name) - 1);
                u.long_name[sizeof(u.long_name) - 1] = 0;
                u.short_name[sizeof(u.short_name) - 1] = 0;
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

        // Keep time before the dedupe, not after it.
        //
        // This used to sit below, and a beacon whose direct copy was lost
        // while a neighbour's forward of it got through would be dropped by
        // dedupe with the clock never touched. The forward cannot arrive
        // first at a car in direct range, since forwarding means receiving
        // and then waiting out jitter, so that only bit when the direct copy
        // was actually lost. Cheap to get right regardless, and the next
        // beacon no longer has to cover for it.
        //
        // Still direct-only: a forwarded copy carries the forwarder's jitter
        // and would drag the whole schedule sideways. Once per frame, because
        // a genuine retransmission of the same id arrives later than the
        // first and syncing to it would step the epoch backwards.
        if (f.src == schedule_.referenceId() && f.hops == FAST_HOPS &&
            !(f.src == lastSyncSrc_ && f.id == lastSyncId_)) {
            lastSyncSrc_ = f.src;
            lastSyncId_ = f.id;
            schedule_.syncTo(rx.rxMs, CYCLE_MS);
        }

        if (!mesh_.firstSight(f.src, f.id, nowMs)) continue;

        // Distinct frames only. Counting every copy meant a channel busy
        // enough to be flooding itself with duplicates and forwards read as
        // a healthy one, which is exactly backwards for a number whose whole
        // job is deciding whether to stay on this channel.
        heardInWindow_++;

        if (f.type == FRAME_POSITION) {
            Position p;
            if (decodePosition(body, bodyLen, p)) {
                mesh_.note(f.src, p, HEARD_FAST, rx.rssi, FAST_HOPS - f.hops, nowMs);
                // A newer belief about the channel wins, wherever it comes
                // from. Only acted on after the tag has already passed, so a
                // stranger cannot walk the ride off its channel.
                // Snapshot before observe(), which is what moves the belief.
                const uint8_t wasIndex = hop_.index();
                const uint8_t wasGen = hop_.generation();
                if (hop_.observe(p.hop)) {
                    // Only claim the move if the radio actually made it.
                    //
                    // esp_wifi_set_channel fails outright while the station is
                    // associated, which it is whenever Meshtastic's own WiFi or
                    // MQTT is up. Taking the new belief anyway left this node
                    // certain it was on a channel it could not hear, and the log
                    // cheerfully agreed with it.
                    if (fastRadio.retuneTo(hop_.channel())) {
                        LOG_INFO("touge: following a hop to channel %u", (unsigned)hop_.channel());
                    } else {
                        LOG_WARN("touge: could not retune to channel %u, staying on %u",
                                 (unsigned)hop_.channel(), (unsigned)fastRadio.channel());
                        hop_.restore(wasIndex, wasGen);
                    }
                }
                // Every position, not only the ones that change the head count.
                // A car can keep its seat on the roster and still move slot, or
                // gain a GPS fix and become the right car to keep time by, and
                // either of those has to reach the schedule when it happens
                // rather than when somebody else next turns up. Nine slots
                // across eight riders is not work worth conserving.
                schedule_.rebuild(nodeId_, rideClock.locked((uint64_t)esp_timer_get_time()),
                                  mesh_.riders(), MAX_RIDERS, millis());
            }
        }

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
    // Nothing held goes out while we are lost.
    //
    // Once the scan starts, the radio is parked on whichever candidate it is
    // listening to this second, not on the ride's channel. Forwards and speech
    // sent from there reach nobody and spend airtime and battery doing it.
    // Beacons are the exception and still go: a beacon carries our channel
    // belief, so two lost cars that land on the same candidate can find each
    // other with one.
    const bool lost = (uint32_t)(now - lastHeardMs_) >= LOST_MS;
    if (!lost) sendDeferred(now);
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
             schedule_.weAreReference() ? " (us)" : "", clock, (unsigned)fastNeighbours(nowMs),
             (unsigned)mesh_.suppressed(), (unsigned)fastRadio.dropped());

    // The same line, to the phone.
    //
    // Everything above is the only honest account of whether the fast lane is
    // working, and until now it went to a serial cable and nowhere else. So
    // diagnosing a silent 2.4 GHz lane meant unplugging a board from a car and
    // carrying it to a laptop, and from the app the lane was indistinguishable
    // from a quiet road. The phone can infer per-car which lane a position
    // came over, but not why the lane is down, what channel the radio settled
    // on, or whether it can hear anybody at all.
    //
    // Goes out on the private port as JSON, which the app already parses and
    // which ignores keys it does not know, so an older app sees nothing new
    // rather than breaking. sendToPhone only queues for BLE; none of this
    // touches the air.
    {
        char js[192];
        int n = snprintf(
            js, sizeof(js),
            "{\"fl\":{\"ch\":%u,\"sl\":%d,\"kn\":%u,\"fa\":%u,\"ck\":\"%s\",\"sp\":%u,\"dr\":%u}}",
            (unsigned)fastRadio.channel(), schedule_.claimed() ? (int)schedule_.slot() : -1,
            (unsigned)schedule_.known(), (unsigned)fastNeighbours(nowMs), clock,
            (unsigned)mesh_.suppressed(), (unsigned)fastRadio.dropped());
        if (n > 0 && (size_t)n < sizeof(js)) {
            meshtastic_MeshPacket *sp = router->allocForSending();
            if (sp) {
                sp->from = nodeId_;
                sp->to = NODENUM_BROADCAST;
                sp->channel = channels.getPrimaryIndex();
                sp->hop_limit = 0;
                sp->hop_start = 0;
                sp->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
                sp->decoded.portnum = meshtastic_PortNum_PRIVATE_APP;
                memcpy(sp->decoded.payload.bytes, js, (size_t)n);
                sp->decoded.payload.size = (uint16_t)n;
                service->sendToPhone(sp);
            }
        }
    }

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
            if (fastRadio.retuneTo(ch)) {
                LOG_DEBUG("touge: quiet for %ums, listening on channel %u", (unsigned)quiet, (unsigned)ch);
            } else {
                // Stuck on one channel, so the scan cannot do its job. Saying so
                // beats printing a sweep that never happened while the ride is
                // somewhere else.
                LOG_WARN("touge: quiet for %ums but cannot leave channel %u to look",
                         (unsigned)quiet, (unsigned)fastRadio.channel());
            }
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
        const uint8_t wasIndex = hop_.index();
        const uint8_t wasGen = hop_.generation();
        hop_.advance();
        // A hop nobody can perform is worse than staying put: the rest of the
        // ride follows the beacon that announced it, and this node sits on the
        // old channel believing it led them there.
        if (!fastRadio.retuneTo(hop_.channel())) {
            LOG_WARN("touge: %u of %u beacons but the radio will not leave channel %u",
                     (unsigned)got, (unsigned)expect, (unsigned)fastRadio.channel());
            hop_.restore(wasIndex, wasGen);
            return;
        }
        LOG_INFO("touge: %u of %u expected beacons, hopping to channel %u", (unsigned)got,
                 (unsigned)expect, (unsigned)hop_.channel());
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
