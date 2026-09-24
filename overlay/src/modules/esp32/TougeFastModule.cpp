#include "TougeFastModule.h"

#if defined(ARCH_ESP32) && !defined(MESHTASTIC_EXCLUDE_TOUGE_FAST)

#include "Channels.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "RTC.h"
#include "PhoneAPI.h"
#include "Router.h"
#include "main.h"
#include "touge/cipher.h"
#if !MESHTASTIC_EXCLUDE_GPS
#include "gps/GPS.h"
#endif
#include <Preferences.h>
#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <string.h>

// The pre-encoded batch path and the write-drop counters live in NimbleBluetooth.cpp
// (core-patches/0007), which only exists where BLE is built.
#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !MESHTASTIC_EXCLUDE_BLUETOOTH
#define TOUGE_HAS_NIMBLE 1
#else
#define TOUGE_HAS_NIMBLE 0
#endif

using namespace touge;

TougeFastModule *tougeFastModule = nullptr;

namespace {

// One TDMA cycle: the one-second, 32-slot schedule in touge/schedule.h. Our
// slot comes round once a cycle, which caps a car at one beacon a second.
const uint32_t CYCLE_MS = SCHEDULE_MS;

// Least time between any two of our beacons. Extra slots are 250 ms apart and
// clear of the lease slot, so this only stops a second send in the same slot
// on the next tick.
const uint32_t EXTRA_MIN_GAP_MS = 100;
static_assert(EXTRA_MIN_GAP_MS > SLOT_MS && EXTRA_MIN_GAP_MS < BLOCK_MS,
              "longer than one slot, shorter than the gap between a row's slots");

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

// How often one car's position is handed to the phone. See inject().
const uint32_t PHONE_POSITION_MS = 1000;

static_assert(PHONE_POSITION_MS >= CYCLE_MS,
              "throttling faster than the lane produces would do nothing");

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

// The pass while there is no ride channel. Only the GNSS forward has work to
// do then, and it wants about a second.
const uint32_t IDLE_PASS_MS = 1000;

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

// What build of the Touge module this is.
//
// Meshtastic's own version string comes from its git hash and does not move
// when this overlay changes, so "is this board running the current firmware?"
// was unanswerable from the phone - which is how an evening went by with three
// boards on three different sets of timing constants and no way to tell. Bump
// it whenever the on-air behaviour changes.
// 26: core-patches/0003, the BLE advertising restart a disconnect could lose.
// 27: the receiver's own fix to the phone ("gf"), and GNSS speed read as km/h.
// 28: core-patches/0004, a board that never found a GNSS stops probing for one.
// 29: early beacons keep the 1 s deadline, early phone positions are held, and km/h + track e5 both ways.
// 30: 1 s 32-slot leased schedule (frame v2); positions to the phone in newest-wins batches
//     (0xC1) once the app says hello, fs/fq link counters, core-patches/0005-0007.
// 31: batch record ages clamp at zero instead of wrapping to 65.5 s.
// 32: extra beacons in free slots of a car's own row, up to 4 Hz with few cars
//     (frame flag 0x10, not forwarded); the phone sends each new fix at once.
// 33: core-patches/0008, a 3 s button hold shuts down with the button still held.
const uint32_t TOUGE_BUILD = 33;

// How long a board hunts before giving up and waiting at home.
//
// Three channels at two and a half seconds is one sweep in seven and a half,
// so this is about three sweeps. Long enough that a board which is merely
// between beacons does not abandon a working channel; short enough that a
// split ride reconverges in well under a minute.
const uint32_t HOME_AFTER_MS = 25000;

// Whether the ride hops channels to escape interference.
//
// Off. Every car derives the same channel from the ride key and stays on it,
// which is the only arrangement that reliably lets two cars find each other on
// 2.4 GHz - the scan meant to reunite a car that fell behind instead kept two
// lost cars permanently out of phase across 1/6/11. The hop machinery is intact
// and can be turned back on once the scan is made phase-stable; until then a
// fixed channel is what carries. This is what every helmet intercom does.
const bool FAST_LANE_HOP = false;


// How often runOnce is asked to look at the world.
//
// Was a bare `return 5` at the bottom of runOnce. It is named here because two
// other things depend on it: a beacon cannot leave its slot any more promptly
// than this, and that is what bounds how far a clock drifts as it is handed
// down the convoy.
const uint32_t TICK_MS = 5;

// The earliest the fast lane may start WiFi after boot. A floor, not the gate:
// the gate is Bluetooth being up. See bleSettled.
const uint32_t FAST_LANE_START_DELAY_MS = 8000;

// How long after the BLE server exists before WiFi may start, for NimBLE to
// finish building and starting its advertising, which is where it allocates.
const uint32_t FAST_LANE_AFTER_BLE_MS = 2000;

// The longest the fast lane waits for Bluetooth at all. A board with Bluetooth
// switched off never gets a BLE server, and must not sit off 2.4 GHz for good
// waiting for one.
const uint32_t FAST_LANE_BLE_WAIT_MAX_MS = 45000;

// What a listener assumes about how late a beacon was.
//
// A beacon leaves on the first tick at or after its slot opens, so it is
// between zero and TICK_MS late, averaging half that. Subtracting the average
// is what stops the error compounding: a lag that is always positive adds up
// hop after hop, while one that is as often early as late cancels out. Rounded
// up, because being a shade early inside your own slot costs nothing and being
// late costs a collision.
const uint32_t SYNC_BIAS_MS = TICK_MS / 2 + 1;

static_assert(SYNC_BIAS_MS * 2 >= TICK_MS, "the correction has to cover the tick it is for");

// The frame-size asserts on the slot layout live in schedule.h. These are the
// ones that also depend on the tick.

// A tick has to be able to land in what is left of a slot once the guard is
// out of it, or a car can be refused its turn every cycle.
static_assert(SLOT_MS > SLOT_GUARD_MS + TICK_MS,
              "the usable part of a slot must outlast a tick");
// The same for an unleased car at the latest start its spread can give it.
static_assert(SHARED_MS >= SHARED_SPREAD_MS + SLOT_GUARD_MS + TICK_MS,
              "the shared window must outlast a tick at every spread offset");

// The deepest car in the convoy still has to fit inside its slot.
//
// Its epoch carries up to SYNC_BIAS_MS of error per hop from the reference plus
// its own wait, and its frame has to finish before the next slot opens: 3 ms
// x 6 + 8 ms = 26 ms against a 27 ms slot. This is what sets SLOT_MS.
static_assert(SYNC_BIAS_MS * (MAX_REF_HOPS + 1) + FRAME_AIRTIME_MS <= SLOT_MS,
              "a car at MAX_REF_HOPS would transmit outside its slot: shorten the chain, "
              "widen the slots, or make the tick faster");

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
    // One lease is one slot a second. A heartbeat faster than the schedule
    // would queue beacons behind the slot and never catch up.
    static_assert(GATE_IDLE_MS >= CYCLE_MS,
                  "a car's slot comes once a cycle, so it cannot heartbeat faster");

#ifdef PIN_GPS_PPS
    pinMode(PIN_GPS_PPS, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), onGpsPulse, RISING);
#endif

    loadIdCounter();

    phoneStore_.clear();
    batchesInFlight_.clear();
    // Counts what the phone actually read (core-patches/0006).
    phonePacketDeliveredHook = &TougeFastModule::onPhoneDelivered;
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
        // On the channel we are sitting on now, not one we have since left.
        if (r[i].chan != fastRadio.channel()) continue;
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
    announcedSlot_ = SLOT_NONE;
    nextBeaconMs_ = 0;
    mesh_.seedIds(idCeiling_ ? idCeiling_ - ID_BLOCK : nodeDB->getNodeNum());

    // Which of the three non-overlapping channels this ride starts on. Derived,
    // so two groups in the same car park usually begin apart.
    hop_.begin(net_.chanByte);
    lastHeardMs_ = millis();
    lastScanMs_ = lastHeardMs_;
    lastHopCheckMs_ = lastHeardMs_;
    heardInWindow_ = 0;

    if (!fastRadio.begin(net_)) {
        LOG_WARN("touge: ESP-NOW would not start (%s, heap %u free, %u largest), LoRa only",
                 esp_err_to_name((esp_err_t)fastRadio.beginError()),
                 (unsigned)fastRadio.beginFreeHeap(), (unsigned)fastRadio.beginLargestBlock());
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
    const bool sent = fastRadio.send(wire, n);
    if (sent) stats_.fast.tx++;
    return sent;
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
    // The staleness mute is gone, and the age is reported instead.
    //
    // It silenced a board whose phone was connected and exchanging normally:
    // every status report from that radio read MUTED(stale fix) while the
    // other board heard nothing from it, and the whole lane was down because
    // of a guard meant to protect it. localPosition.time evidently does not
    // advance the way this assumed.
    //
    // A board with no phone beaconing a frozen fix is a real problem and this
    // is not the way to detect it. The age goes into the status report so the
    // right signal can be chosen from evidence rather than from another guess.
    uint32_t nowSec = getValidTime(RTCQualityFromNet);
    uint32_t fixAge = (nowSec > 0 && localPosition.time > 0 && nowSec > localPosition.time)
                          ? nowSec - localPosition.time
                          : 0;
    (void)fixAge;

    // Extra beacons use free slots of our row, never the lease slot, so the two
    // never compete for the same tick.
    if (sendExtraBeacon(nowMs)) return;

    if (!wantBeacon_) {
        uint32_t moved = sentOnce_ ? distanceM(sentLat_, sentLon_, localPosition.latitude_i,
                                               localPosition.longitude_i)
                                   : GATE_METRES;
        // Time trigger on a fixed 1 s grid, not GATE_IDLE_MS after the last
        // actual send. The send waits for our TDMA slot, up to a cycle, and
        // measuring the next interval from the send folded that wait into every
        // gap - a steady drift toward 1.25 s. nextBeaconMs_ advances from
        // itself, so however long the slot wait ran the cadence holds on the
        // second. Distance still triggers an extra beacon between deadlines.
        bool timeDue = !sentOnce_ || (int32_t)(nowMs - nextBeaconMs_) >= 0;
        if (timeDue || moved >= GATE_METRES) {
            wantBeacon_ = true;
            // Leases lapse and the join listen ends on the clock, not on a
            // frame arriving, so the schedule is re-read once per beacon too.
            // Otherwise a car left alone would hold its lease indefinitely.
            schedule_.rebuild(nodeId_, rideClock.locked((uint64_t)esp_timer_get_time()),
                              mesh_.riders(), MAX_RIDERS, nowMs);
            schedule_.drawSharedTurn(esp_random());
        }
    }
    // A lease nobody has heard yet goes out in its first slot. Waiting for the
    // 1 s deadline could leave it unsent for two seconds, long enough for the
    // neighbours' slot maps to say nobody hears it and the clash check to give
    // it up.
    if (schedule_.claimed() && schedule_.slot() != announcedSlot_) wantBeacon_ = true;
    if (!wantBeacon_) return;

    // Our turn comes round once a cycle, so the wait is bounded by that and
    // the gate above decides everything else.
    //
    // GPS first. A pulse-disciplined cycle needs no reference car, so nobody's
    // departure costs the ride its clock, and two cars meeting for the first
    // time are already in step. When the receiver has no fix this falls back
    // to the cycle recovered from the reference car's beacons rather than
    // going quiet.
    // A reference with nobody to sync to declares the cycle itself, rather
    // than free-running and dragging everyone else's slots along behind it.
    if (schedule_.weAreReference()) schedule_.startEpoch(nowMs);

    uint32_t phase = 0;
    bool mine;
    if (rideClock.phaseMs((uint64_t)esp_timer_get_time(), CYCLE_MS, phase)) {
        mine = schedule_.inSlotAtPhase(phase);
    } else {
        mine = schedule_.inSlot(nowMs);
    }
    if (!mine) return;

    wantBeacon_ = false;
    lastBeaconMs_ = nowMs;
    announcedSlot_ = schedule_.slot();
    // Move the 1 s deadline on only if it has passed. Advancing it on every
    // send let each movement-triggered beacon push it a second later, and a
    // run of them left a car that stopped silent for several seconds.
    if (nextBeaconMs_ == 0) nextBeaconMs_ = nowMs;
    nextBeaconMs_ = nextOnGrid(nextBeaconMs_, GATE_IDLE_MS, nowMs);
    sentLat_ = localPosition.latitude_i;
    sentLon_ = localPosition.longitude_i;
    sentOnce_ = true;

    Position p;
    fillBeacon(p, nowMs);

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

bool TougeFastModule::sendExtraBeacon(uint32_t nowMs)
{
    // Only a fix the ride has not had yet. A parked car, or a phone whose GPS
    // gives one fix a second, has nothing new between lease beacons, and a
    // repeat is airtime for nothing.
    if (!sentOnce_) return false;
    if (localPosition.latitude_i == sentLat_ && localPosition.longitude_i == sentLon_) return false;
    // Extras sit 250 ms apart in the row; this only stops a second send in the
    // same slot on the next tick.
    if ((uint32_t)(nowMs - lastBeaconMs_) < EXTRA_MIN_GAP_MS) return false;

    uint32_t phase = 0;
    bool mine;
    if (rideClock.phaseMs((uint64_t)esp_timer_get_time(), CYCLE_MS, phase)) {
        mine = schedule_.inExtraSlotAtPhase(phase);
    } else {
        mine = schedule_.inExtraSlot(nowMs);
    }
    if (!mine) return false;

    lastBeaconMs_ = nowMs;
    sentLat_ = localPosition.latitude_i;
    sentLon_ = localPosition.longitude_i;

    Position p;
    fillBeacon(p, nowMs);
    p.extra = true;
    uint8_t body[POSITION_MIN];
    size_t n = encodePosition(p, body, sizeof(body));
    // No hops: an extra is for the cars that hear us directly. Forwarding it
    // would multiply the flood by the extra rate; the tail still gets our
    // lease beacon, forwarded, once a second.
    if (n > 0) transmit(FRAME_POSITION, body, n, 0);
    return true;
}

void TougeFastModule::fillBeacon(Position &p, uint32_t nowMs)
{
    p.lat = localPosition.latitude_i;
    p.lon = localPosition.longitude_i;
    // Meshtastic's units, whoever wrote the fix: ground_track is degrees x 1e5
    // (GPS.cpp) and ground_speed whole km/h (the proto comment, and GPS.cpp via
    // TinyGPS kmph()). From build 29 the app sends the same, so one conversion
    // covers the phone's LOC_EXTERNAL fix and the board's own receiver.
    p.headingDeg = (uint16_t)(localPosition.ground_track / 100000);
    p.speedMph = speedToMph((float)localPosition.ground_speed / 3.6f);
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
    // Our lease, so everyone else stays off it, and the generations that
    // decide who keeps a slot two cars claim. Unleased cars send SLOT_NONE
    // from the shared window.
    p.slot = schedule_.slot();
    p.leaseGen = schedule_.leaseGeneration();
    p.schedGen = schedule_.generation();
    schedule_.fillSlotMap(nowMs, p.slotMap);
    // And which channel we think the ride is on. Every car carries this, so a
    // car that missed a hop learns it from whoever it hears next rather than
    // from an announcement it had one chance at.
    p.hop = hopPack(hop_.index(), hop_.generation());
    // Who we think is keeping time, and how far away that is.
    //
    // Relayed, not merely reported: a car that cannot hear the reference still
    // tells its neighbours about it, so the claim walks the length of the
    // convoy one hop per beacon. Without this every neighbourhood elected its
    // own timekeeper and each was free to hop channel on its own, splitting
    // the ride into groups that were never lost enough to go looking for each
    // other.
    p.refId = schedule_.referenceId();
    p.refHops = schedule_.hopsToReference();
    p.refLocked = schedule_.referenceLocked();

    uint8_t battery = powerStatus ? (uint8_t)powerStatus->getBatteryChargePercent() : 255;
    p.batteryPct = battery;
}

meshtastic_Position TougeFastModule::asMeshPosition(const Position &p)
{
    meshtastic_Position mp = meshtastic_Position_init_default;
    mp.latitude_i = p.lat;
    mp.longitude_i = p.lon;
    mp.has_latitude_i = true;
    mp.has_longitude_i = true;
    // Same units as the beacon reads. Both fields are proto3 optional, so
    // without the has_ flags the encoder drops them and the phone sees a car
    // with no speed and no heading.
    mp.ground_track = (uint32_t)p.headingDeg * 100000;
    mp.has_ground_track = true;
    mp.ground_speed = (uint32_t)(p.speedMph * 1.609344f + 0.5f);
    mp.has_ground_speed = true;
    // What the sender said it was, not what we wish it were.
    mp.location_source = p.phoneAttached ? meshtastic_Position_LocSource_LOC_EXTERNAL
                                         : meshtastic_Position_LocSource_LOC_INTERNAL;
    mp.time = getValidTime(RTCQualityFromNet);
    return mp;
}

void TougeFastModule::reassertFastPositions()
{
    for (size_t i = 0; i < restoreCount_; i++) {
        const Rider *r = mesh_.find(restore_[i]);
        // Gone from the roster, or no longer a fast-lane car: the LoRa copy
        // that just landed is the best thing we have and it stays.
        if (r == nullptr || r->via != HEARD_FAST) continue;
        meshtastic_Position mp = asMeshPosition(r->pos);
        nodeDB->updatePosition(restore_[i], mp, RX_SRC_RADIO);
    }
    restoreCount_ = 0;
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
        meshtastic_Position mp = asMeshPosition(p);

        nodeDB->updatePosition(f.src, mp, RX_SRC_RADIO);

        // Stamp when we heard them.
        //
        // updatePosition deliberately does not touch last_heard; updateFrom
        // does, off a LoRa packet's rx_time, and a car carried only by the
        // fast lane never goes through it. Without this a car whose position
        // is a quarter of a second old reads as never heard from at all.
        meshtastic_NodeInfoLite *heard = nodeDB->getMeshNode(f.src);
        if (heard) heard->last_heard = mp.time;

        // And to the phone, which learns positions from packets, never from
        // NodeDB outside a config dump.
        //
        // An app that has said hello gets the newest position per car in
        // batches (SCALE-PLAN step 3). Anything else, including build 29
        // apps, gets build 29's one packet per car per second, with an early
        // arrival held for its deadline by sendHeldPhonePositions.
        if (batchingToPhone()) {
            offerToPhone(phoneRecordFor(f, p, rssi));
        } else if (mesh_.phoneDue(f.src, f.id, PHONE_POSITION_MS, millis())) {
            sendPositionToPhone(f.src, f.id, mp, rssi);
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
    p->has_rx_rssi = rssi != 0; // see the position path above
    p->rx_time = getValidTime(RTCQualityFromNet);
    p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    p->decoded.portnum = meshtastic_PortNum_PRIVATE_APP;
    memcpy(p->decoded.payload.bytes, body, len);
    p->decoded.payload.size = (uint16_t)len;
    service->sendToPhone(p);
}

void TougeFastModule::sendPositionToPhone(uint32_t src, uint32_t packetId, const meshtastic_Position &mp,
                                          int8_t rssi)
{
    meshtastic_MeshPacket *pp = router->allocForSending();
    if (!pp) {
        stats_.dropAlloc++;
        return;
    }
    stats_.fast.queued++;
    pp->from = src;
    pp->to = NODENUM_BROADCAST;
    pp->id = packetId;
    pp->channel = channels.getPrimaryIndex();
    pp->hop_limit = 0;
    pp->hop_start = 0;
    // rx_rssi has explicit presence in this Meshtastic: without the has_ flag
    // the number never reaches the phone. Zero is what an old core gives when
    // it cannot see the RSSI; leave that absent.
    pp->rx_rssi = rssi;
    pp->has_rx_rssi = rssi != 0;
    pp->rx_time = mp.time;
    pp->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    pp->decoded.portnum = meshtastic_PortNum_POSITION_APP;
    size_t n = pb_encode_to_bytes(pp->decoded.payload.bytes, sizeof(pp->decoded.payload.bytes),
                                  &meshtastic_Position_msg, &mp);
    if (n == 0) {
        // An oversized encode is silent and returns 0. Shipping the empty
        // packet would look like a position of nowhere.
        packetPool.release(pp);
        return;
    }
    pp->decoded.payload.size = (uint16_t)n;
    service->sendToPhone(pp);
}

void TougeFastModule::sendHeldPhonePositions(uint32_t nowMs)
{
    // Positions inject() held back for arriving before their car's deadline.
    // The roster slot carries the newest one heard. Bounded like sendDeferred;
    // anything left over is due again on the next tick.
    for (int budget = 0; budget < 4; budget++) {
        const Rider *r = mesh_.nextPhonePending(PHONE_POSITION_MS, nowMs);
        if (r == nullptr) return;
        sendPositionToPhone(r->id, r->phoneFrameId, asMeshPosition(r->pos), (int8_t)r->rssi);
    }
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
        //
        // And evidence about where the ride is. A sweep moves the radio and
        // leaves the belief alone, so a board that finds the ride mid-sweep
        // would otherwise stop sweeping with the radio on one channel and
        // index_ naming another, and every later decision made against the
        // wrong one. Adopting costs nothing when we were already here.
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
        // The car we take the clock from, which is the reference when we can
        // hear it and otherwise the neighbour nearest to it. Waiting for the
        // reference itself would mean a car three hops down the line waiting
        // for a beacon that is never going to arrive.
        //
        // Positions only: they are the one frame sent in the sender's slot.
        // Text and voice go whenever they are ready, and syncing to one put
        // the epoch wherever it happened to land. The slot comes from the
        // beacon itself, so a parent that has just moved slot is read right.
        Position syncBeacon;
        if (schedule_.takesClockFrom(f.src) && f.hops == FAST_HOPS && f.type == FRAME_POSITION &&
            !(f.src == lastSyncSrc_ && f.id == lastSyncId_) &&
            decodePosition(body, bodyLen, syncBeacon) && !syncBeacon.extra) {
            lastSyncSrc_ = f.src;
            lastSyncId_ = f.id;
            schedule_.syncTo(rx.rxMs, syncBeacon.slot, SYNC_BIAS_MS);
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
                stats_.fast.rx++;
                // rx.chan, not fastRadio.channel(): the frame is filed under
                // the channel it arrived on, not the one the radio happens to
                // be sitting on by the time this loop reaches it. A hop
                // between the two stamped everything still in the queue with
                // the new channel, and countOn then read those as cars that
                // had already moved - the hop counting its own backlog as
                // proof it had succeeded.
                // An extra beacon is sent with no hops left but comes straight
                // from its sender, so it counts as direct for the roster.
                const uint8_t hopsAway = p.extra ? 0 : (uint8_t)(FAST_HOPS - f.hops);
                mesh_.note(f.src, p, HEARD_FAST, rx.rssi, hopsAway, nowMs, rx.chan);
                // Lease beacons heard directly only: the slot map says who got
                // through in which slot. A forward says nothing about that, and
                // an extra was not sent in the slot it names, so counting it
                // could make a clashed lease look heard.
                if (f.hops == FAST_HOPS && !p.extra) schedule_.heardSlot(p.slot, f.src, nowMs);
                // A newer belief about the channel wins, wherever it comes
                // from. Only acted on after the tag has already passed, so a
                // stranger cannot walk the ride off its channel.
                // Where the ride actually is, now that the sender has told us
                // what it believes.
                //
                // A sweep retunes the radio and deliberately leaves the belief
                // alone, so a board that finds the ride mid-sweep would
                // otherwise stop sweeping with the radio on one channel and
                // its belief naming another. Gated on the sender's generation
                // so that hearing an older belief cannot undo a hop we are in
                // the middle of announcing.
                uint8_t heardIndex = 0;
                uint8_t heardGen = 0;
                hopUnpack(p.hop, heardIndex, heardGen);
                hop_.adopt(fastRadio.channel(), heardGen);

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
                        LOG_INFO("touge: following %08x to channel %u (gen %u, was %u/%u)",
                                 (unsigned)f.src, (unsigned)hop_.channel(),
                                 (unsigned)hop_.generation(), (unsigned)wasIndex, (unsigned)wasGen);
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
                // rather than when somebody else next turns up. A pass over 28
                // riders and 32 slots is cheap.
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
            // Weakest hearer first, over a window that widens with the
            // number of cars in earshot. A flat slice meant the earliest
            // third of hearers transmitted before a single copy had been
            // counted, so the suppression below never got a chance to run.
            if (n > 0) {
                const uint32_t spread = forwardSpreadMs(fastNeighbours(nowMs));
                mesh_.defer(wire, n, f.src, f.id,
                            nowMs + forwardDelayMs(rx.rssi, spread, esp_random()));
            }
        }
    }
}

namespace
{
uint32_t bleUpMs = 0;

// Whether Bluetooth has had its memory, so WiFi may take what is left.
//
// This used to be a fixed eight seconds after boot, on the measurement that
// Meshtastic had brought BLE up by then. On a Heltec V4 it had not: NimBLE
// started at 11.4 s, found WiFi already holding the heap, and could not
// allocate its advertising data -
//
//     E (12129) BLE_INIT: Malloc failed
//     start(): Error setting advertisement data; rc=519
//
// - so the board booted, ran, and was invisible to every phone. Waiting on the
// event rather than a guess at when it happens is the fix; the eight seconds
// stays as a floor.
bool bleSettled(uint32_t now)
{
    if (now < FAST_LANE_START_DELAY_MS) return false;
#if defined(ARCH_ESP32) && !defined(CONFIG_IDF_TARGET_ESP32S2) && !MESHTASTIC_EXCLUDE_BLUETOOTH
    if (!config.bluetooth.enabled) return true;
    if (nimbleBluetooth && nimbleBluetooth->isActive()) {
        if (bleUpMs == 0) bleUpMs = now;
        return (uint32_t)(now - bleUpMs) >= FAST_LANE_AFTER_BLE_MS;
    }
    return now >= FAST_LANE_BLE_WAIT_MAX_MS;
#else
    return true;
#endif
}
} // namespace

int32_t TougeFastModule::runOnce()
{
    if (nodeId_ == 0) nodeId_ = nodeDB->getNodeNum();

    uint32_t now = millis();

    // Let Bluetooth win the memory race.
    //
    // Bringing up the fast lane starts the WiFi driver, which is tens of
    // kilobytes of heap. On a board with no PSRAM that is enough that NimBLE's
    // setup, which runs a little later in Meshtastic's boot, cannot allocate
    // its advertising object - and NimBLE calls an unguarded `new`, so the
    // failure is an uncaught bad_alloc that aborts the whole device into a boot
    // loop. Our own WiFi bring-up, by contrast, checks its return and falls
    // back to LoRa-only.
    //
    // So we wait until Meshtastic has initialised BLE and taken its memory; if
    // what is left is not enough for WiFi, the fast lane degrades to LoRa
    // rather than taking Bluetooth - and the phone link - down with it. A few
    // seconds of no 2.4 GHz at boot is invisible next to a radio no phone can
    // find. Once open, the gate stays open.
    static bool gateOpen = false;
    if (!gateOpen) {
        if (!bleSettled(now)) return 250;
        gateOpen = true;
        LOG_INFO("touge: bluetooth settled at %u ms, starting fast lane", (unsigned)now);
    }

    // First thing in the tick: PositionModule ran a few milliseconds ago and
    // may have written a stale LoRa fix over a fresher 2.4 GHz one. See
    // handleReceived.
    reassertFastPositions();
    // The channel only changes when somebody reconfigures the ride, so this is
    // checked on a slow clock. Running it every pass would compare and rederive
    // keys fifty times a second for no reason.
    //
    // With no channel this pass used to run every 5 s and sync each time. The
    // pass is now 1 s so the GNSS fix keeps reaching the phone, and the sync
    // stays at 5 s so a failing WiFi bring-up is not retried five times as often.
    const uint32_t syncEveryMs = started_ ? SYNC_EVERY_MS : IDLE_PASS_MS * 5;
    if ((uint32_t)(now - lastSyncMs_) >= syncEveryMs) {
        lastSyncMs_ = now;
        syncChannel();
    }
    // Before the channel check: a tablet with no GPS needs the radio's fix
    // whether or not a ride is set up.
    forwardGnssFix(now);
    if (!started_) return IDLE_PASS_MS; // nothing else to do until there is a channel

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
    sendHeldPhonePositions(now);
    trackPhoneLink(now);
    flushPhoneBatch(now);
    beacon(now);
    mesh_.age(now);
    hopKeeping(now);
    status(now);

    // Fast enough that a 20 ms audio frame is never sitting in the queue long,
    // and slow enough that an idle board is not spinning. It also has to be
    // well under FORWARD_JITTER_MS: at a 20 ms pass every held frame would
    // come due in the same sweep and the jitter would buy nothing. And it is
    // what bounds how promptly a beacon can leave its slot, which is what
    // limits how long the sync chain can be - see SYNC_BIAS_MS.
    return (int32_t)TICK_MS;
}

void TougeFastModule::forwardGnssFix(uint32_t nowMs)
{
#if !MESHTASTIC_EXCLUDE_GPS
    // Stock Meshtastic writes a new GNSS fix into NodeDB and nowhere the phone
    // can see until it reconnects. See touge/gnssfix.h.
    if (!gps || !gps->hasLock()) return;
    // Nobody to hear it. Queued anyway, 32 of these would fill the phone queue
    // and push out the first real packets after a reconnect.
    if (service->api_state == MeshService::STATE_DISCONNECTED) return;

    // GPS::p is the receiver's own solution. localPosition is not: the
    // phone's LOC_EXTERNAL fix overwrites it, and sending that back would be
    // the phone's own fix wearing the radio's label.
    const meshtastic_Position &rx = gps->p;
    GnssFix fix;
    fix.latE7 = rx.latitude_i;
    fix.lonE7 = rx.longitude_i;
    // Above the ellipsoid, like an Android Location's altitude, not MSL.
    fix.altitudeM = rx.altitude_hae;
    fix.speedKmh = rx.ground_speed;
    fix.trackE5 = rx.ground_track;
    fix.sats = rx.sats_in_view;
    fix.hdopE2 = rx.HDOP;
    fix.fixTimeSec = rx.timestamp;
    if (!gnssForward_.due(fix, nowMs)) return;

    char js[160];
    size_t n = formatGnssFix(fix, js, sizeof(js));
    if (n == 0) return;
    meshtastic_MeshPacket *gp = router->allocForSending();
    if (!gp) return;
    gp->from = nodeDB->getNodeNum();
    gp->to = NODENUM_BROADCAST;
    gp->channel = channels.getPrimaryIndex();
    gp->hop_limit = 0;
    gp->hop_start = 0;
    gp->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    gp->decoded.portnum = meshtastic_PortNum_PRIVATE_APP;
    memcpy(gp->decoded.payload.bytes, js, n);
    gp->decoded.payload.size = (uint16_t)n;
    // Straight to the BLE queue, like the status report. Never on the air.
    service->sendToPhone(gp);
    gnssForward_.sent(fix, nowMs);
#else
    (void)nowMs;
#endif
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
    const uint32_t statusWindowMs = nowMs - lastStatusMs_;
    lastStatusMs_ = nowMs;
    reportLinkStats(nowMs, statusWindowMs);

    uint32_t nowSec = getValidTime(RTCQualityFromNet);
    const char *clock = rideClock.locked((uint64_t)esp_timer_get_time()) ? "gps"
                        : schedule_.synced()                            ? "beacon"
                                                                        : "free";
    // A plain buffer rather than String(n).c_str(). The temporary would live
    // just long enough to be correct, which is not a property worth relying on
    // inside a log call somebody will reformat later.
    // The lease generation rides along so a log shows who would win a clash.
    char slotText[24];
    if (schedule_.claimed()) {
        // And how many extra slots it has this second, 0 to 3.
        snprintf(slotText, sizeof(slotText), "%u@g%u+%u", (unsigned)schedule_.slot(),
                 (unsigned)schedule_.leaseGeneration(),
                 (unsigned)__builtin_popcount(schedule_.extraSlots()));
    } else {
        snprintf(slotText, sizeof(slotText), "none@g%u", (unsigned)schedule_.generation());
    }

    LOG_INFO("touge: ch=%u slot=%s/%u known=%u ref=%08x%s +%uhop via=%08x clock=%s fast=%u "
             "suppressed=%u dropped=%u txfail=%u(%d) txpwr=%ddBm",
             (unsigned)fastRadio.channel(), slotText, (unsigned)MAX_SLOTS,
             (unsigned)schedule_.known(), (unsigned)schedule_.referenceId(),
             schedule_.weAreReference() ? " (us)" : "", (unsigned)schedule_.hopsToReference(),
             (unsigned)schedule_.parentId(), clock, (unsigned)fastNeighbours(nowMs),
             (unsigned)mesh_.suppressed(), (unsigned)fastRadio.dropped(),
             (unsigned)fastRadio.sendFailed(), fastRadio.lastSendError(),
             (int)fastRadio.txPowerDbm());

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
        char js[224];
        int n = snprintf(
            js, sizeof(js),
            "{\"fl\":{\"ch\":%u,\"sl\":%d,\"kn\":%u,\"fa\":%u,\"ck\":\"%s\",\"sp\":%u,\"dr\":%u,\"fw\":%u,\"fix\":%u,\"gi\":%u,\"gg\":%u,\"tf\":%u}}",
            (unsigned)fastRadio.channel(), schedule_.claimed() ? (int)schedule_.slot() : -1,
            (unsigned)schedule_.known(), (unsigned)fastNeighbours(nowMs), clock,
            (unsigned)mesh_.suppressed(), (unsigned)fastRadio.dropped(),
            (unsigned)TOUGE_BUILD,
            (unsigned)((nowSec > 0 && localPosition.time > 0 && nowSec > localPosition.time)
                           ? nowSec - localPosition.time
                           : 0),
            (unsigned)hop_.index(), (unsigned)hop_.generation(),
            (unsigned)fastRadio.sendFailed());
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

    // A car that has heard nothing for a while is either alone or on the wrong
    // channel, and it cannot tell which. Rather than sit deaf on a channel the
    // ride may have left, it goes back to the one the key chose - home - where
    // every car on this ride can be found. See the note below.
    if (quiet >= LOST_MS) {
        // Fixed channel: stay home, never sweep.
        //
        // The sweep was meant to reunite a car that fell off the back onto a
        // channel the ride had hopped to. In practice it did the opposite. Two
        // lost cars both derive the same home from the ride key, so they should
        // just meet there - but they went lost within a few seconds of each
        // other, both started sweeping 1/6/11 at the same rate, and stayed
        // permanently out of phase, each landing on a channel as the other left.
        // On the bench two cars on one ride key sat on channels 1 and 11
        // reporting "nobody on it", having passed through each other's channels
        // for minutes. And every reconfigure or reconnect reset the lost timer,
        // so they never survived long enough to reach the wait-at-home phase
        // that would have saved them.
        //
        // So the ride does not hop, and a lost car does one thing: go home and
        // stay. Every car on the same key lands on the same channel and hears
        // every other one, with no phase to fall out of. This is what OpenHelmet
        // and every headset intercom does - one fixed channel - and it trades a
        // congested-channel escape hatch nobody has needed yet for a fast lane
        // that actually carries. See FAST_LANE_HOP.
        if (fastRadio.channel() != hop_.homeChannel()) {
            hop_.goHome();
            if (fastRadio.retuneTo(hop_.channel())) {
                LOG_INFO("touge: quiet for %ums, home on channel %u",
                         (unsigned)quiet, (unsigned)hop_.channel());
            }
        }
        return;
    }

    // The ride does not hop. See the fixed-channel note above: everyone stays
    // on the channel the key chose, which is the only way two cars reliably
    // find each other on 2.4 GHz without a scan to fall out of phase. The
    // interference-escape hop is kept in the code, and off, behind this flag.
    if (!FAST_LANE_HOP) return;

    // Only the car keeping time decides to move, so the ride does not have
    // several cars hopping it in different directions at once.
    if (!schedule_.weAreReference()) return;
    if ((uint32_t)(nowMs - lastHopCheckMs_) < HOP_WINDOW_MS) return;
    lastHopCheckMs_ = nowMs;

    // Expected against received over the window. Every car beacons at least
    // once per idle heartbeat, so a roster that is present but barely audible
    // is interference rather than an empty road.
    // Cars we should have heard from on this channel in this window, not
    // every seat filled in the last ten minutes. Those are different questions
    // and using the roster for both is what made a working channel look
    // broken after any large gathering.
    size_t cars = mesh_.countOn(fastRadio.channel(), HOP_WINDOW_MS, nowMs);
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
        if (fastRadio.send(f.wire, f.len)) stats_.fast.tx++;
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
        // Our own phone's fix comes through here too; only other cars count.
        if (!isFromUs(&mp)) stats_.lora.rx++;
        const Rider *r = mesh_.find(mp.from);
        if (r && r->via == HEARD_FAST && (uint32_t)(millis() - r->atMs) < FAST_PRECEDENCE_MS) {
            // Not STOP any more, and this is the difference between a car
            // being a second stale and a car not existing.
            //
            // STOP breaks callModules, and callModules is how RoutingModule
            // runs - which is the only thing that calls sniffReceived, which
            // is the only thing that rebroadcasts. So refusing a LoRa position
            // from a car we can hear on 2.4 GHz also refused to *relay* it. A
            // middle car that could hear the head on 2.4 GHz would not pass
            // the head's LoRa position back down the line, and a tail car
            // beyond both the head's LoRa range and FAST_HOPS heard nothing
            // from the head on either radio. The car that most needed the
            // relay was the one guaranteed not to get it, and the cost of the
            // bug rose with the length of the convoy.
            //
            // The precedence itself is still worth having: NodeDB is
            // last-write-wins with no staleness guard, so a two-second-old
            // LoRa fix does replace a quarter-second-old 2.4 GHz one. So the
            // packet goes through and the row is put back on the next tick,
            // five milliseconds later, from the fast position we already hold.
            // Briefly stale beats permanently absent.
            if (restoreCount_ < RESTORE_SLOTS) restore_[restoreCount_++] = mp.from;
            return ProcessMessage::CONTINUE;
        }
        return ProcessMessage::CONTINUE;
    }

    // Only packets the phone aimed at this radio in particular. Router::sendLocal
    // delivers those to modules without transmitting, which is what keeps
    // push-to-talk audio off the LoRa side entirely.
    if (mp.which_payload_variant != meshtastic_MeshPacket_decoded_tag) return ProcessMessage::CONTINUE;
    // isFromUs, not a comparison against our node number.
    //
    // MeshService::handleToRadio sets from to zero on everything the phone
    // sends - clients are not allowed to assign node numbers - and it is only
    // filled in by Router::send, on the way to the LoRa chip. A packet
    // addressed to this radio never goes that way: sendLocal sees isToUs and
    // hands it straight to deliverLocal. So from was still zero by the time it
    // reached here, the test below failed, and push-to-talk audio from the
    // phone was never transmitted at all. isFromUs is the idiomatic check and
    // treats zero as ourselves, which is exactly what it is for.
    if (!isFromUs(&mp)) return ProcessMessage::CONTINUE;
    if (!isToUs(&mp)) return ProcessMessage::CONTINUE;

    // The phone saying it reads batches. Local only: never transmitted.
    PhoneHello hello;
    if (decodeHello(mp.decoded.payload.bytes, mp.decoded.payload.size, hello)) {
        if (!helloSeen_ || hello.flags != hello_.flags || hello.mtu != hello_.mtu) {
            LOG_INFO("touge: phone hello, batches=%u preload=%u mtu=%u", (unsigned)((hello.flags & HELLO_BATCHES) != 0),
                     (unsigned)((hello.flags & HELLO_PRELOAD) != 0), (unsigned)hello.mtu);
        }
        hello_ = hello;
        helloSeen_ = true;
        return ProcessMessage::STOP;
    }

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

// ---- Positions to the phone in batches (SCALE-PLAN step 3) ------------------

PhoneRecord TougeFastModule::phoneRecordFor(const Frame &f, const Position &p, int8_t rssi) const
{
    PhoneRecord r;
    r.node = f.src;
    r.lat = p.lat;
    r.lon = p.lon;
    r.frameId = f.id;
    r.headingCdeg = (uint16_t)((p.headingDeg % 360) * 100);
    r.speedDkmh = (uint16_t)(p.speedMph * 16.09344f + 0.5f);
    r.rssi = rssi;
    r.external = p.phoneAttached;
    r.lane = LANE_FAST;
    r.hopsAway = (p.extra || f.hops > FAST_HOPS) ? 0 : (uint8_t)(FAST_HOPS - f.hops);
    r.heardMs = millis();
    return r;
}

void TougeFastModule::offerToPhone(const PhoneRecord &r)
{
    stats_.fast.queued++;
    switch (phoneStore_.offer(r, millis())) {
    case PhoneStore::REPLACED:
        stats_.replaced++;
        break;
    case PhoneStore::STALE:
        stats_.dropStale++;
        break;
    case PhoneStore::EVICTED:
        stats_.dropStoreFull++;
        break;
    default:
        break;
    }
}

void TougeFastModule::flushPhoneBatch(uint32_t nowMs)
{
    if (!batchingToPhone()) return;
    stats_.dropLost += batchesInFlight_.expire(nowMs, BATCH_LOST_MS);
    // Backpressure: with two batches unread, positions wait here, where a newer
    // one replaces an older one, rather than in a queue in front of the phone.
    if (batchesInFlight_.full()) return;

    const size_t budget = batchBudgetForMtu(hello_.mtu);
    if (!phoneStore_.due(nowMs, budget, BATCH_FLUSH_MS)) return;

    uint8_t payload[BATCH_MAX_PAYLOAD];
    size_t taken = 0;
    const uint16_t seq = batchSeq_;
    const size_t len = phoneStore_.takeBatch(payload, budget, seq, nowMs, taken);
    if (len == 0) return;
    batchSeq_++;
    if (!handPhoneBatch(payload, len, seq)) {
        stats_.dropAlloc += (uint32_t)taken;
        return;
    }
    stats_.batches++;
    batchesInFlight_.add(seq, (uint8_t)taken, nowMs);
}

bool TougeFastModule::handPhoneBatch(const uint8_t *payload, size_t len, uint16_t seq)
{
    // Static: a MeshPacket and a FromRadio are over a kilobyte together, too much
    // for the main task's stack on every flush.
    static meshtastic_MeshPacket packet;
    memset(&packet, 0, sizeof(packet));
    packet.from = nodeId_;
    packet.to = NODENUM_BROADCAST;
    packet.id = generatePacketId();
    packet.channel = channels.getPrimaryIndex();
    packet.hop_limit = 0;
    packet.hop_start = 0;
    packet.rx_time = getValidTime(RTCQualityFromNet);
    packet.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    packet.decoded.portnum = meshtastic_PortNum_PRIVATE_APP;
    memcpy(packet.decoded.payload.bytes, payload, len);
    packet.decoded.payload.size = (pb_size_t)len;

#if TOUGE_HAS_NIMBLE
    // Straight into NimBLE's read queue, so the phone's read is answered without
    // waiting for the main task (SCALE-PLAN step 3b). Refused when the link is
    // busy or not streaming, and then the batch takes the ordinary queue.
    if ((hello_.flags & HELLO_PRELOAD) != 0 && preloadedSeq_ < 0) {
        static meshtastic_FromRadio fromRadio;
        static uint8_t fromRadioBytes[meshtastic_FromRadio_size];
        memset(&fromRadio, 0, sizeof(fromRadio));
        fromRadio.which_payload_variant = meshtastic_FromRadio_packet_tag;
        fromRadio.packet = packet;
        const size_t n = pb_encode_to_bytes(fromRadioBytes, sizeof(fromRadioBytes), &meshtastic_FromRadio_msg, &fromRadio);
        if (n > 0 && nimbleOfferToPhone(fromRadioBytes, n)) {
            preloadedSeq_ = seq;
            return true;
        }
    }
#endif

    meshtastic_MeshPacket *pp = router->allocForSending();
    if (!pp) return false;
    *pp = packet;
    service->sendToPhone(pp);
    return true;
}

void TougeFastModule::trackPhoneLink(uint32_t nowMs)
{
    (void)nowMs;
    const int depth = service->toPhoneQueueUsed();
    stats_.queueDepth = (uint16_t)depth;
    if (depth > stats_.queueDepthMax) stats_.queueDepthMax = (uint16_t)depth;

    // A hello lasts one connection. The next phone may be an app that only
    // reads one packet per position, so it has to say hello for itself.
    if (service->api_state == MeshService::STATE_DISCONNECTED) {
        if (helloSeen_) LOG_INFO("touge: phone gone, positions go one packet each until the next hello");
        helloSeen_ = false;
        stats_.dropDisconnect += (uint32_t)phoneStore_.pending() + batchesInFlight_.pendingRecords();
        phoneStore_.clear();
        batchesInFlight_.clear();
        preloadedSeq_ = -1;
        return;
    }

#if TOUGE_HAS_NIMBLE
    NimbleTougeCounters c;
    nimbleTougeCounters(c);
    // Only one batch is ever pre-encoded at a time, so a read is that one.
    if (c.offeredRead != preloadReadSeen_) {
        preloadReadSeen_ = c.offeredRead;
        if (preloadedSeq_ >= 0) notePhoneRead((uint16_t)preloadedSeq_);
        preloadedSeq_ = -1;
    }
    // Lost with a link reset; its records are counted when it expires.
    if (c.offeredLost != preloadLostSeen_) {
        preloadLostSeen_ = c.offeredLost;
        preloadedSeq_ = -1;
    }
#endif
}

void TougeFastModule::notePhoneRead(uint16_t seq)
{
    const uint8_t records = batchesInFlight_.delivered(seq);
    if (records == 0) return;
    stats_.batchesRead++;
    stats_.fast.delivered += records;
}

void TougeFastModule::onPhoneDelivered(const meshtastic_MeshPacket &p)
{
    if (tougeFastModule) tougeFastModule->notePhoneDelivered(p);
}

void TougeFastModule::notePhoneDelivered(const meshtastic_MeshPacket &p)
{
    if (p.which_payload_variant != meshtastic_MeshPacket_decoded_tag) return;
    if (p.decoded.portnum == meshtastic_PortNum_POSITION_APP) {
        if (isFromUs(&p)) return;
        // The fast lane's one-per-packet positions carry no hop budget; LoRa ones do.
        if (p.hop_start == 0 && p.hop_limit == 0) {
            stats_.fast.delivered++;
        } else {
            stats_.lora.delivered++;
        }
        return;
    }
    if (p.decoded.portnum != meshtastic_PortNum_PRIVATE_APP || p.from != nodeId_) return;
    BatchHeader header;
    if (decodeBatchHeader(p.decoded.payload.bytes, p.decoded.payload.size, header)) notePhoneRead(header.seq);
}

void TougeFastModule::queueJsonToPhone(const char *json, size_t len)
{
    if (len == 0 || len > meshtastic_Constants_DATA_PAYLOAD_LEN) return;
    meshtastic_MeshPacket *sp = router->allocForSending();
    if (!sp) return;
    sp->from = nodeId_;
    sp->to = NODENUM_BROADCAST;
    sp->channel = channels.getPrimaryIndex();
    sp->hop_limit = 0;
    sp->hop_start = 0;
    sp->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    sp->decoded.portnum = meshtastic_PortNum_PRIVATE_APP;
    memcpy(sp->decoded.payload.bytes, json, len);
    sp->decoded.payload.size = (pb_size_t)len;
    service->sendToPhone(sp);
}

void TougeFastModule::reportLinkStats(uint32_t nowMs, uint32_t windowMs)
{
    stats_.fast.suppressed = mesh_.suppressed();
    stats_.fastTxFail = fastRadio.sendFailed();
    stats_.storePending = (uint16_t)phoneStore_.pending();
    const uint32_t storeWait = phoneStore_.oldestWaitMs(nowMs);
    const uint32_t flightWait = batchesInFlight_.oldestAgeMs(nowMs);
    stats_.oldestQueuedMs = storeWait > flightWait ? storeWait : flightWait;
    stats_.coreReplaced = service->tougePositionsReplaced;
    stats_.coreEvicted = service->tougePositionsEvicted;
    stats_.coreDropped = service->tougePhoneDropped;
#if TOUGE_HAS_NIMBLE
    NimbleTougeCounters c;
    nimbleTougeCounters(c);
    stats_.writeDropped = c.writeDropped;
    stats_.writeDuplicate = c.writeDuplicate;
    stats_.preloadOffered = c.offered;
    stats_.preloadRead = c.offeredRead;
    stats_.preloadRefused = c.offerRefused;
#endif
    stats_.minFreeHeap = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);

    char line[320];
    if (formatBaseline(stats_, statsAtLastReport_, windowMs, line, sizeof(line)) > 0) {
        LOG_INFO("touge: %s batching=%u", line, (unsigned)batchingToPhone());
    }

    // Nobody to read them, and 32 unread reports would fill the phone queue.
    if (service->api_state != MeshService::STATE_DISCONNECTED) {
        char js[BATCH_MAX_PAYLOAD];
        queueJsonToPhone(js, formatLaneStats(stats_, js, sizeof(js)));
        queueJsonToPhone(js, formatQueueStats(stats_, js, sizeof(js)));
    }
    statsAtLastReport_ = stats_;
    // The high-water mark is per report, so a burst shows in the report after it.
    stats_.queueDepthMax = stats_.queueDepth;
}

#endif
