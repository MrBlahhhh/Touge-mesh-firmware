#pragma once
//
// Whether a 2.4 GHz forward is worth sending, judged from the slot maps every
// car already broadcasts (docs/plans/2026-09-27-voice-coding-plan.md, "The
// relaying we already have").
//
// The lane floods: every car that hears a frame with hops left forwards it,
// weakest signal first, and drops its copy once SUPPRESS_AFTER copies have gone
// by (mesh.h). That stays as the backup. This adds the build 43 LoRa rule
// (relaypref.h) on the fast lane: skip a forward only when fresh evidence shows
// every other car we know already hears the origin steadily, so our copy would
// reach nobody who lacks it. Positions and voice alike are judged;
// fastRelayDropsForward says which skips are taken, by frame type and how many
// cars are on the ride.
//
// ## The evidence
//
// A beacon's slot map holds the tag of the car its sender heard in each slot in
// the last HEARD_WINDOW_MS. So a map from car N says N heard origin T when
// map[T's slot] == slotTag(T). That is the only reading used: the tag is eight
// bits, so T's tag anywhere else in the map is as likely another car, and T's
// own map never shows T.
//
// One map is weak evidence: at 40 % loss it shows T about 72 % of the time over
// a working link. So for each pair (N, T) we keep whether each of N's last
// FAST_RELAY_MAPS maps showed T, newest in bit 0, like slotSeen_ in schedule.h.
// A record moves on once per map counted, so a map lost on its way to us counts
// neither way. Maps come from lease and unleased beacons, direct or relayed:
// the map says what N heard, whichever way it reached us. At most one map a
// second counts (FAST_RELAY_MAP_SPACING_MS), and extras never do: maps whose
// 1.5 s windows overlap more than a second's worth say the same thing twice.
//
// ## Steady
//
// N hears T steadily when N's latest counted map is under FAST_RELAY_FRESH_MS
// old, at least k of its last 8 maps showed T, and a map under
// FAST_RELAY_FRESH_MS old shows T: the latest when a map covers two of T's
// beacons, one of the latest two when it covers one. That is by time, not by
// count, so maps lost on their way to us cannot stretch "the one before" back
// to an old hearing. A link that dies stops counting within HEARD_WINDOW_MS
// and FAST_RELAY_FRESH_MS, 4.5 s, whatever reaches us. Holding the map before
// the latest to 2 s instead took a working link from 47 % to 44 % at bench
// loss and moved neither that bound nor the false skips much.
//
// k depends on how many of T's beacons one of N's maps covers. T beacons once a
// second and a map looks back 1.5 s, so a map covers two of T's beacons when
// T's slot opens within half a second before N's, and one otherwise
// (mapCoversTwoBeacons). A two-beacon map misses T only when both were lost, so
// a weak link shows in it far more often. Modelled with a single k of 6, a link
// losing 75 % read steady 11 % of the time in the two-beacon phase, and a 40 %
// link only 29 % in the one-beacon phase. So k is 5 for one beacon and 7 for
// two.
//
// Measured in test_fastrelay (maps from a real Schedule, N's maps reaching us
// at 40 % loss, judged as T's position arrives and at a random moment each
// second, 20 000 s a case):
//
//   loss T to N   judged steady, one beacon a map (k 5)   two (k 7)
//   20 %          82 %                                     85 %
//   40 %          47 %                                     55 %
//   50 %          28 %                                     34 %
//   75 %          2.0 %                                    2.6 %
//   100 %         0                                        0
//
// So at bench loss a working link counts about half the time, and a car that
// hears one beacon in four is taken for one that hears T about one time in 40.
// A link at 40 % loss that dies reads not steady 2.1 s later on average, 3 s
// (one beacon a map) or 4 s (two) at the 95th percentile.
//
// ## The verdict
//
// A forward is skipped only when every other car on the ride hears the origin
// steadily: not us, not the origin, leased or not, direct or relayed. On the
// ride is on the roster and heard within LEASE_MS, or sending us maps within
// LEASE_MS while on no roster (Mesh::note turned it away, and it stays off
// until its next position finds a seat). Unleased cars count too: they still
// listen, and their beacons carry a map like any other. Otherwise it goes, for
// the strongest reason, NEEDED over STALE over NO_EVIDENCE, as in judgeRelay.
// It also goes when the origin holds no lease (it is in nobody's map), is not
// on the ride, or shares its slot with another car we know; when the roster is
// full and may be hiding a car; and for a lease after a car still on the ride
// lost its seat here, since then more cars are about than either list holds.
// Nobody else on the ride is nobody to forward for, so a two-car ride skips
// every forward. Rider::hopsAway is not read, so review B7's flicker between
// direct and relayed does not move the verdict.
//
// A relayed copy has been on the air from its relayer, so every car that hears
// the relayer has it too: it is skipped when every car hears the origin or the
// relayer steadily. The relayer names itself by its slotTag in header byte 12
// (Frame::relayer, from build 51), and is placed only as the one car on the
// ride with that tag, so a hidden car sharing its slot cannot pass for it. A
// relayed copy whose relayer cannot be placed is judged on the origin alone,
// which is as safe: if every other car, the relayer included, hears the origin
// steadily, our copy reaches nobody who lacks it.
//
// ## RAM
//
// 1004 bytes on every board: a byte of record for each pair among 27 cars, and
// each car's id, lease, latest counted map and the gap to the one before. 27,
// not MAX_RIDERS (28): a roster with every seat taken may be hiding a car and
// never skips, and a 28th car on the ride stops skips for a lease. A history
// per slot instead of per car, 28 x 32 x 16 bits, would be 1.8 KB, too much
// for a Heltec V3 (ram.h). It lives in TougeFastModule, which is allocated from
// the heap at boot, so from build 51 the V3's "touge: heap" line reads about
// 1 KB lower.
//
// Platform-free: time comes in as an argument.

#include <stddef.h>
#include <stdint.h>
#include "mesh.h"
#include "ram.h"
#include "schedule.h"

namespace touge {

// The fewest cars on a ride, the origin included and us not, at which a SKIP
// drops a position forward. A link judged steady still misses about one beacon
// in five, and on a small ride our forward was what covered it.
// 2B sim, p = 0.2, 1-hop positions against 50 with no floor: 3 cars -8.5 points, 4 -8.1, 6 -3.3, 8 -1.2.
// A floor of 8 still cost 9-12 car rides 1.0-0.5 points; at 12 they are level and 13-14 lose 0.4 and 0.3.
static const size_t FAST_RELAY_MIN_CARS = 12;

// Whether a SKIP drops voice forwards from FAST_RELAY_MIN_CARS up as well. A
// skipped voice frame is a hole, where a skipped position is covered by the
// next one. Voice is judged and counted either way ({"fe"} in phonebatch.h).
// 2B sim: above the floor it changes nothing at p = 0.2 and 0.4 and pays only in a clean 25-car park.
static const bool FAST_RELAY_SKIPS_VOICE = false;

// Maps remembered per pair: one byte of record, newest in bit 0.
static const uint8_t FAST_RELAY_MAPS = 8;

// Of those, how many must show the origin, by how many of its beacons each map
// covers. See "Steady" above for the numbers.
static const uint8_t FAST_RELAY_SHOWN_ONE = 5;
static const uint8_t FAST_RELAY_SHOWN_TWO = 7;

// How old a car's latest map may be. Three of its beacons, as MAP_TRUST_MS: at
// 40 % loss the latest of three arrives 94 % of the time, where HEARD_WINDOW_MS
// would leave a car's map fresh only 72 % of the time, and a skip needs every
// car on the ride fresh at once. A car whose maps stop reaching us has most
// likely driven out of our range, which is out of reach of our forward too.
static const uint32_t FAST_RELAY_FRESH_MS = 3 * SCHEDULE_MS;

// A map this soon after the last one counted moves nothing. An unleased car
// beacons again every GATE_METRES and draws a new turn each second, so its
// maps can come several a second. A slot short of a second, so a lease beacon
// sent late in its slot and the next sent early both count. A pass stalled
// between them can lose one, which counts neither way.
static const uint32_t FAST_RELAY_MAP_SPACING_MS = SCHEDULE_MS - SLOT_MS;

// Maps further apart than this are not one record: the car drove off and came
// back, or its maps stopped reaching us, and what it heard then says nothing
// now. At 40 % loss a gap this long comes once in 600 maps.
static const uint32_t FAST_RELAY_GAP_MS = FAST_RELAY_MAPS * SCHEDULE_MS;

// How close to a boundary the phase between two slots may be before a map is
// taken to cover two beacons, the stricter reading. Either way a beacon leaves
// anywhere in the first 17 ms of its slot (SLOT_MS less SLOT_GUARD_MS) and two
// cars' clocks differ by 3 ms a sync hop: under 60 ms in all.
//
// Near half a second the question is whether the origin's older beacon is still
// in the map's window, and the listener stamps it late: after the airtime, and
// when its pass drains the frame. So that side has 48 ms more, the airtime and
// a pass gap well past the usual 5 ms, though not a stall like the tablet V4's
// 120 ms. Stamped with the receive time (build 51), the pass drops out. Near a
// whole second the question is whether the origin's next beacon lands before
// the map, and a late stamp there only helps.
static const uint32_t FAST_RELAY_OLDER_MARGIN_MS = 4 * SLOT_MS;
static const uint32_t FAST_RELAY_NEXT_MARGIN_MS = 3 * SLOT_MS;

// Cars with a record. See "RAM" above.
static const size_t FAST_RELAY_CARS = MAX_RIDERS - 1;

static_assert(FAST_RELAY_MAPS == 8, "the record is one byte per pair");
static_assert(FAST_RELAY_SHOWN_ONE <= FAST_RELAY_SHOWN_TWO && FAST_RELAY_SHOWN_TWO <= FAST_RELAY_MAPS,
              "a map covering two beacons is the weaker evidence, and k cannot pass m");
static_assert(HEARD_WINDOW_MS > SCHEDULE_MS && HEARD_WINDOW_MS < 2 * SCHEDULE_MS,
              "a map covers one or two of a car's beacons, never three");
static_assert(FAST_RELAY_FRESH_MS < LEASE_MS, "a car can go stale while it is still on the ride");
static_assert(FAST_RELAY_GAP_MS > FAST_RELAY_FRESH_MS, "a record survives a stale spell shorter than the gap");
static_assert(FAST_RELAY_MAP_SPACING_MS < SCHEDULE_MS, "a lease beacon a second always counts");

/**
 * Whether each map from a car on `reporterSlot` covers two beacons from the car
 * on `originSlot` rather than one. True within FAST_RELAY_OLDER_MARGIN_MS of
 * half a second, within FAST_RELAY_NEXT_MARGIN_MS of a whole one, and when
 * either holds no lease: an unleased car beacons in a shared window drawn each
 * second, so its phase is anyone's.
 */
bool mapCoversTwoBeacons(uint8_t reporterSlot, uint8_t originSlot);

// In rising order of how strongly the forward goes; judge relies on it.
enum class FastRelayVerdict : uint8_t {
  SKIP,         // every other car on the ride hears the origin (or the relayer) steadily
  NO_EVIDENCE,  // the origin holds no lease, is not on the ride or shares its slot; a car may be missing; or a car has sent us no map
  STALE,        // a car's latest map is older than FAST_RELAY_FRESH_MS
  NEEDED,       // a car's fresh maps do not show the origin (or the relayer) steadily, or not for long enough yet
};

// Cars on the ride as judge counts them: on the roster, heard within LEASE_MS,
// and not us. The origin of a frame judged SKIP is always one of them.
size_t countCarsOnRide(uint32_t selfId, const Rider* riders, size_t maxRiders, uint32_t nowMs);

// Whether `verdict` drops our forward of a `frameType` frame, with
// `carsOnRide` from countCarsOnRide at the verdict's time. With the origin the
// only other car, nobody could want our copy (plan 1C.2's two-car rule), so
// that skip is taken for voice too.
inline bool fastRelayDropsForward(FastRelayVerdict verdict, uint8_t frameType, size_t carsOnRide) {
  if (verdict != FastRelayVerdict::SKIP) return false;
  if (carsOnRide == 1) return true;
  if (carsOnRide < FAST_RELAY_MIN_CARS) return false;
  return frameType != FRAME_VOICE || FAST_RELAY_SKIPS_VOICE;
}

class FastRelay {
 public:
  void reset();

  /**
   * A beacon from `senderId`, noted for the first time: direct or relayed, and
   * once per beacon, so after Mesh::firstSight as the module calls
   * Schedule::heardBeacon. Extras are ignored here, and so is a map under
   * FAST_RELAY_MAP_SPACING_MS after the last one counted from that car. Moves
   * on the record of which cars its map shows, and starts it again when the car
   * has changed lease or its last map was FAST_RELAY_GAP_MS ago.
   */
  void heardMap(uint32_t senderId, const Position& p, uint32_t nowMs);

  /**
   * Whether our forward of a frame from `origin` is worth sending. `relayed`:
   * the copy we heard came from a relay (hopsTravelled above 0, against what
   * its frame type starts with); `relayerTag` is Frame::relayer, the relay's
   * slotTag, 0 when the frame names none. `riders` is the roster,
   * Mesh::riders() with MAX_RIDERS, already holding this frame's sender if it
   * was a position. Ask before Mesh::defer, so a skipped frame never holds a
   * forward slot.
   *
   * `relayerUnplaced`, when given, is set for a relayed copy judged on the
   * origin alone because its relayer could not be placed: no tag, a tag no car
   * on the ride has or more than one has, or a car with no record here.
   */
  FastRelayVerdict judge(uint32_t selfId, uint32_t origin, bool relayed, uint8_t relayerTag,
                         const Rider* riders, size_t maxRiders, uint32_t nowMs,
                         bool* relayerUnplaced = nullptr) const;

 private:
  // The seat holding `id`, or FAST_RELAY_CARS.
  size_t seatOf(uint32_t id) const;
  // A seat for a car new here: a free one, else the one whose latest map is
  // oldest, most likely a car that has left.
  size_t takeSeat(uint32_t id, uint32_t nowMs);
  void clearAsReporter(size_t seat);
  void clearAsOrigin(size_t seat);
  // A seated car that is not us and has sent a map within LEASE_MS.
  bool onRideHere(size_t seat, uint32_t selfId, uint32_t nowMs) const;
  bool fresh(size_t seat, uint32_t nowMs) const;
  bool previousMapFresh(size_t seat, uint32_t nowMs) const;
  bool hearsSteadily(size_t reporter, size_t origin, uint32_t nowMs) const;
  size_t placeRelayer(uint8_t tag, uint32_t selfId, const Rider* riders, size_t maxRiders, uint32_t nowMs) const;
  // How car `k` stands on a forward from `origin`, relayed by `relayer` or not.
  FastRelayVerdict standing(size_t k, size_t origin, size_t relayer, uint32_t nowMs) const;

  uint32_t carId_[FAST_RELAY_CARS] = {};  // 0: empty
  // When the car's latest counted map came. A map not counted refreshes
  // nothing, freshness included: it came under a second after this one, and a
  // skip rests on a counted map being fresh either way.
  uint32_t mapAtMs_[FAST_RELAY_CARS] = {};
  // When a car still on the ride last lost its seat here.
  uint32_t crowdedAtMs_ = 0;
  // The lease the car's latest map came with, SLOT_NONE unleased. Its column is
  // read at this slot in every other car's map.
  uint8_t carSlot_[FAST_RELAY_CARS] = {};
  // How long before the latest counted map the one before it came, in steps of
  // GAP_STEP_MS rounded up; GAP_NONE when there is none or it is further back.
  uint8_t prevGap_[FAST_RELAY_CARS] = {};
  // shown_[n][t]: whether each of car n's last maps showed car t, newest in bit 0.
  uint8_t shown_[FAST_RELAY_CARS][FAST_RELAY_CARS] = {};
  bool crowded_ = false;
};

#if TOUGE_LEAN_RAM
static_assert(sizeof(FastRelay) <= 1024, "a Heltec V3 has no PSRAM; keep the relay record under 1 KB");
#endif

// Since boot: forwards skipped, and why the rest went. noRelayer counts relayed
// copies judged on the origin alone, on top of their verdict's own count.
struct FastRelaySkips {
  uint32_t skipped = 0;
  uint32_t noEvidence = 0;
  uint32_t stale = 0;
  uint32_t needed = 0;
  uint32_t noRelayer = 0;
  void note(FastRelayVerdict v, bool relayerUnplaced = false);
};

}  // namespace touge
