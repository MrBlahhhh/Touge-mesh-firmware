#pragma once
//
// The radio's own GNSS fix, forwarded to the phone.
//
// A tablet with no GPS of its own navigates on the V4's receiver. Stock
// Meshtastic only puts a GNSS fix into NodeDB; the phone sees it in the config
// dump at connect and in our own position broadcasts, which a Touge ride sets
// to every three hours. So this module sends it on the private port at the
// receiver's rate, as JSON the app already knows how to take apart:
//
//   {"gf":{"la":lat_e7,"lo":lon_e7,"al":m,"kh":kmh,"tr":centideg,"sa":n,"hd":hdop_e2,"t":utc_s}}
//
// Only ever the receiver's own solution (GPS::p), never NodeDB's local
// position, which the phone's LOC_EXTERNAL fix overwrites. Forwarding that
// would hand the phone its own fix back labelled as the radio's.

#include <stddef.h>
#include <stdint.h>

namespace touge {

struct GnssFix {
    int32_t latE7 = 0;
    int32_t lonE7 = 0;
    // Metres above the WGS84 ellipsoid (Position.altitude_hae).
    int32_t altitudeM = 0;
    // Meshtastic's GPS.cpp fills Position.ground_speed from TinyGPS kmph(),
    // whole km/h, whatever the proto comment says about units elsewhere.
    uint32_t speedKmh = 0;
    // Position.ground_track as GPS.cpp writes it: degrees * 1e5.
    uint32_t trackE5 = 0;
    uint32_t sats = 0;
    // HDOP * 100, as TinyGPS reports it.
    uint32_t hdopE2 = 0;
    // UTC seconds of the NMEA solution.
    uint32_t fixTimeSec = 0;
};

// Writes the JSON above. Returns its length, or 0 if it did not fit.
size_t formatGnssFix(const GnssFix &fix, char *out, size_t cap);

// Whether a fix is worth sending: a real position, newer than the last one
// sent, and no more than about once a second.
class GnssForward {
  public:
    // Leaves a little slack under a second so a 1 Hz receiver whose fixes land
    // a few ms early is not skipped every other time.
    static const uint32_t MIN_GAP_MS = 900;

    bool due(const GnssFix &fix, uint32_t nowMs) const;
    void sent(const GnssFix &fix, uint32_t nowMs);

  private:
    uint32_t lastFixTimeSec_ = 0;
    uint32_t lastSentMs_ = 0;
    bool sentOnce_ = false;
};

} // namespace touge
