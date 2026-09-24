#include "gnssfix.h"

#include <stdio.h>

namespace touge {

size_t formatGnssFix(const GnssFix &fix, char *out, size_t cap)
{
    if (out == nullptr || cap == 0) return 0;
    int n = snprintf(out, cap,
                     "{\"gf\":{\"la\":%ld,\"lo\":%ld,\"al\":%ld,\"kh\":%lu,\"tr\":%lu,\"sa\":%lu,\"hd\":%lu,\"t\":%lu}}",
                     (long)fix.latE7, (long)fix.lonE7, (long)fix.altitudeM, (unsigned long)fix.speedKmh,
                     (unsigned long)(fix.trackE5 / 1000), (unsigned long)fix.sats, (unsigned long)fix.hdopE2,
                     (unsigned long)fix.fixTimeSec);
    if (n <= 0 || (size_t)n >= cap) return 0;
    return (size_t)n;
}

bool GnssForward::due(const GnssFix &fix, uint32_t nowMs) const
{
    // GPS.cpp resets its position to all zeros when it loses lock.
    if (fix.latE7 == 0 && fix.lonE7 == 0) return false;
    if (fix.fixTimeSec == 0) return false;
    if (!sentOnce_) return true;
    if (fix.fixTimeSec == lastFixTimeSec_) return false;
    return (uint32_t)(nowMs - lastSentMs_) >= MIN_GAP_MS;
}

void GnssForward::sent(const GnssFix &fix, uint32_t nowMs)
{
    lastFixTimeSec_ = fix.fixTimeSec;
    lastSentMs_ = nowMs;
    sentOnce_ = true;
}

} // namespace touge
