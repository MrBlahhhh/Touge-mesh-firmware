#pragma once
//
// AES-256 in counter mode over the frame payload.
//
// CTR rather than a block mode because the payload is 12 bytes and padding it
// out to a 16-byte boundary would add a third to the smallest packet we send.
// CTR also means one function encrypts and decrypts, so there is no chance of
// the two drifting apart.
//
// The whole safety of this rests on the nonce never repeating under one key.
// It is built from the packet id and the sender, and the id is a 32-bit
// counter that survives reboots through NVS. Get that wrong and an
// eavesdropper gets the XOR of two positions for free.

#include <stdint.h>
#include <stddef.h>
#include "ride.h"

namespace touge {

// In place, and symmetric: the same call undoes itself.
void cipherApply(const uint8_t psk[PSK_LEN], uint32_t src, uint32_t id, uint8_t type, uint8_t* buf,
                 size_t len);

} // namespace touge
