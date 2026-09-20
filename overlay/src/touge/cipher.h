#pragma once
//
// Encrypt, then authenticate.
//
// AES-256 in counter mode over the payload, then an HMAC-SHA256 tag over the
// ciphertext and the header fields that identify it. In that order: a tag over
// the ciphertext can be checked before anything is decrypted, so a forged
// frame costs one hash and is thrown away without the key ever touching it.
//
// CTR rather than a block mode because the payload is twelve bytes and padding
// it to a sixteen byte boundary would add a third to the smallest packet we
// send. CTR is also symmetric, so one function encrypts and decrypts and there
// is no chance of the two drifting apart.
//
// The whole safety of the encryption rests on the nonce never repeating under
// one key. It is built from the packet id and the sender, and the id is a
// 32-bit counter that survives reboots through NVS. Get that wrong and an
// eavesdropper gets the XOR of two positions for free.

#include <stdint.h>
#include <stddef.h>
#include "hmac.h"
#include "ride.h"

namespace touge {

// Encrypts `len` bytes in place and appends a tag. Returns the total written,
// or 0 if the tag would not fit in `cap`.
size_t seal(const uint8_t key[PSK_LEN], uint32_t src, uint32_t id, uint8_t type, uint8_t chan,
            uint8_t* buf, size_t len, size_t cap);

// Checks the tag and, only if it holds, decrypts in place. Returns the
// plaintext length, or 0 if the frame was forged, corrupted, or too short to
// carry a tag at all. On 0 the buffer is left encrypted: nothing that failed
// authentication is ever handed back as plaintext.
size_t unseal(const uint8_t key[PSK_LEN], uint32_t src, uint32_t id, uint8_t type, uint8_t chan,
              uint8_t* buf, size_t len);

// The raw keystream operation, exposed because it is symmetric and the tests
// want to build a sealed frame without a radio.
void cipherApply(const uint8_t key[PSK_LEN], uint32_t src, uint32_t id, uint8_t type, uint8_t* buf,
                 size_t len);

} // namespace touge
