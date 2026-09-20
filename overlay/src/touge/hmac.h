#pragma once
//
// HMAC-SHA256, over the SHA-256 we already carry.
//
// Encryption without authentication is the hole this fills. AES-CTR keeps a
// position private but does nothing to stop anyone on the channel flipping
// bits in it, and CTR flips them *predictably*: change one bit of ciphertext
// and exactly that bit of the plaintext changes. Without a tag a stranger can
// move a car on your map by a known amount without knowing the key.
//
// mbedtls has AES-CCM and it would do the job in one call, but it only exists
// on the device, and then the one thing that must never be subtly wrong would
// be the one thing with no test that runs on a laptop. This is built on the
// portable SHA-256 instead, so the tag is checked on the host against RFC 4231
// and the device runs the same code.

#include <stdint.h>
#include <stddef.h>
#include "sha256.h"

namespace touge {

// Eight bytes of the digest. A forgery has a one in 2^64 chance, which against
// a packet whose whole body is twelve bytes is the right trade; a full 32-byte
// tag would nearly triple the size of a position.
static const size_t TAG_LEN = 8;

void hmacSha256(const uint8_t* key, size_t keyLen, const uint8_t* data, size_t dataLen,
                uint8_t out[SHA256_LEN]);

// The tag for one frame. Covers the sender, the packet id, the type, the
// channel byte and the ciphertext. Deliberately not the hop count, which every
// forwarding node decrements: covering it would make a forwarded frame fail
// its own tag at the next node along.
void frameTag(const uint8_t* key, size_t keyLen, uint32_t src, uint32_t id, uint8_t type,
              uint8_t chan, const uint8_t* cipher, size_t cipherLen, uint8_t out[TAG_LEN]);

// Constant time. A comparison that returns early tells an attacker how many
// bytes they guessed right, which turns 2^64 work into 8 x 2^8.
bool tagsMatch(const uint8_t* a, const uint8_t* b, size_t len);

} // namespace touge
