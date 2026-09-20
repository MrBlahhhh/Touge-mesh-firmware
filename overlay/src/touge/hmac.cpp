#include "hmac.h"
#include <string.h>

namespace touge {
namespace {

const size_t BLOCK = 64; // SHA-256 processes 64-byte blocks

} // namespace

void hmacSha256(const uint8_t* key, size_t keyLen, const uint8_t* data, size_t dataLen,
                uint8_t out[SHA256_LEN]) {
  uint8_t k[BLOCK];
  memset(k, 0, sizeof(k));

  // A key longer than the block is replaced by its own digest. A shorter one
  // is zero padded, which is why a 32-byte channel key lands here unchanged.
  if (keyLen > BLOCK) {
    sha256(key, keyLen, k);
  } else if (key != nullptr && keyLen > 0) {
    memcpy(k, key, keyLen);
  }

  uint8_t inner[BLOCK];
  uint8_t outer[BLOCK];
  for (size_t i = 0; i < BLOCK; i++) {
    inner[i] = (uint8_t)(k[i] ^ 0x36);
    outer[i] = (uint8_t)(k[i] ^ 0x5c);
  }

  // The message is hashed with the inner pad prepended, then that digest is
  // hashed again with the outer pad. Our sha256 takes one contiguous buffer,
  // so the inner pass is assembled into a scratch buffer.
  //
  // Bounded by the largest frame we will ever tag rather than by malloc: this
  // runs on a radio, and a heap allocation per packet on a 250 ms cycle is a
  // fragmentation problem waiting to happen.
  uint8_t scratch[BLOCK + 256];
  if (dataLen > sizeof(scratch) - BLOCK) {
    // Refuse rather than truncate. A silently short-tagged frame would verify
    // against only part of itself, which is worse than not sending it.
    memset(out, 0, SHA256_LEN);
    return;
  }

  memcpy(scratch, inner, BLOCK);
  if (dataLen > 0) memcpy(scratch + BLOCK, data, dataLen);
  uint8_t innerDigest[SHA256_LEN];
  sha256(scratch, BLOCK + dataLen, innerDigest);

  uint8_t second[BLOCK + SHA256_LEN];
  memcpy(second, outer, BLOCK);
  memcpy(second + BLOCK, innerDigest, SHA256_LEN);
  sha256(second, BLOCK + SHA256_LEN, out);
}

void frameTag(const uint8_t* key, size_t keyLen, uint32_t src, uint32_t id, uint8_t type,
              uint8_t chan, const uint8_t* cipher, size_t cipherLen, uint8_t out[TAG_LEN]) {
  uint8_t buf[10 + 256];
  if (cipherLen > sizeof(buf) - 10) {
    memset(out, 0, TAG_LEN);
    return;
  }

  buf[0] = (uint8_t)(src >> 24);
  buf[1] = (uint8_t)(src >> 16);
  buf[2] = (uint8_t)(src >> 8);
  buf[3] = (uint8_t)src;
  buf[4] = (uint8_t)(id >> 24);
  buf[5] = (uint8_t)(id >> 16);
  buf[6] = (uint8_t)(id >> 8);
  buf[7] = (uint8_t)id;
  buf[8] = type;
  buf[9] = chan;
  if (cipherLen > 0) memcpy(buf + 10, cipher, cipherLen);

  uint8_t full[SHA256_LEN];
  hmacSha256(key, keyLen, buf, 10 + cipherLen, full);
  memcpy(out, full, TAG_LEN);
}

bool tagsMatch(const uint8_t* a, const uint8_t* b, size_t len) {
  if (a == nullptr || b == nullptr) return false;
  // Every byte, every time. Returning at the first mismatch leaks how far the
  // guess got, and a timing signal turns forging a tag from 2^64 work into
  // eight rounds of 2^8.
  uint8_t diff = 0;
  for (size_t i = 0; i < len; i++) diff = (uint8_t)(diff | (a[i] ^ b[i]));
  return diff == 0;
}

} // namespace touge
