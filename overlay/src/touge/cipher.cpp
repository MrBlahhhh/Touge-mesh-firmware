#include "cipher.h"
#include "hmac.h"
#include <string.h>
#include "mbedtls/aes.h"

namespace touge {

size_t seal(const uint8_t key[PSK_LEN], uint32_t src, uint32_t id, uint8_t type, uint8_t chan,
            uint8_t* buf, size_t len, size_t cap) {
  if (buf == nullptr || cap < len + TAG_LEN) return 0;
  cipherApply(key, src, id, type, buf, len);
  // Over the ciphertext, not the plaintext, so a receiver can reject a forgery
  // without ever decrypting it.
  frameTag(key, PSK_LEN, src, id, type, chan, buf, len, buf + len);
  return len + TAG_LEN;
}

size_t unseal(const uint8_t key[PSK_LEN], uint32_t src, uint32_t id, uint8_t type, uint8_t chan,
              uint8_t* buf, size_t len) {
  if (buf == nullptr || len < TAG_LEN) return 0;
  size_t body = len - TAG_LEN;

  uint8_t want[TAG_LEN];
  frameTag(key, PSK_LEN, src, id, type, chan, buf, body, want);
  // Checked before anything is decrypted, and the buffer is left as it was on
  // failure. Handing back half-decrypted bytes that failed authentication is
  // how a caller ends up parsing an attacker's choice of garbage.
  if (!tagsMatch(want, buf + body, TAG_LEN)) return 0;

  cipherApply(key, src, id, type, buf, body);
  return body;
}

void cipherApply(const uint8_t psk[PSK_LEN], uint32_t src, uint32_t id, uint8_t type, uint8_t* buf,
                 size_t len) {
  if (buf == nullptr || len == 0) return;

  // Sixteen bytes of counter block. The id goes first because it is the part
  // that actually varies packet to packet; the sender follows so two nodes
  // that happen to pick the same id still get different keystream. The type
  // is in there so a position and a text sent with one id cannot collide.
  uint8_t nonce[16];
  memset(nonce, 0, sizeof(nonce));
  nonce[0] = (uint8_t)(id >> 24);
  nonce[1] = (uint8_t)(id >> 16);
  nonce[2] = (uint8_t)(id >> 8);
  nonce[3] = (uint8_t)id;
  nonce[4] = (uint8_t)(src >> 24);
  nonce[5] = (uint8_t)(src >> 16);
  nonce[6] = (uint8_t)(src >> 8);
  nonce[7] = (uint8_t)src;
  nonce[8] = type;

  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  // Encryption key schedule even when decrypting: CTR only ever runs the
  // cipher forwards, over the counter, and never over the data.
  mbedtls_aes_setkey_enc(&aes, psk, 256);

  size_t offset = 0;
  uint8_t stream[16];
  memset(stream, 0, sizeof(stream));
  mbedtls_aes_crypt_ctr(&aes, len, &offset, nonce, stream, buf, buf);

  mbedtls_aes_free(&aes);
}

} // namespace touge
