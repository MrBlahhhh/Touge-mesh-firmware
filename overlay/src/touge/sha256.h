#pragma once
//
// SHA-256, carried rather than pulled from mbedtls.
//
// The board could use mbedtls and the host tests could use OpenSSL, and then
// the one thing that must agree byte for byte between a phone, a board and a
// test would be the one thing tested on three different implementations. One
// copy, compiled everywhere, is the cheaper answer.

#include <stdint.h>
#include <stddef.h>

namespace touge {

static const size_t SHA256_LEN = 32;

void sha256(const uint8_t* data, size_t len, uint8_t out[SHA256_LEN]);

} // namespace touge
