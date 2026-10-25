// Portable SHA-256 / HMAC-SHA256 / HKDF-SHA256 (FIPS 180-4, RFC 2104,
// RFC 5869) used for the local link's key derivation and packet tags. Portable so the same code runs in the host
// unit tests and on the ESP32.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace vc {

class Sha256 {
 public:
  Sha256() { reset(); }
  void reset();
  void update(const uint8_t* data, size_t len);
  void finish(uint8_t out[32]);

 private:
  void block(const uint8_t* p);
  uint32_t h_[8];
  uint8_t buf_[64];
  size_t bufLen_;
  uint64_t totalLen_;
};

void hmacSha256(const uint8_t* key, size_t keyLen, const uint8_t* msg, size_t msgLen, uint8_t out[32]);

// HKDF-SHA256 (RFC 5869): extract with `salt`, expand with `info` into
// `outLen` bytes (<= 255 * 32). Returns false if outLen is too large.
bool hkdfSha256(const uint8_t* salt, size_t saltLen, const uint8_t* ikm, size_t ikmLen, const uint8_t* info,
                size_t infoLen, uint8_t* out, size_t outLen);

// Comparison whose running time does not depend on where the inputs differ.
bool constantTimeEqual(const uint8_t* a, const uint8_t* b, size_t n);

// Overwrites a buffer in a way the compiler may not optimise away.
void secureZero(void* p, size_t n);

}  // namespace vc
