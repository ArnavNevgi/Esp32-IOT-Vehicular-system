#include "vc_sha256.h"

#include <string.h>

namespace vc {

namespace {

const uint32_t K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

inline uint32_t rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

}  // namespace

void Sha256::reset() {
  const uint32_t init[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                            0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  memcpy(h_, init, sizeof(h_));
  bufLen_ = 0;
  totalLen_ = 0;
}

void Sha256::block(const uint8_t* p) {
  uint32_t w[64];
  for (int i = 0; i < 16; ++i)
    w[i] = ((uint32_t)p[4 * i] << 24) | ((uint32_t)p[4 * i + 1] << 16) | ((uint32_t)p[4 * i + 2] << 8) | p[4 * i + 3];
  for (int i = 16; i < 64; ++i) {
    uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
    uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  uint32_t a = h_[0], b = h_[1], c = h_[2], d = h_[3], e = h_[4], f = h_[5], g = h_[6], h = h_[7];
  for (int i = 0; i < 64; ++i) {
    uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
    uint32_t ch = (e & f) ^ (~e & g);
    uint32_t t1 = h + S1 + ch + K[i] + w[i];
    uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
    uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
    uint32_t t2 = S0 + maj;
    h = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }
  h_[0] += a;
  h_[1] += b;
  h_[2] += c;
  h_[3] += d;
  h_[4] += e;
  h_[5] += f;
  h_[6] += g;
  h_[7] += h;
}

void Sha256::update(const uint8_t* data, size_t len) {
  totalLen_ += len;
  while (len > 0) {
    size_t n = 64 - bufLen_;
    if (n > len) n = len;
    memcpy(buf_ + bufLen_, data, n);
    bufLen_ += n;
    data += n;
    len -= n;
    if (bufLen_ == 64) {
      block(buf_);
      bufLen_ = 0;
    }
  }
}

void Sha256::finish(uint8_t out[32]) {
  uint64_t bits = totalLen_ * 8;
  uint8_t pad = 0x80;
  update(&pad, 1);
  uint8_t zero = 0;
  while (bufLen_ != 56) update(&zero, 1);
  uint8_t lenBytes[8];
  for (int i = 0; i < 8; ++i) lenBytes[i] = (uint8_t)(bits >> (56 - 8 * i));
  update(lenBytes, 8);
  for (int i = 0; i < 8; ++i) {
    out[4 * i] = (uint8_t)(h_[i] >> 24);
    out[4 * i + 1] = (uint8_t)(h_[i] >> 16);
    out[4 * i + 2] = (uint8_t)(h_[i] >> 8);
    out[4 * i + 3] = (uint8_t)h_[i];
  }
}

void hmacSha256(const uint8_t* key, size_t keyLen, const uint8_t* msg, size_t msgLen, uint8_t out[32]) {
  uint8_t k[64] = {0};
  if (keyLen > 64) {
    Sha256 s;
    s.update(key, keyLen);
    s.finish(k);
  } else {
    memcpy(k, key, keyLen);
  }
  uint8_t pad[64];
  for (int i = 0; i < 64; ++i) pad[i] = k[i] ^ 0x36;
  Sha256 inner;
  inner.update(pad, 64);
  inner.update(msg, msgLen);
  uint8_t ih[32];
  inner.finish(ih);
  for (int i = 0; i < 64; ++i) pad[i] = k[i] ^ 0x5c;
  Sha256 outer;
  outer.update(pad, 64);
  outer.update(ih, 32);
  outer.finish(out);
  secureZero(k, sizeof(k));
  secureZero(pad, sizeof(pad));
  secureZero(ih, sizeof(ih));
}

bool hkdfSha256(const uint8_t* salt, size_t saltLen, const uint8_t* ikm, size_t ikmLen, const uint8_t* info,
                size_t infoLen, uint8_t* out, size_t outLen) {
  if (outLen > 255 * 32) return false;
  static const uint8_t kZeroSalt[32] = {0};
  if (salt == nullptr || saltLen == 0) {
    salt = kZeroSalt;
    saltLen = sizeof(kZeroSalt);
  }
  uint8_t prk[32];
  hmacSha256(salt, saltLen, ikm, ikmLen, prk);
  // T(i) = HMAC(PRK, T(i-1) | info | i); the input is assembled in a small
  // buffer, so info is limited to what fits beside T(i-1).
  uint8_t t[32];
  uint8_t block[32 + 128 + 1];
  if (infoLen > 128) return false;
  size_t done = 0, tLen = 0;
  for (uint8_t i = 1; done < outLen; ++i) {
    memcpy(block, t, tLen);
    if (infoLen) memcpy(block + tLen, info, infoLen);
    block[tLen + infoLen] = i;
    hmacSha256(prk, sizeof(prk), block, tLen + infoLen + 1, t);
    tLen = 32;
    size_t n = outLen - done < 32 ? outLen - done : 32;
    memcpy(out + done, t, n);
    done += n;
  }
  secureZero(prk, sizeof(prk));
  secureZero(t, sizeof(t));
  secureZero(block, sizeof(block));
  return true;
}

bool constantTimeEqual(const uint8_t* a, const uint8_t* b, size_t n) {
  // Accumulate every difference; no early exit, no data-dependent branch.
  volatile uint8_t diff = 0;
  for (size_t i = 0; i < n; ++i) diff = diff | (uint8_t)(a[i] ^ b[i]);
  return diff == 0;
}

void secureZero(void* p, size_t n) {
  volatile uint8_t* v = (volatile uint8_t*)p;
  while (n--) *v++ = 0;
}

}  // namespace vc
