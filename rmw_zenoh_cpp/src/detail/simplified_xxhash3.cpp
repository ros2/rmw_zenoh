/*
 * xxHash - Extremely Fast Hash algorithm
 * Header File
 * Copyright (C) 2012-2023 Yann Collet
 *
 * BSD 2-Clause License (https://www.opensource.org/licenses/bsd-license.php)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following disclaimer
 *      in the documentation and/or other materials provided with the
 *      distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You can contact the author at:
 *   - xxHash homepage: https://www.xxhash.com
 *   - xxHash source repository: https://github.com/Cyan4973/xxHash
 */

// For the liveliness utilities, we need a way to hash an arbitrary std::string into a
// 16-byte (128-bit) array.
// The xxhash3 algorithm as implemented at https://github.com/Cyan4973/xxHash can do exactly that.
// However, in order to not take an additional dependency here, the code below is extracted from
// https://github.com/Cyan4973/xxHash/commit/8ffd354cdfdf2d555f7cfc7eab023e9808801390,
// and greatly simplified.  In particular, xxHash is really concerned with speed, where we don't
// care that much.  We are interested in the proven collision, dispersion, and randomness
// properties.  Therefore, the simplified version below gets rid of all of the specialized
// speedups and only keeps the completely portable version of the code.
// In the future, if we find that we really need the additional speed, we can trivially take a
// dependency on the real xxHash (which is packaged almost everywhere), and use that instead.

#include <cstddef>
#include <cstdint>

#include "simplified_xxhash3.hpp"

#define XXH_STRIPE_LEN 64
#define XXH_SECRET_CONSUME_RATE 8
#define XXH_ACC_NB (XXH_STRIPE_LEN / sizeof(uint64_t))

#define XXH3_SECRET_SIZE_MIN 136

#define XXH_PRIME32_1  0x9E3779B1U  /*!< 0b10011110001101110111100110110001 */
#define XXH_PRIME32_2  0x85EBCA77U  /*!< 0b10000101111010111100101001110111 */
#define XXH_PRIME32_3  0xC2B2AE3DU  /*!< 0b11000010101100101010111000111101 */
#define XXH_PRIME32_4  0x27D4EB2FU  /*!< 0b00100111110101001110101100101111 */
#define XXH_PRIME32_5  0x165667B1U  /*!< 0b00010110010101100110011110110001 */

/*!< 0b1001111000110111011110011011000110000101111010111100101010000111 */
#define XXH_PRIME64_1  0x9E3779B185EBCA87ULL
/*!< 0b1100001010110010101011100011110100100111110101001110101101001111 */
#define XXH_PRIME64_2  0xC2B2AE3D27D4EB4FULL
/*!< 0b0001011001010110011001111011000110011110001101110111100111111001 */
#define XXH_PRIME64_3  0x165667B19E3779F9ULL
/*!< 0b1000010111101011110010100111011111000010101100101010111001100011 */
#define XXH_PRIME64_4  0x85EBCA77C2B2AE63ULL
/*!< 0b0010011111010100111010110010111100010110010101100110011111000101 */
#define XXH_PRIME64_5  0x27D4EB2F165667C5ULL

#define XXH3_MIDSIZE_MAX 240
#define XXH3_MIDSIZE_STARTOFFSET 3
#define XXH3_MIDSIZE_LASTOFFSET  17

#define XXH_SECRET_DEFAULT_SIZE 192   /* minimum XXH3_SECRET_SIZE_MIN */

static const uint8_t XXH3_kSecret[XXH_SECRET_DEFAULT_SIZE] = {
  0xb8, 0xfe, 0x6c, 0x39, 0x23, 0xa4, 0x4b, 0xbe, 0x7c, 0x01, 0x81, 0x2c, 0xf7, 0x21, 0xad, 0x1c,
  0xde, 0xd4, 0x6d, 0xe9, 0x83, 0x90, 0x97, 0xdb, 0x72, 0x40, 0xa4, 0xa4, 0xb7, 0xb3, 0x67, 0x1f,
  0xcb, 0x79, 0xe6, 0x4e, 0xcc, 0xc0, 0xe5, 0x78, 0x82, 0x5a, 0xd0, 0x7d, 0xcc, 0xff, 0x72, 0x21,
  0xb8, 0x08, 0x46, 0x74, 0xf7, 0x43, 0x24, 0x8e, 0xe0, 0x35, 0x90, 0xe6, 0x81, 0x3a, 0x26, 0x4c,
  0x3c, 0x28, 0x52, 0xbb, 0x91, 0xc3, 0x00, 0xcb, 0x88, 0xd0, 0x65, 0x8b, 0x1b, 0x53, 0x2e, 0xa3,
  0x71, 0x64, 0x48, 0x97, 0xa2, 0x0d, 0xf9, 0x4e, 0x38, 0x19, 0xef, 0x46, 0xa9, 0xde, 0xac, 0xd8,
  0xa8, 0xfa, 0x76, 0x3f, 0xe3, 0x9c, 0x34, 0x3f, 0xf9, 0xdc, 0xbb, 0xc7, 0xc7, 0x0b, 0x4f, 0x1d,
  0x8a, 0x51, 0xe0, 0x4b, 0xcd, 0xb4, 0x59, 0x31, 0xc8, 0x9f, 0x7e, 0xc9, 0xd9, 0x78, 0x73, 0x64,
  0xea, 0xc5, 0xac, 0x83, 0x34, 0xd3, 0xeb, 0xc3, 0xc5, 0x81, 0xa0, 0xff, 0xfa, 0x13, 0x63, 0xeb,
  0x17, 0x0d, 0xdd, 0x51, 0xb7, 0xf0, 0xda, 0x49, 0xd3, 0x16, 0x55, 0x26, 0x29, 0xd4, 0x68, 0x9e,
  0x2b, 0x16, 0xbe, 0x58, 0x7d, 0x47, 0xa1, 0xfc, 0x8f, 0xf8, 0xb8, 0xd1, 0x7a, 0xd0, 0x31, 0xce,
  0x45, 0xcb, 0x3a, 0x8f, 0x95, 0x16, 0x04, 0x28, 0xaf, 0xd7, 0xfb, 0xca, 0xbb, 0x4b, 0x40, 0x7e,
};

/*!< 0b0001011001010110011001111001000110011110001101110111100111111001 */
static const uint64_t PRIME_MX1 = 0x165667919E3779F9ULL;
/*!< 0b1001111110110010000111000110010100011110100110001101111100100101 */
static const uint64_t PRIME_MX2 = 0x9FB21C651E98DF25ULL;

#define XXH_SECRET_LASTACC_START 7 /* not aligned on 8, last secret is different from acc & scrambler */  // NOLINT
#define XXH_SECRET_MERGEACCS_START 11

static uint64_t XXH_readLE64(const void * memPtr)
{
  const uint8_t * bytePtr = static_cast<const uint8_t *>(memPtr);
  return bytePtr[0] |
         (static_cast<uint64_t>(bytePtr[1]) << 8) |
         (static_cast<uint64_t>(bytePtr[2]) << 16) |
         (static_cast<uint64_t>(bytePtr[3]) << 24) |
         (static_cast<uint64_t>(bytePtr[4]) << 32) |
         (static_cast<uint64_t>(bytePtr[5]) << 40) |
         (static_cast<uint64_t>(bytePtr[6]) << 48) |
         (static_cast<uint64_t>(bytePtr[7]) << 56);
}

static uint64_t XXH_mult32to64(uint64_t x, uint64_t y)
{
  return (x & 0xFFFFFFFF) * (y & 0xFFFFFFFF);
}

static uint64_t XXH_mult32to64_add64(uint64_t lhs, uint64_t rhs, uint64_t acc)
{
  return XXH_mult32to64(static_cast<uint32_t>(lhs), static_cast<uint32_t>(rhs)) + acc;
}

static void XXH3_scalarRound(void * acc, void const * input, void const * secret, size_t lane)
{
  uint64_t * xacc = static_cast<uint64_t *>(acc);
  uint8_t const * xinput = static_cast<uint8_t const *>(input);
  uint8_t const * xsecret = static_cast<uint8_t const *>(secret);
  uint64_t const data_val = XXH_readLE64(xinput + lane * 8);
  uint64_t const data_key = data_val ^ XXH_readLE64(xsecret + lane * 8);
  xacc[lane ^ 1] += data_val; /* swap adjacent lanes */
  xacc[lane] = XXH_mult32to64_add64(data_key /* & 0xFFFFFFFF */, data_key >> 32, xacc[lane]);
}

static void XXH3_accumulate_512_scalar(void * acc, const void * input, const void * secret)
{
  size_t i;
  for (i = 0; i < XXH_ACC_NB; i++) {
    XXH3_scalarRound(acc, input, secret, i);
  }
}

static void XXH3_accumulate_scalar(
  uint64_t * acc, const uint8_t * input, const uint8_t * secret,
  size_t nbStripes)
{
  size_t n;
  for (n = 0; n < nbStripes; n++) {
    const uint8_t * const in = input + n * XXH_STRIPE_LEN;
    XXH3_accumulate_512_scalar(acc, in, secret + n * XXH_SECRET_CONSUME_RATE);
  }
}

static uint64_t XXH_xorshift64(uint64_t v64, int shift)
{
  return v64 ^ (v64 >> shift);
}

static void XXH3_scalarScrambleRound(void * acc, void const * secret, size_t lane)
{
  uint64_t * const xacc = static_cast<uint64_t *>(acc);   /* presumed aligned */
  /* no alignment restriction */
  const uint8_t * const xsecret = static_cast<const uint8_t *>(secret);
  uint64_t const key64 = XXH_readLE64(xsecret + lane * 8);
  uint64_t acc64 = xacc[lane];
  acc64 = XXH_xorshift64(acc64, 47);
  acc64 ^= key64;
  acc64 *= XXH_PRIME32_1;
  xacc[lane] = acc64;
}

static void XXH3_scrambleAcc_scalar(void * acc, const void * secret)
{
  size_t i;
  for (i = 0; i < XXH_ACC_NB; i++) {
    XXH3_scalarScrambleRound(acc, secret, i);
  }
}

static void XXH3_hashLong_internal_loop(uint64_t * acc, const uint8_t * input, size_t len)
{
  size_t const nbStripesPerBlock = (sizeof(XXH3_kSecret) - XXH_STRIPE_LEN) /
    XXH_SECRET_CONSUME_RATE;
  size_t const block_len = XXH_STRIPE_LEN * nbStripesPerBlock;
  size_t const nb_blocks = (len - 1) / block_len;

  size_t n;

  for (n = 0; n < nb_blocks; n++) {
    XXH3_accumulate_scalar(acc, input + n * block_len, XXH3_kSecret, nbStripesPerBlock);
    XXH3_scrambleAcc_scalar(acc, XXH3_kSecret + sizeof(XXH3_kSecret) - XXH_STRIPE_LEN);
  }

  /* last partial block */
  size_t const nbStripes = ((len - 1) - (block_len * nb_blocks)) / XXH_STRIPE_LEN;
  XXH3_accumulate_scalar(acc, input + nb_blocks * block_len, XXH3_kSecret, nbStripes);

  /* last stripe */
  const uint8_t * const p = input + len - XXH_STRIPE_LEN;
  XXH3_accumulate_512_scalar(
    acc, p, XXH3_kSecret + sizeof(XXH3_kSecret) - XXH_STRIPE_LEN - XXH_SECRET_LASTACC_START);
}

static simplified_XXH128_hash_t XXH_mult64to128(uint64_t lhs, uint64_t rhs)
{
  uint64_t const lo_lo = XXH_mult32to64(lhs & 0xFFFFFFFF, rhs & 0xFFFFFFFF);
  uint64_t const hi_lo = XXH_mult32to64(lhs >> 32, rhs & 0xFFFFFFFF);
  uint64_t const lo_hi = XXH_mult32to64(lhs & 0xFFFFFFFF, rhs >> 32);
  uint64_t const hi_hi = XXH_mult32to64(lhs >> 32, rhs >> 32);

  /* Now add the products together. These will never overflow. */
  uint64_t const cross = (lo_lo >> 32) + (hi_lo & 0xFFFFFFFF) + lo_hi;
  uint64_t const upper = (hi_lo >> 32) + (cross >> 32) + hi_hi;
  uint64_t const lower = (cross << 32) | (lo_lo & 0xFFFFFFFF);

  simplified_XXH128_hash_t r128;
  r128.low64 = lower;
  r128.high64 = upper;
  return r128;
}

static uint64_t XXH3_mul128_fold64(uint64_t lhs, uint64_t rhs)
{
  simplified_XXH128_hash_t product = XXH_mult64to128(lhs, rhs);
  return product.low64 ^ product.high64;
}

static uint64_t XXH3_mix2Accs(const uint64_t * acc, const uint8_t * secret)
{
  return XXH3_mul128_fold64(acc[0] ^ XXH_readLE64(secret), acc[1] ^ XXH_readLE64(secret + 8));
}

static uint64_t XXH3_avalanche(uint64_t h64)
{
  h64 = XXH_xorshift64(h64, 37);
  h64 *= PRIME_MX1;
  h64 = XXH_xorshift64(h64, 32);
  return h64;
}

static uint64_t XXH3_mergeAccs(const uint64_t * acc, const uint8_t * secret, uint64_t start)
{
  uint64_t result64 = start;
  size_t i = 0;

  for (i = 0; i < 4; i++) {
    result64 += XXH3_mix2Accs(acc + 2 * i, secret + 16 * i);
  }

  return XXH3_avalanche(result64);
}

static simplified_XXH128_hash_t XXH3_hashLong_128b_default(const void * input, size_t len)
{
  uint64_t acc[XXH_ACC_NB] = {
    XXH_PRIME32_3, XXH_PRIME64_1, XXH_PRIME64_2, XXH_PRIME64_3,
    XXH_PRIME64_4, XXH_PRIME32_2, XXH_PRIME64_5, XXH_PRIME32_1
  };

  XXH3_hashLong_internal_loop(acc, static_cast<const uint8_t *>(input), len);

  simplified_XXH128_hash_t h128;
  h128.low64 = XXH3_mergeAccs(
    acc, XXH3_kSecret + XXH_SECRET_MERGEACCS_START, static_cast<uint64_t>(len) * XXH_PRIME64_1);
  h128.high64 = XXH3_mergeAccs(
    acc,
    XXH3_kSecret + sizeof(XXH3_kSecret) - sizeof(acc) - XXH_SECRET_MERGEACCS_START,
    ~(static_cast<uint64_t>(len) * XXH_PRIME64_2));
  return h128;
}

static uint64_t XXH_swap64(uint64_t x)
{
  return ((x << 56) & 0xff00000000000000ULL) |
         ((x << 40) & 0x00ff000000000000ULL) |
         ((x << 24) & 0x0000ff0000000000ULL) |
         ((x << 8) & 0x000000ff00000000ULL) |
         ((x >> 8) & 0x00000000ff000000ULL) |
         ((x >> 24) & 0x0000000000ff0000ULL) |
         ((x >> 40) & 0x000000000000ff00ULL) |
         ((x >> 56) & 0x00000000000000ffULL);
}

static simplified_XXH128_hash_t XXH3_len_9to16_128b(const uint8_t * input, size_t len)
{
  uint64_t const bitflipl = (XXH_readLE64(XXH3_kSecret + 32) ^ XXH_readLE64(XXH3_kSecret + 40));
  uint64_t const bitfliph = (XXH_readLE64(XXH3_kSecret + 48) ^ XXH_readLE64(XXH3_kSecret + 56));
  uint64_t const input_lo = XXH_readLE64(input);
  uint64_t input_hi = XXH_readLE64(input + len - 8);
  simplified_XXH128_hash_t m128 = XXH_mult64to128(input_lo ^ input_hi ^ bitflipl, XXH_PRIME64_1);
  /*
   * Put len in the middle of m128 to ensure that the length gets mixed to
   * both the low and high bits in the 128x64 multiply below.
   */
  m128.low64 += static_cast<uint64_t>(len - 1) << 54;
  input_hi ^= bitfliph;
  /*
   * Add the high 32 bits of input_hi to the high 32 bits of m128, then
   * add the long product of the low 32 bits of input_hi and XXH_PRIME32_2 to
   * the high 64 bits of m128.
   *
   * The best approach to this operation is different on 32-bit and 64-bit.
   */
  if (sizeof(void *) < sizeof(uint64_t)) { /* 32-bit */
    /*
     * 32-bit optimized version, which is more readable.
     *
     * On 32-bit, it removes an ADC and delays a dependency between the two
     * halves of m128.high64, but it generates an extra mask on 64-bit.
     */
    m128.high64 += (input_hi & 0xFFFFFFFF00000000ULL) +
      XXH_mult32to64(static_cast<uint32_t>(input_hi), XXH_PRIME32_2);
  } else {
    /*
     * 64-bit optimized (albeit more confusing) version.
     *
     * Uses some properties of addition and multiplication to remove the mask:
     *
     * Let:
     *    a = input_hi.lo = (input_hi & 0x00000000FFFFFFFF)
     *    b = input_hi.hi = (input_hi & 0xFFFFFFFF00000000)
     *    c = XXH_PRIME32_2
     *
     *    a + (b * c)
     * Inverse Property: x + y - x == y
     *    a + (b * (1 + c - 1))
     * Distributive Property: x * (y + z) == (x * y) + (x * z)
     *    a + (b * 1) + (b * (c - 1))
     * Identity Property: x * 1 == x
     *    a + b + (b * (c - 1))
     *
     * Substitute a, b, and c:
     *    input_hi.hi + input_hi.lo + ((uint64_t)input_hi.lo * (XXH_PRIME32_2 - 1))
     *
     * Since input_hi.hi + input_hi.lo == input_hi, we get this:
     *    input_hi + ((uint64_t)input_hi.lo * (XXH_PRIME32_2 - 1))
     */
    m128.high64 += input_hi + XXH_mult32to64(static_cast<uint32_t>(input_hi), XXH_PRIME32_2 - 1);
  }
  /* m128 ^= XXH_swap64(m128 >> 64); */
  m128.low64 ^= XXH_swap64(m128.high64);

  /* 128x64 multiply: h128 = m128 * XXH_PRIME64_2; */
  simplified_XXH128_hash_t h128 = XXH_mult64to128(m128.low64, XXH_PRIME64_2);
  h128.high64 += m128.high64 * XXH_PRIME64_2;

  h128.low64 = XXH3_avalanche(h128.low64);
  h128.high64 = XXH3_avalanche(h128.high64);
  return h128;
}

static uint32_t XXH_swap32(uint32_t x)
{
  return ((x << 24) & 0xff000000) |
         ((x << 8) & 0x00ff0000) |
         ((x >> 8) & 0x0000ff00) |
         ((x >> 24) & 0x000000ff);
}

static uint32_t XXH_readLE32(const void * memPtr)
{
  const uint8_t * bytePtr = static_cast<const uint8_t *>(memPtr);
  return bytePtr[0] |
         (static_cast<uint32_t>(bytePtr[1]) << 8) |
         (static_cast<uint32_t>(bytePtr[2]) << 16) |
         (static_cast<uint32_t>(bytePtr[3]) << 24);
}

static simplified_XXH128_hash_t XXH3_len_4to8_128b(const uint8_t * input, size_t len)
{
  uint32_t const input_lo = XXH_readLE32(input);
  uint32_t const input_hi = XXH_readLE32(input + len - 4);
  uint64_t const input_64 = input_lo + (static_cast<uint64_t>(input_hi) << 32);
  uint64_t const bitflip = (XXH_readLE64(XXH3_kSecret + 16) ^ XXH_readLE64(XXH3_kSecret + 24));
  uint64_t const keyed = input_64 ^ bitflip;

  /* Shift len to the left to ensure it is even, this avoids even multiplies. */
  simplified_XXH128_hash_t m128 = XXH_mult64to128(keyed, XXH_PRIME64_1 + (len << 2));

  m128.high64 += (m128.low64 << 1);
  m128.low64 ^= (m128.high64 >> 3);

  m128.low64 = XXH_xorshift64(m128.low64, 35);
  m128.low64 *= PRIME_MX2;
  m128.low64 = XXH_xorshift64(m128.low64, 28);
  m128.high64 = XXH3_avalanche(m128.high64);
  return m128;
}

static uint32_t XXH_rotl32(uint32_t x, uint32_t r)
{
  return ((x) << (r)) | ((x) >> (32 - (r)));
}

static uint64_t XXH64_avalanche(uint64_t hash)
{
  hash ^= hash >> 33;
  hash *= XXH_PRIME64_2;
  hash ^= hash >> 29;
  hash *= XXH_PRIME64_3;
  hash ^= hash >> 32;
  return hash;
}

static simplified_XXH128_hash_t XXH3_len_1to3_128b(const uint8_t * input, size_t len)
{
  /* A doubled version of 1to3_64b with different constants. */
  /*
   * len = 1: combinedl = { input[0], 0x01, input[0], input[0] }
   * len = 2: combinedl = { input[1], 0x02, input[0], input[1] }
   * len = 3: combinedl = { input[2], 0x03, input[0], input[1] }
   */
  uint8_t const c1 = input[0];
  uint8_t const c2 = input[len >> 1];
  uint8_t const c3 = input[len - 1];
  uint32_t const combinedl = (static_cast<uint32_t>(c1) << 16) | (static_cast<uint32_t>(c2) << 24) |
    (static_cast<uint32_t>(c3) << 0) | (static_cast<uint32_t>(len) << 8);
  uint32_t const combinedh = XXH_rotl32(XXH_swap32(combinedl), 13);
  uint64_t const bitflipl = (XXH_readLE32(XXH3_kSecret) ^ XXH_readLE32(XXH3_kSecret + 4));
  uint64_t const bitfliph = (XXH_readLE32(XXH3_kSecret + 8) ^ XXH_readLE32(XXH3_kSecret + 12));
  uint64_t const keyed_lo = static_cast<uint64_t>(combinedl) ^ bitflipl;
  uint64_t const keyed_hi = static_cast<uint64_t>(combinedh) ^ bitfliph;
  simplified_XXH128_hash_t h128;
  h128.low64 = XXH64_avalanche(keyed_lo);
  h128.high64 = XXH64_avalanche(keyed_hi);
  return h128;
}

static simplified_XXH128_hash_t XXH3_len_0to16_128b(const uint8_t * input, size_t len)
{
  if (len > 8) {
    return XXH3_len_9to16_128b(input, len);
  }
  if (len >= 4) {
    return XXH3_len_4to8_128b(input, len);
  }
  if (len) {
    return XXH3_len_1to3_128b(input, len);
  }
  simplified_XXH128_hash_t h128;
  uint64_t const bitflipl = XXH_readLE64(XXH3_kSecret + 64) ^ XXH_readLE64(XXH3_kSecret + 72);
  uint64_t const bitfliph = XXH_readLE64(XXH3_kSecret + 80) ^ XXH_readLE64(XXH3_kSecret + 88);
  h128.low64 = XXH64_avalanche(0 ^ bitflipl);
  h128.high64 = XXH64_avalanche(0 ^ bitfliph);
  return h128;
}

static uint64_t XXH3_mix16B(const uint8_t * input, const uint8_t * secret)
{
  uint64_t const input_lo = XXH_readLE64(input);
  uint64_t const input_hi = XXH_readLE64(input + 8);
  return XXH3_mul128_fold64(
    input_lo ^ (XXH_readLE64(secret)),
    input_hi ^ (XXH_readLE64(secret + 8)));
}

static simplified_XXH128_hash_t XXH128_mix32B(
  simplified_XXH128_hash_t acc, const uint8_t * input_1,
  const uint8_t * input_2, const uint8_t * secret)
{
  acc.low64 += XXH3_mix16B(input_1, secret);
  acc.low64 ^= XXH_readLE64(input_2) + XXH_readLE64(input_2 + 8);
  acc.high64 += XXH3_mix16B(input_2, secret + 16);
  acc.high64 ^= XXH_readLE64(input_1) + XXH_readLE64(input_1 + 8);
  return acc;
}

static simplified_XXH128_hash_t XXH3_len_17to128_128b(const uint8_t * input, size_t len)
{
  simplified_XXH128_hash_t acc;
  acc.low64 = len * XXH_PRIME64_1;
  acc.high64 = 0;

  if (len > 32) {
    if (len > 64) {
      if (len > 96) {
        acc = XXH128_mix32B(acc, input + 48, input + len - 64, XXH3_kSecret + 96);
      }
      acc = XXH128_mix32B(acc, input + 32, input + len - 48, XXH3_kSecret + 64);
    }
    acc = XXH128_mix32B(acc, input + 16, input + len - 32, XXH3_kSecret + 32);
  }
  acc = XXH128_mix32B(acc, input, input + len - 16, XXH3_kSecret);
  simplified_XXH128_hash_t h128;
  h128.low64 = acc.low64 + acc.high64;
  h128.high64 = (acc.low64 * XXH_PRIME64_1) + (acc.high64 * XXH_PRIME64_4) + (len * XXH_PRIME64_2);
  h128.low64 = XXH3_avalanche(h128.low64);
  h128.high64 = static_cast<uint64_t>(0) - XXH3_avalanche(h128.high64);
  return h128;
}

static simplified_XXH128_hash_t XXH3_len_129to240_128b(const uint8_t * input, size_t len)
{
  simplified_XXH128_hash_t acc;
  unsigned i;
  acc.low64 = len * XXH_PRIME64_1;
  acc.high64 = 0;
  /*
   *  We set as `i` as offset + 32. We do this so that unchanged
   * `len` can be used as upper bound. This reaches a sweet spot
   * where both x86 and aarch64 get simple agen and good codegen
   * for the loop.
   */
  for (i = 32; i < 160; i += 32) {
    acc = XXH128_mix32B(acc, input + i - 32, input + i - 16, XXH3_kSecret + i - 32);
  }
  acc.low64 = XXH3_avalanche(acc.low64);
  acc.high64 = XXH3_avalanche(acc.high64);
  /*
   * NB: `i <= len` will duplicate the last 32-bytes if
   * len % 32 was zero. This is an unfortunate necessity to keep
   * the hash result stable.
   */
  for (i = 160; i <= len; i += 32) {
    acc = XXH128_mix32B(
      acc, input + i - 32, input + i - 16, XXH3_kSecret + XXH3_MIDSIZE_STARTOFFSET + i - 160);
  }
  /* last bytes */
  acc = XXH128_mix32B(
    acc, input + len - 16, input + len - 32,
    XXH3_kSecret + XXH3_SECRET_SIZE_MIN - XXH3_MIDSIZE_LASTOFFSET - 16);

  simplified_XXH128_hash_t h128;
  h128.low64 = acc.low64 + acc.high64;
  h128.high64 = (acc.low64 * XXH_PRIME64_1) +
    (acc.high64 * XXH_PRIME64_4) +
    (len * XXH_PRIME64_2);
  h128.low64 = XXH3_avalanche(h128.low64);
  h128.high64 = static_cast<uint64_t>(0) - XXH3_avalanche(h128.high64);
  return h128;
}

simplified_XXH128_hash_t simplified_XXH3_128bits(const void * data, size_t len)
{
  if (len <= 16) {
    return XXH3_len_0to16_128b(static_cast<const uint8_t *>(data), len);
  }
  if (len <= 128) {
    return XXH3_len_17to128_128b(static_cast<const uint8_t *>(data), len);
  }
  if (len <= XXH3_MIDSIZE_MAX) {
    return XXH3_len_129to240_128b(static_cast<const uint8_t *>(data), len);
  }
  return XXH3_hashLong_128b_default(data, len);
}
