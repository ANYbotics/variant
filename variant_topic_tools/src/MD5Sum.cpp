/******************************************************************************
 * Copyright (C) 2014 by Ralf Kaestner                                        *
 * ralf.kaestner@gmail.com                                                    *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "variant_topic_tools/MD5Sum.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Static initializations                                                    */
/*****************************************************************************/

const MD5Sum::Digest MD5Sum::initialDigest = {0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476};

const boost::array<uint32_t, 64> MD5Sum::numBitRotations = {
    7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20,
    4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};

const boost::array<uint32_t, 64> MD5Sum::constants = {
    0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501, 0x698098d8, 0x8b44f7af, 0xffff5bb1,
    0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821, 0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453,
    0xd8a1e681, 0xe7d3fbc8, 0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a, 0xfffa3942,
    0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70, 0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05,
    0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665, 0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d,
    0x85845dd1, 0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391};

const MD5Sum::Block MD5Sum::padding = {0x00000080, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                                       0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000};

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MD5Sum::MD5Sum(const std::string& text) {
  clear();
  update(text);
}

MD5Sum::MD5Sum(const MD5Sum& src) = default;

MD5Sum::~MD5Sum() = default;

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

size_t MD5Sum::getNumDigestedBits() const {
  const auto* numDigestedBits = reinterpret_cast<const uint8_t*>(this->numDigestedBits.data());
  size_t i = Size::size() * sizeof(uint32_t) - sizeof(size_t);

  return *reinterpret_cast<const size_t*>(&numDigestedBits[i]);
}

MD5Sum::Digest MD5Sum::getDigest() const {
  Digest digest(this->digest);
  Size numDigestedBits(this->numDigestedBits);
  Block buffer(this->buffer);

  finalize(digest, numDigestedBits, buffer);

  return digest;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MD5Sum::update(const std::string& text) {
  if (!text.empty()) {
    update(reinterpret_cast<const uint8_t*>(text.data()), text.length());
  }
}

void MD5Sum::update(const uint8_t* bytes, size_t numBytes) {
  update(digest, numDigestedBits, buffer, bytes, numBytes);
}

void MD5Sum::clear() {
  digest = initialDigest;

  numDigestedBits.fill(0);
  buffer.fill(0);
}

void MD5Sum::write(std::ostream& stream) const {
  stream << toString();
}

std::string MD5Sum::toString() const {
  Digest digest = getDigest();

  char stringData[2 * Digest::size() * sizeof(uint32_t) + 1];
  const auto* digestData = reinterpret_cast<const uint8_t*>(digest.data());

  for (size_t i = 0; i < Digest::size() * sizeof(uint32_t); ++i) {
    sprintf(&stringData[2 * i], "%02x", digestData[i]);
  }
  stringData[2 * Digest::size() * sizeof(uint32_t)] = 0;

  return std::string(stringData);
}

void MD5Sum::transform(const Block& block, Digest& digest) {
  uint32_t a = digest[0];
  uint32_t b = digest[1];
  uint32_t c = digest[2];
  uint32_t d = digest[3];

  for (size_t i = 0; i < 64; ++i) {
    uint32_t f = 0;
    uint32_t g = 0;
    uint32_t h = 0;

    if (i < 16) {
      f = (b & c) | (~b & d);
      g = i;
    } else if (i < 32) {
      f = (b & d) | (c & ~d);
      g = (5 * i + 1) % 16;
    } else if (i < 48) {
      f = b ^ c ^ d;
      g = (3 * i + 5) % 16;
    } else if (i < 64) {
      f = c ^ (b | ~d);
      g = (7 * i) % 16;
    }

    h = d;
    d = c;
    c = b;
    b += rotateLeft(a + f + constants[i] + block[g], numBitRotations[i]);
    a = h;
  }

  digest[0] += a;
  digest[1] += b;
  digest[2] += c;
  digest[3] += d;
}

uint32_t MD5Sum::rotateLeft(uint32_t value, size_t numBits) {
  return (value << numBits) | (value >> (sizeof(uint32_t) * 8 - numBits));
}

void MD5Sum::update(Digest& digest, Size& numDigestedBits, Block& buffer, const uint8_t* bytes, size_t numBytes) {
  size_t i = numDigestedBits[0] / 8 % 64;
  size_t lengthOfFirstPart = 64 - i;
  size_t j = 0;

  if ((numDigestedBits[0] += (numBytes << 3)) < (numBytes << 3)) {
    ++numDigestedBits[1];
  }
  numDigestedBits[1] += (numBytes >> 29);

  if (numBytes >= lengthOfFirstPart) {
    std::copy(&bytes[0], &bytes[lengthOfFirstPart], &reinterpret_cast<uint8_t*>(buffer.data())[i]);
    transform(buffer, digest);

    for (j = lengthOfFirstPart; j + 64 <= numBytes; j += 64) {
      Block block;

      std::copy(&bytes[j], &bytes[j + 64], reinterpret_cast<uint8_t*>(block.data()));
      transform(block, digest);
    }

    i = 0;
  } else {
    j = 0;
  }

  std::copy(&bytes[j], &bytes[numBytes], &reinterpret_cast<uint8_t*>(buffer.data())[i]);
}

void MD5Sum::finalize(Digest& digest, Size& numDigestedBits, Block& buffer) {
  Size finalNumDigestedBits(numDigestedBits);
  size_t i = numDigestedBits[0] / 8 % 64;
  size_t paddedLength = (i < 56) ? 56 - i : 120 - i;

  update(digest, numDigestedBits, buffer, reinterpret_cast<const uint8_t*>(padding.data()), paddedLength);
  update(digest, numDigestedBits, buffer, reinterpret_cast<const uint8_t*>(finalNumDigestedBits.data()), 8);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

MD5Sum& MD5Sum::operator<<(const std::string& text) {
  update(text);
  return *this;
}

std::ostream& operator<<(std::ostream& stream, const MD5Sum& md5Sum) {
  md5Sum.write(stream);
  return stream;
}

}  // namespace variant_topic_tools
