#pragma once

#include "crc16.h"

#include <cobs.h>

#include <cstddef>
#include <cstdint>

namespace robby::frame {

// The frames of the link in both directions, see proto/README.md:
//   decoded = payload ‖ CRC-16(payload), big-endian    at most 1024 bytes
//   frame   = COBS(decoded) ‖ 0x00                     at most 1030 bytes

constexpr std::size_t CRC_SIZE {2U};
constexpr std::size_t MAX_DECODED {1024U};
constexpr std::size_t MAX_PAYLOAD {MAX_DECODED - CRC_SIZE};
/// COBS overhead and the delimiter included
constexpr std::size_t MAX_ENCODED {COBS_ENCODE_MAX(MAX_DECODED)};

static_assert(MAX_ENCODED == 1030U);

/// Appends the CRC to the `length` bytes of `payload` (which must have room for it) and
/// encodes the result into `out`. Returns the size of the frame, 0 if it doesn't fit.
std::size_t encode(uint8_t* payload, std::size_t length, Crc16Function crc, uint8_t* out, std::size_t out_size);

}  // namespace robby::frame
