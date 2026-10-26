#pragma once

#include <cstddef>
#include <cstdint>

namespace robby {

/// Computes the CRC of a link frame
using Crc16Function = uint16_t (*)(const uint8_t* data, std::size_t length);

/// CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF, not reflected, no final XOR
/// ("123456789" gives 0x29B1). Table based, safe to call from any task.
uint16_t crc16_ccitt_false(const uint8_t* data, std::size_t length);

}  // namespace robby
