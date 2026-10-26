#include "trace/crc16.h"

#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cstdint>

using robby::crc16_ccitt_false;

TEST_CASE("CRC-16/CCITT-FALSE gives the check value")
{
    constexpr std::array<uint8_t, 9> CHECK {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(crc16_ccitt_false(CHECK.data(), CHECK.size()) == 0x29B1U);
}

TEST_CASE("CRC of nothing is the init value") { CHECK(crc16_ccitt_false(nullptr, 0U) == 0xFFFFU); }

TEST_CASE("CRC over the payload and its big-endian trailer is 0")
{
    std::array<uint8_t, 7> data {0x08, 0x07, 0x12, 0x03, 0x10, 0xF4, 0x03};
    const uint16_t crc {crc16_ccitt_false(data.data(), 5U)};
    data[5] = static_cast<uint8_t>(crc >> 8U);
    data[6] = static_cast<uint8_t>(crc & 0xFFU);
    CHECK(crc16_ccitt_false(data.data(), data.size()) == 0U);
}
