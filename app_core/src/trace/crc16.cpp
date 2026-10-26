#include "crc16.h"

#include <array>

namespace {

constexpr uint16_t POLYNOMIAL {0x1021U};
constexpr uint16_t INIT {0xFFFFU};

constexpr std::array<uint16_t, 256> make_table()
{
    std::array<uint16_t, 256> table {};
    for (std::size_t i = 0; i < table.size(); ++i) {
        auto crc = static_cast<uint16_t>(i << 8U);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) != 0U ? static_cast<uint16_t>((crc << 1U) ^ POLYNOMIAL) :
                                          static_cast<uint16_t>(crc << 1U);
        }
        table[i] = crc;
    }
    return table;
}

constexpr std::array<uint16_t, 256> TABLE {make_table()};

}  // namespace

uint16_t robby::crc16_ccitt_false(const uint8_t* data, std::size_t length)
{
    uint16_t crc {INIT};
    for (std::size_t i = 0; i < length; ++i) {
        crc = static_cast<uint16_t>((crc << 8U) ^ TABLE[((crc >> 8U) ^ data[i]) & 0xFFU]);
    }
    return crc;
}
