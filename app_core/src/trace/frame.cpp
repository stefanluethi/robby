#include "frame.h"

std::size_t
robby::frame::encode(uint8_t* payload, std::size_t length, Crc16Function crc, uint8_t* out, std::size_t out_size)
{
    if (length > MAX_PAYLOAD) {
        return 0U;
    }

    const uint16_t checksum {crc(payload, length)};
    payload[length] = static_cast<uint8_t>(checksum >> 8U);
    payload[length + 1U] = static_cast<uint8_t>(checksum & 0xFFU);

    std::size_t encoded_length {0U};
    if (cobs_encode(payload, length + CRC_SIZE, out, out_size, &encoded_length) != COBS_RET_SUCCESS) {
        return 0U;
    }
    return encoded_length;
}
