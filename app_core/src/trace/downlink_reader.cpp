#include "downlink_reader.h"

#include <pb_decode.h>

using namespace robby;

DownlinkReader::DownlinkReader(Crc16Function crc) : _crc {crc} { }

bool DownlinkReader::push(uint8_t byte)
{
    if (byte != 0x00U) {
        if (_length < _encoded.size() - 1U) {
            _encoded[_length++] = byte;
        } else {
            _overflow = true;
        }
        return false;
    }

    // end of a frame
    bool decoded {false};
    if (_overflow) {
        _counters.rx_errors.fetch_add(1U, std::memory_order_relaxed);
    } else if (_length > 0U) {
        // repeated delimiters are no frames
        _encoded[_length++] = byte;
        decoded = decode();
        if (decoded) {
            _counters.rx_frames.fetch_add(1U, std::memory_order_relaxed);
            count_seq(_downlink.seq);
        } else {
            _counters.rx_errors.fetch_add(1U, std::memory_order_relaxed);
        }
    }

    _length = 0U;
    _overflow = false;
    return decoded;
}

bool DownlinkReader::decode()
{
    std::size_t decoded_length {0U};
    if (cobs_decode(_encoded.data(), _length, _decoded.data(), _decoded.size(), &decoded_length) != COBS_RET_SUCCESS ||
        decoded_length < frame::CRC_SIZE) {
        return false;
    }
    // the CRC over the payload and its big-endian trailer is 0
    if (_crc(_decoded.data(), decoded_length) != 0U) {
        return false;
    }

    _downlink = robby_link_Downlink_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(_decoded.data(), decoded_length - frame::CRC_SIZE);
    return pb_decode(&stream, robby_link_Downlink_fields, &_downlink);
}

void DownlinkReader::count_seq(uint32_t seq)
{
    if (_seq_known && seq != 0U) {
        const auto gap = static_cast<int32_t>(seq - _seq_expected);
        // a negative gap is a restart of the outpost, like seq 0
        if (gap > 0) {
            _counters.rx_seq_gaps.fetch_add(static_cast<uint32_t>(gap), std::memory_order_relaxed);
        }
    }
    _seq_known = true;
    _seq_expected = seq + 1U;
}
