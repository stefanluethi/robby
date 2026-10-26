#pragma once

#include "crc16.h"
#include "frame.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include <robby_link.pb.h>

namespace robby {

/// Splits the bytes from the outpost into frames at each 0x00, checks COBS and the CRC,
/// decodes the Downlink and counts what it receives (proto/README.md, "Frames on the
/// UART" and "Sequence numbers").
///
/// Fed by one task; the counters may be read from any task.
class DownlinkReader
{
public:
    struct Counters
    {
        /// Frames that decoded
        std::atomic<uint32_t> rx_frames {0U};
        /// Frames rejected: too long, COBS, CRC or protobuf errors
        std::atomic<uint32_t> rx_errors {0U};
        /// Frames missing by their seq
        std::atomic<uint32_t> rx_seq_gaps {0U};
    };

    explicit DownlinkReader(Crc16Function crc);

    /// Takes the next byte. Returns true when it completed a frame that decoded, which is
    /// then in downlink() until the next call.
    bool push(uint8_t byte);

    const robby_link_Downlink& downlink() const { return _downlink; }
    const Counters& counters() const { return _counters; }

private:
    /// Decodes the frame in _encoded, true if it is a valid Downlink
    bool decode();
    void count_seq(uint32_t seq);

    Crc16Function _crc;
    Counters _counters {};

    std::array<uint8_t, frame::MAX_ENCODED> _encoded {};
    std::size_t _length {0U};
    /// The frame got too long, its bytes are skipped up to the next 0x00
    bool _overflow {false};
    std::array<uint8_t, frame::MAX_DECODED> _decoded {};
    robby_link_Downlink _downlink {};

    bool _seq_known {false};
    uint32_t _seq_expected {0U};
};

}  // namespace robby
