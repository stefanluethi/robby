#pragma once

#include "crc16.h"
#include "frame.h"

#include <array>
#include <cstddef>
#include <cstdint>

#include <robby_link.pb.h>

namespace robby {

/// Where the frames go: the UART on the target, a buffer in the tests.
class ITransport
{
public:
    virtual ~ITransport() = default;

    /// Starts sending a frame. `data` stays valid until the next write returns, so the
    /// transport may send it in the background. Returns false if the frame was dropped.
    virtual bool write(const uint8_t* data, std::size_t length) = 0;
};

/// Packs records into Uplink frames (proto/README.md, "Packing records"): `seq`, then as
/// many records as fit into 1022 bytes. A record that doesn't fit into the open frame
/// sends it and starts the next one. Never builds a whole robby_link_Uplink.
///
/// Not thread safe: the owner serialises the calls (robby::Link holds a mutex).
class UplinkWriter
{
public:
    /// Records per frame, max_count of Uplink.records in robby_link.options
    static constexpr std::size_t MAX_RECORDS {32U};

    struct Counters
    {
        /// Bytes handed to the transport, COBS overhead and delimiters included
        uint32_t tx_bytes {0U};
        uint32_t tx_frames {0U};
        /// Frames the transport dropped
        uint32_t tx_frames_dropped {0U};
        /// Records that could not be encoded
        uint32_t records_dropped {0U};
    };

    /// `first_seq` is the seq of the first frame (0 after a reset; set by the tests)
    UplinkWriter(ITransport& transport, Crc16Function crc, uint32_t first_seq = 0U);

    /// Adds a record to the open frame, sends the frame first if the record doesn't fit
    /// any more. Returns false if the record was dropped (it doesn't encode).
    bool add(const robby_link_Record& record);

    /// Sends the open frame, if it has any records
    void flush();

    /// The seq of the next frame
    uint32_t seq() const { return _seq; }
    const Counters& counters() const { return _counters; }

private:
    void open_frame();

    ITransport& _transport;
    Crc16Function _crc;
    Counters _counters {};

    uint32_t _seq;
    bool _open {false};
    std::size_t _length {0U};
    std::size_t _records {0U};
    /// The payload of the open frame, with room for the CRC
    std::array<uint8_t, frame::MAX_DECODED> _payload {};
    /// Two encoded frames, one may still be going out while the next one is encoded
    std::array<std::array<uint8_t, frame::MAX_ENCODED>, 2> _frames {};
    std::size_t _next_frame {0U};
};

}  // namespace robby
