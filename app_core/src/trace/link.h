#pragma once

#include "driver/serial_driver.h"
#include "trace/downlink_reader.h"
#include "trace/uplink_writer.h"

#include <atomic>
#include <cstdint>

#include <robby_link.pb.h>
#include <rtos/rtos.h>

namespace robby {

/// The protobuf link to the ESP32 outpost (proto/README.md). Uplink: records are packed
/// into frames with `seq`, CRC-16 and COBS and go out by DMA. Downlink: frames from the
/// outpost are checked and decoded. Counts both directions for LinkStats.
///
/// add() and flush() are safe from several tasks; receive() is for one task only.
class Link final : private ITransport
{
public:
    explicit Link(UART_HandleTypeDef& uart);

    /// Adds a record to the open frame (sending it first if the record doesn't fit).
    /// Returns false if the record was dropped.
    bool add(const robby_link_Record& record);

    /// Sends the open frame, if it has any records
    void flush();

    /// For records dropped before they reach the link, e.g. because a queue was full
    void count_dropped_records(uint32_t count) { _records_dropped.fetch_add(count, std::memory_order_relaxed); }
    std::atomic<uint32_t>& dropped_records() { return _records_dropped; }

    /// Takes the next received byte. Returns true when it completed a Downlink, which is
    /// then in downlink().
    bool receive(uint8_t byte) { return _reader.push(byte); }
    const robby_link_Downlink& downlink() const { return _reader.downlink(); }

    /// Fills the LinkStats record and starts the next interval of tx_write_max_us
    void make_stats(uint64_t t_ns, robby_link_Record& record);

    /// Whether the CRC unit computes the CRC, checked at start (else a table does)
    bool hardware_crc() const { return _hardware_crc; }

private:
    // ITransport implementation, called by _writer under _lock
    bool write(const uint8_t* data, std::size_t length) override;

    SerialDriver _serial;
    bool _hardware_crc;
    UplinkWriter _writer;
    DownlinkReader _reader;
    rtos::Mutex _lock {};

    std::atomic<uint32_t> _records_dropped {0U};
    uint32_t _write_cycles {0U};
    uint32_t _write_us {0U};
    uint32_t _write_max_us {0U};
};

}  // namespace robby
