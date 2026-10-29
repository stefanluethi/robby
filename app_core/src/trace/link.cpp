#include "link.h"

#include "trace/crc16.h"

#include <cstring>

#include <stm32f7xx_hal.h>

using namespace robby;

namespace {

/// CRC-16/CCITT-FALSE by the CRC unit. Only the link's writer uses it, under its lock.
uint16_t hardware_crc16(const uint8_t* data, std::size_t length)
{
    CRC->CR |= CRC_CR_RESET;

    // four bytes per write, first byte in the most significant bits
    std::size_t i {0U};
    for (; i + 4U <= length; i += 4U) {
        uint32_t word {};
        std::memcpy(&word, data + i, sizeof(word));
        CRC->DR = __REV(word);
    }
    for (; i < length; ++i) {
        *reinterpret_cast<volatile uint8_t*>(&CRC->DR) = data[i];
    }
    return static_cast<uint16_t>(CRC->DR & 0xFFFFU);
}

/// Sets up the CRC unit, true if it computes the CRC of the frames
bool init_hardware_crc()
{
    __HAL_RCC_CRC_CLK_ENABLE();
    CRC->POL = 0x1021U;
    CRC->INIT = 0xFFFFU;
    // 16 bit polynomial, no reversal of input or output
    CRC->CR = CRC_CR_POLYSIZE_0;

    constexpr uint8_t CHECK[] {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    return hardware_crc16(CHECK, sizeof(CHECK)) == 0x29B1U &&
           hardware_crc16(CHECK, sizeof(CHECK)) == crc16_ccitt_false(CHECK, sizeof(CHECK));
}

void init_cycle_counter()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t cycles_per_us()
{
    const uint32_t cycles {SystemCoreClock / 1'000'000U};
    return cycles > 0U ? cycles : 1U;
}

}  // namespace

Link::Link(UART_HandleTypeDef& uart) :
    _serial {uart},
    _hardware_crc {init_hardware_crc()},
    _writer {*this, _hardware_crc ? &hardware_crc16 : &crc16_ccitt_false},
    _reader {&crc16_ccitt_false}
{
    init_cycle_counter();
}

bool Link::add(const robby_link_Record& record)
{
    _lock.lock();
    const bool added {_writer.add(record)};
    _lock.unlock();
    return added;
}

void Link::flush()
{
    _lock.lock();
    _writer.flush();
    _lock.unlock();
}

bool Link::write(const uint8_t* data, std::size_t length)
{
    const uint32_t start {DWT->CYCCNT};
    const bool written {_serial.write(data, length)};
    const uint32_t cycles {DWT->CYCCNT - start};

    const uint32_t per_us {cycles_per_us()};
    _write_cycles += cycles;
    _write_us += _write_cycles / per_us;
    _write_cycles %= per_us;

    const uint32_t us {cycles / per_us};
    if (us > _write_max_us) {
        _write_max_us = us;
    }
    return written;
}

void Link::make_stats(uint64_t t_ns, robby_link_Record& record)
{
    record = robby_link_Record_init_zero;
    record.which_kind = robby_link_Record_link_stats_tag;
    auto& stats = record.kind.link_stats;
    stats.t_ns = t_ns;

    _lock.lock();
    const auto& tx = _writer.counters();
    stats.tx_bytes = tx.tx_bytes;
    stats.tx_frames = tx.tx_frames;
    stats.tx_frames_dropped = tx.tx_frames_dropped;
    stats.records_dropped = tx.records_dropped + _records_dropped.load(std::memory_order_relaxed);
    const uint32_t write_us {_write_us};
    stats.tx_write_max_us = _write_max_us;
    _write_max_us = 0U;
    _lock.unlock();

    const auto& rx = _reader.counters();
    stats.rx_frames = rx.rx_frames.load(std::memory_order_relaxed);
    stats.rx_errors = rx.rx_errors.load(std::memory_order_relaxed);
    stats.rx_seq_gaps = rx.rx_seq_gaps.load(std::memory_order_relaxed);
    stats.baud = _serial.baud();
    _rates.fill(stats, write_us);
}
