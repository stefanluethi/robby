#pragma once

#include "driver/serial_driver.h"
#include "util/cobs_enc.h"

#include <rtos/rtos.h>
#include <sonde/sonde.hpp>

namespace robby {

/// Serial link to blip (via the ESP32 bridge). Each write goes out as one COBS frame,
/// which the bridge forwards as one WebSocket message. Safe to write from several tasks.
class Link final : public sonde::Sink
{
public:
    explicit Link(UART_HandleTypeDef& uart);

    /// Writes the batches of a tracer, or hand-encoded messages like the actions.
    bool write(const uint8_t* data, std::size_t size) override;

private:
    util::CobsEnc<SerialDriver> _serial;
    rtos::Mutex _lock {};
};

}  // namespace robby
