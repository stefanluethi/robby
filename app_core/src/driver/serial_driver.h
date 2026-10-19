#pragma once

#include <cstddef>
#include <stm32f7xx_hal.h>

#include <rtos/rtos.h>

namespace robby {

/// Sends by interrupt and blocks the calling task until the bytes are out, so that the CPU
/// is free for other tasks while a write is under way. Writes come from one task at a time.
class SerialDriver
{
public:
    explicit SerialDriver(UART_HandleTypeDef& uart);

    std::size_t write(const void* data, std::size_t length);
    std::size_t read(void* data, std::size_t maxLength);

    /// Called by the UART interrupt once a transmission is complete
    void transmit_complete();

private:
    /// Generous for a byte at the 1 Mbaud of the link, so a timeout means a stuck UART
    static constexpr uint32_t TIMEOUT_MS_PER_KB {50U};

    UART_HandleTypeDef& _uart;
    rtos::Semaphore _transmitted {};
};

}  // namespace robby
