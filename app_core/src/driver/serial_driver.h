#pragma once

#include <cstddef>
#include <cstdint>

#include <rtos/rtos.h>
#include <stm32f7xx_hal.h>

namespace robby {

/// Sends by DMA if the UART has a TX DMA channel (enable_usart2_tx_dma()), by interrupt
/// otherwise. A write starts the transmission and returns: the CPU is free while the
/// bytes go out. Writes come from one task at a time.
class SerialDriver
{
public:
    explicit SerialDriver(UART_HandleTypeDef& uart);

    /// Waits until the previous transmission is complete, then starts sending `data`,
    /// which must stay valid until the next write returns. Returns false if the UART is
    /// stuck or busy otherwise; the bytes are then dropped.
    bool write(const uint8_t* data, std::size_t length);

    uint32_t baud() const { return _uart.Init.BaudRate; }

    /// Called by the UART interrupt once a transmission is complete
    void transmit_complete();

private:
    /// Time for the previous transmission beyond its bytes, so a timeout means a stuck UART
    static constexpr uint32_t TIMEOUT_MARGIN_MS {5U};

    UART_HandleTypeDef& _uart;
    /// Given while no transmission is under way
    rtos::Semaphore _idle {};
    uint32_t _timeout_ms {TIMEOUT_MARGIN_MS};
};

/// Sends USART2 (the link) by DMA1 stream 6, channel 4. Called once before the scheduler
/// starts. Set up here and not in robby.ioc, so CubeMX doesn't know about it: the stream
/// must stay unused there.
void enable_usart2_tx_dma(UART_HandleTypeDef& uart);

}  // namespace robby
