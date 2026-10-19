#include "serial_driver.h"

#include <array>

using namespace robby;

namespace {

/// The drivers of the UARTs, for the interrupt to find the driver of its UART
std::array<SerialDriver*, 2> g_drivers {};
std::array<UART_HandleTypeDef*, 2> g_uarts {};

}  // namespace

SerialDriver::SerialDriver(UART_HandleTypeDef& uart) :
    _uart {uart}
{
    for (std::size_t i = 0; i < g_drivers.size(); ++i) {
        if (g_drivers[i] == nullptr) {
            g_uarts[i] = &uart;
            g_drivers[i] = this;
            break;
        }
    }
}

std::size_t SerialDriver::write(const void* data, std::size_t length)
{
    if (length == 0U || length > UINT16_MAX) {
        return 0;
    }

    // a release left over from a transmission that timed out
    _transmitted.tryAcquire(0U);

    if (HAL_UART_Transmit_IT(&_uart, static_cast<const uint8_t*>(data), static_cast<uint16_t>(length)) != HAL_OK) {
        return 0;
    }

    const uint32_t timeout_ms = (length / 1024U + 1U) * TIMEOUT_MS_PER_KB;
    if (!_transmitted.tryAcquire(timeout_ms)) {
        HAL_UART_AbortTransmit(&_uart);
        return 0;
    }
    return length;
}

std::size_t SerialDriver::read(void* data, std::size_t maxLength)
{
    (void) data;
    (void) maxLength;
    return 0;
}

void SerialDriver::transmit_complete()
{
    _transmitted.release();
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    for (std::size_t i = 0; i < g_uarts.size(); ++i) {
        if (g_uarts[i] == huart) {
            g_drivers[i]->transmit_complete();
            return;
        }
    }
}
