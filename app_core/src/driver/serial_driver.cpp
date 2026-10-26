#include "serial_driver.h"

#include <array>

using namespace robby;

namespace {

/// The drivers of the UARTs, for the interrupt to find the driver of its UART
std::array<SerialDriver*, 2> g_drivers {};
std::array<UART_HandleTypeDef*, 2> g_uarts {};

DMA_HandleTypeDef g_usart2_tx_dma {};

/// Interrupt priority of the DMA, the same as the UART's (FreeRTOS calls are allowed)
constexpr uint32_t DMA_IRQ_PRIORITY {6U};

}  // namespace

SerialDriver::SerialDriver(UART_HandleTypeDef& uart) : _uart {uart}
{
    _idle.release();

    for (std::size_t i = 0; i < g_drivers.size(); ++i) {
        if (g_drivers[i] == nullptr) {
            g_uarts[i] = &uart;
            g_drivers[i] = this;
            break;
        }
    }
}

bool SerialDriver::write(const uint8_t* data, std::size_t length)
{
    if (length == 0U || length > UINT16_MAX) {
        return false;
    }

    if (!_idle.tryAcquire(_timeout_ms)) {
        // stuck: start over, this frame is lost
        HAL_UART_AbortTransmit(&_uart);
        _idle.release();
        return false;
    }

    const auto size = static_cast<uint16_t>(length);
    const HAL_StatusTypeDef status {
        _uart.hdmatx != nullptr ? HAL_UART_Transmit_DMA(&_uart, data, size) : HAL_UART_Transmit_IT(&_uart, data, size)
    };
    if (status != HAL_OK) {
        _idle.release();
        return false;
    }

    // the next write waits at most for these bytes, 10 bits each
    const uint32_t baud {_uart.Init.BaudRate > 0U ? _uart.Init.BaudRate : 1U};
    _timeout_ms = static_cast<uint32_t>(uint64_t {length} * 10U * 1000U / baud) + TIMEOUT_MARGIN_MS;
    return true;
}

void SerialDriver::transmit_complete() { _idle.release(); }

void robby::enable_usart2_tx_dma(UART_HandleTypeDef& uart)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    g_usart2_tx_dma.Instance = DMA1_Stream6;
    g_usart2_tx_dma.Init.Channel = DMA_CHANNEL_4;
    g_usart2_tx_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    g_usart2_tx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    g_usart2_tx_dma.Init.MemInc = DMA_MINC_ENABLE;
    g_usart2_tx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_usart2_tx_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_usart2_tx_dma.Init.Mode = DMA_NORMAL;
    g_usart2_tx_dma.Init.Priority = DMA_PRIORITY_LOW;
    g_usart2_tx_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&g_usart2_tx_dma) != HAL_OK) {
        // stays on interrupts
        return;
    }
    __HAL_LINKDMA(&uart, hdmatx, g_usart2_tx_dma);

    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, DMA_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
}

extern "C" void DMA1_Stream6_IRQHandler(void) { HAL_DMA_IRQHandler(&g_usart2_tx_dma); }

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    for (std::size_t i = 0; i < g_uarts.size(); ++i) {
        if (g_uarts[i] == huart) {
            g_drivers[i]->transmit_complete();
            return;
        }
    }
}
