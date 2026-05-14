#pragma once

#include <cstddef>
#include <stm32f7xx_hal.h>

namespace robby {

class SerialDriver
{
public:
    explicit SerialDriver(UART_HandleTypeDef& uart);

    std::size_t write(const void* data, std::size_t length);
    std::size_t read(void* data, std::size_t maxLength);
private:
    UART_HandleTypeDef& _uart;
};

}  // namespace robby
