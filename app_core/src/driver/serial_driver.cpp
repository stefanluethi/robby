#include "serial_driver.h"

using namespace robby;

SerialDriver::SerialDriver(UART_HandleTypeDef& uart) :
    _uart {uart}
{

}
std::size_t SerialDriver::write(const void* data, std::size_t length)
{
    auto result = HAL_UART_Transmit(&_uart, static_cast<const uint8_t*>(data), length, HAL_MAX_DELAY);
    return result == HAL_OK ? length : 0;
}

std::size_t SerialDriver::read(void* data, std::size_t maxLength)
{
    (void) data;
    (void) maxLength;
    return 0;
}
