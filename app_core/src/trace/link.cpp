#include "link.h"

using namespace robby;

Link::Link(UART_HandleTypeDef& uart) :
    _serial {uart}
{ }

bool Link::write(const uint8_t* data, std::size_t size)
{
    _lock.lock();
    _serial.write(data, size);
    _lock.unlock();
    return true;
}
