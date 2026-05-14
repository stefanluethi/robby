#pragma once
#include "driver/serial_driver.h"

namespace robby {

class DataPublisher
{
public:
    explicit DataPublisher(SerialDriver& serial);

private:
    SerialDriver& _serial;
};

}  // namespace robby
