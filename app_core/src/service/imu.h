#pragma once

#include "lsm6dso.h"

#include <cstddef>
#include <cstdint>

class Imu {
public:
    Imu();
    void process();

private:
    size_t serialize_measurement(uint8_t* buffer, int32_t value);

    LSM6DSO_Object_t _driver{};
    uint8_t _message_buffer_[1024]{};
};
