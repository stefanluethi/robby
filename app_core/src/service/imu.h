#pragma once

#include "lsm6dso.h"

#include <rtos/rtos.h>

#include <cstddef>
#include <cstdint>

namespace robby {

struct Acceleration
{
    int16_t x;
    int16_t y;
    int16_t z;
};

using AccelerationFrame = std::array<Acceleration, 100>;

class Imu {
public:
    Imu(rtos::MessageQueue<AccelerationFrame>& frames);
    void start();
    void process();

private:
    typedef union {
        int16_t i16bit[3];
        uint8_t u8bit[6];
    } axis3bit16_t;


    LSM6DSO_Object_t _driver {};
    AccelerationFrame _acceleration_frame {};
    rtos::MessageQueue<AccelerationFrame>& _frames;
};

}