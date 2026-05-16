#pragma once

#include "distance_visualizer.h"
#include "driver/serial_driver.h"
#include "imu.h"

#include <array>

#include <cobs.h>
#include <rtos/rtos.h>

namespace robby {

class DataPublisher
{
public:
    explicit DataPublisher(
        SerialDriver& serial,
        rtos::MessageQueue<AccelerationFrame>& accelerationFrames,
        DistanceMap& distance_map
    );

    void publish();

private:
    SerialDriver& _serial;
    std::array<uint8_t, 1024> _cbor_buffer {};
    std::array<uint8_t, COBS_ENCODE_MAX(1024)> _cobs_buffer {};

    rtos::MessageQueue<AccelerationFrame>& _acceleration_frames;
    AccelerationFrame _acceleration_frame {};
    DistanceMap& _distance_map;

    void encode_and_write(std::size_t length);
};

}  // namespace robby
