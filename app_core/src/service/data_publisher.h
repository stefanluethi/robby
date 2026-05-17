#pragma once

#include "distance_visualizer.h"
#include "driver/serial_driver.h"
#include "imu.h"

#include <array>

#include <cobs.h>
#include <rtos/rtos.h>

namespace robby {

enum class StreamId : uint8_t
{
    ACCELERATION_X = 0,
    ACCELERATION_Y = 1,
    ACCELERATION_Z = 2,
};

constexpr uint8_t to_uint(StreamId stream)
{
    return static_cast<uint8_t>(stream);
}

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
    // Encoding buffers
    SerialDriver& _serial;
    std::array<uint8_t, 1024> _cbor_buffer {};
    std::array<uint8_t, COBS_ENCODE_MAX(1024)> _cobs_buffer {};

    // Message buffers
    rtos::MessageQueue<AccelerationFrame>& _acceleration_frames;
    AccelerationFrame _acceleration_frame {};
    uint32_t _acceleration_sequence_counter {0U};

    DistanceMap& _distance_map;
    std::array<DistanceSensorImage, 3> _sensors {};

    void encode_and_write(std::size_t length);
};

}  // namespace robby
