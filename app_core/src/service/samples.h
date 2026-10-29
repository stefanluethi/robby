#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace robby {

/// The nominal sample period of the accelerometer (833 Hz)
constexpr uint64_t ACCELERATION_SAMPLE_PERIOD_NS {1'000'000'000U / 833U};

template<std::size_t N>
struct Acceleration
{
    /// Uptime when the last sample was read
    uint64_t timestamp_ns {};
    /// Number of valid samples
    std::size_t size {};
    std::array<int16_t, N> x {};
    std::array<int16_t, N> y {};
    std::array<int16_t, N> z {};
};

using AccelerationFrame = Acceleration<100>;

/// Distances of one VL53L8CX in mm, [y][x] (row-major) with the zone x + 8y, negative where invalid
using DistanceSensorImage = std::array<std::array<int16_t, 8>, 8>;

}  // namespace robby
