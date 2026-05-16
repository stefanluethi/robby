#pragma once

#include "service/distance_visualizer.h"
#include "service/imu.h"

namespace robby {

std::size_t serialize(uint8_t* buffer, std::size_t buffer_size, const AccelerationFrame& frame);
std::size_t serialize(uint8_t* buffer, std::size_t buffer_size, const DistanceMap& distances);

}  // namespace robby
