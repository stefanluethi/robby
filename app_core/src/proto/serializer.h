#pragma once

#include "service/distance_visualizer.h"
#include "service/imu.h"

namespace robby {

std::size_t serialize(uint8_t* buffer, std::size_t buffer_size, const int16_t* array, std::size_t length, uint8_t stream_id, uint32_t sequence_number);
std::size_t serialize(uint8_t* buffer, std::size_t buffer_size, const std::array<DistanceSensorImage, 3>& sensors);

}  // namespace robby
