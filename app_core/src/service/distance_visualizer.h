#pragma once

#include "se-oss/log/Log.h"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <array>

#include <rtos/rtos.h>

extern "C" {
#include <vl53l8cx_api.h>
}

namespace robby {

using DistanceSensorImage = std::array<std::array<int16_t, 8>, 8>;
struct DistanceMap
{
    std::array<DistanceSensorImage, 3> sensors {};

    rtos::Semaphore updated {};
    rtos::Mutex lock {};
};

class DistanceVisualizer {
public:
    explicit DistanceVisualizer(se_oss::Logger log, DistanceMap& distance_map) : _log {std::move(log)}, _distance_map {distance_map} {}
    void start();
    void process();
    void conversion_done_callback();

private:
    enum class DistanceSensor : uint8_t {
        RIGHT = 0U,
        MIDDLE = 1U,
        LEFT = 2U,
    };

    static constexpr uint32_t CONF_POLLING_PERIOD = 1U;
    static constexpr uint32_t CONF_SENSOR_FREQUENCY_HZ = 10U;
    static constexpr uint32_t CONF_N_SENSORS = 3U;
    static constexpr uint32_t SENSOR_INTEGRATION_TIME_MS = 10U;

    static constexpr float MAX_DISTANCE_MM = 4000.0F;
    static constexpr uint32_t SENSOR_RESOLUTION = VL53L8CX_RESOLUTION_8X8;
    static constexpr uint32_t SENSOR_RESOLUTION_X = 8U;
    static constexpr uint32_t SENSOR_RESOLUTION_Y = 8U;

    bool setup_sensor(VL53L8CX_Configuration* device);
    void draw_results();
    void trigger_sensor(DistanceSensor sensor);
    size_t serialize_distmap(uint8_t* buffer, const VL53L8CX_ResultsData* sensor_results);

    VL53L8CX_Configuration _devices[CONF_N_SENSORS]{};
    VL53L8CX_ResultsData _results[CONF_N_SENSORS]{};
    bool _data_ready = false;
    se_oss::Logger _log;
    DistanceMap& _distance_map;
};
}
