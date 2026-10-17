#pragma once

#include "conf/log.h"
#include "distance_visualizer.h"
#include "imu.h"
#include "trace/thread_table.h"

#include <array>

#include <rtos/rtos.h>
#include <se-oss/log/LogRegistry.h>
#include <sonde/sonde.hpp>

namespace robby {

class DataPublisher
{
public:
    explicit DataPublisher(
        se_oss::LogRegistry<LogContextId, LogSinkId>& log_registry,
        sonde::Tracer& tracer,
        rtos::MessageQueue<AccelerationFrame>& accelerationFrames,
        DistanceMap& distance_map
    );

    void publish();

private:
    static constexpr uint32_t PUBLISH_INTERVAL_SLOW {1000U};
    /// Only every n-th acceleration sample is traced, the link is too slow for all
    static constexpr std::size_t ACCELERATION_DECIMATION {2U};
    static constexpr float ACCELERATION_SCALE_G {0.000061F};

    se_oss::LogRegistry<LogContextId, LogSinkId>& _log_registry;
    sonde::Tracer& _tracer;

    // Message buffers
    rtos::MessageQueue<AccelerationFrame>& _acceleration_frames;
    AccelerationFrame _acceleration_frame {};

    DistanceMap& _distance_map;
    std::array<DistanceSensorImage, 3> _sensors {};

    ThreadTable _thread_table {};
    std::array<TaskStatus_t, 16> _task_status {};

    uint32_t _last_publish_tick {0U};

    void publish_acceleration();
    void publish_distances();
    void publish_threads();
    void update_thread_table();
};

}  // namespace robby
