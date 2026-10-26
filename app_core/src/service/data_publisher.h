#pragma once

#include "conf/log.h"
#include "distance_visualizer.h"
#include "imu.h"
#include "service/bench_load.h"
#include "trace/link.h"
#include "trace/records.h"

#include <array>

#include <robby_link.pb.h>
#include <rtos/rtos.h>
#include <se-oss/log/LogRegistry.h>

namespace robby {

/// Sends the measurements, the logs and the status of the robot as records over the link,
/// the outpost turns them into blip traces (proto/README.md).
class DataPublisher
{
public:
    explicit DataPublisher(
        se_oss::LogRegistry<LogContextId, LogSinkId>& log_registry,
        Link& link,
        rtos::MessageQueue<AccelerationFrame>& accelerationFrames,
        rtos::MessageQueue<fuelgauge_status_t>& battery,
        DistanceMap& distance_map,
        BenchLoad& bench_load
    );

    void publish();

private:
    static constexpr uint32_t PUBLISH_INTERVAL_SLOW {1000U};
    static constexpr std::size_t MAX_TASKS {16U};

    se_oss::LogRegistry<LogContextId, LogSinkId>& _log_registry;
    Link& _link;
    /// Filled for one record after the other
    robby_link_Record _record {};

    // Message buffers
    rtos::MessageQueue<AccelerationFrame>& _acceleration_frames;
    AccelerationFrame _acceleration_frame {};
    records::ImuBatcher _imu_batcher {};

    rtos::MessageQueue<fuelgauge_status_t>& _battery;
    fuelgauge_status_t _battery_status {};

    DistanceMap& _distance_map;
    std::array<DistanceSensorImage, 3> _sensors {};

    BenchLoad& _bench_load;

    std::array<TaskStatus_t, MAX_TASKS> _task_status {};

    uint32_t _last_publish_tick {0U};

    void publish_tasks(uint64_t t_ns);
};

}  // namespace robby
