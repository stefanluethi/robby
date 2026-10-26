#include "data_publisher.h"

#include "trace/clock.h"

#include <algorithm>

using namespace robby;

DataPublisher::DataPublisher(
    se_oss::LogRegistry<LogContextId, LogSinkId>& log_registry,
    Link& link,
    rtos::MessageQueue<AccelerationFrame>& accelerationFrames,
    rtos::MessageQueue<fuelgauge_status_t>& battery,
    DistanceMap& distance_map,
    BenchLoad& bench_load
) :
    _log_registry(log_registry),
    _link(link),
    _acceleration_frames(accelerationFrames),
    _battery(battery),
    _distance_map(distance_map),
    _bench_load(bench_load)
{
}

void DataPublisher::publish()
{
    // IMU measurement streams, all samples
    while (_acceleration_frames.tryReceive(_acceleration_frame)) {
        if (_imu_batcher.make(_acceleration_frame, _record)) {
            _link.add(_record);
        }
    }

    // Distance map
    if (_distance_map.updated.tryAcquire(0U) && _distance_map.lock.tryLock(100U)) {
        std::ranges::copy(_distance_map.sensors, _sensors.begin());
        _distance_map.lock.unlock();

        records::make_distance(uptime_ns(), _sensors, _record);
        _link.add(_record);
    }

    // Battery, read by the IMU task about once per second
    if (_battery.tryReceive(_battery_status)) {
        records::make_battery(uptime_ns(), _battery_status, _record);
        _link.add(_record);
    }

    // Slowly updated values
    const uint32_t now_ticks = xTaskGetTickCount();
    if (now_ticks - _last_publish_tick > PUBLISH_INTERVAL_SLOW) {
        _last_publish_tick = now_ticks;

        const uint64_t now_ns {uptime_ns()};
        publish_tasks(now_ns);
        _link.make_stats(now_ns, _record);
        _link.add(_record);
    }

    _log_registry.distributeMessages();

    // Synthetic load of the bench, if it's on
    const uint32_t dropped {_bench_load.generate(uptime_ns(), _record, [this](const robby_link_Record& record) {
        _link.add(record);
    })};
    if (dropped > 0U) {
        _link.count_dropped_records(dropped);
    }

    _link.flush();
}

void DataPublisher::publish_tasks(uint64_t t_ns)
{
    uint32_t total_runtime {0U};
    const std::size_t task_count {uxTaskGetSystemState(_task_status.data(), _task_status.size(), &total_runtime)};

    for (std::size_t i = 0; i < std::min(task_count, _task_status.size()); ++i) {
        const auto& task = _task_status[i];
        const records::TaskInfo info {
            .name = task.pcTaskName,
            .priority = static_cast<uint32_t>(task.uxCurrentPriority),
            .runtime = task.ulRunTimeCounter,
            .total_runtime = total_runtime,
            .stack_size_bytes = static_cast<uint32_t>((task.pxEndOfStack - task.pxStackBase) * sizeof(StackType_t)),
            .stack_free_min_bytes = static_cast<uint32_t>(task.usStackHighWaterMark * sizeof(StackType_t)),
        };
        records::make_task(t_ns, info, _record);
        _link.add(_record);
    }
}
