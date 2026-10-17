#include "data_publisher.h"

#include <cstdio>
#include <optional>

using namespace robby;

DataPublisher::DataPublisher(
    se_oss::LogRegistry<LogContextId, LogSinkId>& log_registry,
    sonde::Tracer& tracer,
    rtos::MessageQueue<AccelerationFrame>& accelerationFrames,
    DistanceMap& distance_map
    ) :
    _log_registry(log_registry),
    _tracer(tracer),
    _acceleration_frames(accelerationFrames),
    _distance_map(distance_map)
{ }

void DataPublisher::publish()
{
    // IMU measurement streams
    while (_acceleration_frames.tryReceive(_acceleration_frame)) {
        publish_acceleration();
    }

    // Distance map
    if (_distance_map.updated.tryAcquire(0U) && _distance_map.lock.tryLock(100U)) {
        std::ranges::copy(_distance_map.sensors, _sensors.begin());
        _distance_map.lock.unlock();

        publish_distances();
    }

    // Slowly updated vlaues
    uint32_t now_ticks = xTaskGetTickCount();
    if (xTaskGetTickCount() - _last_publish_tick > PUBLISH_INTERVAL_SLOW) {
        _last_publish_tick = now_ticks;

        // Units, repeated for clients connecting later
        _tracer.metadata("imu/acc/x", "g");
        _tracer.metadata("imu/acc/y", "g");
        _tracer.metadata("imu/acc/z", "g");
        _tracer.metadata("tof/distance", "mm");

        update_thread_table();
        publish_threads();
    }

    _log_registry.distributeMessages();
    _tracer.commit();
}

void DataPublisher::publish_acceleration()
{
    const auto& frame = _acceleration_frame;

    for (std::size_t i = 0; i < frame.size; i += ACCELERATION_DECIMATION) {
        const sonde::Timestamp timestamp {frame.timestamp_ns - (frame.size - 1U - i) * ACCELERATION_SAMPLE_PERIOD_NS};
        _tracer.value(timestamp, "imu/acc/x", frame.x[i] * ACCELERATION_SCALE_G);
        _tracer.value(timestamp, "imu/acc/y", frame.y[i] * ACCELERATION_SCALE_G);
        _tracer.value(timestamp, "imu/acc/z", frame.z[i] * ACCELERATION_SCALE_G);
    }
}

void DataPublisher::publish_distances()
{
    // the rows of all sensors one after another, invalid distances as null
    const std::size_t rows {_sensors.size() * _sensors[0].size()};

    _tracer.value("tof/distance", sonde::array(rows, [&](sonde::Array& image) {
        for (const auto& sensor : _sensors) {
            for (const auto& row : sensor) {
                image.add(sonde::array(row.size(), [&](sonde::Array& distances) {
                    for (int16_t distance : row) {
                        distances.add(distance >= 0 ? std::optional<int16_t> {distance} : std::nullopt);
                    }
                }));
            }
        }
    }));
}

void DataPublisher::publish_threads()
{
    for (std::size_t i = 0; i < _thread_table.size; ++i) {
        const auto& thread = _thread_table.threads[i];

        std::array<char, 8 + sizeof(thread.name)> path {};
        std::snprintf(path.data(), path.size(), "tasks/%s", thread.name);
        _tracer.value(path.data(), thread);
    }
}

void DataPublisher::update_thread_table()
{
    uint32_t total_runtime {0U};
    std::size_t task_count = uxTaskGetSystemState(_task_status.data(), _task_status.size(), &total_runtime);

    for (std::size_t i = 0; i < std::min(task_count, _thread_table.threads.size()); ++i) {
        auto& thread = _thread_table.threads[i];
        auto& task_info = _task_status[i];
        std::strncpy(thread.name, task_info.pcTaskName, sizeof(thread.name));
        thread.name[sizeof(thread.name) - 1U] = '\0';
        thread.priority = task_info.uxCurrentPriority;
        // todo: find fix for counter wrap around
        thread.runtime = total_runtime > 0 ? 100.0F * task_info.ulRunTimeCounter / total_runtime : 0.0F;
        thread.stack_size = task_info.pxEndOfStack - task_info.pxStackBase;
        thread.stack_usage = 100.0F * (1.0F - static_cast<float>(task_info.usStackHighWaterMark) / thread.stack_size);
    }
    _thread_table.size = std::min(task_count, _thread_table.threads.size());
}
