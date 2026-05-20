#include "data_publisher.h"

#include "proto/serializer.h"

using namespace robby;

DataPublisher::DataPublisher(
    se_oss::LogRegistry<LogContextId, LogSinkId>& log_registry,
    util::CobsEnc<SerialDriver>& serial,
    rtos::MessageQueue<AccelerationFrame>& accelerationFrames,
    DistanceMap& distance_map
    ) :
    _log_registry(log_registry),
    _serial(serial),
    _acceleration_frames(accelerationFrames),
    _distance_map(distance_map)
{ }

void DataPublisher::publish()
{
    // IMU measurement streams
    while (_acceleration_frames.tryReceive(_acceleration_frame)) {
        auto length = serialize(
            _cbor_buffer.data(),
            _cbor_buffer.size(),
            _acceleration_frame.x.data(),
            _acceleration_frame.x.size(),
            to_uint(StreamId::ACCELERATION_X),
            _acceleration_sequence_counter
        );
        _serial.write(_cbor_buffer.data(), length);

        length = serialize(
            _cbor_buffer.data(),
            _cbor_buffer.size(),
            _acceleration_frame.y.data(),
            _acceleration_frame.y.size(),
            to_uint(StreamId::ACCELERATION_Y),
            _acceleration_sequence_counter
        );
        _serial.write(_cbor_buffer.data(), length);

        length = serialize(
            _cbor_buffer.data(),
            _cbor_buffer.size(),
            _acceleration_frame.z.data(),
            _acceleration_frame.z.size(),
            to_uint(StreamId::ACCELERATION_Z),
            _acceleration_sequence_counter
        );
        _serial.write(_cbor_buffer.data(), length);

        _acceleration_sequence_counter++;
    }

    // Distance map
    if (_distance_map.updated.tryAcquire(0U) && _distance_map.lock.tryLock(100U)) {
        std::ranges::copy(_distance_map.sensors, _sensors.begin());
        _distance_map.lock.unlock();

        auto length = serialize(_cbor_buffer.data(), _cbor_buffer.size(), _sensors);
        _serial.write(_cbor_buffer.data(), length);
    }

    // Slowly updated vlaues
    uint32_t now_ticks = xTaskGetTickCount();
    if (xTaskGetTickCount() - _last_publish_tick > PUBLISH_INTERVAL_SLOW) {
        _last_publish_tick = now_ticks;

        // Send stream descriptors
        for (const auto& stream_descriptor : _stream_descriptors) {
            auto length = serialize(_cbor_buffer.data(), _cbor_buffer.size(), stream_descriptor);
        _serial.write(_cbor_buffer.data(), length);
        }

        update_thread_table();
        auto length = serialize(_cbor_buffer.data(), _cbor_buffer.size(), _thread_table);
        _serial.write(_cbor_buffer.data(), length);
    }

    _log_registry.distributeMessages();
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

