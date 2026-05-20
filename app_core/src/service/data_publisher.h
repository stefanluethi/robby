#pragma once

#include "conf/log.h"
#include "distance_visualizer.h"
#include "driver/serial_driver.h"
#include "imu.h"
#include "proto/stream.h"
#include "proto/thread_table.h"
#include "util/cobs_enc.h"

#include <array>

#include <cobs.h>
#include <rtos/rtos.h>
#include <se-oss/log/LogRegistry.h>

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
        se_oss::LogRegistry<LogContextId, LogSinkId>& log_registry,
        util::CobsEnc<SerialDriver>& serial,
        rtos::MessageQueue<AccelerationFrame>& accelerationFrames,
        DistanceMap& distance_map
    );

    void publish();

private:
    static constexpr uint32_t PUBLISH_INTERVAL_SLOW {1000U};

    se_oss::LogRegistry<LogContextId, LogSinkId>& _log_registry;

    // Encoding buffers
    util::CobsEnc<SerialDriver>& _serial;
    std::array<uint8_t, 2048> _cbor_buffer {};

    // Message buffers
    rtos::MessageQueue<AccelerationFrame>& _acceleration_frames;
    AccelerationFrame _acceleration_frame {};
    uint16_t _acceleration_sequence_counter {0U};

    DistanceMap& _distance_map;
    std::array<DistanceSensorImage, 3> _sensors {};

    ThreadTable _thread_table {};
    std::array<TaskStatus_t, 16> _task_status {};

    uint32_t _last_publish_tick {0U};

    void update_thread_table();

    static constexpr std::array<StreamDescriptor, 3> _stream_descriptors = {
        StreamDescriptor {
            .id = to_uint(StreamId::ACCELERATION_X),
            .name = "acc::x",
            .scale_factor = 0.000061F,
            .unit = "g",
            .sampling_time = 1.0F / 833.0F,
        },
        StreamDescriptor {
            .id = to_uint(StreamId::ACCELERATION_Y),
            .name = "acc::y",
            .scale_factor = 0.000061F,
            .unit = "g",
            .sampling_time = 1.0F / 833.0F,
        },
        StreamDescriptor {
            .id = to_uint(StreamId::ACCELERATION_Z),
            .name = "acc::z",
            .scale_factor = 0.000061F,
            .unit = "g",
            .sampling_time = 1.0F / 833.0F,
        },
    };
};

}  // namespace robby
