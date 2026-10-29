#pragma once

#include "driver/fuelgauge.h"
#include "lsm6dso.h"
#include "service/imu_odometry.h"
#include "service/samples.h"
#include "trace/link.h"

#include <rtos/rtos.h>
#include <se-oss/log/Log.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace robby {

/// Reads the IMU, and the battery gauge as it shares I2C2: one task owns the bus, so that
/// the speed of the bus can be switched for the gauge between two IMU transfers. Tracks the
/// position of the robot from the accelerometer and the gyro alone (ImuOdometry) and sends
/// it as an ImuPose record after each read of the FIFO, at most every 20 ms (about 35 Hz).
class Imu {
public:
    Imu(
        se_oss::Logger log,
        rtos::MessageQueue<AccelerationFrame>& frames,
        rtos::MessageQueue<fuelgauge_status_t>& battery,
        std::atomic<uint32_t>& dropped_frames,
        Link& link
    );
    void start();
    void process();

private:
    // tag byte followed by the three axes
    static constexpr std::size_t FIFO_WORD_SIZE {7U};
    // the gyro is batched at 208 Hz, a quarter of the accelerometer's 833 Hz
    static constexpr uint32_t ACC_SAMPLES_PER_GYRO_SAMPLE {4U};
    static constexpr uint32_t POSE_INTERVAL_MS {20U};
    // samples arrive every 1.2ms, so a gap this long means the FIFO stopped filling
    static constexpr uint32_t STALL_TIMEOUT_MS {300U};
    static constexpr uint32_t RESET_POLL_LIMIT {100U};
    // the FIFO holds ~500 ms of samples, far more than the few ms a gauge access takes
    static constexpr uint32_t BATTERY_INTERVAL_MS {1000U};

    void disable_gauge_broadcast();
    bool configure();
    void read_fifo(uint16_t num_words);
    /// Learns the actual sample period (about 806 Hz instead of 833 Hz) from the samples
    /// read since the last read
    void measure_sample_period(uint32_t acc_samples);
    void publish_pose();
    void read_battery();

    se_oss::Logger _log;
    LSM6DSO_Object_t _driver {};
    AccelerationFrame _acceleration_frame {};
    std::size_t _num_samples {0U};
    uint32_t _last_sample_tick {0U};
    rtos::MessageQueue<AccelerationFrame>& _frames;
    rtos::MessageQueue<fuelgauge_status_t>& _battery;
    /// Frames lost because the queue was full, counted by the link
    std::atomic<uint32_t>& _dropped_frames;
    uint32_t _last_battery_tick {0U};
    bool _battery_failed {false};

    Link& _link;
    inertial::ImuOdometry _odometry {};
    float _acc_period_s {static_cast<float>(ACCELERATION_SAMPLE_PERIOD_NS) * 1e-9F};
    /// When samples were last read, 0 after a gap
    uint64_t _last_read_ns {0U};
    uint32_t _last_pose_tick {0U};
    bool _pose_ready {false};
    robby_link_Record _record {};
};

}
