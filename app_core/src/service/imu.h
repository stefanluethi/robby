#pragma once

#include "driver/fuelgauge.h"
#include "lsm6dso.h"

#include <rtos/rtos.h>
#include <se-oss/log/Log.h>

#include <array>
#include <cstddef>
#include <cstdint>

namespace robby {

constexpr uint64_t ACCELERATION_SAMPLE_PERIOD_NS {1'000'000'000U / 833U};

template<std::size_t N>
struct Acceleration
{
    /// Uptime when the last sample was read
    uint64_t timestamp_ns {};
    /// Number of valid samples
    std::size_t size {};
    std::array<int16_t, N> x {};
    std::array<int16_t, N> y {};
    std::array<int16_t, N> z {};
};

using AccelerationFrame = Acceleration<100>;

/// Reads the IMU, and the battery gauge as it shares I2C2: one task owns the bus, so that
/// the speed of the bus can be switched for the gauge between two IMU transfers.
class Imu {
public:
    Imu(
        se_oss::Logger log,
        rtos::MessageQueue<AccelerationFrame>& frames,
        rtos::MessageQueue<fuelgauge_status_t>& battery
    );
    void start();
    void process();

private:
    // tag byte followed by the three axes
    static constexpr std::size_t FIFO_WORD_SIZE {7U};
    // samples arrive every 1.2ms, so a gap this long means the FIFO stopped filling
    static constexpr uint32_t STALL_TIMEOUT_MS {300U};
    static constexpr uint32_t RESET_POLL_LIMIT {100U};
    // the FIFO holds ~500 ms of samples, far more than the few ms a gauge access takes
    static constexpr uint32_t BATTERY_INTERVAL_MS {1000U};

    void disable_gauge_broadcast();
    bool configure();
    void read_fifo(uint16_t num_words);
    void read_battery();

    se_oss::Logger _log;
    LSM6DSO_Object_t _driver {};
    AccelerationFrame _acceleration_frame {};
    std::size_t _num_samples {0U};
    uint32_t _last_sample_tick {0U};
    rtos::MessageQueue<AccelerationFrame>& _frames;
    rtos::MessageQueue<fuelgauge_status_t>& _battery;
    uint32_t _last_battery_tick {0U};
    bool _battery_failed {false};
};

}
