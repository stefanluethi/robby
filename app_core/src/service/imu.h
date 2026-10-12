#pragma once

#include "lsm6dso.h"

#include <rtos/rtos.h>
#include <se-oss/log/Log.h>

#include <array>
#include <cstddef>
#include <cstdint>

namespace robby {

template<std::size_t N>
struct Acceleration
{
    std::array<int16_t, N> x {};
    std::array<int16_t, N> y {};
    std::array<int16_t, N> z {};
};

using AccelerationFrame = Acceleration<100>;

class Imu {
public:
    Imu(se_oss::Logger log, rtos::MessageQueue<AccelerationFrame>& frames);
    void start();
    void process();

private:
    // tag byte followed by the three axes
    static constexpr std::size_t FIFO_WORD_SIZE {7U};
    // samples arrive every 1.2ms, so a gap this long means the FIFO stopped filling
    static constexpr uint32_t STALL_TIMEOUT_MS {300U};
    static constexpr uint32_t RESET_POLL_LIMIT {100U};

    bool configure();
    void read_fifo(uint16_t num_words);

    se_oss::Logger _log;
    LSM6DSO_Object_t _driver {};
    AccelerationFrame _acceleration_frame {};
    std::size_t _num_samples {0U};
    uint32_t _last_sample_tick {0U};
    rtos::MessageQueue<AccelerationFrame>& _frames;
};

}
