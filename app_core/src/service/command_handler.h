#pragma once

#include "service/bench_load.h"
#include "trace/link.h"

#include <array>
#include <cstdint>

#include <robby_link.pb.h>
#include <rtos/rtos.h>
#include <se-oss/log/Log.h>

namespace robby {

/// Carries out the actions blip invokes: the outpost sends them as Downlink frames with a
/// typed Invoke, each one is answered with an ActionResult of its id. The outpost owns the
/// list of actions and announces it to blip.
class CommandHandler
{
public:
    CommandHandler(se_oss::Logger log, Link& link, BenchLoad& bench_load);
    /// Called by the UART interrupt with the received bytes
    void data_received_callback(void* data, std::size_t length);
    void process();

private:
    static constexpr std::size_t MAX_RX_CHUNK_LENGTH {256};
    static constexpr uint32_t MOTOR_SPEED_MAX {1000U};

    struct RxChunk
    {
        std::size_t length {};
        std::array<uint8_t, MAX_RX_CHUNK_LENGTH> bytes {};
    };

    RxChunk _rx_chunk_irq {};
    RxChunk _rx_chunk_proc {};
    rtos::MessageQueue<RxChunk> _queue {10};

    robby_link_Record _record {};

    se_oss::Logger _log;
    Link& _link;
    BenchLoad& _bench_load;

    void handle(const robby_link_Downlink& downlink);
    /// Returns why the action was rejected, or nullptr
    const char* invoke(const robby_link_Invoke& invoke);
    const char* set_motor_speed(uint32_t speed);
    const char* set_bench_load(uint32_t rate, uint32_t batch);
};

}  // namespace robby
