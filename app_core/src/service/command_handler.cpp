#include "command_handler.h"

#include "stm32f7xx_hal.h"
#include "trace/records.h"

#include <cstring>
#include <utility>

extern TIM_HandleTypeDef htim9;

using namespace robby;

CommandHandler::CommandHandler(se_oss::Logger log, Link& link, BenchLoad& bench_load) :
    _log {std::move(log)},
    _link {link},
    _bench_load {bench_load}
{
}

void CommandHandler::data_received_callback(void* data, std::size_t length)
{
    if (data == nullptr || length == 0 || length > MAX_RX_CHUNK_LENGTH) {
        return;
    }

    _rx_chunk_irq.length = length;
    std::memcpy(_rx_chunk_irq.bytes.data(), data, length);
    _queue.trySend(_rx_chunk_irq);
}

void CommandHandler::process()
{
    if (!_queue.receive(_rx_chunk_proc)) {
        return;
    }

    for (std::size_t i = 0; i < _rx_chunk_proc.length; ++i) {
        if (_link.receive(_rx_chunk_proc.bytes[i])) {
            handle(_link.downlink());
        }
    }
}

void CommandHandler::handle(const robby_link_Downlink& downlink)
{
    if (downlink.which_kind != robby_link_Downlink_invoke_tag) {
        // a kind added after this firmware
        LOG_WARN(_log, "Unknown downlink (%u)", static_cast<unsigned>(downlink.which_kind));
        return;
    }

    const auto& request = downlink.kind.invoke;
    records::make_action_result(request.id, invoke(request), _record);
    _link.add(_record);
    _link.flush();
}

const char* CommandHandler::invoke(const robby_link_Invoke& invoke)
{
    switch (invoke.which_args) {
        case robby_link_Invoke_motor_speed_tag: return set_motor_speed(invoke.args.motor_speed.speed);
        case robby_link_Invoke_bench_load_tag:
            return set_bench_load(invoke.args.bench_load.rate, invoke.args.bench_load.batch);
        default: return "unknown action";
    }
}

const char* CommandHandler::set_motor_speed(uint32_t speed)
{
    if (speed > MOTOR_SPEED_MAX) {
        return "speed out of range";
    }

    LOG_INFO(_log, "Motor speed: %u", static_cast<unsigned>(speed));
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, speed * 54U);
    return nullptr;
}

const char* CommandHandler::set_bench_load(uint32_t rate, uint32_t batch)
{
    const char* error {_bench_load.configure(rate, batch)};
    if (error == nullptr) {
        LOG_INFO(
            _log,
            "Bench load: %u samples/s, batches of %u",
            static_cast<unsigned>(rate),
            static_cast<unsigned>(batch)
        );
    }
    return error;
}
