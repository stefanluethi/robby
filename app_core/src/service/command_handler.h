#pragma once

#include <array>
#include <cstdint>

#include <se-oss/log/Log.h>
#include <rtos/rtos.h>

namespace robby {

class CommandHandler
{
public:
    explicit CommandHandler(se_oss::Logger log);
    void data_received_callback(void* data, std::size_t length);
    void process();

private:
    static constexpr std::size_t MAX_MESSAGE_LENGTH {129};

    uint8_t _message_buffer_irq[MAX_MESSAGE_LENGTH] {};
    uint8_t _message_buffer_proc[MAX_MESSAGE_LENGTH] {};
    rtos::MessageQueue<uint8_t[MAX_MESSAGE_LENGTH]> _queue {10};
    se_oss::Logger _log;

    bool try_parse_motor_command(const uint8_t* message_raw, std::size_t length);

};

}  // namespace robby
