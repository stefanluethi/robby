#pragma once

#include <array>
#include <cstdint>

#include <rtos/rtos.h>

namespace robby {

class CommandHandler
{
public:
    static CommandHandler* instance();
    void dataReceivedCallback(void* data, std::size_t length);

private:
    static constexpr std::size_t MAX_MESSAGE_LENGTH {129};
    static CommandHandler* _instance;

    std::array<uint8_t, MAX_MESSAGE_LENGTH> _message_buffer {};
    rtos::MessageQueue<std::array<uint8_t, MAX_MESSAGE_LENGTH>> _queue {10};

    CommandHandler() = default;
};

}  // namespace robby
