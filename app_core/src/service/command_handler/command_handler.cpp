#include "command_handler.h"

#include "command_handler_glue.h"
#include <cstdint>
#include <cstdint>
#include <cstring>

extern "C" void CMD_DataReceivedCallback(void* data, size_t length)
{
    robby::CommandHandler::instance()->dataReceivedCallback(data, length);
}

namespace robby {

CommandHandler* CommandHandler::_instance {nullptr};

CommandHandler* CommandHandler::instance() 
{ 
    if (_instance == nullptr) {
        _instance = new CommandHandler();
    }
    return _instance;
}

void CommandHandler::dataReceivedCallback(void* data, std::size_t length)
{ 
    if (data == nullptr || length == 0 || length > MAX_MESSAGE_LENGTH - 1) {
        return;
    }

    _message_buffer[0] = static_cast<uint8_t>(length);
    std::memcpy(_message_buffer.begin() + 1, static_cast<uint8_t*>(data), length);
    _queue.send(_message_buffer);
}

}  // namespace robby
