#include "command_handler.h"
#include "glue/araldite.h"

#include <cstdint>
#include <cstdint>
#include <cstring>


namespace robby {

void CommandHandler::data_received_callback(void* data, std::size_t length)
{ 
    if (data == nullptr || length == 0 || length > MAX_MESSAGE_LENGTH - 1) {
        return;
    }

    _message_buffer[0] = static_cast<uint8_t>(length);
    std::memcpy(_message_buffer.begin() + 1, static_cast<uint8_t*>(data), length);
    puts(data);
    _queue.send(_message_buffer);
}

}  // namespace robby
