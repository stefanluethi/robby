#pragma once

#include <array>
#include <cstdint>

#include <cobs.h>
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
    static constexpr std::size_t MAX_RX_CHUNK_LENGTH {256};

    struct RxChunk {
        std::size_t length {};
        std::array<uint8_t, MAX_RX_CHUNK_LENGTH> bytes {};
    };

    RxChunk _rx_chunk_irq {};
    RxChunk _rx_chunk_proc {};
    rtos::MessageQueue<RxChunk> _queue {10};

    cobs_decode_inc_ctx_t _cobs_decode_ctx {};
    std::array<uint8_t, MAX_MESSAGE_LENGTH> _message_buffer {};
    std::size_t _message_length {};

    se_oss::Logger _log;

    void reset_cobs_decoder();
    void decode_received_bytes(const uint8_t* data, std::size_t length);
    bool try_parse_motor_command(const uint8_t* message_raw, std::size_t length);

};

}  // namespace robby