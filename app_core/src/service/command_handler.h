#pragma once

#include "trace/link.h"

#include <array>
#include <cstdint>

#include <cobs.h>
#include <se-oss/log/Log.h>
#include <rtos/rtos.h>

namespace robby {

/// Carries out the actions blip invokes (proto: Request::Invoke) and announces them
/// (proto: Message::Actions). sonde has no actions yet, so they are encoded here.
class CommandHandler
{
public:
    explicit CommandHandler(se_oss::Logger log, Link& link);
    void data_received_callback(void* data, std::size_t length);
    void process();

private:
    static constexpr std::size_t MAX_MESSAGE_LENGTH {129};
    static constexpr std::size_t MAX_RX_CHUNK_LENGTH {256};
    /// Actions are announced repeatedly for clients connecting later
    static constexpr uint32_t ANNOUNCE_INTERVAL_MS {1000U};

    static constexpr int64_t MOTOR_SPEED_MAX {1000};

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

    std::array<uint8_t, 128> _tx_buffer {};
    std::array<uint8_t, 192> _actions {};
    std::size_t _actions_length {0U};
    uint32_t _last_announce_tick {0U};

    se_oss::Logger _log;
    Link& _link;

    void reset_cobs_decoder();
    void decode_received_bytes(const uint8_t* data, std::size_t length);
    void handle_request(const uint8_t* message_raw, std::size_t length);
    /// Returns why the speed was rejected, or nullptr
    const char* set_motor_speed(int64_t speed);
    void send_result(uint32_t id, const char* error);
    void encode_actions();
};

}  // namespace robby
