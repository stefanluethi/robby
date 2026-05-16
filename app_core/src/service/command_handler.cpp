#include "command_handler.h"
#include "glue/araldite.h"

#include "stm32f7xx_hal.h"

#include <cbor.h>
#include <cstdint>
#include <cstring>
#include <utility>

extern TIM_HandleTypeDef htim9;

using namespace robby;

CommandHandler::CommandHandler(se_oss::Logger log) :
    _log{std::move(log)}
{

}

void CommandHandler::data_received_callback(void* data, std::size_t length)
{
    if (data == nullptr || length == 0 || length > MAX_MESSAGE_LENGTH - 1) {
        return;
    }

    _message_buffer_irq[0] = static_cast<uint8_t>(length);
    std::memcpy(_message_buffer_irq + 1, data, length);
    _queue.send(_message_buffer_irq);
}

void CommandHandler::process()
{
    _queue.receive(_message_buffer_proc);
    try_parse_motor_command(&_message_buffer_proc[1], _message_buffer_proc[0]);
}


/* Message: A1 6D 5365744D6F746F725370656564 F9 52A0
   map(1) { "SetMotorSpeed" => float16(52.0) }        */
bool CommandHandler::try_parse_motor_command(const uint8_t* message_raw, std::size_t length)
{
    // todo: clean up this AI generated sample
    CborParser parser;
    CborValue  map, val;
    CborError  err;

    err = cbor_parser_init(message_raw, length, 0, &parser, &map);
    if (err != CborNoError) {
        LOG_WARN(_log, "cbor_parser_init: %s", cbor_error_string(err));
        return false;
    }

    if (!cbor_value_is_map(&map)) {
        LOG_WARN(_log, "Expected a CBOR map");
        return false;
    }

    err = cbor_value_enter_container(&map, &val);
    if (err != CborNoError) {
        LOG_WARN(_log, "enter container: %s", cbor_error_string(err));
        return false;
    }

    while (!cbor_value_at_end(&val)) {

        /* --- Read the text key --- */
        if (!cbor_value_is_text_string(&val)) {
            LOG_WARN(_log, "Expected a text key");
            return false;
        }

        char   key_buf[64];
        size_t key_len = sizeof(key_buf);
        err = cbor_value_copy_text_string(&val, key_buf, &key_len, &val);
        if (err != CborNoError) {
            LOG_WARN(_log, "copy text: %s", cbor_error_string(err));
            return false;
        }

        if (!cbor_value_is_unsigned_integer(&val)) {
            LOG_WARN(_log, "Expected a uint value for key '%s'", key_buf);
            return false;
        }

        uint64_t speed = 0;
        err = cbor_value_get_uint64(&val, &speed);
        if (err != CborNoError) {
            LOG_WARN(_log, "get uint64: %s", cbor_error_string(err));
            return false;
        }

        LOG_INFO(_log, "Command key: %s", key_buf);
        LOG_INFO(_log, "Command value: %u", (uint32_t)speed);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, speed * 54);

        err = cbor_value_advance(&val);   /* move past the float */
        if (err != CborNoError) {
            LOG_WARN(_log, "advance: %s", cbor_error_string(err));
            return false;
        }
    }

    /* 4. Leave the map */
    err = cbor_value_leave_container(&map, &val);
    if (err != CborNoError) {
        LOG_WARN(_log, "leave container: %s", cbor_error_string(err));
        return false;
    }

    return true;
}