#include "command_handler.h"
#include "glue/araldite.h"

#include "stm32f7xx_hal.h"

#include <cbor.h>
#include <cobs.h>
#include <cstdint>
#include <cstring>
#include <utility>

extern TIM_HandleTypeDef htim9;

using namespace robby;

namespace {

bool find_text(const CborValue& map, const char* key, const char* text)
{
    CborValue value;
    bool equals {false};
    return cbor_value_map_find_value(&map, key, &value) == CborNoError && cbor_value_is_text_string(&value)
        && cbor_value_text_string_equals(&value, text, &equals) == CborNoError && equals;
}

bool find_int(const CborValue& map, const char* key, int64_t& result)
{
    CborValue value;
    return cbor_value_map_find_value(&map, key, &value) == CborNoError && cbor_value_is_integer(&value)
        && cbor_value_get_int64_checked(&value, &result) == CborNoError;
}

}  // namespace

CommandHandler::CommandHandler(se_oss::Logger log, Link& link) :
    _log{std::move(log)},
    _link{link}
{
    reset_cobs_decoder();
    encode_actions();
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
    if (_queue.tryReceive(_rx_chunk_proc, ANNOUNCE_INTERVAL_MS)) {
        decode_received_bytes(_rx_chunk_proc.bytes.data(), _rx_chunk_proc.length);
    }

    uint32_t now_ticks = xTaskGetTickCount();
    if (now_ticks - _last_announce_tick >= ANNOUNCE_INTERVAL_MS) {
        _last_announce_tick = now_ticks;
        if (_actions_length > 0U) {
            _link.write(_actions.data(), _actions_length);
        }
    }
}

void CommandHandler::reset_cobs_decoder()
{
    _message_length = 0;

    if (cobs_decode_inc_begin(&_cobs_decode_ctx) != COBS_RET_SUCCESS) {
        LOG_WARN(_log, "cobs_decode_inc_begin failed");
    }
}

void CommandHandler::decode_received_bytes(const uint8_t* data, std::size_t length)
{
    std::size_t consumed_total = 0;

    while (consumed_total < length) {
        cobs_decode_inc_args_t args {};
        args.enc_src = data + consumed_total;
        args.dec_dst = _message_buffer.data() + _message_length;
        args.enc_src_max = length - consumed_total;
        args.dec_dst_max = _message_buffer.size() - _message_length;

        std::size_t consumed_now = 0;
        std::size_t decoded_now = 0;
        bool message_complete = false;

        const auto ret = cobs_decode_inc(
            &_cobs_decode_ctx,
            &args,
            &consumed_now,
            &decoded_now,
            &message_complete
        );

        consumed_total += consumed_now;
        _message_length += decoded_now;

        if (ret != COBS_RET_SUCCESS) {
            // LOG_WARN(_log, "COBS decode failed: %d", static_cast<int>(ret));
            reset_cobs_decoder();

            if (consumed_now == 0) {
                consumed_total++;
            }

            continue;
        }

        if (message_complete) {
            handle_request(_message_buffer.data(), _message_length);
            reset_cobs_decoder();
            continue;
        }

        if (_message_length >= _message_buffer.size()) {
            LOG_WARN(_log, "COBS decoded message too large");
            reset_cobs_decoder();
        }

        if (consumed_now == 0 && decoded_now == 0) {
            LOG_WARN(_log, "COBS decoder made no progress");
            reset_cobs_decoder();
            consumed_total++;
        }
    }
}


/* proto: Request::Invoke
   {"Invoke": {"id": 7, "path": "motor/speed", "args": {"speed": 500}}} */
void CommandHandler::handle_request(const uint8_t* message_raw, std::size_t length)
{
    CborParser parser;
    CborValue request;
    CborValue invocation;
    CborValue args;

    if (cbor_parser_init(message_raw, length, 0, &parser, &request) != CborNoError || !cbor_value_is_map(&request)
        || cbor_value_map_find_value(&request, "Invoke", &invocation) != CborNoError
        || !cbor_value_is_map(&invocation)) {
        LOG_WARN(_log, "Unknown request");
        return;
    }

    int64_t id {0};
    if (!find_int(invocation, "id", id) || id < 0 || id > UINT32_MAX) {
        LOG_WARN(_log, "Invocation without id");
        return;
    }

    const char* error {"unknown action"};
    if (find_text(invocation, "path", "motor/speed")) {
        int64_t speed {0};
        const bool valid = cbor_value_map_find_value(&invocation, "args", &args) == CborNoError
            && cbor_value_is_map(&args) && find_int(args, "speed", speed);
        error = valid ? set_motor_speed(speed) : "speed must be an integer";
    }

    send_result(static_cast<uint32_t>(id), error);
}

const char* CommandHandler::set_motor_speed(int64_t speed)
{
    if (speed < 0 || speed > MOTOR_SPEED_MAX) {
        return "speed out of range";
    }

    LOG_INFO(_log, "Motor speed: %u", static_cast<uint32_t>(speed));
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, speed * 54);
    return nullptr;
}

/* proto: Message::ActionResult
   {"ActionResult": {"id": 7, "result": {"Ok": null}}} or {"Err": "reason"} */
void CommandHandler::send_result(uint32_t id, const char* error)
{
    CborEncoder encoder;
    CborEncoder message;
    CborEncoder action_result;
    CborEncoder result;

    cbor_encoder_init(&encoder, _tx_buffer.data(), _tx_buffer.size(), 0);
    cbor_encoder_create_map(&encoder, &message, 1U);
    cbor_encode_text_stringz(&message, "ActionResult");
    cbor_encoder_create_map(&message, &action_result, 2U);

    cbor_encode_text_stringz(&action_result, "id");
    cbor_encode_uint(&action_result, id);

    cbor_encode_text_stringz(&action_result, "result");
    cbor_encoder_create_map(&action_result, &result, 1U);
    if (error == nullptr) {
        cbor_encode_text_stringz(&result, "Ok");
        cbor_encode_null(&result);
    } else {
        cbor_encode_text_stringz(&result, "Err");
        cbor_encode_text_stringz(&result, error);
    }
    cbor_encoder_close_container(&action_result, &result);

    cbor_encoder_close_container(&message, &action_result);
    if (cbor_encoder_close_container(&encoder, &message) != CborNoError) {
        LOG_WARN(_log, "Encoding the action result failed");
        return;
    }

    _link.write(_tx_buffer.data(), cbor_encoder_get_buffer_size(&encoder, _tx_buffer.data()));
}

/* proto: Message::Actions, encoded once as it does not change
   {"Actions": [{"path": "motor/speed", "description": "...",
                 "params": [{"name": "speed", "kind": {"Int": {"min": 0, "max": 1000}}, "unit": "‰"}]}]} */
void CommandHandler::encode_actions()
{
    CborEncoder encoder;
    CborEncoder message;
    CborEncoder actions;
    CborEncoder action;
    CborEncoder params;
    CborEncoder param;
    CborEncoder kind;
    CborEncoder bounds;

    cbor_encoder_init(&encoder, _actions.data(), _actions.size(), 0);
    cbor_encoder_create_map(&encoder, &message, 1U);
    cbor_encode_text_stringz(&message, "Actions");
    cbor_encoder_create_array(&message, &actions, 1U);

    cbor_encoder_create_map(&actions, &action, 3U);
    cbor_encode_text_stringz(&action, "path");
    cbor_encode_text_stringz(&action, "motor/speed");
    cbor_encode_text_stringz(&action, "description");
    cbor_encode_text_stringz(&action, "Duty cycle of the motor PWM");

    cbor_encode_text_stringz(&action, "params");
    cbor_encoder_create_array(&action, &params, 1U);
    cbor_encoder_create_map(&params, &param, 3U);
    cbor_encode_text_stringz(&param, "name");
    cbor_encode_text_stringz(&param, "speed");
    cbor_encode_text_stringz(&param, "kind");
    cbor_encoder_create_map(&param, &kind, 1U);
    cbor_encode_text_stringz(&kind, "Int");
    cbor_encoder_create_map(&kind, &bounds, 2U);
    cbor_encode_text_stringz(&bounds, "min");
    cbor_encode_int(&bounds, 0);
    cbor_encode_text_stringz(&bounds, "max");
    cbor_encode_int(&bounds, MOTOR_SPEED_MAX);
    cbor_encoder_close_container(&kind, &bounds);
    cbor_encoder_close_container(&param, &kind);
    cbor_encode_text_stringz(&param, "unit");
    cbor_encode_text_stringz(&param, "‰");
    cbor_encoder_close_container(&params, &param);
    cbor_encoder_close_container(&action, &params);
    cbor_encoder_close_container(&actions, &action);

    cbor_encoder_close_container(&message, &actions);
    if (cbor_encoder_close_container(&encoder, &message) != CborNoError) {
        LOG_WARN(_log, "Encoding the actions failed");
        return;
    }

    _actions_length = cbor_encoder_get_buffer_size(&encoder, _actions.data());
}
