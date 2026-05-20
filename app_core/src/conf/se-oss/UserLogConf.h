#pragma once

#include "se-oss/log/buffer/AtomicBuffer.h"

#include <cbor.h>

namespace se_oss {

class RobbyFormatter
{
public:
    template<typename TFormat, typename... Values>
    static size_t
    format(void* buffer, std::size_t bufferSize, const LogRecord& record, TFormat formatString, const Values&... values)
    {
        auto* byteBuffer = static_cast<uint8_t*>(buffer);
        CborEncoder encoder;
        CborEncoder root_map;
        CborEncoder payload_map;
        CborEncoder log_map;

        size_t nValues = sizeof...(Values);
        (void)nValues;

        cbor_encoder_init(&encoder, byteBuffer, bufferSize, 0);
        cbor_encoder_create_map(&encoder, &root_map, 1U);

        cbor_encode_text_stringz(&root_map, "payload");
        cbor_encoder_create_map(&root_map, &payload_map, 1U);

        cbor_encode_text_stringz(&payload_map, "Log");
        cbor_encoder_create_map(&payload_map, &log_map, 3U);

        // cbor_encoder_create_map(&encoder, &log_map, 3U + (nValues > 0U ? 1U : 0U));

        encodeRecord(&log_map, record);
        encodeFormatString(&log_map, formatString);

        // todo: implement value serialization that works with rust
        // serializeValues(&log_map, std::forward<const Values&>(values)...);

        cbor_encoder_close_container(&payload_map, &log_map);
        cbor_encoder_close_container(&root_map, &payload_map);
        auto result = cbor_encoder_close_container(&encoder, &root_map);

        if (result != CborNoError) {
            return 0U;
        } else {
            return cbor_encoder_get_buffer_size(&encoder, byteBuffer);
        }
    }

private:
    template<typename... Values>
    static void serializeValues(CborEncoder* encoder, const Values&... values)
    {
        CborEncoder arrayEncoder;
        size_t nValues = sizeof...(Values);

        cbor_encode_text_stringz(encoder, "values");
        cbor_encoder_create_array(encoder, &arrayEncoder, nValues);
        // Workaround for missing fold expression in C++14
        std::array<CborError, sizeof...(Values)> errors[] {(encodeValue(&arrayEncoder, values))...};
        (void)errors;
        cbor_encoder_close_container(encoder, &arrayEncoder);
    }

    static void serializeValues(CborEncoder*)
    {
        // no value to serialize
    }

    template<typename T>
    static std::enable_if_t<std::is_unsigned<T>::value && !std::is_same<T, bool>::value, CborError>
    encodeValue(CborEncoder* encoder, T value)
    {
        return cbor_encode_uint(encoder, value);
    }

    template<typename T>
    static std::enable_if_t<std::is_signed<T>::value && !std::is_floating_point<T>::value, CborError>
    encodeValue(CborEncoder* encoder, T value)
    {
        return cbor_encode_int(encoder, value);
    }

    template<typename T>
    static std::enable_if_t<std::is_same<T, bool>::value, CborError> encodeValue(CborEncoder* encoder, T value)
    {
        return cbor_encode_boolean(encoder, value);
    }

    template<typename T>
    static std::enable_if_t<std::is_same<T, float>::value, CborError> encodeValue(CborEncoder* encoder, T value)
    {
        return cbor_encode_float(encoder, value);
    }

    template<typename T>
    static std::enable_if_t<std::is_same<T, double>::value, CborError> encodeValue(CborEncoder* encoder, T value)
    {
        return cbor_encode_double(encoder, value);
    }

    template<typename T>
    static std::enable_if_t<std::is_same<T, const char*>::value || std::is_same<T, char*>::value, CborError>
    encodeValue(CborEncoder* encoder, T value)
    {
        return cbor_encode_text_stringz(encoder, value);
    }

    static void encodeFormatString(CborEncoder* encoder, const char* formatString)
    {
        cbor_encode_text_stringz(encoder, "message");
        cbor_encode_text_stringz(encoder, formatString);
    }

    static void encodeFormatString(CborEncoder* encoder, uint32_t formatStringId)
    {
        cbor_encode_text_stringz(encoder, "message");
        cbor_encode_uint(encoder, formatStringId);
    }

    static void encodeRecord(CborEncoder* encoder, const LogRecord& record)
    {
        cbor_encode_text_stringz(encoder, "timestamp");
        cbor_encode_uint(encoder, record.timestamp);

        cbor_encode_text_stringz(encoder, "level");
        cbor_encode_uint(encoder, toUint(record.metadata.level));

        // cbor_encode_text_stringz(encoder, "context_tag");
        // cbor_encode_uint(encoder, record.metadata.contextTag);
        //
        // cbor_encode_text_stringz(encoder, "log_tag");
        // cbor_encode_uint(encoder, record.metadata.loggerTag);
    }
};

namespace log_conf {

// #define SE_OSS_LOG_REPLACE_STRINGS
using Formatter = RobbyFormatter;
using Buffer = AtomicBuffer<2048>;
constexpr std::size_t MAX_MESSAGE_LENGTH {256};
constexpr LogLevel MAX_LOG_LEVEL {LogLevel::TRACE};

} // namespace log_conf
} // namespace se_oss