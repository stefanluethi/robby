#pragma once

#include "se-oss/log/buffer/AtomicBuffer.h"
#include "se-oss/log/format/StringBuffer.h"

#include <cstring>

namespace se_oss {

/// Formats a log record as its timestamp (uint64_t, native byte order) followed by the
/// message text, which robby::LogSink hands on to sonde.
class RobbyFormatter
{
public:
    static constexpr std::size_t TIMESTAMP_LENGTH {sizeof(uint64_t)};

    template<typename TFormat, typename... Values>
    static size_t
    format(void* buffer, std::size_t bufferSize, const LogRecord& record, TFormat formatString, const Values&... values)
    {
        if (bufferSize <= TIMESTAMP_LENGTH) {
            return 0U;
        }

        auto* byteBuffer = static_cast<uint8_t*>(buffer);
        std::memcpy(byteBuffer, &record.timestamp, TIMESTAMP_LENGTH);

        StringBuffer message {byteBuffer + TIMESTAMP_LENGTH, bufferSize - TIMESTAMP_LENGTH};
        message.append(formatString, values...);

        return TIMESTAMP_LENGTH + message.length();
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
