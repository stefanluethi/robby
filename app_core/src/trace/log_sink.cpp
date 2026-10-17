#include "log_sink.h"

#include "conf/log.h"

#include <algorithm>
#include <cstring>
#include <string_view>

#include <se-oss/log/Log.h>

using namespace robby;

LogSink::LogSink(sonde::Tracer& tracer) :
    _tracer {tracer}
{ }

void LogSink::write(const se_oss::LogMetadata& metadata, const void* data, std::size_t length)
{
    constexpr std::size_t TIMESTAMP_LENGTH {se_oss::RobbyFormatter::TIMESTAMP_LENGTH};

    if (!_filter.passesFilter(metadata) || length < TIMESTAMP_LENGTH) {
        return;
    }

    const auto* bytes = static_cast<const uint8_t*>(data);
    uint64_t timestamp {};
    std::memcpy(&timestamp, bytes, TIMESTAMP_LENGTH);

    // blip knows no fatal level
    const auto level = static_cast<sonde::Level>(
        std::min(se_oss::toUint(metadata.level), se_oss::toUint(se_oss::LogLevel::ERROR))
    );
    const std::string_view component {toString(static_cast<LogContextId>(metadata.contextTag))};
    const std::string_view message {reinterpret_cast<const char*>(bytes + TIMESTAMP_LENGTH), length - TIMESTAMP_LENGTH};

    if (timestamp == se_oss::INVALID_TIME) {
        _tracer.log(level, component, message);
    } else {
        _tracer.log(sonde::Timestamp {timestamp}, level, component, message);
    }
}
