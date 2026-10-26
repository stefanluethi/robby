#include "log_sink.h"

#include "conf/log.h"
#include "trace/records.h"

#include <cstring>

#include <se-oss/log/Log.h>

#include <string_view>

using namespace robby;

// The outpost names the components by these numbers
static_assert(toUint(LogContextId::COMMAND) == robby_link_Component_COMPONENT_CMD);
static_assert(toUint(LogContextId::SPACE_ACQUISITION) == robby_link_Component_COMPONENT_SPACE);
static_assert(toUint(LogContextId::IMU_ACQUISITION) == robby_link_Component_COMPONENT_IMU);
static_assert(se_oss::toUint(se_oss::LogLevel::TRACE) == robby_link_LogLevel_LOG_LEVEL_TRACE);
static_assert(se_oss::toUint(se_oss::LogLevel::ERROR) == robby_link_LogLevel_LOG_LEVEL_ERROR);
static_assert(se_oss::toUint(se_oss::LogLevel::FATAL) == robby_link_LogLevel_LOG_LEVEL_FATAL);

LogSink::LogSink(Link& link, uint64_t (*time)()) : _link {link}, _time {time} { }

void LogSink::write(const se_oss::LogMetadata& metadata, const void* data, std::size_t length)
{
    constexpr std::size_t TIMESTAMP_LENGTH {se_oss::RobbyFormatter::TIMESTAMP_LENGTH};

    if (!_filter.passesFilter(metadata) || length < TIMESTAMP_LENGTH) {
        return;
    }

    const auto* bytes = static_cast<const uint8_t*>(data);
    uint64_t timestamp {};
    std::memcpy(&timestamp, bytes, TIMESTAMP_LENGTH);
    if (timestamp == se_oss::INVALID_TIME) {
        timestamp = _time();
    }

    const std::string_view message {reinterpret_cast<const char*>(bytes + TIMESTAMP_LENGTH), length - TIMESTAMP_LENGTH};
    records::make_log(timestamp, se_oss::toUint(metadata.level), metadata.contextTag, message, _record);
    _link.add(_record);
}
