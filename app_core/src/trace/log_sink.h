#pragma once

#include "trace/link.h"

#include <robby_link.pb.h>
#include <se-oss/log/LogFilter.h>
#include <se-oss/log/sink/ILogSink.h>

namespace robby {

/// Sends the log records formatted by se_oss::RobbyFormatter as LogRecords over the link.
/// Written by the task that distributes the log messages.
class LogSink final : public se_oss::ILogSink
{
public:
    /// `time` gives the uptime for records without a timestamp of their own
    LogSink(Link& link, uint64_t (*time)());

    // ILogSink implementation
    void write(const se_oss::LogMetadata& metadata, const void* data, std::size_t length) override;
    void flush() override { }
    void setLogLevel(se_oss::LogLevel level) override { _filter.setLogLevel(level); }
    void setFilter(se_oss::LogFilterFunction filter) override { _filter.setFilter(filter); }

private:
    Link& _link;
    uint64_t (*_time)();
    se_oss::LogFilter _filter {};
    robby_link_Record _record {};
};

}  // namespace robby
