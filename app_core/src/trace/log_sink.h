#pragma once

#include <se-oss/log/LogFilter.h>
#include <se-oss/log/sink/ILogSink.h>
#include <sonde/sonde.hpp>

namespace robby {

/// Traces the log records formatted by se_oss::RobbyFormatter with sonde. Must be
/// written from the context that traces with the tracer.
class LogSink final : public se_oss::ILogSink
{
public:
    explicit LogSink(sonde::Tracer& tracer);

    // ILogSink implementation
    void write(const se_oss::LogMetadata& metadata, const void* data, std::size_t length) override;
    void flush() override { }
    void setLogLevel(se_oss::LogLevel level) override { _filter.setLogLevel(level); }
    void setFilter(se_oss::LogFilterFunction filter) override { _filter.setFilter(filter); }

private:
    sonde::Tracer& _tracer;
    se_oss::LogFilter _filter {};
};

}  // namespace robby
