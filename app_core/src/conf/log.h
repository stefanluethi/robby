#pragma once

#include <cstdint>

namespace robby {
enum class LogContextId : uint8_t
{
    COMMAND,
    SPACE_ACQUISITION,
    IMU_ACQUISITION,
};

constexpr uint8_t toUint(LogContextId id) { return static_cast<uint8_t>(id); }
constexpr const char* toString(LogContextId id)
{
    switch (id) {
        case LogContextId::COMMAND: return "cmd";
        case LogContextId::SPACE_ACQUISITION: return "space";
        case LogContextId::IMU_ACQUISITION: return "imu";
    }
    return "";
}

enum class LogSinkId : uint8_t
{
    SERIAL,
};

constexpr uint8_t toUint(LogSinkId sink) { return static_cast<uint8_t>(sink); }
constexpr const char* toString(LogSinkId sink)
{
    switch (sink) {
        case LogSinkId::SERIAL: return "serial";
    }
    return "";
}

}
