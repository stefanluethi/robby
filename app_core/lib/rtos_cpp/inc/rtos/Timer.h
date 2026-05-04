#pragma once

#include <FreeRTOS.h>
#include <timers.h>

#include <functional>
#include <memory>

namespace rtos {

class TimerBuilder;

class Timer {
public:
    enum class Mode {
        OneShot,
        Periodical
    };

    Timer(const char* name,
          uint32_t periodMs,
          Mode mode,
          std::function<void()> entry);
    ~Timer();
    Timer() = delete;
    Timer(const Timer&) = delete;
    Timer(Timer&&) = delete;

    static TimerBuilder build();

    TimerHandle_t handle() { return _handle; }

private:
    TimerHandle_t _handle {nullptr};
    std::function<void()> _entry;

    static void entryTrampoline(TimerHandle_t context);
};


class TimerBuilder {
public:
    TimerBuilder() = default;
    TimerBuilder& name(const char* name);
    TimerBuilder& period(uint32_t periodMs);
    TimerBuilder& mode(Timer::Mode mode);
    std::unique_ptr<Timer> callback(std::function<void()> callback);

private:
    const char* _name {nullptr};
    uint32_t _periodMs {UINT32_MAX};
    Timer::Mode _mode {Timer::Mode::Periodical};
};
    
}
