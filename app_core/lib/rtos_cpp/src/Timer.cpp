#include <rtos/Timer.h>
#include <cassert>

namespace rtos {

Timer::Timer(const char* name,
             uint32_t periodMs,
             Mode mode,
             std::function<void()> entry)
{
    _entry = entry;
    _handle = xTimerCreate(name,
                           pdMS_TO_TICKS(periodMs),
                           mode == Timer::Mode::Periodical,
                           static_cast<void*>(this),
                           entryTrampoline); 
    assert(_handle);
    xTimerStart(_handle, 0);
}

Timer::~Timer()
{
    xTimerDelete(_handle, 0);
}

void Timer::entryTrampoline(TimerHandle_t handle)
{
	auto* context = pvTimerGetTimerID(handle);
    if (context == nullptr) {
        return;
    }
    
    auto* self = reinterpret_cast<Timer*>(context);
    if (self->_entry) {
    	self->_entry();
    }
}

TimerBuilder Timer::build()
{
	return TimerBuilder {};
}

TimerBuilder& TimerBuilder::name(const char* name)
{
    _name = name;
    return *this;
}

TimerBuilder& TimerBuilder::period(uint32_t periodMs)
{
    _periodMs = periodMs;
    return *this;
}

TimerBuilder& TimerBuilder::mode(Timer::Mode mode)
{
    _mode = mode;
    return *this;
}

std::unique_ptr<Timer> TimerBuilder::callback(std::function<void()> callback)
{
	return std::make_unique<Timer>(_name, _periodMs, _mode, callback);
}

}
