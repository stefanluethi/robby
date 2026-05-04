#pragma once

#include "util/Core.h"
#include "util/Flags.h"
#include "util/Types.h"

#include <FreeRTOS.h>
#include <event_groups.h>

#include <cassert>
#include <initializer_list>

namespace rtos {

template<typename T>
class EventFlags {
public:
    EventFlags() :
        _handle{xEventGroupCreate()}
    {
        assert(_handle);
    }

    ~EventFlags()
    {
        vEventGroupDelete(_handle);
    }

    EventFlags(const EventFlags&) = delete;
    EventFlags(EventFlags&&) = delete;

    Result set(T flag)
    {
        return set(Flags<T>(flag).raw());
    }
    Result set(std::initializer_list<T> flags)
    {
        return set(Flags<T>(flags).raw());
    }
    
    Result clear(T flag)
    {
        return clear(Flags<T>(flag).raw());
    }
    Result clear(std::initializer_list<T> flags)
    {
        return clear(Flags<T>(flags).raw());
    }

    bool isSet(T flag)
    {
        return (get() & Flags<T>(flag).raw()) > 0;
    }
    bool isSet(std::initializer_list<T> flags)
    {
        return get(get() & Flags<T>(flags).raw()) > 0;
    }


    Result awaitAny(std::initializer_list<T> flags,
                    uint32_t timeoutMs = portMAX_DELAY, 
                    bool clearOnExit = false)
    {
    	return awaitAny(Flags<T>(flags).raw(), timeoutMs, clearOnExit);
    }

    Result awaitAll(std::initializer_list<T> flags,
                    uint32_t timeoutMs = portMAX_DELAY,
                    bool clearOnExit = false)
	{
		return awaitAll(Flags<T>(flags).raw(), timeoutMs, clearOnExit);
	}

    EventGroupHandle_t handle() { return _handle; }

private:
    EventGroupHandle_t _handle {nullptr};

    Result set(uint32_t flags)
    {
        Result result;
        if (!isInterruptContextAcive()) {
            xEventGroupSetBits(_handle, flags);
            result = Result::Ok;
        } else {
            BaseType_t shouldYield {pdFALSE};
            auto enqueued = xEventGroupSetBitsFromISR(_handle, flags, &shouldYield);
            result = enqueued == pdTRUE ? Result::Ok : Result::ErrorBusy;
            portYIELD_FROM_ISR(shouldYield);
        }
        return result;
    }

    Result clear(uint32_t flags)
    {
        Result result;
        if (!isInterruptContextAcive()) {
            xEventGroupClearBits(_handle, flags);
            result = Result::Ok;
        } else {
            auto enqueued = xEventGroupClearBitsFromISR(_handle, flags);
            result = enqueued == pdTRUE ? Result::Ok : Result::ErrorBusy;
        }
        return result;
    }

    uint32_t get()
    {
        uint32_t flags {0U};
        if (!isInterruptContextAcive()) {
            flags = xEventGroupGetBits(_handle);
        } else {
            flags = xEventGroupGetBitsFromISR(_handle);
        }
        return flags;
    }

    Result awaitAny(uint32_t flags,
                    uint32_t timeoutMs,
                    bool clearOnExit)
    {
        Result result;
        uint32_t resultingFlags {0U};
        if (!isInterruptContextAcive()) {
            resultingFlags = xEventGroupWaitBits(
                _handle, 
                flags, 
                clearOnExit ? pdTRUE : pdFALSE, 
                pdFALSE,
                msToTicks(timeoutMs));
            result = Result::Ok;
        } else {
            result = Result::ErrorWouldBlock;
        }
        return result;
    }

    Result awaitAll(uint32_t flags, 
                      uint32_t timeoutMs, 
                      bool clearOnExit)
    {
        Result result;
        uint32_t resultingFlags {0U};
        if (!isInterruptContextAcive()) {
            resultingFlags = xEventGroupWaitBits(
                _handle, 
                flags, 
                clearOnExit ? pdTRUE : pdFALSE, 
                pdTRUE,
                msToTicks(timeoutMs));
            result = resultingFlags == flags ? Result::Ok : Result::ErrorUnknown;
        } else {
            result = Result::ErrorWouldBlock;
        }
        return result;
    }
};
    
}
