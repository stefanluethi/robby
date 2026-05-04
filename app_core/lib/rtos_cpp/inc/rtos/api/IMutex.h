#pragma once

#include <cstdint>

namespace rtos {

class IMutex {
public:
    virtual ~IMutex() = default;

    virtual void lock() = 0;
    virtual bool tryLock(uint32_t timeoutMs = 0U) = 0;
    virtual void unlock() = 0;

protected:
    IMutex() = default;
};
    
}
