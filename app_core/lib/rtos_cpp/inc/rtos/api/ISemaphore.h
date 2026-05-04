#pragma once

#include <cstdint>

namespace rtos {

class ISemaphore {
public:
    virtual ~ISemaphore() = default;

    virtual void acquire() = 0;
    virtual bool tryAcquire(uint32_t timeoutMs = 0U) = 0;
    virtual void release() = 0;

protected:
    ISemaphore() = default;
};
    
}
