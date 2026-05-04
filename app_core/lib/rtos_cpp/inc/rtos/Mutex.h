#pragma once

#include "api/IMutex.h"

#include <FreeRTOS.h>
#include <semphr.h>

namespace rtos {

class Mutex : public IMutex {
public:
    Mutex();
    ~Mutex() override;
    Mutex(const Mutex&) = delete;
    Mutex(Mutex&&) = delete;

    void lock() override;
    bool tryLock(uint32_t timeoutMs) override;
    void unlock() override;

    SemaphoreHandle_t handle() { return _handle; }

private:
    SemaphoreHandle_t _handle {nullptr};
};
    
}
