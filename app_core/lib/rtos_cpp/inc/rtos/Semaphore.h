#pragma once

#include "api/ISemaphore.h"

#include <FreeRTOS.h>
#include <semphr.h>

namespace rtos {

class Semaphore : public ISemaphore {
public:
    Semaphore();
    explicit Semaphore(uint32_t count, uint32_t initial = 0);
    ~Semaphore() override;
    Semaphore(const Semaphore&) = delete;
    Semaphore(Semaphore&&) = delete;
    Semaphore& operator=(const Semaphore&) = delete;
    Semaphore&& operator=(Semaphore&&) = delete;

    void acquire() override;
    bool tryAcquire(uint32_t timeoutMs = 0U) override;
    void release() override;

    SemaphoreHandle_t handle() { return _handle; }

private:
    SemaphoreHandle_t _handle {nullptr};
};
    
}
