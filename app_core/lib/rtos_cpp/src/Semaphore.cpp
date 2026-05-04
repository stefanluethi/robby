#include "rtos/rtos.h"
#include "rtos/Semaphore.h"
#include "rtos/util/Core.h"

#include <assert.h>

namespace rtos {

Semaphore::Semaphore() :
    _handle{xSemaphoreCreateBinary()}
{
    assert(_handle);
}

Semaphore::Semaphore(uint32_t count, uint32_t initial) :
    _handle{xSemaphoreCreateCounting(count, initial)}
{
    assert(_handle);
}

Semaphore::~Semaphore()
{
    vSemaphoreDelete(_handle);
}

void Semaphore::acquire()
{
    // todo: handle fail
    if (!isInterruptContextAcive()) {
        xSemaphoreTake(_handle, portMAX_DELAY);
    } else {
        // we should never land here...
        BaseType_t shouldYield = pdFALSE;
        xSemaphoreTakeFromISR(_handle, &shouldYield);
        portYIELD_FROM_ISR(shouldYield);
    }
}

bool Semaphore::tryAcquire(uint32_t timeoutMs)
{
    bool acquired {false};
    if (!isInterruptContextAcive()) {
        acquired = xSemaphoreTake(_handle, msToTicks(timeoutMs)) == pdTRUE;
    } else {
        BaseType_t shouldYield {pdFALSE};
        acquired = xSemaphoreTakeFromISR(_handle, &shouldYield);
        portYIELD_FROM_ISR(shouldYield);
    }
    return acquired;
}

void Semaphore::release()
{
    // todo: handle error case
    if (!isInterruptContextAcive()) {
        xSemaphoreGive(_handle);
    } else {
        BaseType_t shouldYield {pdFALSE};
        xSemaphoreGiveFromISR(_handle, &shouldYield);
        portYIELD_FROM_ISR(shouldYield);
    }
}

}
