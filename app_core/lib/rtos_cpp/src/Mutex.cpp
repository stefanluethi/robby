#include "rtos/rtos.h"
#include "rtos/Mutex.h"
#include "rtos/util/Core.h"

#include "assert.h"

namespace rtos {

Mutex::Mutex() :
    _handle{xSemaphoreCreateMutex()}
{
    assert(_handle);
}

Mutex::~Mutex()
{
    vSemaphoreDelete(_handle);
}

void Mutex::lock()
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

bool Mutex::tryLock(uint32_t timeoutMs)
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

void Mutex::unlock()
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
