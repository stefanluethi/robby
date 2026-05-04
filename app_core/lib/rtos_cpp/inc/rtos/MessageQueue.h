#pragma once

#include "api/IItemReceiver.h"
#include "api/IItemSender.h"
#include "util/Core.h"

#include <FreeRTOS.h>
#include <queue.h>

#include <functional>
#include <memory>

namespace rtos {

template<typename T>
class MessageQueue final :
    public IItemSender<T>,
    public IItemReceiver<T>
{
public:
    MessageQueue(size_t length)
    {
        _handle = xQueueCreate(length, sizeof(T));
        assert(_handle);
    }

    ~MessageQueue() override
    {
        vQueueDelete(_handle);
    }
    
    MessageQueue(const MessageQueue&) = delete;
    MessageQueue(MessageQueue&&) = delete;

    // IItemSender implementation
    bool send(const T& item) override
    {
        return trySend(item, portMAX_DELAY);
    }

    bool trySend(const T& item, uint32_t timeoutMs = 0U) override
    {
        bool sent {false};
        if (!isInterruptContextAcive()) {
            sent = xQueueSend(_handle, &item, msToTicks(timeoutMs)) == pdTRUE;
        } else {
            BaseType_t shouldYield {pdFALSE};
            sent = xQueueSendFromISR(_handle, &item, &shouldYield) == pdTRUE;
            portYIELD_FROM_ISR(shouldYield);
        }
        return sent;
    }

    // IItemReceiver implementation
    bool receive(T& item) override
    {
        return tryReceive(item, portMAX_DELAY);
    }

    bool tryReceive(T& item, uint32_t timeoutMs = 0U) override
    {
        bool sent {false};
        if (!isInterruptContextAcive()) {
            sent = xQueueReceive(_handle, &item, msToTicks(timeoutMs)) == pdTRUE;
        } else {
            BaseType_t shouldYield {pdFALSE};
            sent = xQueueReceiveFromISR(_handle, &item, &shouldYield) == pdTRUE;
            portYIELD_FROM_ISR(shouldYield);
        }
        return sent;
    }

    QueueHandle_t handle() { return _handle; }

private:
    QueueHandle_t _handle;
};

}
