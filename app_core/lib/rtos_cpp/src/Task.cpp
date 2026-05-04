#include "rtos/Task.h"
#include "rtos/util/Core.h"

#include <cassert>

namespace rtos {

Task::Task(std::function<void()> entry,
           const char* name,
           uint16_t stackSizeBytes,
           uint16_t priority) :
    _entry{std::move(entry)}
{
    auto result = xTaskCreate(entryTrampoline,
                              name,
                              stackSizeBytes / sizeof(size_t),
                              static_cast<void*>(this),
                              priority,
                              &_handle);
    assert(result == pdTRUE);
}

Task::~Task()
{
    vTaskDelete(_handle);
}

uint16_t Task::priority()
{
    if (!isInterruptContextAcive()) {
        return uxTaskPriorityGet(_handle);
    } else {
        return uxTaskPriorityGetFromISR(_handle);
    }
}

void Task::suspend()
{
    vTaskSuspend(_handle);
}

void Task::resume()
{
    if (!isInterruptContextAcive()) {
        vTaskResume(_handle);
    } else {
        auto shouldYield = xTaskResumeFromISR(_handle);
        portYIELD_FROM_ISR(shouldYield);
    }
}

uint32_t Task::stackHighWaterMark()
{
#if INCLUDE_uxTaskGetStackHighWaterMark == 1
    return uxTaskGetStackHighWaterMark(_handle);
#else
    return 0;
#endif
}


void Task::entryTrampoline(void* context)
{
	assert(context);
    auto* self = reinterpret_cast<Task*>(context);
    if (self->_entry) {
    	self->_entry();
    }
    delete self;
}

void Task::sleep(uint32_t ms)
{
    if (!isInterruptContextAcive()) {
        vTaskDelay(msToTicks(ms));
    }
}

void Task::sleepS(uint32_t s)
{
    sleep(s * 1000U);
}

void Task::suspendAll()
{
    vTaskSuspendAll();
}

bool Task::resumeAll()
{
    return xTaskResumeAll() == pdTRUE;
}

TaskBuilder Task::build()
{
	return TaskBuilder {};
}

TaskBuilder& TaskBuilder::name(const char* name)
{
    _name = name;
    return *this;
}

TaskBuilder& TaskBuilder::stackSize(uint16_t sizeBytes)
{
    _stackSize = sizeBytes;
    return *this;
}

TaskBuilder& TaskBuilder::priority(uint16_t priority)
{
    _priority = priority;
    return *this;
}

std::unique_ptr<Task> TaskBuilder::spawn(std::function<void()> entryFunction)
{
	return std::make_unique<Task>(std::move(entryFunction), _name, _stackSize, _priority);
}

}
