#pragma once

#include <FreeRTOS.h>
#include <task.h>

#include <functional>
#include <memory>

namespace rtos {

class TaskBuilder;

class Task {
public:
    Task(std::function<void()> entry,
         const char* name,
         uint16_t stackSizeHalfWords,
         uint16_t priority);
    ~Task();
    Task() = delete;
    Task(const Task&) = delete;
    Task(Task&&) = delete;

    uint16_t priority();
    void suspend();
    void resume();
    uint32_t stackHighWaterMark();

    static void sleep(uint32_t ms);
    static void sleepS(uint32_t s);

    static void suspendAll();
    static bool resumeAll();
    
    static TaskBuilder build();

    TaskHandle_t handle() { return _handle; }

private:
    TaskHandle_t _handle {nullptr};
    std::function<void()> _entry;

    static void entryTrampoline(void* context);
};


class TaskBuilder {
public:
    TaskBuilder() = default;
    TaskBuilder& name(const char* name);
    TaskBuilder& stackSize(uint16_t sizeBytes);
    TaskBuilder& priority(uint16_t priority);
    std::unique_ptr<Task> spawn(std::function<void()> entryFunction);

private:
    const char* _name {nullptr};
    uint16_t _stackSize {1024};
    uint16_t _priority {2};

};
    
}
