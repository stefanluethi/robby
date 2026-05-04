#pragma once

namespace rtos {

class SemaphoreTest {
public:
    static void run();

private:
    enum class TaskFlags {
        TASK_A_DONE,
        TASK_B_DONE,
    };
};

} // rtos
