#include "SemaphoreTest.h"

#include "rtos/Semaphore.h"
#include "rtos/Task.h"
#include "rtos/EventFlags.h"

#include <unity.h>

using namespace rtos;

void SemaphoreTest::run()
{
    auto testBinarySemaphore = std::make_shared<Semaphore>();
    auto testFlags = std::make_shared<EventFlags<TaskFlags>>();

    auto taskA = Task::build()
        .name("task_a")
        .priority(3)
        .spawn([=](){
            auto success = testBinarySemaphore->tryAcquire();
            TEST_ASSERT_FALSE(success);

            testFlags->set(TaskFlags::TASK_A_DONE);
        });

    auto taskB = Task::build()
        .name("task_b")
        .priority(3)
        .spawn([=](){
            auto success = testBinarySemaphore->tryAcquire();
            TEST_ASSERT_FALSE(success);

            testFlags->set(TaskFlags::TASK_B_DONE);
        });

    TEST_ASSERT_EQUAL(testFlags->awaitAll({TaskFlags::TASK_A_DONE, TaskFlags::TASK_B_DONE}, 100), Result::Ok);
}
