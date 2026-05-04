#include "rtos/rtos.h"
#include "unity.h"

#include "SemaphoreTest.h"

using namespace rtos;

extern "C" void setUp() { }
extern "C" void tearDown() { }

void runAllTests(void* arg)
{
    UNITY_BEGIN();

    RUN_TEST(SemaphoreTest::run);

    UNITY_END();
}

extern "C" void testMain(void)
{
    xTaskCreate(runAllTests, "Test Runner", 512, nullptr, 2, nullptr);
    vTaskStartScheduler();
}