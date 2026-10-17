#include "clock.h"

#include <stm32f7xx_hal.h>

uint64_t robby::uptime_ns()
{
    return static_cast<uint64_t>(HAL_GetTick()) * 1'000'000U;
}
