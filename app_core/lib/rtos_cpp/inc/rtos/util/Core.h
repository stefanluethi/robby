#pragma once

#include <FreeRTOS.h>

namespace rtos {

bool isInterruptContextAcive();

constexpr uint32_t msToTicks(uint32_t ms) { return ms / portTICK_PERIOD_MS; }

}
