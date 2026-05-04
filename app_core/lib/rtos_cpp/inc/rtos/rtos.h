#pragma once

#include "EventFlags.h"
#include "MessageQueue.h"
#include "Mutex.h"
#include "Semaphore.h"
#include "Task.h"
#include "Timer.h"

#include <FreeRTOS.h>
#include <cstdint>

namespace rtos {

[[noreturn]] void startScheduler();

}
