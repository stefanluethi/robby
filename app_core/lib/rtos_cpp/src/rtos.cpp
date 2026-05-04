#include "rtos/rtos.h"

#include <task.h>

namespace rtos {

void startScheduler()
{
    vTaskStartScheduler();

    assert(true);
    while(true);
}

}
