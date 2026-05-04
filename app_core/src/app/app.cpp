#include "app.h"
#include "rtos/rtos.h"

extern "C" void APP_launch(void)
{
    
    rtos::startScheduler();
    while(true);
}