#include "rtos/util/Core.h"

// todo: replace with generic ARM register
#include <stm32f7xx_hal.h>

namespace rtos {

bool isInterruptContextAcive()
{
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
}

}
