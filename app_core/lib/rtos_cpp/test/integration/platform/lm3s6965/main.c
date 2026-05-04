#include <stdio.h>

#include "cmsis/ARMCM3.h"

// Redirect file handlers to UART0
// TI LM3S-specific address is used
// Nor readiness check, nor configuration - this stuff works fine under qemu
#define UART0_DR *((uint32_t *) 0x4000c000)

__attribute__((unused)) int _write(__attribute__((unused))int file, const char *data, int len) {
    for (int i = 0; i < len; i++) {
        UART0_DR = data[i];
    }
    return len;
}

int putchar (int c) {
    UART0_DR = (char)c;
    return 1;
}

void testMain(void);

int main(void) {
    testMain();

    // trigger reset
    SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
}
