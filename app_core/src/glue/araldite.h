#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void RESIN_launch_app(void);

void RESIN_DataReceivedCallback(void* data, size_t length);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "glue/hardener.h"

extern glue::Hardener<void*, size_t> g_uartDataReceived;

#endif
