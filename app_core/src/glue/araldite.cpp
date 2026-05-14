#include "araldite.h"

#include "app/app.h"

extern "C" void RESIN_launch_app(void)
{
    robby::App app {};
    app.launch();
}

extern "C" void RESIN_DistanceConversionDoneCallback(void)
{
    g_distanceConversionDone.call();
}


glue::Hardener<void*, size_t> g_uartDataReceived {};
glue::Hardener<> g_distanceConversionDone {};

extern "C" void RESIN_DataReceivedCallback(void* data, size_t length)
{
    g_uartDataReceived.call(data, length);
}


