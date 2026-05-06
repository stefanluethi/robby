#include "app.h"

#include "conf/log.h"
#include "glue/araldite.h"
#include "rtos/Task.h"

#include "service/command_handler.h"
#include "service/distance_visualizer.h"
#include "service/imu.h"
#include "stm32f7xx_hal.h"

#include <rtos/rtos.h>
#include <se-oss/log/Log.h>
#include <se-oss/log/LogRegistry.h>
#include <se-oss/log/sink/ConsoleSink.h>

#include <cstdint>
#include <memory>

using namespace robby;

template <>
auto se_oss::logConf<>()
{
    return LogConf<PrintfFormatter<TimeFormat::ISO8601>, AtomicBuffer<1024>, 128>{};
}

void App::launch()
{
    auto log_registry = std::make_unique<se_oss::LogRegistry<LogContextId, LogSinkId>>();
    log_registry->attachSink(LogSinkId::SERIAL, std::make_unique<se_oss::ConsoleSink>());


    auto command_handler = std::make_unique<CommandHandler>(log_registry->createLogger(LogContextId::COMMAND));
    g_uartDataReceived.register_callback([&](auto* data, auto length){
        command_handler->data_received_callback(data, length);
    });

    auto command_thread = rtos::Task::build()
        .name("command")
        .stackSize(2048)
        .priority(5)
        .spawn([&](){

            while (true) {
                command_handler->process();
            }
        });

    auto once_thread = rtos::Task::build()
        .name("one")
        .stackSize(10 * 1024)
        .priority(4)
        .spawn([](){
            DIST_Init();
            IMU_init();

            while (true) {
                DIST_Process();
            }
        });

    rtos::startScheduler();
}