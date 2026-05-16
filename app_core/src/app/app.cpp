#include "app.h"

#include "conf/log.h"
#include "driver/serial_driver.h"
#include "glue/araldite.h"
#include "rtos/Task.h"
#include "service/command_handler.h"
#include "service/data_publisher.h"
#include "service/distance_visualizer.h"
#include "service/imu.h"

#include <memory>

#include <rtos/rtos.h>
#include <se-oss/log/Log.h>
#include <se-oss/log/LogRegistry.h>
#include <se-oss/log/sink/ConsoleSink.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

using namespace robby;

template <>
auto se_oss::logConf<>()
{
    return LogConf<PrintfFormatter<TimeFormat::HEX_8>, AtomicBuffer<>, 256>{};
}

void App::launch()
{
    // Drivers -----------------------------------------------------------------
    auto* virtual_com_port = new SerialDriver(huart6);

    // Utilities ---------------------------------------------------------------
    auto log_registry = std::make_unique<se_oss::LogRegistry<LogContextId, LogSinkId>>();
    log_registry->attachSink(LogSinkId::SERIAL, std::make_unique<se_oss::ConsoleSink>());
    log_registry->getContext(LogContextId::COMMAND).setLogLevel(se_oss::LogLevel::OFF);
    log_registry->getContext(LogContextId::SPACE_ACQUISITION).setLogLevel(se_oss::LogLevel::OFF);

    // IPC ---------------------------------------------------------------------
    auto* acceleration_frames = new rtos::MessageQueue<AccelerationFrame>(3);
    auto* distance_map = new DistanceMap();

    // Services ----------------------------------------------------------------
    auto* command_handler = new CommandHandler(log_registry->createLogger(LogContextId::COMMAND));
    g_uartDataReceived.register_callback([&](auto* data, auto length){
        command_handler->data_received_callback(data, length);
    });

    auto distance_visualizer = std::make_unique<DistanceVisualizer>(log_registry->createLogger(LogContextId::SPACE_ACQUISITION), *distance_map);
    g_distanceConversionDone.register_callback([&]() {
        distance_visualizer->conversion_done_callback();
    });

    auto imu = std::make_unique<Imu>(*acceleration_frames);

    auto data_publisher = std::make_unique<DataPublisher>(*virtual_com_port, *acceleration_frames, *distance_map);

    // Threads -----------------------------------------------------------------
    auto command_thread = rtos::Task::build()
        .name("command")
        .stackSize(8192)
        .priority(6)
        .spawn([&](){
            while (true) {
                command_handler->process();
            }
        });

    auto once_thread = rtos::Task::build()
        .name("acquisition")
        .stackSize(20 * 1024)
        .priority(3)
        .spawn([&](){
            distance_visualizer->start();

            while (true) {
                distance_visualizer->process();
            }
        });

    auto imu_thread = rtos::Task::build()
        .name("imu")
        .stackSize(2048)
        .priority(5)
        .spawn([&] {
            imu->start();

            while (true) {
                imu->process();
                rtos::Task::sleep(10);
            }
    });

    auto data_publisher_thread = rtos::Task::build()
    .name("data_publisher")
    .stackSize(2048)
    .priority(4)
    .spawn([&] {
            while (true) {
                data_publisher->publish();
                rtos::Task::sleep(10);
            }
    });

    rtos::startScheduler();
}