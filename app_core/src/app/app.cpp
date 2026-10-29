#include "app.h"

#include  "main.h"
#include "conf/log.h"
#include "driver/serial_driver.h"
#include "glue/araldite.h"
#include "rtos/Task.h"
#include "service/bench_load.h"
#include "service/command_handler.h"
#include "service/data_publisher.h"
#include "service/drive.h"
#include "service/distance_visualizer.h"
#include "service/imu.h"
#include "trace/clock.h"
#include "trace/link.h"
#include "trace/log_sink.h"

#include <memory>

#include <rtos/rtos.h>
#include <se-oss/log/Log.h>
#include <se-oss/log/LogRegistry.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

using namespace robby;

void App::launch()
{
    // Drivers -----------------------------------------------------------------
    // huart2: protobuf link to the ESP32 outpost (and on to blip), huart6: virtual COM port
    enable_usart2_tx_dma(huart2);
    auto link = std::make_unique<Link>(huart2);

    // Utilities ---------------------------------------------------------------
    // the logs are sent by the data publisher task
    auto log_registry = std::make_unique<se_oss::LogRegistry<LogContextId, LogSinkId>>();
    log_registry->setTimeProvider(&uptime_ns);
    log_registry->attachSink(LogSinkId::SERIAL, std::make_unique<LogSink>(*link, &uptime_ns));
    // log_registry->getContext(LogContextId::COMMAND).setLogLevel(se_oss::LogLevel::OFF);
    // log_registry->getContext(LogContextId::SPACE_ACQUISITION).setLogLevel(se_oss::LogLevel::OFF);

    // IPC ---------------------------------------------------------------------
    auto* acceleration_frames = new rtos::MessageQueue<AccelerationFrame>(3);
    auto* battery = new rtos::MessageQueue<fuelgauge_status_t>(1);
    auto* distance_map = new DistanceMap();
    auto* bench_load = new BenchLoad();
    auto* drive = new Drive(log_registry->createLogger(LogContextId::DRIVE), *link);

    // Services ----------------------------------------------------------------
    auto* command_handler = new CommandHandler(log_registry->createLogger(LogContextId::COMMAND), *link, *drive, *bench_load);
    g_uartDataReceived.register_callback([&](auto* data, auto length){
        command_handler->data_received_callback(data, length);
    });

    auto distance_visualizer = std::make_unique<DistanceVisualizer>(
        log_registry->createLogger(LogContextId::SPACE_ACQUISITION),
        *distance_map
    );
    g_distanceConversionDone.register_callback([&]() {
        distance_visualizer->conversion_done_callback();
    });

    auto imu = std::make_unique<Imu>(
        log_registry->createLogger(LogContextId::IMU_ACQUISITION),
        *acceleration_frames,
        *battery,
        link->dropped_records(),
        *link
    );

    auto data_publisher = std::make_unique<DataPublisher>(
        *log_registry,
        *link,
        *acceleration_frames,
        *battery,
        *distance_map,
        *bench_load
    );

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

    auto drive_thread = rtos::Task::build()
        .name("drive")
        .stackSize(4096)
        .priority(6)
        .spawn([&] {
            drive->start();

            while (true) {
                drive->run();
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
        .stackSize(4096)
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
        .stackSize(4096)
        .priority(4)
        .spawn([&] {
                while (true) {
                    data_publisher->publish();
                    rtos::Task::sleep(10);
                }
        });

    auto heartbeat_thread = rtos::Task::build()
        .name("heartbeat")
        .stackSize(512)
        .priority(1)
        .spawn([&] {
                while (true) {
                    HAL_GPIO_WritePin(SYS_LD_USER2_GPIO_Port, SYS_LD_USER2_Pin, GPIO_PIN_SET);
                    rtos::Task::sleep(100);
                    HAL_GPIO_WritePin(SYS_LD_USER2_GPIO_Port, SYS_LD_USER2_Pin, GPIO_PIN_RESET);
                    rtos::Task::sleep(900);
                }
        });

    rtos::startScheduler();
}