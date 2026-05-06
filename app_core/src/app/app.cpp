#include "app.h"
#include "glue/araldite.h"
#include "service/command_handler.h"
#include "service/distance_visualizer.h"
#include "service/imu.h"

#include <rtos/rtos.h>

using namespace robby;

void App::launch(void)
{
    auto command_handler = std::make_unique<robby::CommandHandler>();
    g_uartDataReceived.register_callback([&](auto* data, auto length){
        command_handler->data_received_callback(data, length);
    });

    auto oneTask = rtos::Task::build()
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
    while(true);
}