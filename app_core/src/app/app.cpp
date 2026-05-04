#include "app.h"
#include "service/distance/distance_visualizer.h"
#include "service/imu/imu.h"

#include <rtos/rtos.h>

extern "C" void APP_launch(void)
{

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