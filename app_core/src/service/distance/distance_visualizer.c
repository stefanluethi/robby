
#include "distance_visualizer.h"
#include "main.h"
#include "stm32f7xx_hal_gpio.h"
#include "util/colormap.h"

#include <stm32f723e_discovery_lcd.h>
#include <stm32f7xx_hal.h>
#include <vl53l8cx_api.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define CONF_POLLING_PERIOD (1U)
#define CONF_SENSOR_FREQUENCY_HZ (10U)
#define CONF_N_SENSORS (3U)
#define SENSOR_INTEGRATION_TIME_MS (10U)

#define MAX_DISTANCE_MM (4000.0F)
#define SENSOR_RESOLUTION (VL53L8CX_RESOLUTION_8X8)
#define SENSOR_RESOLUTION_X (8U)
#define SENSOR_RESOLUTION_Y (8U)

typedef enum {
    SENSOR_RIGHT = 0U,
    SENSOR_MIDDLE = 1U,
    SENSOR_LEFT = 2U,
} DistanceSensor;

extern UART_HandleTypeDef huart6;

static VL53L8CX_Configuration devices[CONF_N_SENSORS];
static VL53L8CX_ResultsData results[CONF_N_SENSORS];
static bool data_ready;

static bool setup_sensor(VL53L8CX_Configuration *device);
static void draw_results();
static void trigger_sensor(DistanceSensor sensor_index);

void DIST_Init(void) {
    printf("Sensor initialization...\n");

    int32_t status = 0;
    for (uint8_t i = 0; i < CONF_N_SENSORS; ++i) {
        devices[i].platform.address = i;

        status |= vl53l8cx_init(&devices[i]);
    }

    if (status) {
        printf("vl53l8cx_init failed\n");
    }
}

void DIST_Process(void) {

    for (uint8_t i = 0; i < CONF_N_SENSORS; ++i) {
        bool success = setup_sensor(&devices[i]);
        if (success) {
            printf("Distance sensor %d setup failed\n", i);
            return;
        }
    }

    printf("Ranging starts\n");
    for (int32_t i = 0; i < CONF_N_SENSORS; ++i) {
        int32_t status = vl53l8cx_start_ranging(&devices[i]);
    }

    while (1) {
        for (uint8_t i = 0; i < CONF_N_SENSORS; ++i) {
            trigger_sensor(i);

            uint32_t conversion_timeout = 500U;
            while (!data_ready && --conversion_timeout) {
                HAL_Delay(CONF_POLLING_PERIOD);
            }
            data_ready = false;
        }

        for (uint8_t i = 0; i < CONF_N_SENSORS; ++i) {
            int32_t status =
                vl53l8cx_get_ranging_data(&devices[i], &results[i]);
        }
        draw_results();
    }
}

void DIST_ConversionDoneCallback(void) { data_ready = true; }

bool setup_sensor(VL53L8CX_Configuration *device) {
    int32_t status =
        vl53l8cx_set_ranging_frequency_hz(device, CONF_SENSOR_FREQUENCY_HZ);
    status |=
        vl53l8cx_set_ranging_mode(device, VL53L8CX_RANGING_MODE_AUTONOMOUS);
    status |= vl53l8cx_set_resolution(device, SENSOR_RESOLUTION);
    status |= vl53l8cx_set_integration_time_ms(device, SENSOR_INTEGRATION_TIME_MS);
    status |= vl53l8cx_set_external_sync_pin_enable(device, true);
    return status != 0;
}

void draw_results() {
    for (size_t sensor = 0; sensor < CONF_N_SENSORS; ++sensor) {
        uint16_t offset_x = (CONF_N_SENSORS - sensor - 1U) * 84; 
        uint16_t offset_y = 80; 
        for (uint8_t y = 0; y < SENSOR_RESOLUTION_Y; ++y) {
            for (uint8_t x = 0; x < SENSOR_RESOLUTION_X; ++x) {
                size_t cell = x + y * SENSOR_RESOLUTION_X;

                uint8_t status = results[sensor].target_status[cell];
                if (status != 5U && status != 9U) {
                    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                } else {
                    int16_t distance = results[sensor].distance_mm[cell];
                    uint16_t color =
                        map_color_rgb565(((float)distance) / MAX_DISTANCE_MM);
                    BSP_LCD_SetTextColor(color);
                }
                BSP_LCD_FillRect(x * 9 + offset_x, y * 9 + offset_y, 9, 9);
            }
        }
    }
}

void trigger_sensor(DistanceSensor sensor) {
    switch (sensor) {
    case SENSOR_RIGHT:
        HAL_GPIO_WritePin(TOF_SYNC1_GPIO_Port, TOF_SYNC1_Pin, 1U);
        HAL_GPIO_WritePin(TOF_SYNC1_GPIO_Port, TOF_SYNC1_Pin, 0U);
        break;
    case SENSOR_MIDDLE:
        HAL_GPIO_WritePin(TOF_SYNC2_GPIO_Port, TOF_SYNC2_Pin, 1U);
        HAL_GPIO_WritePin(TOF_SYNC2_GPIO_Port, TOF_SYNC2_Pin, 0U);
        break;
    case SENSOR_LEFT:
        HAL_GPIO_WritePin(TOF_SYNC3_GPIO_Port, TOF_SYNC3_Pin, 1U);
        HAL_GPIO_WritePin(TOF_SYNC3_GPIO_Port, TOF_SYNC3_Pin, 0U);
        break;
    }
}