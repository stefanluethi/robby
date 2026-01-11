
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


#define POLLING_PERIOD (1)


extern UART_HandleTypeDef huart6;

static VL53L8CX_Configuration device;
static VL53L8CX_ResultsData results;
static bool data_ready;


static void MX_53L8A1_SimpleRanging_Init(void);
static void MX_53L8A1_SimpleRanging_Process(void);
static void draw_results(VL53L8CX_ResultsData *result);

void DIST_Init(void) { MX_53L8A1_SimpleRanging_Init(); }

/*
 * LM background task
 */
void DIST_Process(void) { MX_53L8A1_SimpleRanging_Process(); }

void MX_53L8A1_SimpleRanging_Init(void) {
    printf("Sensor initialization...\n");

    if (vl53l8cx_init(&device)) {
        printf("vl53l8cx_init failed\n");
    }
}

void MX_53L8A1_SimpleRanging_Process(void) {
    int32_t status = vl53l8cx_set_ranging_frequency_hz(&device, 10);
    status |=
        vl53l8cx_set_ranging_mode(&device, VL53L8CX_RANGING_MODE_CONTINUOUS);
    status |= vl53l8cx_set_resolution(&device, VL53L8CX_RESOLUTION_8X8);
    status |= vl53l8cx_set_external_sync_pin_enable(&device, true);

    // uint8_t vl53l8cx_set_external_sync_pin_enable(
    if (status) {
        printf("Confiuring TOF failed with status %ld\n", status);
    }

    printf("Ranging starts\n");
    status = vl53l8cx_start_ranging(&device);

    while (1) {
        /* polling mode */
        HAL_GPIO_WritePin(TOF_SYNC1_GPIO_Port, TOF_SYNC1_Pin, 1U);
        HAL_GPIO_WritePin(TOF_SYNC1_GPIO_Port, TOF_SYNC1_Pin, 0U);

        while (!data_ready)
        {
            HAL_Delay(POLLING_PERIOD);
        }

        status = vl53l8cx_get_ranging_data(&device, &results);
        draw_results(&results);
    }
}

void DIST_ConversionDoneCallback(void)
{
    data_ready = true;
}

void draw_results(VL53L8CX_ResultsData *result) {
    // pixel (0,0) is top left on the screen while it's bottom left on the
    // sensor
    for (uint8_t y = 0; y < 8U; ++y) {
        for (uint8_t x = 0; x < 8U; ++x) {
            size_t cell = x + y * 8U;

            uint8_t status = result->target_status[cell];
            if (status != 5U && status != 9U) {
                BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
            } else {
                int16_t distance = result->distance_mm[cell];
                uint16_t color = map_color_rgb565(((float)distance) / 4000.0f);
                BSP_LCD_SetTextColor(color);
            }
            BSP_LCD_FillRect(x * 30, y * 30, 30, 30);
        }
    }
}
