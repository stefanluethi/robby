/**
 ******************************************************************************
 * @file          : app_tof.c
 * @author        : IMG SW Application Team
 * @brief         : This file provides code for the configuration
 *                  of the STMicroelectronics.X-CUBE-TOF1.3.4.2 instances.
 ******************************************************************************
 *
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "app_tof.h"
#include "main.h"
#include "stm32f7xx_hal_gpio.h"
#include "util/colormap.h"

#include <stm32f723e_discovery_lcd.h>
#include <stm32f7xx_hal.h>
#include <vl53l8cx_api.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// #include "app_tof_pin_conf.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TIMING_BUDGET (30U) /* 5 ms < TimingBudget < 100 ms */
#define RANGING_FREQUENCY                                                      \
    (10U) /* Ranging frequency Hz (shall be consistent with TimingBudget       \
           * value)                                                            \
           */
#define POLLING_PERIOD (1)

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart6;

static VL53L8CX_Configuration device;
static VL53L8CX_ResultsData results;
bool data_ready;


/* Private function prototypes -----------------------------------------------*/
static void MX_53L8A1_SimpleRanging_Init(void);
static void MX_53L8A1_SimpleRanging_Process(void);
static void draw_results(VL53L8CX_ResultsData *result);

void MX_TOF_Init(void) { MX_53L8A1_SimpleRanging_Init(); }

/*
 * LM background task
 */
void MX_TOF_Process(void) { MX_53L8A1_SimpleRanging_Process(); }

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

void TOF_ConversionDoneCallback(void)
{
    data_ready = true;
}

static void draw_results(VL53L8CX_ResultsData *result) {
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
