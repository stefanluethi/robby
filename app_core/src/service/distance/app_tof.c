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
 #include "util/colormap.h"
 
 #include <stm32f7xx_hal.h>
 #include <stm32f723e_discovery_lcd.h>
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
uint8_t data_ready;

static int32_t status = 0;
volatile uint8_t ToF_EventDetected = 0;

/* Private function prototypes -----------------------------------------------*/
static void MX_53L8A1_SimpleRanging_Init(void);
static void MX_53L8A1_SimpleRanging_Process(void);
static void print_result(VL53L8CX_ResultsData *Result);
static void draw_results(VL53L8CX_ResultsData *result);
static void display_commands_banner(void);

void MX_TOF_Init(void) { MX_53L8A1_SimpleRanging_Init(); }

/*
 * LM background task
 */
void MX_TOF_Process(void) { MX_53L8A1_SimpleRanging_Process(); }

static void MX_53L8A1_SimpleRanging_Init(void) {
    printf("\033[2H\033[2J");
    printf("53L8A1 Simple Ranging demo application\n");
    printf("Sensor initialization...\n");

    if (vl53l8cx_init(&device)) {
        printf("vl53l8cx_init failed\n");
    }
}

static void MX_53L8A1_SimpleRanging_Process(void) {
    status = vl53l8cx_set_ranging_frequency_hz(&device, 10);
    status |=
        vl53l8cx_set_ranging_mode(&device, VL53L8CX_RANGING_MODE_AUTONOMOUS);
    status |= vl53l8cx_set_resolution(&device, VL53L8CX_RESOLUTION_8X8);

    // uint8_t vl53l8cx_set_external_sync_pin_enable(
    if (status) {
        printf("Confiuring TOF failed with status %ld\n", status);
    }

    printf("Ranging starts\n");
    status = vl53l8cx_start_ranging(&device);

    uint8_t data_ready;
    while (1) {
        /* polling mode */
        status = vl53l8cx_check_data_ready(&device, &data_ready);
        if (data_ready) {
            status = vl53l8cx_get_ranging_data(&device, &results);

            // print_result(&results);
            draw_results(&results);
        }

        HAL_Delay(POLLING_PERIOD);
    }
}

static void print_result(VL53L8CX_ResultsData *result) {
    int8_t i;
    int8_t j;
    int8_t k;
    uint8_t zones_per_line;

    zones_per_line = 8;

    display_commands_banner();

    printf("Cell Format :\n\n");
    printf(" \033[38;5;10m%20s\033[0m : %20s\n", "Distance [mm]", "Status");

    printf("\n\n");

    for (j = 0; j < VL53L8CX_RESOLUTION_8X8; j += zones_per_line) {
        for (i = 0; i < zones_per_line; i++) /* number of zones per line */
        {
            printf(" -----------------");
        }
        printf("\n");

        for (i = 0; i < zones_per_line; i++) {
            printf("|                 ");
        }
        printf("|\n");

        /* Print distance and status */
        for (k = (zones_per_line - 1); k >= 0; k--) {
            if (result->nb_target_detected[j + k] > 0) {
                printf("| \033[38;5;10m%5ld\033[0m  :  %5ld ",
                       (long)result->distance_mm[j + k],
                       (long)result->target_status[j + k]);
            } else
                printf("| %5s  :  %5s ", "X", "X");
        }
        printf("|\n");
    }

    for (i = 0; i < zones_per_line; i++) {
        printf(" -----------------");
    }
    printf("\n");
}

static void draw_results(VL53L8CX_ResultsData *result) {
    // pixel (0,0) is top left on the screen while it's bottom left on the sensor
    for (uint8_t y = 0; y < 8U; ++y) {
        for (uint8_t x = 0; x < 8U; ++x) {
            size_t cell = x + (7U - y) * 8U;

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

static void display_commands_banner(void) {
    /* clear screen */
    printf("%c[2H", 27);

    printf("53L8A1 Simple Ranging demo application\n");
    printf("--------------------------------------\n\n");

    printf("Use the following keys to control application\n");
    printf(" 'r' : change resolution\n");
    printf(" 's' : enable signal and ambient\n");
    printf(" 'c' : clear screen\n");
    printf("\n");
}

