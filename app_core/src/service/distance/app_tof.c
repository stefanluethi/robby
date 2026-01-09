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

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_spi.h"
#include <stdbool.h>

/* Includes ------------------------------------------------------------------*/
#include "app_tof.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>

#include <vl53l8cx_api.h>

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
// static VL53L8CX_Object_t sensor;
// static VL53L8CX_Capabilities_t cap;
// static VL53L8CX_ProfileConfig_t profile;
static VL53L8CX_ResultsData results;
uint8_t data_ready;
uint8_t resolution;

static int32_t status = 0;
volatile uint8_t ToF_EventDetected = 0;

/* Private function prototypes -----------------------------------------------*/
static void MX_53L8A1_SimpleRanging_Init(void);
static void MX_53L8A1_SimpleRanging_Process(void);
static void print_result(VL53L8CX_ResultsData *Result);
static void toggle_resolution(void);
static void toggle_signal_and_ambient(void);
static void clear_screen(void);
static void display_commands_banner(void);
static void handle_cmd(uint8_t cmd);
static uint8_t get_key(void);
static uint32_t com_has_data(void);

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
        printf("Confiuring TOF failed with status %d\n", status);
    }

    printf("Ranging starts\n");
    status = vl53l8cx_start_ranging(&device);

    uint8_t data_ready;
    while (1) {
        /* polling mode */
        status = vl53l8cx_check_data_ready(&device, &data_ready);
        if (data_ready) {
            status = vl53l8cx_get_resolution(&device, &resolution);
            status = vl53l8cx_get_ranging_data(&device, &results);

            print_result(&results);
        }

        if (com_has_data()) {
            handle_cmd(get_key());
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

    for (j = 0; j < 64; j += zones_per_line) {
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

static void toggle_resolution(void) {
    //   VL53L8CX_Stop(&sensor);

    //   switch (profile.RangingProfile) {
    //   case VL53L8CX_PROFILE_4x4_AUTONOMOUS:
    //     profile.RangingProfile = VL53L8CX_PROFILE_8x8_AUTONOMOUS;
    //     break;

    //   case VL53L8CX_PROFILE_4x4_CONTINUOUS:
    //     profile.RangingProfile = VL53L8CX_PROFILE_8x8_CONTINUOUS;
    //     break;

    //   case VL53L8CX_PROFILE_8x8_AUTONOMOUS:
    //     profile.RangingProfile = VL53L8CX_PROFILE_4x4_AUTONOMOUS;
    //     break;

    //   case VL53L8CX_PROFILE_8x8_CONTINUOUS:
    //     profile.RangingProfile = VL53L8CX_PROFILE_4x4_CONTINUOUS;
    //     break;

    //   default:
    //     break;
    //   }

    //   VL53L8CX_ConfigProfile(&sensor, &profile);
    //   VL53L8CX_Start(&sensor,
    //                                 VL53L8CX_MODE_BLOCKING_CONTINUOUS);
}

static void toggle_signal_and_ambient(void) {
    //   VL53L8CX_Stop(&sensor);

    //   profile.EnableAmbient = (profile.EnableAmbient) ? 0U : 1U;
    //   profile.EnableSignal = (profile.EnableSignal) ? 0U : 1U;

    //   VL53L8CX_ConfigProfile(&sensor, &profile);
    //   VL53L8CX_Start(&sensor,
    //                                 VL53L8CX_MODE_BLOCKING_CONTINUOUS);
}

static void clear_screen(void) { printf("%c[2J", 27); /* 27 is ESC command */ }

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

static void handle_cmd(uint8_t cmd) {
    switch (cmd) {
    case 'r':
        toggle_resolution();
        clear_screen();
        break;

    case 's':
        toggle_signal_and_ambient();
        clear_screen();
        break;

    case 'c':
        clear_screen();
        break;

    default:
        break;
    }
}

static uint8_t get_key(void) {
    uint8_t cmd = 0;

    HAL_UART_Receive(&huart6, &cmd, 1, HAL_MAX_DELAY);

    return cmd;
}

static uint32_t com_has_data(void) {
    return __HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE);
    ;
}
