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
#include "tof_io.h"
#include <stdbool.h>

/* Includes ------------------------------------------------------------------*/
#include "app_tof.h"
#include "main.h"
#include <stdio.h>

#include <ranging_sensor.h>
#include <vl53l8cx.h>

// #include "app_tof_pin_conf.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define TIMING_BUDGET (30U) /* 5 ms < TimingBudget < 100 ms */
#define RANGING_FREQUENCY                                                      \
  (10U) /* Ranging frequency Hz (shall be consistent with TimingBudget value)  \
         */
#define POLLING_PERIOD (1)

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart6;

static VL53L8CX_Object_t sensor;
static VL53L8CX_Capabilities_t cap;
static VL53L8CX_ProfileConfig_t profile;
static VL53L8CX_Result_t result;


static int32_t status = 0;
volatile uint8_t ToF_EventDetected = 0;

/* Private function prototypes -----------------------------------------------*/
static void MX_53L8A1_SimpleRanging_Init(void);
static void MX_53L8A1_SimpleRanging_Process(void);
static void print_result(VL53L8CX_Result_t *Result);
static void toggle_resolution(void);
static void toggle_signal_and_ambient(void);
static void clear_screen(void);
static void display_commands_banner(void);
static void handle_cmd(uint8_t cmd);
static uint8_t get_key(void);
static uint32_t com_has_data(void);
static bool VL53L8CX_Probe(void);

void MX_TOF_Init(void) {
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN TOF_Init_PreTreatment */

  /* USER CODE END TOF_Init_PreTreatment */

  /* Initialize the peripherals and the TOF components */

  MX_53L8A1_SimpleRanging_Init();

  /* USER CODE BEGIN TOF_Init_PostTreatment */

  /* USER CODE END TOF_Init_PostTreatment */
}

/*
 * LM background task
 */
void MX_TOF_Process(void) {
  /* USER CODE BEGIN TOF_Process_PreTreatment */

  /* USER CODE END TOF_Process_PreTreatment */

  MX_53L8A1_SimpleRanging_Process();

  /* USER CODE BEGIN TOF_Process_PostTreatment */

  /* USER CODE END TOF_Process_PostTreatment */
}

static void MX_53L8A1_SimpleRanging_Init(void) {
  /* Initialize Virtual COM Port */
  // BSP_COM_Init(COM1);

  /* Initialize button */
  // BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Sensor reset */
  // HAL_GPIO_WritePin(VL53L8A1_PWR_EN_C_PORT, VL53L8A1_PWR_EN_C_PIN,
  // GPIO_PIN_RESET); HAL_Delay(2); HAL_GPIO_WritePin(VL53L8A1_PWR_EN_C_PORT,
  // VL53L8A1_PWR_EN_C_PIN, GPIO_PIN_SET); HAL_Delay(2);
  // HAL_GPIO_WritePin(VL53L8A1_LPn_C_PORT, VL53L8A1_LPn_C_PIN, GPIO_PIN_RESET);
  // HAL_Delay(2);
  // HAL_GPIO_WritePin(VL53L8A1_LPn_C_PORT, VL53L8A1_LPn_C_PIN, GPIO_PIN_SET);
  // HAL_Delay(2);

  printf("\033[2H\033[2J");
  printf("53L8A1 Simple Ranging demo application\n");
  printf("Sensor initialization...\n");

  if (!VL53L8CX_Probe()) {
    printf("VL53L8A1_VL53L8CX_Init failed\n");
  }
}

static void MX_53L8A1_SimpleRanging_Process(void) {
  uint32_t Id;

  VL53L8CX_ReadID(&sensor, &Id);
  VL53L8CX_GetCapabilities(&sensor, &cap);

  profile.RangingProfile = VL53L8CX_PROFILE_4x4_CONTINUOUS;
  profile.TimingBudget = TIMING_BUDGET;
  profile.Frequency =
      RANGING_FREQUENCY;     /* Ranging frequency Hz (shall be consistent with
                                TimingBudget value) */
  profile.EnableAmbient = 0; /* Enable: 1, Disable: 0 */
  profile.EnableSignal = 0;  /* Enable: 1, Disable: 0 */

  /* set the profile if different from default one */
  VL53L8CX_ConfigProfile(&sensor, &profile);

  status = VL53L8CX_Start(&sensor,
                                         VL53L8CX_MODE_BLOCKING_CONTINUOUS);

  if (status != VL53L8CX_OK) {
    printf("VL53L8A1_VL53L8CX_Start failed\n");
    while (1)
      ;
  }

  while (1) {
    /* polling mode */
    status = VL53L8CX_GetDistance(&sensor, &result);

    if (status == VL53L8CX_OK) {
      print_result(&result);
    }

    if (com_has_data()) {
      handle_cmd(get_key());
    }

    HAL_Delay(POLLING_PERIOD);
  }
}

static void print_result(VL53L8CX_Result_t *Result) {
  int8_t i;
  int8_t j;
  int8_t k;
  int8_t l;
  uint8_t zones_per_line;

  zones_per_line = ((profile.RangingProfile == VL53L8CX_PROFILE_8x8_AUTONOMOUS) ||
                    (profile.RangingProfile == VL53L8CX_PROFILE_8x8_CONTINUOUS))
                       ? 8
                       : 4;

  display_commands_banner();

  printf("Cell Format :\n\n");
  for (l = 0; l < VL53L8CX_NB_TARGET_PER_ZONE; l++) {
    printf(" \033[38;5;10m%20s\033[0m : %20s\n", "Distance [mm]", "Status");
    if ((profile.EnableAmbient != 0) || (profile.EnableSignal != 0)) {
      printf(" %20s : %20s\n", "Signal [kcps/spad]", "Ambient [kcps/spad]");
    }
  }

  printf("\n\n");

  for (j = 0; j < Result->NumberOfZones; j += zones_per_line) {
    for (i = 0; i < zones_per_line; i++) /* number of zones per line */
    {
      printf(" -----------------");
    }
    printf("\n");

    for (i = 0; i < zones_per_line; i++) {
      printf("|                 ");
    }
    printf("|\n");

    for (l = 0; l < VL53L8CX_NB_TARGET_PER_ZONE; l++) {
      /* Print distance and status */
      for (k = (zones_per_line - 1); k >= 0; k--) {
        if (Result->ZoneResult[j + k].NumberOfTargets > 0)
          printf("| \033[38;5;10m%5ld\033[0m  :  %5ld ",
                 (long)Result->ZoneResult[j + k].Distance[l],
                 (long)Result->ZoneResult[j + k].Status[l]);
        else
          printf("| %5s  :  %5s ", "X", "X");
      }
      printf("|\n");

      if ((profile.EnableAmbient != 0) || (profile.EnableSignal != 0)) {
        /* Print Signal and Ambient */
        for (k = (zones_per_line - 1); k >= 0; k--) {
          if (Result->ZoneResult[j + k].NumberOfTargets > 0) {
            if (profile.EnableSignal != 0) {
              printf("| %5ld  :  ", (long)Result->ZoneResult[j + k].Signal[l]);
            } else
              printf("| %5s  :  ", "X");

            if (profile.EnableAmbient != 0) {
              printf("%5ld ", (long)Result->ZoneResult[j + k].Ambient[l]);
            } else
              printf("%5s ", "X");
          } else
            printf("| %5s  :  %5s ", "X", "X");
        }
        printf("|\n");
      }
    }
  }

  for (i = 0; i < zones_per_line; i++) {
    printf(" -----------------");
  }
  printf("\n");
}

static void toggle_resolution(void) {
  VL53L8CX_Stop(&sensor);

  switch (profile.RangingProfile) {
  case VL53L8CX_PROFILE_4x4_AUTONOMOUS:
    profile.RangingProfile = VL53L8CX_PROFILE_8x8_AUTONOMOUS;
    break;

  case VL53L8CX_PROFILE_4x4_CONTINUOUS:
    profile.RangingProfile = VL53L8CX_PROFILE_8x8_CONTINUOUS;
    break;

  case VL53L8CX_PROFILE_8x8_AUTONOMOUS:
    profile.RangingProfile = VL53L8CX_PROFILE_4x4_AUTONOMOUS;
    break;

  case VL53L8CX_PROFILE_8x8_CONTINUOUS:
    profile.RangingProfile = VL53L8CX_PROFILE_4x4_CONTINUOUS;
    break;

  default:
    break;
  }

  VL53L8CX_ConfigProfile(&sensor, &profile);
  VL53L8CX_Start(&sensor,
                                VL53L8CX_MODE_BLOCKING_CONTINUOUS);
}

static void toggle_signal_and_ambient(void) {
  VL53L8CX_Stop(&sensor);

  profile.EnableAmbient = (profile.EnableAmbient) ? 0U : 1U;
  profile.EnableSignal = (profile.EnableSignal) ? 0U : 1U;

  VL53L8CX_ConfigProfile(&sensor, &profile);
  VL53L8CX_Start(&sensor,
                                VL53L8CX_MODE_BLOCKING_CONTINUOUS);
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

static bool VL53L8CX_Probe(void) {
  bool ret;
  VL53L8CX_IO_t IOCtx;
  uint32_t id;

  /* Configure the ranging sensor driver */
  IOCtx.Address = 0;
  IOCtx.Init = TOFIO_Init;
  IOCtx.DeInit = TOFIO_DeInit;
  IOCtx.WriteReg = TOFIO_WriteReg;
  IOCtx.ReadReg = TOFIO_ReadReg;
  IOCtx.GetTick = TOFIO_GetTick;

  if (VL53L8CX_RegisterBusIO(&sensor, &IOCtx) != VL53L8CX_OK) {
    ret = false;
  } else {
    if (VL53L8CX_ReadID(&sensor, &id) != VL53L8CX_OK) {
      ret = false;
    } else if (id != VL53L8CX_ID) {
      ret = false;
    } else if (VL53L8CX_Init(
                   &sensor) != VL53L8CX_OK) {
      ret = false;
    } else if (VL53L8CX_GetCapabilities(
                   &sensor,
                   &cap) != VL53L8CX_OK) {
      ret = false;
    } else {
      ret = true;
    }
  }

  return ret;
}
