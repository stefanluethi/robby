
#include "distance_visualizer.h"
#include "main.h"
#include "rtos/Task.h"
#include "stm32f7xx_hal_gpio.h"
#include "stm32f7xx_hal_uart.h"
#include "util/colormap.h"

#include "service/imu.h"

#include <cbor.h>
#include <stm32f723e_discovery_lcd.h>
#include <stm32f7xx_hal.h>

extern "C" {
#include <vl53l8cx_api.h>
}

#include <stddef.h>
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

extern UART_HandleTypeDef huart2;

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
static size_t serialize_distmap(uint8_t* buffer, const VL53L8CX_ResultsData* sensor_results);

static uint8_t message_buffer[1024];

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
        vl53l8cx_start_ranging(&devices[i]);
    }

    while (1) {
        for (uint8_t i = 0; i < CONF_N_SENSORS; ++i) {
            trigger_sensor(i);

            uint32_t conversion_timeout = 500U;
            while (!data_ready && --conversion_timeout) {
                rtos::Task::sleep(CONF_POLLING_PERIOD);
            }
            data_ready = false;
        }

        for (uint8_t i = 0; i < CONF_N_SENSORS; ++i) {
            vl53l8cx_get_ranging_data(&devices[i], &results[i]);
        }
        draw_results();
        size_t length = serialize_distmap(message_buffer, results);
        HAL_UART_Transmit(&huart2, message_buffer, length, HAL_MAX_DELAY);

        IMU_process();
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
                        util::map_color_rgb565(((float)distance) / MAX_DISTANCE_MM);
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
        HAL_GPIO_WritePin(TOF_SYNC1_GPIO_Port, TOF_SYNC1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(TOF_SYNC1_GPIO_Port, TOF_SYNC1_Pin, GPIO_PIN_RESET);
        break;
    case SENSOR_MIDDLE:
        HAL_GPIO_WritePin(TOF_SYNC2_GPIO_Port, TOF_SYNC2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(TOF_SYNC2_GPIO_Port, TOF_SYNC2_Pin, GPIO_PIN_RESET);
        break;
    case SENSOR_LEFT:
        HAL_GPIO_WritePin(TOF_SYNC3_GPIO_Port, TOF_SYNC3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(TOF_SYNC3_GPIO_Port, TOF_SYNC3_Pin, GPIO_PIN_RESET);
        break;
    }
}

size_t serialize_distmap(uint8_t* buffer, const VL53L8CX_ResultsData* sensor_results)
{
    CborEncoder encoder;
    CborEncoder root_map;
    CborEncoder payload_map;
    CborEncoder distance_map;
    CborEncoder array_x;
    CborEncoder array_y;
    
    cbor_encoder_init(&encoder, message_buffer, sizeof(message_buffer), 0);
    cbor_encoder_create_map(&encoder, &root_map, 1U);

    cbor_encode_text_stringz(&root_map, "payload");
    cbor_encoder_create_map(&root_map, &payload_map, 1);

    cbor_encode_text_stringz(&payload_map, "DistanceMap");
    cbor_encoder_create_map(&payload_map, &distance_map, 1);
    
    cbor_encode_text_stringz(&distance_map, "distances_mm");
    cbor_encoder_create_array(&distance_map, &array_x, 24);
    for (size_t i = 0; i < 3; ++i) {
        for (size_t x = 0; x < 8; ++x) {
            cbor_encoder_create_array(&array_x, &array_y, 8);
            for (size_t y = 0; y < 8; ++y) {
                uint8_t status = sensor_results[2 - i].target_status[x + y * SENSOR_RESOLUTION_X];
                int16_t distance = -1;
                if (status == 5U || status == 9U) {
                    distance = sensor_results[2 - i].distance_mm[x + y * SENSOR_RESOLUTION_X];
                }
                cbor_encode_uint(&array_y, distance >= 0 ? distance : UINT16_MAX);
            }
            cbor_encoder_close_container(&array_x, &array_y);
        }
    }

    cbor_encoder_close_container(&distance_map, &array_x);
    cbor_encoder_close_container(&payload_map, &distance_map);
    cbor_encoder_close_container(&root_map, &payload_map);
    CborError result = cbor_encoder_close_container(&encoder, &root_map);

    if (result != CborNoError) {
        return 0U;
    } else {
        return cbor_encoder_get_buffer_size(&encoder, buffer);
    }
}


// A1                                   # map(1)
//    67                                # text(7)
//       7061796C6F6164                 # "payload"
//    A1                                # map(1)
//       6B                             # text(11)
//          44697374616E63654D6170      # "DistanceMap"
//       A1                             # map(1)
//          6C                          # text(12)
//             64697374616E6365735F6D6D # "distances_mm"
//          98 18                       # array(24)
//             88                       # array(8)
//                19 0A21               # unsigned(2593)
//                19 0A0F               # unsigned(2575)
//                19 0206               # unsigned(518)
//                19 0D9E               # unsigned(3486)
//                19 0820               # unsigned(2080)
//                19 0C7F               # unsigned(3199)
//                19 0C8E               # unsigned(3214)
//                19 0C3D               # unsigned(3133)
//             88                       # array(8)
//                19 0C32               # unsigned(3122)
//                19 0197               # unsigned(407)
//                19 0DD9               # unsigned(3545)
//                19 0536               # unsigned(1334)
//                19 08AC               # unsigned(2220)
//                19 0ED3               # unsigned(3795)
//                19 0E6E               # unsigned(3694)
//                19 046E               # unsigned(1134)
//             88                       # array(8)
//                19 0603               # unsigned(1539)
//                19 0833               # unsigned(2099)
//                19 0856               # unsigned(2134)
//                19 0E1C               # unsigned(3612)
//                19 037D               # unsigned(893)
//                19 0DC4               # unsigned(3524)
//                19 0811               # unsigned(2065)
//                19 01A2               # unsigned(418)
//             88                       # array(8)
//                19 053B               # unsigned(1339)
//                19 025F               # unsigned(607)
//                19 0B78               # unsigned(2936)
//                19 06BE               # unsigned(1726)
//                19 060B               # unsigned(1547)
//                19 0353               # unsigned(851)
//                19 07AC               # unsigned(1964)
//                19 0748               # unsigned(1864)
//             88                       # array(8)
//                19 0227               # unsigned(551)
//                19 0DAA               # unsigned(3498)
//                19 0F49               # unsigned(3913)
//                19 0478               # unsigned(1144)
//                19 0102               # unsigned(258)
//                19 0317               # unsigned(791)
//                19 09CB               # unsigned(2507)
//                18 49                 # unsigned(73)
//             88                       # array(8)
//                18 20                 # unsigned(32)
//                19 0CE3               # unsigned(3299)
//                19 0257               # unsigned(599)
//                19 0110               # unsigned(272)
//                19 0804               # unsigned(2052)
//                19 0715               # unsigned(1813)
//                19 0566               # unsigned(1382)
//                19 0F6B               # unsigned(3947)
//             88                       # array(8)
//                19 0874               # unsigned(2164)
//                19 0DA5               # unsigned(3493)
//                19 0394               # unsigned(916)
//                19 09BD               # unsigned(2493)
//                19 0BEB               # unsigned(3051)
//                19 03F2               # unsigned(1010)
//                19 09AC               # unsigned(2476)
//                19 01D8               # unsigned(472)
//             88                       # array(8)
//                18 34                 # unsigned(52)
//                19 03CD               # unsigned(973)
//                19 03B6               # unsigned(950)
//                19 05F4               # unsigned(1524)
//                19 09A3               # unsigned(2467)
//                19 0415               # unsigned(1045)
//                19 0BFC               # unsigned(3068)
//                18 CC                 # unsigned(204)
//             88                       # array(8)
//                19 0B8F               # unsigned(2959)
//                19 0E9A               # unsigned(3738)
//                19 06C4               # unsigned(1732)
//                19 0544               # unsigned(1348)
//                19 04FC               # unsigned(1276)
//                19 020C               # unsigned(524)
//                19 0ECF               # unsigned(3791)
//                19 0515               # unsigned(1301)
//             88                       # array(8)
//                19 04E4               # unsigned(1252)
//                18 5B                 # unsigned(91)
//                19 0496               # unsigned(1174)
//                19 08EA               # unsigned(2282)
//                19 01CD               # unsigned(461)
//                19 088E               # unsigned(2190)
//                19 0D59               # unsigned(3417)
//                19 0D7C               # unsigned(3452)
//             88                       # array(8)
//                19 0F88               # unsigned(3976)
//                19 056C               # unsigned(1388)
//                19 04E6               # unsigned(1254)
//                19 01F1               # unsigned(497)
//                19 025E               # unsigned(606)
//                19 0939               # unsigned(2361)
//                19 0A78               # unsigned(2680)
//                19 02FF               # unsigned(767)
//             88                       # array(8)
//                19 03C4               # unsigned(964)
//                18 39                 # unsigned(57)
//                19 09CE               # unsigned(2510)
//                19 04CE               # unsigned(1230)
//                19 04C4               # unsigned(1220)
//                19 01AB               # unsigned(427)
//                19 0641               # unsigned(1601)
//                19 0350               # unsigned(848)
//             88                       # array(8)
//                19 0EAF               # unsigned(3759)
//                19 0F8A               # unsigned(3978)
//                19 01EE               # unsigned(494)
//                19 0A97               # unsigned(2711)
//                19 02EE               # unsigned(750)
//                19 0BAC               # unsigned(2988)
//                19 06D2               # unsigned(1746)
//                19 0460               # unsigned(1120)
//             88                       # array(8)
//                19 0D64               # unsigned(3428)
//                19 056E               # unsigned(1390)
//                19 03A4               # unsigned(932)
//                19 02F9               # unsigned(761)
//                19 0896               # unsigned(2198)
//                19 05D7               # unsigned(1495)
//                19 0C39               # unsigned(3129)
//                19 0112               # unsigned(274)
//             88                       # array(8)
//                19 09F2               # unsigned(2546)
//                19 047F               # unsigned(1151)
//                19 0587               # unsigned(1415)
//                19 03E2               # unsigned(994)
//                19 01E8               # unsigned(488)
//                19 0C4D               # unsigned(3149)
//                19 0F24               # unsigned(3876)
//                19 04EA               # unsigned(1258)
//             88                       # array(8)
//                19 04CF               # unsigned(1231)
//                19 0E39               # unsigned(3641)
//                19 0BF3               # unsigned(3059)
//                19 0B1C               # unsigned(2844)
//                19 0CC5               # unsigned(3269)
//                19 0E05               # unsigned(3589)
//                19 094C               # unsigned(2380)
//                19 0175               # unsigned(373)
//             88                       # array(8)
//                19 0BB3               # unsigned(2995)
//                19 0868               # unsigned(2152)
//                19 0D1C               # unsigned(3356)
//                19 01EF               # unsigned(495)
//                19 0BE9               # unsigned(3049)
//                19 07B1               # unsigned(1969)
//                19 0C48               # unsigned(3144)
//                19 0A80               # unsigned(2688)
//             88                       # array(8)
//                19 0CF3               # unsigned(3315)
//                19 050B               # unsigned(1291)
//                19 0AA6               # unsigned(2726)
//                19 0B1B               # unsigned(2843)
//                19 0A4F               # unsigned(2639)
//                19 0F5A               # unsigned(3930)
//                19 0D66               # unsigned(3430)
//                19 0803               # unsigned(2051)
//             88                       # array(8)
//                19 0427               # unsigned(1063)
//                18 65                 # unsigned(101)
//                19 0E89               # unsigned(3721)
//                19 0F12               # unsigned(3858)
//                19 03AB               # unsigned(939)
//                19 0542               # unsigned(1346)
//                19 07F9               # unsigned(2041)
//                19 0405               # unsigned(1029)
//             88                       # array(8)
//                18 96                 # unsigned(150)
//                19 0777               # unsigned(1911)
//                19 0390               # unsigned(912)
//                19 036B               # unsigned(875)
//                19 08D2               # unsigned(2258)
//                19 0B9B               # unsigned(2971)
//                18 49                 # unsigned(73)
//                19 0A12               # unsigned(2578)
//             88                       # array(8)
//                19 080F               # unsigned(2063)
//                19 0832               # unsigned(2098)
//                19 0820               # unsigned(2080)
//                19 03DA               # unsigned(986)
//                19 01CB               # unsigned(459)
//                19 0653               # unsigned(1619)
//                19 0E4B               # unsigned(3659)
//                19 0659               # unsigned(1625)
//             88                       # array(8)
//                19 06C8               # unsigned(1736)
//                19 0CB1               # unsigned(3249)
//                19 0373               # unsigned(883)
//                19 040E               # unsigned(1038)
//                19 0496               # unsigned(1174)
//                19 02B8               # unsigned(696)
//                19 05B8               # unsigned(1464)
//                19 0D64               # unsigned(3428)
//             88                       # array(8)
//                19 05CB               # unsigned(1483)
//                19 0330               # unsigned(816)
//                19 0431               # unsigned(1073)
//                19 0A28               # unsigned(2600)
//                19 0375               # unsigned(885)
//                19 0D9D               # unsigned(3485)
//                19 024A               # unsigned(586)
//                19 0342               # unsigned(834)
//             88                       # array(8)
//                19 01A7               # unsigned(423)
//                19 0BF5               # unsigned(3061)
//                19 0262               # unsigned(610)
//                19 0D08               # unsigned(3336)
//                19 063B               # unsigned(1595)
//                19 0A4E               # unsigned(2638)
//                19 0E6E               # unsigned(3694)
//                19 043E               # unsigned(1086)