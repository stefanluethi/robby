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

#include <cstdint>
#include <cstdio>

extern "C" {
#include <vl53l8cx_api.h>
}

#define CONF_POLLING_PERIOD (1U)
#define CONF_SENSOR_FREQUENCY_HZ (10U)
#define CONF_N_SENSORS (3U)
#define SENSOR_INTEGRATION_TIME_MS (10U)

#define MAX_DISTANCE_MM (4000.0F)
#define SENSOR_RESOLUTION (VL53L8CX_RESOLUTION_8X8)
#define SENSOR_RESOLUTION_X (8U)
#define SENSOR_RESOLUTION_Y (8U)

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

typedef enum {
    SENSOR_RIGHT = 0U,
    SENSOR_MIDDLE = 1U,
    SENSOR_LEFT = 2U,
} DistanceSensor;

void DistanceVisualizer::start()
{
    LOG_INFO(_log, "Sensor initialization...");

    int32_t status = 0;
    for (uint8_t i = 0; i < CONF_N_SENSORS; ++i) {
        _devices[i].platform.address = i;

        status |= vl53l8cx_init(&_devices[i]);
    }

    if (status) {
        LOG_WARN(_log, "vl53l8cx_init failed");
    }

    // start
    for (uint8_t i = 0; i < CONF_N_SENSORS; ++i) {
        bool failed = setup_sensor(&_devices[i]);
        if (failed) {
            LOG_WARN(_log, "Distance sensor %d setup failed", i);
            return;
        }
    }

    LOG_INFO(_log, "Ranging starts");
    for (uint8_t i = 0; i < CONF_N_SENSORS; ++i) {
        vl53l8cx_start_ranging(&_devices[i]);
    }
}

void DistanceVisualizer::process()
{
    for (uint8_t i = 0; i < CONF_N_SENSORS; ++i) {
        trigger_sensor(static_cast<DistanceSensor>(i));

        uint32_t conversion_timeout = 500U;
        while (!_data_ready && --conversion_timeout) {
            rtos::Task::sleep(CONF_POLLING_PERIOD);
        }
        _data_ready = false;
    }

    for (uint8_t i = 0; i < CONF_N_SENSORS; ++i) {
        vl53l8cx_get_ranging_data(&_devices[i], &_results[i]);
    }

    draw_results();

    size_t length = serialize_distmap(_message_buffer, _results);
    HAL_UART_Transmit(&huart2, _message_buffer, length, HAL_MAX_DELAY);
}

void DistanceVisualizer::conversion_done_callback()
{
    _data_ready = true;
}

bool DistanceVisualizer::setup_sensor(VL53L8CX_Configuration* device)
{
    int32_t status =
        vl53l8cx_set_ranging_frequency_hz(device, CONF_SENSOR_FREQUENCY_HZ);
    status |=
        vl53l8cx_set_ranging_mode(device, VL53L8CX_RANGING_MODE_AUTONOMOUS);
    status |= vl53l8cx_set_resolution(device, SENSOR_RESOLUTION);
    status |= vl53l8cx_set_integration_time_ms(device, SENSOR_INTEGRATION_TIME_MS);
    status |= vl53l8cx_set_external_sync_pin_enable(device, true);
    return status != 0;
}

void DistanceVisualizer::draw_results()
{
    for (size_t sensor = 0; sensor < CONF_N_SENSORS; ++sensor) {
        uint16_t offset_x = static_cast<uint16_t>((CONF_N_SENSORS - sensor - 1U) * 84U);
        uint16_t offset_y = 80U;

        for (uint8_t y = 0; y < SENSOR_RESOLUTION_Y; ++y) {
            for (uint8_t x = 0; x < SENSOR_RESOLUTION_X; ++x) {
                size_t cell = x + y * SENSOR_RESOLUTION_X;

                uint8_t status = _results[sensor].target_status[cell];
                if (status != 5U && status != 9U) {
                    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
                } else {
                    int16_t distance = _results[sensor].distance_mm[cell];
                    uint16_t color =
                        util::map_color_rgb565(static_cast<float>(distance) / MAX_DISTANCE_MM);
                    BSP_LCD_SetTextColor(color);
                }

                BSP_LCD_FillRect(x * 9U + offset_x, y * 9U + offset_y, 9U, 9U);
            }
        }
    }
}

void DistanceVisualizer::trigger_sensor(DistanceSensor sensor)
{
    switch (sensor) {
    case DistanceSensor::RIGHT:
        HAL_GPIO_WritePin(TOF_SYNC1_GPIO_Port, TOF_SYNC1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(TOF_SYNC1_GPIO_Port, TOF_SYNC1_Pin, GPIO_PIN_RESET);
        break;

    case DistanceSensor::MIDDLE:
        HAL_GPIO_WritePin(TOF_SYNC2_GPIO_Port, TOF_SYNC2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(TOF_SYNC2_GPIO_Port, TOF_SYNC2_Pin, GPIO_PIN_RESET);
        break;

    case DistanceSensor::LEFT:
        HAL_GPIO_WritePin(TOF_SYNC3_GPIO_Port, TOF_SYNC3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(TOF_SYNC3_GPIO_Port, TOF_SYNC3_Pin, GPIO_PIN_RESET);
        break;
    }
}

size_t DistanceVisualizer::serialize_distmap(
    uint8_t* buffer,
    const VL53L8CX_ResultsData* sensor_results)
{
    CborEncoder encoder;
    CborEncoder root_map;
    CborEncoder payload_map;
    CborEncoder distance_map;
    CborEncoder array_x;
    CborEncoder array_y;

    cbor_encoder_init(&encoder, buffer, sizeof(_message_buffer), 0);
    cbor_encoder_create_map(&encoder, &root_map, 1U);

    cbor_encode_text_stringz(&root_map, "payload");
    cbor_encoder_create_map(&root_map, &payload_map, 1U);

    cbor_encode_text_stringz(&payload_map, "DistanceMap");
    cbor_encoder_create_map(&payload_map, &distance_map, 1U);

    cbor_encode_text_stringz(&distance_map, "distances_mm");
    cbor_encoder_create_array(&distance_map, &array_x, CONF_N_SENSORS * SENSOR_RESOLUTION_X);

    for (size_t i = 0; i < CONF_N_SENSORS; ++i) {
        for (size_t x = 0; x < SENSOR_RESOLUTION_X; ++x) {
            cbor_encoder_create_array(&array_x, &array_y, SENSOR_RESOLUTION_Y);

            for (size_t y = 0; y < SENSOR_RESOLUTION_Y; ++y) {
                size_t sensor_index = CONF_N_SENSORS - 1U - i;
                size_t cell = x + y * SENSOR_RESOLUTION_X;

                uint8_t status = sensor_results[sensor_index].target_status[cell];
                int16_t distance = -1;

                if (status == 5U || status == 9U) {
                    distance = sensor_results[sensor_index].distance_mm[cell];
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
    }

    return cbor_encoder_get_buffer_size(&encoder, buffer);
}
