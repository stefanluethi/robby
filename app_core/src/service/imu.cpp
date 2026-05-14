#include "imu.h"

#include "custom_mems_conf.h"
#include "lsm6dso.h"

#include <cbor.h>
#include <cstdint>
#include <stm32f7xx_hal.h>

extern UART_HandleTypeDef huart2;

Imu::Imu()
{
    LSM6DSO_IO_t io_ctx;

    io_ctx.BusType = LSM6DSO_I2C_BUS;
    io_ctx.Address = LSM6DSO_I2C_ADD_L;
    io_ctx.Init = CUSTOM_LSM6DSO_0_I2C_Init;
    io_ctx.DeInit = CUSTOM_LSM6DSO_0_I2C_DeInit;
    io_ctx.ReadReg = CUSTOM_LSM6DSO_0_I2C_ReadReg;
    io_ctx.WriteReg = CUSTOM_LSM6DSO_0_I2C_WriteReg;
    io_ctx.GetTick = BSP_GetTick;

    LSM6DSO_RegisterBusIO(&_driver, &io_ctx);
    LSM6DSO_Init(&_driver);

    LSM6DSO_ACC_SetOutputDataRate(&_driver, 104.0F);
    LSM6DSO_ACC_SetFullScale(&_driver, 4);

    LSM6DSO_GYRO_SetOutputDataRate(&_driver, 104.0F);
    LSM6DSO_GYRO_SetFullScale(&_driver, 2000);

    LSM6DSO_ACC_Enable(&_driver);
    LSM6DSO_GYRO_Enable(&_driver);
}

void Imu::process()
{
    LSM6DSO_Axes_t accel;
    LSM6DSO_Axes_t gyro;

    LSM6DSO_ACC_GetAxes(&_driver, &accel);
    LSM6DSO_GYRO_GetAxes(&_driver, &gyro);

    size_t length = serialize_measurement(_message_buffer_, accel.z);
    HAL_UART_Transmit(&huart2, _message_buffer_, length, HAL_MAX_DELAY);
}

size_t Imu::serialize_measurement(uint8_t* buffer, int32_t value)
{
    CborEncoder encoder;
    CborEncoder root_map;
    CborEncoder payload_map;
    CborEncoder stream_frame_map;
    CborEncoder values;

    cbor_encoder_init(&encoder, buffer, sizeof(_message_buffer_), 0);
    cbor_encoder_create_map(&encoder, &root_map, 1U);

    cbor_encode_text_stringz(&root_map, "payload");
    cbor_encoder_create_map(&root_map, &payload_map, 1U);

    cbor_encode_text_stringz(&payload_map, "StreamFrame");
    cbor_encoder_create_map(&payload_map, &stream_frame_map, 3U);

    cbor_encode_text_stringz(&stream_frame_map, "id");
    cbor_encode_uint(&stream_frame_map, 0U);

    cbor_encode_text_stringz(&stream_frame_map, "sequence");
    cbor_encode_uint(&stream_frame_map, 0U);

    cbor_encode_text_stringz(&stream_frame_map, "values");
    cbor_encoder_create_array(&stream_frame_map, &values, 1U);
    cbor_encode_int(&values, value);

    cbor_encoder_close_container(&stream_frame_map, &values);
    cbor_encoder_close_container(&payload_map, &stream_frame_map);
    cbor_encoder_close_container(&root_map, &payload_map);
    CborError result = cbor_encoder_close_container(&encoder, &root_map);

    if (result != CborNoError) {
        return 0U;
    }

    return cbor_encoder_get_buffer_size(&encoder, buffer);
}
