#include "imu.h"

#include "custom_mems_conf.h"
#include "lsm6dso.h"

#include <cbor.h>
#include <cobs.h>

#include <stm32f7xx_hal.h>

using namespace robby;

Imu::Imu(rtos::MessageQueue<AccelerationFrame>& frames) :
    _frames{frames}
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

    // example from
    // https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dso_STdC/examples/lsm6dso_fifo.c
    /* Restore default configuration */
    lsm6dso_reset_set(&_driver.Ctx, PROPERTY_ENABLE);

    uint8_t rst;
    do {
        lsm6dso_reset_get(&_driver.Ctx, &rst);
    } while (rst);

    /* Disable I3C interface */
    lsm6dso_i3c_disable_set(&_driver.Ctx, LSM6DSO_I3C_DISABLE);
    /* Enable Block Data Update */
    lsm6dso_block_data_update_set(&_driver.Ctx, PROPERTY_ENABLE);
    lsm6dso_xl_full_scale_set(&_driver.Ctx, LSM6DSO_2g);
    lsm6dso_gy_full_scale_set(&_driver.Ctx, LSM6DSO_2000dps);
    lsm6dso_fifo_watermark_set(&_driver.Ctx, 210);
    lsm6dso_fifo_xl_batch_set(&_driver.Ctx, LSM6DSO_XL_BATCHED_AT_833Hz);
    lsm6dso_fifo_gy_batch_set(&_driver.Ctx, LSM6DSO_GY_BATCHED_AT_833Hz);
    /* Set FIFO mode to Stream mode (aka Continuous Mode) */
    lsm6dso_fifo_mode_set(&_driver.Ctx, LSM6DSO_STREAM_MODE);
}

void Imu::start()
{
    /* Set Output Data Rate */
    lsm6dso_xl_data_rate_set(&_driver.Ctx, LSM6DSO_XL_ODR_833Hz);
    lsm6dso_gy_data_rate_set(&_driver.Ctx, LSM6DSO_GY_ODR_833Hz);
}

void Imu::process()
{
    std::size_t num_samples = 0;

    uint8_t fifo_overrun = 0;
    lsm6dso_fifo_ovr_flag_get(&_driver.Ctx, &fifo_overrun);

    /* Read watermark flag */
    uint8_t wmflag = 0;
    lsm6dso_fifo_wtm_flag_get(&_driver.Ctx, &wmflag);

    /* Read number of samples in FIFO */
    uint16_t num_samples_available = 0;
    lsm6dso_fifo_data_level_get(&_driver.Ctx, &num_samples_available);

    if (wmflag == 0) {
        if (num_samples_available == 0) {
            // flush fifo if overrun, should not be necessary by device just stops
            // measuring otherwise
            lsm6dso_fifo_mode_set(&_driver.Ctx, LSM6DSO_BYPASS_MODE);
            lsm6dso_fifo_mode_set(&_driver.Ctx, LSM6DSO_STREAM_MODE);
        }

        return;
    }

    if (num_samples_available == 0) {
        return;
    }

    for (uint16_t i = 0; i < num_samples_available && num_samples < _acceleration_frame.size(); i++) {
        /* Read FIFO tag */
        lsm6dso_fifo_tag_t reg_tag;
        lsm6dso_fifo_sensor_tag_get(&_driver.Ctx, &reg_tag);

        axis3bit16_t data_raw_acceleration;
        axis3bit16_t data_raw_angular_rate;

        switch (reg_tag) {
            case LSM6DSO_XL_NC_TAG:
                memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
                lsm6dso_fifo_out_raw_get(&_driver.Ctx, data_raw_acceleration.u8bit);
                _acceleration_frame[num_samples].x = lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[0]);
                _acceleration_frame[num_samples].y = lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
                _acceleration_frame[num_samples].z = lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);
                num_samples++;
                break;

            case LSM6DSO_GYRO_NC_TAG:
                memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
                lsm6dso_fifo_out_raw_get(&_driver.Ctx, data_raw_angular_rate.u8bit);
                // angular_rate_mdps[0] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0]);
                // angular_rate_mdps[1] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1]);
                // angular_rate_mdps[2] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2]);
                break;

            default:
                /* Flush unused samples */
                axis3bit16_t dummy;
                memset(dummy.u8bit, 0x00, 3 * sizeof(int16_t));
                lsm6dso_fifo_out_raw_get(&_driver.Ctx, dummy.u8bit);
                break;
        }
    }

    if (!_frames.trySend(_acceleration_frame)) {
        // message something
    }
}
