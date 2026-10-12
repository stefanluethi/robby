#include "imu.h"

#include "custom_mems_conf.h"
#include "lsm6dso.h"

#include <stm32f7xx_hal.h>

#include <utility>

using namespace robby;

Imu::Imu(se_oss::Logger log, rtos::MessageQueue<AccelerationFrame>& frames) :
    _log{std::move(log)},
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
}

void Imu::start()
{
    configure();
}

bool Imu::configure()
{
    stmdev_ctx_t* ctx = &_driver.Ctx;

    // example from
    // https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dso_STdC/examples/lsm6dso_fifo.c
    /* Restore default configuration */
    lsm6dso_reset_set(ctx, PROPERTY_ENABLE);

    uint8_t rst = 1U;
    for (uint32_t i = 0; rst != 0U; ++i) {
        if (i >= RESET_POLL_LIMIT || lsm6dso_reset_get(ctx, &rst) != 0) {
            LOG_WARN(_log, "IMU reset failed");
            return false;
        }
    }

    int32_t ret = 0;
    /* Disable I3C interface */
    ret |= lsm6dso_i3c_disable_set(ctx, LSM6DSO_I3C_DISABLE);
    /* Needed to read a FIFO word (tag and data) in one transfer */
    ret |= lsm6dso_auto_increment_set(ctx, PROPERTY_ENABLE);
    /* Enable Block Data Update */
    ret |= lsm6dso_block_data_update_set(ctx, PROPERTY_ENABLE);
    ret |= lsm6dso_xl_full_scale_set(ctx, LSM6DSO_2g);
    ret |= lsm6dso_gy_full_scale_set(ctx, LSM6DSO_2000dps);
    ret |= lsm6dso_fifo_xl_batch_set(ctx, LSM6DSO_XL_BATCHED_AT_833Hz);
    // gyro samples are unused, keeping them out of the FIFO halves the I2C traffic
    ret |= lsm6dso_fifo_gy_batch_set(ctx, LSM6DSO_GY_NOT_BATCHED);

    /* Set FIFO mode to Stream mode (aka Continuous Mode) */
    // written as a whole instead of lsm6dso_fifo_mode_set(), whose read-modify-write would keep
    // corrupted batching bits
    lsm6dso_fifo_ctrl4_t fifo_ctrl4 {};
    fifo_ctrl4.fifo_mode = LSM6DSO_STREAM_MODE;
    ret |= lsm6dso_write_reg(ctx, LSM6DSO_FIFO_CTRL4, reinterpret_cast<uint8_t*>(&fifo_ctrl4), 1);

    /* Set Output Data Rate */
    ret |= lsm6dso_xl_data_rate_set(ctx, LSM6DSO_XL_ODR_833Hz);
    ret |= lsm6dso_gy_data_rate_set(ctx, LSM6DSO_GY_ODR_833Hz);

    // samples before the gap don't belong to the same frame
    _num_samples = 0U;
    _last_sample_tick = xTaskGetTickCount();

    if (ret != 0) {
        LOG_WARN(_log, "IMU configuration failed");
        return false;
    }
    return true;
}

void Imu::process()
{
    uint16_t num_words = 0;
    if (lsm6dso_fifo_data_level_get(&_driver.Ctx, &num_words) == 0 && num_words > 0) {
        read_fifo(num_words);
    }

    // FIFO_CTRL4 was observed being overwritten at runtime (most likely by a disturbed I2C
    // transfer), which stops the FIFO for good. Recover by configuring the sensor from scratch.
    if (xTaskGetTickCount() - _last_sample_tick > STALL_TIMEOUT_MS) {
        uint8_t fifo_ctrl4 = 0;
        lsm6dso_read_reg(&_driver.Ctx, LSM6DSO_FIFO_CTRL4, &fifo_ctrl4, 1);
        LOG_WARN(
            _log,
            "No IMU samples for %u ms (FIFO level %u, FIFO_CTRL4 0x%02x), reconfiguring",
            static_cast<unsigned>(STALL_TIMEOUT_MS),
            static_cast<unsigned>(num_words),
            static_cast<unsigned>(fifo_ctrl4)
        );
        configure();
    }
}

void Imu::read_fifo(uint16_t num_words)
{
    for (uint16_t i = 0; i < num_words; i++) {
        std::array<uint8_t, FIFO_WORD_SIZE> word {};
        if (lsm6dso_read_reg(&_driver.Ctx, LSM6DSO_FIFO_DATA_OUT_TAG, word.data(), word.size()) != 0) {
            return;
        }

        // tag is in the upper 5 bits, followed by the tag counter and parity
        if ((word[0] >> 3U) != LSM6DSO_XL_NC_TAG) {
            continue;
        }

        _acceleration_frame.x[_num_samples] = static_cast<int16_t>(word[1] | (word[2] << 8U));
        _acceleration_frame.y[_num_samples] = static_cast<int16_t>(word[3] | (word[4] << 8U));
        _acceleration_frame.z[_num_samples] = static_cast<int16_t>(word[5] | (word[6] << 8U));
        _last_sample_tick = xTaskGetTickCount();

        if (++_num_samples == _acceleration_frame.x.size()) {
            _num_samples = 0U;
            if (!_frames.trySend(_acceleration_frame)) {
                // message something
            }
        }
    }
}
