#include "imu.h"

#include "custom_mems_conf.h"
#include "driver/fuelgauge.h"
#include "lsm6dso.h"
#include "trace/clock.h"
#include "trace/records.h"

#include <stm32f7xx_hal.h>

#include <algorithm>
#include <utility>

using namespace robby;

Imu::Imu(
    se_oss::Logger log,
    rtos::MessageQueue<AccelerationFrame>& frames,
    rtos::MessageQueue<fuelgauge_status_t>& battery,
    std::atomic<uint32_t>& dropped_frames,
    Link& link
    ) :
    _log{std::move(log)},
    _frames{frames},
    _battery{battery},
    _dropped_frames{dropped_frames},
    _link{link}
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
    // the gauge shares I2C2 and must stop broadcasting before the FIFO gets configured
    disable_gauge_broadcast();
    configure();
}

void Imu::disable_gauge_broadcast()
{
    // A gauge in BROADCAST mode writes its charging current to 0xD4 every 15 s, the address of
    // the IMU, which overwrites FIFO_CTRL4 and stops the FIFO. New battery packs ship with it on.
    uint8_t i2c_configuration = 0U;
    switch (fuelgauge_disable_broadcast(&i2c_configuration)) {
        case FUELGAUGE_BROADCAST_OFF:
            LOG_INFO(_log, "Gauge broadcast is off (I2C configuration 0x%02x)", static_cast<unsigned>(i2c_configuration));
            break;
        case FUELGAUGE_BROADCAST_CLEARED:
            LOG_INFO(_log, "Gauge broadcast turned off (I2C configuration 0x%02x)", static_cast<unsigned>(i2c_configuration));
            break;
        case FUELGAUGE_BROADCAST_SEALED:
            LOG_WARN(_log, "Gauge is sealed, can't check its broadcast mode");
            break;
        case FUELGAUGE_BROADCAST_NO_GAUGE:
            LOG_WARN(_log, "Gauge not responding, can't check its broadcast mode");
            break;
        case FUELGAUGE_BROADCAST_FAILED:
            LOG_WARN(_log, "Turning off the gauge broadcast failed (I2C configuration 0x%02x)", static_cast<unsigned>(i2c_configuration));
            break;
    }
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
    // turning on the spot at full speed is about 320 dps
    ret |= lsm6dso_gy_full_scale_set(ctx, LSM6DSO_500dps);
    ret |= lsm6dso_fifo_xl_batch_set(ctx, LSM6DSO_XL_BATCHED_AT_833Hz);
    // for the heading of the IMU pose; 208 Hz adds a quarter to the I2C traffic, not double
    ret |= lsm6dso_fifo_gy_batch_set(ctx, LSM6DSO_GY_BATCHED_AT_208Hz);

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
    _last_read_ns = 0U;
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

    // after a read of the FIFO, at most every 20 ms: a loop takes about 28 ms (reading the
    // FIFO word by word, then the sleep), so about 35 Hz
    if (xTaskGetTickCount() - _last_pose_tick >= POSE_INTERVAL_MS) {
        _last_pose_tick = xTaskGetTickCount();
        publish_pose();
    }

    if (xTaskGetTickCount() - _last_battery_tick >= BATTERY_INTERVAL_MS) {
        _last_battery_tick = xTaskGetTickCount();
        read_battery();
    }
}

void Imu::read_battery()
{
    fuelgauge_status_t status {};
    const bool read = fuelgauge_read_status(&status);

    // only the changes are logged, not every second of a missing gauge
    if (read == _battery_failed) {
        _battery_failed = !read;
        if (read) {
            LOG_INFO(_log, "Reading the battery gauge works again");
        } else {
            LOG_WARN(_log, "Reading the battery gauge failed");
        }
    }

    if (read) {
        _battery.trySend(status);
    }
}

void Imu::read_fifo(uint16_t num_words)
{
    uint32_t acc_samples {0U};
    for (uint16_t i = 0; i < num_words; i++) {
        std::array<uint8_t, FIFO_WORD_SIZE> word {};
        if (lsm6dso_read_reg(&_driver.Ctx, LSM6DSO_FIFO_DATA_OUT_TAG, word.data(), word.size()) != 0) {
            break;
        }

        const auto x {static_cast<int16_t>(word[1] | (word[2] << 8U))};
        const auto y {static_cast<int16_t>(word[3] | (word[4] << 8U))};
        const auto z {static_cast<int16_t>(word[5] | (word[6] << 8U))};

        // tag is in the upper 5 bits, followed by the tag counter and parity
        const auto tag {static_cast<uint8_t>(word[0] >> 3U)};
        if (tag == LSM6DSO_GYRO_NC_TAG) {
            _odometry.rotation(x, y, z, _acc_period_s * static_cast<float>(ACC_SAMPLES_PER_GYRO_SAMPLE));
            continue;
        }
        if (tag != LSM6DSO_XL_NC_TAG) {
            continue;
        }

        _odometry.acceleration(x, y, z, _acc_period_s);
        ++acc_samples;

        _acceleration_frame.x[_num_samples] = x;
        _acceleration_frame.y[_num_samples] = y;
        _acceleration_frame.z[_num_samples] = z;
        _last_sample_tick = xTaskGetTickCount();

        if (++_num_samples == _acceleration_frame.x.size()) {
            _num_samples = 0U;
            _acceleration_frame.timestamp_ns = uptime_ns();
            _acceleration_frame.size = _acceleration_frame.x.size();
            if (!_frames.trySend(_acceleration_frame)) {
                _dropped_frames.fetch_add(1U, std::memory_order_relaxed);
            }
        }
    }
    measure_sample_period(acc_samples);
}

void Imu::measure_sample_period(uint32_t acc_samples)
{
    if (acc_samples == 0U) {
        return;
    }
    const uint64_t now_ns {uptime_ns()};
    const uint64_t elapsed_ns {now_ns - _last_read_ns};
    // a gap, e.g. after configure(), says nothing about the rate
    if (_last_read_ns != 0U && elapsed_ns < 100'000'000U) {
        const float period_s {static_cast<float>(elapsed_ns) * 1e-9F / static_cast<float>(acc_samples)};
        const float nominal_s {static_cast<float>(ACCELERATION_SAMPLE_PERIOD_NS) * 1e-9F};
        // the reads jitter by a few ms: averaged over about 10 s
        _acc_period_s += 0.01F * (std::clamp(period_s, 0.9F * nominal_s, 1.1F * nominal_s) - _acc_period_s);
    }
    _last_read_ns = now_ns;
}

void Imu::publish_pose()
{
    const auto& pose {_odometry.pose()};
    if (pose.ready && !_pose_ready) {
        const auto& bias {_odometry.gyro_bias()};
        LOG_INFO(
            _log,
            "IMU pose calibrated at rest, gyro bias %d %d %d mdps, %u us per sample",
            static_cast<int>(bias.x * 57295.8F),
            static_cast<int>(bias.y * 57295.8F),
            static_cast<int>(bias.z * 57295.8F),
            static_cast<unsigned>(_acc_period_s * 1e6F)
        );
    }
    _pose_ready = pose.ready;

    records::make_imu_pose(uptime_ns(), pose, _record);
    _link.add(_record);
}
