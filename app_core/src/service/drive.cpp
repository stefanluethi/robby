#include "drive.h"

#include "stm32f7xx_hal.h"
#include "trace/clock.h"
#include "trace/records.h"

#include <cstdlib>

#include <rtos/rtos.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim9;

using namespace robby;

namespace {

/// TIM9 counts to 54000 per PWM period
constexpr uint32_t PWM_COUNTS_PER_PERMILLE {54U};
constexpr float PERIOD_S {static_cast<float>(drive::PERIOD_MS) / 1000.0F};

}  // namespace

// The motors face opposite ways: the robot moves forward with DIR low on motor 1, high on
// motor 2. Assumed: motor 1 is the left one, and each encoder counts up while its motor
// runs with DIR high. If the robot turns the wrong way, swap the wheels; if a wheel's
// counts go down while it rolls forward, the encoder sign is wrong.
Drive::Drive(Link& link) :
    _link {link},
    _left {Motor {TIM_CHANNEL_1, MOT_EN_1_GPIO_Port, MOT_EN_1_Pin, true, &htim2}},
    _right {Motor {TIM_CHANNEL_2, MOT_EN_2_GPIO_Port, MOT_EN_2_Pin, false, &htim3}}
{
}

const char* Drive::set(int32_t speed, int32_t turn)
{
    if (std::abs(speed) > drive::MAX_WHEEL_SPEED || std::abs(turn) > drive::MAX_WHEEL_SPEED) {
        return "speed or turn out of range";
    }

    taskENTER_CRITICAL();
    _speed = speed;
    _turn = turn;
    taskEXIT_CRITICAL();
    return nullptr;
}

void Drive::start()
{
    for (Wheel* wheel : {&_left, &_right}) {
        HAL_TIM_Encoder_Start(wheel->motor.encoder, TIM_CHANNEL_ALL);
        wheel->last_counter = __HAL_TIM_GET_COUNTER(wheel->motor.encoder);
        apply(wheel->motor, 0);
    }
    HAL_GPIO_WritePin(MOT_SLEEP_GPIO_Port, MOT_SLEEP_Pin, GPIO_PIN_RESET);
    _last_wake = xTaskGetTickCount();
}

void Drive::run()
{
    vTaskDelayUntil(&_last_wake, pdMS_TO_TICKS(drive::PERIOD_MS));

    taskENTER_CRITICAL();
    const int32_t speed {_speed};
    const int32_t turn {_turn};
    taskEXIT_CRITICAL();

    drive::Sample sample {};
    sample.t_ns = uptime_ns();
    sample.speed = {.left = measure(_left), .right = measure(_right)};
    sample.counts = {.left = _left.counts, .right = _right.counts};

    if (speed == 0 && turn == 0) {
        // stopped: the driver sleeps (outputs open), the wheels coast
        HAL_GPIO_WritePin(MOT_SLEEP_GPIO_Port, MOT_SLEEP_Pin, GPIO_PIN_RESET);
        _left.loop.reset();
        _right.loop.reset();
        sample.target = {.left = 0, .right = 0};
        sample.duty = {.left = 0, .right = 0};
    } else {
        HAL_GPIO_WritePin(MOT_SLEEP_GPIO_Port, MOT_SLEEP_Pin, GPIO_PIN_SET);
        sample.target = drive::mix(speed, turn);
        sample.duty = {
            .left = _left.loop.update(sample.target.left, sample.speed.left, PERIOD_S),
            .right = _right.loop.update(sample.target.right, sample.speed.right, PERIOD_S),
        };
    }
    apply(_left.motor, sample.duty.left);
    apply(_right.motor, sample.duty.right);

    // sent with the next flush of the data publisher
    records::make_drive(sample, _record);
    _link.add(_record);
}

int32_t Drive::measure(Wheel& wheel)
{
    const uint32_t counter {__HAL_TIM_GET_COUNTER(wheel.motor.encoder)};
    int32_t counts {drive::counts_between(wheel.last_counter, counter)};
    wheel.last_counter = counter;
    if (wheel.motor.reversed) {
        counts = -counts;
    }
    // wraps like the int32 of the record
    wheel.counts = static_cast<int32_t>(static_cast<uint32_t>(wheel.counts) + static_cast<uint32_t>(counts));
    return drive::speed_of(counts, drive::PERIOD_MS);
}

void Drive::apply(const Motor& motor, int32_t duty)
{
    // DRV8874 in PH/EN mode: the PWM sets the duty cycle, DIR (PH) the direction
    const bool dir_high {(duty >= 0) != motor.reversed};
    HAL_GPIO_WritePin(motor.dir_port, motor.dir_pin, dir_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim9, motor.pwm_channel, static_cast<uint32_t>(std::abs(duty)) * PWM_COUNTS_PER_PERMILLE);
}
