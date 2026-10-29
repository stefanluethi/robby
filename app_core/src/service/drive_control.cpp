#include "drive_control.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

using namespace robby;

drive::Wheels drive::mix(int32_t speed, int32_t turn)
{
    turn = std::clamp(turn, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
    // the turn wins: what's left for the speed
    const int32_t speed_max {MAX_WHEEL_SPEED - std::abs(turn)};
    speed = std::clamp(speed, -speed_max, speed_max);
    return {.left = speed - turn, .right = speed + turn};
}

int32_t drive::speed_of(int32_t counts, uint32_t period_ms)
{
    if (period_ms == 0U) {
        return 0;
    }
    // um/s / 1000, rounded towards 0
    const int64_t um_per_s {
        int64_t {counts} * WHEEL_CIRCUMFERENCE_UM * 1000 / (int64_t {COUNTS_PER_WHEEL_REV} * period_ms)
    };
    return static_cast<int32_t>(um_per_s / 1000);
}

drive::RunawayCheck::Fault drive::RunawayCheck::update(int32_t duty, int32_t speed)
{
    const bool against {
        std::abs(duty) >= AGAINST_MIN_DUTY && std::abs(speed) >= AGAINST_MIN_SPEED && (duty > 0) != (speed > 0)
    };
    _against = against ? _against + 1U : 0U;
    _still = (std::abs(duty) >= DUTY_MAX && speed == 0) ? _still + 1U : 0U;

    if (_against >= AGAINST_PERIODS) {
        return Fault::AGAINST_DUTY;
    }
    if (_still >= STILL_PERIODS) {
        return Fault::STILL_AT_FULL_DUTY;
    }
    return Fault::NONE;
}

void drive::RunawayCheck::reset()
{
    _against = 0U;
    _still = 0U;
}

int32_t drive::SpeedLoop::update(int32_t target, int32_t measured, float dt_s)
{
    const auto error {static_cast<float>(target - measured)};
    const float without_integral {_gains.feedforward * static_cast<float>(target) + _gains.kp * error};
    const auto limit {static_cast<float>(DUTY_MAX)};

    const float integral {std::clamp(_integral + _gains.ki * error * dt_s, -limit, limit)};
    const float duty {without_integral + integral};
    // conditional integration: keep the new integral unless it pushes further into saturation
    const bool saturated {(duty > limit && error > 0.0F) || (duty < -limit && error < 0.0F)};
    if (!saturated) {
        _integral = integral;
    }

    return static_cast<int32_t>(std::lround(std::clamp(without_integral + _integral, -limit, limit)));
}
