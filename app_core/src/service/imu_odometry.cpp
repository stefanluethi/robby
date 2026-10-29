#include "imu_odometry.h"

#include <algorithm>
#include <cmath>
#include <numbers>

using namespace robby;
using namespace robby::inertial;

namespace {

constexpr float PI {std::numbers::pi_v<float>};

Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(float k, const Vec3& v) { return {k * v.x, k * v.y, k * v.z}; }
Vec3& operator+=(Vec3& a, const Vec3& b)
{
    a = {a.x + b.x, a.y + b.y, a.z + b.z};
    return a;
}
float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
float norm(const Vec3& v) { return std::sqrt(dot(v, v)); }

Vec3 scaled(int16_t x, int16_t y, int16_t z, float scale)
{
    return {static_cast<float>(x) * scale, static_cast<float>(y) * scale, static_cast<float>(z) * scale};
}

/// -pi..pi
float wrap(float angle)
{
    if (angle > PI) {
        angle -= 2.0F * PI;
    } else if (angle < -PI) {
        angle += 2.0F * PI;
    }
    return angle;
}

/// Moving average and variance of the distance to it
void average(Vec3& mean, float& var, bool& started, const Vec3& sample, float alpha)
{
    if (!started) {
        mean = sample;
        var = 0.0F;
        started = true;
        return;
    }
    const Vec3 d {sample - mean};
    mean += alpha * d;
    var = (1.0F - alpha) * (var + alpha * dot(d, d));
}

}  // namespace

void ImuOdometry::acceleration(int16_t x, int16_t y, int16_t z, float dt_s)
{
    const Vec3 acc {scaled(x, y, z, ACC_MPS2_PER_LSB)};
    average(_acc_mean, _acc_var, _have_acc, acc, std::min(1.0F, dt_s / FAST_TAU_S));

    _rest_time_s = at_rest() ? _rest_time_s + dt_s : 0.0F;
    _pose.stationary = _rest_time_s >= REST_TIME_S;

    if (_pose.stationary) {
        if (!_pose.ready) {
            _gyro_bias = _gyro_mean;
            learn_gravity(_acc_mean, 1.0F);
            _pose.ready = true;
        } else if (norm(_acc_mean - _gravity) > NEW_TILT) {
            learn_gravity(_acc_mean, 1.0F);
        } else {
            learn_gravity(acc, std::min(1.0F, dt_s / SLOW_TAU_S));
        }
        _pose.speed_mm_s = 0.0F;
        return;
    }
    if (!_pose.ready) {
        return;
    }

    // forward acceleration in the floor plane, without gravity
    const float forward {dot(acc - _gravity, _forward_level)};
    _pose.speed_mm_s += forward * 1000.0F * dt_s;
    _pose.x_mm += _pose.speed_mm_s * std::cos(_pose.heading_rad) * dt_s;
    _pose.y_mm += _pose.speed_mm_s * std::sin(_pose.heading_rad) * dt_s;
}

void ImuOdometry::rotation(int16_t x, int16_t y, int16_t z, float dt_s)
{
    const Vec3 rate {scaled(x, y, z, GYRO_RAD_PER_S_PER_LSB)};
    average(_gyro_mean, _gyro_var, _have_gyro, rate, std::min(1.0F, dt_s / FAST_TAU_S));

    if (!_pose.ready) {
        return;
    }
    // a turn starting before the next accelerometer sample ends the rest already
    if (_pose.stationary && at_rest()) {
        _gyro_bias += std::min(1.0F, dt_s / SLOW_TAU_S) * (rate - _gyro_bias);
        return;
    }
    // counterclockwise about up: with z up, the gyro's z axis
    _pose.heading_rad = wrap(_pose.heading_rad + dot(rate - _gyro_bias, _up) * dt_s);
}

bool ImuOdometry::at_rest() const
{
    if (!_have_acc || !_have_gyro) {
        return false;
    }
    const bool quiet {std::sqrt(_acc_var) < REST_ACC_SD && std::sqrt(_gyro_var) < REST_GYRO_SD};
    if (!quiet || !_pose.ready) {
        return quiet;
    }
    // steady acceleration or turning isn't rest; another tilt is
    return std::abs(norm(_acc_mean) - norm(_gravity)) < REST_ACC_OFFSET &&
           norm(_gyro_mean - _gyro_bias) < REST_GYRO_OFFSET;
}

void ImuOdometry::learn_gravity(const Vec3& acc, float alpha)
{
    _gravity += alpha * (acc - _gravity);

    const float length {norm(_gravity)};
    if (length <= 0.0F) {
        return;
    }
    _up = (1.0F / length) * _gravity;
    // the forward axis in the floor plane
    const Vec3 level {_forward - dot(_forward, _up) * _up};
    const float level_length {norm(level)};
    if (level_length > 0.0F) {
        _forward_level = (1.0F / level_length) * level;
    }
}
