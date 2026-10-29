#pragma once

#include <cstdint>

namespace robby::inertial {

// The position of the robot from the IMU alone (LSM6DSO accelerometer and gyro), without
// the HAL and independent of the wheel encoders: the Imu service feeds it the samples of
// the FIFO.
//
// - Heading: the gyro rate about the up axis, integrated. The up axis is the direction of
//   gravity measured at rest, so the IMU may sit tilted in the robot.
// - Position: the robot can't move sideways, so only the acceleration along its forward
//   axis (in the floor plane) is integrated to a speed, which moves the position along
//   the heading. The centripetal and sideways parts are left out.
// - Zero velocity updates: while the IMU sees the robot at rest (little vibration, the
//   acceleration as strong as gravity, no rotation) the speed is 0 and the gravity vector
//   and the gyro bias are learned. A rest at another tilt (set down elsewhere, a slope)
//   takes its gravity vector at once. Until the first rest the pose stays at 0 (not ready).
//
// The speed error still grows with the time since the last rest, the position error with
// its square: the pose drifts on long drives without stops. Steady driving without
// vibration looks like rest to an IMU, which stops the pose; so does a steady acceleration
// below about 0.14 g without vibration.

struct Vec3
{
    float x;
    float y;
    float z;
};

/// LSM6DSO at +-2 g: 0.061 mg per LSB
inline constexpr float ACC_MPS2_PER_LSB {0.061e-3F * 9.80665F};
/// LSM6DSO at +-500 dps: 17.5 mdps per LSB
inline constexpr float GYRO_RAD_PER_S_PER_LSB {17.5e-3F * 3.14159265F / 180.0F};

/// The forward axis of the robot in the axes of the IMU; it doesn't need to be level. Seen
/// on the robot (pushed forward by hand): y, so x points to the right and z up.
inline constexpr Vec3 FORWARD_AXIS {0.0F, 1.0F, 0.0F};

struct Pose
{
    /// x forward and y to the left of where the robot stood at its first rest
    float x_mm;
    float y_mm;
    /// Counterclockwise from x, -pi..pi
    float heading_rad;
    /// Along the heading, negative backwards
    float speed_mm_s;
    /// The IMU sees the robot at rest
    bool stationary;
    /// Calibrated at a first rest, before that the pose stays 0
    bool ready;
};

class ImuOdometry
{
public:
    // Rest: the standard deviation of the acceleration and the rotation rate (fast moving
    // averages) below these, the mean acceleration as strong as gravity within
    // REST_ACC_OFFSET (its direction may change, with the tilt) and the mean rotation this
    // close to the bias, for REST_TIME_S
    static constexpr float REST_ACC_SD {0.015F * 9.80665F};
    static constexpr float REST_GYRO_SD {0.01F};
    static constexpr float REST_ACC_OFFSET {0.01F * 9.80665F};
    static constexpr float REST_GYRO_OFFSET {0.02F};
    static constexpr float REST_TIME_S {0.3F};
    /// Time constant of the moving averages of the rest detection
    static constexpr float FAST_TAU_S {0.05F};
    /// Time constant of learning the gravity vector and the gyro bias at rest
    static constexpr float SLOW_TAU_S {1.0F};
    /// A rest this far from the gravity vector (about 1 degree) is at another tilt: its
    /// gravity vector is taken at once
    static constexpr float NEW_TILT {0.02F * 9.80665F};

    explicit ImuOdometry(Vec3 forward = FORWARD_AXIS) : _forward {forward} { }

    /// One accelerometer sample in LSB, `dt_s` after the one before
    void acceleration(int16_t x, int16_t y, int16_t z, float dt_s);
    /// One gyro sample in LSB, `dt_s` after the one before
    void rotation(int16_t x, int16_t y, int16_t z, float dt_s);

    const Pose& pose() const { return _pose; }
    /// The gyro bias learned at rest in rad/s
    const Vec3& gyro_bias() const { return _gyro_bias; }

private:
    Vec3 _forward;
    Pose _pose {};

    // rest detection
    Vec3 _acc_mean {};
    float _acc_var {0.0F};
    Vec3 _gyro_mean {};
    float _gyro_var {0.0F};
    bool _have_acc {false};
    bool _have_gyro {false};
    float _rest_time_s {0.0F};

    // learned at rest
    Vec3 _gravity {};
    Vec3 _gyro_bias {};
    /// Unit vectors: up, and forward in the floor plane
    Vec3 _up {};
    Vec3 _forward_level {};

    bool at_rest() const;
    void learn_gravity(const Vec3& acc, float alpha);
};

}  // namespace robby::inertial
