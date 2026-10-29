#include "service/imu_odometry.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <numbers>

using namespace robby::inertial;
using Catch::Approx;

namespace {

constexpr float G {9.80665F};
constexpr float ACC_DT {1.0F / 833.0F};
constexpr float PI {std::numbers::pi_v<float>};
/// Gyro samples per accelerometer sample (208 Hz of 833 Hz)
constexpr int GYRO_EVERY {4};

/// The IMU tilted like the one in robby: gravity -0.12 g on x, -0.16 g on y, 0.98 g on z
/// (unit length). The robot's axes in the IMU's: up, forward (x tilted), left.
struct Tilted
{
    Vec3 up;
    Vec3 forward;
    Vec3 left;
};

Vec3 normalized(Vec3 v)
{
    const float n {std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z)};
    return {v.x / n, v.y / n, v.z / n};
}

Vec3 cross(const Vec3& a, const Vec3& b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Tilted tilted()
{
    const Vec3 up {normalized({-0.12F, -0.16F, 0.98F})};
    // x in the floor plane
    const float k {up.x};
    const Vec3 forward {normalized({1.0F - k * up.x, -k * up.y, -k * up.z})};
    return {up, forward, cross(up, forward)};
}

int16_t lsb(float value, float scale) { return static_cast<int16_t>(std::lround(value / scale)); }

/// Feeds `seconds` of samples: forward acceleration in m/s², turn rate in rad/s
/// counterclockwise, a gyro bias in rad/s and a vibration in m/s² (alternating sign)
struct Robot
{
    ImuOdometry odometry;
    Tilted axes {tilted()};
    Vec3 gyro_bias {};
    int sample {0};

    explicit Robot(Vec3 forward_axis) : odometry {forward_axis} { }

    void run(float seconds, float forward_acc, float turn_rate, float vibration = 0.0F)
    {
        const int n {static_cast<int>(std::lround(seconds / ACC_DT))};
        for (int i = 0; i < n; ++i, ++sample) {
            const float shake {(sample % 2 == 0) ? vibration : -vibration};
            const auto& [up, fw, left] = axes;
            const Vec3 acc {
                G * up.x + forward_acc * fw.x + shake * left.x,
                G * up.y + forward_acc * fw.y + shake * left.y,
                G * up.z + forward_acc * fw.z + shake * left.z,
            };
            odometry.acceleration(
                lsb(acc.x, ACC_MPS2_PER_LSB),
                lsb(acc.y, ACC_MPS2_PER_LSB),
                lsb(acc.z, ACC_MPS2_PER_LSB),
                ACC_DT
            );
            if (sample % GYRO_EVERY == 0) {
                const Vec3 rate {
                    turn_rate * up.x + gyro_bias.x,
                    turn_rate * up.y + gyro_bias.y,
                    turn_rate * up.z + gyro_bias.z
                };
                odometry.rotation(
                    lsb(rate.x, GYRO_RAD_PER_S_PER_LSB),
                    lsb(rate.y, GYRO_RAD_PER_S_PER_LSB),
                    lsb(rate.z, GYRO_RAD_PER_S_PER_LSB),
                    GYRO_EVERY * ACC_DT
                );
            }
        }
    }

    const Pose& pose() const { return odometry.pose(); }
};

/// The forward axis of the tilted IMU of these tests: x
Robot robot()
{
    return Robot {
        {1.0F, 0.0F, 0.0F}
    };
}

}  // namespace

TEST_CASE("The pose waits for a first rest")
{
    auto r {robot()};
    r.run(0.1F, 0.0F, 0.0F);
    CHECK_FALSE(r.pose().ready);
    CHECK_FALSE(r.pose().stationary);

    r.run(0.3F, 0.0F, 0.0F);
    CHECK(r.pose().ready);
    CHECK(r.pose().stationary);
    CHECK(r.pose().x_mm == 0.0F);
    CHECK(r.pose().heading_rad == 0.0F);
}

TEST_CASE("Accelerating and braking along the tilted forward axis moves the pose forward")
{
    auto r {robot()};
    r.run(0.5F, 0.0F, 0.0F);
    REQUIRE(r.pose().ready);

    // 0.5 m/s² for 1 s, then -0.5 m/s² for 1 s: 0.5 m at 0.5 m/s at most
    r.run(1.0F, 0.5F, 0.0F, 0.5F);
    CHECK_FALSE(r.pose().stationary);
    CHECK(r.pose().speed_mm_s == Approx(500.0F).margin(10.0F));
    r.run(1.0F, -0.5F, 0.0F, 0.5F);
    CHECK(r.pose().speed_mm_s == Approx(0.0F).margin(10.0F));
    CHECK(r.pose().x_mm == Approx(500.0F).margin(15.0F));
    CHECK(r.pose().y_mm == Approx(0.0F).margin(1.0F));

    // back at rest
    r.run(0.5F, 0.0F, 0.0F);
    CHECK(r.pose().stationary);
    CHECK(r.pose().speed_mm_s == 0.0F);
}

TEST_CASE("Turning about the tilted up axis turns the heading counterclockwise")
{
    auto r {robot()};
    r.run(0.5F, 0.0F, 0.0F);

    // 90 °/s for 1 s
    r.run(1.0F, 0.0F, PI / 2.0F);
    CHECK(r.pose().heading_rad == Approx(PI / 2.0F).margin(0.02F));
    // rest takes about 0.7 s after a fast turn: the moving variance settles, then REST_TIME_S
    r.run(1.0F, 0.0F, 0.0F);
    CHECK(r.pose().stationary);
    CHECK(r.pose().heading_rad == Approx(PI / 2.0F).margin(0.02F));

    // then forward goes to +y
    r.run(1.0F, 0.5F, 0.0F, 0.5F);
    r.run(1.0F, -0.5F, 0.0F, 0.5F);
    CHECK(r.pose().x_mm == Approx(0.0F).margin(15.0F));
    CHECK(r.pose().y_mm == Approx(500.0F).margin(15.0F));

    // clockwise wraps through -pi
    r.run(2.5F, 0.0F, -PI / 2.0F);
    CHECK(r.pose().heading_rad == Approx(-3.0F * PI / 4.0F).margin(0.05F));
}

TEST_CASE("The gyro bias is learned at rest and doesn't turn the heading")
{
    auto r {robot()};
    r.gyro_bias = {0.005F, -0.004F, 0.008F};
    r.run(3.0F, 0.0F, 0.0F);
    CHECK(r.pose().heading_rad == 0.0F);
    CHECK(r.odometry.gyro_bias().z == Approx(0.008F).margin(0.0005F));

    // a turn after it doesn't include the bias
    r.run(1.0F, 0.0F, PI / 2.0F);
    CHECK(r.pose().heading_rad == Approx(PI / 2.0F).margin(0.02F));
}

TEST_CASE("Steady acceleration or turning isn't rest")
{
    auto r {robot()};
    r.run(0.5F, 0.0F, 0.0F);

    // without vibration: quiet, but stronger than gravity (0.2 g: 2 % stronger) or away from
    // the bias. The first ms still count as rest, until the moving average gets there.
    r.run(1.0F, 2.0F, 0.0F);
    CHECK_FALSE(r.pose().stationary);
    CHECK(r.pose().speed_mm_s == Approx(2000.0F).margin(60.0F));

    r.run(1.0F, 0.0F, 1.0F);
    CHECK_FALSE(r.pose().stationary);
}

TEST_CASE("A rest after a drift sets the speed back to 0")
{
    auto r {robot()};
    r.run(0.5F, 0.0F, 0.0F);

    // vibration only, but an acceleration offset of 0.03 g (a slope) builds up a speed
    r.run(1.0F, 0.03F * G, 0.0F, 0.5F);
    CHECK(r.pose().speed_mm_s > 200.0F);

    // the slope is learned as gravity while resting on it
    r.run(2.0F, 0.03F * G, 0.0F);
    CHECK(r.pose().stationary);
    CHECK(r.pose().speed_mm_s == 0.0F);
    const float x {r.pose().x_mm};
    r.run(1.0F, 0.03F * G, 0.0F);
    CHECK(r.pose().x_mm == x);
}

TEST_CASE("A rest at another tilt takes its gravity vector at once")
{
    // calibrated tilted, as on a stand, then set down level
    auto r {robot()};
    r.run(0.5F, 0.0F, 0.0F);
    REQUIRE(r.pose().ready);
    r.axes = {
        .up = {0.0F, 0.0F, 1.0F},
        .forward = {1.0F, 0.0F, 0.0F},
        .left = {0.0F, 1.0F, 0.0F}
    };

    // the change of tilt itself counts as moving (it's being handled) until the new rest
    r.run(1.0F, 0.0F, 0.0F);
    CHECK(r.pose().stationary);
    CHECK(r.pose().speed_mm_s == 0.0F);
    const float x {r.pose().x_mm};
    r.run(5.0F, 0.0F, 0.0F);
    CHECK(r.pose().x_mm == x);

    // and drives on it without the old tilt in the forward acceleration
    r.run(1.0F, 0.5F, 0.0F, 0.5F);
    r.run(1.0F, -0.5F, 0.0F, 0.5F);
    CHECK(r.pose().speed_mm_s == Approx(0.0F).margin(10.0F));
    CHECK(r.pose().x_mm - x == Approx(500.0F).margin(20.0F));
    CHECK(r.pose().y_mm == Approx(0.0F).margin(1.0F));
}

TEST_CASE("The robot's IMU has y forward")
{
    // level, y forward: x points to the right
    Robot r {FORWARD_AXIS};
    r.axes = {
        .up = {0.0F,  0.0F, 1.0F},
        .forward = {0.0F,  1.0F, 0.0F},
        .left = {-1.0F, 0.0F, 0.0F}
    };
    r.run(0.5F, 0.0F, 0.0F);
    r.run(1.0F, 0.5F, 0.0F, 0.5F);
    r.run(1.0F, -0.5F, 0.0F, 0.5F);
    CHECK(r.pose().x_mm == Approx(500.0F).margin(15.0F));
    CHECK(r.pose().y_mm == Approx(0.0F).margin(1.0F));
}
