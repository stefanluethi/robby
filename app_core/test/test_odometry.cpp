#include "service/odometry.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <numbers>

using namespace robby;
using namespace robby::drive;
using Catch::Matchers::WithinAbs;

namespace {

constexpr double PI {std::numbers::pi};
constexpr double TRACK_MM {TRACK_WIDTH_UM / 1000.0};

/// The encoder counts of a wheel that rolls `mm`
int32_t counts_of(double mm)
{
    return static_cast<int32_t>(std::lround(mm * COUNTS_PER_WHEEL_REV * 1000.0 / WHEEL_CIRCUMFERENCE_UM));
}

/// Drives the robot along an arc in `steps` updates: `distance_mm` of its middle, turning
/// by `turn_rad` (positive left)
void roll(Odometry& odometry, double distance_mm, double turn_rad, int steps)
{
    const double left {distance_mm - turn_rad * TRACK_MM / 2.0};
    const double right {distance_mm + turn_rad * TRACK_MM / 2.0};
    // like an encoder: the counts of a step are what the total has grown by
    for (int i = 0; i < steps; ++i) {
        odometry.update(
            counts_of(left * (i + 1) / steps) - counts_of(left * i / steps),
            counts_of(right * (i + 1) / steps) - counts_of(right * i / steps)
        );
    }
}

}  // namespace

TEST_CASE("The pose starts at the origin, facing x")
{
    const Odometry odometry {};
    CHECK(odometry.pose().x_mm == 0.0F);
    CHECK(odometry.pose().y_mm == 0.0F);
    CHECK(odometry.pose().heading_rad == 0.0F);
}

TEST_CASE("Both wheels forward move the robot along x, backwards back")
{
    Odometry odometry {};
    // one wheel revolution
    odometry.update(COUNTS_PER_WHEEL_REV, COUNTS_PER_WHEEL_REV);
    CHECK_THAT(odometry.pose().x_mm, WithinAbs(WHEEL_CIRCUMFERENCE_UM / 1000.0, 0.01));
    CHECK(odometry.pose().y_mm == 0.0F);
    CHECK(odometry.pose().heading_rad == 0.0F);

    odometry.update(-2 * COUNTS_PER_WHEEL_REV, -2 * COUNTS_PER_WHEEL_REV);
    CHECK_THAT(odometry.pose().x_mm, WithinAbs(-WHEEL_CIRCUMFERENCE_UM / 1000.0, 0.01));
}

TEST_CASE("Wheels in opposite directions turn the robot on the spot, the right one forward turns left")
{
    Odometry odometry {};
    roll(odometry, 0.0, PI / 2.0, 50);
    CHECK_THAT(odometry.pose().heading_rad, WithinAbs(PI / 2.0, 0.01));
    CHECK_THAT(odometry.pose().x_mm, WithinAbs(0.0, 0.01));
    CHECK_THAT(odometry.pose().y_mm, WithinAbs(0.0, 0.01));

    roll(odometry, 0.0, -PI, 50);
    CHECK_THAT(odometry.pose().heading_rad, WithinAbs(-PI / 2.0, 0.01));
}

TEST_CASE("A quarter circle to the left ends at (r, r), facing y, in many updates or one")
{
    constexpr double radius {500.0};
    for (const int steps : {1, 60}) {
        Odometry odometry {};
        roll(odometry, radius * PI / 2.0, PI / 2.0, steps);
        CHECK_THAT(odometry.pose().x_mm, WithinAbs(radius, 1.0));
        CHECK_THAT(odometry.pose().y_mm, WithinAbs(radius, 1.0));
        CHECK_THAT(odometry.pose().heading_rad, WithinAbs(PI / 2.0, 0.01));
    }
}

TEST_CASE("Driving forward after a turn to the right moves along -y")
{
    Odometry odometry {};
    roll(odometry, 0.0, -PI / 2.0, 20);
    roll(odometry, 1000.0, 0.0, 50);
    CHECK_THAT(odometry.pose().x_mm, WithinAbs(0.0, 2.0));
    CHECK_THAT(odometry.pose().y_mm, WithinAbs(-1000.0, 1.0));
}

TEST_CASE("The heading stays within -pi..pi")
{
    Odometry odometry {};
    // three quarters to the left is a quarter to the right
    roll(odometry, 0.0, 3.0 * PI / 2.0, 100);
    CHECK_THAT(odometry.pose().heading_rad, WithinAbs(-PI / 2.0, 0.01));

    // ten turns, back where it started
    roll(odometry, 0.0, PI / 2.0 + 20.0 * PI, 1000);
    CHECK_THAT(odometry.pose().heading_rad, WithinAbs(0.0, 0.02));
    CHECK(std::abs(odometry.pose().heading_rad) <= PI);
}

TEST_CASE("A reset puts the pose back to the origin")
{
    Odometry odometry {};
    roll(odometry, 300.0, 1.0, 10);
    odometry.reset();
    CHECK(odometry.pose().x_mm == 0.0F);
    CHECK(odometry.pose().y_mm == 0.0F);
    CHECK(odometry.pose().heading_rad == 0.0F);
}
