#include "odometry.h"

#include <cmath>
#include <numbers>

using namespace robby;

namespace {

constexpr float MM_PER_COUNT {
    static_cast<float>(drive::WHEEL_CIRCUMFERENCE_UM) / 1000.0F / static_cast<float>(drive::COUNTS_PER_WHEEL_REV)
};
constexpr float TRACK_WIDTH_MM {static_cast<float>(drive::TRACK_WIDTH_UM) / 1000.0F};
constexpr float PI {std::numbers::pi_v<float>};

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

}  // namespace

void drive::Odometry::update(int32_t left_counts, int32_t right_counts)
{
    const float left_mm {static_cast<float>(left_counts) * MM_PER_COUNT};
    const float right_mm {static_cast<float>(right_counts) * MM_PER_COUNT};
    const float distance {(left_mm + right_mm) / 2.0F};
    const float turn {(right_mm - left_mm) / TRACK_WIDTH_MM};

    // Along the arc: the chord is 2 sin(turn/2) / turn of its length, in the direction of
    // the heading halfway through the turn. A period turns by far less than pi.
    const float half {turn / 2.0F};
    const float chord {std::abs(half) < 1e-4F ? distance : distance * std::sin(half) / half};
    const float direction {_pose.heading_rad + half};
    _pose.x_mm += chord * std::cos(direction);
    _pose.y_mm += chord * std::sin(direction);
    _pose.heading_rad = wrap(_pose.heading_rad + turn);
}
