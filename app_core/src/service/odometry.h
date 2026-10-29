#pragma once

#include "service/drive_control.h"

#include <cstdint>

namespace robby::drive {

// The position of the robot from the wheel encoders (dead reckoning), without the HAL: the
// Drive service feeds it the counts of each period.

/// Distance between the middles of the two wheels, where they touch the ground. Not
/// measured: check it by turning the robot on the spot 10 times (pose/heading has to come
/// back to where it started); if the heading turns too far, the track is wider.
inline constexpr int32_t TRACK_WIDTH_UM {120'000};

/// x forward and y to the left of where the robot stood at the reset, as seen from above
struct Pose
{
    float x_mm;
    float y_mm;
    /// Counterclockwise from x, -pi..pi
    float heading_rad;
};

class Odometry
{
public:
    /// Moves the pose by the encoder counts of each wheel since the last update, positive
    /// forward. Exact for a wheel speed that is constant over the update.
    void update(int32_t left_counts, int32_t right_counts);

    const Pose& pose() const { return _pose; }
    void reset() { _pose = {}; }

private:
    Pose _pose {};
};

}  // namespace robby::drive
