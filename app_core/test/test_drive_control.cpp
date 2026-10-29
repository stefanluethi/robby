#include "service/drive_control.h"

#include <catch2/catch_test_macros.hpp>

using namespace robby;
using namespace robby::drive;

namespace robby::drive {

bool operator==(const Wheels& a, const Wheels& b) { return a.left == b.left && a.right == b.right; }

}  // namespace robby::drive

TEST_CASE("Speed and turn mix into the wheel speeds, the turn wins at the limit")
{
    CHECK(mix(0, 0) == Wheels {0, 0});
    CHECK(mix(200, 0) == Wheels {200, 200});
    CHECK(mix(-200, 0) == Wheels {-200, -200});
    // positive turns left: the right wheel is faster
    CHECK(mix(200, 50) == Wheels {150, 250});
    CHECK(mix(0, 100) == Wheels {-100, 100});
    // too fast for the outer wheel: the speed is reduced, the turn kept
    CHECK(mix(400, 100) == Wheels {200, 400});
    CHECK(mix(-400, -100) == Wheels {-200, -400});
    CHECK(mix(400, 400) == Wheels {-400, 400});
    // out of range
    CHECK(mix(1000, 1000) == Wheels {-400, 400});
}

TEST_CASE("Encoder counts become mm/s")
{
    // one wheel revolution per second is its circumference per second
    CHECK(speed_of(COUNTS_PER_WHEEL_REV, 1000U) == WHEEL_CIRCUMFERENCE_UM / 1000);
    CHECK(speed_of(-COUNTS_PER_WHEEL_REV, 1000U) == -(WHEEL_CIRCUMFERENCE_UM / 1000));
    // one count in 20 ms
    CHECK(speed_of(1, 20U) == 4);
    CHECK(speed_of(0, 20U) == 0);
    CHECK(speed_of(10, 0U) == 0);
}

TEST_CASE("Counts between two readings wrap with the 16 and the 32 bit timers")
{
    CHECK(counts_between(100U, 150U) == 50);
    CHECK(counts_between(150U, 100U) == -50);
    // TIM3 counts to 2^16
    CHECK(counts_between(65'530U, 4U) == 10);
    CHECK(counts_between(4U, 65'530U) == -10);
    // TIM2 counts to 2^32
    CHECK(counts_between(0xFFFF'FFFBU, 5U) == 10);
    CHECK(counts_between(5U, 0xFFFF'FFFBU) == -10);
}

TEST_CASE("The speed loop is feedforward plus PI")
{
    SpeedLoop loop {
        Gains {.feedforward = 3.0F, .kp = 2.0F, .ki = 10.0F}
    };

    // on target: only the feedforward
    CHECK(loop.update(100, 100, 0.02F) == 300);
    // 10 mm/s too slow: 3 * 100 + 2 * 10 + 10 * 10 * 0.02
    CHECK(loop.update(100, 90, 0.02F) == 322);
    // the integral stays
    CHECK(loop.update(100, 100, 0.02F) == 302);

    loop.reset();
    CHECK(loop.update(100, 100, 0.02F) == 300);
    CHECK(loop.update(-100, -100, 0.02F) == -300);
}

TEST_CASE("The speed loop saturates without winding up")
{
    SpeedLoop loop {
        Gains {.feedforward = 3.0F, .kp = 2.0F, .ki = 10.0F}
    };

    // a stalled wheel: the duty saturates, the integral doesn't grow
    for (int i = 0; i < 1000; ++i) {
        CHECK(loop.update(400, 0, 0.02F) == DUTY_MAX);
    }
    // once it catches up, it's back to the feedforward right away
    CHECK(loop.update(100, 100, 0.02F) == 300);

    for (int i = 0; i < 1000; ++i) {
        CHECK(loop.update(-400, 0, 0.02F) == -DUTY_MAX);
    }
    CHECK(loop.update(-100, -100, 0.02F) == -300);
}
