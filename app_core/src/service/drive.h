#pragma once

#include "main.h"
#include "service/drive_control.h"
#include "service/odometry.h"
#include "trace/link.h"

#include <cstdint>

#include <robby_link.pb.h>
#include <rtos/rtos.h>
#include <se-oss/log/Log.h>

namespace robby {

/// Holds the speed of both wheels with their encoders (MotorDrive), every 20 ms, and sends
/// a Drive record each period. While speed and turn are both 0 the motor driver sleeps, so
/// the wheels turn freely. A wheel that runs against its duty or stands still at full duty
/// stops both (RunawayCheck) until the next MotorDrive. Tracks the position of the robot
/// from the same counts and sends it as a Pose record each period, also while the wheels
/// coast.
class Drive
{
public:
    Drive(se_oss::Logger log, Link& link);

    /// From any task. Speed and turn in mm/s, see MotorDrive. Returns why they were
    /// rejected, or nullptr.
    const char* set(int32_t speed, int32_t turn);

    /// Starts the encoders, from the task that calls run()
    void start();
    /// One period of the loop: waits for its start, then reads the encoders and sets the PWMs
    void run();

private:
    struct Motor
    {
        uint32_t pwm_channel;
        GPIO_TypeDef* dir_port;
        uint16_t dir_pin;
        /// The robot moves forward with DIR low on this motor
        bool reversed;
        TIM_HandleTypeDef* encoder;
        /// The encoder counts down while the robot moves forward
        bool encoder_reversed;
        const char* name;
    };

    struct Wheel
    {
        Motor motor;
        drive::SpeedLoop loop {};
        drive::RunawayCheck runaway {};
        /// Set in the period before
        int32_t duty {0};
        uint32_t last_counter {0U};
        /// Since the reset, positive forward
        int32_t counts {0};
    };

    se_oss::Logger _log;
    Link& _link;
    Wheel _left;
    Wheel _right;

    // Set by set(), read by run(), under a critical section
    int32_t _speed {0};
    int32_t _turn {0};

    drive::Odometry _odometry {};

    TickType_t _last_wake {0U};
    robby_link_Record _record {};

    /// Reads the encoder: the counts over the last period, positive forward
    static int32_t measure(Wheel& wheel);
    /// Whether the wheel ran away, see RunawayCheck; then stops the drive and logs why
    bool check_runaway(Wheel& wheel, int32_t speed);
    void stop();
    static void apply(const Motor& motor, int32_t duty);
};

}  // namespace robby
