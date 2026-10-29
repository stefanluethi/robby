#pragma once

#include <cstdint>

namespace robby::drive {

// The speed loop of the wheels, without the HAL: the Drive service runs it on the encoders
// and the PWMs.

// JGA25-370 motors (130 rpm at 12 V) with 60 mm wheels

/// Hall encoder pulses per motor revolution, on each channel
inline constexpr int32_t ENCODER_PULSES_PER_MOTOR_REV {11};
/// Gearbox of the 130 rpm version. Not confirmed by a datasheet: check it with the counts of
/// the Drive record, one wheel revolution has to give COUNTS_PER_WHEEL_REV.
inline constexpr int32_t GEAR_RATIO {45};
/// The timers count both edges of both channels
inline constexpr int32_t COUNTS_PER_WHEEL_REV {ENCODER_PULSES_PER_MOTOR_REV * 4 * GEAR_RATIO};
/// pi * 60 mm
inline constexpr int32_t WHEEL_CIRCUMFERENCE_UM {188'496};

/// 130 rpm * 188 mm is 408 mm/s at 12 V without load
inline constexpr int32_t MAX_WHEEL_SPEED {400};
inline constexpr int32_t DUTY_MAX {1000};
inline constexpr uint32_t PERIOD_MS {20U};

/// Speeds in mm/s, positive forward
struct Wheels
{
    int32_t left;
    int32_t right;
};

/// The wheel speeds for a forward speed and a turn (positive left): left = speed - turn,
/// right = speed + turn. If a wheel would go faster than MAX_WHEEL_SPEED, the speed is
/// reduced and the turn kept. Both are within +-MAX_WHEEL_SPEED.
Wheels mix(int32_t speed, int32_t turn);

/// The counts between two readings of an encoder timer; the timers count up to 2^16 or
/// 2^32, a period has far fewer than 2^15 counts
inline int32_t counts_between(uint32_t previous, uint32_t now)
{
    return static_cast<int16_t>(static_cast<uint16_t>(now - previous));
}

/// The speed in mm/s of `counts` in `period_ms`
int32_t speed_of(int32_t counts, uint32_t period_ms);

struct Gains
{
    /// Duty per speed without load, per mille per mm/s
    float feedforward;
    /// Per mille per mm/s of error
    float kp;
    /// Per mille per mm of error (mm/s over s)
    float ki;
};

/// Duty cycle of 1000 per mille drives a wheel at about 270 mm/s on the 2S battery
/// (8 V of 12 V). To tune on the robot with the Drive record.
inline constexpr Gains DEFAULT_GAINS {.feedforward = 3.0F, .kp = 1.5F, .ki = 10.0F};

/// PI controller with feedforward of one wheel
class SpeedLoop
{
public:
    explicit SpeedLoop(Gains gains = DEFAULT_GAINS) : _gains {gains} { }

    /// The duty cycle, -DUTY_MAX..DUTY_MAX, for one period of `dt_s`. The integral stops
    /// while the duty is saturated in the direction of the error.
    int32_t update(int32_t target, int32_t measured, float dt_s);
    void reset() { _integral = 0.0F; }

private:
    Gains _gains;
    /// Per mille
    float _integral {0.0F};
};

/// Notices a wheel the loop has lost: one that turns against its duty (the encoder counts
/// the wrong way, so the loop drives it ever faster), or one that doesn't move at full duty
/// (no encoder counts, or blocked).
class RunawayCheck
{
public:
    enum class Fault
    {
        NONE,
        /// Turned against its duty for AGAINST_PERIODS periods in a row
        AGAINST_DUTY,
        /// Stood still at full duty for STILL_PERIODS periods in a row
        STILL_AT_FULL_DUTY,
    };

    /// Duty and speed must have opposite signs with at least these magnitudes
    static constexpr int32_t AGAINST_MIN_DUTY {200};
    static constexpr int32_t AGAINST_MIN_SPEED {30};
    /// 100 ms at 50 Hz
    static constexpr uint32_t AGAINST_PERIODS {5U};
    /// 1 s at 50 Hz
    static constexpr uint32_t STILL_PERIODS {50U};

    /// Once per period with the duty set in the period before and the speed it gave
    Fault update(int32_t duty, int32_t speed);
    void reset();

private:
    uint32_t _against {0U};
    uint32_t _still {0U};
};

/// One period of the loop of both wheels, for the Drive record
struct Sample
{
    uint64_t t_ns;
    Wheels target;
    Wheels speed;
    Wheels duty;
    Wheels counts;
};

}  // namespace robby::drive
