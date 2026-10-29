#pragma once

#include "driver/fuelgauge.h"
#include "service/drive_control.h"
#include "service/odometry.h"
#include "service/samples.h"

#include <array>
#include <cstddef>
#include <cstdint>

#include <robby_link.pb.h>

#include <string_view>

namespace robby::records {

// Fill a robby_link_Record from the data of app_core. Each one sets the whole record, so
// one record struct can be reused for all kinds.

/// Turns acceleration frames into ImuBatch records with one timestamp per batch.
///
/// The samples of a frame follow those of the previous one, so they are spread between
/// the times both frames were read: the trace has no gap between frames, whatever the
/// actual sample rate of the IMU (about 806 Hz instead of 833 Hz). After a gap, e.g. a
/// reconfiguration of the IMU, they are counted back from the read time at the nominal rate.
class ImuBatcher
{
public:
    /// Returns false for an empty frame, which gives no record
    bool make(const AccelerationFrame& frame, robby_link_Record& record);

private:
    /// When the previous frame was read, 0 before the first one
    uint64_t _last_timestamp_ns {0U};
};

void make_distance(uint64_t t_ns, const std::array<DistanceSensorImage, 3>& sensors, robby_link_Record& record);

void make_battery(uint64_t t_ns, const fuelgauge_status_t& battery, robby_link_Record& record);

struct TaskInfo
{
    const char* name;
    uint32_t priority;
    /// FreeRTOS run time counter of the task and of all tasks together
    uint32_t runtime;
    uint32_t total_runtime;
    uint32_t stack_size_bytes;
    uint32_t stack_free_min_bytes;
};

/// Sends the run time and the stack usage as shares in 0.01 %
void make_task(uint64_t t_ns, const TaskInfo& task, robby_link_Record& record);

/// The rates of the LinkStats records: the differences of the counters to the previous
/// record over the time between them, since the reset for the first record.
class LinkRates
{
public:
    /// Fills the rates of `stats` from its time, counters and baud rate, and from
    /// `tx_write_us`, the total time spent writing (wrapping at 2^32 like the counters)
    void fill(robby_link_LinkStats& stats, uint32_t tx_write_us);

private:
    uint64_t _t_ns {0U};
    uint32_t _tx_bytes {0U};
    uint32_t _tx_frames {0U};
    uint32_t _tx_write_us {0U};
};

void make_drive(const drive::Sample& sample, robby_link_Record& record);

/// The pose rounded to mm and hundredths of a degree
void make_pose(uint64_t t_ns, const drive::Pose& pose, robby_link_Record& record);

/// `level` and `component` are the numbers of se_oss::LogLevel and robby::LogContextId,
/// which match robby_link_LogLevel and robby_link_Component. The message is cut to 255
/// bytes, at a UTF-8 character boundary.
void make_log(uint64_t t_ns, uint8_t level, uint8_t component, std::string_view message, robby_link_Record& record);

/// `error` is nullptr for success; it's cut to 63 bytes.
void make_action_result(uint32_t id, const char* error, robby_link_Record& record);

/// Copies `text` into a nanopb string field of `size` bytes (NUL included), cut at a
/// UTF-8 character boundary
void copy_string(char* field, std::size_t size, std::string_view text);

}  // namespace robby::records
