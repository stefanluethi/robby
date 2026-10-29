#include "records.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>

using namespace robby;

namespace {

static_assert(sizeof(robby_link_ImuBatch::x) / sizeof(robby_link_ImuBatch::x[0]) >= AccelerationFrame {}.x.size());
static_assert(sizeof(robby_link_DistanceImage::mm) / sizeof(robby_link_DistanceImage::mm[0]) == 3U * 8U * 8U);

constexpr bool is_utf8_continuation(char c) { return (static_cast<uint8_t>(c) & 0xC0U) == 0x80U; }

/// count × scale / dt_ns, at most UINT32_MAX
uint32_t rate(uint64_t count, uint64_t scale, uint64_t dt_ns)
{
    constexpr uint64_t max {UINT32_MAX};
    if (dt_ns == 0U || (count != 0U && scale > UINT64_MAX / count)) {
        return dt_ns == 0U ? 0U : static_cast<uint32_t>(max);
    }
    return static_cast<uint32_t>(std::min(count * scale / dt_ns, max));
}

}  // namespace

bool records::ImuBatcher::make(const AccelerationFrame& frame, robby_link_Record& record)
{
    if (frame.size == 0U || frame.size > frame.x.size()) {
        return false;
    }

    const uint64_t nominal_ns {frame.size * ACCELERATION_SAMPLE_PERIOD_NS};
    const uint64_t since_last_ns {frame.timestamp_ns - _last_timestamp_ns};
    const bool follows_last {
        _last_timestamp_ns != 0U && frame.timestamp_ns > _last_timestamp_ns && since_last_ns > nominal_ns / 2U &&
        since_last_ns < nominal_ns * 3U / 2U
    };
    const uint64_t period_ns {follows_last ? since_last_ns / frame.size : ACCELERATION_SAMPLE_PERIOD_NS};
    _last_timestamp_ns = frame.timestamp_ns;

    record = robby_link_Record_init_zero;
    record.which_kind = robby_link_Record_imu_tag;
    auto& batch = record.kind.imu;
    const uint64_t span_ns {(frame.size - 1U) * period_ns};
    batch.t_first_ns = frame.timestamp_ns > span_ns ? frame.timestamp_ns - span_ns : 0U;
    batch.period_ns = static_cast<uint32_t>(period_ns);
    batch.x_count = static_cast<pb_size_t>(frame.size);
    batch.y_count = static_cast<pb_size_t>(frame.size);
    batch.z_count = static_cast<pb_size_t>(frame.size);
    std::copy_n(frame.x.begin(), frame.size, batch.x);
    std::copy_n(frame.y.begin(), frame.size, batch.y);
    std::copy_n(frame.z.begin(), frame.size, batch.z);
    return true;
}

void records::make_distance(uint64_t t_ns, const std::array<DistanceSensorImage, 3>& sensors, robby_link_Record& record)
{
    record = robby_link_Record_init_zero;
    record.which_kind = robby_link_Record_distance_tag;
    auto& image = record.kind.distance;
    image.t_ns = t_ns;

    // The sensors sit side by side, so zone row y of all three makes one row of 24:
    // index (y * 3 + s) * 8 + x for sensors[s][y][x], invalid zones as -1
    pb_size_t count {0U};
    for (std::size_t y = 0; y < 8U; ++y) {
        for (const auto& sensor : sensors) {
            for (int16_t distance : sensor[y]) {
                image.mm[count++] = distance >= 0 ? distance : int16_t {-1};
            }
        }
    }
    image.mm_count = count;
}

void records::make_battery(uint64_t t_ns, const fuelgauge_status_t& battery, robby_link_Record& record)
{
    record = robby_link_Record_init_zero;
    record.which_kind = robby_link_Record_battery_tag;
    auto& out = record.kind.battery;
    out.t_ns = t_ns;
    out.soc_pct = battery.soc;
    out.voltage_mv = battery.voltage;
    out.cell1_mv = battery.cell1_voltage;
    out.cell2_mv = battery.cell2_voltage;
    out.current_ma = battery.current;
}

void records::make_drive(const drive::Sample& sample, robby_link_Record& record)
{
    record = robby_link_Record_init_zero;
    record.which_kind = robby_link_Record_drive_tag;
    auto& out = record.kind.drive;
    out.t_ns = sample.t_ns;
    out.left_target_mm_s = sample.target.left;
    out.right_target_mm_s = sample.target.right;
    out.left_speed_mm_s = sample.speed.left;
    out.right_speed_mm_s = sample.speed.right;
    out.left_duty = sample.duty.left;
    out.right_duty = sample.duty.right;
    out.left_counts = sample.counts.left;
    out.right_counts = sample.counts.right;
}

void records::make_pose(uint64_t t_ns, const drive::Pose& pose, robby_link_Record& record)
{
    record = robby_link_Record_init_zero;
    record.which_kind = robby_link_Record_pose_tag;
    auto& out = record.kind.pose;
    out.t_ns = t_ns;
    out.x_mm = static_cast<int32_t>(std::lround(pose.x_mm));
    out.y_mm = static_cast<int32_t>(std::lround(pose.y_mm));
    out.heading_cdeg = static_cast<int32_t>(std::lround(pose.heading_rad * (18000.0F / std::numbers::pi_v<float>)));
}

void records::make_imu_pose(uint64_t t_ns, const inertial::Pose& pose, robby_link_Record& record)
{
    record = robby_link_Record_init_zero;
    record.which_kind = robby_link_Record_imu_pose_tag;
    auto& out = record.kind.imu_pose;
    out.t_ns = t_ns;
    out.x_mm = static_cast<int32_t>(std::lround(pose.x_mm));
    out.y_mm = static_cast<int32_t>(std::lround(pose.y_mm));
    out.heading_cdeg = static_cast<int32_t>(std::lround(pose.heading_rad * (18000.0F / std::numbers::pi_v<float>)));
    out.speed_mm_s = static_cast<int32_t>(std::lround(pose.speed_mm_s));
    out.stationary = pose.stationary;
    out.ready = pose.ready;
}

void records::make_task(uint64_t t_ns, const TaskInfo& task, robby_link_Record& record)
{
    record = robby_link_Record_init_zero;
    record.which_kind = robby_link_Record_task_tag;
    auto& out = record.kind.task;
    out.t_ns = t_ns;
    copy_string(out.name, sizeof(out.name), task.name != nullptr ? task.name : "");
    out.priority = task.priority;
    // todo: the run time counters wrap around after a while
    out.runtime_bp =
        task.total_runtime > 0U ? static_cast<uint32_t>(uint64_t {task.runtime} * 10'000U / task.total_runtime) : 0U;
    out.stack_size_bytes = task.stack_size_bytes;
    // the high water mark: the largest share of the stack that was ever used
    const uint32_t free_bytes {std::min(task.stack_free_min_bytes, task.stack_size_bytes)};
    out.stack_usage_bp = task.stack_size_bytes > 0U ? static_cast<uint32_t>(
                                                          uint64_t {task.stack_size_bytes - free_bytes} * 10'000U /
                                                          task.stack_size_bytes
                                                      )
                                                    : 0U;
}

void records::LinkRates::fill(robby_link_LinkStats& stats, uint32_t tx_write_us)
{
    const uint64_t dt_ns {stats.t_ns > _t_ns ? stats.t_ns - _t_ns : 0U};
    // wrapping differences of the counters
    const uint32_t bytes {stats.tx_bytes - _tx_bytes};
    const uint32_t frames {stats.tx_frames - _tx_frames};
    const uint32_t write_us {tx_write_us - _tx_write_us};

    stats.tx_bytes_per_s = rate(bytes, 1'000'000'000U, dt_ns);
    stats.tx_frames_per_ks = rate(frames, 1'000'000'000'000U, dt_ns);
    // 1 µs = 1000 ns, 100 % = 10000
    stats.tx_write_time_bp = rate(write_us, 10'000'000U, dt_ns);
    // bytes/s of baud / 10 bytes/s (start and stop bit), 100 % = 10000
    stats.tx_load_bp = stats.baud > 0U ? rate(stats.tx_bytes_per_s, 100'000U, stats.baud) : 0U;

    _t_ns = stats.t_ns;
    _tx_bytes = stats.tx_bytes;
    _tx_frames = stats.tx_frames;
    _tx_write_us = tx_write_us;
}

void records::make_log(
    uint64_t t_ns,
    uint8_t level,
    uint8_t component,
    std::string_view message,
    robby_link_Record& record
)
{
    record = robby_link_Record_init_zero;
    record.which_kind = robby_link_Record_log_tag;
    auto& out = record.kind.log;
    out.t_ns = t_ns;
    out.level = static_cast<robby_link_LogLevel>(std::min<uint8_t>(level, robby_link_LogLevel_LOG_LEVEL_FATAL));
    out.component_id = static_cast<robby_link_Component>(component);
    copy_string(out.message, sizeof(out.message), message);
}

void records::make_action_result(uint32_t id, const char* error, robby_link_Record& record)
{
    record = robby_link_Record_init_zero;
    record.which_kind = robby_link_Record_action_result_tag;
    auto& out = record.kind.action_result;
    out.id = id;
    copy_string(out.error, sizeof(out.error), error != nullptr ? error : "");
}

void records::copy_string(char* field, std::size_t size, std::string_view text)
{
    if (size == 0U) {
        return;
    }
    std::size_t length {std::min(text.size(), size - 1U)};
    if (length < text.size()) {
        // don't cut a multi-byte character
        while (length > 0U && is_utf8_continuation(text[length])) {
            --length;
        }
    }
    std::memcpy(field, text.data(), length);
    field[length] = '\0';
}
