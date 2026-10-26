#include "records.h"

#include <algorithm>
#include <cstring>

using namespace robby;

namespace {

static_assert(sizeof(robby_link_ImuBatch::x) / sizeof(robby_link_ImuBatch::x[0]) >= AccelerationFrame {}.x.size());
static_assert(sizeof(robby_link_DistanceImage::mm) / sizeof(robby_link_DistanceImage::mm[0]) == 3U * 8U * 8U);

constexpr bool is_utf8_continuation(char c) { return (static_cast<uint8_t>(c) & 0xC0U) == 0x80U; }

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

    // sensors[s][x][y] in this order: index (s * 8 + x) * 8 + y, invalid zones as -1
    pb_size_t count {0U};
    for (const auto& sensor : sensors) {
        for (const auto& row : sensor) {
            for (int16_t distance : row) {
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
    out.stack_free_min_bytes = task.stack_free_min_bytes;
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
