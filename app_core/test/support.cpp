#include "support.h"

#include "trace/crc16.h"
#include "trace/frame.h"

#include <cobs.h>

#include <fstream>
#include <iterator>

#include <pb_decode.h>
#include <pb_encode.h>

#include <stdexcept>

using nlohmann::json;

namespace {

std::string path_of(const std::string& name) { return std::string {ROBBY_VECTORS_DIR} + "/" + name; }

template<class T>
json array_of(const T* values, pb_size_t count)
{
    json array = json::array();
    for (pb_size_t i = 0; i < count; ++i) {
        array.push_back(values[i]);
    }
    return array;
}

const char* level_name(robby_link_LogLevel level)
{
    switch (level) {
        case robby_link_LogLevel_LOG_LEVEL_TRACE: return "LOG_LEVEL_TRACE";
        case robby_link_LogLevel_LOG_LEVEL_DEBUG: return "LOG_LEVEL_DEBUG";
        case robby_link_LogLevel_LOG_LEVEL_INFO: return "LOG_LEVEL_INFO";
        case robby_link_LogLevel_LOG_LEVEL_WARN: return "LOG_LEVEL_WARN";
        case robby_link_LogLevel_LOG_LEVEL_ERROR: return "LOG_LEVEL_ERROR";
        case robby_link_LogLevel_LOG_LEVEL_FATAL: return "LOG_LEVEL_FATAL";
    }
    return "?";
}

const char* component_name(robby_link_Component component)
{
    switch (component) {
        case robby_link_Component_COMPONENT_CMD: return "COMPONENT_CMD";
        case robby_link_Component_COMPONENT_SPACE: return "COMPONENT_SPACE";
        case robby_link_Component_COMPONENT_IMU: return "COMPONENT_IMU";
        case robby_link_Component_COMPONENT_DRIVE: return "COMPONENT_DRIVE";
    }
    return "?";
}

template<class T>
json samples(const T& batch)
{
    return {
        {"t_first_ns", batch.t_first_ns},
        {"period_ns", batch.period_ns},
        {"x", array_of(batch.x, batch.x_count)},
        {"y", array_of(batch.y, batch.y_count)},
        {"z", array_of(batch.z, batch.z_count)},
    };
}

}  // namespace

test::Bytes test::read_vector(const std::string& name)
{
    std::ifstream file {path_of(name), std::ios::binary};
    if (!file) {
        throw std::runtime_error {"missing vector " + name};
    }
    return {std::istreambuf_iterator<char> {file}, std::istreambuf_iterator<char> {}};
}

json test::read_json(const std::string& name)
{
    std::ifstream file {path_of(name + ".json")};
    if (!file) {
        throw std::runtime_error {"missing vector " + name};
    }
    return json::parse(file);
}

std::optional<test::Uplink> test::decode_uplink(const Bytes& payload)
{
    Uplink uplink {};
    pb_istream_t stream = pb_istream_from_buffer(payload.data(), payload.size());
    while (stream.bytes_left > 0U) {
        pb_wire_type_t wire_type {};
        uint32_t tag {0U};
        bool eof {false};
        if (!pb_decode_tag(&stream, &wire_type, &tag, &eof)) {
            return std::nullopt;
        }
        if (tag == robby_link_Uplink_seq_tag && wire_type == PB_WT_VARINT) {
            if (!pb_decode_varint32(&stream, &uplink.seq)) {
                return std::nullopt;
            }
        } else if (tag == robby_link_Uplink_records_tag && wire_type == PB_WT_STRING) {
            robby_link_Record record = robby_link_Record_init_zero;
            pb_istream_t substream {};
            if (!pb_make_string_substream(&stream, &substream) ||
                !pb_decode(&substream, robby_link_Record_fields, &record) ||
                !pb_close_string_substream(&stream, &substream)) {
                return std::nullopt;
            }
            uplink.records.push_back(record);
        } else if (!pb_skip_field(&stream, wire_type)) {
            return std::nullopt;
        }
    }
    return uplink;
}

std::optional<test::Bytes> test::unframe(const Bytes& frame)
{
    Bytes decoded(robby::frame::MAX_DECODED);
    std::size_t length {0U};
    if (cobs_decode(frame.data(), frame.size(), decoded.data(), decoded.size(), &length) != COBS_RET_SUCCESS ||
        length < robby::frame::CRC_SIZE || robby::crc16_ccitt_false(decoded.data(), length) != 0U) {
        return std::nullopt;
    }
    decoded.resize(length - robby::frame::CRC_SIZE);
    return decoded;
}

test::Bytes test::frame_of(Bytes payload)
{
    const std::size_t length {payload.size()};
    payload.resize(length + robby::frame::CRC_SIZE);
    Bytes frame(robby::frame::MAX_ENCODED);
    frame.resize(robby::frame::encode(payload.data(), length, &robby::crc16_ccitt_false, frame.data(), frame.size()));
    return frame;
}

test::Bytes test::encode_downlink(const robby_link_Downlink& downlink)
{
    Bytes payload(robby_link_Downlink_size);
    pb_ostream_t stream = pb_ostream_from_buffer(payload.data(), payload.size());
    if (!pb_encode(&stream, robby_link_Downlink_fields, &downlink)) {
        throw std::runtime_error {"encoding the downlink failed"};
    }
    payload.resize(stream.bytes_written);
    return payload;
}

json test::to_json(const robby_link_Record& record)
{
    const auto& kind = record.kind;
    switch (record.which_kind) {
        case robby_link_Record_imu_tag:
            return {
                {"imu", samples(kind.imu)}
            };
        case robby_link_Record_synthetic_tag:
            return {
                {"synthetic", samples(kind.synthetic)}
            };
        case robby_link_Record_distance_tag:
            return {
                {"distance", {{"t_ns", kind.distance.t_ns}, {"mm", array_of(kind.distance.mm, kind.distance.mm_count)}}}
            };
        case robby_link_Record_battery_tag: {
            const auto& b = kind.battery;
            return {
                {"battery",
                 {{"t_ns", b.t_ns},
                  {"soc_pct", b.soc_pct},
                  {"voltage_mv", b.voltage_mv},
                  {"cell1_mv", b.cell1_mv},
                  {"cell2_mv", b.cell2_mv},
                  {"current_ma", b.current_ma}}}
            };
        }
        case robby_link_Record_task_tag: {
            const auto& t = kind.task;
            return {
                {"task",
                 {{"t_ns", t.t_ns},
                  {"name", t.name},
                  {"priority", t.priority},
                  {"runtime_bp", t.runtime_bp},
                  {"stack_size_bytes", t.stack_size_bytes},
                  {"stack_usage_bp", t.stack_usage_bp}}}
            };
        }
        case robby_link_Record_log_tag: {
            const auto& l = kind.log;
            return {
                {"log",
                 {{"t_ns", l.t_ns},
                  {"level", level_name(l.level)},
                  {"component_id", component_name(l.component_id)},
                  {"message", l.message}}}
            };
        }
        case robby_link_Record_action_result_tag:
            return {
                {"action_result", {{"id", kind.action_result.id}, {"error", kind.action_result.error}}}
            };
        case robby_link_Record_link_stats_tag: {
            const auto& s = kind.link_stats;
            return {
                {"link_stats",
                 {{"t_ns", s.t_ns},
                  {"tx_bytes", s.tx_bytes},
                  {"tx_frames", s.tx_frames},
                  {"tx_write_max_us", s.tx_write_max_us},
                  {"tx_frames_dropped", s.tx_frames_dropped},
                  {"records_dropped", s.records_dropped},
                  {"rx_frames", s.rx_frames},
                  {"rx_errors", s.rx_errors},
                  {"rx_seq_gaps", s.rx_seq_gaps},
                  {"baud", s.baud},
                  {"tx_bytes_per_s", s.tx_bytes_per_s},
                  {"tx_frames_per_ks", s.tx_frames_per_ks},
                  {"tx_load_bp", s.tx_load_bp},
                  {"tx_write_time_bp", s.tx_write_time_bp}}}
            };
        }
        case robby_link_Record_drive_tag: {
            const auto& d = kind.drive;
            return {
                {"drive",
                 {{"t_ns", d.t_ns},
                  {"left_target_mm_s", d.left_target_mm_s},
                  {"right_target_mm_s", d.right_target_mm_s},
                  {"left_speed_mm_s", d.left_speed_mm_s},
                  {"right_speed_mm_s", d.right_speed_mm_s},
                  {"left_duty", d.left_duty},
                  {"right_duty", d.right_duty},
                  {"left_counts", d.left_counts},
                  {"right_counts", d.right_counts}}}
            };
        }
        case robby_link_Record_pose_tag: {
            const auto& p = kind.pose;
            return {
                {"pose",
                 {{"t_ns", p.t_ns}, {"x_mm", p.x_mm}, {"y_mm", p.y_mm}, {"heading_cdeg", p.heading_cdeg}}}
            };
        }
        case robby_link_Record_imu_pose_tag: {
            const auto& p = kind.imu_pose;
            return {
                {"imu_pose",
                 {{"t_ns", p.t_ns},
                  {"x_mm", p.x_mm},
                  {"y_mm", p.y_mm},
                  {"heading_cdeg", p.heading_cdeg},
                  {"speed_mm_s", p.speed_mm_s},
                  {"stationary", p.stationary},
                  {"ready", p.ready}}}
            };
        }
        default: return json::object();
    }
}

json test::to_json(const Uplink& uplink)
{
    json records = json::array();
    for (const auto& record : uplink.records) {
        records.push_back(to_json(record));
    }
    return {
        {"seq",     uplink.seq},
        {"records", records   }
    };
}

json test::to_json(const robby_link_Downlink& downlink)
{
    json value {
        {"seq", downlink.seq}
    };
    if (downlink.which_kind == robby_link_Downlink_invoke_tag) {
        const auto& invoke = downlink.kind.invoke;
        json out {
            {"id", invoke.id}
        };
        if (invoke.which_args == robby_link_Invoke_motor_drive_tag) {
            out["motor_drive"] = {
                {"speed", invoke.args.motor_drive.speed},
                {"turn",  invoke.args.motor_drive.turn }
            };
        } else if (invoke.which_args == robby_link_Invoke_bench_load_tag) {
            out["bench_load"] = {
                {"rate",  invoke.args.bench_load.rate },
                {"batch", invoke.args.bench_load.batch}
            };
        }
        value["invoke"] = out;
    }
    return value;
}
