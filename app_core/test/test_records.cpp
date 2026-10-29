#include "support.h"
#include "trace/crc16.h"
#include "trace/records.h"
#include "trace/uplink_writer.h"

#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <numbers>
#include <string>

using namespace test;
using namespace robby;

namespace {

/// Sends the records as one frame with the given seq
template<class... Records>
Bytes frame_with(uint32_t seq, const Records&... records)
{
    Transport transport {};
    UplinkWriter writer {transport, &crc16_ccitt_false, seq};
    (writer.add(records), ...);
    writer.flush();
    return transport.frames.size() == 1U ? transport.frames[0] : Bytes {};
}

}  // namespace

TEST_CASE("An acceleration frame becomes the ImuBatch of the vector")
{
    const auto json = read_json("uplink_imu_full");
    const auto& imu = json["value"]["records"][0]["imu"];
    const uint64_t t_first {imu["t_first_ns"].get<uint64_t>()};
    const uint64_t period {imu["period_ns"].get<uint64_t>()};

    AccelerationFrame frame {};
    frame.size = imu["x"].size();
    for (std::size_t i = 0; i < frame.size; ++i) {
        frame.x[i] = imu["x"][i].get<int16_t>();
        frame.y[i] = imu["y"][i].get<int16_t>();
        frame.z[i] = imu["z"][i].get<int16_t>();
    }

    records::ImuBatcher batcher {};
    robby_link_Record record {};

    // the frame before, read one frame time earlier
    AccelerationFrame previous {frame};
    previous.timestamp_ns = t_first - period;
    REQUIRE(batcher.make(previous, record));

    // the last sample of the frame was read at its timestamp
    frame.timestamp_ns = t_first + (frame.size - 1U) * period;
    REQUIRE(batcher.make(frame, record));
    CHECK(record.kind.imu.t_first_ns == t_first);
    CHECK(record.kind.imu.period_ns == period);

    CHECK(frame_with(json["value"]["seq"].get<uint32_t>(), record) == read_vector("uplink_imu_full.frame"));
}

TEST_CASE("Acceleration frames are spread between their read times")
{
    records::ImuBatcher batcher {};
    robby_link_Record record {};
    AccelerationFrame frame {};
    frame.size = 100U;

    SECTION("the first frame takes the nominal period")
    {
        frame.timestamp_ns = 1'000'000'000U;
        REQUIRE(batcher.make(frame, record));
        CHECK(record.which_kind == robby_link_Record_imu_tag);
        CHECK(record.kind.imu.period_ns == ACCELERATION_SAMPLE_PERIOD_NS);
        CHECK(record.kind.imu.t_first_ns == frame.timestamp_ns - 99U * ACCELERATION_SAMPLE_PERIOD_NS);
        CHECK(record.kind.imu.x_count == 100U);
        CHECK(record.kind.imu.y_count == 100U);
        CHECK(record.kind.imu.z_count == 100U);
    }

    SECTION("a frame following the last one takes the measured period")
    {
        frame.timestamp_ns = 1'000'000'000U;
        batcher.make(frame, record);
        // 806 Hz
        frame.timestamp_ns += 124'069'400U;
        REQUIRE(batcher.make(frame, record));
        CHECK(record.kind.imu.period_ns == 1'240'694U);
        CHECK(record.kind.imu.t_first_ns == frame.timestamp_ns - 99U * 1'240'694U);
    }

    SECTION("after a gap the nominal period is used again")
    {
        frame.timestamp_ns = 1'000'000'000U;
        batcher.make(frame, record);
        frame.timestamp_ns += 500'000'000U;
        REQUIRE(batcher.make(frame, record));
        CHECK(record.kind.imu.period_ns == ACCELERATION_SAMPLE_PERIOD_NS);
    }

    SECTION("a partial frame keeps its size")
    {
        frame.size = 7U;
        frame.timestamp_ns = 1'000'000'000U;
        REQUIRE(batcher.make(frame, record));
        CHECK(record.kind.imu.x_count == 7U);
    }

    SECTION("an empty frame gives no record")
    {
        frame.size = 0U;
        CHECK_FALSE(batcher.make(frame, record));
    }
}

TEST_CASE("The distance map becomes the DistanceImage of the vector")
{
    const auto json = read_json("uplink_distance");
    const auto& distance = json["value"]["records"][0]["distance"];

    // sensors[s][y][x] = mm[(y * 3 + s) * 8 + x]
    std::array<DistanceSensorImage, 3> sensors {};
    for (std::size_t s = 0; s < 3U; ++s) {
        for (std::size_t y = 0; y < 8U; ++y) {
            for (std::size_t x = 0; x < 8U; ++x) {
                // the sensors report other negative values for invalid zones too
                const int16_t mm {distance["mm"][(y * 3U + s) * 8U + x].get<int16_t>()};
                sensors[s][y][x] = mm < 0 ? int16_t {-7} : mm;
            }
        }
    }

    robby_link_Record record {};
    records::make_distance(distance["t_ns"].get<uint64_t>(), sensors, record);
    CHECK(record.kind.distance.mm_count == 192U);
    CHECK(frame_with(json["value"]["seq"].get<uint32_t>(), record) == read_vector("uplink_distance.frame"));
}

TEST_CASE("Battery, tasks and link stats become the records of the vector")
{
    const auto json = read_json("uplink_status");
    const auto& records_json = json["value"]["records"];
    const auto& b = records_json[0]["battery"];

    const fuelgauge_status_t battery {
        .soc = b["soc_pct"].get<uint8_t>(),
        .voltage = b["voltage_mv"].get<uint16_t>(),
        .cell1_voltage = b["cell1_mv"].get<uint16_t>(),
        .cell2_voltage = b["cell2_mv"].get<uint16_t>(),
        .current = b["current_ma"].get<int16_t>(),
    };
    robby_link_Record battery_record {};
    records::make_battery(b["t_ns"].get<uint64_t>(), battery, battery_record);

    std::array<robby_link_Record, 3> tasks {};
    std::array<std::string, 3> names {};
    // the high water marks that give the stack usage of the vector
    constexpr std::array<uint32_t, 3> stack_free {2900U, 1088U, 236U};
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        const auto& t = records_json[i + 1U]["task"];
        names[i] = t["name"].get<std::string>();
        const records::TaskInfo info {
            .name = names[i].c_str(),
            .priority = t["priority"].get<uint32_t>(),
            // runtime_bp = runtime / total in 0.01 %
            .runtime = t["runtime_bp"].get<uint32_t>() * 7U,
            .total_runtime = 70'000U,
            .stack_size_bytes = t["stack_size_bytes"].get<uint32_t>(),
            .stack_free_min_bytes = stack_free[i],
        };
        records::make_task(t["t_ns"].get<uint64_t>(), info, tasks[i]);
        CHECK(tasks[i].kind.task.stack_usage_bp == t["stack_usage_bp"].get<uint32_t>());
    }

    // LinkStats is filled by robby::Link on the target, the same fields
    const auto& s = records_json[4]["link_stats"];
    robby_link_Record stats = robby_link_Record_init_zero;
    stats.which_kind = robby_link_Record_link_stats_tag;
    stats.kind.link_stats = {
        .t_ns = s["t_ns"].get<uint64_t>(),
        .tx_bytes = s["tx_bytes"].get<uint32_t>(),
        .tx_frames = s["tx_frames"].get<uint32_t>(),
        .tx_write_max_us = s["tx_write_max_us"].get<uint32_t>(),
        .tx_frames_dropped = s["tx_frames_dropped"].get<uint32_t>(),
        .records_dropped = s["records_dropped"].get<uint32_t>(),
        .rx_frames = s["rx_frames"].get<uint32_t>(),
        .rx_errors = s["rx_errors"].get<uint32_t>(),
        .rx_seq_gaps = s["rx_seq_gaps"].get<uint32_t>(),
        .baud = s["baud"].get<uint32_t>(),
        .tx_bytes_per_s = s["tx_bytes_per_s"].get<uint32_t>(),
        .tx_frames_per_ks = s["tx_frames_per_ks"].get<uint32_t>(),
        .tx_load_bp = s["tx_load_bp"].get<uint32_t>(),
        .tx_write_time_bp = s["tx_write_time_bp"].get<uint32_t>(),
    };

    CHECK(
        frame_with(json["value"]["seq"].get<uint32_t>(), battery_record, tasks[0], tasks[1], tasks[2], stats) ==
        read_vector("uplink_status.frame")
    );
}

TEST_CASE("A task without run time has 0 %")
{
    robby_link_Record record {};
    records::make_task(
        1U,
        {.name = "IDLE",
         .priority = 0U,
         .runtime = 5U,
         .total_runtime = 0U,
         .stack_size_bytes = 0U,
         .stack_free_min_bytes = 0U},
        record
    );
    CHECK(record.kind.task.runtime_bp == 0U);
    CHECK(record.kind.task.stack_usage_bp == 0U);
    CHECK(std::string {record.kind.task.name} == "IDLE");
}

TEST_CASE("The stack usage is the used share of the stack")
{
    robby_link_Record record {};
    records::TaskInfo info {.name = "t", .priority = 1U, .runtime = 0U, .total_runtime = 1U,
                            .stack_size_bytes = 1000U, .stack_free_min_bytes = 250U};
    records::make_task(1U, info, record);
    CHECK(record.kind.task.stack_usage_bp == 7500U);

    // a high water mark larger than the stack is taken as an unused stack
    info.stack_free_min_bytes = 2000U;
    records::make_task(1U, info, record);
    CHECK(record.kind.task.stack_usage_bp == 0U);
}

TEST_CASE("The link rates cover the time since the previous LinkStats")
{
    records::LinkRates rates {};
    robby_link_LinkStats stats {};
    stats.baud = 1'000'000U;

    SECTION("the first one covers the time since the reset")
    {
        stats.t_ns = 2'000'000'000U;
        stats.tx_bytes = 20'000U;
        stats.tx_frames = 25U;
        rates.fill(stats, 100'000U);
        CHECK(stats.tx_bytes_per_s == 10'000U);
        CHECK(stats.tx_frames_per_ks == 12'500U);
        // 10 kB/s of 100 kB/s
        CHECK(stats.tx_load_bp == 1'000U);
        // 0.1 s of 2 s
        CHECK(stats.tx_write_time_bp == 500U);
    }

    SECTION("later ones the difference to the one before, also when the counters wrap")
    {
        stats.t_ns = 1'000'000'000U;
        stats.tx_bytes = UINT32_MAX - 99U;
        stats.tx_frames = UINT32_MAX;
        rates.fill(stats, UINT32_MAX - 499U);

        stats.t_ns = 1'500'000'000U;
        stats.tx_bytes = 4'900U;
        stats.tx_frames = 4U;
        rates.fill(stats, 4'500U);
        CHECK(stats.tx_bytes_per_s == 10'000U);
        CHECK(stats.tx_frames_per_ks == 10'000U);
        CHECK(stats.tx_load_bp == 1'000U);
        CHECK(stats.tx_write_time_bp == 100U);
    }

    SECTION("no time passed, no rates")
    {
        stats.tx_bytes = 100U;
        rates.fill(stats, 0U);
        CHECK(stats.tx_bytes_per_s == 0U);
        CHECK(stats.tx_frames_per_ks == 0U);
        CHECK(stats.tx_load_bp == 0U);
    }
}

TEST_CASE("Logs and action results become the records of the vector")
{
    const auto json = read_json("uplink_log");
    const auto& l = json["value"]["records"][0]["log"];

    // se_oss::LogLevel::WARN, robby::LogContextId::IMU_ACQUISITION
    robby_link_Record log {};
    records::make_log(l["t_ns"].get<uint64_t>(), 3U, 2U, l["message"].get<std::string>(), log);

    robby_link_Record ok {};
    records::make_action_result(1U, nullptr, ok);
    robby_link_Record rejected {};
    records::make_action_result(2U, "speed out of range", rejected);

    CHECK(frame_with(json["value"]["seq"].get<uint32_t>(), log, ok, rejected) == read_vector("uplink_log.frame"));
}

TEST_CASE("Strings are cut to the field at a character boundary")
{
    char field[8] {};

    records::copy_string(field, sizeof(field), "short");
    CHECK(std::string {field} == "short");

    records::copy_string(field, sizeof(field), "exactly");
    CHECK(std::string {field} == "exactly");

    records::copy_string(field, sizeof(field), "much too long");
    CHECK(std::string {field} == "much to");

    // "abcde" + "ü" (2 bytes) + "x": the ü doesn't fit into 7 bytes as a whole
    records::copy_string(field, sizeof(field), "abcdef\xC3\xBCx");
    CHECK(std::string {field} == "abcdef");
    records::copy_string(field, sizeof(field), "abcde\xC3\xBCx");
    CHECK(std::string {field} == "abcde\xC3\xBC");
}

TEST_CASE("A long log message is cut to 255 bytes")
{
    const std::string message(400U, 'm');
    robby_link_Record record {};
    records::make_log(1U, 2U, 0U, message, record);
    CHECK(std::strlen(record.kind.log.message) == 255U);
}

TEST_CASE("A level above FATAL is sent as FATAL")
{
    robby_link_Record record {};
    records::make_log(1U, 0xFFU, 0U, "x", record);
    CHECK(record.kind.log.level == robby_link_LogLevel_LOG_LEVEL_FATAL);
}

TEST_CASE("A drive sample becomes the Drive record of the vector")
{
    const auto json = read_json("uplink_drive");
    const auto& d = json["value"]["records"][0]["drive"];

    const drive::Sample sample {
        .t_ns = d["t_ns"].get<uint64_t>(),
        .target = {.left = d["left_target_mm_s"].get<int32_t>(), .right = d["right_target_mm_s"].get<int32_t>()},
        .speed = {.left = d["left_speed_mm_s"].get<int32_t>(), .right = d["right_speed_mm_s"].get<int32_t>()},
        .duty = {.left = d["left_duty"].get<int32_t>(), .right = d["right_duty"].get<int32_t>()},
        .counts = {.left = d["left_counts"].get<int32_t>(), .right = d["right_counts"].get<int32_t>()},
    };
    robby_link_Record record {};
    records::make_drive(sample, record);

    CHECK(to_json(record) == json["value"]["records"][0]);
    CHECK(frame_with(json["value"]["seq"].get<uint32_t>(), record) == read_vector("uplink_drive.frame"));
}

TEST_CASE("A pose becomes the Pose record of the vector, rounded to mm and 0.01 degree")
{
    const auto json = read_json("uplink_pose");
    const auto& p = json["value"]["records"][0]["pose"];

    const drive::Pose pose {
        .x_mm = static_cast<float>(p["x_mm"].get<int32_t>()) + 0.4F,
        .y_mm = static_cast<float>(p["y_mm"].get<int32_t>()) - 0.4F,
        .heading_rad = static_cast<float>(p["heading_cdeg"].get<int32_t>()) * std::numbers::pi_v<float> / 18000.0F,
    };
    robby_link_Record record {};
    records::make_pose(p["t_ns"].get<uint64_t>(), pose, record);

    CHECK(to_json(record) == json["value"]["records"][0]);
    CHECK(frame_with(json["value"]["seq"].get<uint32_t>(), record) == read_vector("uplink_pose.frame"));
}

TEST_CASE("A heading of pi is 180 degrees")
{
    robby_link_Record record {};
    records::make_pose(0U, {.x_mm = 0.0F, .y_mm = 0.0F, .heading_rad = std::numbers::pi_v<float>}, record);
    CHECK(record.kind.pose.heading_cdeg == 18000);
    records::make_pose(0U, {.x_mm = -0.6F, .y_mm = 0.5F, .heading_rad = -std::numbers::pi_v<float>}, record);
    CHECK(record.kind.pose.heading_cdeg == -18000);
    CHECK(record.kind.pose.x_mm == -1);
    CHECK(record.kind.pose.y_mm == 1);
}
