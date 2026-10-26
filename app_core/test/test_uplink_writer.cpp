#include "support.h"
#include "trace/crc16.h"
#include "trace/records.h"
#include "trace/uplink_writer.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_range.hpp>
#include <string>

using namespace test;
using robby::UplinkWriter;

namespace {

const char* const UPLINK_VECTORS[] {
    "uplink_imu_full",
    "uplink_imu_worst",
    "uplink_distance",
    "uplink_status",
    "uplink_log",
    "uplink_synthetic",
};

robby_link_Record action_result(uint32_t id)
{
    robby_link_Record record {};
    robby::records::make_action_result(id, nullptr, record);
    return record;
}

robby_link_Record full_imu_batch()
{
    robby_link_Record record = robby_link_Record_init_zero;
    record.which_kind = robby_link_Record_imu_tag;
    auto& batch = record.kind.imu;
    batch.t_first_ns = 3'600'000'000'000U;
    batch.period_ns = 1'240'694U;
    batch.x_count = batch.y_count = batch.z_count = 100U;
    for (int i = 0; i < 100; ++i) {
        batch.x[i] = static_cast<int16_t>(-150 + i);
        batch.y[i] = static_cast<int16_t>(100 - i);
        batch.z[i] = static_cast<int16_t>(16'380 + i);
    }
    return record;
}

}  // namespace

TEST_CASE("Each uplink vector decodes to its JSON")
{
    const std::string name {GENERATE(from_range(UPLINK_VECTORS))};
    CAPTURE(name);

    const auto uplink = decode_uplink(read_vector(name + ".bin"));
    REQUIRE(uplink);
    CHECK(to_json(*uplink) == read_json(name)["value"]);
}

TEST_CASE("The writer re-encodes each uplink vector to its .frame, byte for byte")
{
    const std::string name {GENERATE(from_range(UPLINK_VECTORS))};
    CAPTURE(name);

    const auto uplink = decode_uplink(read_vector(name + ".bin"));
    REQUIRE(uplink);

    Transport transport {};
    UplinkWriter writer {transport, &robby::crc16_ccitt_false, uplink->seq};
    for (const auto& record : uplink->records) {
        REQUIRE(writer.add(record));
    }
    CHECK(transport.frames.empty());
    writer.flush();

    REQUIRE(transport.frames.size() == 1U);
    CHECK(transport.frames[0] == read_vector(name + ".frame"));
    CHECK(writer.seq() == uplink->seq + 1U);
    CHECK(writer.counters().tx_frames == 1U);
    CHECK(writer.counters().tx_bytes == transport.frames[0].size());
}

TEST_CASE("A frame without records isn't sent")
{
    Transport transport {};
    UplinkWriter writer {transport, &robby::crc16_ccitt_false};
    writer.flush();
    CHECK(transport.frames.empty());
    CHECK(writer.seq() == 0U);
}

TEST_CASE("seq starts at 0 and counts the frames")
{
    Transport transport {};
    UplinkWriter writer {transport, &robby::crc16_ccitt_false};
    for (uint32_t i = 0; i < 3U; ++i) {
        writer.add(action_result(i));
        writer.flush();
    }

    REQUIRE(transport.frames.size() == 3U);
    for (uint32_t i = 0; i < 3U; ++i) {
        const auto payload = unframe(transport.frames[i]);
        REQUIRE(payload);
        const auto uplink = decode_uplink(*payload);
        REQUIRE(uplink);
        CHECK(uplink->seq == i);
        REQUIRE(uplink->records.size() == 1U);
        CHECK(uplink->records[0].kind.action_result.id == i);
    }
}

TEST_CASE("Records are packed into frames of at most 1022 bytes, in order")
{
    Transport transport {};
    UplinkWriter writer {transport, &robby::crc16_ccitt_false, 0xFFFF'FFFEU};

    // 7 batches of 100 samples don't fit into one frame, nor do 2
    constexpr uint32_t BATCHES {7U};
    for (uint32_t i = 0; i < BATCHES; ++i) {
        auto record = full_imu_batch();
        record.kind.imu.t_first_ns += i;
        REQUIRE(writer.add(record));
    }
    writer.flush();

    REQUIRE(transport.frames.size() == BATCHES);
    uint64_t next_t {3'600'000'000'000U};
    uint32_t seq {0xFFFF'FFFEU};
    for (const auto& frame : transport.frames) {
        CHECK(frame.size() <= robby::frame::MAX_ENCODED);
        const auto payload = unframe(frame);
        REQUIRE(payload);
        CHECK(payload->size() <= robby::frame::MAX_PAYLOAD);
        const auto uplink = decode_uplink(*payload);
        REQUIRE(uplink);
        // wraps at 2^32
        CHECK(uplink->seq == seq++);
        for (const auto& record : uplink->records) {
            CHECK(record.kind.imu.t_first_ns == next_t++);
        }
    }
    CHECK(next_t == 3'600'000'000'000U + BATCHES);
}

TEST_CASE("Small records fill a frame up to 32 records")
{
    Transport transport {};
    UplinkWriter writer {transport, &robby::crc16_ccitt_false};
    for (uint32_t i = 0; i < 70U; ++i) {
        REQUIRE(writer.add(action_result(i)));
    }
    writer.flush();

    REQUIRE(transport.frames.size() == 3U);
    std::size_t counts[3] {};
    uint32_t next_id {0U};
    for (std::size_t f = 0; f < 3U; ++f) {
        const auto uplink = decode_uplink(*unframe(transport.frames[f]));
        REQUIRE(uplink);
        counts[f] = uplink->records.size();
        for (const auto& record : uplink->records) {
            CHECK(record.kind.action_result.id == next_id++);
        }
    }
    CHECK(counts[0] == UplinkWriter::MAX_RECORDS);
    CHECK(counts[1] == UplinkWriter::MAX_RECORDS);
    CHECK(counts[2] == 6U);
}

TEST_CASE("A frame the transport drops is counted and leaves a gap in seq")
{
    Transport transport {};
    UplinkWriter writer {transport, &robby::crc16_ccitt_false};

    writer.add(action_result(1));
    writer.flush();
    transport.drop = true;
    writer.add(action_result(2));
    writer.flush();
    transport.drop = false;
    writer.add(action_result(3));
    writer.flush();

    REQUIRE(transport.frames.size() == 2U);
    CHECK(decode_uplink(*unframe(transport.frames[1]))->seq == 2U);
    CHECK(writer.counters().tx_frames == 2U);
    CHECK(writer.counters().tx_frames_dropped == 1U);
    CHECK(writer.counters().records_dropped == 0U);
}

TEST_CASE("A record that doesn't encode is dropped")
{
    Transport transport {};
    UplinkWriter writer {transport, &robby::crc16_ccitt_false};

    auto record = full_imu_batch();
    // more samples than the static array holds
    record.kind.imu.x_count = 101U;
    CHECK_FALSE(writer.add(record));
    CHECK(writer.counters().records_dropped == 1U);

    writer.flush();
    CHECK(transport.frames.empty());
}

TEST_CASE("Bytes per IMU sample on the wire")
{
    // the realistic batch of the vectors: 100 samples at rest
    const auto frame = read_vector("uplink_imu_full.frame");
    const double per_sample {static_cast<double>(frame.size()) / 100.0};
    CHECK(per_sample < 7.5);

    // 7 batches in their frames, as the publisher sends them
    Transport transport {};
    UplinkWriter writer {transport, &robby::crc16_ccitt_false, 1000U};
    for (int i = 0; i < 7; ++i) {
        writer.add(full_imu_batch());
    }
    writer.flush();
    std::size_t bytes {0U};
    for (const auto& f : transport.frames) {
        bytes += f.size();
    }
    const double synthetic_per_sample {static_cast<double>(bytes) / 700.0};
    UNSCOPED_INFO("IMU vector: " << per_sample << " B/sample, test batches: " << synthetic_per_sample << " B/sample");
    CHECK(synthetic_per_sample < 7.5);
}
