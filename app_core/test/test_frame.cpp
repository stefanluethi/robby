#include "support.h"
#include "trace/frame.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_range.hpp>
#include <string>

using namespace test;

namespace {

const char* const VECTORS[] {
    "uplink_empty",
    "uplink_imu_full",
    "uplink_imu_worst",
    "uplink_distance",
    "uplink_status",
    "uplink_log",
    "uplink_synthetic",
    "downlink_motor_speed",
    "downlink_motor_speed_zero",
    "downlink_bench_load",
    "downlink_invoke_no_args",
};

}  // namespace

TEST_CASE("The frame of each vector's payload is its .frame, byte for byte")
{
    const std::string name {GENERATE(from_range(VECTORS))};
    CAPTURE(name);

    const Bytes payload {read_vector(name + ".bin")};
    const Bytes expected {read_vector(name + ".frame")};
    const auto json = read_json(name);
    CHECK(payload.size() == json["payload_bytes"].get<std::size_t>());

    CHECK(frame_of(payload) == expected);
    CHECK(unframe(expected) == payload);
}

TEST_CASE("An empty payload is the frame 03 FF FF 00") { CHECK(frame_of({}) == Bytes {0x03, 0xFF, 0xFF, 0x00}); }

TEST_CASE("A payload larger than 1022 bytes doesn't make a frame")
{
    Bytes payload(robby::frame::MAX_PAYLOAD + 1U + robby::frame::CRC_SIZE, 0x55);
    Bytes out(2048);
    CHECK(
        robby::frame::encode(
            payload.data(),
            robby::frame::MAX_PAYLOAD + 1U,
            &robby::crc16_ccitt_false,
            out.data(),
            out.size()
        ) == 0U
    );
}

TEST_CASE("The largest payload without zeros fits the largest frame")
{
    Bytes payload(robby::frame::MAX_DECODED, 0x55);
    Bytes out(robby::frame::MAX_ENCODED);
    const std::size_t length {robby::frame::encode(
        payload.data(),
        robby::frame::MAX_PAYLOAD,
        &robby::crc16_ccitt_false,
        out.data(),
        out.size()
    )};
    REQUIRE(length > 0U);
    CHECK(length <= robby::frame::MAX_ENCODED);
    out.resize(length);
    CHECK(out.back() == 0x00);
    CHECK(unframe(out) == Bytes(robby::frame::MAX_PAYLOAD, 0x55));
}
