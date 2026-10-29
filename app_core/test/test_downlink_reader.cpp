#include "support.h"
#include "trace/crc16.h"
#include "trace/downlink_reader.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_range.hpp>
#include <string>
#include <vector>

using namespace test;
using robby::DownlinkReader;

namespace {

const char* const DOWNLINK_VECTORS[] {
    "downlink_motor_drive",
    "downlink_motor_drive_zero",
    "downlink_bench_load",
    "downlink_invoke_no_args",
};

/// Feeds the bytes, returns the Downlinks that decoded
std::vector<robby_link_Downlink> feed(DownlinkReader& reader, const Bytes& bytes)
{
    std::vector<robby_link_Downlink> decoded {};
    for (uint8_t byte : bytes) {
        if (reader.push(byte)) {
            decoded.push_back(reader.downlink());
        }
    }
    return decoded;
}

Bytes motor_drive_frame(uint32_t seq, uint32_t id, int32_t speed)
{
    robby_link_Downlink downlink = robby_link_Downlink_init_zero;
    downlink.seq = seq;
    downlink.which_kind = robby_link_Downlink_invoke_tag;
    downlink.kind.invoke.id = id;
    downlink.kind.invoke.which_args = robby_link_Invoke_motor_drive_tag;
    downlink.kind.invoke.args.motor_drive.speed = speed;
    downlink.kind.invoke.args.motor_drive.turn = -speed;
    return frame_of(encode_downlink(downlink));
}

Bytes concat(std::initializer_list<Bytes> parts)
{
    Bytes all {};
    for (const auto& part : parts) {
        all.insert(all.end(), part.begin(), part.end());
    }
    return all;
}

}  // namespace

TEST_CASE("Each downlink vector decodes to its JSON")
{
    const std::string name {GENERATE(from_range(DOWNLINK_VECTORS))};
    CAPTURE(name);

    DownlinkReader reader {&robby::crc16_ccitt_false};
    const auto decoded = feed(reader, read_vector(name + ".frame"));

    REQUIRE(decoded.size() == 1U);
    CHECK(to_json(decoded[0]) == read_json(name)["value"]);
    CHECK(reader.counters().rx_frames == 1U);
    CHECK(reader.counters().rx_errors == 0U);
}

TEST_CASE("Our Downlink encoding matches the vectors")
{
    const std::string name {GENERATE(from_range(DOWNLINK_VECTORS))};
    CAPTURE(name);

    DownlinkReader reader {&robby::crc16_ccitt_false};
    const auto decoded = feed(reader, read_vector(name + ".frame"));
    REQUIRE(decoded.size() == 1U);
    CHECK(encode_downlink(decoded[0]) == read_vector(name + ".bin"));
}

TEST_CASE("Frames split across reads and back to back decode")
{
    DownlinkReader reader {&robby::crc16_ccitt_false};
    const Bytes stream {concat({
        read_vector("downlink_motor_drive.frame"),
        read_vector("downlink_motor_drive_zero.frame"),
        read_vector("downlink_bench_load.frame"),
        read_vector("downlink_invoke_no_args.frame"),
    })};

    std::vector<robby_link_Downlink> decoded {};
    // in chunks of 5 bytes, as the DMA might hand them over
    for (std::size_t i = 0; i < stream.size(); i += 5U) {
        const Bytes chunk(stream.begin() + i, stream.begin() + std::min(i + 5U, stream.size()));
        const auto part = feed(reader, chunk);
        decoded.insert(decoded.end(), part.begin(), part.end());
    }

    REQUIRE(decoded.size() == 4U);
    CHECK(decoded[0].kind.invoke.args.motor_drive.speed == 200);
    CHECK(decoded[0].kind.invoke.args.motor_drive.turn == -50);
    CHECK(decoded[1].kind.invoke.which_args == robby_link_Invoke_motor_drive_tag);
    CHECK(decoded[1].kind.invoke.args.motor_drive.speed == 0);
    CHECK(decoded[1].kind.invoke.args.motor_drive.turn == 0);
    CHECK(decoded[2].kind.invoke.args.bench_load.rate == 20'000U);
    CHECK(decoded[3].kind.invoke.which_args == 0U);
    CHECK(reader.counters().rx_seq_gaps == 0U);
}

TEST_CASE("Repeated delimiters are no frames")
{
    DownlinkReader reader {&robby::crc16_ccitt_false};
    CHECK(feed(reader, {0x00, 0x00, 0x00}).empty());
    CHECK(reader.counters().rx_errors == 0U);
    CHECK(reader.counters().rx_frames == 0U);
}

TEST_CASE("A frame with a wrong CRC is counted and skipped")
{
    DownlinkReader reader {&robby::crc16_ccitt_false};
    Bytes broken {read_vector("downlink_motor_drive.frame")};
    broken[4] ^= 0x01U;

    const auto decoded = feed(reader, concat({broken, read_vector("downlink_bench_load.frame")}));
    REQUIRE(decoded.size() == 1U);
    CHECK(decoded[0].kind.invoke.id == 3U);
    CHECK(reader.counters().rx_errors == 1U);
    CHECK(reader.counters().rx_frames == 1U);
}

TEST_CASE("Frames that fail COBS or protobuf are counted")
{
    DownlinkReader reader {&robby::crc16_ccitt_false};

    // a code byte pointing past the end
    CHECK(feed(reader, {0x09, 0x01, 0x00}).empty());
    // too short for a CRC
    CHECK(feed(reader, {0x02, 0x08, 0x00}).empty());
    // valid CRC, but no protobuf: a field with wire type 7
    CHECK(feed(reader, frame_of({0x0F, 0x01})).empty());

    CHECK(reader.counters().rx_errors == 3U);
    CHECK(reader.counters().rx_frames == 0U);
}

TEST_CASE("A frame longer than 1 KiB is dropped, the next one decodes")
{
    DownlinkReader reader {&robby::crc16_ccitt_false};
    Bytes long_frame(1500U, 0x01);
    long_frame.push_back(0x00);

    const auto decoded = feed(reader, concat({long_frame, read_vector("downlink_motor_drive.frame")}));
    REQUIRE(decoded.size() == 1U);
    CHECK(reader.counters().rx_errors == 1U);
}

TEST_CASE("Gaps in seq are counted, restarts aren't")
{
    DownlinkReader reader {&robby::crc16_ccitt_false};

    // the first frame sets the expectation
    feed(reader, motor_drive_frame(7, 1, 100));
    feed(reader, motor_drive_frame(8, 2, 100));
    CHECK(reader.counters().rx_seq_gaps == 0U);

    // 9 and 10 lost
    feed(reader, motor_drive_frame(11, 3, 100));
    CHECK(reader.counters().rx_seq_gaps == 2U);

    // the outpost restarted
    feed(reader, motor_drive_frame(0, 4, 100));
    feed(reader, motor_drive_frame(1, 5, 100));
    CHECK(reader.counters().rx_seq_gaps == 2U);

    // going back counts as a restart too
    feed(reader, motor_drive_frame(5, 6, 100));
    feed(reader, motor_drive_frame(3, 7, 100));
    CHECK(reader.counters().rx_seq_gaps == 5U);

    // wraps at 2^32
    feed(reader, motor_drive_frame(0xFFFF'FFFFU, 8, 100));
    const auto gaps = reader.counters().rx_seq_gaps.load();
    feed(reader, motor_drive_frame(1, 9, 100));
    CHECK(reader.counters().rx_seq_gaps == gaps + 1U);
    CHECK(reader.counters().rx_frames == 9U);
}

TEST_CASE("A frame lost to a CRC error shows as a gap")
{
    DownlinkReader reader {&robby::crc16_ccitt_false};
    Bytes broken {motor_drive_frame(2, 2, 100)};
    broken[3] ^= 0x10U;

    feed(reader, concat({motor_drive_frame(1, 1, 100), broken, motor_drive_frame(3, 3, 100)}));
    CHECK(reader.counters().rx_frames == 2U);
    CHECK(reader.counters().rx_errors == 1U);
    CHECK(reader.counters().rx_seq_gaps == 1U);
}
