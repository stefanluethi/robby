#include "service/bench_load.h"
#include "support.h"
#include "trace/crc16.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <pb_encode.h>

#include <string>
#include <vector>

using namespace test;
using robby::BenchLoad;

namespace {

constexpr uint64_t START_NS {5'000'000'000U};
constexpr uint64_t TICK_NS {10'000'000U};

/// Runs the generator every 10 ms for `duration_ns`, returns the records
std::vector<robby_link_Record> run(BenchLoad& load, uint64_t& now, uint64_t duration_ns, uint32_t* dropped = nullptr)
{
    std::vector<robby_link_Record> out {};
    robby_link_Record record {};
    const uint64_t end {now + duration_ns};
    for (; now <= end; now += TICK_NS) {
        const uint32_t d {load.generate(now, record, [&](const robby_link_Record& r) { out.push_back(r); })};
        if (dropped != nullptr) {
            *dropped += d;
        }
    }
    return out;
}

}  // namespace

TEST_CASE("The bench load is off at first")
{
    BenchLoad load {};
    uint64_t now {START_NS};
    CHECK(run(load, now, 1'000'000'000U).empty());
}

TEST_CASE("The bench load checks its arguments")
{
    BenchLoad load {};
    CHECK(load.configure(0U, 0U) == nullptr);
    CHECK(load.configure(100'000U, 100U) == nullptr);
    CHECK(std::string {load.configure(100'001U, 100U)} == "rate must be an integer from 0 to 100000");
    CHECK(std::string {load.configure(1'000U, 101U)} == "batch must be an integer from 0 to 100");
    // a rejected configuration keeps the previous one
    CHECK(load.rate() == 100'000U);
}

TEST_CASE("The bench load emits its rate in contiguous batches")
{
    BenchLoad load {};
    REQUIRE(load.configure(20'000U, 0U) == nullptr);

    uint64_t now {START_NS};
    const auto records = run(load, now, 1'000'000'000U);

    // 20000 samples in 1 s, batches of 100, the last ones may still be due
    REQUIRE(records.size() >= 198U);
    REQUIRE(records.size() <= 200U);

    uint64_t next {START_NS};
    for (const auto& record : records) {
        REQUIRE(record.which_kind == robby_link_Record_synthetic_tag);
        const auto& batch = record.kind.synthetic;
        CHECK(batch.period_ns == 50'000U);
        CHECK(batch.x_count == 100U);
        CHECK(batch.y_count == 100U);
        CHECK(batch.z_count == 100U);
        CHECK(batch.t_first_ns == next);
        // never ahead of time
        next = batch.t_first_ns + 100U * batch.period_ns;
    }
    CHECK(next <= now);
}

TEST_CASE("The bench load takes the batch size")
{
    BenchLoad load {};
    REQUIRE(load.configure(1'000U, 10U) == nullptr);
    uint64_t now {START_NS};
    const auto records = run(load, now, 100'000'000U);
    REQUIRE_FALSE(records.empty());
    CHECK(records[0].kind.synthetic.x_count == 10U);
    CHECK(records[0].kind.synthetic.period_ns == 1'000'000U);
}

TEST_CASE("A bench load that falls behind drops batches instead of sending them late")
{
    BenchLoad load {};
    REQUIRE(load.configure(10'000U, 100U) == nullptr);

    uint64_t now {START_NS};
    run(load, now, 50'000'000U);

    // the publisher was blocked for a second
    now += 1'000'000'000U;
    uint32_t dropped {0U};
    std::vector<robby_link_Record> out {};
    robby_link_Record record {};
    dropped += load.generate(now, record, [&](const robby_link_Record& r) { out.push_back(r); });

    // at most 100 ms of backlog are sent, the rest is dropped
    CHECK(out.size() <= 11U);
    CHECK(dropped >= 89U);
    CHECK(out.size() + dropped >= 99U);
    // measured at the last sample of each batch
    const auto& first = out.front().kind.synthetic;
    CHECK(now - (first.t_first_ns + 99U * first.period_ns) <= BenchLoad::MAX_BACKLOG_NS);
    // the batch before the first one sent was too late
    CHECK(now - (first.t_first_ns - first.period_ns) > BenchLoad::MAX_BACKLOG_NS);
}

TEST_CASE("Batches that span the backlog limit or more aren't dropped")
{
    // 99 ms per batch at 1000/s (the bench's first ramp step), 198 ms at 500/s, 1.98 s at 50/s
    const uint32_t rate {GENERATE(1'000U, 500U, 50U)};
    CAPTURE(rate);

    BenchLoad load {};
    REQUIRE(load.configure(rate, 100U) == nullptr);

    uint64_t now {START_NS};
    uint32_t dropped {0U};
    // the publisher runs every 10 ms
    const auto records = run(load, now, 20'000'000'000U, &dropped);

    CHECK(dropped == 0U);
    // 20 s worth of samples, the last batch may not be due yet
    const std::size_t expected {rate * 20U / 100U};
    CHECK(records.size() >= expected - 1U);
    CHECK(records.size() <= expected + 1U);

    uint64_t next {START_NS};
    for (const auto& record : records) {
        CHECK(record.kind.synthetic.t_first_ns == next);
        next += 100U * record.kind.synthetic.period_ns;
    }
}

TEST_CASE("A publisher a bit slower than the batches drops nothing")
{
    BenchLoad load {};
    REQUIRE(load.configure(1'000U, 100U) == nullptr);

    // every 37 ms, a few batches due at once now and then
    uint64_t now {START_NS};
    uint32_t dropped {0U};
    std::size_t sent {0U};
    robby_link_Record record {};
    for (; now <= START_NS + 10'000'000'000U; now += 37'000'000U) {
        dropped += load.generate(now, record, [&](const robby_link_Record&) { ++sent; });
    }
    CHECK(dropped == 0U);
    CHECK(sent >= 99U);
}

TEST_CASE("Rate 0 stops the bench load")
{
    BenchLoad load {};
    REQUIRE(load.configure(10'000U, 100U) == nullptr);
    uint64_t now {START_NS};
    CHECK_FALSE(run(load, now, 100'000'000U).empty());

    REQUIRE(load.configure(0U, 0U) == nullptr);
    CHECK(run(load, now, 1'000'000'000U).empty());
}

TEST_CASE("Synthetic samples take about as many bytes as the IMU's")
{
    BenchLoad load {};
    REQUIRE(load.configure(100'000U, 100U) == nullptr);
    uint64_t now {START_NS};
    const auto records = run(load, now, 10'000'000U);
    REQUIRE_FALSE(records.empty());

    std::size_t size {0U};
    REQUIRE(pb_get_encoded_size(&size, robby_link_Record_fields, &records[0]));
    // the IMU vector: 728 bytes for 100 samples
    CHECK(size >= 650U);
    CHECK(size <= 760U);
}
