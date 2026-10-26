#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <robby_link.pb.h>

namespace robby {

/// The load generator of the "bench/load" action: Synthetic records with IMU-like samples
/// at `rate` samples/s in batches of `batch`, on top of the normal stream.
///
/// configure() may be called from any task, generate() from the publishing task only.
class BenchLoad
{
public:
    static constexpr uint32_t MAX_RATE {100'000U};
    static constexpr uint32_t MAX_BATCH {100U};
    static constexpr uint32_t DEFAULT_BATCH {100U};
    /// Batches whose last sample is due longer than this are dropped instead of sent late
    static constexpr uint64_t MAX_BACKLOG_NS {100'000'000U};

    /// Sets the load, a rate of 0 stops it. `batch` 0 takes the default. Returns why the
    /// arguments were rejected, or nullptr.
    const char* configure(uint32_t rate, uint32_t batch);

    /// Emits the batches due at `now_ns` with `emit(const robby_link_Record&)`, filled into
    /// `record`. Returns the number of batches dropped because they fell behind.
    template<class Emit>
    uint32_t generate(uint64_t now_ns, robby_link_Record& record, Emit&& emit)
    {
        uint32_t dropped {0U};
        while (next(now_ns, record, dropped)) {
            emit(record);
        }
        return dropped;
    }

    uint32_t rate() const { return _config.load(std::memory_order_relaxed) >> BATCH_BITS; }

private:
    static constexpr uint32_t BATCH_BITS {8U};
    static_assert(MAX_BATCH < (1U << BATCH_BITS));
    static_assert(MAX_RATE < (1U << (32U - BATCH_BITS)));

    /// Fills the next batch into `record` if it is due
    bool next(uint64_t now_ns, robby_link_Record& record, uint32_t& dropped);

    /// rate << BATCH_BITS | batch, so that both change together
    std::atomic<uint32_t> _config {0U};

    // state of the generating task
    uint32_t _active {0U};
    uint64_t _next_ns {0U};
    uint32_t _sample {0U};
};

}  // namespace robby
