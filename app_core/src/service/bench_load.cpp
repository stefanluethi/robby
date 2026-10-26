#include "bench_load.h"

using namespace robby;

const char* BenchLoad::configure(uint32_t rate, uint32_t batch)
{
    if (rate > MAX_RATE) {
        return "rate must be an integer from 0 to 100000";
    }
    if (batch > MAX_BATCH) {
        return "batch must be an integer from 0 to 100";
    }
    if (batch == 0U) {
        batch = DEFAULT_BATCH;
    }
    _config.store(rate << BATCH_BITS | batch, std::memory_order_relaxed);
    return nullptr;
}

bool BenchLoad::next(uint64_t now_ns, robby_link_Record& record, uint32_t& dropped)
{
    const uint32_t config {_config.load(std::memory_order_relaxed)};
    const uint32_t rate {config >> BATCH_BITS};
    const uint32_t batch {config & ((1U << BATCH_BITS) - 1U)};

    if (config != _active) {
        // a new load starts now
        _active = config;
        _next_ns = now_ns;
    }
    if (rate == 0U || batch == 0U) {
        return false;
    }

    const uint64_t period_ns {1'000'000'000U / rate};
    const uint64_t batch_ns {batch * period_ns};

    // a batch is due once its last sample is, and late by as long as that one waited
    const uint64_t last_sample_ns {batch_ns - period_ns};
    if (_next_ns + last_sample_ns > now_ns) {
        return false;
    }
    const uint64_t late_ns {now_ns - (_next_ns + last_sample_ns)};
    if (late_ns > MAX_BACKLOG_NS) {
        // skips the batches whose last sample is more than MAX_BACKLOG_NS late, no more
        const uint64_t skipped {(late_ns - MAX_BACKLOG_NS - 1U) / batch_ns + 1U};
        dropped += static_cast<uint32_t>(skipped);
        _next_ns += skipped * batch_ns;
        _sample += static_cast<uint32_t>(skipped * batch);
        if (_next_ns + last_sample_ns > now_ns) {
            return false;
        }
    }

    record = robby_link_Record_init_zero;
    record.which_kind = robby_link_Record_synthetic_tag;
    auto& out = record.kind.synthetic;
    out.t_first_ns = _next_ns;
    out.period_ns = static_cast<uint32_t>(period_ns);
    out.x_count = static_cast<pb_size_t>(batch);
    out.y_count = static_cast<pb_size_t>(batch);
    out.z_count = static_cast<pb_size_t>(batch);

    // like the accelerometer at rest: x and y near 0, z near 1 g (16393 LSB), so the
    // samples take as many bytes as the IMU's
    for (uint32_t i = 0; i < batch; ++i, ++_sample) {
        out.x[i] = static_cast<int16_t>(static_cast<int32_t>(_sample % 64U) - 160);
        out.y[i] = static_cast<int16_t>(120 - static_cast<int32_t>(_sample % 32U));
        out.z[i] = static_cast<int16_t>(16'360 + static_cast<int32_t>(_sample % 64U));
    }

    _next_ns += batch_ns;
    return true;
}
