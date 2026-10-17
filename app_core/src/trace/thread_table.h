#pragma once
#include <array>
#include <cstdint>

#include <sonde/sonde.hpp>

namespace robby {

struct ThreadInfo
{
    char name[32];
    uint8_t priority;
    float stack_usage;
    uint32_t stack_size;
    float runtime;
};

struct ThreadTable
{
    std::array<ThreadInfo, 16> threads;
    std::size_t size;
};

inline void encode(sonde::Encoder& encoder, const ThreadInfo& thread)
{
    encoder.record(4, [&](sonde::Record& record) {
        record.field("priority", thread.priority);
        record.field("runtime", thread.runtime);
        record.field("stack_usage", thread.stack_usage);
        record.field("stack_size", thread.stack_size);
    });
}

}  // namespace robby
