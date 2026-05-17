#pragma once
#include <array>
#include <cstdint>

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

}  // namespace robby
