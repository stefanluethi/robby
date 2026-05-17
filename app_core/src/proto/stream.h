#pragma once
#include <cstdint>

namespace robby {

struct StreamDescriptor
{
    uint16_t id {UINT16_MAX};
    const char* name {""};
    float scale_factor {1.0F};
    const char* unit {"u"};
    float sampling_time {1.0F};
};

}  // namespace robby
