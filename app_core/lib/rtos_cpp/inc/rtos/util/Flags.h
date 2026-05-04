#pragma once

#include <cstdint>
#include <initializer_list>
#include <type_traits>

namespace rtos {

template<typename T>
class Flags {
public:
    Flags(uint32_t raw) : _raw{raw} { }
    Flags(T flag) : _raw(1U << static_cast<uint32_t>(flag)) { }
    Flags(std::initializer_list<T> flags)
    {
        for (const auto flag : flags) {
            _raw |= 1U << static_cast<uint32_t>(flag);
        }
    }

    ~Flags() { }
    Flags(const Flags&) = default;
    Flags(Flags&&) = default;

    void set(T flag) { _raw |= 1U << static_cast<uint32_t>(flag); }
    void clear(T flag) { _raw &= ~(1U << static_cast<uint32_t>(flag)); }
    constexpr bool isSet(T flag) const { return _raw & (1U << static_cast<uint32_t>(flag)); }
    constexpr uint32_t raw() const { return _raw; }
private:
    static_assert(std::is_enum<T>::value == true, "Flag type must be an enumeration");

    uint32_t _raw {0U};
};
    
}
