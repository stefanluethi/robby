#pragma once

#include <cobs.h>

namespace util {

template <class T>
class CobsEnc : se_oss::IWriter
{
public:
    template<typename... Args>
    explicit CobsEnc(Args&&... args) : _sink {std::forward<Args>(args)...}
    {
    }

    ~CobsEnc() override = default;

    std::size_t read(void* data, std::size_t maxLength)
    {
        return _sink.read(data, maxLength);
    }

    // IWriter implementation
    void write(const void* data, std::size_t length) override
    {
        if (length == 0) {
            return;
        }
        std::size_t cobs_length {0U};
        auto encoder_result = cobs_encode(data, length, _cobs_buffer.data(), _cobs_buffer.size(), &cobs_length);
        if (encoder_result == COBS_RET_SUCCESS) {
            _sink.write(_cobs_buffer.data(), cobs_length);
        }
    }

    void flush() override { }

private:
    T _sink;
    std::array<uint8_t, COBS_ENCODE_MAX(2048)> _cobs_buffer {};
};

}  // namespace util
