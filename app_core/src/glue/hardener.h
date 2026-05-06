#pragma once

#include <functional>

namespace glue {

template<typename... TArgs>
class Hardener {
public:
    void register_callback(std::function<void(TArgs...)> callback) {
        _callback = std::move(callback);
    }

    void call(TArgs... args) {
        if (_callback) {
            return _callback(args...);
        }
    }

private:
    std::function<void(TArgs...)> _callback {};
};

}