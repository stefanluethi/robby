#pragma once

#include <cassert>
#include <initializer_list>

namespace rtos {

template<typename T>
class SharedPin {
public:
    SharedPin(T* item) : _item{item} { assert(_item); }
    ~SharedPin() {}
    SharedPin() = delete;
    SharedPin(const SharedPin&) = default;
    SharedPin(SharedPin&&) = default;
    SharedPin& operator=(const SharedPin&) = default;
    SharedPin&& operator=(SharedPin&&) = delete;

    T* operator->() { return _item; }
    T& operator*() { return *_item; }


private:
    T* _item;
};

template<typename T, typename... TArgs>
SharedPin<T> pinToHeap(TArgs... args)
{
    return SharedPin<T,TArgs...>(new T(args...));
}

}
