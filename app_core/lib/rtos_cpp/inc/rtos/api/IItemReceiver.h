#pragma once

namespace rtos {

template<typename T>
class IItemReceiver {
public:
    virtual ~IItemReceiver() = default;

    virtual bool receive(T& item) = 0;
    virtual bool tryReceive(T& item, uint32_t timeoutMs) = 0;

protected:
    IItemReceiver() = default;
};
    
}
