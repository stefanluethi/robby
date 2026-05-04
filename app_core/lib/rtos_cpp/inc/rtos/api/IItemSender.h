#pragma once

namespace rtos {

template<typename T>
class IItemSender {
public:
    virtual ~IItemSender() = default;

    virtual bool send(const T& item) = 0;
    virtual bool trySend(const T& item, uint32_t timeoutMs = 0U) = 0;

protected:
    IItemSender() = default;
};
    
}
