#pragma once

#include <cstdint>
#include <initializer_list>

namespace rtos {

enum class Result : uint8_t {
	Ok,
	ErrorTimeout,
	ErrorBusy,
    ErrorWouldBlock,
	ErrorUnknown,
};

}
