cmake_minimum_required(VERSION 3.28)
# project(rtos_cpp)

# enable_language(C CXX ASM)
# set(CMAKE_CXX_STANDARD 14)

add_library(rtos_cpp INTERFACE)
target_sources(rtos_cpp INTERFACE
        src/rtos.cpp
        src/Mutex.cpp
        src/Semaphore.cpp
        src/Task.cpp
        src/Timer.cpp
        src/util/CoreArm.cpp
        # src/util/CoreStub.cpp
        src/util/New.cpp
)

target_include_directories(
        rtos_cpp
        INTERFACE
        inc
        # let's not about this...
        # ${CMAKE_SOURCE_DIR}/Core/Inc
        # ${CMAKE_SOURCE_DIR}/Drivers/STM32F7xx_HAL_Driver/Inc
        # ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F7xx/Include
)

target_link_libraries(
        rtos_cpp
        INTERFACE
        freertos_kernel
)

# add_subdirectory(test)
