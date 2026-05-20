include(FetchContent)

add_subdirectory(vl53l8cx)
add_subdirectory(rtos_cpp)

## STM32CubeF7 #################################################################
# STM32CubeF7 Extract for STM32F723E-DISCO board
# https://github.com/STMicroelectronics/STM32CubeF7
# commit: 4d308aa

add_library(
    stm32cube7_utils
    STATIC
    STM32CubeF7/Utilities/Fonts/font8.c
    STM32CubeF7/Utilities/Fonts/font12.c
    STM32CubeF7/Utilities/Fonts/font16.c
    STM32CubeF7/Utilities/Fonts/font20.c
    STM32CubeF7/Utilities/Fonts/font24.c
)

target_include_directories(
    stm32cube7_utils
    PUBLIC
    STM32CubeF7/Utilities
    STM32CubeF7/Utilities/Fonts
)

add_library(
    stm32cubef7
    INTERFACE
)

target_sources(
    stm32cubef7
    INTERFACE
    STM32CubeF7/Drivers/BSP/Components/st7789h2/st7789h2.c
    STM32CubeF7/Drivers/BSP/STM32F723E-Discovery/stm32f723e_discovery_audio.c
    STM32CubeF7/Drivers/BSP/STM32F723E-Discovery/stm32f723e_discovery_lcd.c
    STM32CubeF7/Drivers/BSP/STM32F723E-Discovery/stm32f723e_discovery_psram.c
    STM32CubeF7/Drivers/BSP/STM32F723E-Discovery/stm32f723e_discovery_qspi.c
    STM32CubeF7/Drivers/BSP/STM32F723E-Discovery/stm32f723e_discovery_ts.c
    STM32CubeF7/Drivers/BSP/STM32F723E-Discovery/stm32f723e_discovery.c
    # STM32CubeF7/Utilities/Log/lcd_log.c
)

target_include_directories(
    stm32cubef7
    INTERFACE
    STM32CubeF7/Drivers/BSP/Components
    STM32CubeF7/Drivers/BSP/STM32F723E-Discovery
    STM32CubeF7/Utilities/Log
)

target_link_libraries(stm32cubef7 INTERFACE stm32cube7_utils)

add_library(st::cubef7 ALIAS stm32cubef7)

## tinycbor ####################################################################

set(WITH_FLOATING_POINT OFF CACHE BOOL "" FORCE)
set(WITH_CBOR2JSON OFF CACHE BOOL "" FORCE)
set(WITH_TOOLS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
        tinycbor
        GIT_REPOSITORY https://github.com/intel/tinycbor.git
        GIT_TAG v7.0
)
FetchContent_MakeAvailable(tinycbor)

## se-oss ######################################################################

add_library(se_oss_log_conf INTERFACE)
target_include_directories(se_oss_log_conf INTERFACE ${CMAKE_SOURCE_DIR}/src/conf/se-oss)
target_link_libraries(se_oss_log_conf INTERFACE tinycbor)

set(SE_OSS_COMPONENT_LOG ON CACHE BOOL "" FORCE)
# set(SE_OSS_LOG_SUPPORT_CBOR ON CACHE BOOL "" FORCE)
FetchContent_Declare(
        se-oss
        GIT_REPOSITORY https://github.com/sourceengineers/se-oss.git
        GIT_TAG        0bd2fb79846667c9f509ff960949095d9dac2869
)
FetchContent_MakeAvailable(se-oss)

## FreeRTOS ####################################################################
add_library(freertos_config INTERFACE)

target_include_directories(freertos_config SYSTEM
        INTERFACE
        ${CMAKE_SOURCE_DIR}/src/conf/freertos
        systemview/Config
        systemview/FreeRTOSV11
        systemview/SEGGER
)

target_compile_definitions(freertos_config
        INTERFACE
        projCOVERAGE_TEST=0
)

# target_compile_definitions(freertos_config INTERFACE ${definitions})
target_compile_options(freertos_config INTERFACE $<$<COMPILE_LANGUAGE:CXX>:-fpermissive>)

set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)
set(FREERTOS_PORT "GCC_ARM_CM7" CACHE STRING "" FORCE)

FetchContent_Declare(freertos_kernel
        GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
        GIT_TAG        V11.0.1
)

FetchContent_MakeAvailable(freertos_kernel)

## SystemView ##################################################################
add_library(segger_systemview STATIC)

target_sources(
        segger_systemview
        PRIVATE
        systemview/FreeRTOSV11/SEGGER_SYSVIEW_FreeRTOS.c
        systemview/FreeRTOSV11/Config/Cortex-M/SEGGER_SYSVIEW_Config_FreeRTOS.c
        systemview/SEGGER/SEGGER_RTT.c
        systemview/SEGGER/SEGGER_RTT_ASM_ARMv7M.S
        systemview/SEGGER/SEGGER_RTT_printf.c
        systemview/SEGGER/SEGGER_SYSVIEW.c
        systemview/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.c
)

target_include_directories(
        segger_systemview
        PUBLIC
        systemview/SEGGER
        systemview/FreeRTOSV10
        PRIVATE
        systemview/Config
)

target_link_libraries(
        segger_systemview
        PRIVATE
        freertos_kernel
)

add_library(segger::systemview ALIAS segger_systemview)

## nanocobs ####################################################################

FetchContent_Declare(
        nanocobs
        GIT_REPOSITORY https://github.com/charlesnicholson/nanocobs.git
        GIT_TAG        v0.2.0
)
FetchContent_MakeAvailable(nanocobs)

add_library(nanocobs STATIC ${nanocobs_SOURCE_DIR}/cobs.c)
target_include_directories(nanocobs PUBLIC ${nanocobs_SOURCE_DIR})
