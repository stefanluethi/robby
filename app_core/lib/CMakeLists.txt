include(FetchContent)

FetchContent_Declare(
    st-tof1
    GIT_REPOSITORY https://github.com/STMicroelectronics/x-cube-tof1
    GIT_TAG        v3.4.2
)

FetchContent_MakeAvailable(st-tof1)

add_library(st_tof_vl53l8cx STATIC)
target_sources(
    st_tof_vl53l8cx
    PRIVATE
    ${st-tof1_SOURCE_DIR}/Drivers/BSP/Components/vl53l8cx/vl53l8cx.c
    ${st-tof1_SOURCE_DIR}/Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_api.c
    ${st-tof1_SOURCE_DIR}/Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_detection_thresholds.c
    ${st-tof1_SOURCE_DIR}/Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_motion_indicator.c
    ${st-tof1_SOURCE_DIR}/Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_xtalk.c
    ${st-tof1_SOURCE_DIR}/Drivers/BSP/Components/vl53l8cx/porting/platform.c
)

target_include_directories(
    st_tof_vl53l8cx
    PUBLIC
    ${st-tof1_SOURCE_DIR}/Drivers/BSP/Components/Common
    ${st-tof1_SOURCE_DIR}/Drivers/BSP/Components/vl53l8cx
    ${st-tof1_SOURCE_DIR}/Drivers/BSP/Components/vl53l8cx/modules
    ${st-tof1_SOURCE_DIR}/Drivers/BSP/Components/vl53l8cx/porting
)

add_library(st::tof::vl53l8cx ALIAS st_tof_vl53l8cx)
