# sources downloaded from https://www.st.com/en/embedded-software/stsw-img040.html

add_library(st_tof_vl53l8cx STATIC)
target_sources(
    st_tof_vl53l8cx
    PRIVATE
    VL53L8CX_ULD_API/src/vl53l8cx_api.c
    VL53L8CX_ULD_API/src/vl53l8cx_plugin_detection_thresholds.c
    VL53L8CX_ULD_API/src/vl53l8cx_plugin_motion_indicator.c
    VL53L8CX_ULD_API/src/vl53l8cx_plugin_xtalk.c
)

target_include_directories(
    st_tof_vl53l8cx
    PUBLIC
    VL53L8CX_ULD_API/inc
    platform
)

add_library(st::tof::vl53l8cx ALIAS st_tof_vl53l8cx)
