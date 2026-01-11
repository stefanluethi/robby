add_subdirectory(vl53l8cx)

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