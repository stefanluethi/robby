
add_compile_options(-ffunction-sections -fdata-sections  -fno-common -O0)

add_subdirectory(platform/lm3s6965)

add_executable(rtos_integration_test
        TestRunner.cpp
        SemaphoreTest.cpp
        FreeRtosHooks.cpp
)

target_include_directories(
        rtos_integration_test
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(rtos_integration_test PRIVATE gcc)

target_link_libraries(
        rtos_integration_test
        PUBLIC
        platform::lm3s6965
        rtos
        unity::framework
)

target_link_options(
        rtos_integration_test
        PRIVATE -Wl,-gc-sections,--print-memory-usage
)

# longjump did not work with RTOS
target_compile_definitions(
        unity
        PRIVATE UNITY_EXCLUDE_SETJMP_H
)
