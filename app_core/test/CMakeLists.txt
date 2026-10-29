cmake_minimum_required(VERSION 3.22)

# Host unit tests of the trace path: nanopb encoding, framing, the downlink, the bench
# load and the speed loop of the wheels, checked against the golden vectors of ../../proto/vectors.
#
#   cmake -S app_core/test -B app_core/build/test && cmake --build app_core/build/test
#   ctest --test-dir app_core/build/test --output-on-failure

project(app_core_test C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

include(FetchContent)
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/robby_link.cmake)

FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.8.1
        GIT_SHALLOW    TRUE
)
FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG        v3.11.3
        GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(Catch2 json)

add_executable(
        app_core_tests
        test_crc16.cpp
        test_frame.cpp
        test_uplink_writer.cpp
        test_downlink_reader.cpp
        test_records.cpp
        test_bench_load.cpp
        test_drive_control.cpp
        support.cpp
)
target_compile_options(app_core_tests PRIVATE -Wall -Wextra)
target_compile_definitions(app_core_tests PRIVATE ROBBY_VECTORS_DIR="${ROBBY_PROTO_DIR}/vectors")
target_link_libraries(app_core_tests PRIVATE robby_link_core Catch2::Catch2WithMain nlohmann_json::nlohmann_json)

include(CTest)
include(Catch)
catch_discover_tests(app_core_tests)
