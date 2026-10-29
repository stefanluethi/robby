# The protobuf link to the ESP32 outpost: nanopb, the C code generated from
# proto/robby_link.proto, and the parts of the trace path that build without the HAL.
# Included by the firmware (CMakeLists.txt) and by the host tests (test/CMakeLists.txt).
#
# Defines:
#   robby_link        nanopb runtime + generated robby_link.pb.c (target robby_link)
#   nanocobs          COBS
#   robby_link_core   the portable encoder/framing code of src/ (target robby_link_core)

include_guard(GLOBAL)
include(FetchContent)

set(ROBBY_APP_CORE_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
set(ROBBY_PROTO_DIR ${ROBBY_APP_CORE_DIR}/../proto)

## nanopb ######################################################################
# Only the sources: nanopb's own CMakeLists builds a generator package we don't need.

FetchContent_Declare(
        nanopb
        GIT_REPOSITORY https://github.com/nanopb/nanopb.git
        GIT_TAG        0.4.9.1
        GIT_SHALLOW    TRUE
        SOURCE_SUBDIR  do-not-add-subdirectory
)
FetchContent_MakeAvailable(nanopb)

## Code generation #############################################################
# nanopb_generator.py of the fetched release, it needs protoc and the protobuf Python
# package. Without them, uvx runs the generator of the same release from PyPI.

find_package(Python3 COMPONENTS Interpreter)
find_program(PROTOC_EXECUTABLE protoc)
find_program(UVX_EXECUTABLE uvx)

set(ROBBY_NANOPB_GENERATOR "" CACHE STRING "Command running nanopb_generator (empty: detect)")
if (NOT ROBBY_NANOPB_GENERATOR)
    set(_python_ok FALSE)
    if (Python3_FOUND AND PROTOC_EXECUTABLE)
        execute_process(
                COMMAND ${Python3_EXECUTABLE} -c "import google.protobuf"
                RESULT_VARIABLE _python_result
                OUTPUT_QUIET ERROR_QUIET
        )
        if (_python_result EQUAL 0)
            set(_python_ok TRUE)
        endif ()
    endif ()

    if (_python_ok)
        set(ROBBY_NANOPB_GENERATOR ${Python3_EXECUTABLE} ${nanopb_SOURCE_DIR}/generator/nanopb_generator.py)
    elseif (UVX_EXECUTABLE)
        set(ROBBY_NANOPB_GENERATOR ${UVX_EXECUTABLE} --from nanopb==0.4.9.1 nanopb_generator)
    else ()
        message(FATAL_ERROR "nanopb_generator needs protoc and python3 with protobuf, or uvx")
    endif ()
endif ()
message(STATUS "nanopb generator: ${ROBBY_NANOPB_GENERATOR}")

set(ROBBY_LINK_GEN_DIR ${CMAKE_BINARY_DIR}/robby_link)
add_custom_command(
        OUTPUT ${ROBBY_LINK_GEN_DIR}/robby_link.pb.c ${ROBBY_LINK_GEN_DIR}/robby_link.pb.h
        COMMAND ${CMAKE_COMMAND} -E make_directory ${ROBBY_LINK_GEN_DIR}
        # the blip annotations generate no code: -x keeps blip/options.pb.h out of the #includes
        COMMAND ${ROBBY_NANOPB_GENERATOR} -q -I ${ROBBY_PROTO_DIR} -D ${ROBBY_LINK_GEN_DIR}
                -x blip/options.proto ${ROBBY_PROTO_DIR}/robby_link.proto
        DEPENDS ${ROBBY_PROTO_DIR}/robby_link.proto ${ROBBY_PROTO_DIR}/robby_link.options
                ${ROBBY_PROTO_DIR}/blip/options.proto
        COMMENT "Generating robby_link.pb.c from robby_link.proto"
        VERBATIM
)

## Libraries ###################################################################

add_library(
        robby_link
        STATIC
        ${ROBBY_LINK_GEN_DIR}/robby_link.pb.c
        ${nanopb_SOURCE_DIR}/pb_common.c
        ${nanopb_SOURCE_DIR}/pb_encode.c
        ${nanopb_SOURCE_DIR}/pb_decode.c
)
target_include_directories(robby_link SYSTEM PUBLIC ${ROBBY_LINK_GEN_DIR} ${nanopb_SOURCE_DIR})
# Same options on the target and in the host tests
target_compile_definitions(robby_link PUBLIC PB_NO_ERRMSG=1)

## nanocobs ####################################################################

FetchContent_Declare(
        nanocobs
        GIT_REPOSITORY https://github.com/charlesnicholson/nanocobs.git
        GIT_TAG        v0.2.0
)
FetchContent_MakeAvailable(nanocobs)

add_library(nanocobs STATIC ${nanocobs_SOURCE_DIR}/cobs.c)
target_include_directories(nanocobs PUBLIC ${nanocobs_SOURCE_DIR})

# The trace path, the speed loop and the odometry of the wheels without the HAL and FreeRTOS
add_library(
        robby_link_core
        STATIC
        ${ROBBY_APP_CORE_DIR}/src/trace/crc16.cpp
        ${ROBBY_APP_CORE_DIR}/src/trace/frame.cpp
        ${ROBBY_APP_CORE_DIR}/src/trace/uplink_writer.cpp
        ${ROBBY_APP_CORE_DIR}/src/trace/downlink_reader.cpp
        ${ROBBY_APP_CORE_DIR}/src/trace/records.cpp
        ${ROBBY_APP_CORE_DIR}/src/service/bench_load.cpp
        ${ROBBY_APP_CORE_DIR}/src/service/drive_control.cpp
        ${ROBBY_APP_CORE_DIR}/src/service/imu_odometry.cpp
        ${ROBBY_APP_CORE_DIR}/src/service/odometry.cpp
)
target_include_directories(robby_link_core PUBLIC ${ROBBY_APP_CORE_DIR}/src)
target_link_libraries(robby_link_core PUBLIC robby_link nanocobs)
# also in the host build, where the tests use exceptions
target_compile_options(robby_link_core PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti>)
