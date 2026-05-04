add_library(platform_lm3s6965 STATIC)

target_sources(
        platform_lm3s6965
        PRIVATE
        main.c
        startup.c
)

target_link_options(
        platform_lm3s6965
        PUBLIC -T ${CMAKE_CURRENT_LIST_DIR}/standalone.ld
)

add_library(platform::lm3s6965 ALIAS platform_lm3s6965)
