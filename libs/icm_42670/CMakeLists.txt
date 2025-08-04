cmake_minimum_required(VERSION 3.13)

project(icm_42670)

add_library(icm_42670 icm_42670.c)

target_include_directories(icm_42670 PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(icm_42670 PUBLIC
    pico_stdlib
    hardware_spi
    hardware_adc
    hardware_timer
)