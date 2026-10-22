cmake_minimum_required(VERSION 3.13)

# Pico 2 (RP2350, Arm) 기본. 다른 보드면 -DPICO_BOARD=... 로 바꿉니다.
set(PICO_BOARD pico2 CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(pico2_servo_bridge C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(pico2_servo_bridge
    src/main.c
    src/w5500.c
    src/sts.c
    src/link.c
)
target_include_directories(pico2_servo_bridge PRIVATE src)
target_compile_options(pico2_servo_bridge PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(pico2_servo_bridge
    pico_stdlib
    pico_unique_id
    hardware_spi
    hardware_uart
    hardware_pwm
    hardware_watchdog
)

# 상태 출력은 USB 시리얼 (UART0 은 STS 버스가 씀)
pico_enable_stdio_usb(pico2_servo_bridge 1)
pico_enable_stdio_uart(pico2_servo_bridge 0)
pico_add_extra_outputs(pico2_servo_bridge)
