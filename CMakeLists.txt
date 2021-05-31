# SPDX-License-Identifier: Apache-2.0

# For the sake of demonstration, we add the driver directory as a zephyr module
# by hand. If your driver is a project that's managed by west, you can remove this line.
list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/bbq10kbd_module
  )

cmake_minimum_required(VERSION 3.13.1)

set(OPENOCD_NRF5_SUBFAMILY "nrf52")

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND DTS_ROOT ${CMAKE_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})