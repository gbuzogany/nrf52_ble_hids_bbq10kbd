# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BBQ10KBD_DRIVER)
  # Add bbq10kbd_api.h to the set of global include paths.
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    bbq10kbd.c
    )
endif()
