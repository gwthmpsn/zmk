# Copyright (c) 2020-2021 The ZMK Contributors
# SPDX-License-Identifier: MIT

add_subdirectory_ifdef(CONFIG_ZMK_BATTERY battery)
add_subdirectory_ifdef(CONFIG_EC11 ec11)
add_subdirectory_ifdef(CONFIG_PINNACLE cirque_trackpad)
add_subdirectory_ifdef(CONFIG_PMW33XX pmw33xx)
# new impl
add_subdirectory_ifdef(CONFIG_PMW3360 pixart/pmw3360)
add_subdirectory_ifdef(CONFIG_PMW3389 pixart/pmw3389)
