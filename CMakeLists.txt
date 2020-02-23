# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.13.1)

message("UMDOM: Запуск сборки.")

set(ZEPHYR_EXTRA_MODULES 
                        $ENV{ZEPHYR_UMDOM}/modbus
                        $ENV{ZEPHYR_UMDOM}/stm32_uart3_dma1_ch2_3
                        $ENV{ZEPHYR_UMDOM}/am2320)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)

project(blinky)

include_directories(modules/include)

FILE(GLOB app_sources 
    src/*.c 
    modules/src/*.c)

target_sources(app PRIVATE ${app_sources})

