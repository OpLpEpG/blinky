# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.13.1)

message("UMDOM: Запуск сборки.")

set(ZEPHYR_EXTRA_MODULES 
                        $ENV{ZEPHYR_UMDOM}/modbus
                        $ENV{ZEPHYR_UMDOM}/stm32_uart3_dma1_ch2_3)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)

project(blinky)

FILE(GLOB app_sources src/*.c)

target_sources(app PRIVATE ${app_sources})

