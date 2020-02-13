# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.13.1)

# CMake file to compile this Modbus component as a west module in Zephyr
set(CONFIG_USE_MODBUS TRUE)

message("UMDOM: Запуск сборки.")

set(UMDOM_BASE $ENV{ZEPHYR_BASE}/../modules/umdom)

set(ZEPHYR_EXTRA_MODULES 
                        ${UMDOM_BASE}/modbus
                        ${UMDOM_BASE}/stm32_uart3_dma1_ch2_3)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)

project(blinky)

FILE(GLOB app_sources src/*.c)

target_sources(app PRIVATE ${app_sources})

