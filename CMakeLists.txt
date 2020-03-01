# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.13.1)

message("UMDOM: Запуск сборки.")

set(ZEPHYR_EXTRA_MODULES 
                        $ENV{ZEPHYR_UMDOM}/modbus
                        $ENV{ZEPHYR_UMDOM}/stm32_uart3_dma1_ch2_3
                        $ENV{ZEPHYR_UMDOM}/am2320
                        $ENV{ZEPHYR_UMDOM}/bh1750
                        $ENV{ZEPHYR_UMDOM}/RTC)
# добавляем вручную dtr для BME280      
# TODO: изучить dte overlay и создать для проекта
add_definitions(-DDT_BOSCH_BME280_BUS_I2C 
                -DDT_INST_0_BOSCH_BME280_BUS_NAME="I2C_1"
                -DDT_INST_0_BOSCH_BME280_BASE_ADDRESS=0x77
                -DDT_INST_0_BOSCH_BME280_LABEL="BME280")                        

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)

project(blinky)

include_directories(modules/include)

FILE(GLOB app_sources 
    src/*.c 
    modules/src/*.c)

target_sources(app PRIVATE ${app_sources})

