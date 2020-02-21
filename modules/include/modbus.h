#pragma once
#ifdef CONFIG_DEV_MODBUS

#include <zephyr.h>
#include <device.h>
#include <stdint.h>
#include <lightmodbus.h>
#include <slave.h>

extern ModbusError modbus_init( uint8_t adr, 
                                ModbusRegisterCallbackFunction func,
                   	            uint16_t registerCount,
	                            uint16_t inputRegisterCount,
	                            uint16_t coilCount,
	                            uint16_t discreteInputCount);
#endif
