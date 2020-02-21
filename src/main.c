/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * 
 * west build -t guiconfig
 * 
 * debug UART1 A9 - TxD => RxD	orange
 * 			   A10 - RxD <= TxD	blue
 *            GND  	  <=> GND	white
 * 
 * I2C1 =    B6 SCL
 * 			 B7 SDA
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <lightmodbus.h>
#include <slave.h>
#include <parser.h>
#include <stdio.h>
#include <sys/printk.h>
#include <drivers/dma.h>

#include <console/console.h>
#include <sys/ring_buffer.h>
#include <logging/log.h>

#include <modbus.h>

LOG_MODULE_REGISTER(MAIN, LOG_LEVEL_DBG);

//static void dma_callback(void *callback_arg, u32_t channel, int error_code)
//{
//		gpio_pin_write(gpio, LED, 1);
//}


void main(void)       
{

//	u32_t cnt = 0;
	console_getline_init();

	modbus_init(27);

	// init_drivers();

    LOG_INF("UART3 DMA init_drivers()");
    
	while (1) {
		char *s = console_getline();
	    
		printk("line: %s\n", s);
		printk("last char was: 0x%x\n", s[strlen(s) - 1]);
		
	}
}
