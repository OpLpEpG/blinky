/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * debug UART B6 - TxD => RxD	orange
 * 			  B7 - RxD <= TxD	blue
 *            GND  	  <=> GND	white
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
#include "usart3_dma1_ch2_3.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(MAIN, LOG_LEVEL_DBG);



#define LED_PORT	DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED		DT_ALIAS_LED0_GPIOS_PIN

/* 1000 msec = 1 sec */
#define SLEEP_TIME	1000


//Registers and coils
uint8_t coils[2] = { 0 };
uint16_t regs[32] = { 0 };

//For storing exit codes
uint8_t sec, mec;

//static void uart_irq_callback(struct device *port)
//{
//
//}

struct device *gpio;
uart3_dma_api_t const *uart;
struct k_sem rxReady;
static ModbusSlave slave;

typedef	struct __attribute__( ( __packed__ ) )
	{
		uint8_t address;
		uint8_t function;
		uint32_t magic_code;
		uint16_t crc;
	} goto_boot_t;

static ModbusError goto_boot_loader( struct modbusSlave *status, ModbusParser *parser )
{
	goto_boot_t *d = (goto_boot_t*) parser->frame;

	if (d->magic_code == 0x12345678)
	{
	    LOG_INF("goto_boot_loader");
	}
	return MODBUS_ERROR_PARSE; 
}

static ModbusSlaveUserFunction msuf[1] =
{
	{100, goto_boot_loader}
};

static void uart3_dma_fkt_rx(u8_t *pD, size_t len)
{
	memcpy(slave.request.frame, pD, len);
	slave.request.length = len;

	LOG_HEXDUMP_DBG(slave.request.frame, len, "slave.request.frame");

	k_sem_give(&rxReady);

	uart->readBuffer();
}

static void init_drivers(void){
	/* Set LED pin as output */	
	gpio = device_get_binding(LED_PORT);	
	gpio_pin_configure(gpio, LED, GPIO_DIR_OUT);
	gpio_pin_write(gpio, LED, 0);

    // uart3 dma1 tx ch2 rx ch3
    struct device *uartDM;
	uartDM = device_get_binding("UART3_DMA");
	__ASSERT(uartDM, "invalid uart dev 3");
	uart = uartDM->driver_api;
    k_sem_init(&rxReady, 0, 1);
	uart->init(true, &uart3_dma_fkt_rx);
	
	//Init slave (input registers and discrete inputs work just the same)
	slave.address = 27;
	slave.registers = regs;
	slave.registerCount = 32;
	slave.coils = coils;
	slave.coilCount = 16;
	slave.userFunctions = msuf;
	slave.userFunctionCount = 1; 	
	modbusSlaveInit( &slave );
}

//static void dma_callback(void *callback_arg, u32_t channel, int error_code)
//{
//		gpio_pin_write(gpio, LED, 1);
//}

static void resetUart(void)
{
	uart->init(false, &uart3_dma_fkt_rx);
	uart->init(true, &uart3_dma_fkt_rx);
}

void modbus(void) 
{
	
	while (1)
	{
		if (k_sem_take(&rxReady, K_FOREVER)) {
			resetUart();
		    LOG_ERR("k_sem_take ERROR");
		}
		ModbusError err = modbusParseRequest(&slave);
	    LOG_INF("modbusParseRequest: %d", err);
		LOG_HEXDUMP_DBG(slave.response.frame, slave.response.length, "slave.response.frame");

		regs[0]++;

  		gpio_pin_write(gpio, LED, 1);
		uart3_dma_error_t r = uart->writeBuffer(slave.response.frame, slave.response.length, 1000);	
		k_sleep(5);	
  		gpio_pin_write(gpio, LED, 0);
		if(r){
			resetUart();
	    	LOG_ERR("ModbusError %d", r);
		}  
	}	

}


void main(void)       
{

//	u32_t cnt = 0;
	console_getline_init();

	init_drivers();

    LOG_INF("UART3 DMA init_drivers()");
    
	while (1) {
		char *s = console_getline();
	    
		printk("line: %s\n", s);
		printk("last char was: 0x%x\n", s[strlen(s) - 1]);
		
	}
}
K_THREAD_DEFINE(modbus_th, 1024, modbus, NULL, NULL, NULL, 7, K_ESSENTIAL, K_NO_WAIT);

