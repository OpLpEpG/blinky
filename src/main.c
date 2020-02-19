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

#include <kernel.h>
#include <power/reboot.h>
#include <drivers/timer/system_timer.h>

#include <console/console.h>
#include <sys/ring_buffer.h>
#include "usart3_dma1_ch2_3.h"
#include <logging/log.h>

extern void sys_clock_disable(void);

LOG_MODULE_REGISTER(MAIN, LOG_LEVEL_DBG);

#define svc(code) __asm__ volatile ("svc %[immediate]"::[immediate] "I" (code))

#define LED_PORT	DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED		DT_ALIAS_LED0_GPIOS_PIN

/* 1000 msec = 1 sec */
#define SLEEP_TIME	1000


//Registers and coils
uint8_t coils[2] = { 0 };
uint16_t regs[32] = { 0 };

//For storing exit codes
uint8_t sec, mec;

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
		uart->init(false, NULL);		

		(void)irq_lock();
		sys_clock_disable();
   		__set_MSP(*(uint32_t*) FLASH_BASE);		   
   		SCB->VTOR = FLASH_BASE; /* Vector Table Relocation in Internal FLASH. */
   		svc(0);  // go to boot svc call
	}
	return MODBUS_ERROR_PARSE; 
}

static ModbusError in_program( struct modbusSlave *status, ModbusParser *parser )
{
    LOG_INF("in_program %x", parser->base.address);
	memcpy(status->response.frame, status->request.frame, status->request.length);
	status->response.length = status->request.length;
	return  MODBUS_OK;
}

static ModbusSlaveUserFunction msuf[2] =
{
	{100, goto_boot_loader},
	{102, in_program}
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

	gpio_pin_configure(gpio, LED, GPIO_OUTPUT);

	gpio_pin_set(gpio, LED, 0);

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
	slave.userFunctionCount = 2; 	
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
		regs[0]++;
		// TODO: any works in modbus callbucks
		ModbusError err = modbusParseRequest(&slave);
	    LOG_INF("modbusParseRequest: %d", err);
		LOG_HEXDUMP_DBG(slave.response.frame, slave.response.length, "slave.response.frame");
		// send modbusRequest
  		gpio_pin_set(gpio, LED, 1);
		uart3_dma_error_t r = uart->writeBuffer(slave.response.frame, slave.response.length, 1000);	
  		gpio_pin_set(gpio, LED, 0);
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

