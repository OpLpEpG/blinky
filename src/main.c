/*
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
#include <lightmodbus.h>
#include <slave.h>
#include <console/console.h>
#include <logging/log.h>
#include <modbus.h>
LOG_MODULE_REGISTER(MAIN, LOG_LEVEL_DBG);

#define CONST_BEGIN 0x08000C00

typedef	struct __attribute__( ( __packed__ ) )
	{
		uint8_t address;
		uint8_t function;
		uint32_t magic_code;
		uint16_t crc;
	} umdom_const_t;	
	
const umdom_const_t* const umdom_const = (const umdom_const_t*) CONST_BEGIN;

uint16_t regs[16], iregs[16]; //Holding registers and input registers arrays
uint8_t writeacc[16]; //Some write locks

static uint16_t callbackFunction(ModbusRegisterQuery query, ModbusDataType datatype, uint16_t index, uint16_t value, void *ctx )
{
    //All can be read
    if ( query == MODBUS_REGQ_R_CHECK ) return 1;
    //writeacc determines if holding register can be written
    if ( query == MODBUS_REGQ_W_CHECK ) return writeacc[index];
    //Read
    if ( query == MODBUS_REGQ_R )
    {
        if ( datatype == MODBUS_HOLDING_REGISTER ) return regs[index];
        if ( datatype == MODBUS_INPUT_REGISTER ) return iregs[index];
    }
    //Write
    if ( query == MODBUS_REGQ_W && datatype == MODBUS_HOLDING_REGISTER )
        iregs[index] = value;
    return 0;
}

void main(void)       
{
	console_getline_init();
	modbus_init(umdom_const->address, callbackFunction,5,0,5,0);

    LOG_INF("UART3 DMA init_drivers()");    
	while (1) {
		char *s = console_getline();
	    
		printk("line: %s\n", s);
		printk("last char was: 0x%x\n", s[strlen(s) - 1]);
		
	}
}
