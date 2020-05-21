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
#include <drivers/sensor.h>

#include <drivers/i2c.h>
#include <stm32f1xx_ll_i2c.h>

// #include <am2320.h>


#include <lightmodbus.h>
#include <slave.h>
#include <console/console.h>
#include <string.h>
#include <modbus.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(MAIN, LOG_LEVEL_DBG);

#define SENSOR_RUM_1 3

#define CONST_BEGIN 0x08000C00
#define CONST_LAST  0x08000FFF
// readonly const registers
typedef	struct __attribute__( ( __packed__ ) )
	{
		u16_t address;
		u16_t sensor_rum1_len;
		uint32_t magic_code;
		uint16_t crc;
	} umdom_const_t;	
const umdom_const_t* const umdom_const = (const umdom_const_t*) CONST_BEGIN;

struct sensor_mb2 {
	/** Integer part of the value. +-65535 */
	s16_t val1;
	/** Fractional part of the value (in 1/1000 parts). */
	s16_t val2;
} __attribute__( ( __packed__ ) );
struct sensor_mb1 {
	/** Integer part of the value.+-127 */
	s8_t val1;
	/** Fractional part of the value (in 1/100 parts). */
	s8_t val2;
} __attribute__( ( __packed__ ) );


// readonly registers
static struct __attribute__( ( __packed__ ) )
{
    s64_t time;
    struct sensor_value sensor_rum1[SENSOR_RUM_1]; 
} mb_input_r;
typedef	struct __attribute__( ( __packed__ ) )
{
	u16_t address;
	u16_t function;
	uint32_t magic_code;
	uint16_t crc;
} mb_virtual_input_regs_t;	

// rw registers
static struct __attribute__( ( __packed__ ) )
{
    s64_t timebegin;
} main_setup;

enum{
    MB_CONST = 0xC00,
    MB_CONST_LAST = MB_CONST + sizeof(umdom_const_t)-2,
    MB_INP_REG = 0x000,
    MB_INP_REG_END = 0x000 + sizeof(mb_input_r)-2,
    MB_HOLD_REG = 0x000,
    MB_HOLD_REG_LAST = 0x000 + sizeof(main_setup)-2,
};


//k_uptime_delta	

uint16_t regs[16], iregs[16]; //Holding registers and input registers arrays
uint8_t writeacc[16]; //Some write locks

static uint16_t callbackFunction(ModbusRegisterQuery query, ModbusDataType datatype, uint16_t index, uint16_t value, void *ctx )
{
    //All can be read
    if ( query == MODBUS_REGQ_R_CHECK ) return 1;

    //writeacc determines if holding register can be written
    else if ( query == MODBUS_REGQ_W_CHECK ) 
    //Read
    {
        //return !writeacc[index];

        switch (datatype)
        {
        case MODBUS_HOLDING_REGISTER:
            /* code */
            break;
        case MODBUS_INPUT_REGISTER:
            /* code */
            break;
        case MODBUS_COIL:
            /* code */
            break;
        case MODBUS_DISCRETE_INPUT:
            /* code */
            break;
        
        default:
            break;
        }
        return 1;
    }
    else if ( query == MODBUS_REGQ_R )
    {
        if ( datatype == MODBUS_HOLDING_REGISTER ) return regs[index];
        if ( datatype == MODBUS_INPUT_REGISTER ) return iregs[index];
    }
    //Write
    if ( query == MODBUS_REGQ_W && datatype == MODBUS_HOLDING_REGISTER )
        regs[index] = value;
    return 0;
}

static u8_t flag_AM2320, flag_BME280, flag_BH1750;

void main(void)       
{

	console_getline_init();
	modbus_init(umdom_const->address & 0x7F, callbackFunction,sizeof(regs),sizeof(iregs),5,0);

    LOG_INF("BEGIN CONSOLE =>");    
	while (1) {
		char *s = console_getline();
        if (strcmp(s, "GA") == 0) printk("[%s] Gett Address Device: %d\n", s, umdom_const->address & 0x7F);
        else if (strcmp(s, "R") == 0) LOG_HEXDUMP_DBG(regs, sizeof(regs), "[R] regs");
        else if (strcmp(s, "IR") == 0) LOG_HEXDUMP_DBG(iregs, sizeof(iregs), "[IR] iregs");
        else if (strcmp(s, "H") == 0) flag_AM2320 = (flag_AM2320+1) % 2;
        else if (strcmp(s, "B") == 0) flag_BME280 = (flag_BME280+1) % 2;
        else if (strcmp(s, "L") == 0) flag_BH1750 = (flag_BH1750+1) % 2;
        else
        {
	        printk("unknown command: %s\n", s);
	    	//printk("last char was: 0x%x\n", s[strlen(s) - 1]);
        }		
	}
}

static void sensor_value_to_mb1(struct sensor_value *val, struct sensor_mb1* out)
{
	out->val1 = val->val1;
    out->val2 = val->val2 / 10000;
}

static void sensor_value_to_mb2(struct sensor_value *val, struct sensor_mb2* out)
{
	out->val1 = val->val1;
    out->val2 = val->val2 / 1000;
}

static void sensor(void) 
{
    struct device *bme280;

    struct device *bh1750 = device_get_binding("BH1750");
	if (!bh1750) LOG_ERR("invalid dev BH1750");
    struct device *am2320 = device_get_binding("AM2320");
	if (!am2320) LOG_ERR("invalid dev am2320");
    bme280 = device_get_binding("BME280");
	if(!bme280) LOG_ERR("invalid dev bme280");
   	
	while (1)
	{   
        if (flag_AM2320 && am2320)
        {
            struct sensor_value t,h;
            sensor_sample_fetch(am2320);
            sensor_channel_get(am2320, SENSOR_CHAN_AMBIENT_TEMP, &t);
            sensor_channel_get(am2320, SENSOR_CHAN_HUMIDITY, &h);
            struct sensor_mb1 mt,mh;
            sensor_value_to_mb1(&t, &mt);
            sensor_value_to_mb1(&h, &mh);
            printk("\033[1mSENSOR am2320:\033[0m temp:\033[32m%d.%02d\033[0m hum:\033[1;32m%d.%02d\033[0m\n", mt.val1, mt.val2, mh.val1, mh.val2);
        }
        if (flag_BH1750 && bh1750)
        {
            struct sensor_value l;
            sensor_sample_fetch(bh1750);
            sensor_channel_get(bh1750, SENSOR_CHAN_LIGHT, &l);
            struct sensor_mb2 ml;
            sensor_value_to_mb2(&l, &ml);
            printk("\033[1mSENSOR bh1750:\033[0m light:\033[1;32m%d.%03d\033[0m lux\n", ml.val1, ml.val2);
            // 
        }
        if (flag_BME280 && bme280)
        {
            struct sensor_value t,h, p;
            sensor_sample_fetch(bme280);
            sensor_channel_get(bme280, SENSOR_CHAN_AMBIENT_TEMP, &t);
            sensor_channel_get(bme280, SENSOR_CHAN_HUMIDITY, &h);
            sensor_channel_get(bme280, SENSOR_CHAN_PRESS, &p);
            uint32_t pr = sensor_value_to_double(&p)*10*0.750062;
            // printk("\033[1mSENSOR bme280:\033[0m temp:\033[32m%d.%06d\033[0m hum:\033[1;32m%d.%06d\033[0m pre:\033[1;32m%d.%06d\033[0m\n", t.val1, t.val2, h.val1, h.val2, p.val1, p.val2);
            printk("\033[1mSENSOR bme280:\033[0m temp:\033[32m%d.%06d\033[0m hum:\033[1;32m%d.%06d\033[0m pre:\033[1;32m%d\033[0m\n", t.val1, t.val2, h.val1, h.val2, pr);
            // 
        }
        k_msleep(2000);
	}	
}
K_THREAD_DEFINE(sensor_th, 1024, sensor, NULL, NULL, NULL, 7, K_ESSENTIAL, 0);

