#include <zephyr.h>
#include <device.h>
//
//int am2320_init(struct device *dev)
//{
//	u8_t id, idx;
//
//	struct device *i2c
//
//	dev = device_get_binding("I2C1");
//	if (data->i2c == NULL) {
//		LOG_ERR("Could not get pointer to %s device.", I2C1);
//		return -EINVAL;
//	}
//
//	/* check chip ID */
//	if (i2c_reg_read_byte(data->i2c, cfg->i2c_addr,
//			      HTS221_REG_WHO_AM_I, &id) < 0) {
//		LOG_ERR("Failed to read chip ID.");
//		return -EIO;
//	}
//
//	if (id != HTS221_CHIP_ID) {
//		LOG_ERR("Invalid chip ID.");
//		return -EINVAL;
//	}
//
//	/* check if CONFIG_HTS221_ODR is valid */
//	for (idx = 0U; idx < ARRAY_SIZE(hts221_odr_strings); idx++) {
//		if (!strcmp(hts221_odr_strings[idx], CONFIG_HTS221_ODR)) {
//			break;
//		}
//	}
//
//	if (idx == ARRAY_SIZE(hts221_odr_strings)) {
//		LOG_ERR("Invalid ODR value.");
//		return -EINVAL;
//	}
//
//	if (i2c_reg_write_byte(data->i2c, cfg->i2c_addr,
//			       HTS221_REG_CTRL1,
//			       (idx + 1) << HTS221_ODR_SHIFT | HTS221_BDU_BIT |
//			       HTS221_PD_BIT) < 0) {
//		LOG_ERR("Failed to configure chip.");
//		return -EIO;
//	}
//
//	/*
//	 * the device requires about 2.2 ms to download the flash content
//	 * into the volatile mem
//	 */
//	k_sleep(K_MSEC(3));
//
//	if (hts221_read_conversion_data(dev) < 0) {
//		LOG_ERR("Failed to read conversion data.");
//		return -EINVAL;
//	}
//
//#ifdef CONFIG_HTS221_TRIGGER
//	if (hts221_init_interrupt(dev) < 0) {
//		LOG_ERR("Failed to initialize interrupt.");
//		return -EIO;
//	}
//#endif
//
//	return 0;
//}
//