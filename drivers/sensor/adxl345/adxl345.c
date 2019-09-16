#include <kernel.h>
#include <drivers/sensor.h>
#include <drivers/i2c.h>
#include <logging/log.h>

#include "adxl345.h"

#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
LOG_MODULE_REGISTER(ADXL345);

static int adxl345_bus_access(struct device *dev, u8_t reg,
			      void *data, size_t length)
{
	struct adxl345_data *adxl345_data = dev->driver_data;

#ifdef CONFIG_ADXL345_I2C
	const struct adxl345_dev_config *cfg = dev->config->config_info;
	const u8_t reg_buffer[1] = {ADXL345_TO_I2C_REG(reg)};

	if (reg & ADXL345_READ) {
		return i2c_write_read(adxl345_data->bus, cfg->i2c_addr,
				      reg_buffer, 1,
				      (u8_t *) data, length);
	} else {
		if (length != 1) {
			return -EINVAL;
		}

		return i2c_reg_write_byte(adxl345_data->bus, cfg->i2c_addr,
					  reg_buffer[0], *(u8_t *)data);
	}
#endif
};

/**
 * Read from device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl345_reg_read(struct device *dev,
			    u8_t reg_addr,
			    u8_t *reg_data)
{
	return adxl345_bus_access(dev, ADXL345_REG_READ(reg_addr),
				  reg_data, 1);
}

/**
 * Multibyte read from device. A register read begins with the address
 * and autoincrements for each additional byte in the transfer.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @param count - Number of bytes to read.
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl345_reg_read_multiple(struct device *dev,
				     u8_t reg_addr,
				     u8_t *reg_data,
				     u16_t count)
{
	return adxl345_bus_access(dev, ADXL345_REG_READ(reg_addr),
				  reg_data, count);
}

/**
 * Get the DATA_READY status.
 * @param dev - The device structure.
 * @param ready - The resulting true/false readiness
 * @return 0 in case of success, negative error code otherwise.
 */
int adxl345_get_data_ready(struct device *dev, bool *ready)
{
	u8_t buf[1];
	int ret;

	ret = adxl345_reg_read(dev, ADXL345_INT_ENABLE, buf);

	*ready = (buf[0] & ADXL345_DATA_READY) != 0;

	return ret;
}

static int adxl345_attr_set(struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	switch (attr) {
	default:
		return -ENOTSUP;
	}
}

/**
 * Retrieve 3-axis acceleration data
 * @param dev - The device structure.
 * @param accel_data - pointer to a variable of type adxl345_xyz_accel_data
 *		      where (x, y, z) acceleration data will be stored.
 * @return 0 in case of success, negative error code otherwise.
 */
static int adxl345_get_accel_data(struct device *dev,
			          struct adxl345_xyz_accel_data *accel_data)
{
	u8_t buf[6];
	bool ready = false;
	int ret;

	if (!IS_ENABLED(CONFIG_ADXL345_TRIGGER)) {
		do {
			adxl345_get_data_ready(dev, &ready);
		} while (!ready);
	}

	ret = adxl345_reg_read_multiple(dev, ADXL345_DATAX0, buf, 6);

	accel_data->x = (buf[0] << 8) | (buf[1] & 0xF0);
	accel_data->y = (buf[2] << 8) | (buf[3] & 0xF0);
	accel_data->z = (buf[4] << 8) | (buf[5] & 0xF0);

	return ret;
}

static int adxl345_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct adxl345_data *data = dev->driver_data;

	return adxl345_get_accel_data(dev, &data->sample);
}

static void adxl345_accel_convert(struct sensor_value *val, s16_t value)
{
	// TODO double check all this against the datasheet --georgev
	/*
	 * Sensor resolution is 100mg/LSB, 12-bit value needs to be right
	 * shifted by 4 or divided by 16. Overall this results in a scale of 160
	 */
	s32_t micro_ms2 = value * (SENSOR_G / (16 * 1000 / 100));

	val->val1 = micro_ms2 / 1000000;
	val->val2 = micro_ms2 % 1000000;
}

static int adxl345_channel_get(struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct adxl345_data *data = dev->driver_data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		adxl345_accel_convert(val, data->sample.x);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		adxl345_accel_convert(val, data->sample.y);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		adxl345_accel_convert(val, data->sample.z);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		adxl345_accel_convert(val++, data->sample.x);
		adxl345_accel_convert(val++, data->sample.y);
		adxl345_accel_convert(val, data->sample.z);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int adxl345_probe(struct device *dev)
{
	u8_t dev_id;
	int ret;

	ret = adxl345_reg_read(dev, ADXL345_DEVID, &dev_id);
	if (ret) {
		return ret;
	}

	if (dev_id != ADXL345_ID) {
		LOG_ERR("failed to read id (0x%X)", dev_id);
		return -ENODEV;
	}

#ifdef CONFIG_ADXL345_TRIGGER
	if (adxl345_init_interrupt(dev) < 0) {
		LOG_ERR("Failed to initialize interrupt!");
		return -EIO;
	}
#endif

	return 0;
}

static int adxl345_init(struct device *dev)
{
	struct adxl345_data *data = dev->driver_data;
	const struct adxl345_dev_config *cfg = dev->config->config_info;

#ifdef CONFIG_ADXL345_I2C
	data->bus  = device_get_binding(cfg->i2c_port);
	if (data->bus == NULL) {
		LOG_ERR("Failed to get pointer to %s device!",
			cfg->i2c_port);
		return -EINVAL;
	}
#endif

	if (adxl345_probe(dev) < 0) {
		return -ENODEV;
	}

	return 0;
}

static const struct sensor_driver_api adxl345_api_funcs = {
	.attr_set     = adxl345_attr_set,
	.sample_fetch = adxl345_sample_fetch,
	.channel_get  = adxl345_channel_get,
#ifdef CONFIG_ADXL345_TRIGGER
	.trigger_set = adxl345_trigger_set,
#endif
};

static struct adxl345_data adxl345_data;

static const struct adxl345_dev_config adxl345_config = {
#ifdef CONFIG_ADXL345_I2C
	.i2c_port = DT_INST_0_ADI_ADXL345_BUS_NAME,
	.i2c_addr = DT_INST_0_ADI_ADXL345_BASE_ADDRESS,
#endif
#ifdef CONFIG_ADXL345_TRIGGER
	.gpio_port = DT_INST_0_ADI_ADXL345_INT1_GPIOS_CONTROLLER,
	.int_gpio = DT_INST_0_ADI_ADXL345_INT1_GPIOS_PIN,
#endif
};

DEVICE_AND_API_INIT(adxl345, DT_INST_0_ADI_ADXL345_LABEL, adxl345_init,
		    &adxl345_data, &adxl345_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &adxl345_api_funcs);
