/*
 * Copyright (c) 2019 George Hilliard
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <drivers/sensor.h>
#include <drivers/i2c.h>
#include <logging/log.h>

#include "bh1730fvc.h"

#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
LOG_MODULE_REGISTER(BH1730FVC);

static int bh1730fvc_read_registers(struct device *dev, const u8_t reg,
				    void *data, size_t length)
{
	struct bh1730fvc_data *bh1730fvc_data = dev->driver_data;
	const struct bh1730fvc_dev_config *cfg = dev->config->config_info;

	const u8_t read_reg_cmd = reg |
		BH1730FVC_COMMAND | BH1730FVC_TRANSACTION_ADDR;

	return i2c_write_read(bh1730fvc_data->bus, cfg->i2c_addr,
			      &read_reg_cmd, 1,
			      (u8_t *) data, length);
}

static int bh1730fvc_write_register(struct device *dev, const u8_t reg, u8_t value)
{
	struct bh1730fvc_data *bh1730fvc_data = dev->driver_data;
	const struct bh1730fvc_dev_config *cfg = dev->config->config_info;

	const u8_t read_reg_cmd = reg |
		BH1730FVC_COMMAND | BH1730FVC_TRANSACTION_ADDR;

	return i2c_burst_write(bh1730fvc_data->bus, cfg->i2c_addr,
			       read_reg_cmd, &value, 1);
}

static int bh1730fvc_attr_set(struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr,
			      const struct sensor_value *val)
{
	return -ENOTSUP;
}

static int bh1730fvc_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	int ret;
	u8_t buf[4];

	struct bh1730fvc_data *bh1730fvc_data = dev->driver_data;

	if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	ret = bh1730fvc_write_register(dev, BH1730FVC_CONTROL,
				       BH1730FVC_CONTROL_ONE_TIME
				       | BH1730FVC_CONTROL_ADC_EN
				       | BH1730FVC_CONTROL_POWER);
	if (ret) {
		return ret;
	}

	do {
		ret = bh1730fvc_read_registers(dev, BH1730FVC_CONTROL, buf, 1);
		if (ret) {
			return ret;
		}
	} while(!(buf[0] & BH1730FVC_CONTROL_ADC_EN));

	ret = bh1730fvc_read_registers(dev, BH1730FVC_DATA0LOW, buf, 4);
	if (ret) {
		return ret;
	}

	bh1730fvc_data->sample.visible = (buf[1] << 8) | buf[0];
	bh1730fvc_data->sample.infrared = (buf[3] << 8) | buf[2];

	return 0;
}

static int bh1730fvc_channel_get(struct device *dev,
				 enum sensor_channel chan,
				 struct sensor_value *val)
{
	struct bh1730fvc_data *bh1730fvc_data = dev->driver_data;

	switch (chan) {
	case SENSOR_CHAN_LIGHT:
		return bh1730fvc_data->sample.visible;
	case SENSOR_CHAN_IR:
		return bh1730fvc_data->sample.infrared;
	default:
		return -ENOTSUP;
	}
}

static int bh1730fvc_probe(struct device *dev)
{
	int ret;
	u8_t bh1730fvc_id;

	ret = bh1730fvc_read_registers(dev, BH1730FVC_ID, &bh1730fvc_id, 1);
	if (ret) {
		return ret;
	}

	if (BH1730FVC_ID_PART(bh1730fvc_id) != BH1730FVC_ID_PART_VALUE) {
		LOG_ERR("failed to read ID (%u)", bh1730fvc_id);
		return -ENODEV;
	}

	return 0;
}

static int bh1730fvc_init(struct device *dev)
{
	struct bh1730fvc_data *data = dev->driver_data;
	const struct bh1730fvc_dev_config *cfg = dev->config->config_info;

	data->bus = device_get_binding(cfg->i2c_port);
	if (data->bus == NULL) {
		LOG_ERR("Failed to get pointer to %s device!",
			cfg->i2c_port);
		return -EINVAL;
	}

	if (bh1730fvc_probe(dev) != 0) {
		return -ENODEV;
	}

	return 0;
}

static const struct sensor_driver_api bh1730fvc_api_funcs = {
	.attr_set     = bh1730fvc_attr_set,
	.sample_fetch = bh1730fvc_sample_fetch,
	.channel_get  = bh1730fvc_channel_get,
#ifdef CONFIG_BH1730FVC_TRIGGER
	.trigger_set = bh1730fvc_trigger_set,
#endif
};

static struct bh1730fvc_dev_config bh1730fvc_config = {
	.i2c_port = DT_INST_0_ROHM_BH1730FVC_BUS_NAME,
	.i2c_addr = DT_INST_0_ROHM_BH1730FVC_BASE_ADDRESS,
#ifdef CONFIG_BH1730FVC_TRIGGER
	.gpio_port = DT_INST_0_ROHM_BH1730FVC_INT1_GPIOS_CONTROLLER,
	.int_gpio = DT_INST_0_ROHM_BH1730FVC_INT1_GPIOS_PIN,
#endif
};

static struct bh1730fvc_data bh1730fvc_data;

DEVICE_AND_API_INIT(bh1730fvc, DT_INST_0_ROHM_BH1730FVC_LABEL, bh1730fvc_init,
		    &bh1730fvc_data, &bh1730fvc_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &bh1730fvc_api_funcs);
