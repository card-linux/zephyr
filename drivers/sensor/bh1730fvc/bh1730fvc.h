#ifndef ZEPHYR_DRIVERS_SENSOR_BH1730FVC_H
#define ZEPHYR_DRIVERS_SENSOR_BH1730FVC_H

#include <zephyr/types.h>

// Register map
#define BH1730FVC_CONTROL		0x00
#define BH1730FVC_TIMING		0x01
#define BH1730FVC_INTERRUPT		0x02
#define BH1730FVC_THLLOW		0x03
#define BH1730FVC_THLHIGH		0x04
#define BH1730FVC_THHLOW		0x05
#define BH1730FVC_THHHIGH		0x06
#define BH1730FVC_GAIN			0x07
#define BH1730FVC_ID			0x12
#define BH1730FVC_DATA0LOW		0x14
#define BH1730FVC_DATA0HIGH		0x15
#define BH1730FVC_DATA1LOW		0x16
#define BH1730FVC_DATA1HIGH		0x17

// Command fields
#define BH1730FVC_COMMAND		BIT(7)
#define BH1730FVC_TRANSACTION_SPECIAL	(BIT(6) | BIT(5))
#define BH1730FVC_TRANSACTION_ADDR	0x0
#define BH1730FVC_COMMAND_RESET		BIT(2)

// CONTROL fields
#define BH1730FVC_CONTROL_ADC_VALID	BIT(4)
#define BH1730FVC_CONTROL_ONE_TIME	BIT(3)
#define BH1730FVC_CONTROL_ADC_EN	BIT(1)
#define BH1730FVC_CONTROL_POWER		BIT(0)

// ID fields
#define BH1730FVC_ID_PART(x)		(((x) >> 4) & 0xF)
#define BH1730FVC_ID_PART_VALUE		7

struct bh1730fvc_dev_config {
	const char *i2c_port;
	u16_t i2c_addr;
#ifdef CONFIG_BH1730FVC_TRIGGER
	const char *gpio_port;
	u8_t int_gpio;
#endif
};

struct bh1730fvc_lux_data {
	u16_t visible;
	u16_t infrared;
};

struct bh1730fvc_data {
	struct device *bus;
	struct bh1730fvc_lux_data sample;
#ifdef CONFIG_BH1730FVC_TRIGGER
	struct device *gpio;
	struct gpio_callback gpio_cb;
#endif
};

#endif /* ZEPHYR_DRIVERS_SENSOR_BH1730FVC_H */
