#ifndef ZEPHYR_DRIVERS_SENSOR_ADXL345_ADXL345_H_
#define ZEPHYR_DRIVERS_SENSOR_ADXL345_ADXL345_H_

#include <zephyr/types.h>

/* ADXL345 Register Map */
#define	ADXL345_DEVID           0x00 // R   Device ID.
#define ADXL345_THRESH_TAP      0x1D // R/W Tap threshold.
#define ADXL345_OFSX            0x1E // R/W X-axis offset.
#define ADXL345_OFSY            0x1F // R/W Y-axis offset.
#define ADXL345_OFSZ            0x20 // R/W Z-axis offset.
#define ADXL345_DUR             0x21 // R/W Tap duration.
#define ADXL345_LATENT          0x22 // R/W Tap latency.
#define ADXL345_WINDOW          0x23 // R/W Tap window.
#define ADXL345_THRESH_ACT      0x24 // R/W Activity threshold.
#define ADXL345_THRESH_INACT    0x25 // R/W Inactivity threshold.
#define ADXL345_TIME_INACT      0x26 // R/W Inactivity time.
#define ADXL345_ACT_INACT_CTL   0x27 // R/W Axis enable control for activity
// and inactivity detection.
#define ADXL345_THRESH_FF       0x28 // R/W Free-fall threshold.
#define ADXL345_TIME_FF         0x29 // R/W Free-fall time.
#define ADXL345_TAP_AXES        0x2A // R/W Axis control for tap/double tap.
#define ADXL345_ACT_TAP_STATUS  0x2B // R   Source of tap/double tap.
#define ADXL345_BW_RATE         0x2C // R/W Data rate and power mode control.
#define ADXL345_POWER_CTL       0x2D // R/W Power saving features control.
#define ADXL345_INT_ENABLE      0x2E // R/W Interrupt enable control.
#define ADXL345_INT_MAP         0x2F // R/W Interrupt mapping control.
#define ADXL345_INT_SOURCE      0x30 // R   Source of interrupts.
#define ADXL345_DATA_FORMAT     0x31 // R/W Data format control.
#define ADXL345_DATAX0          0x32 // R   X-Axis Data 0.
#define ADXL345_DATAX1          0x33 // R   X-Axis Data 1.
#define ADXL345_DATAY0          0x34 // R   Y-Axis Data 0.
#define ADXL345_DATAY1          0x35 // R   Y-Axis Data 1.
#define ADXL345_DATAZ0          0x36 // R   Z-Axis Data 0.
#define ADXL345_DATAZ1          0x37 // R   Z-Axis Data 1.
#define ADXL345_FIFO_CTL        0x38 // R/W FIFO control.
#define ADXL345_FIFO_STATUS     0x39 // R   FIFO status.

/* ADXL345_INT_ENABLE / ADXL345_INT_MAP / ADXL345_INT_SOURCE definition */
#define ADXL345_DATA_READY      (1 << 7)
#define ADXL345_SINGLE_TAP      (1 << 6)
#define ADXL345_DOUBLE_TAP      (1 << 5)
#define ADXL345_ACTIVITY        (1 << 4)
#define ADXL345_INACTIVITY      (1 << 3)
#define ADXL345_FREE_FALL       (1 << 2)
#define ADXL345_WATERMARK       (1 << 1)
#define ADXL345_OVERRUN         (1 << 0)

#define ADXL345_ID              0xE5

#define ADXL345_READ		0x01u
#define ADXL345_REG_READ(x)	(((x & 0xFF) << 1) | ADXL345_READ)
#define ADXL345_REG_WRITE(x)	((x & 0xFF) << 1)
#define ADXL345_TO_I2C_REG(x)	((x) >> 1)

struct adxl345_dev_config {
#ifdef CONFIG_ADXL345_I2C
	const char *i2c_port;
	u16_t i2c_addr;
#endif
#ifdef CONFIG_ADXL345_TRIGGER
	const char *gpio_port;
	u8_t int_gpio;
#endif
};

struct adxl345_xyz_accel_data {
	s16_t x;
	s16_t y;
	s16_t z;
};

struct adxl345_data {
	struct device *bus;
	struct adxl345_xyz_accel_data sample;

#ifdef CONFIG_ADXL345_TRIGGER
	struct device *gpio;
	struct gpio_callback gpio_cb;

	sensor_trigger_handler_t th_handler;
	struct sensor_trigger th_trigger;
	sensor_trigger_handler_t drdy_handler;
	struct sensor_trigger drdy_trigger;
#endif /* CONFIG_ADXL345_TRIGGER */
};

#endif /* ZEPHYR_DRIVERS_SENSOR_ADXL345_ADXL345_H_ */
