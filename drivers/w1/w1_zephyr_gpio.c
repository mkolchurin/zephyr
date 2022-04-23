/*
 * Copyright (c) 2022 Maxim Kolchurin
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_w1_gpio

/**
 * @brief Driver for 1-Wire Master using GPIO.
 *
 * This driver implements the 1-Wire interface using an GPIO.
 */

#include <stdint.h>
#include <stdbool.h>
#include <drivers/w1.h>
#include <logging/log.h>
#include <sys/__assert.h>
#include <drivers/gpio.h>
#include <kernel.h>
#define PWM_NUM 1

LOG_MODULE_REGISTER(w1_gpio, CONFIG_W1_LOG_LEVEL);

#define W1_1_LENGTH 6
#define W1_0_LENGTH 60
#define W1_BIT_LENGTH 70
#define W1_READ_LENGTH_A 6
#define W1_READ_LENGTH_B 6
#define W1_READ_LENGTH_C 55
#define W1_RESET_RECOVERY 0
#define W1_RESET_A 480
#define W1_RESET_B 70
#define W1_RESET_C 410

/* usec upper this limit will use k_usleep() instead k_busy_wait() */
#define W1_SOFT_WAIT_LIMIT 80

struct w1_gpio_config {
	/** w1 controller config, common to all drivers */
	struct w1_controller_config ctrl_config;
	/** GPIO used for 1-Wire communication */
	struct gpio_dt_spec gpio_dt;
};
struct w1_gpio_data {
	/** w1 controller data, common to all drivers */
	struct w1_controller_data ctrl_data;
	bool overdrive_active;
};


static inline int w1_gpio_configure_output(const struct gpio_dt_spec *spec)
{
	return gpio_pin_configure_dt(spec, GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN);
}

static inline int w1_gpio_configure_input(const struct gpio_dt_spec *spec)
{
	return gpio_pin_configure_dt(spec, GPIO_INPUT);
}

static inline void w1_wait(int usec)
{
	if (usec > W1_SOFT_WAIT_LIMIT) {
		k_usleep(usec);
	} else {
		k_busy_wait(usec);
	}
}

static int gpio_write_bit(const struct device *dev, uint8_t bit)
{
	const struct w1_gpio_config *cfg = dev->config;

	int us_start = (bit) ? W1_1_LENGTH : W1_0_LENGTH;
	int us_end = W1_BIT_LENGTH - us_start;

	gpio_pin_set_dt(&cfg->gpio_dt, 0);
	w1_wait(us_start);
	gpio_pin_set_dt(&cfg->gpio_dt, 1);
	w1_wait(us_end);
	return 0;
}

static int gpio_read_bit(const struct device *dev, uint8_t *bit)
{
	const struct w1_gpio_config *cfg = dev->config;
	const struct gpio_dt_spec *gpio_dt = &cfg->gpio_dt;
	int err = 0;

	err = gpio_pin_set_dt(&cfg->gpio_dt, 0);
	if (err < 0) {
		return err;
	}
	w1_wait(W1_READ_LENGTH_A);
	err = w1_gpio_configure_input(gpio_dt);
	if (err < 0) {
		return err;
	}
	w1_wait(W1_READ_LENGTH_B);
	*bit = gpio_pin_get_dt(gpio_dt);
	err = w1_gpio_configure_output(gpio_dt);
	if (err < 0) {
		return err;
	}
	w1_wait(W1_READ_LENGTH_C);
	return err;
}

static int w1_gpio_reset_bus(const struct device *dev)
{
	const struct w1_gpio_config *cfg = dev->config;
	const struct gpio_dt_spec *gpio_dt = &cfg->gpio_dt;
	int err = 0;

	w1_wait(W1_RESET_RECOVERY);
	err = gpio_pin_set_dt(&cfg->gpio_dt, 0);
	if (err < 0) {
		return err;
	}
	w1_wait(W1_RESET_A);
	err = w1_gpio_configure_input(gpio_dt);
	if (err < 0) {
		return err;
	}
	w1_wait(W1_RESET_B);
	int presense = gpio_pin_get_dt(&cfg->gpio_dt);

	err = w1_gpio_configure_output(gpio_dt);
	if (err < 0) {
		return err;
	}
	w1_wait(W1_RESET_C);
	return presense ? 0 : 1;
}

static int w1_gpio_read_bit(const struct device *dev)
{
	uint8_t bit = 0;
	int err = 0;

	err = gpio_read_bit(dev, &bit);

	return (err < 0) ? err : bit;
}

static int w1_gpio_write_bit(const struct device *dev, const bool bit)
{
	return gpio_write_bit(dev, bit);
}

static int w1_gpio_read_byte(const struct device *dev)
{
	uint8_t rx_byte = 0;
	uint8_t bit;
	int err = 0;

	for (int i = 0; i < 8; i++) {
		bit = 0;
		err = gpio_read_bit(dev, &bit);
		rx_byte |= (bit << i);
		if (err < 0) {
			return err;
		}
	}
	return rx_byte;
}

static int w1_gpio_write_byte(const struct device *dev, const uint8_t byte)
{
	int err;

	for (int i = 0; i < 8; i++) {
		err = gpio_write_bit(dev, ((byte & (0x1 << i)) >> i));
		if (err < 0) {
			return err;
		}
	}
	return 0;
}

static int w1_gpio_configure(const struct device *dev,
			     enum w1_driver_settings_type type,
			     uint32_t value)
{
	const struct w1_gpio_config *cfg = dev->config;
	struct w1_gpio_data *data = dev->data;


	int ret = 0;
	bool temp;

	switch (type) {
	case W1_SETTING_SPEED:
		ret = 0;
		break;
	default:
		ret = -ENOTSUP;
	}
	return ret;
}

void capture_callback(const struct device *dev,
		      uint32_t pwm, uint32_t period_cycles,
		      uint32_t pulse_cycles, int status, void *user_data)
{
	LOG_INF("CAPTURE %d", sys_clock_cycle_get_32());
}

static int w1_gpio_init(const struct device *dev)
{
	const struct w1_gpio_config *cfg = dev->config;
	struct w1_gpio_data *data = dev->data;

	if (!device_is_ready(cfg->gpio_dt.port)) {
		LOG_ERR("Device %s is not ready", cfg->gpio_dt.port->name);
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&cfg->gpio_dt, GPIO_OPEN_DRAIN | GPIO_OUTPUT);

	if (ret < 0) {
		LOG_ERR("%d failed to configure %s pin %d",
			ret, cfg->gpio_dt.port->name, cfg->gpio_dt.pin);
		return -EINVAL;
	}

	data->overdrive_active = false;

	return 0;
}

static const struct w1_driver_api w1_gpio_driver_api = {
	.reset_bus = w1_gpio_reset_bus,
	.read_bit = w1_gpio_read_bit,
	.write_bit = w1_gpio_write_bit,
	.read_byte = w1_gpio_read_byte,
	.write_byte = w1_gpio_write_byte,
	.configure = w1_gpio_configure,
};

#define W1_ZEPHYR_GPIO_INIT(inst)					       \
	static const struct w1_gpio_config w1_gpio_cfg_##inst = {	       \
		.gpio_dt = GPIO_DT_SPEC_INST_GET_BY_IDX(inst, ctrl_gpios, 0),  \
		.ctrl_config.client_count = W1_INST_PERIPHERALS_COUNT(inst),   \
	};								       \
	static struct w1_gpio_data w1_gpio_data_##inst = {};		       \
	DEVICE_DT_INST_DEFINE(inst, &w1_gpio_init, NULL, &w1_gpio_data_##inst, \
			      &w1_gpio_cfg_##inst, POST_KERNEL,		       \
			      CONFIG_W1_INIT_PRIORITY, &w1_gpio_driver_api);   \

DT_INST_FOREACH_STATUS_OKAY(W1_ZEPHYR_GPIO_INIT)
