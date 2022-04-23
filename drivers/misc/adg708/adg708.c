
#include <drivers/gpio.h>
#include <zephyr.h>
#include <logging/log.h>
#include <sys/byteorder.h>
#include <errno.h>

#define DT_DRV_COMPAT adi_adg708

LOG_MODULE_REGISTER(adg708, CONFIG_ADG708_LOG_LEVEL);

struct adg708_gpio_t {
	struct gpio_dt_spec a0;
	struct gpio_dt_spec a1;
	struct gpio_dt_spec a2;
	struct gpio_dt_spec nEN;
};

struct adg708_config {
	struct adg708_gpio_t gpios;
};

int adg708_select_channel(const struct device *dev, uint32_t channel)
{
	struct adg708_config *config = (struct adg708_config *)dev->config;
	gpio_pin_set_dt(&config->gpios.a0, channel & 1);
	gpio_pin_set_dt(&config->gpios.a1, (channel & 2) >> 1);
	gpio_pin_set_dt(&config->gpios.a2, (channel & 4) >> 2);
	gpio_pin_set_dt(&config->gpios.nEN, 0);
	LOG_DBG("set %d ch", channel);
	return 0;
}
int adg708_reset(const struct device *dev)
{
	struct adg708_config *config = (struct adg708_config *)dev->config;

	gpio_pin_set(config->gpios.nEN.port, config->gpios.nEN.pin, 1);

	LOG_DBG("reset");
	return 0;
}

int adg708_init(const struct device *dev)
{
	struct adg708_config *config = (struct adg708_config *)dev->config;
	int ret = 0;
	ret += gpio_pin_configure_dt(&config->gpios.a0,
				     config->gpios.a0.dt_flags | GPIO_OUTPUT_HIGH);
	ret += gpio_pin_configure_dt(&config->gpios.a1,
				     config->gpios.a1.dt_flags | GPIO_OUTPUT_HIGH);
	ret += gpio_pin_configure_dt(&config->gpios.a2,
				     config->gpios.a2.dt_flags | GPIO_OUTPUT_HIGH);
	ret += gpio_pin_configure_dt(&config->gpios.nEN,
				     config->gpios.nEN.dt_flags | GPIO_OUTPUT_HIGH);

	if (ret != 0) {
		LOG_ERR("Failed to init");
		return -ENODEV;
	}
	adg708_reset(dev);
	LOG_DBG("inited");
	return 0;
}

#define ADG708_INIT(n) \
static const struct adg708_config \
	pinmux_adg708_port_cfg_id##n = { .gpios = { \
					       .a0 = GPIO_DT_SPEC_INST_GET_BY_IDX(n, ctrl_gpios, 0), \
					       .a1 = GPIO_DT_SPEC_INST_GET_BY_IDX(n, ctrl_gpios, 1), \
					       .a2 = GPIO_DT_SPEC_INST_GET_BY_IDX(n, ctrl_gpios, 2), \
					       .nEN = GPIO_DT_SPEC_INST_GET_BY_IDX(n, ctrl_gpios, \
										   3), \
				       } }; \
DEVICE_DT_INST_DEFINE(n, adg708_init, NULL, NULL, &pinmux_adg708_port_cfg_id##n, POST_KERNEL, \
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL); \

DT_INST_FOREACH_STATUS_OKAY(ADG708_INIT);