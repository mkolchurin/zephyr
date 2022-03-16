#include <zephyr.h>
#include <logging/log.h>
#include <device.h>
#include <drivers/flash.h>
#include <drivers/eeprom.h>

#define DT_DRV_COMPAT eeprom_flash

LOG_MODULE_REGISTER(flash_eeprom, CONFIG_FLASH_LOG_LEVEL);

static const struct flash_parameters emul_parameters = {
	.write_block_size = 1,
	.erase_value = 0xff,
};

static const struct flash_pages_layout flash_layout[] = { { .pages_count = 32,
							    .pages_size = 512 } };

struct flash_eeprom_emul_config {
	struct device *eeprom;
};

static int emul_init(const struct device *dev)
{
	const struct flash_eeprom_emul_config *config = dev->config;
	const struct device *at24 = config->eeprom;
	if (!device_is_ready(at24)) {
		LOG_ERR("EEPROM EMUL '%s' is not ready\n", (at24)->name);
		return -ENODEV;
	}
	LOG_DBG("Inited");
	return 0;
}

static int emul_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	const struct flash_eeprom_emul_config *config = dev->config;
	const struct device *eeprom = config->eeprom;
	LOG_DBG("rd len '%d'  off '%d'", len, offset);
	return eeprom_read(eeprom, offset, data, len);
}
static int emul_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	const struct flash_eeprom_emul_config *config = dev->config;
	const struct device *eeprom = config->eeprom;
	LOG_HEXDUMP_DBG(data, len, "WR");
	return eeprom_write(eeprom, offset, data, len);
}
static int emul_erase(const struct device *dev, off_t offset, size_t size)
{
	const struct flash_eeprom_emul_config *config = dev->config;
	const struct device *eeprom = config->eeprom;
	int ret;
	while (size > 0) {
		ret = eeprom_write(eeprom, offset, &emul_parameters.erase_value, 1);
		size--;
	}
	return ret;
}

void emul_page_layout(const struct device *dev, const struct flash_pages_layout **layout,
		      size_t *layout_size)
{
	ARG_UNUSED(dev);

	*layout = flash_layout;
	*layout_size = ARRAY_SIZE(flash_layout);
}

static const struct flash_parameters *emul_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &emul_parameters;
}

static const struct flash_driver_api emul_api = {
	.erase = emul_erase,
	.write = emul_write,
	.read = emul_read,
	.get_parameters = emul_get_parameters,
	.page_layout = emul_page_layout,
};

#define FLASH_EEPROM_DEF(n)                                                                        \
	struct flash_eeprom_emul_config config_##n = {                                             \
		.eeprom = DEVICE_DT_GET(DT_INST_PARENT(n)),                                           \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, emul_init, NULL, NULL, &config_##n, POST_KERNEL,                 \
			 CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &emul_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_EEPROM_DEF)