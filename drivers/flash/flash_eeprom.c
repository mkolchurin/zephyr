#include <zephyr.h>
#include <logging/log.h>
#include <device.h>
#include <drivers/flash.h>
#include <drivers/eeprom.h>

#define DT_DRV_COMPAT eeprom_flash

LOG_MODULE_REGISTER(flash_eeprom, CONFIG_FLASH_LOG_LEVEL);

static const struct flash_parameters emul_parameters = {
	.write_block_size = 1,
#ifdef CONFIG_FLASH_EEPROM_PSEUDO_ERASE
	.erase_value = FLASH_EEPROM_ERASE_SYMBOL,
#else	
	.erase_value = 0,
#endif
	
};

static struct flash_pages_layout flash_layout = { .pages_count = 0, .pages_size = 0 };

struct flash_eeprom_emul_config {
	struct device *eeprom;
	size_t page_size;
	size_t page_count;
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
	int ret = 0;
#ifdef CONFIG_FLASH_EEPROM_PSEUDO_ERASE
	const struct flash_eeprom_emul_config *config = dev->config;
	const struct device *eeprom = config->eeprom;
	
	while (size > 0) {
		ret = eeprom_write(eeprom, offset, &emul_parameters.erase_value, 1);
		size--;
	}
#endif
	return ret;
}

void emul_page_layout(const struct device *dev, const struct flash_pages_layout **layout,
		      size_t *layout_size)
{
	const struct flash_eeprom_emul_config *config = dev->config;
	flash_layout.pages_count = config->page_count;
	flash_layout.pages_size = config->page_size;

	*layout = &flash_layout;
	*layout_size = 1;
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

#define EEPROM_DEVICE(n) DEVICE_DT_GET(DT_INST_PARENT(n))

#define FLASH_EEPROM_DEF(n)                                                                        \
	struct flash_eeprom_emul_config config_##n = {                                             \
		.eeprom = EEPROM_DEVICE(n),                                                        \
		.page_size = DT_INST_PROP(n, page_size),                                           \
		.page_count = DT_INST_PROP(n, page_count),                                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, emul_init, NULL, NULL, &config_##n, POST_KERNEL,                  \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &emul_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_EEPROM_DEF)