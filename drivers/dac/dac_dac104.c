/*
 * Copyright (c) 2020 M2I Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/spi.h>
#include <drivers/dac.h>
#include <logging/log.h>
#include <sys/byteorder.h>

#define DT_DRV_COMPAT ti_dac104_dac

LOG_MODULE_REGISTER(dac_dac104, CONFIG_DAC_LOG_LEVEL);

#define DAC104_MAX_CHANNEL 4

enum DAC104_channel {
	DAC104_CH_A = (0b00),
	DAC104_CH_B = (0b01),
	DAC104_CH_C = (0b10),
	DAC104_CH_D = (0b11)
};
enum DAC104_opmode {
	DAC104_WR_ONE_NOT_UPDATE = (0b00),
	DAC104_WR_ONE_UPDATE = (0b01),
	DAC104_WR_ALL_UPDATE = (0b10),
	DAC104_POWER_DOWN_OUTPUTS = (0b11)
};

struct dac104_config {
	struct spi_dt_spec bus;
	uint8_t resolution;
	uint8_t reference;
};

struct dac104_data {
	uint8_t configured;
};

static int dac104_reg_write(const struct device *dev, uint8_t addr,
			      	uint16_t data)
{
	const struct dac104_config *config = dev->config;

    uint16_t tx_message = data & 0xFFF;

	const struct spi_buf buf[1] = {
		{
			.buf = &tx_message,
			.len = sizeof(tx_message)
		}
	};
	struct spi_buf_set tx = {
		.buffers = buf,
		.count = ARRAY_SIZE(buf),
	};

    char op = DAC104_WR_ONE_UPDATE;
    
    WRITE_BIT(tx_message, 12, op & 0b01);
    WRITE_BIT(tx_message, 13, ((op & 0b10) >> 1));
    WRITE_BIT(tx_message, 14, addr & 0b01);
    WRITE_BIT(tx_message, 15, ((addr & 0b10) >> 1));  
	tx_message = sys_cpu_to_be16(tx_message);

	LOG_DBG("tx %d, op = '%d', addr = '%d', data = '%d'", tx_message, op, addr, data);
	LOG_HEXDUMP_DBG((uint8_t *)tx_message, 2, "adc_write" );
	
	if (k_is_in_isr()) {
		/* Prevent SPI transactions from an ISR */
		return -EWOULDBLOCK;
	}

	return spi_write_dt(&config->bus, &tx);
}

static int dac104_write_value(const struct device *dev, uint8_t channel,
				uint32_t value)
{
	const struct dac104_config *config = dev->config;
	struct dac104_data *data = dev->data;
	int ret;

	if (channel > DAC104_MAX_CHANNEL - 1) {
		LOG_ERR("unsupported channel %d", channel);
		return -ENOTSUP;
	}

	if (!(data->configured & BIT(channel))) {
		LOG_ERR("Channel not initialized");
		return -EINVAL;
	}

	if (value >= (1 << config->resolution)) {
		LOG_ERR("Value %d out of range", value);
		return -EINVAL;
	}

	ret = dac104_reg_write(dev, DAC104_CH_A+channel, value);
	if (ret) {
		return -EIO;
	}

	return 0;
}

static int dac104_channel_setup(const struct device *dev,
				   const struct dac_channel_cfg *channel_cfg)
{
	const struct dac104_config *config = dev->config;
	struct dac104_data *data = dev->data;

	if (channel_cfg->channel_id > (DAC104_MAX_CHANNEL - 1)) {
		LOG_ERR("Unsupported channel %d", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	if (channel_cfg->resolution != config->resolution) {
		LOG_ERR("Unsupported resolution %d", channel_cfg->resolution);
		return -ENOTSUP;
	}

	data->configured |= BIT(channel_cfg->channel_id);
	dac104_write_value(dev, channel_cfg->channel_id, 0);
	return 0;
}



static int dac104_init(const struct device *dev)
{
	const struct dac104_config *config = dev->config;
	struct dac104_data *data = dev->data;
	int ret;

	if (!spi_is_ready(&config->bus)) {
		LOG_ERR("SPI bus %s not ready", config->bus.bus->name);
		return -ENODEV;
	}
	LOG_DBG("SPI freq %d", config->bus.config.frequency);
	LOG_DBG("DAC104 init");
	
	data->configured = 0;

	return 0;
}

static const struct dac_driver_api dac104_driver_api = {
	.channel_setup = dac104_channel_setup,
	.write_value = dac104_write_value,
};

#define DAC_DAC104_INIT(num) \
	static struct dac104_data dac104_data_##num; \
	static const struct dac104_config dac104_config_##num = { \
		.bus = SPI_DT_SPEC_INST_GET(num, \
			SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | \
			SPI_WORD_SET(8) | SPI_MODE_CPHA, 0), \
			.resolution = 12, \
	}; \
	DEVICE_DT_INST_DEFINE(num, \
			    &dac104_init, NULL, \
			    &dac104_data_##num, \
			    &dac104_config_##num, POST_KERNEL, \
			    CONFIG_DAC_INIT_PRIORITY, \
			    &dac104_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DAC_DAC104_INIT);
