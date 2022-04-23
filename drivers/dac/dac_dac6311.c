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

#define DT_DRV_COMPAT ti_dac6311_dac

LOG_MODULE_REGISTER(dac_dac6311, CONFIG_DAC_LOG_LEVEL);

enum DAC6311_opmode_e {
	DAC6311_NORMAL_MODE,
	DAC6311_TO1KOHM_MODE,
	DAC6311_TO100KOHM_MODE,
	DAC6311_HIZ_MODE
};

struct dac6311_config {
	struct spi_dt_spec bus;
	uint8_t resolution;
	uint8_t reference;
};

struct dac6311_data {
	uint8_t configured;
};

static int dac6311_reg_write(const struct device *dev, uint16_t data, enum DAC6311_opmode_e op)
{
	const struct dac6311_config *config = dev->config;

	uint16_t tx_message = data << 4;

	WRITE_BIT(tx_message, 14, (((uint8_t)op & 0b10) >> 1));
	WRITE_BIT(tx_message, 15, (uint8_t)op & 0b01);

	tx_message = sys_cpu_to_be16(tx_message);
	LOG_DBG("tx %d, op = '%d', data = '%d'", tx_message, op, data);
	LOG_HEXDUMP_DBG((uint8_t *)&tx_message, 2, "adc_write");


	const struct spi_buf buf[1] = { { .buf = &tx_message, .len = sizeof(tx_message) } };
	struct spi_buf_set tx = {
		.buffers = buf,
		.count = ARRAY_SIZE(buf),
	};

	if (k_is_in_isr()) {
		/* Prevent SPI transactions from an ISR */
		return -EWOULDBLOCK;
	}

	return spi_write_dt(&config->bus, &tx);
}

static int dac6311_write_value(const struct device *dev, uint8_t channel, uint32_t value)
{
	const struct dac6311_config *config = dev->config;
	struct dac6311_data *data = dev->data;
	int ret;

	if (channel != 0) {
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

	if (value != 0) {
		ret = dac6311_reg_write(dev, value, DAC6311_NORMAL_MODE);
	} else {
		ret = dac6311_reg_write(dev, value, DAC6311_TO1KOHM_MODE);
	}
	if (ret) {
		return -EIO;
	}

	return 0;
}

static int dac6311_channel_setup(const struct device *dev,
				 const struct dac_channel_cfg *channel_cfg)
{
	const struct dac6311_config *config = dev->config;
	struct dac6311_data *data = dev->data;

	if (channel_cfg->channel_id != 0) {
		LOG_ERR("Unsupported channel %d", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	if (channel_cfg->resolution != config->resolution) {
		LOG_ERR("Unsupported resolution %d", channel_cfg->resolution);
		return -ENOTSUP;
	}

	data->configured |= BIT(channel_cfg->channel_id);
	dac6311_write_value(dev, 0, DAC6311_TO1KOHM_MODE);
	return 0;
}

static int dac6311_init(const struct device *dev)
{
	const struct dac6311_config *config = dev->config;
	struct dac6311_data *data = dev->data;
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

static const struct dac_driver_api dac6311_driver_api = {
	.channel_setup = dac6311_channel_setup,
	.write_value = dac6311_write_value,
};

#define DAC_DAC6311_INIT(num)								    \
	static struct dac6311_data dac6311_data_##num;					    \
	static const struct dac6311_config dac6311_config_##num = {			    \
		.bus = SPI_DT_SPEC_INST_GET(num,					    \
					    SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |	    \
					    SPI_WORD_SET(8) | SPI_MODE_CPHA,		    \
					    0),						    \
		.resolution = 10,							    \
	};										    \
	DEVICE_DT_INST_DEFINE(num, &dac6311_init, NULL, &dac6311_data_##num,		    \
			      &dac6311_config_##num, POST_KERNEL, CONFIG_DAC_INIT_PRIORITY, \
			      &dac6311_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DAC_DAC6311_INIT);
