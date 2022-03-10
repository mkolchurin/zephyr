#include <drivers/adc.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <kernel.h>
#include <logging/log.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <zephyr.h>

#define DT_DRV_COMPAT ti_ads8698_adc

LOG_MODULE_REGISTER(adc_ads869x, CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include <stdlib.h>

#include "adc_context.h"
#define ADC_RESOLUTION 18

// Command Register
#define NO_OP 0x00
#define STDBY 0x82
#define PWR_DN 0x83
#define RST 0x85
#define AUTO_RST 0xA0
// Program registers R/W
#define WR 0x01
#define RD 0x00

// program registers address
#define AUTO_SEQ_EN 0x01
#define CH_POWER_DOWN 0x02
#define FEATURES_SELECT 0x03
#define ALARM_Overview_TF 0x10
#define ALARM_ch_0_3_TF 0x11
#define ALARM_ch_0_3_AF 0x12
#define ALARM_ch_4_7_TF 0x13
#define ALARM_ch_4_7_AF 0x14

#define ADS8688_PROG_REG(x) (x << 9)
#define ADS8688_PROG_WR_BIT BIT(8)

#define ADS8688_CMD_REG(x) (x << 8)
#define ADS8688_CMD_REG_NOOP 0x00
#define ADS8688_CMD_REG_RST 0x85
#define ADS8688_CMD_REG_MAN_CH(chan) (0xC0 | (4 * chan))
#define ADS8688_CMD_DONT_CARE_BITS 16

#define ADS8688_PROG_REG(x) (x << 9)
#define ADS8688_PROG_REG_RANGE_CH(chan) (0x05 + chan)
#define ADS8688_PROG_WR_BIT BIT(8)
#define ADS8688_PROG_DONT_CARE_BITS 8

// #define ADS8698_AUX_CH 8
// channels range
enum ADS8698_ranges {
	RANGE_CH_PLUSMINUS_2_5 = 0b0,
	RANGE_CH_PLUSMINUS_1_25 = 0b1,
	RANGE_CH_PLUSMINUS_0_625 = 0b10,
	RANGE_CH_PLUS2_5 = 0b101,
	RANGE_CH_PLUS1_25 = 0b0110
};

typedef int (*adc_ads869x_value_func)(const struct device *dev, unsigned int chan, void *data,
				      uint32_t *result);

struct adc_ads869x_chan_cfg {
	/** Pointer to function used to obtain input mV */
	adc_ads869x_value_func func;
	/** Pointer to data that are passed to @a func on call */
	void *func_data;
	/** Constant mV input value */
	uint32_t const_value;
	/** Gain used on output value */
	enum adc_gain gain;
	/** Reference source */
	enum adc_reference ref;
};

struct adc_ads869x_config {
	struct spi_dt_spec bus;
	uint16_t channels;
};

struct adc_ads869x_data {
	/** Structure that handle state of ongoing read operation */
	struct adc_context ctx;
	/** Pointer to ADC emulator own device structure */
	const struct device *dev;
	/** Pointer to memory where next sample will be written */
	uint32_t *buf;
	/** Pointer to where will be data stored in case of repeated sampling */
	uint32_t *repeat_buf;
	/** Mask with channels that will be sampled */
	uint32_t channels;
	/** Mask created from requested resolution in read operation */
	uint16_t ref_ext0;
	/** Reference voltage for ADC_REF_INTERNAL source */
	uint16_t ref_int;
	/** Array of each channel configuration */
	struct adc_ads869x_chan_cfg *chan_cfg;
	/** Structure used for acquisition thread */
	struct k_thread thread;
	/** Semaphore used to control acquisiton thread */
	struct k_sem sem;
	/** Mutex used to control access to channels config and ref voltages */
	struct k_mutex cfg_mtx;
	uint16_t differential;
	/** Stack for acquisition thread */
	K_KERNEL_STACK_MEMBER(stack, CONFIG_ADC_ADS869X_ACQUISITION_THREAD_STACK_SIZE);
};

static int ads869x_command_reg(const struct device *dev, uint8_t command);
static int ads869x_prog_reg(const struct device *dev, uint8_t addr, uint8_t command, bool rw_bit);

static int ads869x_set_range(const struct device *dev, uint8_t channel, enum ADS8698_ranges range)
{
	uint8_t ch = ADS8688_PROG_REG_RANGE_CH(channel);

	int err = ads869x_prog_reg(dev, ch, range, 1);
	if (err) {
		LOG_ERR("Set range command failed %d", err);
	}
	return 0;
}

static int ads869x_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	const struct adc_ads869x_config *config = dev->config;
	struct adc_ads869x_data *data = dev->data;

	if ((channel_cfg->gain != ADC_GAIN_4) && (channel_cfg->gain != ADC_GAIN_2) &&
	    (channel_cfg->gain != ADC_GAIN_1 || channel_cfg->differential != 1)) {
		LOG_ERR("unsupported channel gain %d", channel_cfg->gain);
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_EXTERNAL0 &&
	    channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("unsupported channel reference '%d'", channel_cfg->reference);
		return -ENOTSUP;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("unsupported acquisition_time '%d'", channel_cfg->acquisition_time);
		return -ENOTSUP;
	}

	if (channel_cfg->channel_id >= config->channels) {
		LOG_ERR("unsupported channel id '%d'", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	WRITE_BIT(data->differential, channel_cfg->channel_id, channel_cfg->differential);

	// if(channel_cfg->channel_id < 8){
		switch (channel_cfg->gain) {
		case ADC_GAIN_4:
			if (channel_cfg->differential == 1)
				ads869x_set_range(dev, channel_cfg->channel_id, RANGE_CH_PLUSMINUS_0_625);
			else
				ads869x_set_range(dev, channel_cfg->channel_id, RANGE_CH_PLUS1_25);
			break;
		case ADC_GAIN_2:
			if (channel_cfg->differential == 1)
				ads869x_set_range(dev, channel_cfg->channel_id, RANGE_CH_PLUSMINUS_1_25);
			else
				ads869x_set_range(dev, channel_cfg->channel_id, RANGE_CH_PLUS2_5);
			break;
		case ADC_GAIN_1:
			if (channel_cfg->differential == 1)
				ads869x_set_range(dev, channel_cfg->channel_id, RANGE_CH_PLUSMINUS_2_5);
		default:
			LOG_ERR("unsupported channel gain");
			break;
		}
	// }
	LOG_DBG("Set gain %d; differential %d; ACQ time %d", channel_cfg->gain,
		channel_cfg->differential, channel_cfg->acquisition_time);

	data->chan_cfg->gain = channel_cfg->gain;
	data->chan_cfg->ref = channel_cfg->reference;

	return 0;
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_ads869x_data *data = CONTAINER_OF(ctx, struct adc_ads869x_data, ctx);

	if (repeat_sampling) {
		data->buf = data->repeat_buf;
	}
}

static int ads869x_validate_buffer_size(const struct device *dev,
					const struct adc_sequence *sequence)
{
	const struct adc_ads869x_config *config = dev->config;
	uint8_t channels = 0;
	size_t needed;
	uint32_t mask;

	for (mask = BIT(config->channels - 1); mask != 0; mask >>= 1) {
		if (mask & sequence->channels) {
			channels++;
		}
	}

	needed = channels * sizeof(uint32_t);
	if (sequence->options) {
		needed *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static int ads869x_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	const struct adc_ads869x_config *config = dev->config;
	struct adc_ads869x_data *data = dev->data;
	int err;

	if (sequence->resolution != ADC_RESOLUTION) {
		LOG_ERR("unsupported resolution %d", sequence->resolution);
		return -ENOTSUP;
	}

	if (find_msb_set(sequence->channels) > config->channels) {
		LOG_ERR("unsupported channels in mask: 0x%08x", sequence->channels);
		return -ENOTSUP;
	}

	err = ads869x_validate_buffer_size(dev, sequence);
	if (err) {
		LOG_ERR("buffer size too small");
		return err;
	}

	data->buf = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}
static int ads869x_read_async(const struct device *dev, const struct adc_sequence *sequence,
			      struct k_poll_signal *async)
{
	struct adc_ads869x_data *data = dev->data;
	int err;

	adc_context_lock(&data->ctx, async ? true : false, async);
	err = ads869x_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

static int ads869x_read(const struct device *dev, const struct adc_sequence *sequence)
{
	return ads869x_read_async(dev, sequence, NULL);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_ads869x_data *data = CONTAINER_OF(ctx, struct adc_ads869x_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buf = data->buf;

	k_sem_give(&data->sem);
}

/**
 * Succsess - 0 if write; answer if read; error code elsewise
 * 
 */
static int ads869x_prog_reg(const struct device *dev, uint8_t addr, uint8_t command, bool rw_bit)
{
	const struct adc_ads869x_config *config = dev->config;
	uint32_t rw = rw_bit ? ADS8688_PROG_WR_BIT : 0;
	uint32_t tmp = ADS8688_PROG_REG(addr) | rw | command;
	tmp <<= 8;
	uint32_t tx = sys_cpu_to_be32(tmp);

	const struct spi_buf tx_buf[1] = { { .buf = &(((uint8_t *)&tx)[1]), .len = 3 } };
	const struct spi_buf_set tx_buf_set = { .buffers = tx_buf, .count = ARRAY_SIZE(tx_buf) };
	int err = spi_write_dt(&config->bus, &tx_buf_set);
	if (err)
		return -err;
	return 0;
}
static int ads869x_command_reg(const struct device *dev, uint8_t command)
{
	const struct adc_ads869x_config *config = dev->config;

	uint32_t cmd = command << 8;
	cmd <<= 16;
	cmd = sys_cpu_to_be32(cmd);

	int err;
	const struct spi_buf tx_buf[1] = { { .buf = (uint8_t *)&cmd, .len = 4 } };
	const struct spi_buf_set tx = { .buffers = tx_buf, .count = ARRAY_SIZE(tx_buf) };

	err = spi_write_dt(&config->bus, &tx);
	LOG_HEXDUMP_DBG((uint8_t *)&cmd, 2, "wr cmd");
	if (err) {
		LOG_ERR("wr cmd failed %d", err);
		return err;
	}
	return 0;
}

int ads8698_convert_to_mv(const struct device *dev, uint8_t channel, unsigned int register_value)
{
	const struct adc_ads869x_config *config = dev->config;
	struct adc_ads869x_data *data = dev->data;
	int bipolar = (data->differential & BIT(channel)) == BIT(channel) ? 1 : 0;
	switch (data->chan_cfg->gain) {
	case ADC_GAIN_1:
		if (bipolar)
			return ((WRITE_BIT(register_value, 17, 0) * 5120) >> 18); // 0x40000);
		else
			return ((register_value * 5120) >> 18);
	case ADC_GAIN_2:
		if (bipolar)
			return ((WRITE_BIT(register_value, 17, 0) * 10240) >> 18);
		else
			return ((register_value * 10240) >> 18);
	case ADC_GAIN_3:
		if (bipolar)
			return ((WRITE_BIT(register_value, 17, 0) * 20480) >> 18);
	default:
		LOG_ERR("Wrong gain, convert to mv failed");
		return (0);
	}
}

static int ads869x_read_channel(const struct device *dev, uint8_t channel, uint32_t *result)
{
	const struct adc_ads869x_config *config = dev->config;
	struct adc_ads869x_data *data = dev->data;
	union tx_rx_t {
		uint8_t d8[5];
		uint32_t d32;
	};
	union tx_rx_t t[2];

	int tmp = ADS8688_CMD_REG(ADS8688_CMD_REG_MAN_CH(channel));
	tmp <<= ADS8688_CMD_DONT_CARE_BITS;
	t[0].d32 = sys_cpu_to_be32(tmp);

	tmp = ADS8688_CMD_REG(ADS8688_CMD_REG_NOOP);
	tmp <<= ADS8688_CMD_DONT_CARE_BITS;
	t[1].d32 = sys_cpu_to_be32(tmp);
	LOG_HEXDUMP_DBG(&t[0].d8, 5, "ADC TX");

	struct spi_buf tx_buf[1] = { { .buf = &(t[0].d8[0]), .len = 5 } };
	struct spi_buf_set tx = { .buffers = tx_buf, .count = ARRAY_SIZE(tx_buf) };
	int err;

	err = spi_write_dt(&config->bus, &tx);

	tx_buf[0].buf = NULL; 
	tx_buf[0].len = 5;
	const struct spi_buf rx_buf[] = { { .buf = &(t[1].d8[0]), .len = 5 } };
	const struct spi_buf_set rx = { .buffers = rx_buf, .count = ARRAY_SIZE(rx_buf) };
	tx.buffers = tx_buf; tx.count = ARRAY_SIZE(tx_buf);

	err = spi_transceive_dt(&config->bus, &tx, &rx);
	
	if (err) {
		return err;
	}
	*result = (unsigned int) ((((unsigned int) t[1].d8[2]) << 16
			| ((unsigned int) t[1].d8[3]) << 8 | (unsigned int) t[1].d8[4]) >> 6);
	
	LOG_HEXDUMP_DBG(&t[1].d8, 5, "ADC RX");
	return 0;
}

/**
 * @brief Main function of thread which is used to collect samples from
 *        emulated ADC. When adc_context_start_sampling give semaphore,
 *        for each requested channel value function is called. Returned
 *        mV value is converted to output using reference voltage, gain
 *        and requested resolution.
 *
 * @param data Internal data of ADC emulator
 *
 * @return This thread should not end
 */
static void ads869x_acquisition_thread(struct adc_ads869x_data *data)
{
	int err;

	while (true) {
		k_sem_take(&data->sem, K_FOREVER);
		err = 0;
		while (data->channels) {
			uint32_t result = 0;
			unsigned int chan = find_lsb_set(data->channels) - 1;
			err = ads869x_read_channel(data->dev, chan, &result);
			if (err) {
				adc_context_complete(&data->ctx, err);
				break;
			}
			LOG_DBG("read channel %d, result = %d", chan, result);
			WRITE_BIT(data->channels, chan, 0);
			*data->buf++ = result;
		}

		if (!err) {
			adc_context_on_sampling_done(&data->ctx, data->dev);
		}
	}
}

/**
 * @brief Function called on init for each ADC ADS8698 device. It setups all
 *        channels to return constant 0 mV and create acquisition thread.
 *
 * @param dev ads869x device
 *
 * @return 0 on success
 */
static int adc_ads869x_init(const struct device *dev)
{
	LOG_DBG("Try to init ads869x");

	const struct adc_ads869x_config *config = dev->config;
	struct adc_ads869x_data *data = dev->data;

	data->dev = dev;

	k_sem_init(&data->sem, 0, 1);

	LOG_DBG("slave - %d; freq - %d; name - %s", config->bus.config.slave,
		config->bus.config.frequency, config->bus.bus->name);

	if (!spi_is_ready(&config->bus)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}
	LOG_DBG("SPI is ready, creating thread");

	ads869x_command_reg(data->dev, RST);
	ads869x_command_reg(data->dev, ADS8688_CMD_REG_MAN_CH(0));

	k_thread_create(&data->thread, data->stack,
			CONFIG_ADC_ADS869X_ACQUISITION_THREAD_STACK_SIZE,
			(k_thread_entry_t)ads869x_acquisition_thread, data, NULL, NULL,
			CONFIG_ADC_ADS869X_ACQUISITION_THREAD_PRIO, 0, K_NO_WAIT);

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define ADC_ADS869X_INIT(_num)                                                                     \
	static struct adc_driver_api adc_ads869x_api_##_num = {                                    \
		.channel_setup = ads869x_channel_setup,                                            \
		.read = ads869x_read,                                                              \
		.ref_internal = 20480,                                                             \
		IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = ads869x_read_async, ))                 \
	};                                                                                         \
                                                                                                   \
	static struct adc_ads869x_chan_cfg adc_ads869x_ch_cfg_##_num[9];                           \
                                                                                                   \
	static const struct adc_ads869x_config adc_ads869x_config_##_num = {                       \
		.bus = SPI_DT_SPEC_INST_GET(_num,                                                  \
					    SPI_OP_MODE_MASTER | SPI_MODE_CPHA |                   \
						    SPI_TRANSFER_MSB | SPI_WORD_SET(8),            \
					    0),                                                    \
		.channels = 9,                                                                     \
	};                                                                                         \
                                                                                                   \
	static struct adc_ads869x_data adc_ads869x_data_##_num = {                                 \
		ADC_CONTEXT_INIT_TIMER(adc_ads869x_data_##_num, ctx),                              \
		ADC_CONTEXT_INIT_LOCK(adc_ads869x_data_##_num, ctx),                               \
		ADC_CONTEXT_INIT_SYNC(adc_ads869x_data_##_num, ctx),                               \
		.chan_cfg = adc_ads869x_ch_cfg_##_num,                                             \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(_num, adc_ads869x_init, NULL, &adc_ads869x_data_##_num,              \
			      &adc_ads869x_config_##_num, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,   \
			      &adc_ads869x_api_##_num);
DT_INST_FOREACH_STATUS_OKAY(ADC_ADS869X_INIT);