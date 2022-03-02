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
#define CHANNEL_SPACING 0xC0
#define MAN_CH0_SEL 0xC0
#define MAC_CH1_SEL 0xC4
#define MAC_CH2_SEL 0xC8
#define MAC_CH3_SEL 0xCC
#define MAC_CH4_SEL 0xD0
#define MAC_CH5_SEL 0xD4
#define MAC_CH6_SEL 0xD8
#define MAC_CH7_SEL 0xDC
#define MAC_CH_AUX_SEL 0xE0

// Program registers R/W
#define WR 0x01
#define RD 0x00

// program registers address
#define AUTO_SEQ_EN 0x01
#define CH_POWER_DOWN 0x02
#define FEATURES_SELECT 0x03
#define CH_0_IR 0x05
#define CH_1_IR 0x06
#define CH_2_IR 0x07
#define CH_3_IR 0x08
#define CH_4_IR 0x09
#define CH_5_IR 0x0A
#define CH_6_IR 0x0B
#define CH_7_IR 0x0C
#define ALARM_Overview_TF 0x10
#define ALARM_ch_0_3_TF 0x11
#define ALARM_ch_0_3_AF 0x12
#define ALARM_ch_4_7_TF 0x13
#define ALARM_ch_4_7_AF 0x14

// channels range
enum ADS8698_ranges {
  RANGE_CH_PLUSMINUS_2_5 = 0x00,
  RANGE_CH_PLUSMINUS_1_25 = 0x01,
  RANGE_CH_PLUSMINUS_0_625 = 0x02,
  RANGE_CH_PLUS2_5 = 0x05,
  RANGE_CH_PLUS1_25 = 0x06
};

typedef int (*adc_ads869x_value_func)(const struct device* dev,
                                      unsigned int chan, void* data,
                                      uint32_t* result);

struct adc_ads869x_chan_cfg {
  /** Pointer to function used to obtain input mV */
  adc_ads869x_value_func func;
  /** Pointer to data that are passed to @a func on call */
  void* func_data;
  /** Constant mV input value */
  uint32_t const_value;
  /** Gain used on output value */
  enum adc_gain gain;
  /** Reference source */
  enum adc_reference ref;
};

struct adc_ads869x_config {
  struct spi_dt_spec bus;
  uint8_t channels;
};

struct adc_ads869x_data {
  /** Structure that handle state of ongoing read operation */
  struct adc_context ctx;
  /** Pointer to ADC emulator own device structure */
  const struct device* dev;
  /** Pointer to memory where next sample will be written */
  uint32_t* buf;
  /** Pointer to where will be data stored in case of repeated sampling */
  uint32_t* repeat_buf;
  /** Mask with channels that will be sampled */
  uint32_t channels;
  /** Mask created from requested resolution in read operation */
  uint16_t ref_ext0;
  /** Reference voltage for ADC_REF_INTERNAL source */
  uint16_t ref_int;
  /** Array of each channel configuration */
  struct adc_ads869x_chan_cfg* chan_cfg;
  /** Structure used for acquisition thread */
  struct k_thread thread;
  /** Semaphore used to control acquisiton thread */
  struct k_sem sem;
  /** Mutex used to control access to channels config and ref voltages */
  struct k_mutex cfg_mtx;
  uint16_t differential;
  /** Stack for acquisition thread */
  K_KERNEL_STACK_MEMBER(stack,
                        CONFIG_ADC_ADS869X_ACQUISITION_THREAD_STACK_SIZE);
};

static int ads869x_write(const struct device* dev, uint8_t* cmd_ptr,
                         size_t size);

static int ads869x_set_range(const struct device* dev, uint8_t channel,
                             enum ADS8698_ranges range) {
  uint8_t ch_wr = (channel << 1 | 0b1);
  uint8_t param[] = {ch_wr, range};
  return ads869x_write(dev, param, sizeof(param));
}

static int ads869x_channel_setup(const struct device* dev,
                                 const struct adc_channel_cfg* channel_cfg) {
  const struct adc_ads869x_config* config = dev->config;
  struct adc_ads869x_data* data = dev->data;

  if ((channel_cfg->gain != ADC_GAIN_1) && (channel_cfg->gain != ADC_GAIN_2) &&
      (channel_cfg->gain != ADC_GAIN_3 && channel_cfg->differential != 1)) {
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

  WRITE_BIT(data->differential, channel_cfg->channel_id,
            channel_cfg->differential);

  switch (channel_cfg->gain) {
    case ADC_GAIN_1:
      if (channel_cfg->differential == 1)
        ads869x_set_range(dev, channel_cfg->channel_id,
                          RANGE_CH_PLUSMINUS_0_625);
      else
        ads869x_set_range(dev, channel_cfg->channel_id, RANGE_CH_PLUS1_25);
      break;
    case ADC_GAIN_2:
      if (channel_cfg->differential == 1)
        ads869x_set_range(dev, channel_cfg->channel_id,
                          RANGE_CH_PLUSMINUS_1_25);
      else
        ads869x_set_range(dev, channel_cfg->channel_id, RANGE_CH_PLUS2_5);
      break;
    case ADC_GAIN_3:
      if (channel_cfg->differential == 1)
        ads869x_set_range(dev, channel_cfg->channel_id, RANGE_CH_PLUSMINUS_2_5);
    default:
      LOG_ERR("unsupported channel gain");
      break;
  }
  LOG_DBG("Set gain %d; differential %d; ACQ time %d", channel_cfg->gain,
          channel_cfg->differential, channel_cfg->acquisition_time);
  data->chan_cfg->gain = channel_cfg->gain;
  data->chan_cfg->ref = channel_cfg->reference;
  
  return 0;
}

#define ADS8688_PROG_REG(x) (x << 9)
#define ADS8688_PROG_REG_RANGE_CH(chan) (0x05 + chan)
#define ADS8688_PROG_WR_BIT BIT(8)
#define ADS8688_PROG_DONT_CARE_BITS 8

#define ADS8688_CMD_DONT_CARE_BITS 16
#define ADS8688_CMD_REG(x) (x << 8)
#define ADS8688_CMD_REG_MAN_CH(chan) (0xC0 | (4 * chan))

static void adc_context_update_buffer_pointer(struct adc_context* ctx,
                                              bool repeat_sampling) {
  struct adc_ads869x_data* data =
      CONTAINER_OF(ctx, struct adc_ads869x_data, ctx);

  if (repeat_sampling) {
    data->buf = data->repeat_buf;
  }
}

static int ads869x_validate_buffer_size(const struct device* dev,
                                        const struct adc_sequence* sequence) {
  const struct adc_ads869x_config* config = dev->config;
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

static int ads869x_start_read(const struct device* dev,
                              const struct adc_sequence* sequence) {
  const struct adc_ads869x_config* config = dev->config;
  struct adc_ads869x_data* data = dev->data;
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
static int ads869x_read_async(const struct device* dev,
                              const struct adc_sequence* sequence,
                              struct k_poll_signal* async) {
  struct adc_ads869x_data* data = dev->data;
  int err;

  adc_context_lock(&data->ctx, async ? true : false, async);
  err = ads869x_start_read(dev, sequence);
  adc_context_release(&data->ctx, err);

  return err;
}

static int ads869x_read(const struct device* dev,
                        const struct adc_sequence* sequence) {
  return ads869x_read_async(dev, sequence, NULL);
}

static void adc_context_start_sampling(struct adc_context* ctx) {
  struct adc_ads869x_data* data =
      CONTAINER_OF(ctx, struct adc_ads869x_data, ctx);

  data->channels = ctx->sequence.channels;
  data->repeat_buf = data->buf;

  k_sem_give(&data->sem);
}

static int ads869x_write(const struct device* dev, uint8_t* cmd_ptr,
                         size_t size) {
  const struct adc_ads869x_config* config = dev->config;
  // struct adc_ads869x_data* data = dev->data;
  // uint8_t to[size];

  int err;
  const struct spi_buf tx_buf[1] = {{.buf = cmd_ptr, .len = size}};
  const struct spi_buf_set tx = {.buffers = tx_buf,
                                 .count = ARRAY_SIZE(tx_buf)};



  // to[0] = cmd;

  err = spi_write_dt(&config->bus, &tx);
  LOG_HEXDUMP_DBG(cmd_ptr, size, "wr cmd");
  // err = spi_transceive_dt(&config->bus, &tx, &rx);
  if (err) {
    return err;
  }
  return 0;
}

int ads8698_convert_to_mv(const struct device* dev, uint8_t channel,
                          unsigned int register_value) {
  const struct adc_ads869x_config* config = dev->config;
  struct adc_ads869x_data* data = dev->data;
  // WRITE_BIT(data->differential, channel_cfg->channel_id,
  //         channel_cfg->differential);

  int bipolar = (data->differential & BIT(channel)) == BIT(channel) ? 1 : 0;

  switch (data->chan_cfg->gain) {
    case ADC_GAIN_3:
      if (bipolar) return ((register_value - 0x20000) * 20480 / 0x40000);
    case ADC_GAIN_2:
      if (bipolar)
        return ((register_value - 0x20000) * 10240 / 0x40000);
      else
        return ((register_value)*10240 / 0x40000);
    case ADC_GAIN_1:
      if (bipolar)
        return ((register_value - 0x20000) * 5120 / 0x40000);
      else
        return ((register_value)*5120 / 0x40000);
    default:
      LOG_ERR("Wrong gain, convert to mv failed");
      return (0);
  }
}

static int ads869x_read_channel(const struct device* dev, uint8_t channel,
                                uint32_t* result) {
  const struct adc_ads869x_config* config = dev->config;
  struct adc_ads869x_data* data = dev->data;
  uint8_t to[2] = {0};
  uint8_t from[3] = {0};

  int err;
  const struct spi_buf tx_buf[2] = {{.buf = to, .len = sizeof(to)},
                                    {.buf = NULL, .len = 3}};

  // uint8_t shade_from[2] = {0};
  const struct spi_buf rx_buf[2] = {{.buf = NULL, .len = 2},
                                    {.buf = from, .len = sizeof(from)}};
  const struct spi_buf_set rx = {.buffers = rx_buf,
                                 .count = ARRAY_SIZE(rx_buf)};
  const struct spi_buf_set tx = {.buffers = tx_buf,
                                 .count = ARRAY_SIZE(tx_buf)};

  uint16_t cmd =
      /*sys_cpu_to_be32(*/ ADS8688_CMD_REG_MAN_CH(channel);
  to[0] = ((uint8_t*)&cmd)[0];
  to[1] = ((uint8_t*)&cmd)[1];

  LOG_HEXDUMP_DBG(to, sizeof(to), "ADC TX");
  err = spi_transceive_dt(&config->bus, &tx, &rx);
  if (err) {
    return err;
  }
  LOG_HEXDUMP_DBG(from, sizeof(from), "ADC RX");
  // LOG_HEXDUMP_DBG(shade_from, sizeof(shade_from), "shade ADC RX");
  // *result = sys_get_be32(
  //     (unsigned int)((((unsigned int)from[2]) << 16 |
  //                     ((unsigned int)from[3]) << 8 | (unsigned int)from[4])
  //                     >>
  //                    6));
  *result = (*((uint32_t*)from)) >> 6; //>>6
  *result = ads8698_convert_to_mv(dev, channel, *result);
  LOG_DBG("reg '%d' mv '%d'", (*((uint32_t*)from)) >> 6, *result);
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
static void ads869x_acquisition_thread(struct adc_ads869x_data* data) {
  int err;

  while (true) {
    k_sem_take(&data->sem, K_FOREVER);
    int first = 1;
    err = 0;
 
    int fake_ch = find_msb_set(data->channels);
    WRITE_BIT(data->channels, fake_ch, 1);

    while (data->channels) {
      uint32_t result = 0;
      unsigned int chan = find_lsb_set(data->channels) - 1;

      LOG_DBG("reading channel %d", chan);

      err = ads869x_read_channel(data->dev, chan, &result);
      if (err) {
        adc_context_complete(&data->ctx, err);
        break;
      }

      LOG_DBG("read channel %d, result = %d", chan, result);

      WRITE_BIT(data->channels, chan, 0);
      if (first) {
        first = 0;
      } else {
        *data->buf++ = result;
      }
    }

    if (!err) {
      adc_context_on_sampling_done(&data->ctx, data->dev);
    }
  }
}

/**
 * @brief Function called on init for each ADC emulator device. It setups all
 *        channels to return constant 0 mV and create acquisition thread.
 *
 * @param dev ads869x device
 *
 * @return 0 on success
 */
static int adc_ads869x_init(const struct device* dev) {
  LOG_DBG("Try to init ads869x");

  const struct adc_ads869x_config* config = dev->config;
  struct adc_ads869x_data* data = dev->data;

  data->dev = dev;

  k_sem_init(&data->sem, 0, 1);

  LOG_DBG("slave - %d; freq - %d; name - %s", config->bus.config.slave,
          config->bus.config.frequency, config->bus.bus->name);

  if (!spi_is_ready(&config->bus)) {
    LOG_ERR("SPI bus is not ready");
    return -ENODEV;
  }
  LOG_DBG("SPI is ready, creating thread");
  uint8_t rst_cmd = RST;
  ads869x_write(data->dev, &rst_cmd, 1);
  k_thread_create(&data->thread, data->stack,
                  CONFIG_ADC_ADS869X_ACQUISITION_THREAD_STACK_SIZE,
                  (k_thread_entry_t)ads869x_acquisition_thread, data, NULL,
                  NULL, CONFIG_ADC_ADS869X_ACQUISITION_THREAD_PRIO, 0,
                  K_NO_WAIT);

  adc_context_unlock_unconditionally(&data->ctx);

  return 0;
}
// #define INST_DT_ADS8698(n, t) DT_INST(n, ti_ads8698_adc)

#define ADC_ADS869X_INIT(_num)                                                \
  static struct adc_driver_api adc_ads869x_api_##_num = {                     \
      .channel_setup = ads869x_channel_setup,                                 \
      .read = ads869x_read,                                                   \
      .ref_internal = 0,                                                      \
      IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = ads869x_read_async, ))};    \
                                                                              \
  static struct adc_ads869x_chan_cfg adc_ads869x_ch_cfg_##_num[9];            \
                                                                              \
  static const struct adc_ads869x_config adc_ads869x_config_##_num = {        \
      .bus = SPI_DT_SPEC_INST_GET(                                            \
          _num, SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8), 0),  \
      .channels = 9,                                                          \
  };                                                                          \
                                                                              \
  static struct adc_ads869x_data adc_ads869x_data_##_num = {                  \
      ADC_CONTEXT_INIT_TIMER(adc_ads869x_data_##_num, ctx),                   \
      ADC_CONTEXT_INIT_LOCK(adc_ads869x_data_##_num, ctx),                    \
      ADC_CONTEXT_INIT_SYNC(adc_ads869x_data_##_num, ctx),                    \
      .chan_cfg = adc_ads869x_ch_cfg_##_num,                                  \
      .ref_int = 0,                                                           \
  };                                                                          \
                                                                              \
  DEVICE_DT_INST_DEFINE(_num, adc_ads869x_init, NULL,                         \
                        &adc_ads869x_data_##_num, &adc_ads869x_config_##_num, \
                        POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,                \
                        &adc_ads869x_api_##_num);
DT_INST_FOREACH_STATUS_OKAY(ADC_ADS869X_INIT);

// static struct adc_driver_api adc_ads869x_api_0_num = {
//     .channel_setup = ads869x_channel_setup,
//     .read = ads869x_read,
//     .ref_internal = 0,
//     IF_ENABLED(CONFIG_ADC_ASYNC, (.read_async = ads869x_read_async, ))};

// static struct adc_ads869x_chan_cfg adc_ads869x_ch_cfg_0_num[9];

// static const struct adc_ads869x_config adc_ads869x_config_0_num = {
//     .bus = SPI_DT_SPEC_INST_GET(0,
//                            SPI_OP_MODE_MASTER | SPI_FRAME_FORMAT_TI |
//                                SPI_TRANSFER_MSB | SPI_WORD_SET(8),
//                            0),
//     .channels = 9,
// };

// static struct adc_ads869x_data adc_ads869x_data_0_num = {
//     ADC_CONTEXT_INIT_TIMER(adc_ads869x_data_0_num, ctx),
//     ADC_CONTEXT_INIT_LOCK(adc_ads869x_data_0_num, ctx),
//     ADC_CONTEXT_INIT_SYNC(adc_ads869x_data_0_num, ctx),
//     .chan_cfg = adc_ads869x_ch_cfg_0_num,
//     .ref_int = 0,
// };

// // INST_DT_ADS8698(0, 0)

// DEVICE_DT_INST_DEFINE(0, adc_ads869x_init, NULL, &adc_ads869x_data_0_num,
//                       &adc_ads869x_config_0_num, POST_KERNEL,
//                       CONFIG_ADC_INIT_PRIORITY, &adc_ads869x_api_0_num);
