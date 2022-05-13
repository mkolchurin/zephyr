/*
 * Copyright (c) 2017 Intel Corporation
 * Copyright (c) 2018 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_APDS9960_APDS9960_H_
#define ZEPHYR_DRIVERS_SENSOR_APDS9960_APDS9960_H_

#include <zephyr/drivers/gpio.h>

#define APDS9960_ENABLE_REG		0x80
#define APDS9960_ENABLE_GEN		BIT(6)
#define APDS9960_ENABLE_PIEN		BIT(5)
#define APDS9960_ENABLE_AIEN		BIT(4)
#define APDS9960_ENABLE_WEN		BIT(3)
#define APDS9960_ENABLE_PEN		BIT(2)
#define APDS9960_ENABLE_AEN		BIT(1)
#define APDS9960_ENABLE_PON		BIT(0)

#define APDS9960_ATIME_REG		0x81
#define APDS9960_WTIME_REG		0x83
#define APDS9960_INT_AILTL_REG		0x84
#define APDS9960_INT_AILTH_REG		0x85
#define APDS9960_INT_AIHTL_REG		0x86
#define APDS9960_INT_AIHTH_REG		0x87
#define APDS9960_PILT_REG		0x89
#define APDS9960_PIHT_REG		0x8B

#define APDS9960_PERS_REG		0x8C
#define APDS9960_PERS_PPERS		(BIT(4) | BIT(5) | BIT(6) | BIT(7))
#define APDS9960_APERS_MASK		(BIT(0) | BIT(1) | BIT(2) | BIT(3))

#define APDS9960_CONFIG1_REG		0x8D
#define APDS9960_CONFIG1_WLONG		BIT(1)

#define APDS9960_PPULSE_REG		0x8E
#define APDS9960_PPULSE_LENGTH_4US	0
#define APDS9960_PPULSE_LENGTH_8US	BIT(6)
#define APDS9960_PPULSE_LENGTH_16US	BIT(7)
#define APDS9960_PPULSE_LENGTH_32US	(BIT(7) | BIT(6))

#define APDS9960_CONTROL_REG		0x8F
#define APDS9960_CONTROL_LDRIVE		(BIT(6) | BIT(7))
#define APDS9960_CONTROL_PGAIN		(BIT(3) | BIT(2))
#define APDS9960_CONTROL_AGAIN		(BIT(0) | BIT(1))
/* LED Drive values */
#define APDS9960_LED_DRIVE_100MA	0
#define APDS9960_LED_DRIVE_50MA		BIT(6)
#define APDS9960_LED_DRIVE_25MA		BIT(7)
#define APDS9960_LED_DRIVE_12_5MA	(BIT(6) | BIT(7))
/* Proximity Gain (PGAIN) values */
#define APDS9960_PGAIN_1X		0
#define APDS9960_PGAIN_2X		BIT(2)
#define APDS9960_PGAIN_4X		BIT(3)
#define APDS9960_PGAIN_8X		(BIT(2) | BIT(3))
/* ALS Gain (AGAIN) values */
#define APDS9960_AGAIN_1X		0
#define APDS9960_AGAIN_4X		BIT(0)
#define APDS9960_AGAIN_16X		BIT(1)
#define APDS9960_AGAIN_64X		(BIT(0) | BIT(1))

#define APDS9960_CONFIG2_REG		0x90
#define APDS9960_CONFIG2_CPSIEN		BIT(6)
#define APDS9960_CONFIG2_PSIEN		BIT(7)
/* LED Boost values */
#define APDS9960_PLED_BOOST_100		0
#define APDS9960_PLED_BOOST_150		BIT(4)
#define APDS9960_PLED_BOOST_200		BIT(5)
#define APDS9960_PLED_BOOST_300		(BIT(5) | BIT(4))

#define APDS9960_ID_REG			0x92
/* Acceptable device IDs */
#define APDS9960_ID_1			0xAB
#define APDS9960_ID_2			0x9C

#define APDS9960_STATUS_REG		0x93
#define APDS9960_STATUS_CPSAT		BIT(7)
#define APDS9960_STATUS_PGSAT		BIT(6)
#define APDS9960_STATUS_PINT		BIT(5)
#define APDS9960_STATUS_AINT		BIT(4)
#define APDS9960_STATUS_GINT		BIT(2)
#define APDS9960_STATUS_PVALID		BIT(1)
#define APDS9960_STATUS_AVALID		BIT(0)

#define APDS9960_CDATAL_REG		0x94
#define APDS9960_CDATAH_REG		0x95
#define APDS9960_RDATAL_REG		0x96
#define APDS9960_RDATAH_REG		0x97
#define APDS9960_GDATAL_REG		0x98
#define APDS9960_GDATAH_REG		0x99
#define APDS9960_BDATAL_REG		0x9A
#define APDS9960_BDATAH_REG		0x9B
#define APDS9960_PDATA_REG		0x9C

#define APDS9960_POFFSET_UR_REG		0x9D
#define APDS9960_POFFSET_DL_REG		0x9E

#define APDS9960_CONFIG3_REG		0x9F
#define APDS9960_CONFIG3_PCMP		BIT(5)
#define APDS9960_CONFIG3_SAI		BIT(4)
#define APDS9960_CONFIG3_PMSK_U		BIT(3)
#define APDS9960_CONFIG3_PMSK_D		BIT(2)
#define APDS9960_CONFIG3_PMSK_L		BIT(1)
#define APDS9960_CONFIG3_PMSK_R		BIT(0)

#define APDS9960_GPENTH_REG		0xA0
#define APDS9960_GEXTH_REG		0xA1

#define APDS9960_GCONFIG1_REG		0xA2
#define APDS9960_GCONFIG1_GFIFOTH	(BIT(7) | BIT(6))
#define APDS9960_GCONFIG1_GEXMSK	(BIT(5) | BIT(4) | BIT(3) | BIT(2))
#define APDS9960_GCONFIG1_GEXPERS	(BIT(1) | BIT(0))

#define APDS9960_GCONFIG2_REG		0xA3
#define APDS9960_GCONFIG2_GGAIN		(BIT(6) | BIT(5))
#define APDS9960_GCONFIG2_GLDRIVE	(BIT(4) | BIT(3))
#define APDS9960_GCONFIG2_WTIME		(BIT(2) | BIT(1) | BIT(0))
/* Gesture Gain (GGAIN) values */
#define APDS9960_GGAIN_1X		0
#define APDS9960_GGAIN_2X		BIT(5)
#define APDS9960_GGAIN_4X		BIT(6)
#define APDS9960_GGAIN_8X		(BIT(6) | BIT(5))
/* Gesture LED Drive Strength values */
#define APDS9960_LED_GDRIVE_100MA	0
#define APDS9960_LED_GDRIVE_50MA	BIT(3)
#define APDS9960_LED_GDRIVE_25MA	BIT(4)
#define APDS9960_LED_GDRIVE_12_5MA	(BIT(4) | BIT(3))
/* Gesture wait time values */
#define APDS9960_GWTIME_0MS		0
#define APDS9960_GWTIME_2_8MS		1
#define APDS9960_GWTIME_5_6MS		2
#define APDS9960_GWTIME_8_4MS		3
#define APDS9960_GWTIME_14_0MS		4
#define APDS9960_GWTIME_22_4MS		5
#define APDS9960_GWTIME_30_8MS		6
#define APDS9960_GWTIME_39_2MS		7

#define APDS9960_GOFFSET_U_REG		0xA4
#define APDS9960_GOFFSET_D_REG		0xA5
#define APDS9960_GOFFSET_L_REG		0xA7
#define APDS9960_GOFFSET_R_REG		0xA9

#define APDS9960_GPULSE_REG		0xA6
#define APDS9960_GPULSE_GPLEN		(BIT(7) | BIT(6))
#define APDS9960_GPULSE_GPULSE		(BIT(5) | BIT(4) | BIT(3) |\
					BIT(2) | BIT(1) | BIT(0))
/* Gesture Pulse Length values */
#define APDS9960_GPLEN_0US		0
#define APDS9960_GPLEN_8US		BIT(6)
#define APDS9960_GPLEN_16US		BIT(7)
#define APDS9960_GPLEN_32US		(BIT(7) | BIT(6))

#define APDS9960_GCONFIG3_REG		0xAA
#define APDS9960_GCONFIG3_GDIMS		(BIT(1) | BIT(0))

/* Gesture Registers */
#define APDS9960_GCONFIG4_REG		0xAB
#define APDS9960_GCONFIG4_GFIFO_CLR	BIT(2)
#define APDS9960_GCONFIG4_GIEN		BIT(1)
#define APDS9960_GCONFIG4_GMODE		BIT(0)

#define APDS9960_GFLVL_REG		0xAE

#define APDS9960_GSTATUS_REG		0xAF
#define APDS9960_GSTATUS_GFOV		BIT(1)
#define APDS9960_GSTATUS_GVALID		BIT(0)

#define APDS9960_IFORCE_REG		0xE4
#define APDS9960_PICLEAR_REG		0xE5
#define APDS9960_CICLEAR_REG		0xE6
#define APDS9960_AICLEAR_REG		0xE7

#define APDS9960_GFIFO_U_REG		0xFC
#define APDS9960_GFIFO_D_REG		0xFD
#define APDS9960_GFIFO_L_REG		0xFE
#define APDS9960_GFIFO_R_REG		0xFF

/* Default values */
#define APDS9960_DEFAULT_ATIME		219
#define APDS9960_DEFAULT_WTIME		255
#define APDS9960_DEFAULT_CONFIG1	0x60
#define APDS9960_DEFAULT_PERS		BIT(4)
#define APDS9960_DEFAULT_CONFIG2	(BIT(6) | BIT(0))
#define APDS9960_DEFAULT_GESTURE_PPULSE	0x89
#define APDS9960_DEFAULT_POFFSET_UR	0
#define APDS9960_DEFAULT_POFFSET_DL	0
#define APDS9960_DEFAULT_LDRIVE		APDS9960_LED_DRIVE_100MA

#ifdef CONFIG_APDS9960_TRIGGER
#define APDS9960_DEFAULT_PILT		0
#define APDS9960_DEFAULT_PIHT		50
#define APDS9960_DEFAULT_AILT		10
#define APDS9960_DEFAULT_AIHT		1000
#define APDS9960_DEFAULT_CONFIG3	0
#else
#define APDS9960_DEFAULT_PILT		0
#define APDS9960_DEFAULT_PIHT		1
#define APDS9960_DEFAULT_AILT		0xFFFF
#define APDS9960_DEFAULT_AIHT		0
#define APDS9960_DEFAULT_CONFIG3	APDS9960_CONFIG3_SAI
#endif

#define APDS9960_DEFAULT_GPENTH		40
#define APDS9960_DEFAULT_GEXTH		30
#define APDS9960_DEFAULT_GCONF1		0x40
#define APDS9960_DEFAULT_GGAIN		APDS9960_GGAIN_4X
#define APDS9960_DEFAULT_GLDRIVE	APDS9960_LED_DRIVE_100MA
#define APDS9960_DEFAULT_GWTIME		APDS9960_GWTIME_2_8MS
#define APDS9960_DEFAULT_GOFFSET	0
#define APDS9960_DEFAULT_GPULSE		0xC9
#define APDS9960_DEFAULT_GCONF3		0

struct apds9960_config {
	char *i2c_name;
	char *gpio_name;
	uint8_t gpio_pin;
	unsigned int gpio_flags;
	uint8_t i2c_address;
	uint8_t pgain;
	uint8_t again;
	uint8_t ppcount;
	uint8_t pled_boost;
};

struct apds9960_data {
	const struct device *i2c;
	const struct device *gpio;
	struct gpio_callback gpio_cb;
	struct k_work work;
	const struct device *dev;
	uint16_t sample_crgb[4];
	uint8_t pdata;
	uint8_t gpio_pin;

#ifdef CONFIG_APDS9960_TRIGGER
	sensor_trigger_handler_t p_th_handler;
	struct sensor_trigger p_th_trigger;
#else
	struct k_sem data_sem;
#endif
};

static inline void apds9960_setup_int(struct apds9960_data *drv_data,
				      bool enable)
{
	unsigned int flags = enable
		? GPIO_INT_EDGE_TO_ACTIVE
		: GPIO_INT_DISABLE;

	gpio_pin_interrupt_configure(drv_data->gpio,
				     drv_data->gpio_pin,
				     flags);
}

#ifdef CONFIG_APDS9960_TRIGGER
void apds9960_work_cb(struct k_work *work);

int apds9960_attr_set(const struct device *dev,
		      enum sensor_channel chan,
		      enum sensor_attribute attr,
		      const struct sensor_value *val);

int apds9960_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler);
#endif /* CONFIG_APDS9960_TRIGGER */

#endif /* ZEPHYR_DRIVERS_SENSOR_APDS9960_APDS9960_H_*/
