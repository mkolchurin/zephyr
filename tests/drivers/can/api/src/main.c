/*
 * Copyright (c) 2022 Vestas Wind Systems A/S
 * Copyright (c) 2019 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/can.h>
#include <zephyr/ztest.h>

/**
 * @addtogroup t_can_driver
 * @{
 * @defgroup t_can_api test_can_api
 * @}
 */

/**
 * Test bitrates in bits/second.
 */
#define TEST_BITRATE_1 125000
#define TEST_BITRATE_2 250000

/**
 * @brief Test timeouts.
 */
#define TEST_SEND_TIMEOUT    K_MSEC(100)
#define TEST_RECEIVE_TIMEOUT K_MSEC(100)
#define TEST_RECOVER_TIMEOUT K_MSEC(100)

/**
 * @brief Standard (11-bit) CAN IDs and masks used for testing.
 */
#define TEST_CAN_STD_ID_1      0x555
#define TEST_CAN_STD_ID_2      0x556
#define TEST_CAN_STD_MASK_ID_1 0x55A
#define TEST_CAN_STD_MASK_ID_2 0x56A
#define TEST_CAN_STD_MASK      0x7F0
#define TEST_CAN_SOME_STD_ID   0x123

/**
 * @brief Extended (29-bit) CAN IDs and masks used for testing.
 */
#define TEST_CAN_EXT_ID_1      0x15555555
#define TEST_CAN_EXT_ID_2      0x15555556
#define TEST_CAN_EXT_MASK_ID_1 0x1555555A
#define TEST_CAN_EXT_MASK_ID_2 0x1555556A
#define TEST_CAN_EXT_MASK      0x1FFFFFF0

/**
 * @brief Global variables.
 */
static ZTEST_DMEM const struct device *can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
static struct k_sem rx_callback_sem;
static struct k_sem tx_callback_sem;

CAN_MSGQ_DEFINE(can_msgq, 5);

/**
 * @brief Standard (11-bit) CAN ID frame 1.
 */
const struct zcan_frame test_std_frame_1 = {
	.id_type = CAN_STANDARD_IDENTIFIER,
	.rtr     = CAN_DATAFRAME,
	.id      = TEST_CAN_STD_ID_1,
	.dlc     = 8,
	.data    = {1, 2, 3, 4, 5, 6, 7, 8}
};

/**
 * @brief Standard (11-bit) CAN ID frame 2.
 */
const struct zcan_frame test_std_frame_2 = {
	.id_type = CAN_STANDARD_IDENTIFIER,
	.rtr     = CAN_DATAFRAME,
	.id      = TEST_CAN_STD_ID_2,
	.dlc     = 8,
	.data    = {1, 2, 3, 4, 5, 6, 7, 8}
};

/**
 * @brief Extended (29-bit) CAN ID frame 1.
 */
const struct zcan_frame test_ext_frame_1 = {
	.id_type = CAN_EXTENDED_IDENTIFIER,
	.rtr     = CAN_DATAFRAME,
	.id      = TEST_CAN_EXT_ID_1,
	.dlc     = 8,
	.data    = {1, 2, 3, 4, 5, 6, 7, 8}
};

/**
 * @brief Extended (29-bit) CAN ID frame 1.
 */
const struct zcan_frame test_ext_frame_2 = {
	.id_type = CAN_EXTENDED_IDENTIFIER,
	.rtr     = CAN_DATAFRAME,
	.id      = TEST_CAN_EXT_ID_2,
	.dlc     = 8,
	.data    = {1, 2, 3, 4, 5, 6, 7, 8}
};

/**
 * @brief Standard (11-bit) CAN ID RTR frame 1.
 */
const struct zcan_frame test_std_rtr_frame_1 = {
	.id_type = CAN_STANDARD_IDENTIFIER,
	.rtr     = CAN_REMOTEREQUEST,
	.id      = TEST_CAN_STD_ID_1,
	.dlc     = 0,
	.data    = {0}
};

/**
 * @brief Extended (29-bit) CAN ID RTR frame 1.
 */
const struct zcan_frame test_ext_rtr_frame_1 = {
	.id_type = CAN_EXTENDED_IDENTIFIER,
	.rtr     = CAN_REMOTEREQUEST,
	.id      = TEST_CAN_EXT_ID_1,
	.dlc     = 0,
	.data    = {0}
};

/**
 * @brief Standard (11-bit) CAN ID filter 1. This filter matches
 * ``test_std_frame_1``.
 */
const struct zcan_filter test_std_filter_1 = {
	.id_type = CAN_STANDARD_IDENTIFIER,
	.rtr = CAN_DATAFRAME,
	.id = TEST_CAN_STD_ID_1,
	.rtr_mask = 1,
	.id_mask = CAN_STD_ID_MASK
};

/**
 * @brief Standard (11-bit) CAN ID filter 2. This filter matches
 * ``test_std_frame_2``.
 */
const struct zcan_filter test_std_filter_2 = {
	.id_type = CAN_STANDARD_IDENTIFIER,
	.rtr = CAN_DATAFRAME,
	.id = TEST_CAN_STD_ID_2,
	.rtr_mask = 1,
	.id_mask = CAN_STD_ID_MASK
};

/**
 * @brief Standard (11-bit) CAN ID masked filter 1. This filter matches
 * ``test_std_frame_1``.
 */
const struct zcan_filter test_std_masked_filter_1 = {
	.id_type = CAN_STANDARD_IDENTIFIER,
	.rtr = CAN_DATAFRAME,
	.id = TEST_CAN_STD_MASK_ID_1,
	.rtr_mask = 1,
	.id_mask = TEST_CAN_STD_MASK
};

/**
 * @brief Standard (11-bit) CAN ID masked filter 2. This filter matches
 * ``test_std_frame_2``.
 */
const struct zcan_filter test_std_masked_filter_2 = {
	.id_type = CAN_STANDARD_IDENTIFIER,
	.rtr = CAN_DATAFRAME,
	.id = TEST_CAN_STD_MASK_ID_2,
	.rtr_mask = 1,
	.id_mask = TEST_CAN_STD_MASK
};

/**
 * @brief Extended (29-bit) CAN ID filter 1. This filter matches
 * ``test_ext_frame_1``.
 */
const struct zcan_filter test_ext_filter_1 = {
	.id_type = CAN_EXTENDED_IDENTIFIER,
	.rtr = CAN_DATAFRAME,
	.id = TEST_CAN_EXT_ID_1,
	.rtr_mask = 1,
	.id_mask = CAN_EXT_ID_MASK
};

/**
 * @brief Extended (29-bit) CAN ID filter 2. This filter matches
 * ``test_ext_frame_2``.
 */
const struct zcan_filter test_ext_filter_2 = {
	.id_type = CAN_EXTENDED_IDENTIFIER,
	.rtr = CAN_DATAFRAME,
	.id = TEST_CAN_EXT_ID_2,
	.rtr_mask = 1,
	.id_mask = CAN_EXT_ID_MASK
};

/**
 * @brief Extended (29-bit) CAN ID masked filter 1. This filter matches
 * ``test_ext_frame_1``.
 */
const struct zcan_filter test_ext_masked_filter_1 = {
	.id_type = CAN_EXTENDED_IDENTIFIER,
	.rtr = CAN_DATAFRAME,
	.id = TEST_CAN_EXT_MASK_ID_1,
	.rtr_mask = 1,
	.id_mask = TEST_CAN_EXT_MASK
};

/**
 * @brief Extended (29-bit) CAN ID masked filter 2. This filter matches
 * ``test_ext_frame_2``.
 */
const struct zcan_filter test_ext_masked_filter_2 = {
	.id_type = CAN_EXTENDED_IDENTIFIER,
	.rtr = CAN_DATAFRAME,
	.id = TEST_CAN_EXT_ID_1,
	.rtr_mask = 1,
	.id_mask = TEST_CAN_EXT_MASK
};

/**
 * @brief Standard (11-bit) CAN ID RTR filter 1. This filter matches
 * ``test_std_rtr_frame_1``.
 */
const struct zcan_filter test_std_rtr_filter_1 = {
	.id_type = CAN_STANDARD_IDENTIFIER,
	.rtr = CAN_REMOTEREQUEST,
	.id = TEST_CAN_STD_ID_1,
	.rtr_mask = 1,
	.id_mask = CAN_STD_ID_MASK
};

/**
 * @brief Extended (29-bit) CAN ID RTR filter 1. This filter matches
 * ``test_ext_rtr_frame_1``.
 */
const struct zcan_filter test_ext_rtr_filter_1 = {
	.id_type = CAN_EXTENDED_IDENTIFIER,
	.rtr = CAN_REMOTEREQUEST,
	.id = TEST_CAN_EXT_ID_1,
	.rtr_mask = 1,
	.id_mask = CAN_EXT_ID_MASK
};

/**
 * @brief Standard (11-bit) CAN ID filter. This filter matches
 * ``TEST_CAN_SOME_STD_ID``.
 */
const struct zcan_filter test_std_some_filter = {
	.id_type = CAN_STANDARD_IDENTIFIER,
	.rtr = CAN_DATAFRAME,
	.id = TEST_CAN_SOME_STD_ID,
	.rtr_mask = 1,
	.id_mask = CAN_STD_ID_MASK
};

/**
 * @brief Assert that two CAN frames are equal given a CAN ID mask.
 *
 * @param frame1  First CAN frame.
 * @param frame2  Second CAN frame.
 * @param id_mask CAN ID mask.
 */
static inline void assert_frame_equal(const struct zcan_frame *frame1,
				      const struct zcan_frame *frame2,
				      uint32_t id_mask)
{
	zassert_equal(frame1->id_type, frame2->id_type, "ID type does not match");
	zassert_equal(frame1->rtr, frame2->rtr, "RTR bit does not match");
	zassert_equal(frame1->id | id_mask, frame2->id | id_mask, "ID does not match");
	zassert_equal(frame1->dlc, frame2->dlc, "DLC does not match");
	zassert_mem_equal(frame1->data, frame2->data, frame1->dlc, "Received data differ");
}

/**
 * @brief Standard (11-bit) CAN ID transmit callback 1.
 *
 * See @a can_tx_callback_t() for argument description.
 */
static void tx_std_callback_1(const struct device *dev, int error, void *user_data)
{
	const struct zcan_frame *frame = user_data;

	k_sem_give(&tx_callback_sem);

	zassert_equal(dev, can_dev, "CAN device does not match");
	zassert_equal(frame->id, TEST_CAN_STD_ID_1, "ID does not match");
}

/**
 * @brief Standard (11-bit) CAN ID transmit callback 2.
 *
 * See @a can_tx_callback_t() for argument description.
 */
static void tx_std_callback_2(const struct device *dev, int error, void *user_data)
{
	const struct zcan_frame *frame = user_data;

	k_sem_give(&tx_callback_sem);

	zassert_equal(dev, can_dev, "CAN device does not match");
	zassert_equal(frame->id, TEST_CAN_STD_ID_2, "ID does not match");
}

/**
 * @brief Extended (29-bit) CAN ID transmit callback 1.
 *
 * See @a can_tx_callback_t() for argument description.
 */
static void tx_ext_callback_1(const struct device *dev, int error, void *user_data)
{
	const struct zcan_frame *frame = user_data;

	k_sem_give(&tx_callback_sem);

	zassert_equal(dev, can_dev, "CAN device does not match");
	zassert_equal(frame->id, TEST_CAN_EXT_ID_1, "ID does not match");
}

/**
 * @brief Extended (29-bit) CAN ID transmit callback 2.
 *
 * See @a can_tx_callback_t() for argument description.
 */
static void tx_ext_callback_2(const struct device *dev, int error, void *user_data)
{
	const struct zcan_frame *frame = user_data;

	k_sem_give(&tx_callback_sem);

	zassert_equal(dev, can_dev, "CAN device does not match");
	zassert_equal(frame->id, TEST_CAN_EXT_ID_2, "ID does not match");
}

/**
 * @brief Standard (11-bit) CAN ID receive callback 1.
 *
 * See @a can_rx_callback_t() for argument description.
 */
static void rx_std_callback_1(const struct device *dev, struct zcan_frame *frame,
			      void *user_data)
{
	struct zcan_filter *filter = user_data;

	assert_frame_equal(frame, &test_std_frame_1, 0);
	zassert_equal(dev, can_dev, "CAN device does not match");
	zassert_equal_ptr(filter, &test_std_filter_1, "filter does not match");

	k_sem_give(&rx_callback_sem);
}

/**
 * @brief Standard (11-bit) CAN ID receive callback 2.
 *
 * See @a can_rx_callback_t() for argument description.
 */
static void rx_std_callback_2(const struct device *dev, struct zcan_frame *frame,
			      void *user_data)
{
	struct zcan_filter *filter = user_data;

	assert_frame_equal(frame, &test_std_frame_2, 0);
	zassert_equal(dev, can_dev, "CAN device does not match");
	zassert_equal_ptr(filter, &test_std_filter_2, "filter does not match");

	k_sem_give(&rx_callback_sem);
}

/**
 * @brief Standard (11-bit) masked CAN ID receive callback 1.
 *
 * See @a can_rx_callback_t() for argument description.
 */
static void rx_std_mask_callback_1(const struct device *dev, struct zcan_frame *frame,
				   void *user_data)
{
	struct zcan_filter *filter = user_data;

	assert_frame_equal(frame, &test_std_frame_1, 0x0F);
	zassert_equal(dev, can_dev, "CAN device does not match");
	zassert_equal_ptr(filter, &test_std_masked_filter_1, "filter does not match");

	k_sem_give(&rx_callback_sem);
}

/**
 * @brief Standard (11-bit) masked CAN ID receive callback 2.
 *
 * See @a can_rx_callback_t() for argument description.
 */
static void rx_std_mask_callback_2(const struct device *dev, struct zcan_frame *frame,
				   void *user_data)
{
	struct zcan_filter *filter = user_data;

	assert_frame_equal(frame, &test_std_frame_2, 0x0F);
	zassert_equal(dev, can_dev, "CAN device does not match");
	zassert_equal_ptr(filter, &test_std_masked_filter_2, "filter does not match");

	k_sem_give(&rx_callback_sem);
}

/**
 * @brief Extended (29-bit) CAN ID receive callback 1.
 *
 * See @a can_rx_callback_t() for argument description.
 */
static void rx_ext_callback_1(const struct device *dev, struct zcan_frame *frame,
			      void *user_data)
{
	struct zcan_filter *filter = user_data;

	assert_frame_equal(frame, &test_ext_frame_1, 0);
	zassert_equal(dev, can_dev, "CAN device does not match");
	zassert_equal_ptr(filter, &test_ext_filter_1, "filter does not match");

	k_sem_give(&rx_callback_sem);
}

/**
 * @brief Extended (29-bit) CAN ID receive callback 2.
 *
 * See @a can_rx_callback_t() for argument description.
 */
static void rx_ext_callback_2(const struct device *dev, struct zcan_frame *frame,
			      void *user_data)
{
	struct zcan_filter *filter = user_data;

	assert_frame_equal(frame, &test_ext_frame_2, 0);
	zassert_equal(dev, can_dev, "CAN device does not match");
	zassert_equal_ptr(filter, &test_ext_filter_2, "filter does not match");

	k_sem_give(&rx_callback_sem);
}

/**
 * @brief Extended (29-bit) masked CAN ID receive callback 1.
 *
 * See @a can_rx_callback_t() for argument description.
 */
static void rx_ext_mask_callback_1(const struct device *dev, struct zcan_frame *frame,
				   void *user_data)
{
	struct zcan_filter *filter = user_data;

	assert_frame_equal(frame, &test_ext_frame_1, 0x0F);
	zassert_equal(dev, can_dev, "CAN device does not match");
	zassert_equal_ptr(filter, &test_ext_masked_filter_1, "filter does not match");

	k_sem_give(&rx_callback_sem);
}

/**
 * @brief Extended (29-bit) masked CAN ID receive callback 2.
 *
 * See @a can_rx_callback_t() for argument description.
 */
static void rx_ext_mask_callback_2(const struct device *dev, struct zcan_frame *frame,
				   void *user_data)
{
	struct zcan_filter *filter = user_data;

	assert_frame_equal(frame, &test_ext_frame_2, 0x0F);
	zassert_equal(dev, can_dev, "CAN device does not match");
	zassert_equal_ptr(filter, &test_ext_masked_filter_2, "filter does not match");

	k_sem_give(&rx_callback_sem);
}

/**
 * @brief Send a CAN test frame with asserts.
 *
 * This function will block until the frame is transmitted or a test timeout
 * occurs.
 *
 * @param dev   Pointer to the device structure for the driver instance.
 * @param frame Pointer to the CAN frame to send.
 */
static void send_test_frame(const struct device *dev, const struct zcan_frame *frame)
{
	int err;

	err = can_send(dev, frame, TEST_SEND_TIMEOUT, NULL, NULL);
	zassert_not_equal(err, -EBUSY, "arbitration lost in loopback mode");
	zassert_equal(err, 0, "failed to send frame (err %d)", err);
}

/**
 * @brief Send a CAN test frame with asserts.
 *
 * This function will block until the frame is queued or a test timeout
 * occurs.
 *
 * @param dev      Pointer to the device structure for the driver instance.
 * @param frame    Pointer to the CAN frame to send.
 * @param callback Transmit callback function.
 */
static void send_test_frame_nowait(const struct device *dev, const struct zcan_frame *frame,
				   can_tx_callback_t callback)
{
	int err;

	err = can_send(dev, frame, TEST_SEND_TIMEOUT, callback, (void *)frame);
	zassert_not_equal(err, -EBUSY, "arbitration lost in loopback mode");
	zassert_equal(err, 0, "failed to send frame (err %d)", err);
}

/**
 * @brief Add a CAN message queue with asserts.
 *
 * @param dev    Pointer to the device structure for the driver instance.
 * @param filter CAN filter for the CAN message queue.
 *
 * @return CAN filter ID.
 */
static inline int add_rx_msgq(const struct device *dev, const struct zcan_filter *filter)
{
	int filter_id;

	filter_id = can_add_rx_filter_msgq(dev, &can_msgq, filter);
	zassert_not_equal(filter_id, -ENOSPC, "no filters available");
	zassert_true(filter_id >= 0, "negative filter number");

	return filter_id;
}

/**
 * @brief Add a CAN filter with asserts.
 *
 * @param dev      Pointer to the device structure for the driver instance.
 * @param filter   CAN filter.
 * @param callback Receive callback function.
 *
 * @return CAN filter ID.
 */
static inline int add_rx_filter(const struct device *dev,
				const struct zcan_filter *filter,
				can_rx_callback_t callback)
{
	int filter_id;

	k_sem_reset(&rx_callback_sem);

	filter_id = can_add_rx_filter(dev, callback, (void *)filter, filter);
	zassert_not_equal(filter_id, -ENOSPC, "no filters available");
	zassert_true(filter_id >= 0, "negative filter number");

	return filter_id;
}

/**
 * @brief Perform a send/receive test with a set of CAN ID filters and CAN frames.
 *
 * @param filter1 CAN filter 1
 * @param filter2 CAN filter 2
 * @param frame1  CAN frame 1
 * @param frame2  CAN frame 2
 */
static void send_receive(const struct zcan_filter *filter1,
			 const struct zcan_filter *filter2,
			 const struct zcan_frame *frame1,
			 const struct zcan_frame *frame2)
{
	struct zcan_frame frame_buffer;
	uint32_t mask = 0U;
	int filter_id_1;
	int filter_id_2;
	int err;

	filter_id_1 = add_rx_msgq(can_dev, filter1);
	send_test_frame(can_dev, frame1);

	err = k_msgq_get(&can_msgq, &frame_buffer, TEST_RECEIVE_TIMEOUT);
	zassert_equal(err, 0, "receive timeout");

	if (filter1->id_type == CAN_STANDARD_IDENTIFIER) {
		if (filter1->id_mask != CAN_STD_ID_MASK) {
			mask = 0x0F;
		}
	} else {
		if (filter1->id_mask != CAN_EXT_ID_MASK) {
			mask = 0x0F;
		}
	}

	assert_frame_equal(&frame_buffer, frame1, mask);
	can_remove_rx_filter(can_dev, filter_id_1);

	k_sem_reset(&tx_callback_sem);

	if (frame1->id_type == CAN_STANDARD_IDENTIFIER) {
		if (filter1->id_mask == CAN_STD_ID_MASK) {
			filter_id_1 = add_rx_filter(can_dev, filter1, rx_std_callback_1);
			filter_id_2 = add_rx_filter(can_dev, filter2, rx_std_callback_2);
			send_test_frame_nowait(can_dev, frame1, tx_std_callback_1);
			send_test_frame_nowait(can_dev, frame2, tx_std_callback_2);
		} else {
			filter_id_1 = add_rx_filter(can_dev, filter1, rx_std_mask_callback_1);
			filter_id_2 = add_rx_filter(can_dev, filter2, rx_std_mask_callback_2);
			send_test_frame_nowait(can_dev, frame1, tx_std_callback_1);
			send_test_frame_nowait(can_dev, frame2, tx_std_callback_2);
		}
	} else {
		if (filter1->id_mask == CAN_EXT_ID_MASK) {
			filter_id_1 = add_rx_filter(can_dev, filter1, rx_ext_callback_1);
			filter_id_2 = add_rx_filter(can_dev, filter2, rx_ext_callback_2);
			send_test_frame_nowait(can_dev, frame1, tx_ext_callback_1);
			send_test_frame_nowait(can_dev, frame2, tx_ext_callback_2);
		} else {
			filter_id_1 = add_rx_filter(can_dev, filter1, rx_ext_mask_callback_1);
			filter_id_2 = add_rx_filter(can_dev, filter2, rx_ext_mask_callback_2);
			send_test_frame_nowait(can_dev, frame1, tx_ext_callback_1);
			send_test_frame_nowait(can_dev, frame2, tx_ext_callback_2);
		}
	}

	err = k_sem_take(&rx_callback_sem, TEST_RECEIVE_TIMEOUT);
	zassert_equal(err, 0, "receive timeout");

	err = k_sem_take(&rx_callback_sem, TEST_RECEIVE_TIMEOUT);
	zassert_equal(err, 0, "receive timeout");

	err = k_sem_take(&tx_callback_sem, TEST_SEND_TIMEOUT);
	zassert_equal(err, 0, "missing TX callback");

	err = k_sem_take(&tx_callback_sem, TEST_SEND_TIMEOUT);
	zassert_equal(err, 0, "missing TX callback");

	can_remove_rx_filter(can_dev, filter_id_1);
	can_remove_rx_filter(can_dev, filter_id_2);
}

/**
 * @brief Perform a send/receive test with a set of CAN ID filters and CAN frames, RTR and data
 * frames.
 *
 * @param data_filter CAN data filter
 * @param rtr_filter  CAN RTR filter
 * @param data_frame  CAN data frame
 * @param rtr_frame   CAN RTR frame
 */
void send_receive_rtr(const struct zcan_filter *data_filter,
		      const struct zcan_filter *rtr_filter,
		      const struct zcan_frame *data_frame,
		      const struct zcan_frame *rtr_frame)
{
	struct zcan_frame frame;
	int filter_id;
	int err;

	filter_id = add_rx_msgq(can_dev, rtr_filter);

	/* Verify that RTR filter does not match data frame */
	send_test_frame(can_dev, data_frame);
	err = k_msgq_get(&can_msgq, &frame, TEST_RECEIVE_TIMEOUT);
	zassert_equal(err, -EAGAIN, "Data frame passed RTR filter");

	/* Verify that RTR filter matches RTR frame */
	send_test_frame(can_dev, rtr_frame);
	err = k_msgq_get(&can_msgq, &frame, TEST_RECEIVE_TIMEOUT);
	zassert_equal(err, 0, "receive timeout");
	assert_frame_equal(&frame, rtr_frame, 0);

	can_remove_rx_filter(can_dev, filter_id);

	filter_id = add_rx_msgq(can_dev, data_filter);

	/* Verify that data filter does not match RTR frame */
	send_test_frame(can_dev, rtr_frame);
	err = k_msgq_get(&can_msgq, &frame, TEST_RECEIVE_TIMEOUT);
	zassert_equal(err, -EAGAIN, "RTR frame passed data filter");

	/* Verify that data filter matches data frame */
	send_test_frame(can_dev, data_frame);
	err = k_msgq_get(&can_msgq, &frame, TEST_RECEIVE_TIMEOUT);
	zassert_equal(err, 0, "receive timeout");
	assert_frame_equal(&frame, data_frame, 0);

	can_remove_rx_filter(can_dev, filter_id);
}

/**
 * @brief Test getting the CAN core clock rate.
 */
ZTEST_USER(can_api, test_get_core_clock)
{
	uint32_t rate;
	int err;

	err = can_get_core_clock(can_dev, &rate);
	zassert_equal(err, 0, "failed to get CAN core clock rate (err %d)", err);
	zassert_not_equal(rate, 0, "CAN core clock rate is 0");
}

/**
 * @brief Test getting the CAN controller capabilities.
 */
ZTEST_USER(can_api, test_get_capabilities)
{
	can_mode_t cap;
	int err;

	err = can_get_capabilities(can_dev, &cap);
	zassert_equal(err, 0, "failed to get CAN capabilities (err %d)", err);
	zassert_not_equal(cap & CAN_MODE_LOOPBACK, 0, "CAN loopback mode not supported");
}

/**
 * @brief CAN state change callback.
 */
static void state_change_callback(const struct device *dev, enum can_state state,
				  struct can_bus_err_cnt err_cnt, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(state);
	ARG_UNUSED(err_cnt);
	ARG_UNUSED(user_data);
}

/**
 * @brief Test setting the CAN state change callback.
 */
ZTEST(can_api, test_set_state_change_callback)
{
	/* It is not possible to provoke a change of state, but test the API call */
	can_set_state_change_callback(can_dev, state_change_callback, NULL);
	can_set_state_change_callback(can_dev, NULL, NULL);
}

/**
 * @brief Test setting a too high bitrate.
 */
ZTEST_USER(can_api, test_set_bitrate_too_high)
{
	uint32_t max;
	int err;

	err = can_get_max_bitrate(can_dev, &max);
	if (err == -ENOSYS) {
		ztest_test_skip();
	}

	zassert_equal(err, 0, "failed to get max bitrate (err %d)", err);
	zassert_not_equal(max, 0, "max bitrate is 0");

	err = can_set_bitrate(can_dev, max + 1);
	zassert_equal(err, -ENOTSUP, "too high bitrate accepted");
}

/**
 * @brief Test setting bitrate.
 */
ZTEST_USER(can_api, test_set_bitrate)
{
	int err;

	err = can_set_bitrate(can_dev, TEST_BITRATE_1);
	zassert_equal(err, 0, "failed to set bitrate");
}

/**
 * @brief Test sending a message with no filters installed.
 *
 * This basic test work since the CAN controller is in loopback mode and
 * therefore ACKs its own frame.
 */
ZTEST_USER(can_api, test_send_and_forget)
{
	send_test_frame(can_dev, &test_std_frame_1);
}

/**
 * @brief Test adding basic filters.
 *
 * Test each filter type but only one filter at a time.
 */
ZTEST(can_api, test_add_filter)
{
	int filter_id;

	filter_id = add_rx_filter(can_dev, &test_std_filter_1, rx_std_callback_1);
	can_remove_rx_filter(can_dev, filter_id);

	filter_id = add_rx_filter(can_dev, &test_ext_filter_1, rx_ext_callback_1);
	can_remove_rx_filter(can_dev, filter_id);

	filter_id = add_rx_msgq(can_dev, &test_std_filter_1);
	can_remove_rx_filter(can_dev, filter_id);

	filter_id = add_rx_msgq(can_dev, &test_ext_filter_1);
	can_remove_rx_filter(can_dev, filter_id);

	filter_id = add_rx_filter(can_dev, &test_std_masked_filter_1, rx_std_mask_callback_1);
	can_remove_rx_filter(can_dev, filter_id);

	filter_id = add_rx_filter(can_dev, &test_ext_masked_filter_1, rx_ext_mask_callback_1);
	can_remove_rx_filter(can_dev, filter_id);
}

/**
 * @brief Test adding up to and above the maximum number of RX filters.
 *
 * @param id_type CAN frame identifier type
 * @param id_mask filter
 */
static void add_remove_max_filters(enum can_ide id_type)
{
	uint32_t id_mask = id_type == CAN_STANDARD_IDENTIFIER ? CAN_STD_ID_MASK : CAN_EXT_ID_MASK;
	struct zcan_filter filter = {
		.id_type = id_type,
		.rtr = CAN_DATAFRAME,
		.id = 0,
		.rtr_mask = 1,
		.id_mask = id_mask,
	};
	int filter_id;
	int max;
	int i;

	max = can_get_max_filters(can_dev, id_type);
	if (max == -ENOSYS || max == 0) {
		/*
		 * Skip test if max is not known or no filters of the given type
		 * is supported.
		 */
		ztest_test_skip();
	}

	zassert_true(max > 0, "failed to get max filters (err %d)", max);

	int filter_ids[max];

	for (i = 0; i < max; i++) {
		filter.id++;
		filter_ids[i] = add_rx_msgq(can_dev, &filter);
	}

	filter.id++;
	filter_id = can_add_rx_filter_msgq(can_dev, &can_msgq, &filter);
	zassert_equal(filter_id, -ENOSPC, "added more than max filters");

	for (i = 0; i < max; i++) {
		can_remove_rx_filter(can_dev, filter_ids[i]);
	}
}

/**
 * @brief Test max standard (11-bit) CAN RX filters.
 */
ZTEST_USER(can_api, test_max_std_filters)
{
	add_remove_max_filters(CAN_STANDARD_IDENTIFIER);
}

/**
 * @brief Test max extended (29-bit) CAN RX filters.
 */
ZTEST_USER(can_api, test_max_ext_filters)
{
	add_remove_max_filters(CAN_EXTENDED_IDENTIFIER);
}

/**
 * @brief Test that no message is received when nothing was sent.
 */
ZTEST_USER(can_api, test_receive_timeout)
{
	struct zcan_frame frame;
	int filter_id;
	int err;

	filter_id = add_rx_msgq(can_dev, &test_std_filter_1);

	err = k_msgq_get(&can_msgq, &frame, TEST_RECEIVE_TIMEOUT);
	zassert_equal(err, -EAGAIN, "received a frame without sending one");

	can_remove_rx_filter(can_dev, filter_id);
}

/**
 * @brief Test that transmit callback function is called.
 */
ZTEST(can_api, test_send_callback)
{
	int err;

	k_sem_reset(&tx_callback_sem);

	send_test_frame_nowait(can_dev, &test_std_frame_1, tx_std_callback_1);

	err = k_sem_take(&tx_callback_sem, TEST_SEND_TIMEOUT);
	zassert_equal(err, 0, "missing TX callback");
}

/**
 * @brief Test send/receive with standard (11-bit) CAN IDs.
 */
ZTEST(can_api, test_send_receive_std_id)
{
	send_receive(&test_std_filter_1, &test_std_filter_2,
		     &test_std_frame_1, &test_std_frame_2);
}

/**
 * @brief Test send/receive with extended (29-bit) CAN IDs.
 */
ZTEST(can_api, test_send_receive_ext_id)
{
	send_receive(&test_ext_filter_1, &test_ext_filter_2,
		     &test_ext_frame_1, &test_ext_frame_2);
}

/**
 * @brief Test send/receive with standard (11-bit) masked CAN IDs.
 */
ZTEST(can_api, test_send_receive_std_id_masked)
{
	send_receive(&test_std_masked_filter_1, &test_std_masked_filter_2,
		     &test_std_frame_1, &test_std_frame_2);
}

/**
 * @brief Test send/receive with extended (29-bit) masked CAN IDs.
 */
ZTEST(can_api, test_send_receive_ext_id_masked)
{
	send_receive(&test_ext_masked_filter_1, &test_ext_masked_filter_2,
		     &test_ext_frame_1, &test_ext_frame_2);
}

/**
 * @brief Test send/receive with messages buffered in a CAN message queue.
 */
ZTEST_USER(can_api, test_send_receive_msgq)
{
	struct k_msgq_attrs attrs;
	struct zcan_frame frame;
	int filter_id;
	int nframes;
	int err;
	int i;

	filter_id = add_rx_msgq(can_dev, &test_std_filter_1);

	k_msgq_get_attrs(&can_msgq, &attrs);
	nframes = attrs.max_msgs;

	for (i = 0; i < nframes; i++) {
		send_test_frame(can_dev, &test_std_frame_1);
	}

	for (i = 0; i < nframes; i++) {
		err = k_msgq_get(&can_msgq, &frame, TEST_RECEIVE_TIMEOUT);
		zassert_equal(err, 0, "receive timeout");
		assert_frame_equal(&frame, &test_std_frame_1, 0);
	}

	for (i = 0; i < nframes; i++) {
		send_test_frame(can_dev, &test_std_frame_1);
	}

	for (i = 0; i < nframes; i++) {
		err = k_msgq_get(&can_msgq, &frame, TEST_RECEIVE_TIMEOUT);
		zassert_equal(err, 0, "receive timeout");
		assert_frame_equal(&frame, &test_std_frame_1, 0);
	}

	can_remove_rx_filter(can_dev, filter_id);
}

/**
 * @brief Test send/receive with standard (11-bit) CAN IDs and remote transmission request (RTR).
 */
ZTEST_USER(can_api, test_send_receive_std_id_rtr)
{
	send_receive_rtr(&test_std_filter_1, &test_std_rtr_filter_1,
			 &test_std_frame_1, &test_std_rtr_frame_1);
}

/**
 * @brief Test send/receive with extended (29-bit) CAN IDs and remote transmission request (RTR).
 */
ZTEST_USER(can_api, test_send_receive_ext_id_rtr)
{
	send_receive_rtr(&test_ext_filter_1, &test_ext_rtr_filter_1,
			 &test_ext_frame_1, &test_ext_rtr_frame_1);
}

/**
 * @brief Test that non-matching CAN frames do not pass a filter.
 */
ZTEST(can_api, test_send_receive_wrong_id)
{
	struct zcan_frame frame_buffer;
	int filter_id;
	int err;

	filter_id = add_rx_msgq(can_dev, &test_std_filter_1);

	send_test_frame(can_dev, &test_std_frame_2);

	err = k_msgq_get(&can_msgq, &frame_buffer, TEST_RECEIVE_TIMEOUT);
	zassert_equal(err, -EAGAIN, "recevied a frame that should not pass the filter");

	can_remove_rx_filter(can_dev, filter_id);
}

/**
 * @brief Test that frames with invalid Data Length Code (DLC) are rejected.
 */
ZTEST_USER(can_api, test_send_invalid_dlc)
{
	struct zcan_frame frame;
	int err;

	frame.dlc = CAN_MAX_DLC + 1;

	err = can_send(can_dev, &frame, TEST_SEND_TIMEOUT, NULL, NULL);
	zassert_equal(err, -EINVAL, "sent a frame with an invalid DLC");
}

ZTEST_USER(can_api, test_recover)
{
	int err;

	/* It is not possible to provoke a bus off state, but test the API call */
	err = can_recover(can_dev, TEST_RECOVER_TIMEOUT);
	if (err == -ENOTSUP) {
		ztest_test_skip();
	}

	zassert_equal(err, 0, "failed to recover (err %d)", err);
}

ZTEST_USER(can_api, test_get_state)
{
	struct can_bus_err_cnt err_cnt;
	enum can_state state;
	int err;

	err = can_get_state(can_dev, NULL, NULL);
	zassert_equal(err, 0, "failed to get CAN state without destinations (err %d)", err);

	err = can_get_state(can_dev, &state, NULL);
	zassert_equal(err, 0, "failed to get CAN state (err %d)", err);

	err = can_get_state(can_dev, NULL, &err_cnt);
	zassert_equal(err, 0, "failed to get CAN error counters (err %d)", err);

	err = can_get_state(can_dev, &state, &err_cnt);
	zassert_equal(err, 0, "failed to get CAN state + error counters (err %d)", err);
}

ZTEST_USER(can_api, test_filters_preserved_through_mode_change)
{
	struct zcan_frame frame;
	int filter_id;
	int err;

	filter_id = add_rx_msgq(can_dev, &test_std_filter_1);
	send_test_frame(can_dev, &test_std_frame_1);

	err = k_msgq_get(&can_msgq, &frame, TEST_RECEIVE_TIMEOUT);
	zassert_equal(err, 0, "receive timeout");
	assert_frame_equal(&frame, &test_std_frame_1, 0);

	err = can_set_mode(can_dev, CAN_MODE_NORMAL);
	zassert_equal(err, 0, "failed to set normal mode (err %d)", err);

	err = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
	zassert_equal(err, 0, "failed to set loopback-mode (err %d)", err);

	send_test_frame(can_dev, &test_std_frame_1);

	err = k_msgq_get(&can_msgq, &frame, TEST_RECEIVE_TIMEOUT);
	zassert_equal(err, 0, "receive timeout");
	assert_frame_equal(&frame, &test_std_frame_1, 0);

	can_remove_rx_filter(can_dev, filter_id);
}

ZTEST_USER(can_api, test_filters_preserved_through_bitrate_change)
{
	struct zcan_frame frame;
	int filter_id;
	int err;

	filter_id = add_rx_msgq(can_dev, &test_std_filter_1);
	send_test_frame(can_dev, &test_std_frame_1);

	err = k_msgq_get(&can_msgq, &frame, TEST_RECEIVE_TIMEOUT);
	zassert_equal(err, 0, "receive timeout");
	assert_frame_equal(&frame, &test_std_frame_1, 0);

	err = can_set_bitrate(can_dev, TEST_BITRATE_2);
	zassert_equal(err, 0, "failed to set bitrate");

	err = can_set_bitrate(can_dev, TEST_BITRATE_1);
	zassert_equal(err, 0, "failed to set bitrate");

	send_test_frame(can_dev, &test_std_frame_1);

	err = k_msgq_get(&can_msgq, &frame, TEST_RECEIVE_TIMEOUT);
	zassert_equal(err, 0, "receive timeout");
	assert_frame_equal(&frame, &test_std_frame_1, 0);

	can_remove_rx_filter(can_dev, filter_id);
}

void *can_api_setup(void)
{
	int err;

	k_sem_init(&rx_callback_sem, 0, 2);
	k_sem_init(&tx_callback_sem, 0, 2);

	k_object_access_grant(&can_msgq, k_current_get());
	k_object_access_grant(can_dev, k_current_get());

	zassert_true(device_is_ready(can_dev), "CAN device not ready");

	err = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
	zassert_equal(err, 0, "failed to set loopback mode (err %d)", err);

	return NULL;
}

ZTEST_SUITE(can_api, NULL, can_api_setup, NULL, NULL, NULL);
