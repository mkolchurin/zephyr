/*
 * Copyright Runtime.io 2018. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief A driver for sending and receiving mcumgr packets over UART.
 */

#include <string.h>
#include <kernel.h>
#include <mgmt/mcumgr/serial.h>
#include <drivers/console/modbus_mcumgr.h>

static struct modbus_smp *modbus_com;

/** Callback to execute when a valid fragment has been received. */
static modbus_mcumgr_recv_fn *modbus_mgumgr_recv_cb;

/** Contains the fragment currently being received. */
static struct modbus_mcumgr_rx_buf *modbus_mcumgr_cur_buf;

/**
 * Whether the line currently being read should be ignored.  This is true if
 * the line is too long or if there is no buffer available to hold it.
 */
static bool modbus_mcumgr_ignoring;

/** Contains buffers to hold incoming request fragments. */
K_MEM_SLAB_DEFINE(modbus_mcumgr_slab, sizeof(struct modbus_mcumgr_rx_buf),
		  CONFIG_UART_MCUMGR_RX_BUF_COUNT, 1);

static struct modbus_mcumgr_rx_buf *modbus_mcumgr_alloc_rx_buf(void)
{
	struct modbus_mcumgr_rx_buf *rx_buf;
	void *block;
	int rc;

	rc = k_mem_slab_alloc(&modbus_mcumgr_slab, &block, K_NO_WAIT);
	if (rc != 0) {
		return NULL;
	}

	rx_buf = block;
	rx_buf->length = 0;
	return rx_buf;
}

void modbus_mcumgr_free_rx_buf(struct modbus_mcumgr_rx_buf *rx_buf)
{
	void *block;

	block = rx_buf;
	k_mem_slab_free(&modbus_mcumgr_slab, &block);
}

/**
 * Processes a single incoming byte.
 */
static struct modbus_mcumgr_rx_buf *uart_mcumgr_rx_byte(uint8_t byte)
{
	struct modbus_mcumgr_rx_buf *rx_buf;

	if (!modbus_mcumgr_ignoring) {
		if (modbus_mcumgr_cur_buf == NULL) {
			modbus_mcumgr_cur_buf = modbus_mcumgr_alloc_rx_buf();
			if (modbus_mcumgr_cur_buf == NULL) {
				/* Insufficient buffers; drop this fragment. */
				modbus_mcumgr_ignoring = true;
			}
		}
	}

	rx_buf = modbus_mcumgr_cur_buf;
	if (!modbus_mcumgr_ignoring) {
		if (rx_buf->length >= sizeof(rx_buf->data)) {
			/* Line too long; drop this fragment. */
			modbus_mcumgr_free_rx_buf(modbus_mcumgr_cur_buf);
			modbus_mcumgr_cur_buf = NULL;
			modbus_mcumgr_ignoring = true;
		} else {
			rx_buf->data[rx_buf->length++] = byte;
		}
	}

	if (byte == '\n') {
		/* Fragment complete. */
		if (modbus_mcumgr_ignoring) {
			modbus_mcumgr_ignoring = false;
		} else {
			modbus_mcumgr_cur_buf = NULL;
			return rx_buf;
		}
	}

	return NULL;
}




/**
 * ISR that is called when UART bytes are received.
 */
static void modbus_mcumgr_isr()
{
	struct modbus_mcumgr_rx_buf *rx_buf;
	uint8_t buf;
	int chunk_len = 1;
	while (modbus_vcom_poll_in(modbus_com, &buf) > 0) {
		rx_buf = uart_mcumgr_rx_byte(buf);
		if (rx_buf != NULL) {
			modbus_mgumgr_recv_cb(rx_buf);
		}
	}
}
/*
 * Sends raw data over the UART.
 */
static int modbus_mcumgr_send_raw(const void *data, int len, void *arg)
{
	const uint8_t *u8p;

	u8p = data;
	while (len--) {
		modbus_vcom_poll_out(modbus_com, *u8p++);
	}

	return 0;
}

int modbus_mcumgr_send(const uint8_t *data, int len)
{
	return mcumgr_serial_tx_pkt(data, len, modbus_mcumgr_send_raw, NULL);
}

void modbus_mcumgr_register(struct modbus_smp *mb, modbus_mcumgr_recv_fn *cb, const void **isr_handler)
{
	*isr_handler = modbus_mcumgr_isr;
	modbus_com = mb;
	modbus_mgumgr_recv_cb = cb;
	// modbus_mcumgr_setup(uart_mcumgr_dev);
}
