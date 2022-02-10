/*
 * Copyright Runtime.io 2018. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief UART transport for the mcumgr SMP protocol.
 */

#include <string.h>
#include <zephyr.h>
#include <init.h>
#include "net/buf.h"
#include <drivers/console/modbus_mcumgr.h>
#include "mgmt/mgmt.h"
#include <mgmt/mcumgr/serial.h>
#include "mgmt/mcumgr/buf.h"
#include "mgmt/mcumgr/smp.h"
#include <logging/log.h>
#include <mgmt/mcumgr/smp_modbus.h>

LOG_MODULE_REGISTER(smp_modbus);

static void smp_modbus_process_rx_queue(struct k_work *work);

K_FIFO_DEFINE(smp_modbus_rx_fifo);
K_WORK_DEFINE(smp_modbus_work, smp_modbus_process_rx_queue);

static struct mcumgr_serial_rx_ctxt smp_modbus_rx_ctxt;
static struct zephyr_smp_transport smp_modbus_transport;
static struct modbus_smp modbus_com;
static void (*modbus_isr_handler)(void);

/**
 * Processes a single line (fragment) coming from the mcumgr UART driver.
 */
static void smp_uart_process_frag(struct modbus_mcumgr_rx_buf *rx_buf)
{
	struct net_buf *nb;

	/* Decode the fragment and write the result to the global receive
	 * context.
	 */
	nb = mcumgr_serial_process_frag(&smp_modbus_rx_ctxt, rx_buf->data, rx_buf->length);

	/* Release the encoded fragment. */
	modbus_mcumgr_free_rx_buf(rx_buf);

	/* If a complete packet has been received, pass it to SMP for
	 * processing.
	 */
	if (nb != NULL) {
		zephyr_smp_rx_req(&smp_modbus_transport, nb);
	}
}

static void smp_modbus_process_rx_queue(struct k_work *work)
{
	struct modbus_mcumgr_rx_buf *rx_buf;

	while ((rx_buf = k_fifo_get(&smp_modbus_rx_fifo, K_NO_WAIT)) != NULL) {
		smp_uart_process_frag(rx_buf);
	}
}

/**
 * Enqueues a received SMP fragment for later processing.  This function
 * executes in the interrupt context.
 */
static void smp_modbus_rx_frag(struct modbus_mcumgr_rx_buf *rx_buf)
{
	k_fifo_put(&smp_modbus_rx_fifo, rx_buf);
	k_work_submit(&smp_modbus_work);
}

static uint16_t smp_uart_get_mtu(const struct net_buf *nb)
{
	return 128;
}

int modbus_virtual_com_init(struct modbus_smp *ctx)
{
	ring_buf_init(&ctx->rx_buf.rb, sizeof(ctx->rx_buf.buffer), ctx->rx_buf.buffer);
	ring_buf_init(&ctx->tx_buf.rb, sizeof(ctx->tx_buf.buffer), ctx->tx_buf.buffer);
	return 0;
}

int modbus_virtual_com_wr_callback(uint8_t reg)
{
	uint8_t c = (uint8_t)reg & 0xFF;
	int ret = ring_buf_put(&modbus_com.rx_buf.rb, &c, 1);
	if (ret > 0)
		LOG_DBG("WR Handler, set %dB", ret);
	else
		LOG_ERR("WR Handler, failed %d", ret);

	if (modbus_isr_handler != NULL)
		(*modbus_isr_handler)();
	return ret;
}

int modbus_virtual_com_rd_callback(uint8_t *reg)
{
	uint8_t c;
	int ret = ring_buf_get(&modbus_com.tx_buf.rb, &c, 1);
	if (ret > 0) {
		*reg = (uint16_t)c;
		LOG_DBG("RD Handler, set %dB", ret);
	} else
		LOG_ERR("RD Handler, failed %d", ret);
	return ret;
}

void modbus_vcom_poll_out(struct modbus_smp *ctx, const char c)
{
	int ret = ring_buf_put(&ctx->tx_buf.rb, &c, 1);
	LOG_DBG("poll out %d", ret);
}

int modbus_vcom_poll_in(struct modbus_smp *ctx, char *c)
{
	if (ring_buf_is_empty(&ctx->rx_buf.rb))
		return 0;
	int ret = ring_buf_get(&ctx->rx_buf.rb, c, 1);
	LOG_DBG("poll in %c", *c);
	return ret;
}

int modbus_vcom_rx_bytes_count(){
	return ring_buf_size_get(&modbus_com.rx_buf.rb);
}
int modbus_vcom_tx_bytes_count(){
	return ring_buf_size_get(&modbus_com.tx_buf.rb);
}
static int modbus_mcumgr_send_raw(const void *data, int len, void *arg)
{
	const uint8_t *u8p;

	u8p = data;
	while (len--) {
		modbus_vcom_poll_out(&modbus_com, *u8p++);
	}

	return 0;
}

static int smp_uart_tx_pkt(struct zephyr_smp_transport *zst, struct net_buf *nb)
{
	int rc;

	rc = mcumgr_serial_tx_pkt(nb->data, nb->len, modbus_mcumgr_send_raw,
				  NULL); //uart_mcumgr_send(, );
	mcumgr_buf_free(nb);

	return rc;
}

static int smp_modbus_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	zephyr_smp_transport_init(&smp_modbus_transport, smp_uart_tx_pkt, smp_uart_get_mtu, NULL,
				  NULL);

	modbus_virtual_com_init(&modbus_com);
	modbus_mcumgr_register(&modbus_com, &smp_modbus_rx_frag, &modbus_isr_handler);
	// uart_mcumgr_register(smp_uart_rx_frag);

	return 0;
}

SYS_INIT(smp_modbus_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
