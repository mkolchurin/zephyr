/*
 * Copyright Runtime.io 2018. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief A driver for sending and receiving mcumgr packets over UART.
 *
 * @see include/mgmt/serial.h
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CONSOLE_MODBUS_MCUMGR_H_
#define ZEPHYR_INCLUDE_DRIVERS_CONSOLE_MODBUS_MCUMGR_H_

#include <stdlib.h>
#include <zephyr/types.h>
#include <sys/ring_buffer.h>

#ifdef __cplusplus
extern "C" {
#endif


#define RB_SIZE CONFIG_MODBUS_MCUMGR_RX_BUF_SIZE

struct rb_s {
	struct ring_buf rb;
	uint8_t buffer[RB_SIZE];
};

struct modbus_smp {
	struct rb_s rx_buf;
	struct rb_s tx_buf;
};

/**
 * @brief Contains an mcumgr fragment received over UART.
 */
struct modbus_mcumgr_rx_buf {
	void *fifo_reserved;   /* 1st word reserved for use by fifo */
	uint8_t data[CONFIG_MODBUS_MCUMGR_RX_BUF_SIZE];
	int length;
};

void modbus_vcom_poll_out(struct modbus_smp *ctx, const char c);
int modbus_vcom_poll_in(struct modbus_smp *ctx, char *c);
int modbus_vcom_tx_bytes_count();
/** @typedef uart_mcumgr_recv_fn
 * @brief Function that gets called when an mcumgr packet is received.
 *
 * Function that gets called when an mcumgr packet is received.  This function
 * gets called in the interrupt context.  Ownership of the specified buffer is
 * transferred to the callback when this function gets called.
 *
 * @param rx_buf                A buffer containing the incoming mcumgr packet.
 */
typedef void modbus_mcumgr_recv_fn(struct modbus_mcumgr_rx_buf *rx_buf);

/**
 * @brief Sends an mcumgr packet over UART.
 *
 * @param data                  Buffer containing the mcumgr packet to send.
 * @param len                   The length of the buffer, in bytes.
 *
 * @return                      0 on success; negative error code on failure.
 */
int modbus_mcumgr_send(const uint8_t *data, int len);

/**
 * @brief Frees the supplied receive buffer.
 *
 * @param rx_buf                The buffer to free.
 */
void modbus_mcumgr_free_rx_buf(struct modbus_mcumgr_rx_buf *rx_buf);

/**
 * @brief Registers an mcumgr UART receive handler.
 *
 * Configures the mcumgr UART driver to call the specified function when an
 * mcumgr request packet is received.
 *
 * @param cb                    The callback to execute when an mcumgr request
 *                                  packet is received.
 */
void modbus_mcumgr_register(struct modbus_smp *mb, modbus_mcumgr_recv_fn *cb, const void **isr_handler);

#ifdef __cplusplus
}
#endif

#endif
