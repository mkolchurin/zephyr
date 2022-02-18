/*
 * Copyright (c) 2019, Prevas A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief UDP transport for the mcumgr SMP protocol.
 */

#include <zephyr.h>
#include <init.h>
#include <errno.h>
#include <net/buf.h>
#include <mgmt/mgmt.h>
#include <mgmt/mcumgr/buf.h>
#include <mgmt/mcumgr/smp.h>
#include <mgmt/mcumgr/smp_pipe.h>

#define LOG_LEVEL CONFIG_MCUMGR_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(smp_pipe);

struct config {
	struct zephyr_smp_transport smp_transport;
	char recv_buffer[CONFIG_MCUMGR_SMP_PIPE_MTU];
};

struct configs {
	struct config pipe;
};

static struct configs pipe_config;
static int (*smp_pipe_tx)(unsigned char *buf, unsigned int len);

static uint16_t smp_pipe_get_mtu(const struct net_buf *nb)
{
	ARG_UNUSED(nb);

	return CONFIG_MCUMGR_SMP_PIPE_MTU;
}

// static int smp_pipe_ud_copy(struct net_buf *dst, const struct net_buf *src)
// {
// 	struct sockaddr *src_ud = net_buf_user_data(src);
// 	struct sockaddr *dst_ud = net_buf_user_data(dst);

// 	net_ipaddr_copy(dst_ud, src_ud);

// 	return MGMT_ERR_EOK;
// }

int smp_pipe_rx(uint8_t *buf, uint32_t len)
{
	struct net_buf *nb;
	nb = mcumgr_buf_alloc();
	if (!nb) {
		LOG_ERR("Failed to allocate mcumgr buffer");
		/* No free space, drop smp frame */
		return -ENOMEM;
	}
	net_buf_add_mem(nb, buf, len);
	zephyr_smp_rx_req(&(pipe_config.pipe.smp_transport), nb);
	return 0;
}

static int _smp_pipe_tx(struct zephyr_smp_transport *zst, struct net_buf *nb)
{
	ARG_UNUSED(zst);

	int ret = 0;
	if (smp_pipe_tx != NULL) {
        uint8_t data[nb->len];
        uint32_t len = nb->len;
        memcpy(data, nb->data, len);
		LOG_DBG("Call pipe tx impl");
		ret = (*smp_pipe_tx)(data, len);
	} else {
		LOG_ERR("Pipe tx not implemented");
		return MGMT_ERR_EUNKNOWN;
	}
	mcumgr_buf_free(nb);

	return ret < 0 ? MGMT_ERR_EINVAL : MGMT_ERR_EOK;
}

static int smp_pipe_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	zephyr_smp_transport_init(&pipe_config.pipe.smp_transport, _smp_pipe_tx, smp_pipe_get_mtu,
				  NULL/*smp_pipe_ud_copy*/, NULL);
	smp_pipe_tx = NULL;
	return MGMT_ERR_EOK;
}

SYS_INIT(smp_pipe_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

int smp_pipe_set_tx(int (*t)(unsigned char *m, unsigned int s))
{
	smp_pipe_tx = t;
}
