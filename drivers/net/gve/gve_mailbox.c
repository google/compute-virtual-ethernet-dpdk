/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 Google LLC
 */
#include <rte_malloc.h>

#include "gve_mailbox.h"
#include "base/gve_osdep.h"
#include "gve_register.h"
#include "gve_ethdev.h"

static int gve_mbx_check_reset_complete(struct gve_priv *priv)
{
	volatile void *reset_status_reg;
	uint32_t reg_val;
	int i;

	reset_status_reg = (void *)((uint64_t)priv->reg_bar0 +
				    GVE_MBX_RESET_STATUS);
	for (i = 0; i < 2000; i++) {
		reg_val = rte_read32(reset_status_reg);

		if (reg_val != 0xFFFFFFFF && (reg_val & 0x1))
			return 0;

		rte_delay_us_sleep(5000);
	}

	PMD_DRV_LOG(ERR, "Mailbox device reset timeout!");
	return -EBUSY;
}

int gve_mbx_reset(struct gve_priv *priv)
{
	volatile void *reset_reg;
	uint32_t reg_val;

	reset_reg = (void *)((uint64_t)priv->reg_bar0 + GVE_MBX_RESET_CTRL);
	reg_val = rte_read32(reset_reg);

	rte_write32(reg_val | BIT(0), reset_reg);
	return gve_mbx_check_reset_complete(priv);
}

static void gve_mbx_free_tx_ring(struct gve_mailbox *mbx)
{
	if (!mbx->tx)
		return;

	rte_free(mbx->tx->bufs);
	gve_free_dma_mem(&mbx->tx->desc_ring);

	free(mbx->tx);
	mbx->tx = NULL;
}

static int gve_mbx_alloc_tx_ring(struct gve_mailbox *mbx)
{
	size_t desc_ring_size;
	int err;

	mbx->tx = calloc(1, sizeof(*mbx->tx));
	if (!mbx->tx) {
		PMD_DRV_LOG(ERR, "Failed to allocate mailbox TX queue struct.");
		return -ENOMEM;
	}

	mbx->tx->q_type = GVE_MBX_QUEUE_TYPE_TX;
	mbx->tx->ring_size = GVE_MBX_DEFAULT_RING_SIZE;

	desc_ring_size = mbx->tx->ring_size * sizeof(struct gve_mbx_desc);
	if (!gve_alloc_dma_mem(&mbx->tx->desc_ring, desc_ring_size)) {
		PMD_DRV_LOG(ERR, "Failed to allocate mailbox TX desc ring.");
		err = -ENOMEM;
		goto free_mbx;
	}

	mbx->tx->bufs = rte_zmalloc("mbx_tx_bufs",
				    mbx->tx->ring_size * sizeof(*mbx->tx->bufs),
				    0);
	if (!mbx->tx->bufs) {
		PMD_DRV_LOG(ERR, "Failed to allocate mailbox TX buffers.");
		err = -ENOMEM;
		goto free_desc_ring;
	}

	return 0;
free_desc_ring:
	gve_free_dma_mem(&mbx->tx->desc_ring);
free_mbx:
	gve_mbx_free_tx_ring(mbx);
	return err;
}

static void gve_mbx_free_rx_rcv_bufs(struct gve_mailbox *mbx)
{
	struct gve_mbx_queue *mbx_rx = mbx->rx;
	int i;

	if (!mbx_rx || !mbx_rx->bufs)
		return;

	for (i = 0; i < mbx_rx->ring_size - 1; ++i)
		gve_free_dma_mem(&mbx_rx->bufs[i]);

	rte_free(mbx_rx->bufs);
	mbx_rx->bufs = NULL;
}

static int gve_mbx_alloc_rx_rcv_bufs(struct gve_mailbox *mbx)
{
	int err;
	int i;

	mbx->rx->bufs = rte_zmalloc("mbx_rx_bufs",
				   mbx->rx->ring_size * sizeof(*mbx->rx->bufs),
				   0);
	if (!mbx->rx->bufs)
		return -ENOMEM;

	for (i = 0; i < mbx->rx->ring_size - 1; ++i) {
		if (!gve_alloc_dma_mem(&mbx->rx->bufs[i], GVE_MBX_BUF_SIZE)) {
			err = -ENOMEM;
			goto err;
		}
	}

	return 0;
err:
	while (i--)
		gve_free_dma_mem(&mbx->rx->bufs[i]);
	return err;
}

static void gve_mbx_free_rx_ring(struct gve_mailbox *mbx)
{
	if (!mbx->rx)
		return;

	gve_mbx_free_rx_rcv_bufs(mbx);
	gve_free_dma_mem(&mbx->rx->desc_ring);

	free(mbx->rx);
	mbx->rx = NULL;
}

static int gve_mbx_alloc_rx_ring(struct gve_mailbox *mbx)
{
	size_t desc_ring_size;
	int err;

	mbx->rx = calloc(1, sizeof(*mbx->rx));
	if (!mbx->rx) {
		PMD_DRV_LOG(ERR, "Failed to allocate mailbox RX queue struct.");
		return -ENOMEM;
	}

	mbx->rx->q_type = GVE_MBX_QUEUE_TYPE_RX;
	mbx->rx->ring_size = GVE_MBX_DEFAULT_RING_SIZE;

	desc_ring_size = mbx->rx->ring_size * sizeof(struct gve_mbx_desc);
	if (!gve_alloc_dma_mem(&mbx->rx->desc_ring, desc_ring_size)) {
		PMD_DRV_LOG(ERR, "Failed to allocate mailbox RX desc ring.");
		err = -ENOMEM;
		goto free_mbx;
	}

	mbx->rx->next_to_post = 0;
	mbx->rx->next_to_clean = 0;

	err = gve_mbx_alloc_rx_rcv_bufs(mbx);
	if (err) {
		err = -ENOMEM;
		goto free_desc_ring;
	}

	return 0;

free_desc_ring:
	gve_free_dma_mem(&mbx->rx->desc_ring);
free_mbx:
	free(mbx->rx);
	mbx->rx = NULL;
	return err;
}

static void gve_mbx_free_msg_queue(struct gve_mailbox *mbx)
{
	struct gve_mbx_msg_queue *msg_queue = mbx->msg_queue;

	if (!msg_queue)
		return;

	if (msg_queue->mbx_msgs) {
		rte_free(msg_queue->mbx_msgs);
		msg_queue->mbx_msgs = NULL;
	}

	if (msg_queue->msg_queue_map.mem) {
		rte_free(msg_queue->msg_queue_map.mem);
		msg_queue->msg_queue_map.mem = NULL;
	}

	free(mbx->msg_queue);
	mbx->msg_queue = NULL;
}

static int gve_mbx_alloc_msg_queue(struct gve_mailbox *mbx)
{
	struct gve_mbx_msg_queue *msg_queue;
	uint32_t msg_queue_map_footprint;
	int err;

	mbx->msg_queue = calloc(1, sizeof(*mbx->msg_queue));
	if (!mbx->msg_queue)
		return -ENOMEM;
	msg_queue = mbx->msg_queue;

	msg_queue->size = GVE_MBX_MSG_QUEUE_LEN;
	msg_queue->msg_timeout_ms = GVE_MBX_DEFAULT_MSG_TIMEOUT_MS;

	msg_queue->mbx_msgs =
		rte_zmalloc("gve_mbx__mbx_msgs",
			    msg_queue->size * sizeof(struct gve_mbx_msg *), 0);
	if (!msg_queue->mbx_msgs) {
		err = -ENOMEM;
		goto err;
	}

	/* Set up message queue bitmap. */
	msg_queue_map_footprint =
		rte_bitmap_get_memory_footprint(msg_queue->size);
	msg_queue->msg_queue_map.mem = rte_zmalloc("gve_mbx__msg_queue_map",
			msg_queue_map_footprint, 0);
	if (!msg_queue->msg_queue_map.mem) {
		PMD_DRV_LOG(ERR,
			    "Failed to alloc bitmap for mailbox message queue.");
		err = -ENOMEM;
		goto err;
	}

	msg_queue->msg_queue_map.bmp =
		rte_bitmap_init_with_all_set(msg_queue->size,
				msg_queue->msg_queue_map.mem,
				msg_queue_map_footprint);
	if (!msg_queue->msg_queue_map.bmp) {
		err = -ENOMEM;
		goto err;
	}

	rte_spinlock_init(&msg_queue->mbx_msg_q_lock);

	return 0;
err:
	gve_mbx_free_msg_queue(mbx);
	return err;
}

static void gve_mbx_free(struct gve_priv *priv)
{
	struct gve_mailbox *mbx = priv->mbx;

	if (!mbx)
		return;

	gve_mbx_free_msg_queue(mbx);
	gve_mbx_free_rx_ring(mbx);
	gve_mbx_free_tx_ring(mbx);

	free(priv->mbx);
	priv->mbx = NULL;
}

static int gve_mbx_alloc(struct gve_priv *priv)
{
	int err;

	priv->mbx = calloc(1, sizeof(struct gve_mailbox));
	if (!priv->mbx) {
		PMD_DRV_LOG(ERR, "Failed to allocate mailbox struct.");
		return -ENOMEM;
	}

	err = gve_mbx_alloc_rx_ring(priv->mbx);
	if (err)
		goto err;


	err = gve_mbx_alloc_tx_ring(priv->mbx);
	if (err)
		goto err;

	err = gve_mbx_alloc_msg_queue(priv->mbx);
	if (err)
		goto err;

	return 0;
err:
	gve_mbx_free(priv);
	return err;
}

static void gve_mbx_post_rx_bufs(struct gve_mailbox *mbx)
{
	uint16_t ntp = mbx->rx->next_to_post;
	struct gve_mbx_desc *desc;
	uint16_t last_post;

	while (((ntp + 1) & mbx->rx->len_mask) != mbx->rx->next_to_clean) {
		desc = GVE_MBX_DESC(mbx->rx, ntp);

		desc->flags = rte_le_to_cpu_16(GVE_MBX_FLAG_BUF |
					       GVE_MBX_FLAG_RD);
		desc->buf_len = GVE_MBX_BUF_SIZE;
		desc->addr_high =
			rte_cpu_to_le_32(mbx->rx->bufs[ntp].pa >> 32);
		desc->addr_low =
			rte_cpu_to_le_32((mbx->rx->bufs[ntp].pa) & 0xFFFFFFFF);

		last_post = ntp;
		ntp = (ntp + 1) & mbx->rx->len_mask;
	}

	/* Update tail doorbell if buffers were posted. */
	if (ntp != mbx->rx->next_to_post) {
		mbx->rx->next_to_post = ntp;
		rte_write32(last_post, &mbx->rx->reg->queue_tail);
	}
}

void gve_mbx_teardown(struct gve_priv *priv)
{
	int err;

	gve_clear_control_plane_ok(priv);

	err = gve_mbx_reset(priv);
	if (err)
		PMD_DRV_LOG(ERR, "Failed to reset in mailbox mode.");

	gve_mbx_free(priv);
}

static void gve_mbx_reg_init(struct gve_mbx_queue *mbx_q, void *bar0)
{
	if (mbx_q->q_type == GVE_MBX_QUEUE_TYPE_RX) {
		mbx_q->reg = (volatile struct gve_mbx_registers *)
			((uint64_t)bar0 + GVE_MBX_RX_BASE);

		mbx_q->len_mask = mbx_q->ring_size - 1;
		mbx_q->len_ena_mask = GVE_MBX_RX_ENABLE_M;
	} else {
		mbx_q->reg = (volatile struct gve_mbx_registers *)
			((uint64_t)bar0 + GVE_MBX_TX_BASE);

		mbx_q->len_mask = mbx_q->ring_size - 1;
		mbx_q->len_ena_mask = GVE_MBX_TX_ENABLE_M;
	}

	rte_write32(0, &mbx_q->reg->queue_tail);
	rte_write32(0, &mbx_q->reg->queue_head);
	rte_write32(mbx_q->desc_ring.pa, &mbx_q->reg->base_addr_low);
	rte_write32(mbx_q->desc_ring.pa >> 32, &mbx_q->reg->base_addr_high);
	rte_write32((mbx_q->ring_size | mbx_q->len_ena_mask),
		    &mbx_q->reg->queue_len);
}

int gve_mbx_init(struct gve_priv *priv)
{
	struct gve_mailbox *mbx;
	int err;

	err = gve_mbx_reset(priv);
	if (err) {
		PMD_DRV_LOG(ERR, "Failed to reset in mailbox mode.");
		return err;
	}

	err = gve_mbx_alloc(priv);
	if (err) {
		PMD_DRV_LOG(ERR, "Failed to allocate mailbox queues.");
		return err;
	}

	mbx = priv->mbx;
	mbx->priv = priv;

	/* Init the mailbox queues */
	gve_mbx_reg_init(mbx->tx, priv->reg_bar0);
	gve_mbx_reg_init(mbx->rx, priv->reg_bar0);

	gve_mbx_post_rx_bufs(mbx);

	gve_set_control_plane_ok(priv);

	return 0;
}

