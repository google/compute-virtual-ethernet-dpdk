/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 Google LLC
 */
#include <rte_alarm.h>
#include <rte_malloc.h>
#include <rte_time.h>

#include "gve_mailbox.h"
#include "base/gve_osdep.h"
#include "gve_register.h"
#include "gve_ethdev.h"
#include "gve_version.h"

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

static void gve_mbx_msg_comp_init(struct gve_mbx_msg *msg)
{
	pthread_mutex_init(&msg->comp.mutex, NULL);
	pthread_cond_init(&msg->comp.cond, NULL);

	pthread_mutex_lock(&msg->comp.mutex);
	msg->status = GVE_MBX_STATUS_UNSET;
	pthread_mutex_unlock(&msg->comp.mutex);
}

static int gve_mbx_wait_for_completion(struct gve_mbx_msg *msg,
				       uint64_t timeout_ms)
{
	struct timespec ts;
	int err = 0;
	int ret = 0;

	clock_gettime(CLOCK_REALTIME, &ts);
	ts = rte_ns_to_timespec(rte_timespec_to_ns(&ts) + timeout_ms * 1000000);

	pthread_mutex_lock(&msg->comp.mutex);
	while (ret || msg->status == GVE_MBX_STATUS_UNSET) {
		ret = pthread_cond_timedwait(&msg->comp.cond, &msg->comp.mutex,
					     &ts);
		/* Only exit on timeout error. */
		if (ret == ETIMEDOUT) {
			err = ETIMEDOUT;
			goto ret;
		}
	}

	err = gve_mbx_get_err_from_status(msg->status);
ret:
	pthread_mutex_unlock(&msg->comp.mutex);
	return -err;
}

static void gve_mbx_msg_complete(struct gve_mbx_msg *msg, int status)
{
	pthread_mutex_lock(&msg->comp.mutex);
	msg->status = status;
	pthread_cond_signal(&msg->comp.cond);
	pthread_mutex_unlock(&msg->comp.mutex);
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

static void gve_mbx_process_msg_completion(struct gve_mailbox *mbx,
					   struct gve_mbx_desc *recv_desc)
{
	struct gve_mbx_msg_queue *msg_queue = mbx->msg_queue;
	struct gve_mbx_msg *mbx_msg;
	uint16_t cookie, index;
	uint32_t opcode;
	int status;

	status = rte_le_to_cpu_16(recv_desc->cmd_retval);
	cookie = rte_le_to_cpu_16(recv_desc->cmd_cookie);
	opcode = rte_le_to_cpu_32(recv_desc->cmd_opcode);

	index = cookie & GVE_MBX_COOKIE_INDEX_MASK;
	if (index >= msg_queue->size) {
		return;
	}

	rte_spinlock_lock(&msg_queue->mbx_msg_q_lock);
	mbx_msg = msg_queue->mbx_msgs[index];
	if (rte_bitmap_get(msg_queue->msg_queue_map.bmp, index) || !mbx_msg) {
		PMD_DRV_LOG(ERR, "No pending mailbox message for response: cookie=0x%x, opcode=0x%x",
			    cookie, opcode);
		return;
	}

	if (mbx_msg->sw_cookie != cookie) {
		PMD_DRV_LOG(ERR, "Mailbox cookie mismatch: expected 0x%x, received 0x%x.",
			    mbx_msg->sw_cookie, cookie);
		return;
	}

	if (mbx_msg->opcode != opcode) {
		PMD_DRV_LOG(ERR, "Mailbox opcode mismatch: expected 0x%x, recieved 0x%x.",
			    mbx_msg->opcode, opcode);
		return;
	}

	gve_mbx_msg_complete(mbx_msg, status);
	rte_spinlock_unlock(&msg_queue->mbx_msg_q_lock);
}

static int
gve_mbx_process_negotiate_caps_resp(struct gve_mailbox *mbx,
				     struct gve_dma_mem *recv_msg)
{
	struct gve_mbx_negotiate_caps_resp *resp =
		(struct gve_mbx_negotiate_caps_resp *)recv_msg->va;
	struct gve_priv *priv = mbx->priv;
	uint64_t negotiated_caps;
	uint32_t irq_db_offset;
	uint8_t bar_idx;

	if (rte_le_to_cpu_32(resp->msg_version) != GVE_MBX_CAPS_MSG_V1) {
		PMD_DRV_LOG(ERR, "Unsupported negotiate caps response version: %u",
			    rte_le_to_cpu_32(resp->msg_version));
		return -EINVAL;
	}

	if (rte_le_to_cpu_32(resp->msg_size) < sizeof(*resp)) {
		PMD_DRV_LOG(ERR, "Invalid negotiate caps response size: %u",
			    rte_le_to_cpu_32(resp->msg_size));
		return -EINVAL;
	}

	bar_idx = resp->db_bar;
	priv->db_bar = priv->pci_dev->mem_resource[bar_idx].addr;
	if (priv->db_bar == NULL) {
		PMD_DRV_LOG(ERR, "Failed to map doorbell BAR %u", bar_idx);
		return -ENOMEM;
	}

	irq_db_offset = rte_le_to_cpu_32(resp->mbx_irq_db_offset);
	if (priv->db_bar != NULL)
		mbx->irq_db = (rte_be32_t __iomem *)((uint8_t *)priv->db_bar + irq_db_offset);

	priv->max_nb_txq = rte_le_to_cpu_16(resp->max_tx_num_queues);
	priv->max_nb_rxq = rte_le_to_cpu_16(resp->max_rx_num_queues);

	priv->default_tx_num_queues = rte_le_to_cpu_16(resp->default_tx_num_queues);
	priv->default_rx_num_queues = rte_le_to_cpu_16(resp->default_rx_num_queues);

	priv->max_mtu = rte_le_to_cpu_16(resp->max_mtu);

	rte_memcpy(priv->dev_addr.addr_bytes, resp->mac, RTE_ETHER_ADDR_LEN);

	priv->default_tx_desc_cnt = rte_le_to_cpu_16(resp->default_tx_ring_size);
	priv->default_rx_desc_cnt = rte_le_to_cpu_16(resp->default_rx_ring_size);

	priv->max_tx_desc_cnt = rte_le_to_cpu_16(resp->max_tx_ring_size);
	priv->max_rx_desc_cnt = rte_le_to_cpu_16(resp->max_rx_ring_size);

	priv->min_tx_desc_cnt = rte_le_to_cpu_16(resp->min_tx_ring_size);
	priv->min_rx_desc_cnt = rte_le_to_cpu_16(resp->min_rx_ring_size);

	negotiated_caps = rte_le_to_cpu_64(resp->negotiated_caps);

	mbx->msg_queue->msg_timeout_ms = rte_le_to_cpu_16(resp->mbx_response_timeout_ms);
	priv->num_ntfy_blks = rte_le_to_cpu_16(resp->num_msix_vectors);
	priv->tx_queue_watchdog_timeout_ms = rte_le_to_cpu_16(resp->tx_queue_watchdog_timeout_ms);

	priv->max_packet_buffer_size = rte_le_to_cpu_16(resp->max_packet_buffer_size);
	priv->max_header_buffer_size = rte_le_to_cpu_16(resp->max_header_buffer_size);

	priv->hash_key_size = rte_le_to_cpu_16(resp->hash_key_size);
	priv->hash_lut_size = rte_le_to_cpu_16(resp->hash_lut_size);

	PMD_DRV_LOG(INFO,
		    "GVE Mailbox Caps: negotiated_caps=0x%" PRIx64 ", db_bar=%u, mbx_irq_db_offset=%u, "
		    "max_txq=%u, max_rxq=%u, default_tx_q=%u, default_rx_q=%u, max_mtu=%u, "
		    "tx_ring_sizes(min/default/max)=%u/%u/%u, "
		    "rx_ring_sizes(min/default/max)=%u/%u/%u, "
		    "mbx_timeout_ms=%u, num_msix=%u, watchdog_timeout_ms=%u, "
		    "max_pkt_buf_size=%u, max_hdr_buf_size=%u, hash_key_size=%u, hash_lut_size=%u",
		    negotiated_caps, bar_idx, irq_db_offset,
		    priv->max_nb_txq, priv->max_nb_rxq,
		    priv->default_tx_num_queues, priv->default_rx_num_queues, priv->max_mtu,
		    priv->min_tx_desc_cnt, priv->default_tx_desc_cnt, priv->max_tx_desc_cnt,
		    priv->min_rx_desc_cnt, priv->default_rx_desc_cnt, priv->max_rx_desc_cnt,
		    mbx->msg_queue->msg_timeout_ms, priv->num_ntfy_blks,
		    priv->tx_queue_watchdog_timeout_ms, priv->max_packet_buffer_size,
		    priv->max_header_buffer_size, priv->hash_key_size, priv->hash_lut_size);

	PMD_DRV_LOG(INFO, "MAC addr: " RTE_ETHER_ADDR_PRT_FMT,
		    RTE_ETHER_ADDR_BYTES(&priv->dev_addr));

	return 0;
}

static int
gve_mbx_process_get_interrupt_dbs_resp(struct gve_mailbox *mbx,
					struct gve_dma_mem *recv_msg)
{
	struct gve_mbx_get_interrupt_dbs_resp *resp =
		(struct gve_mbx_get_interrupt_dbs_resp *)recv_msg->va;
	struct gve_priv *priv = mbx->priv;
	uint16_t num_vecs = rte_le_to_cpu_16(resp->num_vecs);
	uint16_t i;

	if (num_vecs == 0) {
		PMD_DRV_LOG(ERR, "No interrupt vectors returned in GET_INTERRUPT_DBS");
		return -EINVAL;
	}

	if (priv->irq_db_offsets == NULL) {
		priv->irq_db_offsets = rte_zmalloc("gve_irq_db_offsets",
						   priv->num_ntfy_blks * sizeof(uint32_t),
						   0);
		if (priv->irq_db_offsets == NULL) {
			PMD_DRV_LOG(ERR, "Failed to allocate memory for irq_db_offsets");
			return -ENOMEM;
		}
	}

	for (i = 0; i < num_vecs; i++) {
		uint16_t vec_id = 1 + i;
		if (vec_id < priv->num_ntfy_blks) {
			priv->irq_db_offsets[vec_id] =
				rte_le_to_cpu_32(resp->info[i].irq_db_offset);
		}
	}

	return 0;
}

static int
gve_mbx_process_get_ptype_map_resp(struct gve_mailbox *mbx,
				   struct gve_dma_mem *recv_msg)
{
	struct gve_mbx_get_ptype_map_resp *resp =
		(struct gve_mbx_get_ptype_map_resp *)recv_msg->va;
	struct gve_priv *priv = mbx->priv;
	struct gve_ptype_lut *ptype_lut = priv->ptype_lut_dqo;
	int i;

	if (!ptype_lut)
		return -EINVAL;

	for (i = 0; i < GVE_NUM_PTYPES; i++) {
		ptype_lut->ptypes[i].l3_type = resp->ptypes[i].l3_type;
		ptype_lut->ptypes[i].l4_type = resp->ptypes[i].l4_type;
	}

	return 0;
}

static int
gve_mbx_process_config_tx_queues_resp(struct gve_mailbox *mbx,
				      struct gve_dma_mem *recv_msg)
{
	struct gve_mbx_config_tx_queues_resp *resp =
		(struct gve_mbx_config_tx_queues_resp *)recv_msg->va;
	struct gve_priv *priv = mbx->priv;
	uint16_t num_queues = rte_le_to_cpu_16(resp->num_queues);
	uint32_t tail_db_offset;
	uint32_t q_id;
	uint16_t i;

	for (i = 0; i < num_queues; i++) {
		q_id = rte_le_to_cpu_32(resp->queues[i].queue_id);
		tail_db_offset = rte_le_to_cpu_32(resp->queues[i].tail_db_offset);
		if (q_id < priv->max_nb_txq && priv->txqs[q_id] != NULL) {
			priv->txqs[q_id]->qtx_tail =
				(rte_be32_t __iomem *)((uint8_t *)priv->db_bar + tail_db_offset);
		}
	}
	return 0;
}

static int
gve_mbx_process_config_rx_queues_resp(struct gve_mailbox *mbx,
				      struct gve_dma_mem *recv_msg)
{
	struct gve_mbx_config_rx_queues_resp *resp =
		(struct gve_mbx_config_rx_queues_resp *)recv_msg->va;
	struct gve_priv *priv = mbx->priv;
	uint16_t num_queues = rte_le_to_cpu_16(resp->num_queues);
	uint32_t tail_db_offset;
	uint32_t q_id;
	uint16_t i;

	for (i = 0; i < num_queues; i++) {
		q_id = rte_le_to_cpu_32(resp->queues[i].queue_id);
		tail_db_offset = rte_le_to_cpu_32(resp->queues[i].tail_db_offset);
		if (q_id < priv->max_nb_rxq && priv->rxqs[q_id] != NULL) {
			priv->rxqs[q_id]->qrx_tail =
				(rte_be32_t __iomem *)((uint8_t *)priv->db_bar + tail_db_offset);
		}
	}
	return 0;
}

static int
gve_mbx_process_report_link_status_resp(struct gve_mailbox *mbx,
					struct gve_dma_mem *recv_msg)
{
	struct gve_mbx_report_link_status_resp *resp =
		(struct gve_mbx_report_link_status_resp *)recv_msg->va;
	struct gve_priv *priv = mbx->priv;

	priv->link_status = resp->link_status;
	priv->link_speed = rte_le_to_cpu_64(resp->link_speed);
	return 0;
}

static int gve_mbx_process_msg(struct  gve_mailbox *mbx, uint32_t opcode,
			       struct gve_dma_mem *recv_msg)
{
	int err = 0;

	switch (opcode) {
	case GVE_MBX_NEGOTIATE_CAPABILITIES:
		return gve_mbx_process_negotiate_caps_resp(mbx, recv_msg);
	case GVE_MBX_GET_INTERRUPT_DBS:
		return gve_mbx_process_get_interrupt_dbs_resp(mbx, recv_msg);
	case GVE_MBX_GET_PTYPE_MAP:
		return gve_mbx_process_get_ptype_map_resp(mbx, recv_msg);
	case GVE_MBX_REPORT_LINK_STATUS:
		return gve_mbx_process_report_link_status_resp(mbx, recv_msg);
	case GVE_MBX_CONFIG_TX_QUEUES:
		return gve_mbx_process_config_tx_queues_resp(mbx, recv_msg);
	case GVE_MBX_CONFIG_RX_QUEUES:
		return gve_mbx_process_config_rx_queues_resp(mbx, recv_msg);
	case GVE_MBX_ENABLE_TX_QUEUES:
	case GVE_MBX_ENABLE_RX_QUEUES:
	case GVE_MBX_DISABLE_TX_QUEUES:
	case GVE_MBX_DISABLE_RX_QUEUES:
		return 0;
	default:
		err = -EBADMSG;
	}

	return err;
}

static int gve_mbx_receive_msg(struct gve_mailbox *mbx)
{
	struct gve_priv *priv = mbx->priv;
	struct gve_mbx_desc *recv_desc;
	struct gve_dma_mem *recv_msg;
	uint16_t ntc, flags;
	uint32_t opcode;
	int err = 0;

	if (!gve_get_control_plane_ok(priv))
		return -EIO;

	rte_spinlock_lock(&mbx->rx->q_lock);

	ntc = mbx->rx->next_to_clean;
	recv_desc = GVE_MBX_DESC(mbx->rx, ntc);
	flags = rte_le_to_cpu_16(recv_desc->flags);

	/* Check if desc is marked as done. */
	if (!(flags & GVE_MBX_FLAG_DD)) {
		err = -EAGAIN;
		goto unlock_and_return;
	}

	recv_msg = &mbx->rx->bufs[ntc];

	opcode = rte_le_to_cpu_32(recv_desc->cmd_opcode);
	if (opcode == GVE_MBX_EVENT) {
		if (recv_desc->cmd_retval != GVE_MBX_STATUS_PASSED) {
			PMD_DRV_LOG(ERR, "Unexpected error status in MBX_EVENT notification: 0x%x",
				    recv_desc->cmd_retval);
			err = -EBADMSG;
			goto update_tail;
		}
		goto update_tail;
	}

	if (recv_desc->cmd_retval != GVE_MBX_STATUS_PASSED) {
		gve_mbx_process_msg_completion(mbx, recv_desc);
		err = -EBADMSG;
		goto update_tail;
	}

	err = gve_mbx_process_msg(mbx, opcode, recv_msg);
	gve_mbx_process_msg_completion(mbx, recv_desc);

update_tail:
	memset(recv_desc, 0, sizeof(*recv_desc));

	ntc = (ntc + 1) & mbx->rx->len_mask;
	mbx->rx->next_to_clean = ntc;

unlock_and_return:
	rte_spinlock_unlock(&mbx->rx->q_lock);
	return err;
}

static void gve_mbx_rx_poll(struct gve_mailbox *mbx)
{
	struct gve_mbx_queue *rx = mbx->rx;
	uint16_t posted_bufs;
	int err;

	do {
		err = gve_mbx_receive_msg(mbx);
		if (err == -EIO) {
			PMD_DRV_LOG(ERR, "Mailbox queue not set up.");
			break;
		} else if (err && err != -EAGAIN) {
			PMD_DRV_LOG(ERR, "Failed to receive mbx message: %d",
				    err);
		}

		posted_bufs = (rx->next_to_post - rx->next_to_clean) & rx->len_mask;
		if (posted_bufs < GVE_MBX_MSG_QUEUE_LEN)
			gve_mbx_post_rx_bufs(mbx);
	} while (err != -EAGAIN);

	/* Post buffers and update tail. */
	gve_mbx_post_rx_bufs(mbx);
}

static void gve_mbx_clean_send_queue(struct gve_mailbox *mbx)
{
	struct gve_mbx_desc *desc;
	uint16_t ntc;
	uint32_t i;

	ntc = mbx->tx->next_to_clean;

	/* Attempt to clean the entire queue */
	for (i = 0; i < mbx->tx->len_mask; i++) {
		/* should clean from ntc */
		desc = GVE_MBX_DESC(mbx->tx, ntc);

		/* check if desc is marked as done */
		if (!(rte_le_to_cpu_16(desc->flags) & GVE_MBX_FLAG_DD))
			break;

		gve_free_dma_mem(&mbx->tx->bufs[ntc]);
		memset(desc, 0, sizeof(*desc));

		ntc = (ntc + 1) & mbx->tx->len_mask;
	}

	mbx->tx->next_to_clean = ntc;
}

static int gve_mbx_send_msg(struct gve_mailbox *mbx, uint32_t opcode,
			    uint16_t msg_bytes, uint8_t *msg, uint16_t cookie)
{
	struct gve_mbx_desc *send_desc;
	struct gve_dma_mem *send_msg;
	uint16_t flags;

	gve_mbx_clean_send_queue(mbx);

	/* Allocate DMA region for message. */
	send_msg = &mbx->tx->bufs[mbx->tx->next_to_post];
	if (!gve_alloc_dma_mem(send_msg, GVE_MBX_BUF_SIZE)) {
		return -ENOMEM;
	}
	send_msg->size = GVE_MBX_BUF_SIZE;

	/* Fill out descriptor. */
	send_desc = GVE_MBX_DESC(mbx->tx, mbx->tx->next_to_post);
	send_desc->destination = rte_cpu_to_le_16(0x0801); /* send message to CP */
	send_desc->pfid_vfid = 0;
	send_desc->buf_len = rte_cpu_to_le_16(msg_bytes);
	send_desc->cmd_opcode = rte_cpu_to_le_32(opcode);
	send_desc->cmd_cookie = rte_cpu_to_le_16(cookie);
	send_desc->addr_high =
			rte_cpu_to_le_32(send_msg->pa >> 32);
	send_desc->addr_low =
			rte_cpu_to_le_32((send_msg->pa) & 0xFFFFFFFF);

	/* Set required flags. */
	flags = GVE_MBX_FLAG_BUF | GVE_MBX_FLAG_RD;
	send_desc->flags = rte_cpu_to_le_16(flags);

	/* Copy message into DMA region. */
	if (msg && msg_bytes)
		rte_memcpy(send_msg->va, msg, msg_bytes);

	mbx->tx->next_to_post = (mbx->tx->next_to_post + 1) & mbx->tx->len_mask;
	rte_write32(mbx->tx->next_to_post, &mbx->tx->reg->queue_tail);
	return 0;
}

static bool gve_mbx_in_reset(struct gve_mailbox *mbx)
{
	if (!mbx->rx)
		return true;

	return !(rte_read32(&mbx->rx->reg->queue_len) & GVE_MBX_RX_LEN_M);
}

static int gve_mbx_get_free_send_idx(struct gve_mbx_msg_queue *mbx_msg_queue,
				     u16 *index)
{
	uint64_t slab = 0;
	uint32_t idx = 0;
	int ret;

	ret = rte_bitmap_scan(mbx_msg_queue->msg_queue_map.bmp, &idx, &slab);
	if (!ret)
		return -EBUSY;
	idx += rte_bsf64(slab);

	*index = idx;
	return 0;
}

static int gve_mbx_send_msg_wait(struct gve_mailbox *mbx, uint32_t opcode,
				 uint16_t msg_bytes, uint8_t *msg)
{
	struct gve_mbx_msg *mbx_msg;
	u16 cookie, index = 0;
	int err;

	if (gve_mbx_in_reset(mbx)) {
		PMD_DRV_LOG(ERR,
			    "Mailbox reset detected, cannot send mbx msg");
		return -EIO;
	}

	if (!gve_get_control_plane_ok(mbx->priv))
		return -EIO;

	mbx_msg = rte_zmalloc(NULL, sizeof(*mbx_msg), 0);
	if (!mbx_msg)
		return -ENOMEM;

	gve_mbx_msg_comp_init(mbx_msg);

	rte_spinlock_lock(&mbx->msg_queue->mbx_msg_q_lock);

	if (gve_mbx_get_free_send_idx(mbx->msg_queue, &index)) {
		err = -EBUSY;
		goto err_unlock;
	}

	/* create sw cookie */
	mbx->msg_queue->counter++;
	cookie = BIT(15) |
		 ((mbx->msg_queue->counter & 0x1FF) << GVE_MBX_COOKIE_INDEX_BITS) |
		 (index & GVE_MBX_COOKIE_INDEX_MASK);

	mbx_msg->sw_cookie = cookie;
	mbx_msg->opcode = opcode;

	rte_bitmap_clear(mbx->msg_queue->msg_queue_map.bmp, index);
	mbx->msg_queue->mbx_msgs[index] = mbx_msg;


	err = gve_mbx_send_msg(mbx, opcode, msg_bytes, msg, cookie);
	if (err)
		goto err_unmap_cookie;

	rte_spinlock_unlock(&mbx->msg_queue->mbx_msg_q_lock);

	err = gve_mbx_wait_for_completion(mbx_msg,
					  mbx->msg_queue->msg_timeout_ms);

	rte_spinlock_lock(&mbx->msg_queue->mbx_msg_q_lock);
err_unmap_cookie:
	rte_bitmap_set(mbx->msg_queue->msg_queue_map.bmp, index);
	mbx->msg_queue->mbx_msgs[index] = NULL;

err_unlock:
	rte_spinlock_unlock(&mbx->msg_queue->mbx_msg_q_lock);
	pthread_mutex_destroy(&mbx_msg->comp.mutex);
	pthread_cond_destroy(&mbx_msg->comp.cond);
	rte_free(mbx_msg);
	return err;
}

static void gve_mbx_task(void *arg)
{
	struct gve_mailbox *mbx = arg;

	gve_mbx_rx_poll(mbx);
	rte_eal_alarm_set(300000, gve_mbx_task, mbx);
}

void gve_mbx_teardown(struct gve_priv *priv)
{
	int err;

	gve_clear_control_plane_ok(priv);

	rte_eal_alarm_cancel(gve_mbx_task, priv->mbx);

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

	rte_eal_alarm_set(300000, gve_mbx_task, mbx);

	gve_set_control_plane_ok(priv);

	return 0;
}

int
gve_mbx_get_device_properties(struct gve_priv *priv)
{
	struct gve_mbx_negotiate_caps_req req;
	int err;

	memset(&req, 0, sizeof(req));
	req.msg_version = rte_cpu_to_le_32(GVE_MBX_CAPS_MSG_V1);
	req.msg_size = rte_cpu_to_le_32(sizeof(req));
	req.supported_caps = rte_cpu_to_le_64(GVE_MBX_CAP_DQO_RDA);
	req.os_type = GVE_OS_TYPE_DPDK;
	req.driver_major = GVE_VERSION_MAJOR;
	req.driver_minor = GVE_VERSION_MINOR;
	req.driver_sub = GVE_VERSION_SUB;
	req.os_version_major = rte_cpu_to_le_32(DPDK_VERSION_MAJOR);
	req.os_version_minor = rte_cpu_to_le_32(DPDK_VERSION_MINOR);
	req.os_version_sub = rte_cpu_to_le_32(DPDK_VERSION_SUB);

	populate_driver_version_strings((char *)req.os_version_str,
					 (char *)req.driver_version_str);

	err = gve_mbx_send_msg_wait(priv->mbx, GVE_MBX_NEGOTIATE_CAPABILITIES,
				    sizeof(req), (uint8_t *)&req);
	if (err)
		PMD_DRV_LOG(ERR, "Failed to negotiate capabilities over mailbox: %d", err);

	return err;
}

int
gve_mbx_get_interrupt_dbs(struct gve_priv *priv)
{
	struct gve_mbx_get_interrupt_dbs_req req;
	int err;

	/* TODO: Support reading in more vectors than can fit into a single response. */

	if (priv->num_ntfy_blks <= 1)
		return 0;

	memset(&req, 0, sizeof(req));
	req.start_msix_index = rte_cpu_to_le_16(1);
	req.num_vecs = rte_cpu_to_le_16(priv->num_ntfy_blks - 1);

	err = gve_mbx_send_msg_wait(priv->mbx, GVE_MBX_GET_INTERRUPT_DBS,
				    sizeof(req), (uint8_t *)&req);
	if (err)
		PMD_DRV_LOG(ERR, "Failed to get interrupt doorbells over mailbox: %d", err);

	return err;
}

int
gve_mbx_get_ptype_map(struct gve_priv *priv)
{
	int err;

	err = gve_mbx_send_msg_wait(priv->mbx, GVE_MBX_GET_PTYPE_MAP, 0, NULL);
	if (err)
		PMD_DRV_LOG(ERR, "Failed to get ptype map over mailbox: %d", err);

	return err;
}

int
gve_mbx_report_link_speed(struct gve_priv *priv)
{
	int err;

	err = gve_mbx_send_msg_wait(priv->mbx, GVE_MBX_REPORT_LINK_STATUS, 0, NULL);
	if (err)
		PMD_DRV_LOG(ERR, "Failed to report link speed over mailbox: %d", err);

	return err;
}

static int
gve_mbx_enable_tx_queues(struct gve_priv *priv, uint32_t num_queues)
{
	struct gve_mbx_enable_tx_queues_req *req;
	struct gve_dma_mem mem;
	uint32_t bytes;
	uint16_t i;
	int err;

	bytes = sizeof(*req) + num_queues * sizeof(uint16_t);
	if (gve_alloc_dma_mem(&mem, bytes) == NULL)
		return -ENOMEM;

	req = (struct gve_mbx_enable_tx_queues_req *)mem.va;
	memset(req, 0, bytes);
	req->num_queues = rte_cpu_to_le_16(num_queues);

	for (i = 0; i < num_queues; i++)
		req->queue_ids[i] = rte_cpu_to_le_16(priv->txqs[i]->queue_id);

	err = gve_mbx_send_msg_wait(priv->mbx, GVE_MBX_ENABLE_TX_QUEUES,
				    bytes, (uint8_t *)req);
	gve_free_dma_mem(&mem);
	if (err)
		PMD_DRV_LOG(ERR, "Failed to enable tx queues over mailbox: %d", err);

	return err;
}

int
gve_mbx_create_tx_queues(struct gve_priv *priv, uint32_t num_queues)
{
	struct gve_mbx_config_tx_queues_req *config_req;
	struct gve_dma_mem config_mem;
	uint32_t config_bytes;
	struct gve_tx_queue *txq;
	uint32_t q_id;
	uint16_t i;
	int err;

	config_bytes = sizeof(*config_req) + num_queues * sizeof(struct gve_mbx_tx_queue_info);
	if (gve_alloc_dma_mem(&config_mem, config_bytes) == NULL)
		return -ENOMEM;

	config_req = (struct gve_mbx_config_tx_queues_req *)config_mem.va;
	memset(config_req, 0, config_bytes);
	config_req->num_queues = rte_cpu_to_le_16(num_queues);

	for (i = 0; i < num_queues; i++) {
		txq = priv->txqs[i];
		q_id = txq->queue_id;

		config_req->queues[i].queue_id = rte_cpu_to_le_32(q_id);
		config_req->queues[i].msix_vector = rte_cpu_to_le_16(i + 1);
		config_req->queues[i].queue_page_list_id =
			rte_cpu_to_le_32(GVE_RAW_ADDRESSING_QPL_ID);
		config_req->queues[i].ring_base_addr = rte_cpu_to_le_64(txq->tx_ring_phys_addr);
		config_req->queues[i].comp_ring_base_addr =
			rte_cpu_to_le_64(txq->compl_ring_phys_addr);
		config_req->queues[i].ring_size = rte_cpu_to_le_16(txq->nb_tx_desc);
		config_req->queues[i].comp_ring_size = rte_cpu_to_le_16(txq->nb_complq_desc);
	}

	err = gve_mbx_send_msg_wait(priv->mbx, GVE_MBX_CONFIG_TX_QUEUES,
				    config_bytes, (uint8_t *)config_req);
	gve_free_dma_mem(&config_mem);
	if (err) {
		PMD_DRV_LOG(ERR, "Failed to config tx queues over mailbox: %d", err);
		return err;
	}

	return gve_mbx_enable_tx_queues(priv, num_queues);
}

int
gve_mbx_destroy_tx_queues(struct gve_priv *priv, uint32_t num_queues)
{
	struct gve_mbx_disable_queues_req *req;
	struct gve_dma_mem mem;
	uint32_t bytes;
	uint16_t i;
	int err;

	bytes = sizeof(*req) + num_queues * sizeof(uint16_t);
	if (gve_alloc_dma_mem(&mem, bytes) == NULL)
		return -ENOMEM;

	req = (struct gve_mbx_disable_queues_req *)mem.va;
	memset(req, 0, bytes);
	req->num_queues = rte_cpu_to_le_16(num_queues);

	for (i = 0; i < num_queues; i++)
		req->queue_ids[i] = rte_cpu_to_le_16(priv->txqs[i]->queue_id);

	err = gve_mbx_send_msg_wait(priv->mbx, GVE_MBX_DISABLE_TX_QUEUES,
				    bytes, (uint8_t *)req);
	gve_free_dma_mem(&mem);
	if (err)
		PMD_DRV_LOG(ERR, "Failed to disable tx queues over mailbox: %d", err);

	return err;
}

static int
gve_mbx_enable_rx_queues(struct gve_priv *priv, uint32_t num_queues)
{
	struct gve_mbx_enable_rx_queues_req *req;
	struct gve_dma_mem mem;
	uint32_t bytes;
	uint16_t i;
	int err;

	bytes = sizeof(*req) + num_queues * sizeof(uint16_t);
	if (gve_alloc_dma_mem(&mem, bytes) == NULL)
		return -ENOMEM;

	req = (struct gve_mbx_enable_rx_queues_req *)mem.va;
	memset(req, 0, bytes);
	req->num_queues = rte_cpu_to_le_16(num_queues);

	for (i = 0; i < num_queues; i++)
		req->queue_ids[i] = rte_cpu_to_le_16(priv->rxqs[i]->queue_id);

	err = gve_mbx_send_msg_wait(priv->mbx, GVE_MBX_ENABLE_RX_QUEUES,
				    bytes, (uint8_t *)req);
	gve_free_dma_mem(&mem);
	if (err)
		PMD_DRV_LOG(ERR, "Failed to enable rx queues over mailbox: %d", err);

	return err;
}

int
gve_mbx_create_rx_queues(struct gve_priv *priv, uint32_t num_queues)
{
	struct gve_mbx_config_rx_queues_req *config_req;
	struct gve_dma_mem config_mem;
	uint32_t config_bytes;
	struct gve_rx_queue *rxq;
	uint32_t q_id;
	uint16_t i;
	int err;

	config_bytes = sizeof(*config_req) + num_queues * sizeof(struct gve_mbx_rx_queue_info);
	if (gve_alloc_dma_mem(&config_mem, config_bytes) == NULL)
		return -ENOMEM;

	config_req = (struct gve_mbx_config_rx_queues_req *)config_mem.va;
	memset(config_req, 0, config_bytes);
	config_req->num_queues = rte_cpu_to_le_16(num_queues);

	for (i = 0; i < num_queues; i++) {
		rxq = priv->rxqs[i];
		q_id = rxq->queue_id;

		config_req->queues[i].queue_id = rte_cpu_to_le_32(q_id);
		config_req->queues[i].msix_vector = rte_cpu_to_le_16(i + 1);
		config_req->queues[i].queue_page_list_id =
			rte_cpu_to_le_32(GVE_RAW_ADDRESSING_QPL_ID);
		config_req->queues[i].ring_base_addr = rte_cpu_to_le_64(rxq->compl_ring_phys_addr);
		config_req->queues[i].data_ring_base_addr =
			rte_cpu_to_le_64(rxq->rx_ring_phys_addr);
		config_req->queues[i].ring_size = rte_cpu_to_le_16(rxq->nb_rx_desc);
		config_req->queues[i].data_ring_size = rte_cpu_to_le_16(rxq->nb_rx_desc);
		config_req->queues[i].packet_buf_size = rte_cpu_to_le_16(rxq->rx_buf_len);
	}

	err = gve_mbx_send_msg_wait(priv->mbx, GVE_MBX_CONFIG_RX_QUEUES,
				    config_bytes, (uint8_t *)config_req);
	gve_free_dma_mem(&config_mem);
	if (err) {
		PMD_DRV_LOG(ERR, "Failed to config rx queues over mailbox: %d", err);
		return err;
	}

	return gve_mbx_enable_rx_queues(priv, num_queues);
}

int
gve_mbx_destroy_rx_queues(struct gve_priv *priv, uint32_t num_queues)
{
	struct gve_mbx_disable_queues_req *req;
	struct gve_dma_mem mem;
	uint32_t bytes;
	uint16_t i;
	int err;

	bytes = sizeof(*req) + num_queues * sizeof(uint16_t);
	if (gve_alloc_dma_mem(&mem, bytes) == NULL)
		return -ENOMEM;

	req = (struct gve_mbx_disable_queues_req *)mem.va;
	memset(req, 0, bytes);
	req->num_queues = rte_cpu_to_le_16(num_queues);

	for (i = 0; i < num_queues; i++)
		req->queue_ids[i] = rte_cpu_to_le_16(priv->rxqs[i]->queue_id);

	err = gve_mbx_send_msg_wait(priv->mbx, GVE_MBX_DISABLE_RX_QUEUES,
				    bytes, (uint8_t *)req);
	gve_free_dma_mem(&mem);
	if (err)
		PMD_DRV_LOG(ERR, "Failed to disable rx queues over mailbox: %d", err);

	return err;
}
