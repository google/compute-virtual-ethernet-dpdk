/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 Google LLC
 */
#include <rte_spinlock.h>

#include "base/gve_osdep.h"
#include "base/gve.h"

/* Mailbox Queue defines */
#define GVE_MBX_RESET_CTRL		0x0840700C
#define GVE_MBX_RESET_STATUS		0x08407008

#define GVE_MBX_RX_BASE			0x08400000
#define GVE_MBX_TX_BASE			(GVE_MBX_RX_BASE + 0x14)

#define GVE_MBX_RX_LEN_M		RTE_GENMASK32(12, 0)
#define GVE_MBX_RX_ENABLE_M		BIT(31)
#define GVE_MBX_RX_HEAD_M		RTE_GENMASK32(12, 0)

#define GVE_MBX_TX_LEN_M		RTE_GENMASK32(9, 0)
#define GVE_MBX_TX_ENABLE_M		BIT(31)
#define GVE_MBX_TX_HEAD_M		RTE_GENMASK32(9, 0)

#define GVE_MBX_DEFAULT_RING_SIZE	64
/* Length of msg queue < mbx queue to allow for async messages from device */
#define GVE_MBX_MSG_QUEUE_LEN		48

enum gve_mbx_caps_msg_version {
	GVE_MBX_CAPS_MSG_V1 = 1,
};

enum gve_mbx_caps {
	GVE_MBX_CAP_DQO_RDA		= BIT(0),
};

#define GVE_OS_TYPE_DPDK		5

#define GVE_MBX_BUF_SIZE		4906

#define GVE_MBX_DEFAULT_MSG_TIMEOUT_MS	10000

/* GVE Mailbox descriptor flags */
#define GVE_MBX_FLAG_DD_S		0
#define GVE_MBX_FLAG_ERR_S		2
#define GVE_MBX_FLAG_RD_S               10
#define GVE_MBX_FLAG_BUF_S              12

#define GVE_MBX_FLAG_DD			BIT(GVE_MBX_FLAG_DD_S)	/* 0x1 */
#define GVE_MBX_FLAG_ERR		BIT(GVE_MBX_FLAG_ERR_S) /* 0x4 */
#define GVE_MBX_FLAG_RD                 BIT(GVE_MBX_FLAG_RD_S)  /* 0x400  */
#define GVE_MBX_FLAG_BUF                BIT(GVE_MBX_FLAG_BUF_S) /* 0x1000 */

#define GVE_MBX_DESC(R, i) \
	(&(((struct gve_mbx_desc *)((R)->desc_ring.va))[i]))

#define GVE_MBX_COOKIE_INDEX_BITS	6
#define GVE_MBX_COOKIE_INDEX_MASK \
	RTE_GENMASK32(GVE_MBX_COOKIE_INDEX_BITS - 1, 0)

struct gve_priv;

enum gve_mbx_queue_type {
	GVE_MBX_QUEUE_TYPE_UNKNOWN,
	GVE_MBX_QUEUE_TYPE_RX,
	GVE_MBX_QUEUE_TYPE_TX,
};

struct gve_mbx_registers {
	/* Lower 6bits are 0 to meet the 64-byte alignment */
	rte_le32_t base_addr_low;
	rte_le32_t base_addr_high;
	/* Max size required by the hw is 1023 */
	rte_le32_t queue_len;
	rte_le32_t queue_head;
	rte_le32_t queue_tail;
};

struct gve_mbx_queue {
	enum gve_mbx_queue_type q_type;
	struct gve_dma_mem desc_ring;
	uint16_t buf_size;
	uint16_t ring_size;
	struct gve_dma_mem *bufs;
	volatile struct gve_mbx_registers *reg;
	uint16_t len_mask;
	uint32_t len_ena_mask;
	uint16_t next_to_clean;
	uint16_t next_to_post;
	rte_spinlock_t q_lock; /* mbx q lock */
};

enum gve_mbx_status {
	GVE_MBX_STATUS_UNSET				= 0,
	GVE_MBX_STATUS_PASSED				= 1,
	GVE_MBX_STATUS_UNSUPPORTED_ERROR		= 0xFFEF,
	GVE_MBX_STATUS_ABORTED_ERROR			= 0xFFF0,
	GVE_MBX_STATUS_ALREADY_EXISTS_ERROR		= 0xFFF1,
	GVE_MBX_STATUS_CANCELLED_ERROR			= 0xFFF2,
	GVE_MBX_STATUS_DATA_LOSS_ERROR			= 0xFFF3,
	GVE_MBX_STATUS_DEADLINE_EXCEEDED_ERROR		= 0xFFF4,
	GVE_MBX_STATUS_FAILED_PRECONDITION_ERROR	= 0xFFF5,
	GVE_MBX_STATUS_INTERNAL_ERROR			= 0xFFF6,
	GVE_MBX_STATUS_INVALID_ARGUMENT_ERROR		= 0xFFF7,
	GVE_MBX_STATUS_NOT_FOUND_ERROR			= 0xFFF8,
	GVE_MBX_STATUS_OUT_OF_RANGE_ERROR		= 0xFFF9,
	GVE_MBX_STATUS_PERMISSION_DENIED_ERROR		= 0xFFFA,
	GVE_MBX_STATUS_UNAUTHENTICATED_ERROR		= 0xFFFB,
	GVE_MBX_STATUS_RESOURCE_EXHAUSTED_ERROR		= 0xFFFC,
	GVE_MBX_STATUS_UNAVAILABLE_ERROR		= 0xFFFD,
	GVE_MBX_STATUS_UNIMPLEMENTED_ERROR		= 0xFFFE,
	GVE_MBX_STATUS_UNKNOWN_ERROR			= 0xFFFF,
};

static inline int gve_mbx_get_err_from_status(int mbx_status) {
	switch (mbx_status) {
	case GVE_MBX_STATUS_PASSED:
		return 0;
	case GVE_MBX_STATUS_INVALID_ARGUMENT_ERROR:
		return EINVAL;
	default:
		return EBADMSG;
	}
}

enum gve_mbx_opcode {
	GVE_MBX_NEGOTIATE_CAPABILITIES		= 0x6001,
	GVE_MBX_EVENT				= 0x6002,
	GVE_MBX_GET_INTERRUPT_DBS		= 0x6005,
	GVE_MBX_GET_PTYPE_MAP			= 0x6006,
	GVE_MBX_REPORT_LINK_STATUS		= 0x6007,
	GVE_MBX_CONFIG_TX_QUEUES		= 0x6008,
	GVE_MBX_CONFIG_RX_QUEUES		= 0x6009,
	GVE_MBX_ENABLE_TX_QUEUES		= 0x600c,
	GVE_MBX_ENABLE_RX_QUEUES		= 0x600d,
	GVE_MBX_DISABLE_TX_QUEUES		= 0x600e,
	GVE_MBX_DISABLE_RX_QUEUES		= 0x600f,
	GVE_MBX_QUERY_RSS			= 0x6010,
	GVE_MBX_CONFIGURE_RSS			= 0x6011,
};

struct gve_mbx_event {
	rte_le32_t event_mask;
};

#define GVE_MBX_NO_INTERRUPT 0xffff
#define GVE_RAW_ADDRESSING_QPL_ID 0xFFFFFFFF

struct gve_mbx_tx_queue_info {
	rte_le32_t queue_id;
	rte_le16_t msix_vector;
	rte_le16_t reserved1;
	rte_le32_t queue_page_list_id;
	rte_le32_t reserved2;
	rte_le64_t ring_base_addr;
	rte_le64_t comp_ring_base_addr;
	rte_le16_t ring_size;
	rte_le16_t comp_ring_size;
	rte_le32_t reserved3;
} __packed;

struct gve_mbx_config_tx_queues_req {
	rte_le16_t num_queues;
	rte_le16_t reserved[3];
	struct gve_mbx_tx_queue_info queues[];
} __packed;

struct gve_mbx_configured_tx_queue_info {
	rte_le32_t queue_id;
	rte_le32_t tail_db_offset;
} __packed;

struct gve_mbx_config_tx_queues_resp {
	rte_le16_t num_queues;
	rte_le16_t reserved[3];
	struct gve_mbx_configured_tx_queue_info queues[];
} __packed;

struct gve_mbx_rx_queue_info {
	rte_le32_t queue_id;
	rte_le16_t msix_vector;
	rte_le16_t reserved1;
	rte_le32_t queue_page_list_id;
	rte_le32_t flags;
	rte_le64_t ring_base_addr;
	rte_le64_t data_ring_base_addr;
	rte_le16_t ring_size;
	rte_le16_t data_ring_size;
	rte_le16_t packet_buf_size;
	rte_le16_t header_buf_size;
} __packed;

struct gve_mbx_config_rx_queues_req {
	rte_le16_t num_queues;
	rte_le16_t reserved[3];
	struct gve_mbx_rx_queue_info queues[];
} __packed;

struct gve_mbx_configured_rx_queue_info {
	rte_le32_t queue_id;
	rte_le32_t tail_db_offset;
} __packed;

struct gve_mbx_config_rx_queues_resp {
	rte_le16_t num_queues;
	rte_le16_t reserved[3];
	struct gve_mbx_configured_rx_queue_info queues[];
} __packed;

struct gve_mbx_enable_tx_queues_req {
	rte_le16_t num_queues;
	rte_le16_t queue_ids[];
} __packed;

struct gve_mbx_enable_rx_queues_req {
	rte_le16_t num_queues;
	rte_le16_t queue_ids[];
} __packed;

struct gve_mbx_disable_queues_req {
	rte_le16_t num_queues;
	rte_le16_t queue_ids[];
} __packed;

struct gve_mbx_get_ptype_map_resp {
	struct gve_ptype_entry ptypes[GVE_NUM_PTYPES];
};

struct gve_mbx_report_link_status_resp {
	rte_le64_t link_speed;
	uint8_t link_status;
	uint8_t reserved[7];
};

struct gve_mbx_get_interrupt_dbs_req {
	rte_le16_t start_msix_index;
	rte_le16_t num_vecs;
};

struct gve_mbx_interrupt_db_info {
	rte_le32_t irq_db_offset;
	rte_le32_t irq_coalesce_db_offset;
};

struct gve_mbx_get_interrupt_dbs_resp {
	rte_le16_t start_msix_index;
	rte_le16_t num_vecs;
	struct gve_mbx_interrupt_db_info info[];
};

struct gve_mbx_negotiate_caps_req {
	rte_le32_t msg_version;
	rte_le32_t msg_size;
	rte_le64_t supported_caps;
	uint8_t os_type;
	uint8_t driver_major;
	uint8_t driver_minor;
	uint8_t driver_sub;
	rte_le32_t os_version_major;
	rte_le32_t os_version_minor;
	rte_le32_t os_version_sub;
	uint8_t os_version_str[512];
	uint8_t driver_version_str[64];
};

struct gve_mbx_negotiate_caps_resp {
	rte_le32_t msg_version;
	rte_le32_t msg_size;
	rte_le64_t negotiated_caps;
	uint8_t db_bar;
	uint8_t pad[3];
	rte_le32_t mbx_irq_db_offset;
	rte_le16_t mbx_response_timeout_ms;
	rte_le16_t tx_queue_watchdog_timeout_ms;
	rte_le16_t num_msix_vectors;
	rte_le16_t default_tx_num_queues;
	rte_le16_t default_rx_num_queues;
	rte_le16_t max_tx_num_queues;
	rte_le16_t max_rx_num_queues;
	rte_le16_t max_mtu;
	uint8_t mac[RTE_ETHER_ADDR_LEN];
	rte_le16_t default_tx_ring_size;
	rte_le16_t default_rx_ring_size;
	rte_le16_t max_tx_ring_size;
	rte_le16_t max_rx_ring_size;
	rte_le16_t min_tx_ring_size;
	rte_le16_t min_rx_ring_size;
	rte_le16_t max_packet_buffer_size;
	rte_le16_t max_header_buffer_size;
	rte_le16_t hash_key_size;
	rte_le16_t hash_lut_size;
};

struct gve_mbx_completion {
	pthread_cond_t cond;
	pthread_mutex_t mutex;
};

struct gve_mbx_msg {
	struct gve_mbx_completion comp;
	uint16_t sw_cookie;
	int status; /* gve_mbx_status */
	uint32_t opcode;
};

struct gve_mbx_msg_queue_map {
	struct rte_bitmap *bmp;
	void *mem;
};

struct gve_mbx_msg_queue {
	struct gve_mbx_msg_queue_map msg_queue_map;
	struct gve_mbx_msg **mbx_msgs;
	uint16_t size;
	uint32_t counter;
	uint32_t msg_timeout_ms;
	rte_spinlock_t mbx_msg_q_lock;
};

struct gve_mailbox {
	struct gve_mbx_queue *tx;
	struct gve_mbx_queue *rx;
	struct gve_mbx_msg_queue *msg_queue;
	struct gve_priv *priv;
	rte_thread_t mbx_thread;
	rte_be32_t __iomem *irq_db;
};

struct gve_mbx_desc {
	rte_le16_t flags;		/* DD bit, extra payload etc */
	rte_le16_t destination;		/* send to CP 0x0801 */
	rte_le16_t buf_len;		/* 0 when no extra payload, max is 4k */
	union {
		rte_le16_t retval;	/* MBX RX: status of message */
		rte_le16_t pfid_vfid;	/* MBX TX: func_id, 0 for PF */
	};
	rte_le32_t cmd_opcode;
	rte_le16_t cmd_retval;		/* size of the message */
	rte_le16_t reserved1;
	rte_le32_t function_id;
	rte_le16_t reserved2;
	rte_le16_t cmd_cookie;		/* for SW use */
	rte_le32_t addr_high;		/* of the allocated buffer */
	rte_le32_t addr_low;		/* of the allocated buffer */
};

enum gve_mbx_hash_alg {
	GVE_MBX_HASH_ALG_TOEPLITZ = 1,
};

struct gve_mbx_rss_info {
	rte_le16_t hash_types;
	u8 hash_alg;			/* gve_mbx_hash_alg */
	u8 reserved;
	rte_le16_t hash_key_size;
	rte_le16_t hash_lut_size;	/* in number of elements */
	u8 hash_key[256];
	rte_le32_t hash_lut[];
};

struct gve_rss_config;
int gve_mbx_query_rss(struct gve_priv *priv);
int gve_mbx_configure_rss(struct gve_priv *priv,
			  struct gve_rss_config *rss_config);

int gve_mbx_reset(struct gve_priv *priv);
int gve_mbx_init(struct gve_priv *priv);
void gve_mbx_teardown(struct gve_priv *priv);
int gve_mbx_get_device_properties(struct gve_priv *priv);
int gve_mbx_get_interrupt_dbs(struct gve_priv *priv);
int gve_mbx_get_ptype_map(struct gve_priv *priv);
int gve_mbx_report_link_speed(struct gve_priv *priv);
int gve_mbx_create_tx_queues(struct gve_priv *priv, uint32_t num_queues);
int gve_mbx_destroy_tx_queues(struct gve_priv *priv, uint32_t num_queues);
int gve_mbx_create_rx_queues(struct gve_priv *priv, uint32_t num_queues);
int gve_mbx_destroy_rx_queues(struct gve_priv *priv, uint32_t num_queues);

