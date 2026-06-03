/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 Google LLC
 */
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

