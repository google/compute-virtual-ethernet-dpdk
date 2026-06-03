/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 Google LLC
 */

/* Mailbox Queue defines */
#define GVE_MBX_RESET_CTRL		0x0840700C
#define GVE_MBX_RESET_STATUS		0x08407008

struct gve_priv;

int gve_mbx_reset(struct gve_priv *priv);
