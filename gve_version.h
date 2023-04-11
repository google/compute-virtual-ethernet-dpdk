/* SPDX-License-Identifier: MIT
 * Google Virtual Ethernet (gve) driver
 * Copyright (C) 2015-2023 Google, Inc.
 */

#ifndef _GVE_VERSION_H_
#define _GVE_VERSION_H_

#include <rte_version.h>

#define GVE_VERSION_PREFIX "GVE-"
#define GVE_VERSION_MAJOR 0
#define GVE_VERSION_MINOR 9
#define GVE_VERSION_SUB 0

#define DPDK_VERSION_MAJOR (100 * RTE_VER_YEAR + RTE_VER_MONTH)
#define DPDK_VERSION_MINOR RTE_VER_MINOR
#define DPDK_VERSION_SUB RTE_VER_RELEASE


const char *
gve_version_string(void);


#endif /* GVE_VERSION_H */
