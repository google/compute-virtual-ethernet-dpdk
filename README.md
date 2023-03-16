# Introduction
The GVE PMD provides poll mode driver support for Google Virtual Ethernet device (also called as gVNIC).

# Build
This repository is not an independent buildable unit. It should be built as a whole package in https://github.com/DPDK 

1. Get DPDK from https://github.com/DPDK/dpdk. Note: since there might be cases when the source in `compute-virtual-ethernet-dpdk-gve` might have build/performance dependency on DPDK. To avoid such regression, please create an issue in github.
2. Overwrite the Gve source files: `cp -r * $DPDK/drivers/net/gve`
3. Build DPDK `meson -Dexamples=all build; ninja -C build`
