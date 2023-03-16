# Introduction
The GVE PMD provides poll mode driver support for Google Virtual Ethernet device (also called as gVNIC).

# Build
This repository is not an independent buildable unit. It should be built as a whole package in https://github.com/DPDK 

1. Get DPDK from https://github.com/DPDK/dpdk. 
2. Overwrite the Gve source files: `cp -r * $DPDK/drivers/net/gve`
3. Build DPDK `meson -Dexamples=all build; ninja -C build`

Note: This is an out of tree driver and may not be part of official DPDK release. There might be instances when the source in `compute-virtual-ethernet-dpdk` might have build/performance issues when built andrun with DPDK. Please create an issue for such cases.
