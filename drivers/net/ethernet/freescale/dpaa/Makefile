#
# Makefile for the Freescale DPAA Ethernet controllers
#

# Include FMan headers
FMAN        = $(srctree)/drivers/net/ethernet/freescale/fman
ccflags-y += -I$(FMAN)
ccflags-y += -I$(FMAN)/inc
ccflags-y += -I$(FMAN)/flib

obj-$(CONFIG_FSL_DPAA_ETH) += fsl_dpa.o

fsl_dpa-objs += dpaa_eth.o dpaa_eth_sg.o dpaa_eth_common.o dpaa_ethtool.o dpaa_eth_sysfs.o
ifeq ($(CONFIG_FSL_DPAA_ETH_DEBUGFS),y)
fsl_dpa-objs += dpaa_debugfs.o
endif
