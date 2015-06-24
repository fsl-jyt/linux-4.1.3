subdir-ccflags-y += -I$(srctree)/drivers/net/ethernet/freescale/fman/flib \
				     -I$(srctree)/drivers/net/ethernet/freescale/fman/inc \
				     -I$(srctree)/drivers/net/ethernet/freescale/fman

obj-y		+= fsl_fman.o

fsl_fman-objs	:= fman.o fm_muram.o fm.o fm_drv.o fm_port_drv.o

obj-y	+= port/
obj-y	+= mac/
obj-y	+= sp/
