/*
 * Copyright 2008-2015 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __FM_DRV_H__
#define __FM_DRV_H__

#include <asm/mpc85xx.h>

#include "fsl_fman_drv.h"
#include "fm_port_ext.h"

#ifndef CONFIG_FSL_FM_MAX_FRAME_SIZE
#define CONFIG_FSL_FM_MAX_FRAME_SIZE 0
#endif

#ifndef CONFIG_FSL_FM_RX_EXTRA_HEADROOM
#define CONFIG_FSL_FM_RX_EXTRA_HEADROOM       16
#endif

/* SoC info */
#define SOC_VERSION(svr)        (((svr) & 0xFFF7FF00) >> 8)
#define SOC_MAJOR_REV(svr)      (((svr) & 0x000000F0) >> 4)
#define SOC_MINOR_REV(svr)      ((svr) & 0x0000000F)

/* Port defines */
#define NUM_OF_FM_PORTS			63
#define FIRST_RX_PORT			0x08
#define FIRST_TX_PORT			0x28
#define LAST_RX_PORT			0x11
#define LAST_TX_PORT			0x31

#define TX_10G_PORT_BASE		0x30
#define RX_10G_PORT_BASE		0x10

struct fm_port_t;

struct fm_port_drv_t {
	u8 id;
	char name[20];
	bool active;
	phys_addr_t phys_base_addr;
	void __iomem *base_addr;	/* Port's *virtual* address */
	resource_size_t mem_size;
	struct fm_port_params_t port_params;
	struct fm_buffer_prefix_content_t buff_prefix_content;
	struct fm_port_t *fm_port;
	struct fm_drv_t *fm;
	u16 tx_ch;
	struct device *dev;
	struct fm_revision_info_t fm_rev_info;
};

struct fm_drv_t {
	u8 id;
	char name[10];
	bool active;
	phys_addr_t fm_phys_base_addr;
	void __iomem *fm_base_addr;
	resource_size_t fm_mem_size;
	phys_addr_t fm_muram_phys_base_addr;
	resource_size_t fm_muram_mem_size;
	int irq;
	int err_irq;
	struct fm_params_t params;
	void *fm_dev;
	struct muram_info *muram;

	struct fm_port_drv_t ports[NUM_OF_FM_PORTS];

	struct device *dev;
	struct resource *res;

	struct fm_revision_info_t fm_rev_info;
	u32 qman_channel_base;
	u32 num_of_qman_channels;
	u32 *qman_channels;

};

u32 get_qman_channel_id(struct fm_drv_t *fm_drv, u32 port_id);

#endif /* __FM_DRV_H__ */
