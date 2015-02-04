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

#ifndef __MEMAC_H
#define __MEMAC_H

#include "fsl_fman_memac_mii_acc.h"
#include "fm_mac.h"
#include "fsl_fman_memac.h"

#define MEMAC_DEFAULT_EXCEPTIONS					\
	((u32)(MEMAC_IMASK_TSECC_ER | MEMAC_IMASK_TECC_ER |	\
		MEMAC_IMASK_RECC_ER | MEMAC_IMASK_MGI))

struct memac_t {
	/* Pointer to MAC memory mapped registers */
	struct memac_regs __iomem *regs;
	/* Pointer to MII memory mapped registers */
	struct memac_mii_access_mem_map __iomem *mii_regs;
	/* MAC address of device */
	u64 addr;
	/* Ethernet physical interface  */
	enum e_enet_mode enet_mode;
	void *dev_id; /* device cookie used by the exception cbs */
	fm_mac_exception_cb *exception_cb;
	fm_mac_exception_cb *event_cb;
	/* Whether a particular individual address
	 * recognition register is being used
	 */
	bool ind_addr_reg_used[MEMAC_NUM_OF_PADDRS];
	/* MAC address for particular individual address
	 * recognition register
	 */
	u64 paddr[MEMAC_NUM_OF_PADDRS];
	/* Number of individual addresses in registers for this station. */
	u8 num_of_ind_addr_in_regs;
	/* Pointer to driver's global address hash table  */
	struct eth_hash_t *multicast_addr_hash;
	/* Pointer to driver's individual address hash table  */
	struct eth_hash_t *unicast_addr_hash;
	bool debug_mode;
	u8 mac_id;
	u32 exceptions;
	struct memac_cfg *memac_drv_param;
	void *fm;
	struct fm_revision_info_t fm_rev_info;
};

/* Internal PHY access */
#define PHY_MDIO_ADDR               0

/* Internal PHY Registers - SGMII */
#define PHY_SGMII_CR_RESET_AN           0x0200
#define PHY_SGMII_CR_AN_ENABLE          0x1000
#define PHY_SGMII_CR_DEF_VAL            0x1140
#define PHY_SGMII_DEV_ABILITY_SGMII     0x4001
#define PHY_SGMII_DEV_ABILITY_1000X     0x01A0
#define PHY_SGMII_IF_MODE_DUPLEX_FULL   0x0000
#define PHY_SGMII_IF_MODE_DUPLEX_HALF   0x0010
#define PHY_SGMII_IF_MODE_SPEED_GB      0x0008
#define PHY_SGMII_IF_MODE_SPEED_100M    0x0004
#define PHY_SGMII_IF_MODE_SPEED_10M     0x0000
#define PHY_SGMII_IF_MODE_AN            0x0002
#define PHY_SGMII_IF_MODE_SGMII         0x0001
#define PHY_SGMII_IF_MODE_1000X         0x0000

/* Offset from the MEM map to the MDIO mem map */
#define MEMAC_TO_MII_OFFSET         0x030

void *memac_config(struct fm_mac_params_t *p_fm_mac_param);
int memac_set_promiscuous(struct fm_mac_dev *fm_mac_dev, bool new_val);
int memac_modify_mac_address(struct fm_mac_dev *fm_mac_dev,
			     enet_addr_t *p_enet_addr);
int memac_adjust_link(struct fm_mac_dev *fm_mac_dev,
		      enum ethernet_speed speed);
int memac_cfg_max_frame_len(struct fm_mac_dev *fm_mac_dev, u16 new_val);
int memac_cfg_reset_on_init(struct fm_mac_dev *fm_mac_dev, bool enable);
int memac_cfg_fixed_link(struct fm_mac_dev *fm_mac_dev, bool enable);
int memac_enable(struct fm_mac_dev *fm_mac_dev, enum comm_mode mode);
int memac_disable(struct fm_mac_dev *fm_mac_dev, enum comm_mode mode);
int memac_init(struct fm_mac_dev *fm_mac_dev);
int memac_free(struct fm_mac_dev *fm_mac_dev);
int memac_accept_rx_pause_frames(struct fm_mac_dev *fm_mac_dev, bool en);
int memac_set_tx_pause_frames(struct fm_mac_dev *fm_mac_dev, u8 priority,
			      u16 pause_time, u16 thresh_time);
int memac_set_exception(struct fm_mac_dev *fm_mac_dev,
			enum fm_mac_exceptions exception, bool enable);
int memac_add_hash_mac_address(struct fm_mac_dev *fm_mac_dev,
			       enet_addr_t *p_eth_addr);
int memac_del_hash_mac_address(struct fm_mac_dev *fm_mac_dev,
			       enet_addr_t *p_eth_addr);

#endif /* __MEMAC_H */
