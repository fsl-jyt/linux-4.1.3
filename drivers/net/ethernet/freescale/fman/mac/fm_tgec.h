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

#ifndef __TGEC_H
#define __TGEC_H

#include "enet_ext.h"

#include "fm_mac.h"

#define TGEC_DEFAULT_EXCEPTIONS			 \
	((u32)((TGEC_IMASK_MDIO_SCAN_EVENT)	|\
		(TGEC_IMASK_REM_FAULT)			|\
		(TGEC_IMASK_LOC_FAULT)			|\
		(TGEC_IMASK_TX_ECC_ER)			|\
		(TGEC_IMASK_TX_FIFO_UNFL)		|\
		(TGEC_IMASK_TX_FIFO_OVFL)		|\
		(TGEC_IMASK_TX_ER)			|\
		(TGEC_IMASK_RX_FIFO_OVFL)		|\
		(TGEC_IMASK_RX_ECC_ER)			|\
		(TGEC_IMASK_RX_JAB_FRM)			|\
		(TGEC_IMASK_RX_OVRSZ_FRM)		|\
		(TGEC_IMASK_RX_RUNT_FRM)		|\
		(TGEC_IMASK_RX_FRAG_FRM)		|\
		(TGEC_IMASK_RX_CRC_ER)			|\
		(TGEC_IMASK_RX_ALIGN_ER)))

#define MAX_PACKET_ALIGNMENT        31
#define MAX_INTER_PACKET_GAP        0x7f
#define MAX_INTER_PALTERNATE_BEB    0x0f
#define MAX_RETRANSMISSION          0x0f
#define MAX_COLLISION_WINDOW        0x03ff

/* number of pattern match registers (entries) */
#define TGEC_NUM_OF_PADDRS          1

/* Group address bit indication */
#define GROUP_ADDRESS               0x0000010000000000LL

/* Hash table size (= 32 bits*8 regs) */
#define TGEC_HASH_TABLE_SIZE             512

struct tgec_t {
	/* pointer to 10G memory mapped registers. */
	struct tgec_regs __iomem *regs;
	/* MAC address of device; */
	u64 addr;
	/* Ethernet physical interface  */
	enum e_enet_mode enet_mode;
	void *dev_id; /* device cookie used by the exception cbs */
	fm_mac_exception_cb *exception_cb;
	fm_mac_exception_cb *event_cb;
	/* Whether a particular individual address recognition
	 * register is being used
	 */
	bool ind_addr_reg_used[TGEC_NUM_OF_PADDRS];
	/* MAC address for particular individual address
	 * recognition register
	 */
	u64 paddr[TGEC_NUM_OF_PADDRS];
	/* Number of individual addresses in registers for this station. */
	u8 num_of_ind_addr_in_regs;
	/* pointer to driver's global address hash table  */
	struct eth_hash_t *multicast_addr_hash;
	/* pointer to driver's individual address hash table  */
	struct eth_hash_t *unicast_addr_hash;
	bool debug_mode;
	u8 mac_id;
	u32 exceptions;
	struct tgec_cfg *tgec_drv_param;
	void *fm;
	struct fm_revision_info_t fm_rev_info;
};

void *tgec_config(struct fm_mac_params_t *p_fm_mac_params);
int tgec_set_promiscuous(struct fm_mac_dev *fm_mac_dev, bool new_val);
int tgec_modify_mac_address(struct fm_mac_dev *fm_mac_dev,
			    enet_addr_t *p_enet_addr);
int tgec_cfg_max_frame_len(struct fm_mac_dev *fm_mac_dev, u16 new_val);
int tgec_enable(struct fm_mac_dev *fm_mac_dev, enum comm_mode mode);
int tgec_disable(struct fm_mac_dev *fm_mac_dev, enum comm_mode mode);
int tgec_init(struct fm_mac_dev *fm_mac_dev);
int tgec_free(struct fm_mac_dev *fm_mac_dev);
int tgec_accept_rx_pause_frames(struct fm_mac_dev *fm_mac_dev, bool en);
int tgec_set_tx_pause_frames(struct fm_mac_dev *fm_mac_dev, u8 priority,
			     u16 pause_time, u16 thresh_time);
int tgec_set_exception(struct fm_mac_dev *fm_mac_dev,
		       enum fm_mac_exceptions exception, bool enable);
int tgec_add_hash_mac_address(struct fm_mac_dev *fm_mac_dev,
			      enet_addr_t *p_eth_addr);
int tgec_del_hash_mac_address(struct fm_mac_dev *fm_mac_dev,
			      enet_addr_t *p_eth_addr);
int tgec_get_version(struct fm_mac_dev *fm_mac_dev, u32 *mac_version);

#endif /* __TGEC_H */
