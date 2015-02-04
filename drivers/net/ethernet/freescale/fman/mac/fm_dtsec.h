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

#ifndef __DTSEC_H
#define __DTSEC_H

#include "enet_ext.h"

#include "fm_mac.h"

#define DTSEC_DEFAULT_EXCEPTIONS		 \
	((u32)((DTSEC_IMASK_BREN)		|\
			(DTSEC_IMASK_RXCEN)	|\
			(DTSEC_IMASK_BTEN)	|\
			(DTSEC_IMASK_TXCEN)	|\
			(DTSEC_IMASK_TXEEN)	|\
			(DTSEC_IMASK_ABRTEN)	|\
			(DTSEC_IMASK_LCEN)	|\
			(DTSEC_IMASK_CRLEN)	|\
			(DTSEC_IMASK_XFUNEN)	|\
			(DTSEC_IMASK_IFERREN)	|\
			(DTSEC_IMASK_MAGEN)	|\
			(DTSEC_IMASK_TDPEEN)	|\
			(DTSEC_IMASK_RDPEEN)))

#define MAX_PACKET_ALIGNMENT        31
#define MAX_INTER_PACKET_GAP        0x7f
#define MAX_INTER_PALTERNATE_BEB    0x0f
#define MAX_RETRANSMISSION          0x0f
#define MAX_COLLISION_WINDOW        0x03ff

/* number of pattern match registers (entries) */
#define DTSEC_NUM_OF_PADDRS             15

/* Group address bit indication */
#define GROUP_ADDRESS                   0x0000010000000000LL

/* Hash table size (32 bits*8 regs) */
#define DTSEC_HASH_TABLE_SIZE                 256
/* Extended Hash table size (32 bits*16 regs) */
#define EXTENDED_HASH_TABLE_SIZE        512

/* number of pattern match registers (entries) */
#define DTSEC_TO_MII_OFFSET             0x1000
/* maximum number of phys */
#define MAX_PHYS                    32

#define     VAL32BIT    0x100000000LL
#define     VAL22BIT    0x00400000
#define     VAL16BIT    0x00010000
#define     VAL12BIT    0x00001000

/* CAR1/2 bits */
#define CAR1_TR64   0x80000000
#define CAR1_TR127  0x40000000
#define CAR1_TR255  0x20000000
#define CAR1_TR511  0x10000000
#define CAR1_TRK1   0x08000000
#define CAR1_TRMAX  0x04000000
#define CAR1_TRMGV  0x02000000

#define CAR1_RBYT   0x00010000
#define CAR1_RPKT   0x00008000
#define CAR1_RMCA   0x00002000
#define CAR1_RBCA   0x00001000
#define CAR1_RXPF   0x00000400
#define CAR1_RALN   0x00000100
#define CAR1_RFLR   0x00000080
#define CAR1_RCDE   0x00000040
#define CAR1_RCSE   0x00000020
#define CAR1_RUND   0x00000010
#define CAR1_ROVR   0x00000008
#define CAR1_RFRG   0x00000004
#define CAR1_RJBR   0x00000002
#define CAR1_RDRP   0x00000001

#define CAR2_TFCS   0x00040000
#define CAR2_TBYT   0x00002000
#define CAR2_TPKT   0x00001000
#define CAR2_TMCA   0x00000800
#define CAR2_TBCA   0x00000400
#define CAR2_TXPF   0x00000200
#define CAR2_TDRP   0x00000001

struct internal_statistics_t {
	u64 tr64;
	u64 tr127;
	u64 tr255;
	u64 tr511;
	u64 tr1k;
	u64 trmax;
	u64 trmgv;
	u64 rfrg;
	u64 rjbr;
	u64 rdrp;
	u64 raln;
	u64 rund;
	u64 rovr;
	u64 rxpf;
	u64 txpf;
	u64 rbyt;
	u64 rpkt;
	u64 rmca;
	u64 rbca;
	u64 rflr;
	u64 rcde;
	u64 rcse;
	u64 tbyt;
	u64 tpkt;
	u64 tmca;
	u64 tbca;
	u64 tdrp;
	u64 tfcs;
};

struct dtsec_t {
	/* pointer to dTSEC memory mapped registers */
	struct dtsec_regs __iomem *regs;
	/* pointer to dTSEC MII memory mapped registers */
	struct dtsec_mii_reg __iomem *mii_regs;
	/* MAC address of device */
	u64 addr;
	/* Ethernet physical interface */
	enum e_enet_mode enet_mode;
	void *dev_id; /* device cookie used by the exception cbs */
	fm_mac_exception_cb *exception_cb;
	fm_mac_exception_cb *event_cb;
	/* Whether a particular individual address recognition
	 * register is being used
	 */
	bool ind_addr_reg_used[DTSEC_NUM_OF_PADDRS];
	/* MAC address for particular individual
	 * address recognition register
	 */
	u64 paddr[DTSEC_NUM_OF_PADDRS];
	/* Number of individual addresses in registers for this station */
	u8 num_of_ind_addr_in_regs;
	struct internal_statistics_t internal_statistics;
	/* pointer to driver's global address hash table */
	struct eth_hash_t *multicast_addr_hash;
	/* pointer to driver's individual address hash table */
	struct eth_hash_t *unicast_addr_hash;
	u8 mac_id;
	u8 tbi_phy_addr;
	u32 exceptions;
	bool ptp_tsu_enabled;
	bool en_tsu_err_exeption;
	struct dtsec_cfg *dtsec_drv_param;
	u16 clk_freq;
	void *fm;
	struct fm_revision_info_t fm_rev_info;
};

void *dtsec_config(struct fm_mac_params_t *fm_mac_param);
int dtsec_set_promiscuous(struct fm_mac_dev *fm_mac_dev, bool new_val);
int dtsec_modify_mac_address(struct fm_mac_dev *fm_mac_dev,
			     enet_addr_t *enet_addr);
int dtsec_adjust_link(struct fm_mac_dev *fm_mac_dev,
		      enum ethernet_speed speed);
int dtsec_restart_autoneg(struct fm_mac_dev *fm_mac_dev);
int dtsec_cfg_max_frame_len(struct fm_mac_dev *fm_mac_dev, u16 new_val);
int dtsec_cfg_pad_and_crc(struct fm_mac_dev *fm_mac_dev, bool new_val);
int dtsec_enable(struct fm_mac_dev *fm_mac_dev, enum comm_mode mode);
int dtsec_disable(struct fm_mac_dev *fm_mac_dev, enum comm_mode mode);
int dtsec_init(struct fm_mac_dev *fm_mac_dev);
int dtsec_free(struct fm_mac_dev *fm_mac_dev);
int dtsec_accept_rx_pause_frames(struct fm_mac_dev *fm_mac_dev, bool en);
int dtsec_set_tx_pause_frames(struct fm_mac_dev *fm_mac_dev, u8 priority,
			      u16 pause_time, u16 thresh_time);
int dtsec_set_exception(struct fm_mac_dev *fm_mac_dev,
			enum fm_mac_exceptions exception, bool enable);
int dtsec_add_hash_mac_address(struct fm_mac_dev *fm_mac_dev,
			       enet_addr_t *eth_addr);
int dtsec_del_hash_mac_address(struct fm_mac_dev *fm_mac_dev,
			       enet_addr_t *eth_addr);
int dtsec_get_version(struct fm_mac_dev *fm_mac_dev, u32 *mac_version);

#endif /* __DTSEC_H */
