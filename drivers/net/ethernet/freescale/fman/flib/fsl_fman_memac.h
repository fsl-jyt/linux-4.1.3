/*
 * Copyright 2008 - 2015 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
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

#ifndef __FSL_FMAN_MEMAC_H
#define __FSL_FMAN_MEMAC_H

#include <linux/io.h>

#include "fsl_enet.h"
/* Num of additional exact match MAC adr regs */
#define MEMAC_NUM_OF_PADDRS 7

/* Control and Configuration Register (COMMAND_CONFIG) */
/* 00 Magic Packet detection */
#define CMD_CFG_MG		0x80000000
/* 07 Rx low power indication */
#define CMD_CFG_REG_LOWP_RXETY	0x01000000
/* 08 Tx Low Power Idle Enable */
#define CMD_CFG_TX_LOWP_ENA	0x00800000
/* 10 Disable SFD check */
#define CMD_CFG_SFD_ANY		0x00200000
/* 12 Enable PFC */
#define CMD_CFG_PFC_MODE	0x00080000
/* 14 Payload length check disable */
#define CMD_CFG_NO_LEN_CHK	0x00020000
/* 15 Force idle generation */
#define CMD_CFG_SEND_IDLE	0x00010000
/* 18 Control frame rx enable */
#define CMD_CFG_CNT_FRM_EN	0x00002000
/* 19 S/W Reset, self clearing bit */
#define CMD_CFG_SW_RESET	0x00001000
/* 20 Enable Tx padding of frames */
#define CMD_CFG_TX_PAD_EN	0x00000800
/* 21 XGMII/GMII loopback enable */
#define CMD_CFG_LOOPBACK_EN	0x00000400
/* 22 Tx source MAC addr insertion */
#define CMD_CFG_TX_ADDR_INS	0x00000200
/* 23 Ignore Pause frame quanta */
#define CMD_CFG_PAUSE_IGNORE	0x00000100
/* 24 Terminate/frwd Pause frames */
#define CMD_CFG_PAUSE_FWD	0x00000080
/* 25 Terminate/frwd CRC of frames */
#define CMD_CFG_CRC_FWD		0x00000040
/* 26 Frame padding removal */
#define CMD_CFG_PAD_EN		0x00000020
/* 27 Promiscuous operation enable */
#define CMD_CFG_PROMIS_EN	0x00000010
/* 28 WAN mode enable */
#define CMD_CFG_WAN_MODE	0x00000008
/* 30 MAC receive path enable */
#define CMD_CFG_RX_EN		0x00000002
/* 31 MAC transmit path enable */
#define CMD_CFG_TX_EN		0x00000001

/* Transmit FIFO Sections Register (TX_FIFO_SECTIONS) */
#define TX_FIFO_SECTIONS_TX_EMPTY_MASK			0xFFFF0000
#define TX_FIFO_SECTIONS_TX_AVAIL_MASK			0x0000FFFF
#define TX_FIFO_SECTIONS_TX_EMPTY_DEFAULT_10G		0x00400000
#define TX_FIFO_SECTIONS_TX_EMPTY_DEFAULT_1G		0x00100000
#define TX_FIFO_SECTIONS_TX_EMPTY_PFC_10G		0x00360000
#define TX_FIFO_SECTIONS_TX_EMPTY_PFC_1G		0x00040000
#define TX_FIFO_SECTIONS_TX_AVAIL_10G			0x00000019
#define TX_FIFO_SECTIONS_TX_AVAIL_1G			0x00000020
#define TX_FIFO_SECTIONS_TX_AVAIL_SLOW_10G		0x00000060

#define GET_TX_EMPTY_DEFAULT_VALUE(_val)				\
do {									\
	_val &= ~TX_FIFO_SECTIONS_TX_EMPTY_MASK;			\
	((_val == TX_FIFO_SECTIONS_TX_AVAIL_10G) ?			\
			(_val |= TX_FIFO_SECTIONS_TX_EMPTY_DEFAULT_10G) :\
			(_val |= TX_FIFO_SECTIONS_TX_EMPTY_DEFAULT_1G));\
} while (0)

#define GET_TX_EMPTY_PFC_VALUE(_val)					\
do {									\
	_val &= ~TX_FIFO_SECTIONS_TX_EMPTY_MASK;			\
	((_val == TX_FIFO_SECTIONS_TX_AVAIL_10G) ?			\
			(_val |= TX_FIFO_SECTIONS_TX_EMPTY_PFC_10G) :	\
			(_val |= TX_FIFO_SECTIONS_TX_EMPTY_PFC_1G));	\
} while (0)

/* Interface Mode Register (IF_MODE) */
/* 30-31 Mask on i/f mode bits */
#define IF_MODE_MASK		0x00000003
/* 30-31 XGMII (10G) interface */
#define IF_MODE_XGMII		0x00000000
/* 30-31 GMII (1G) interface */
#define IF_MODE_GMII		0x00000002
#define IF_MODE_RGMII		0x00000004
#define IF_MODE_RGMII_AUTO	0x00008000
#define IF_MODE_RGMII_1000  0x00004000	/* 10 - 1000Mbps RGMII */
#define IF_MODE_RGMII_100   0x00000000	/* 00 - 100Mbps RGMII */
#define IF_MODE_RGMII_10    0x00002000	/* 01 - 10Mbps RGMII */
#define IF_MODE_RGMII_SP_MASK 0x00006000	/* Setsp mask bits */
#define IF_MODE_RGMII_FD    0x00001000	/* Full duplex RGMII */
#define IF_MODE_HD	    0x00000040	/* Half duplex operation */

/* Hash table Control Register (HASHTABLE_CTRL) */
#define HASH_CTRL_MCAST_SHIFT	26
/* 23 Mcast frame rx for hash */
#define HASH_CTRL_MCAST_EN	0x00000100
/* 26-31 Hash table address code */
#define HASH_CTRL_ADDR_MASK	0x0000003F
/* MAC mcast indication */
#define GROUP_ADDRESS		0x0000010000000000LL
#define HASH_TABLE_SIZE		64	/* Hash tbl size */

/* Transmit Inter-Packet Gap Length Register (TX_IPG_LENGTH) */
#define MEMAC_TX_IPG_LENGTH_MASK	0x0000003F

/* Statistics Configuration Register (STATN_CONFIG) */
#define STATS_CFG_CLR		0x00000004	/* 29 Reset all counters */
#define STATS_CFG_CLR_ON_RD	0x00000002	/* 30 Clear on read */
/* 31 Saturate at the maximum val */
#define STATS_CFG_SATURATE	0x00000001

/* Interrupt Mask Register (IMASK) */
/* 1 Magic pkt detect indication */
#define MEMAC_IMASK_MGI		0x40000000
/* 2 Timestamp FIFO ECC error evnt */
#define MEMAC_IMASK_TSECC_ER 0x20000000
/* 6 Transmit frame ECC error evnt */
#define MEMAC_IMASK_TECC_ER	0x02000000
/* 7 Receive frame ECC error evnt */
#define MEMAC_IMASK_RECC_ER	0x01000000

#define MEMAC_ALL_ERRS_IMASK					\
		((u32)(MEMAC_IMASK_TSECC_ER	|	\
		       MEMAC_IMASK_TECC_ER		|	\
		       MEMAC_IMASK_RECC_ER		|	\
		       MEMAC_IMASK_MGI))

/* PCS (XG). Link sync (G) */
#define MEMAC_IEVNT_PCS			0x80000000
/* Auto-negotiation */
#define MEMAC_IEVNT_AN			0x40000000
/* Link Training/New page */
#define MEMAC_IEVNT_LT			0x20000000
/* Magic pkt detection */
#define MEMAC_IEVNT_MGI			0x00004000
/* Timestamp FIFO ECC error */
#define MEMAC_IEVNT_TS_ECC_ER		0x00002000
/* Rx FIFO overflow */
#define MEMAC_IEVNT_RX_FIFO_OVFL	0x00001000
/* Tx FIFO underflow */
#define MEMAC_IEVNT_TX_FIFO_UNFL	0x00000800
/* Tx FIFO overflow */
#define MEMAC_IEVNT_TX_FIFO_OVFL	0x00000400
/* Tx frame ECC error */
#define MEMAC_IEVNT_TX_ECC_ER		0x00000200
/* Rx frame ECC error */
#define MEMAC_IEVNT_RX_ECC_ER		0x00000100
/* Link Interruption flt */
#define MEMAC_IEVNT_LI_FAULT		0x00000080
/* Rx FIFO empty */
#define MEMAC_IEVNT_RX_EMPTY		0x00000040
/* Tx FIFO empty */
#define MEMAC_IEVNT_TX_EMPTY		0x00000020
/* Low Power Idle */
#define MEMAC_IEVNT_RX_LOWP		0x00000010
/* Phy loss of signal */
#define MEMAC_IEVNT_PHY_LOS		0x00000004
/* Remote fault (XGMII) */
#define MEMAC_IEVNT_REM_FAULT		0x00000002
/* Local fault (XGMII) */
#define MEMAC_IEVNT_LOC_FAULT		0x00000001

#define DEFAULT_PAUSE_QUANTA	0xf000
#define DEFAULT_FRAME_LENGTH	0x600
#define DEFAULT_TX_IPG_LENGTH	12

#define CLXY_PAUSE_QUANTA_CLX_PQNT	0x0000FFFF
#define CLXY_PAUSE_QUANTA_CLY_PQNT	0xFFFF0000
#define CLXY_PAUSE_THRESH_CLX_QTH	0x0000FFFF
#define CLXY_PAUSE_THRESH_CLY_QTH	0xFFFF0000

struct mac_addr {
	/* Lower 32 bits of 48-bit MAC address */
	u32 mac_addr_l;
	/* Upper 16 bits of 48-bit MAC address */
	u32 mac_addr_u;
};

/* memory map */
struct memac_regs {
	u32 res0000[2];			/* General Control and Status */
	u32 command_config;		/* 0x008 Ctrl and cfg */
	struct mac_addr mac_addr0;	/* 0x00C-0x010 MAC_ADDR_0...1 */
	u32 maxfrm;			/* 0x014 Max frame length */
	u32 res0018[1];
	u32 rx_fifo_sections;		/* Receive FIFO configuration reg */
	u32 tx_fifo_sections;		/* Transmit FIFO configuration reg */
	u32 res0024[2];
	u32 hashtable_ctrl;		/* 0x02C Hash table control */
	u32 res0030[4];
	u32 ievent;			/* 0x040 Interrupt event */
	u32 tx_ipg_length;		/* 0x044 Transmitter inter-packet-gap */
	u32 res0048;
	u32 imask;			/* 0x04C Interrupt mask */
	u32 res0050;
	u32 pause_quanta[4];		/* 0x054 Pause quanta */
	u32 pause_thresh[4];		/* 0x064 Pause quanta threshold */
	u32 rx_pause_status;		/* 0x074 Receive pause status */
	u32 res0078[2];
	struct mac_addr mac_addr[MEMAC_NUM_OF_PADDRS];/* 0x80-0x0B4 mac padr */
	u32 lpwake_timer;		/* 0x0B8 Low Power Wakeup Timer */
	u32 sleep_timer;		/* 0x0BC Transmit EEE Low Power Timer */
	u32 res00c0[8];
	u32 statn_config;		/* 0x0E0 Statistics configuration */
	u32 res00e4[7];
	/* Rx Statistics Counter */
	u32 reoct_l;
	u32 reoct_u;
	u32 roct_l;
	u32 roct_u;
	u32 raln_l;
	u32 raln_u;
	u32 rxpf_l;
	u32 rxpf_u;
	u32 rfrm_l;
	u32 rfrm_u;
	u32 rfcs_l;
	u32 rfcs_u;
	u32 rvlan_l;
	u32 rvlan_u;
	u32 rerr_l;
	u32 rerr_u;
	u32 ruca_l;
	u32 ruca_u;
	u32 rmca_l;
	u32 rmca_u;
	u32 rbca_l;
	u32 rbca_u;
	u32 rdrp_l;
	u32 rdrp_u;
	u32 rpkt_l;
	u32 rpkt_u;
	u32 rund_l;
	u32 rund_u;
	u32 r64_l;
	u32 r64_u;
	u32 r127_l;
	u32 r127_u;
	u32 r255_l;
	u32 r255_u;
	u32 r511_l;
	u32 r511_u;
	u32 r1023_l;
	u32 r1023_u;
	u32 r1518_l;
	u32 r1518_u;
	u32 r1519x_l;
	u32 r1519x_u;
	u32 rovr_l;
	u32 rovr_u;
	u32 rjbr_l;
	u32 rjbr_u;
	u32 rfrg_l;
	u32 rfrg_u;
	u32 rcnp_l;
	u32 rcnp_u;
	u32 rdrntp_l;
	u32 rdrntp_u;
	u32 res01d0[12];
	/* Tx Statistics Counter */
	u32 teoct_l;
	u32 teoct_u;
	u32 toct_l;
	u32 toct_u;
	u32 res0210[2];
	u32 txpf_l;
	u32 txpf_u;
	u32 tfrm_l;
	u32 tfrm_u;
	u32 tfcs_l;
	u32 tfcs_u;
	u32 tvlan_l;
	u32 tvlan_u;
	u32 terr_l;
	u32 terr_u;
	u32 tuca_l;
	u32 tuca_u;
	u32 tmca_l;
	u32 tmca_u;
	u32 tbca_l;
	u32 tbca_u;
	u32 res0258[2];
	u32 tpkt_l;
	u32 tpkt_u;
	u32 tund_l;
	u32 tund_u;
	u32 t64_l;
	u32 t64_u;
	u32 t127_l;
	u32 t127_u;
	u32 t255_l;
	u32 t255_u;
	u32 t511_l;
	u32 t511_u;
	u32 t1023_l;
	u32 t1023_u;
	u32 t1518_l;
	u32 t1518_u;
	u32 t1519x_l;
	u32 t1519x_u;
	u32 res02a8[6];
	u32 tcnp_l;
	u32 tcnp_u;
	u32 res02c8[14];
	/* Line Interface Control */
	u32 if_mode;		/* 0x300 Interface Mode Control */
	u32 if_status;		/* 0x304 Interface Status */
	u32 res0308[14];
	/* HiGig/2 */
	u32 hg_config;		/* 0x340 Control and cfg */
	u32 res0344[3];
	u32 hg_pause_quanta;	/* 0x350 Pause quanta */
	u32 res0354[3];
	u32 hg_pause_thresh;	/* 0x360 Pause quanta threshold */
	u32 res0364[3];
	u32 hgrx_pause_status;	/* 0x370 Receive pause status */
	u32 hg_fifos_status;	/* 0x374 fifos status */
	u32 rhm;		/* 0x378 rx messages counter */
	u32 thm;		/* 0x37C tx messages counter */
};

struct memac_cfg {
	bool reset_on_init;
	bool rx_error_discard;
	bool pause_ignore;
	bool pause_forward_enable;
	bool no_length_check_enable;
	bool cmd_frame_enable;
	bool send_idle_enable;
	bool wan_mode_enable;
	bool promiscuous_mode_enable;
	bool tx_addr_ins_enable;
	bool loopback_enable;
	bool lgth_check_nostdr;
	bool time_stamp_enable;
	bool pad_enable;
	bool phy_tx_ena_on;
	bool rx_sfd_any;
	bool rx_pbl_fwd;
	bool tx_pbl_fwd;
	bool debug_mode;
	bool wake_on_lan;
	u16 max_frame_length;
	u16 pause_quanta;
	u32 tx_ipg_length;
};

void fman_memac_defconfig(struct memac_cfg *cfg);

int fman_memac_init(struct memac_regs __iomem *regs, struct memac_cfg *cfg,
		    enum enet_interface enet_interface,
		    enum enet_speed enet_speed, bool slow_10g_if,
		    u32 exceptions);

void fman_memac_enable(struct memac_regs __iomem *regs, bool apply_rx,
		       bool apply_tx);

void fman_memac_disable(struct memac_regs __iomem *regs, bool apply_rx,
			bool apply_tx);

void fman_memac_set_promiscuous(struct memac_regs __iomem *regs, bool val);

void fman_memac_add_addr_in_paddr(struct memac_regs __iomem *regs,
				  u8 *adr, u8 paddr_num);

void fman_memac_clear_addr_in_paddr(struct memac_regs __iomem *regs,
				    u8 paddr_num);

void fman_memac_set_tx_pause_frames(struct memac_regs __iomem *regs,
				    u8 priority, u16 pause_time,
				    u16 thresh_time);

u16 fman_memac_get_max_frame_len(struct memac_regs __iomem *regs);

void fman_memac_set_exception(struct memac_regs __iomem *regs, u32 val,
			      bool enable);

int fman_memac_reset(struct memac_regs __iomem *regs);

void fman_memac_set_hash_table(struct memac_regs __iomem *regs, u32 val);

void fman_memac_set_rx_ignore_pause_frames(struct memac_regs __iomem *regs,
					   bool enable);

u32 fman_memac_get_event(struct memac_regs __iomem *regs, u32 ev_mask);

void fman_memac_ack_event(struct memac_regs __iomem *regs, u32 ev_mask);

u32 fman_memac_get_interrupt_mask(struct memac_regs __iomem *regs);

void fman_memac_adjust_link(struct memac_regs __iomem *regs,
			    enum enet_interface iface_mode,
			    enum enet_speed speed, bool full_dx);

#endif	/* __FSL_FMAN_MEMAC_H */
