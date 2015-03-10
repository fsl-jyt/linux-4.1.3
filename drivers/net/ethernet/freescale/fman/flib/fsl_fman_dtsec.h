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

#ifndef __FSL_FMAN_DTSEC_H
#define __FSL_FMAN_DTSEC_H

#include <linux/io.h>

#include "fsl_enet.h"

/* dTSEC Init sequence
 * To prepare dTSEC block for transfer use the following call sequence:
 * - fman_dtsec_defconfig() - This step is optional and yet recommended. Its
 * use is to obtain the default dTSEC configuration parameters.
 * - Change dtsec configuration in &dtsec_cfg. This structure will be used
 * to customize the dTSEC behavior.
 * - fman_dtsec_init() - Applies the configuration on dTSEC hardware. Note that
 * dTSEC is initialized while both Tx and Rx are disabled.
 * - fman_dtsec_set_mac_address() - Set the station address (mac address).
 * This is used by dTSEC to match against received packets.
 *
 * - fman_dtsec_adjust_link() - Set the link speed and duplex parameters
 * after the PHY establishes the link.
 * - dtsec_enable_tx() and dtsec_enable_rx() to enable transmission and
 * reception.
 */

/*  dTSEC Graceful stop
 * Tx graceful stop is not supported due to the following errata:
 *	P4080 - DTSEC_A004
 *	All other devices - DTSEC_A0014
 * To temporary stop dTSEC activity use fman_dtsec_stop_rx().
 * Note that this function requests dTSEC graceful stop but return before
 * this stop is complete. To query for graceful stop completion use
 * fman_dtsec_get_event() and check DTSEC_IEVENT_GRSC bis.
 * Alternatively the dTSEC interrupt mask can be set to enable graceful stop
 * interrupts.
 * To resume operation after graceful stop use fman_dtsec_start_tx() and
 * fman_dtsec_start_rx().
 */

/* dTSEC interrupt handling
 *
 * This code does not provide an interrupt handler for dTSEC. Instead this
 * handler should be implemented and registered to the operating system by the
 * caller. Some primitives for accessing the event status and mask registers
 * are provided.
 * See "dTSEC Events" section for a list of events that dTSEC can generate.
 */

/* dTSEC Events
 * Interrupt events cause dTSEC event bits to be set. Software may poll the
 * event register at any time to check for pending interrupts. If an event
 * occurs and its corresponding enable bit is set in the interrupt mask
 * register, the event also causes a hardware interrupt at the PIC.
 * To poll for event status use the fman_dtsec_get_event() function.
 * To configure the interrupt mask use fman_dtsec_enable_interrupt() and
 * fman_dtsec_disable_interrupt() functions.
 * After servicing a dTSEC interrupt use fman_dtsec_ack_event to reset the
 * serviced event bit.
 * The following events may be signaled by dTSEC hardware:
 * %DTSEC_IEVENT_BABR - Babbling receive error. This bit indicates that
 * a frame was received with length in excess of the MAC's maximum frame length
 * register.
 * %DTSEC_IEVENT_RXC - Receive control (pause frame) interrupt. A pause
 * control frame was received while Rx pause frame handling is enabled.
 * Also see fman_dtsec_handle_rx_pause().
 * %DTSEC_IEVENT_MSRO - MIB counter overflow. The count for one of the MIB
 * counters has exceeded the size of its register.
 * %DTSEC_IEVENT_GTSC - Graceful transmit stop complete. Graceful stop is now
 * complete. The transmitter is in a stopped state, in which only pause frames
 * can be transmitted.
 * Also see fman_dtsec_stop_tx().
 * %DTSEC_IEVENT_BABT - Babbling transmit error. The transmitted frame length
 * has exceeded the value in the MAC's Maximum Frame Length register.
 * %DTSEC_IEVENT_TXC - Transmit control (pause frame) interrupt. his bit
 * indicates that a control frame was transmitted.
 * %DTSEC_IEVENT_TXE - Transmit error. This bit indicates that an error
 * occurred on the transmitted channel. This bit is set whenever any transmit
 * error occurs which causes the dTSEC to discard all or part of a frame
 * (LC, CRL, XFUN).
 * %DTSEC_IEVENT_LC - Late collision. This bit indicates that a collision
 * occurred beyond the collision window (slot time) in half-duplex mode.
 * The frame is truncated with a bad CRC and the remainder of the frame
 * is discarded.
 * %DTSEC_IEVENT_CRL - Collision retry limit. is bit indicates that the number
 * of successive transmission collisions has exceeded the MAC's half-duplex
 * register's retransmission maximum count. The frame is discarded without
 * being transmitted and transmission of the next frame commences. This only
 * occurs while in half-duplex mode.
 * The number of retransmit attempts can be set in
 * &dtsec_halfdup_cfg.retransmit before calling fman_dtsec_init().
 * %DTSEC_IEVENT_XFUN - Transmit FIFO underrun. This bit indicates that the
 * transmit FIFO became empty before the complete frame was transmitted.
 * The frame is truncated with a bad CRC and the remainder of the frame is
 * discarded.
 * %DTSEC_IEVENT_MAG - TBD
 * %DTSEC_IEVENT_MMRD - MII management read completion.
 * %DTSEC_IEVENT_MMWR - MII management write completion.
 * %DTSEC_IEVENT_GRSC - Graceful receive stop complete. It allows the user to
 * know if the system has completed the stop and it is safe to write to receive
 * registers (status, control or configuration registers) that are used by the
 * system during normal operation.
 * %DTSEC_IEVENT_TDPE - Internal data error on transmit. This bit indicates
 * that the dTSEC has detected a parity error on its stored transmit data, which
 * is likely to compromise the validity of recently transferred frames.
 * %DTSEC_IEVENT_RDPE - Internal data error on receive. This bit indicates that
 * the dTSEC has detected a parity error on its stored receive data, which is
 * likely to compromise the validity of recently transferred frames.
 */
/* Interrupt Mask Register (IMASK) */
#define DTSEC_IMASK_BREN	0x80000000
#define DTSEC_IMASK_RXCEN	0x40000000
#define DTSEC_IMASK_MSROEN	0x04000000
#define DTSEC_IMASK_GTSCEN	0x02000000
#define DTSEC_IMASK_BTEN	0x01000000
#define DTSEC_IMASK_TXCEN	0x00800000
#define DTSEC_IMASK_TXEEN	0x00400000
#define DTSEC_IMASK_LCEN	0x00040000
#define DTSEC_IMASK_CRLEN	0x00020000
#define DTSEC_IMASK_XFUNEN	0x00010000
#define DTSEC_IMASK_ABRTEN	0x00008000
#define DTSEC_IMASK_IFERREN	0x00004000
#define DTSEC_IMASK_MAGEN	0x00000800
#define DTSEC_IMASK_MMRDEN	0x00000400
#define DTSEC_IMASK_MMWREN	0x00000200
#define DTSEC_IMASK_GRSCEN	0x00000100
#define DTSEC_IMASK_TDPEEN	0x00000002
#define DTSEC_IMASK_RDPEEN	0x00000001

#define DTSEC_EVENTS_MASK				\
	((u32)(DTSEC_IMASK_BREN	| \
				DTSEC_IMASK_RXCEN   | \
				DTSEC_IMASK_BTEN    | \
				DTSEC_IMASK_TXCEN   | \
				DTSEC_IMASK_TXEEN   | \
				DTSEC_IMASK_ABRTEN  | \
				DTSEC_IMASK_LCEN    | \
				DTSEC_IMASK_CRLEN   | \
				DTSEC_IMASK_XFUNEN  | \
				DTSEC_IMASK_IFERREN | \
				DTSEC_IMASK_MAGEN   | \
				DTSEC_IMASK_TDPEEN  | \
				DTSEC_IMASK_RDPEEN))

/* dtsec timestamp event bits */
#define TMR_PEMASK_TSREEN	0x00010000
#define TMR_PEVENT_TSRE		0x00010000

/* Group address bit indication */
#define MAC_GROUP_ADDRESS	0x0000010000000000ULL
/* size in bytes of L2 address */
#define MAC_ADDRLEN		6

#define DEFAULT_HALFDUP_ON		false
#define DEFAULT_HALFDUP_RETRANSMIT	0xf
#define DEFAULT_HALFDUP_COLL_WINDOW	0x37
#define DEFAULT_HALFDUP_EXCESS_DEFER	true
#define DEFAULT_HALFDUP_NO_BACKOFF	false
#define DEFAULT_HALFDUP_BP_NO_BACKOFF	false
#define DEFAULT_HALFDUP_ALT_BACKOFF_VAL	0x0A
#define DEFAULT_HALFDUP_ALT_BACKOFF_EN	false
#define DEFAULT_RX_DROP_BCAST		false
#define DEFAULT_RX_SHORT_FRM		true
#define DEFAULT_RX_LEN_CHECK		false
#define DEFAULT_TX_PAD_CRC		true
#define DEFAULT_TX_CRC			false
#define DEFAULT_RX_CTRL_ACC		false
#define DEFAULT_TX_PAUSE_TIME		0xf000
#define DEFAULT_TBIPA			5
#define DEFAULT_RX_PREPEND		0
#define DEFAULT_PTP_TSU_EN		true
#define DEFAULT_PTP_EXCEPTION_EN	true
#define DEFAULT_PREAMBLE_LEN		7
#define DEFAULT_RX_PREAMBLE		false
#define DEFAULT_TX_PREAMBLE		false
#define DEFAULT_LOOPBACK		false
#define DEFAULT_RX_TIME_STAMP_EN	false
#define DEFAULT_TX_TIME_STAMP_EN	false
#define DEFAULT_RX_FLOW			true
#define DEFAULT_TX_FLOW			true
#define DEFAULT_RX_GROUP_HASH_EXD	false
#define DEFAULT_TX_PAUSE_TIME_EXTD	0
#define DEFAULT_RX_PROMISC		false
#define DEFAULT_NON_BACK_TO_BACK_IPG1	0x40
#define DEFAULT_NON_BACK_TO_BACK_IPG2	0x60
#define DEFAULT_MIN_IFG_ENFORCEMENT	0x50
#define DEFAULT_BACK_TO_BACK_IPG	0x60
#define DEFAULT_MAXIMUM_FRAME		0x600
#define DEFAULT_TBI_PHY_ADDR		5
#define DEFAULT_WAKE_ON_LAN		false

/* register related defines (bits, field offsets..) */
#define DTSEC_ID1_ID			0xffff0000
#define DTSEC_ID1_REV_MJ		0x0000FF00
#define DTSEC_ID1_REV_MN		0x000000ff

#define DTSEC_ID2_INT_REDUCED_OFF	0x00010000
#define DTSEC_ID2_INT_NORMAL_OFF	0x00020000

#define DTSEC_ECNTRL_CLRCNT		0x00004000
#define DTSEC_ECNTRL_AUTOZ		0x00002000
#define DTSEC_ECNTRL_STEN		0x00001000
#define DTSEC_ECNTRL_CFG_RO		0x80000000
#define DTSEC_ECNTRL_GMIIM		0x00000040
#define DTSEC_ECNTRL_TBIM		0x00000020
#define DTSEC_ECNTRL_SGMIIM		0x00000002
#define DTSEC_ECNTRL_RPM		0x00000010
#define DTSEC_ECNTRL_R100M		0x00000008
#define DTSEC_ECNTRL_RMM		0x00000004
#define DTSEC_ECNTRL_QSGMIIM		0x00000001

#define DTSEC_TCTRL_THDF		0x00000800
#define DTSEC_TCTRL_TTSE		0x00000040
#define DTSEC_TCTRL_GTS			0x00000020
#define DTSEC_TCTRL_TFC_PAUSE		0x00000010

#define RCTRL_PAL_MASK		0x001f0000
#define RCTRL_PAL_SHIFT		16
#define RCTRL_CFA		0x00008000
#define RCTRL_GHTX		0x00000400
#define RCTRL_RTSE		0x00000040
#define RCTRL_GRS		0x00000020
#define RCTRL_BC_REJ		0x00000010
#define RCTRL_MPROM		0x00000008
#define RCTRL_RSF		0x00000004
#define RCTRL_UPROM		0x00000001
#define RCTRL_PROM		(RCTRL_UPROM | RCTRL_MPROM)

#define TMR_CTL_ESFDP		0x00000800
#define TMR_CTL_ESFDE		0x00000400

#define MACCFG1_SOFT_RESET	0x80000000
#define MACCFG1_LOOPBACK	0x00000100
#define MACCFG1_RX_FLOW		0x00000020
#define MACCFG1_TX_FLOW		0x00000010
#define MACCFG1_TX_EN		0x00000001
#define MACCFG1_RX_EN		0x00000004

#define MACCFG2_NIBBLE_MODE		0x00000100
#define MACCFG2_BYTE_MODE		0x00000200
#define MACCFG2_PRE_AM_RX_EN		0x00000080
#define MACCFG2_PRE_AM_TX_EN		0x00000040
#define MACCFG2_LENGTH_CHECK		0x00000010
#define MACCFG2_MAGIC_PACKET_EN	0x00000008
#define MACCFG2_PAD_CRC_EN		0x00000004
#define MACCFG2_CRC_EN			0x00000002
#define MACCFG2_FULL_DUPLEX		0x00000001
#define MACCFG2_PREAMBLE_LENGTH_MASK	0x0000f000
#define MACCFG2_PREAMBLE_LENGTH_SHIFT	12

#define IPGIFG_NON_BACK_TO_BACK_IPG_1_SHIFT	24
#define IPGIFG_NON_BACK_TO_BACK_IPG_2_SHIFT	16
#define IPGIFG_MIN_IFG_ENFORCEMENT_SHIFT	8

#define IPGIFG_NON_BACK_TO_BACK_IPG_1	0x7F000000
#define IPGIFG_NON_BACK_TO_BACK_IPG_2	0x007F0000
#define IPGIFG_MIN_IFG_ENFORCEMENT	0x0000FF00
#define IPGIFG_BACK_TO_BACK_IPG		0x0000007F

#define HAFDUP_ALT_BEB				0x00080000
#define HAFDUP_BP_NO_BACKOFF			0x00040000
#define HAFDUP_NO_BACKOFF			0x00020000
#define HAFDUP_EXCESS_DEFER			0x00010000
#define HAFDUP_COLLISION_WINDOW		0x000003ff
#define HAFDUP_ALTERNATE_BEB_TRUNCATION_MASK	0x00f00000
#define HAFDUP_ALTERNATE_BEB_TRUNCATION_SHIFT	20
#define HAFDUP_RETRANSMISSION_MAX_SHIFT	12
#define HAFDUP_RETRANSMISSION_MAX		0x0000f000

#define NUM_OF_HASH_REGS	8	/* Number of hash table registers */

#define PTV_PTE_MASK		0xffff0000
#define PTV_PT_MASK		0x0000ffff
#define PTV_PTE_SHIFT		16

/* CAR1/2 bits */
#define DTSEC_CAR1_TR64		0x80000000
#define DTSEC_CAR1_TR127	0x40000000
#define DTSEC_CAR1_TR255	0x20000000
#define DTSEC_CAR1_TR511	0x10000000
#define DTSEC_CAR1_TRK1		0x08000000
#define DTSEC_CAR1_TRMAX	0x04000000
#define DTSEC_CAR1_TRMGV	0x02000000

#define DTSEC_CAR1_RBYT		0x00010000
#define DTSEC_CAR1_RPKT		0x00008000
#define DTSEC_CAR1_RFCS		0x00004000
#define DTSEC_CAR1_RMCA		0x00002000
#define DTSEC_CAR1_RBCA		0x00001000
#define DTSEC_CAR1_RXCF		0x00000800
#define DTSEC_CAR1_RXPF		0x00000400
#define DTSEC_CAR1_RXUO		0x00000200
#define DTSEC_CAR1_RALN		0x00000100
#define DTSEC_CAR1_RFLR		0x00000080
#define DTSEC_CAR1_RCDE		0x00000040
#define DTSEC_CAR1_RCSE		0x00000020
#define DTSEC_CAR1_RUND		0x00000010
#define DTSEC_CAR1_ROVR		0x00000008
#define DTSEC_CAR1_RFRG		0x00000004
#define DTSEC_CAR1_RJBR		0x00000002
#define DTSEC_CAR1_RDRP		0x00000001

#define DTSEC_CAR2_TJBR		0x00080000
#define DTSEC_CAR2_TFCS		0x00040000
#define DTSEC_CAR2_TXCF		0x00020000
#define DTSEC_CAR2_TOVR		0x00010000
#define DTSEC_CAR2_TUND		0x00008000
#define DTSEC_CAR2_TFRG		0x00004000
#define DTSEC_CAR2_TBYT		0x00002000
#define DTSEC_CAR2_TPKT		0x00001000
#define DTSEC_CAR2_TMCA		0x00000800
#define DTSEC_CAR2_TBCA		0x00000400
#define DTSEC_CAR2_TXPF		0x00000200
#define DTSEC_CAR2_TDFR		0x00000100
#define DTSEC_CAR2_TEDF		0x00000080
#define DTSEC_CAR2_TSCL		0x00000040
#define DTSEC_CAR2_TMCL		0x00000020
#define DTSEC_CAR2_TLCL		0x00000010
#define DTSEC_CAR2_TXCL		0x00000008
#define DTSEC_CAR2_TNCL		0x00000004
#define DTSEC_CAR2_TDRP		0x00000001

/* memory map */
struct dtsec_regs {
	/* dTSEC General Control and Status Registers */
	u32 tsec_id;		/* 0x000 ETSEC_ID register */
	u32 tsec_id2;		/* 0x004 ETSEC_ID2 register */
	u32 ievent;		/* 0x008 Interrupt event register */
	u32 imask;		/* 0x00C Interrupt mask register */
	u32 reserved0010[1];
	u32 ecntrl;		/* 0x014 E control register */
	u32 ptv;		/* 0x018 Pause time value register */
	u32 tbipa;		/* 0x01C TBI PHY address register */
	u32 tmr_ctrl;		/* 0x020 Time-stamp Control register */
	u32 tmr_pevent;		/* 0x024 Time-stamp event register */
	u32 tmr_pemask;		/* 0x028 Timer event mask register */
	u32 reserved002c[5];
	u32 tctrl;		/* 0x040 Transmit control register */
	u32 reserved0044[3];
	u32 rctrl;		/* 0x050 Receive control register */
	u32 reserved0054[11];
	u32 igaddr[8];		/* 0x080-0x09C Individual/group address */
	u32 gaddr[8];		/* 0x0A0-0x0BC Group address registers 0-7 */
	u32 reserved00c0[16];
	u32 maccfg1;		/* 0x100 MAC configuration #1 */
	u32 maccfg2;		/* 0x104 MAC configuration #2 */
	u32 ipgifg;		/* 0x108 IPG/IFG */
	u32 hafdup;		/* 0x10C Half-duplex */
	u32 maxfrm;		/* 0x110 Maximum frame */
	u32 reserved0114[10];
	u32 ifstat;		/* 0x13C Interface status */
	u32 macstnaddr1;	/* 0x140 Station Address,part 1 */
	u32 macstnaddr2;	/* 0x144 Station Address,part 2 */
	struct {
		u32 exact_match1;	/* octets 1-4 */
		u32 exact_match2;	/* octets 5-6 */
	} macaddr[15];		/* 0x148-0x1BC mac exact match addresses 1-15 */
	u32 reserved01c0[16];
	u32 tr64;	/* 0x200 Tx and Rx 64 byte frame counter */
	u32 tr127;	/* 0x204 Tx and Rx 65 to 127 byte frame counter */
	u32 tr255;	/* 0x208 Tx and Rx 128 to 255 byte frame counter */
	u32 tr511;	/* 0x20C Tx and Rx 256 to 511 byte frame counter */
	u32 tr1k;	/* 0x210 Tx and Rx 512 to 1023 byte frame counter */
	u32 trmax;	/* 0x214 Tx and Rx 1024 to 1518 byte frame counter */
	u32 trmgv;
	/* 0x218 Tx and Rx 1519 to 1522 byte good VLAN frame count */
	u32 rbyt;	/* 0x21C receive byte counter */
	u32 rpkt;	/* 0x220 receive packet counter */
	u32 rfcs;	/* 0x224 receive FCS error counter */
	u32 rmca;	/* 0x228 RMCA Rx multicast packet counter */
	u32 rbca;	/* 0x22C Rx broadcast packet counter */
	u32 rxcf;	/* 0x230 Rx control frame packet counter */
	u32 rxpf;	/* 0x234 Rx pause frame packet counter */
	u32 rxuo;	/* 0x238 Rx unknown OP code counter */
	u32 raln;	/* 0x23C Rx alignment error counter */
	u32 rflr;	/* 0x240 Rx frame length error counter */
	u32 rcde;	/* 0x244 Rx code error counter */
	u32 rcse;	/* 0x248 Rx carrier sense error counter */
	u32 rund;	/* 0x24C Rx undersize packet counter */
	u32 rovr;	/* 0x250 Rx oversize packet counter */
	u32 rfrg;	/* 0x254 Rx fragments counter */
	u32 rjbr;	/* 0x258 Rx jabber counter */
	u32 rdrp;	/* 0x25C Rx drop */
	u32 tbyt;	/* 0x260 Tx byte counter */
	u32 tpkt;	/* 0x264 Tx packet counter */
	u32 tmca;	/* 0x268 Tx multicast packet counter */
	u32 tbca;	/* 0x26C Tx broadcast packet counter */
	u32 txpf;	/* 0x270 Tx pause control frame counter */
	u32 tdfr;	/* 0x274 Tx deferral packet counter */
	u32 tedf;	/* 0x278 Tx excessive deferral packet counter */
	u32 tscl;	/* 0x27C Tx single collision packet counter */
	u32 tmcl;	/* 0x280 Tx multiple collision packet counter */
	u32 tlcl;	/* 0x284 Tx late collision packet counter */
	u32 txcl;	/* 0x288 Tx excessive collision packet counter */
	u32 tncl;	/* 0x28C Tx total collision counter */
	u32 reserved0290[1];
	u32 tdrp;	/* 0x294 Tx drop frame counter */
	u32 tjbr;	/* 0x298 Tx jabber frame counter */
	u32 tfcs;	/* 0x29C Tx FCS error counter */
	u32 txcf;	/* 0x2A0 Tx control frame counter */
	u32 tovr;	/* 0x2A4 Tx oversize frame counter */
	u32 tund;	/* 0x2A8 Tx undersize frame counter */
	u32 tfrg;	/* 0x2AC Tx fragments frame counter */
	u32 car1;	/* 0x2B0 carry register one register* */
	u32 car2;	/* 0x2B4 carry register two register* */
	u32 cam1;	/* 0x2B8 carry register one mask register */
	u32 cam2;	/* 0x2BC carry register two mask register */
	u32 reserved02c0[848];
};

/* struct dtsec_cfg - dTSEC configuration
 * Transmit half-duplex flow control, under software
 * control for 10/100-Mbps half-duplex media. If set,
 * back pressure is applied to media by raising carrier.
 * halfdup_retransmit:
 * Number of retransmission attempts following a collision.
 * If this is exceeded dTSEC aborts transmission due to
 * excessive collisions. The standard specifies the
 * attempt limit to be 15.
 * halfdup_coll_window:
 * The number of bytes of the frame during which
 * collisions may occur. The default value of 55
 * corresponds to the frame byte at the end of the
 * standard 512-bit slot time window. If collisions are
 * detected after this byte, the late collision event is
 * asserted and transmission of current frame is aborted.
 * rx_drop_bcast:
 * Discard broadcast frames. If set, all broadcast frames
 * will be discarded by dTSEC.
 * rx_short_frm:
 * Accept short frames. If set, dTSEC will accept frames
 * of length 14..63 bytes.
 * rx_len_check:
 * Length check for received frames. If set, the MAC
 * checks the frame's length field on receive to ensure it
 * matches the actual data field length. This only works
 * for received frames with length field less than 1500.
 * No check is performed for larger frames.
 * tx_pad_crc:
 * Pad and append CRC. If set, the MAC pads all
 * transmitted short frames and appends a CRC to every
 * frame regardless of padding requirement.
 * tx_crc:
 * Transmission CRC enable. If set, the MAC appends a CRC
 * to all frames. If frames presented to the MAC have a
 * valid length and contain a valid CRC, tx_crc should be
 * reset.
 * This field is ignored if tx_pad_crc is set.
 * rx_ctrl_acc:
 * Control frame accept. If set, this overrides 802.3
 * standard control frame behavior, and all Ethernet frames
 * that have an ethertype of 0x8808 are treated as normal
 * Ethernet frames and passed up to the packet interface on
 * a DA match. Received pause control frames are passed to
 * the packet interface only if Rx flow control is also
 * disabled. See fman_dtsec_handle_rx_pause() function.
 * tx_pause_time:
 * Transmit pause time value. This pause value is used as
 * part of the pause frame to be sent when a transmit pause
 * frame is initiated. If set to 0 this disables
 * transmission of pause frames.
 * rx_preamble:
 * Receive preamble enable. If set, the MAC recovers the
 * received Ethernet 7-byte preamble and passes it to the
 * packet interface at the start of each received frame.
 * This field should be reset for internal MAC loop-back
 * mode.
 * tx_preamble:	User defined preamble enable for transmitted frames.
 * If set, a user-defined preamble must passed to the MAC
 * and it is transmitted instead of the standard preamble.
 * preamble_len:
 * Length, in bytes, of the preamble field preceding each
 * Ethernet start-of-frame delimiter byte. The default
 * value of 0x7 should be used in order to guarantee
 * reliable operation with IEEE 802.3 compliant hardware.
 * rx_prepend:
 * Packet alignment padding length. The specified number
 * of bytes (1-31) of zero padding are inserted before the
 * start of each received frame. For Ethernet, where
 * optional preamble extraction is enabled, the padding
 * appears before the preamble, otherwise the padding
 * precedes the layer 2 header.
 *
 * This structure contains basic dTSEC configuration and must be passed to
 * fman_dtsec_init() function. A default set of configuration values can be
 * obtained by calling fman_dtsec_defconfig().
 */
struct dtsec_cfg {
	bool halfdup_on;
	bool halfdup_alt_backoff_en;
	bool halfdup_excess_defer;
	bool halfdup_no_backoff;
	bool halfdup_bp_no_backoff;
	u32 halfdup_alt_backoff_val;
	u16 halfdup_retransmit;
	u16 halfdup_coll_window;
	bool rx_drop_bcast;
	bool rx_short_frm;
	bool rx_len_check;
	bool tx_pad_crc;
	bool tx_crc;
	bool rx_ctrl_acc;
	u16 tx_pause_time;
	u16 tbipa;
	bool ptp_tsu_en;
	bool ptp_exception_en;
	bool rx_preamble;
	bool tx_preamble;
	u32 preamble_len;
	u32 rx_prepend;
	bool loopback;
	bool rx_time_stamp_en;
	bool tx_time_stamp_en;
	bool rx_flow;
	bool tx_flow;
	bool rx_group_hash_exd;
	bool rx_promisc;
	u8 tbi_phy_addr;
	u16 tx_pause_time_extd;
	u16 maximum_frame;
	u32 non_back_to_back_ipg1;
	u32 non_back_to_back_ipg2;
	u32 min_ifg_enforcement;
	u32 back_to_back_ipg;
	bool wake_on_lan;
};

/**
 * fman_dtsec_defconfig() - Get default dTSEC configuration
 * @cfg:	pointer to configuration structure.
 *
 * Call this function to obtain a default set of configuration values for
 * initializing dTSEC. The user can overwrite any of the values before calling
 * fman_dtsec_init(), if specific configuration needs to be applied.
 */
void fman_dtsec_defconfig(struct dtsec_cfg *cfg);

/**
 * fman_dtsec_init() - Init dTSEC hardware block
 * @regs:		Pointer to dTSEC register block
 * @cfg:		dTSEC configuration data
 * @iface_mode:		dTSEC interface mode, the type of MAC - PHY interface.
 * @iface_speed:	1G or 10G
 * @macaddr:		MAC station address to be assigned to the device
 * @fm_rev_maj:		major rev number
 * @fm_rev_min:		minor rev number
 * @exceptions_mask:	initial exceptions mask
 *
 * This function initializes dTSEC and applies basic configuration.
 *
 * dTSEC initialization sequence:
 * Before enabling Rx/Tx call dtsec_set_address() to set MAC address,
 * fman_dtsec_adjust_link() to configure interface speed and duplex and finally
 * dtsec_enable_tx()/dtsec_enable_rx() to start transmission and reception.
 *
 * Return: 0 if successful, an error code otherwise.
 */
int fman_dtsec_init(struct dtsec_regs __iomem *regs, struct dtsec_cfg *cfg,
		    enum enet_interface iface_mode,
		    enum enet_speed iface_speed,
		    u8 *macaddr, u8 fm_rev_maj,
		    u8 fm_rev_min, u32 exception_mask);

/**
 * fman_dtsec_enable() - Enable dTSEC Tx and Tx
 * @regs:	Pointer to dTSEC register block
 * @apply_rx:	enable rx side
 * @apply_tx:	enable tx side
 *
 * This function resets Tx and Rx graceful stop bit and enables dTSEC Tx and Rx.
 */
void fman_dtsec_enable(struct dtsec_regs __iomem *regs, bool apply_rx,
		       bool apply_tx);

/**
 * fman_dtsec_disable() - Disable dTSEC Tx and Rx
 * @regs:	Pointer to dTSEC register block
 * @apply_rx:	disable rx side
 * @apply_tx:	disable tx side
 *
 * This function disables Tx and Rx in dTSEC.
 */
void fman_dtsec_disable(struct dtsec_regs __iomem *regs, bool apply_rx,
			bool apply_tx);

/**
 * fman_dtsec_get_revision() - Get dTSEC hardware revision
 * @regs:	  Pointer to dTSEC register block
 *
 * Call this function to obtain the dTSEC hardware version.
 * Return: dtsec_id content
 */
u32 fman_dtsec_get_revision(struct dtsec_regs __iomem *regs);

/**
 * fman_dtsec_set_mac_address() - Set MAC station address
 * @regs:	  Pointer to dTSEC register block
 * @macaddr:    MAC address array
 *
 * This function sets MAC station address. To enable unicast reception call
 * this after fman_dtsec_init(). While promiscuous mode is disabled dTSEC will
 * match the destination address of received unicast frames against this
 * address.
 */
void fman_dtsec_set_mac_address(struct dtsec_regs __iomem *regs, u8 *macaddr);

/**
 * fman_dtsec_set_uc_promisc() - Sets unicast promiscuous mode
 * @regs:	Pointer to dTSEC register block
 * @enable:	Enable unicast promiscuous mode
 *
 * Use this function to enable/disable dTSEC L2 address filtering. If the
 * address filtering is disabled all unicast packets are accepted.
 * To set dTSEC in promiscuous mode call both fman_dtsec_set_uc_promisc() and
 * fman_dtsec_set_mc_promisc() to disable filtering for both unicast and
 * multicast addresses.
 */
void fman_dtsec_set_uc_promisc(struct dtsec_regs __iomem *regs, bool enable);

/**
 * fman_dtsec_adjust_link() - Adjust dTSEC speed/duplex settings
 * @regs:	Pointer to dTSEC register block
 * @iface_mode: dTSEC interface mode
 * @speed:	Link speed
 * @full_dx:	True for full-duplex, false for half-duplex.
 *
 * This function configures the MAC to function and the desired rates. Use it
 * to configure dTSEC after fman_dtsec_init() and whenever the link speed
 * changes (for instance following PHY auto-negociation).
 * Returns: 0 if successful, an error code otherwise.
 */
int fman_dtsec_adjust_link(struct dtsec_regs __iomem *regs,
			   enum enet_interface iface_mode,
			   enum enet_speed speed, bool full_dx);

/**
 * fman_dtsec_set_max_frame_len() - Set max frame length
 * @regs:	Pointer to dTSEC register block
 * @length:	Max frame length.
 *
 * Sets maximum frame length for received and transmitted frames. Frames that
 * exceeds this length are truncated.
 */
void fman_dtsec_set_max_frame_len(struct dtsec_regs __iomem *regs, u16 length);

/**
 * fman_dtsec_get_max_frame_len() - Query max frame length
 * @regs:	Pointer to dTSEC register block
 *
 * Return: the current value of the maximum frame length.
 */
u16 fman_dtsec_get_max_frame_len(struct dtsec_regs __iomem *regs);

/**
 * fman_dtsec_handle_rx_pause() - Configure pause frame handling
 * @regs:	Pointer to dTSEC register block
 * @en:		Enable pause frame handling in dTSEC
 *
 * If enabled, dTSEC will handle pause frames internally. This must be disabled
 * if dTSEC is set in half-duplex mode.
 * If pause frame handling is disabled and &dtsec_cfg.rx_ctrl_acc is set, pause
 * frames will be transferred to the packet interface just like regular Ethernet
 * frames.
 */
void fman_dtsec_handle_rx_pause(struct dtsec_regs __iomem *regs, bool en);

/**
 * fman_dtsec_set_tx_pause_frames() - Configure Tx pause time
 * @regs:	Pointer to dTSEC register block
 * @time:	Time value included in pause frames
 *
 * Call this function to set the time value used in transmitted pause frames.
 * If time is 0, transmission of pause frames is disabled
 */
void fman_dtsec_set_tx_pause_frames(struct dtsec_regs __iomem *regs, u16 time);

/**
 * fman_dtsec_ack_event() - Acknowledge handled events
 * @regs:	Pointer to dTSEC register block
 * @ev_mask:	Events to acknowledge
 *
 * After handling events signaled by dTSEC in either polling or interrupt mode,
 * call this function to reset the associated status bits in dTSEC event
 * register.
 */
void fman_dtsec_ack_event(struct dtsec_regs __iomem *regs, u32 ev_mask);

/**
 * fman_dtsec_get_event() - Returns currently asserted events
 * @regs:	Pointer to dTSEC register block
 * @ev_mask:	Mask of relevant events
 *
 * Call this function to obtain a bit-mask of events that are currently asserted
 * in dTSEC, taken from IEVENT register.
 * Returns: a bit-mask of events asserted in dTSEC.
 */
u32 fman_dtsec_get_event(struct dtsec_regs __iomem *regs, u32 ev_mask);

/**
 * fman_dtsec_get_interrupt_mask() - Returns a bit-mask of enabled interrupts
 * @regs:	  Pointer to dTSEC register block
 *
 * Call this function to obtain a bit-mask of enabled interrupts
 * in dTSEC, taken from IMASK register.
 * Returns: a bit-mask of enabled interrupts in dTSEC.
 */
u32 fman_dtsec_get_interrupt_mask(struct dtsec_regs __iomem *regs);

void fman_dtsec_enable_tmr_interrupt(struct dtsec_regs __iomem *regs);

void fman_dtsec_disable_tmr_interrupt(struct dtsec_regs __iomem *regs);

/**
 * fman_dtsec_disable_interrupt() - Disables interrupts for the specified events
 * @regs:	Pointer to dTSEC register block
 * @ev_mask:	Mask of relevant events
 *
 * Call this function to disable interrupts in dTSEC for the specified events.
 * To enable interrupts use fman_dtsec_enable_interrupt().
 */
void fman_dtsec_disable_interrupt(struct dtsec_regs __iomem *regs, u32 ev_mask);

/**
 * fman_dtsec_enable_interrupt() - Enable interrupts for the specified events
 * @regs:	Pointer to dTSEC register block
 * @ev_mask:	Mask of relevant events
 *
 * Call this function to enable interrupts in dTSEC for the specified events.
 * To disable interrupts use fman_dtsec_disable_interrupt().
 */
void fman_dtsec_enable_interrupt(struct dtsec_regs __iomem *regs, u32 ev_mask);

/**
 * fman_dtsec_set_bucket() - Enables/disables a filter bucket
 * @regs:	Pointer to dTSEC register block
 * @bucket:	Bucket index
 * @enable:	true/false to enable/disable this bucket
 *
 * This function enables or disables the specified bucket. Enabling a bucket
 * associated with an address configures dTSEC to accept received packets
 * with that destination address.
 * Multiple addresses may be associated with the same bucket. Disabling a
 * bucket will affect all addresses associated with that bucket. A bucket that
 * is enabled requires further filtering and verification in the upper layers
 */
void fman_dtsec_set_bucket(struct dtsec_regs __iomem *regs, int bucket,
			   bool enable);

/**
 * fman_dtsec_set_mc_promisc() - Set multicast promiscuous mode
 * @regs:	Pointer to dTSEC register block
 * @enable:	Enable multicast promiscuous mode
 *
 * Call this to enable/disable L2 address filtering for multicast packets.
 */
void fman_dtsec_set_mc_promisc(struct dtsec_regs __iomem *regs, bool enable);

/**
 * fman_dtsec_get_clear_carry_regs() - Read and clear carry bits
 * @regs:	Pointer to dTSEC register block
 * @car1:	car1 register value
 * @car2:	car2 register value
 *
 * When set, the carry bits signal that an overflow occurred on the
 * corresponding counters.
 * Note that the carry bits (CAR1-2 registers) will assert the
 * %DTSEC_IEVENT_MSRO interrupt if unmasked (via CAM1-2 regs).
 * Return: true if overflow occurred, otherwise - false
 */
bool fman_dtsec_get_clear_carry_regs(struct dtsec_regs __iomem *regs,
				     u32 *car1, u32 *car2);

u32 fman_dtsec_check_and_clear_tmr_event(struct dtsec_regs __iomem *regs);

void fman_dtsec_start_tx(struct dtsec_regs __iomem *regs);
void fman_dtsec_start_rx(struct dtsec_regs __iomem *regs);
void fman_dtsec_stop_rx(struct dtsec_regs __iomem *regs);
u32 fman_dtsec_get_rctrl(struct dtsec_regs __iomem *regs);

#endif	/* __FSL_FMAN_DTSEC_H */
