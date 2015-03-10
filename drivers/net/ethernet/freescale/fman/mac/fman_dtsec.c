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

#include "fsl_fman_dtsec.h"

void fman_dtsec_stop_rx(struct dtsec_regs __iomem *regs)
{
	/* Assert the graceful stop bit */
	iowrite32be(ioread32be(&regs->rctrl) | RCTRL_GRS, &regs->rctrl);
}

void fman_dtsec_start_tx(struct dtsec_regs __iomem *regs)
{
	/* clear the graceful stop bit */
	iowrite32be(ioread32be(&regs->tctrl) & ~DTSEC_TCTRL_GTS, &regs->tctrl);
}

void fman_dtsec_start_rx(struct dtsec_regs __iomem *regs)
{
	/* clear the graceful stop bit */
	iowrite32be(ioread32be(&regs->rctrl) & ~RCTRL_GRS, &regs->rctrl);
}

void fman_dtsec_defconfig(struct dtsec_cfg *cfg)
{
	cfg->halfdup_on = DEFAULT_HALFDUP_ON;
	cfg->halfdup_retransmit = DEFAULT_HALFDUP_RETRANSMIT;
	cfg->halfdup_coll_window = DEFAULT_HALFDUP_COLL_WINDOW;
	cfg->halfdup_excess_defer = DEFAULT_HALFDUP_EXCESS_DEFER;
	cfg->halfdup_no_backoff = DEFAULT_HALFDUP_NO_BACKOFF;
	cfg->halfdup_bp_no_backoff = DEFAULT_HALFDUP_BP_NO_BACKOFF;
	cfg->halfdup_alt_backoff_val = DEFAULT_HALFDUP_ALT_BACKOFF_VAL;
	cfg->halfdup_alt_backoff_en = DEFAULT_HALFDUP_ALT_BACKOFF_EN;
	cfg->rx_drop_bcast = DEFAULT_RX_DROP_BCAST;
	cfg->rx_short_frm = DEFAULT_RX_SHORT_FRM;
	cfg->rx_len_check = DEFAULT_RX_LEN_CHECK;
	cfg->tx_pad_crc = DEFAULT_TX_PAD_CRC;
	cfg->tx_crc = DEFAULT_TX_CRC;
	cfg->rx_ctrl_acc = DEFAULT_RX_CTRL_ACC;
	cfg->tx_pause_time = DEFAULT_TX_PAUSE_TIME;
	/* PHY address 0 is reserved (DPAA RM) */
	cfg->tbipa = DEFAULT_TBIPA;
	cfg->rx_prepend = DEFAULT_RX_PREPEND;
	cfg->ptp_tsu_en = DEFAULT_PTP_TSU_EN;
	cfg->ptp_exception_en = DEFAULT_PTP_EXCEPTION_EN;
	cfg->preamble_len = DEFAULT_PREAMBLE_LEN;
	cfg->rx_preamble = DEFAULT_RX_PREAMBLE;
	cfg->tx_preamble = DEFAULT_TX_PREAMBLE;
	cfg->loopback = DEFAULT_LOOPBACK;
	cfg->rx_time_stamp_en = DEFAULT_RX_TIME_STAMP_EN;
	cfg->tx_time_stamp_en = DEFAULT_TX_TIME_STAMP_EN;
	cfg->rx_flow = DEFAULT_RX_FLOW;
	cfg->tx_flow = DEFAULT_TX_FLOW;
	cfg->rx_group_hash_exd = DEFAULT_RX_GROUP_HASH_EXD;
	cfg->tx_pause_time_extd = DEFAULT_TX_PAUSE_TIME_EXTD;
	cfg->rx_promisc = DEFAULT_RX_PROMISC;
	cfg->non_back_to_back_ipg1 = DEFAULT_NON_BACK_TO_BACK_IPG1;
	cfg->non_back_to_back_ipg2 = DEFAULT_NON_BACK_TO_BACK_IPG2;
	cfg->min_ifg_enforcement = DEFAULT_MIN_IFG_ENFORCEMENT;
	cfg->back_to_back_ipg = DEFAULT_BACK_TO_BACK_IPG;
	cfg->maximum_frame = DEFAULT_MAXIMUM_FRAME;
	cfg->tbi_phy_addr = DEFAULT_TBI_PHY_ADDR;
	cfg->wake_on_lan = DEFAULT_WAKE_ON_LAN;
}

int fman_dtsec_init(struct dtsec_regs __iomem *regs, struct dtsec_cfg *cfg,
		    enum enet_interface iface_mode,
		    enum enet_speed iface_speed,
		    u8 *macaddr,
		    u8 __maybe_unused fm_rev_maj,
		    u8 __maybe_unused fm_rev_min, u32 exception_mask)
{
	bool is_rgmii, is_sgmii, is_qsgmii;
	int i;
	u32 tmp;

	/* Soft reset */
	iowrite32be(MACCFG1_SOFT_RESET, &regs->maccfg1);
	iowrite32be(0, &regs->maccfg1);

	/* dtsec_id2 */
	tmp = ioread32be(&regs->tsec_id2);

	/* check RGMII support */
	if (iface_mode == E_ENET_IF_RGMII || iface_mode == E_ENET_IF_RMII)
		if (tmp & DTSEC_ID2_INT_REDUCED_OFF)
			return -EINVAL;

	if (iface_mode == E_ENET_IF_SGMII || iface_mode == E_ENET_IF_MII)
		if (tmp & DTSEC_ID2_INT_REDUCED_OFF)
			return -EINVAL;

	is_rgmii = (bool)((iface_mode == E_ENET_IF_RGMII) ? true : false);
	is_sgmii = (bool)((iface_mode == E_ENET_IF_SGMII) ? true : false);
	is_qsgmii = (bool)((iface_mode == E_ENET_IF_QSGMII) ? true : false);

	tmp = 0;
	if (is_rgmii || iface_mode == E_ENET_IF_GMII)
		tmp |= DTSEC_ECNTRL_GMIIM;
	if (is_sgmii)
		tmp |= (DTSEC_ECNTRL_SGMIIM | DTSEC_ECNTRL_TBIM);
	if (is_qsgmii)
		tmp |= (DTSEC_ECNTRL_SGMIIM | DTSEC_ECNTRL_TBIM |
			DTSEC_ECNTRL_QSGMIIM);
	if (is_rgmii)
		tmp |= DTSEC_ECNTRL_RPM;
	if (iface_speed == E_ENET_SPEED_100)
		tmp |= DTSEC_ECNTRL_R100M;

	iowrite32be(tmp, &regs->ecntrl);

	tmp = 0;
	if (cfg->halfdup_on)
		tmp |= DTSEC_TCTRL_THDF;
	if (cfg->tx_time_stamp_en)
		tmp |= DTSEC_TCTRL_TTSE;

	iowrite32be(tmp, &regs->tctrl);

	tmp = 0;

	if (cfg->tx_pause_time)
		tmp |= cfg->tx_pause_time;
	if (cfg->tx_pause_time_extd)
		tmp |= cfg->tx_pause_time_extd << PTV_PTE_SHIFT;
	iowrite32be(tmp, &regs->ptv);

	tmp = 0;
	tmp |= (cfg->rx_prepend << RCTRL_PAL_SHIFT) & RCTRL_PAL_MASK;
	if (cfg->rx_ctrl_acc)
		tmp |= RCTRL_CFA;
	if (cfg->rx_group_hash_exd)
		tmp |= RCTRL_GHTX;
	if (cfg->rx_time_stamp_en)
		tmp |= RCTRL_RTSE;
	if (cfg->rx_drop_bcast)
		tmp |= RCTRL_BC_REJ;
	if (cfg->rx_short_frm)
		tmp |= RCTRL_RSF;
	if (cfg->rx_promisc)
		tmp |= RCTRL_PROM;

	iowrite32be(tmp, &regs->rctrl);

	/* Assign a Phy Address to the TBI (TBIPA).
	 * Done also in cases where TBI is not selected to avoid conflict with
	 * the external PHY's Physical address
	 */
	iowrite32be(cfg->tbipa, &regs->tbipa);

	iowrite32be(0, &regs->tmr_ctrl);

	if (cfg->ptp_tsu_en) {
		tmp = 0;
		tmp |= TMR_PEVENT_TSRE;
		iowrite32be(tmp, &regs->tmr_pevent);

		if (cfg->ptp_exception_en) {
			tmp = 0;
			tmp |= TMR_PEMASK_TSREEN;
			iowrite32be(tmp, &regs->tmr_pemask);
		}
	}

	tmp = 0;
	if (cfg->loopback)
		tmp |= MACCFG1_LOOPBACK;
	if (cfg->rx_flow)
		tmp |= MACCFG1_RX_FLOW;
	if (cfg->tx_flow)
		tmp |= MACCFG1_TX_FLOW;
	iowrite32be(tmp, &regs->maccfg1);

	tmp = 0;

	if (iface_speed < E_ENET_SPEED_1000)
		tmp |= MACCFG2_NIBBLE_MODE;
	else if (iface_speed == E_ENET_SPEED_1000)
		tmp |= MACCFG2_BYTE_MODE;

	tmp |= (cfg->preamble_len << MACCFG2_PREAMBLE_LENGTH_SHIFT) &
		MACCFG2_PREAMBLE_LENGTH_MASK;
	if (cfg->rx_preamble)
		tmp |= MACCFG2_PRE_AM_RX_EN;
	if (cfg->tx_preamble)
		tmp |= MACCFG2_PRE_AM_TX_EN;
	if (cfg->rx_len_check)
		tmp |= MACCFG2_LENGTH_CHECK;
	if (cfg->tx_pad_crc)
		tmp |= MACCFG2_PAD_CRC_EN;
	if (cfg->tx_crc)
		tmp |= MACCFG2_CRC_EN;
	if (!cfg->halfdup_on)
		tmp |= MACCFG2_FULL_DUPLEX;
	iowrite32be(tmp, &regs->maccfg2);

	tmp = (((cfg->non_back_to_back_ipg1 <<
		 IPGIFG_NON_BACK_TO_BACK_IPG_1_SHIFT)
		& IPGIFG_NON_BACK_TO_BACK_IPG_1)
	       | ((cfg->non_back_to_back_ipg2 <<
		   IPGIFG_NON_BACK_TO_BACK_IPG_2_SHIFT)
		 & IPGIFG_NON_BACK_TO_BACK_IPG_2)
	       | ((cfg->min_ifg_enforcement << IPGIFG_MIN_IFG_ENFORCEMENT_SHIFT)
		 & IPGIFG_MIN_IFG_ENFORCEMENT)
	       | (cfg->back_to_back_ipg & IPGIFG_BACK_TO_BACK_IPG));
	iowrite32be(tmp, &regs->ipgifg);

	tmp = 0;

	if (cfg->halfdup_alt_backoff_en)
		tmp = HAFDUP_ALT_BEB;
		tmp |= (cfg->halfdup_alt_backoff_val <<
			HAFDUP_ALTERNATE_BEB_TRUNCATION_SHIFT) &
			HAFDUP_ALTERNATE_BEB_TRUNCATION_MASK;
	if (cfg->halfdup_bp_no_backoff)
		tmp |= HAFDUP_BP_NO_BACKOFF;
	if (cfg->halfdup_no_backoff)
		tmp |= HAFDUP_NO_BACKOFF;
	if (cfg->halfdup_excess_defer)
		tmp |= HAFDUP_EXCESS_DEFER;
	tmp |= ((cfg->halfdup_retransmit << HAFDUP_RETRANSMISSION_MAX_SHIFT)
		& HAFDUP_RETRANSMISSION_MAX);
	tmp |= (cfg->halfdup_coll_window & HAFDUP_COLLISION_WINDOW);

	iowrite32be(tmp, &regs->hafdup);

	/* Initialize Maximum frame length */
	iowrite32be(cfg->maximum_frame, &regs->maxfrm);

	iowrite32be(0xffffffff, &regs->cam1);
	iowrite32be(0xffffffff, &regs->cam2);

	iowrite32be(exception_mask, &regs->imask);

	iowrite32be(0xffffffff, &regs->ievent);

	tmp = (u32)((macaddr[5] << 24) |
		    (macaddr[4] << 16) | (macaddr[3] << 8) | macaddr[2]);
	iowrite32be(tmp, &regs->macstnaddr1);

	tmp = (u32)((macaddr[1] << 24) | (macaddr[0] << 16));
	iowrite32be(tmp, &regs->macstnaddr2);

	/* HASH */
	for (i = 0; i < NUM_OF_HASH_REGS; i++) {
		/* Initialize IADDRx */
		iowrite32be(0, &regs->igaddr[i]);
		/* Initialize GADDRx */
		iowrite32be(0, &regs->gaddr[i]);
	}

	return 0;
}

u16 fman_dtsec_get_max_frame_len(struct dtsec_regs __iomem *regs)
{
	return (u16)ioread32be(&regs->maxfrm);
}

void fman_dtsec_set_max_frame_len(struct dtsec_regs __iomem *regs, u16 length)
{
	iowrite32be(length, &regs->maxfrm);
}

void fman_dtsec_set_mac_address(struct dtsec_regs __iomem *regs, u8 *adr)
{
	u32 tmp;

	tmp = (u32)((adr[5] << 24) |
		    (adr[4] << 16) | (adr[3] << 8) | adr[2]);
	iowrite32be(tmp, &regs->macstnaddr1);

	tmp = (u32)((adr[1] << 24) | (adr[0] << 16));
	iowrite32be(tmp, &regs->macstnaddr2);
}

void fman_dtsec_set_bucket(struct dtsec_regs __iomem *regs, int bucket,
			   bool enable)
{
	int reg_idx = (bucket >> 5) & 0xf;
	int bit_idx = bucket & 0x1f;
	u32 bit_mask = 0x80000000 >> bit_idx;
	u32 __iomem *reg;

	if (reg_idx > 7)
		reg = &regs->gaddr[reg_idx - 8];
	else
		reg = &regs->igaddr[reg_idx];

	if (enable)
		iowrite32be(ioread32be(reg) | bit_mask, reg);
	else
		iowrite32be(ioread32be(reg) & (~bit_mask), reg);
}

int fman_dtsec_adjust_link(struct dtsec_regs __iomem *regs,
			   enum enet_interface __maybe_unused iface_mode,
			   enum enet_speed speed, bool full_dx)
{
	u32 tmp;

	if ((speed == E_ENET_SPEED_1000) && !full_dx)
		return -EINVAL;

	tmp = ioread32be(&regs->maccfg2);
	if (!full_dx)
		tmp &= ~MACCFG2_FULL_DUPLEX;
	else
		tmp |= MACCFG2_FULL_DUPLEX;

	tmp &= ~(MACCFG2_NIBBLE_MODE | MACCFG2_BYTE_MODE);
	if (speed < E_ENET_SPEED_1000)
		tmp |= MACCFG2_NIBBLE_MODE;
	else if (speed == E_ENET_SPEED_1000)
		tmp |= MACCFG2_BYTE_MODE;
	iowrite32be(tmp, &regs->maccfg2);

	tmp = ioread32be(&regs->ecntrl);
	if (speed == E_ENET_SPEED_100)
		tmp |= DTSEC_ECNTRL_R100M;
	else
		tmp &= ~DTSEC_ECNTRL_R100M;
	iowrite32be(tmp, &regs->ecntrl);

	return 0;
}

void fman_dtsec_set_uc_promisc(struct dtsec_regs __iomem *regs, bool enable)
{
	u32 tmp;

	tmp = ioread32be(&regs->rctrl);

	if (enable)
		tmp |= RCTRL_UPROM;
	else
		tmp &= ~RCTRL_UPROM;

	iowrite32be(tmp, &regs->rctrl);
}

void fman_dtsec_set_mc_promisc(struct dtsec_regs __iomem *regs, bool enable)
{
	u32 tmp;

	tmp = ioread32be(&regs->rctrl);

	if (enable)
		tmp |= RCTRL_MPROM;
	else
		tmp &= ~RCTRL_MPROM;

	iowrite32be(tmp, &regs->rctrl);
}

void fman_dtsec_enable(struct dtsec_regs __iomem *regs, bool apply_rx,
		       bool apply_tx)
{
	u32 tmp;

	tmp = ioread32be(&regs->maccfg1);

	if (apply_rx)
		tmp |= MACCFG1_RX_EN;

	if (apply_tx)
		tmp |= MACCFG1_TX_EN;

	iowrite32be(tmp, &regs->maccfg1);
}

void fman_dtsec_disable(struct dtsec_regs __iomem *regs, bool apply_rx,
			bool apply_tx)
{
	u32 tmp;

	tmp = ioread32be(&regs->maccfg1);

	if (apply_rx)
		tmp &= ~MACCFG1_RX_EN;

	if (apply_tx)
		tmp &= ~MACCFG1_TX_EN;

	iowrite32be(tmp, &regs->maccfg1);
}

void fman_dtsec_set_tx_pause_frames(struct dtsec_regs __iomem *regs, u16 time)
{
	u32 ptv = 0;

	if (time) {
		ptv = ioread32be(&regs->ptv);
		ptv &= PTV_PTE_MASK;
		ptv |= time & PTV_PT_MASK;
		iowrite32be(ptv, &regs->ptv);

		/* trigger the transmission of a flow-control pause frame */
		iowrite32be(ioread32be(&regs->maccfg1) | MACCFG1_TX_FLOW,
			    &regs->maccfg1);
	} else
		iowrite32be(ioread32be(&regs->maccfg1) & ~MACCFG1_TX_FLOW,
			    &regs->maccfg1);
}

void fman_dtsec_handle_rx_pause(struct dtsec_regs __iomem *regs, bool en)
{
	u32 tmp;

	tmp = ioread32be(&regs->maccfg1);
	if (en)
		tmp |= MACCFG1_RX_FLOW;
	else
		tmp &= ~MACCFG1_RX_FLOW;
	iowrite32be(tmp, &regs->maccfg1);
}

u32 fman_dtsec_get_rctrl(struct dtsec_regs __iomem *regs)
{
	return ioread32be(&regs->rctrl);
}

u32 fman_dtsec_get_revision(struct dtsec_regs __iomem *regs)
{
	return ioread32be(&regs->tsec_id);
}

u32 fman_dtsec_get_event(struct dtsec_regs __iomem *regs, u32 ev_mask)
{
	return ioread32be(&regs->ievent) & ev_mask;
}

void fman_dtsec_ack_event(struct dtsec_regs __iomem *regs, u32 ev_mask)
{
	iowrite32be(ev_mask, &regs->ievent);
}

u32 fman_dtsec_get_interrupt_mask(struct dtsec_regs __iomem *regs)
{
	return ioread32be(&regs->imask);
}

u32 fman_dtsec_check_and_clear_tmr_event(struct dtsec_regs __iomem *regs)
{
	u32 event;

	event = ioread32be(&regs->tmr_pevent);
	event &= ioread32be(&regs->tmr_pemask);

	if (event)
		iowrite32be(event, &regs->tmr_pevent);
	return event;
}

void fman_dtsec_enable_tmr_interrupt(struct dtsec_regs __iomem *regs)
{
	iowrite32be(ioread32be(&regs->tmr_pemask) | TMR_PEMASK_TSREEN,
		    &regs->tmr_pemask);
}

void fman_dtsec_disable_tmr_interrupt(struct dtsec_regs __iomem *regs)
{
	iowrite32be(ioread32be(&regs->tmr_pemask) & ~TMR_PEMASK_TSREEN,
		    &regs->tmr_pemask);
}

void fman_dtsec_enable_interrupt(struct dtsec_regs __iomem *regs, u32 ev_mask)
{
	iowrite32be(ioread32be(&regs->imask) | ev_mask, &regs->imask);
}

void fman_dtsec_disable_interrupt(struct dtsec_regs __iomem *regs, u32 ev_mask)
{
	iowrite32be(ioread32be(&regs->imask) & ~ev_mask, &regs->imask);
}
