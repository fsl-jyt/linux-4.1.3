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

#include "fsl_fman_memac.h"

u32 fman_memac_get_event(struct memac_regs __iomem *regs, u32 ev_mask)
{
	return ioread32be(&regs->ievent) & ev_mask;
}

u32 fman_memac_get_interrupt_mask(struct memac_regs __iomem *regs)
{
	return ioread32be(&regs->imask);
}

void fman_memac_ack_event(struct memac_regs __iomem *regs, u32 ev_mask)
{
	iowrite32be(ev_mask, &regs->ievent);
}

void fman_memac_set_promiscuous(struct memac_regs __iomem *regs, bool val)
{
	u32 tmp;

	tmp = ioread32be(&regs->command_config);

	if (val)
		tmp |= CMD_CFG_PROMIS_EN;
	else
		tmp &= ~CMD_CFG_PROMIS_EN;

	iowrite32be(tmp, &regs->command_config);
}

void fman_memac_clear_addr_in_paddr(struct memac_regs __iomem *regs,
				    u8 paddr_num)
{
	if (paddr_num == 0) {
		iowrite32be(0, &regs->mac_addr0.mac_addr_l);
		iowrite32be(0, &regs->mac_addr0.mac_addr_u);
	} else {
		iowrite32be(0x0, &regs->mac_addr[paddr_num - 1].mac_addr_l);
		iowrite32be(0x0, &regs->mac_addr[paddr_num - 1].mac_addr_u);
	}
}

void fman_memac_add_addr_in_paddr(struct memac_regs __iomem *regs,
				  u8 *adr, u8 paddr_num)
{
	u32 tmp0, tmp1;

	tmp0 = (u32)(adr[0] | adr[1] << 8 | adr[2] << 16 | adr[3] << 24);
	tmp1 = (u32)(adr[4] | adr[5] << 8);

	if (paddr_num == 0) {
		iowrite32be(tmp0, &regs->mac_addr0.mac_addr_l);
		iowrite32be(tmp1, &regs->mac_addr0.mac_addr_u);
	} else {
		iowrite32be(tmp0, &regs->mac_addr[paddr_num - 1].mac_addr_l);
		iowrite32be(tmp1, &regs->mac_addr[paddr_num - 1].mac_addr_u);
	}
}

void fman_memac_enable(struct memac_regs __iomem *regs, bool apply_rx,
		       bool apply_tx)
{
	u32 tmp;

	tmp = ioread32be(&regs->command_config);

	if (apply_rx)
		tmp |= CMD_CFG_RX_EN;

	if (apply_tx)
		tmp |= CMD_CFG_TX_EN;

	iowrite32be(tmp, &regs->command_config);
}

void fman_memac_disable(struct memac_regs __iomem *regs, bool apply_rx,
			bool apply_tx)
{
	u32 tmp;

	tmp = ioread32be(&regs->command_config);

	if (apply_rx)
		tmp &= ~CMD_CFG_RX_EN;

	if (apply_tx)
		tmp &= ~CMD_CFG_TX_EN;

	iowrite32be(tmp, &regs->command_config);
}

int fman_memac_reset(struct memac_regs __iomem *regs)
{
	u32 tmp;
	int count;

	tmp = ioread32be(&regs->command_config);

	tmp |= CMD_CFG_SW_RESET;

	iowrite32be(tmp, &regs->command_config);

	count = 100;
	do {
		udelay(1);
	} while ((ioread32be(&regs->command_config) & CMD_CFG_SW_RESET) &&
		 --count);

	if (count == 0)
		return -EBUSY;

	return 0;
}

int fman_memac_init(struct memac_regs __iomem *regs,
		    struct memac_cfg *cfg,
		    enum enet_interface enet_interface,
		    enum enet_speed enet_speed,
		    bool slow_10g_if,
		    u32 exceptions)
{
	u32 tmp;

	/* Config */
	tmp = 0;
	if (cfg->wan_mode_enable)
		tmp |= CMD_CFG_WAN_MODE;
	if (cfg->promiscuous_mode_enable)
		tmp |= CMD_CFG_PROMIS_EN;
	if (cfg->pause_forward_enable)
		tmp |= CMD_CFG_PAUSE_FWD;
	if (cfg->pause_ignore)
		tmp |= CMD_CFG_PAUSE_IGNORE;
	if (cfg->tx_addr_ins_enable)
		tmp |= CMD_CFG_TX_ADDR_INS;
	if (cfg->loopback_enable)
		tmp |= CMD_CFG_LOOPBACK_EN;
	if (cfg->cmd_frame_enable)
		tmp |= CMD_CFG_CNT_FRM_EN;
	if (cfg->send_idle_enable)
		tmp |= CMD_CFG_SEND_IDLE;
	if (cfg->no_length_check_enable)
		tmp |= CMD_CFG_NO_LEN_CHK;
	if (cfg->rx_sfd_any)
		tmp |= CMD_CFG_SFD_ANY;
	if (cfg->pad_enable)
		tmp |= CMD_CFG_TX_PAD_EN;
	if (cfg->wake_on_lan)
		tmp |= CMD_CFG_MG;

	tmp |= CMD_CFG_CRC_FWD;

	iowrite32be(tmp, &regs->command_config);

	/* Max Frame Length */
	iowrite32be((u32)cfg->max_frame_length, &regs->maxfrm);

	/* Pause Time */
	iowrite32be((u32)cfg->pause_quanta, &regs->pause_quanta[0]);
	iowrite32be((u32)0, &regs->pause_thresh[0]);

	/* IF_MODE */
	tmp = 0;
	switch (enet_interface) {
	case E_ENET_IF_XGMII:
	case E_ENET_IF_XFI:
		tmp |= IF_MODE_XGMII;
		break;
	default:
		tmp |= IF_MODE_GMII;
		if (enet_interface == E_ENET_IF_RGMII && !cfg->loopback_enable)
			tmp |= IF_MODE_RGMII | IF_MODE_RGMII_AUTO;
	}
	iowrite32be(tmp, &regs->if_mode);

	/* TX_FIFO_SECTIONS */
	tmp = 0;
	if (enet_interface == E_ENET_IF_XGMII ||
	    enet_interface == E_ENET_IF_XFI) {
		if (slow_10g_if) {
			tmp |= (TX_FIFO_SECTIONS_TX_AVAIL_SLOW_10G |
				TX_FIFO_SECTIONS_TX_EMPTY_DEFAULT_10G);
		} else {
			tmp |= (TX_FIFO_SECTIONS_TX_AVAIL_10G |
				TX_FIFO_SECTIONS_TX_EMPTY_DEFAULT_10G);
		}
	} else {
		tmp |= (TX_FIFO_SECTIONS_TX_AVAIL_1G |
			TX_FIFO_SECTIONS_TX_EMPTY_DEFAULT_1G);
	}
	iowrite32be(tmp, &regs->tx_fifo_sections);

	/* clear all pending events and set-up interrupts */
	fman_memac_ack_event(regs, 0xffffffff);
	fman_memac_set_exception(regs, exceptions, true);

	return 0;
}

void fman_memac_set_exception(struct memac_regs __iomem *regs, u32 val,
			      bool enable)
{
	u32 tmp;

	tmp = ioread32be(&regs->imask);
	if (enable)
		tmp |= val;
	else
		tmp &= ~val;

	iowrite32be(tmp, &regs->imask);
}

void fman_memac_set_hash_table(struct memac_regs __iomem *regs, u32 val)
{
	iowrite32be(val, &regs->hashtable_ctrl);
}

u16 fman_memac_get_max_frame_len(struct memac_regs __iomem *regs)
{
	u32 tmp;

	tmp = ioread32be(&regs->maxfrm);

	return (u16)tmp;
}

void fman_memac_set_tx_pause_frames(struct memac_regs __iomem *regs,
				    u8 priority, u16 pause_time,
				    u16 thresh_time)
{
	u32 tmp;

	tmp = ioread32be(&regs->tx_fifo_sections);

	GET_TX_EMPTY_DEFAULT_VALUE(tmp);
	iowrite32be(tmp, &regs->tx_fifo_sections);

	tmp = ioread32be(&regs->command_config);
	tmp &= ~CMD_CFG_PFC_MODE;
	priority = 0;

	iowrite32be(tmp, &regs->command_config);

	tmp = ioread32be(&regs->pause_quanta[priority / 2]);
	if (priority % 2)
		tmp &= CLXY_PAUSE_QUANTA_CLX_PQNT;
	else
		tmp &= CLXY_PAUSE_QUANTA_CLY_PQNT;
	tmp |= ((u32)pause_time << (16 * (priority % 2)));
	iowrite32be(tmp, &regs->pause_quanta[priority / 2]);

	tmp = ioread32be(&regs->pause_thresh[priority / 2]);
	if (priority % 2)
		tmp &= CLXY_PAUSE_THRESH_CLX_QTH;
	else
		tmp &= CLXY_PAUSE_THRESH_CLY_QTH;
	tmp |= ((u32)thresh_time << (16 * (priority % 2)));
	iowrite32be(tmp, &regs->pause_thresh[priority / 2]);
}

void fman_memac_set_rx_ignore_pause_frames(struct memac_regs __iomem *regs,
					   bool enable)
{
	u32 tmp;

	tmp = ioread32be(&regs->command_config);
	if (enable)
		tmp |= CMD_CFG_PAUSE_IGNORE;
	else
		tmp &= ~CMD_CFG_PAUSE_IGNORE;

	iowrite32be(tmp, &regs->command_config);
}

void fman_memac_adjust_link(struct memac_regs __iomem *regs,
			    enum enet_interface iface_mode,
			    enum enet_speed speed, bool full_dx)
{
	u32 tmp;

	tmp = ioread32be(&regs->if_mode);

	if (full_dx)
		tmp &= ~IF_MODE_HD;
	else
		tmp |= IF_MODE_HD;

	if (iface_mode == E_ENET_IF_RGMII) {
		/* Configure RGMII in manual mode */
		tmp &= ~IF_MODE_RGMII_AUTO;
		tmp &= ~IF_MODE_RGMII_SP_MASK;

		if (full_dx)
			tmp |= IF_MODE_RGMII_FD;
		else
			tmp &= ~IF_MODE_RGMII_FD;

		switch (speed) {
		case E_ENET_SPEED_1000:
			tmp |= IF_MODE_RGMII_1000;
			break;
		case E_ENET_SPEED_100:
			tmp |= IF_MODE_RGMII_100;
			break;
		case E_ENET_SPEED_10:
			tmp |= IF_MODE_RGMII_10;
			break;
		default:
			break;
		}
	}

	iowrite32be(tmp, &regs->if_mode);
}

void fman_memac_defconfig(struct memac_cfg *cfg)
{
	cfg->reset_on_init = false;
	cfg->wan_mode_enable = false;
	cfg->promiscuous_mode_enable = false;
	cfg->pause_forward_enable = false;
	cfg->pause_ignore = false;
	cfg->tx_addr_ins_enable = false;
	cfg->loopback_enable = false;
	cfg->cmd_frame_enable = false;
	cfg->rx_error_discard = false;
	cfg->send_idle_enable = false;
	cfg->no_length_check_enable = true;
	cfg->lgth_check_nostdr = false;
	cfg->time_stamp_enable = false;
	cfg->tx_ipg_length = DEFAULT_TX_IPG_LENGTH;
	cfg->max_frame_length = DEFAULT_FRAME_LENGTH;
	cfg->pause_quanta = DEFAULT_PAUSE_QUANTA;
	cfg->pad_enable = true;
	cfg->phy_tx_ena_on = false;
	cfg->rx_sfd_any = false;
	cfg->rx_pbl_fwd = false;
	cfg->tx_pbl_fwd = false;
	cfg->debug_mode = false;
	cfg->wake_on_lan = false;
}
