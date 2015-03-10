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

#include "fsl_fman_tgec.h"

void fman_tgec_set_mac_address(struct tgec_regs __iomem *regs, u8 *adr)
{
	u32 tmp0, tmp1;

	tmp0 = (u32)(adr[0] | adr[1] << 8 | adr[2] << 16 | adr[3] << 24);
	tmp1 = (u32)(adr[4] | adr[5] << 8);
	iowrite32be(tmp0, &regs->mac_addr_0);
	iowrite32be(tmp1, &regs->mac_addr_1);
}

void fman_tgec_enable(struct tgec_regs __iomem *regs, bool apply_rx,
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

void fman_tgec_disable(struct tgec_regs __iomem *regs, bool apply_rx,
		       bool apply_tx)
{
	u32 tmp_reg_32;

	tmp_reg_32 = ioread32be(&regs->command_config);
	if (apply_rx)
		tmp_reg_32 &= ~CMD_CFG_RX_EN;
	if (apply_tx)
		tmp_reg_32 &= ~CMD_CFG_TX_EN;
	iowrite32be(tmp_reg_32, &regs->command_config);
}

void fman_tgec_set_promiscuous(struct tgec_regs __iomem *regs, bool val)
{
	u32 tmp;

	tmp = ioread32be(&regs->command_config);
	if (val)
		tmp |= CMD_CFG_PROMIS_EN;
	else
		tmp &= ~CMD_CFG_PROMIS_EN;
	iowrite32be(tmp, &regs->command_config);
}

void fman_tgec_set_hash_table(struct tgec_regs __iomem *regs, u32 value)
{
	iowrite32be(value, &regs->hashtable_ctrl);
}

void fman_tgec_set_tx_pause_frames(struct tgec_regs __iomem *regs,
				   u16 pause_time)
{
	iowrite32be((u32)pause_time, &regs->pause_quant);
}

void fman_tgec_set_rx_ignore_pause_frames(struct tgec_regs __iomem *regs,
					  bool en)
{
	u32 tmp;

	tmp = ioread32be(&regs->command_config);
	if (en)
		tmp |= CMD_CFG_PAUSE_IGNORE;
	else
		tmp &= ~CMD_CFG_PAUSE_IGNORE;
	iowrite32be(tmp, &regs->command_config);
}

u32 fman_tgec_get_event(struct tgec_regs __iomem *regs, u32 ev_mask)
{
	return ioread32be(&regs->ievent) & ev_mask;
}

void fman_tgec_ack_event(struct tgec_regs __iomem *regs, u32 ev_mask)
{
	iowrite32be(ev_mask, &regs->ievent);
}

u32 fman_tgec_get_interrupt_mask(struct tgec_regs __iomem *regs)
{
	return ioread32be(&regs->imask);
}

u32 fman_tgec_get_revision(struct tgec_regs __iomem *regs)
{
	return ioread32be(&regs->tgec_id);
}

void fman_tgec_enable_interrupt(struct tgec_regs __iomem *regs, u32 ev_mask)
{
	iowrite32be(ioread32be(&regs->imask) | ev_mask, &regs->imask);
}

void fman_tgec_disable_interrupt(struct tgec_regs __iomem *regs, u32 ev_mask)
{
	iowrite32be(ioread32be(&regs->imask) & ~ev_mask, &regs->imask);
}

void fman_tgec_defconfig(struct tgec_cfg *cfg)
{
	cfg->wan_mode_enable = DEFAULT_WAN_MODE_ENABLE;
	cfg->promiscuous_mode_enable = DEFAULT_PROMISCUOUS_MODE_ENABLE;
	cfg->pause_forward_enable = DEFAULT_PAUSE_FORWARD_ENABLE;
	cfg->pause_ignore = DEFAULT_PAUSE_IGNORE;
	cfg->tx_addr_ins_enable = DEFAULT_TX_ADDR_INS_ENABLE;
	cfg->loopback_enable = DEFAULT_LOOPBACK_ENABLE;
	cfg->cmd_frame_enable = DEFAULT_CMD_FRAME_ENABLE;
	cfg->rx_error_discard = DEFAULT_RX_ERROR_DISCARD;
	cfg->send_idle_enable = DEFAULT_SEND_IDLE_ENABLE;
	cfg->no_length_check_enable = DEFAULT_NO_LENGTH_CHECK_ENABLE;
	cfg->lgth_check_nostdr = DEFAULT_LGTH_CHECK_NOSTDR;
	cfg->time_stamp_enable = DEFAULT_TIME_STAMP_ENABLE;
	cfg->tx_ipg_length = DEFAULT_TX_IPG_LENGTH;
	cfg->max_frame_length = DEFAULT_MAX_FRAME_LENGTH;
	cfg->pause_quant = DEFAULT_PAUSE_QUANT;
}

int fman_tgec_init(struct tgec_regs __iomem *regs, struct tgec_cfg *cfg,
		   u32 exception_mask)
{
	u32 tmp;

	/* Config */
	tmp = CMF_CFG_CRC_FWD;
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
		tmp |= CMD_CFG_CMD_FRM_EN;
	if (cfg->rx_error_discard)
		tmp |= CMD_CFG_RX_ER_DISC;
	if (cfg->send_idle_enable)
		tmp |= CMD_CFG_SEND_IDLE;
	if (cfg->no_length_check_enable)
		tmp |= CMD_CFG_NO_LEN_CHK;
	if (cfg->time_stamp_enable)
		tmp |= CMD_CFG_EN_TIMESTAMP;
	iowrite32be(tmp, &regs->command_config);

	/* Max Frame Length */
	iowrite32be((u32)cfg->max_frame_length, &regs->maxfrm);
	/* Pause Time */
	iowrite32be(cfg->pause_quant, &regs->pause_quant);

	/* clear all pending events and set-up interrupts */
	fman_tgec_ack_event(regs, 0xffffffff);
	fman_tgec_enable_interrupt(regs, exception_mask);

	return 0;
}

void fman_tgec_set_erratum_tx_fifo_corruption_10gmac_a007(struct tgec_regs
							  __iomem *regs)
{
	u32 tmp;

	/* restore the default tx ipg Length */
	tmp = (ioread32be(&regs->tx_ipg_len) & ~TGEC_TX_IPG_LENGTH_MASK) | 12;

	iowrite32be(tmp, &regs->tx_ipg_len);
}
