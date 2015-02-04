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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "crc_mac_addr_ext.h"

#include "fm_common.h"
#include "fsl_fman_tgec.h"
#include "fm_tgec.h"

#include <linux/string.h>
#include <linux/slab.h>
#include <linux/bitrev.h>

static int check_init_parameters(struct tgec_t *tgec)
{
	if (ENET_SPEED_FROM_MODE(tgec->enet_mode) < ENET_SPEED_10000) {
		pr_err("10G MAC driver only support 10G speed\n");
		return -EINVAL;
	}
	if (tgec->addr == 0) {
		pr_err("Ethernet 10G MAC Must have valid MAC Address\n");
		return -EINVAL;
	}
	if (!tgec->exception_cb) {
		pr_err("uninitialized exception_cb\n");
		return -EINVAL;
	}
	if (!tgec->event_cb) {
		pr_err("uninitialized event_cb\n");
		return -EINVAL;
	}

	/* FM_LEN_CHECK_ERRATA_FMAN_SW002 Errata workaround */
	if (!tgec->tgec_drv_param->no_length_check_enable) {
		pr_warn("Length Check!\n");
		return -EINVAL;
	}

	return 0;
}

static int get_exception_flag(enum fm_mac_exceptions exception)
{
	u32 bit_mask;

	switch (exception) {
	case FM_MAC_EX_10G_MDIO_SCAN_EVENT:
		bit_mask = TGEC_IMASK_MDIO_SCAN_EVENT;
		break;
	case FM_MAC_EX_10G_MDIO_CMD_CMPL:
		bit_mask = TGEC_IMASK_MDIO_CMD_CMPL;
		break;
	case FM_MAC_EX_10G_REM_FAULT:
		bit_mask = TGEC_IMASK_REM_FAULT;
		break;
	case FM_MAC_EX_10G_LOC_FAULT:
		bit_mask = TGEC_IMASK_LOC_FAULT;
		break;
	case FM_MAC_EX_10G_TX_ECC_ER:
		bit_mask = TGEC_IMASK_TX_ECC_ER;
		break;
	case FM_MAC_EX_10G_TX_FIFO_UNFL:
		bit_mask = TGEC_IMASK_TX_FIFO_UNFL;
		break;
	case FM_MAC_EX_10G_TX_FIFO_OVFL:
		bit_mask = TGEC_IMASK_TX_FIFO_OVFL;
		break;
	case FM_MAC_EX_10G_TX_ER:
		bit_mask = TGEC_IMASK_TX_ER;
		break;
	case FM_MAC_EX_10G_RX_FIFO_OVFL:
		bit_mask = TGEC_IMASK_RX_FIFO_OVFL;
		break;
	case FM_MAC_EX_10G_RX_ECC_ER:
		bit_mask = TGEC_IMASK_RX_ECC_ER;
		break;
	case FM_MAC_EX_10G_RX_JAB_FRM:
		bit_mask = TGEC_IMASK_RX_JAB_FRM;
		break;
	case FM_MAC_EX_10G_RX_OVRSZ_FRM:
		bit_mask = TGEC_IMASK_RX_OVRSZ_FRM;
		break;
	case FM_MAC_EX_10G_RX_RUNT_FRM:
		bit_mask = TGEC_IMASK_RX_RUNT_FRM;
		break;
	case FM_MAC_EX_10G_RX_FRAG_FRM:
		bit_mask = TGEC_IMASK_RX_FRAG_FRM;
		break;
	case FM_MAC_EX_10G_RX_LEN_ER:
		bit_mask = TGEC_IMASK_RX_LEN_ER;
		break;
	case FM_MAC_EX_10G_RX_CRC_ER:
		bit_mask = TGEC_IMASK_RX_CRC_ER;
		break;
	case FM_MAC_EX_10G_RX_ALIGN_ER:
		bit_mask = TGEC_IMASK_RX_ALIGN_ER;
		break;
	default:
		bit_mask = 0;
		break;
	}

	return bit_mask;
}

static u32 get_mac_addr_hash_code(u64 eth_addr)
{
	u32 crc;

	/* CRC calculation */
	GET_MAC_ADDR_CRC(eth_addr, crc);

	crc = bitrev32(crc);

	return crc;
}

static void tgec_err_exception(void *handle)
{
	struct tgec_t *tgec = (struct tgec_t *)handle;
	u32 event;
	struct tgec_regs __iomem *regs = tgec->regs;

	/* do not handle MDIO events */
	event = fman_tgec_get_event(regs,
				    ~(TGEC_IMASK_MDIO_SCAN_EVENT |
				    TGEC_IMASK_MDIO_CMD_CMPL));
	event &= fman_tgec_get_interrupt_mask(regs);

	fman_tgec_ack_event(regs, event);

	if (event & TGEC_IMASK_REM_FAULT)
		tgec->exception_cb(tgec->dev_id, FM_MAC_EX_10G_REM_FAULT);
	if (event & TGEC_IMASK_LOC_FAULT)
		tgec->exception_cb(tgec->dev_id, FM_MAC_EX_10G_LOC_FAULT);
	if (event & TGEC_IMASK_TX_ECC_ER)
		tgec->exception_cb(tgec->dev_id, FM_MAC_EX_10G_TX_ECC_ER);
	if (event & TGEC_IMASK_TX_FIFO_UNFL)
		tgec->exception_cb(tgec->dev_id, FM_MAC_EX_10G_TX_FIFO_UNFL);
	if (event & TGEC_IMASK_TX_FIFO_OVFL)
		tgec->exception_cb(tgec->dev_id, FM_MAC_EX_10G_TX_FIFO_OVFL);
	if (event & TGEC_IMASK_TX_ER)
		tgec->exception_cb(tgec->dev_id, FM_MAC_EX_10G_TX_ER);
	if (event & TGEC_IMASK_RX_FIFO_OVFL)
		tgec->exception_cb(tgec->dev_id, FM_MAC_EX_10G_RX_FIFO_OVFL);
	if (event & TGEC_IMASK_RX_ECC_ER)
		tgec->exception_cb(tgec->dev_id, FM_MAC_EX_10G_RX_ECC_ER);
	if (event & TGEC_IMASK_RX_JAB_FRM)
		tgec->exception_cb(tgec->dev_id, FM_MAC_EX_10G_RX_JAB_FRM);
	if (event & TGEC_IMASK_RX_OVRSZ_FRM)
		tgec->exception_cb(tgec->dev_id, FM_MAC_EX_10G_RX_OVRSZ_FRM);
	if (event & TGEC_IMASK_RX_RUNT_FRM)
		tgec->exception_cb(tgec->dev_id, FM_MAC_EX_10G_RX_RUNT_FRM);
	if (event & TGEC_IMASK_RX_FRAG_FRM)
		tgec->exception_cb(tgec->dev_id, FM_MAC_EX_10G_RX_FRAG_FRM);
	if (event & TGEC_IMASK_RX_LEN_ER)
		tgec->exception_cb(tgec->dev_id, FM_MAC_EX_10G_RX_LEN_ER);
	if (event & TGEC_IMASK_RX_CRC_ER)
		tgec->exception_cb(tgec->dev_id, FM_MAC_EX_10G_RX_CRC_ER);
	if (event & TGEC_IMASK_RX_ALIGN_ER)
		tgec->exception_cb(tgec->dev_id, FM_MAC_EX_10G_RX_ALIGN_ER);
}

static void free_init_resources(struct tgec_t *tgec)
{
	fm_unregister_intr(tgec->fm, FM_MOD_MAC, tgec->mac_id,
			   FM_INTR_TYPE_ERR);

	/* release the driver's group hash table */
	free_hash_table(tgec->multicast_addr_hash);
	tgec->multicast_addr_hash = NULL;

	/* release the driver's individual hash table */
	free_hash_table(tgec->unicast_addr_hash);
	tgec->unicast_addr_hash = NULL;
}

static bool is_init_done(struct tgec_cfg *tgec_drv_parameters)
{
	/* Checks if tGEC driver parameters were initialized */
	if (!tgec_drv_parameters)
		return true;

	return false;
}

int tgec_enable(struct fm_mac_dev *fm_mac_dev, enum comm_mode mode)
{
	struct tgec_t *tgec = (struct tgec_t *)fm_mac_dev;

	if (!is_init_done(tgec->tgec_drv_param))
		return -EINVAL;

	fman_tgec_enable(tgec->regs,
			 (mode & COMM_MODE_RX), (mode & COMM_MODE_TX));

	return 0;
}

int tgec_disable(struct fm_mac_dev *fm_mac_dev, enum comm_mode mode)
{
	struct tgec_t *tgec = (struct tgec_t *)fm_mac_dev;

	if (!is_init_done(tgec->tgec_drv_param))
		return -EINVAL;

	fman_tgec_disable(tgec->regs,
			  (mode & COMM_MODE_RX), (mode & COMM_MODE_TX));

	return 0;
}

int tgec_set_promiscuous(struct fm_mac_dev *fm_mac_dev, bool new_val)
{
	struct tgec_t *tgec = (struct tgec_t *)fm_mac_dev;

	if (!is_init_done(tgec->tgec_drv_param))
		return -EINVAL;

	fman_tgec_set_promiscuous(tgec->regs, new_val);

	return 0;
}

int tgec_cfg_max_frame_len(struct fm_mac_dev *fm_mac_dev, u16 new_val)
{
	struct tgec_t *tgec = (struct tgec_t *)fm_mac_dev;

	if (is_init_done(tgec->tgec_drv_param))
		return -EINVAL;

	tgec->tgec_drv_param->max_frame_length = new_val;

	return 0;
}

int tgec_set_tx_pause_frames(struct fm_mac_dev *fm_mac_dev,
			     u8 __maybe_unused priority,
			     u16 pause_time,
			     u16 __maybe_unused thresh_time)
{
	struct tgec_t *tgec = (struct tgec_t *)fm_mac_dev;

	if (!is_init_done(tgec->tgec_drv_param))
		return -EINVAL;

	fman_tgec_set_tx_pause_frames(tgec->regs, pause_time);

	return 0;
}

int tgec_accept_rx_pause_frames(struct fm_mac_dev *fm_mac_dev, bool en)
{
	struct tgec_t *tgec = (struct tgec_t *)fm_mac_dev;

	if (!is_init_done(tgec->tgec_drv_param))
		return -EINVAL;

	fman_tgec_set_rx_ignore_pause_frames(tgec->regs, !en);

	return 0;
}

int tgec_modify_mac_address(struct fm_mac_dev *fm_mac_dev,
			    enet_addr_t *p_enet_addr)
{
	struct tgec_t *tgec = (struct tgec_t *)fm_mac_dev;

	if (!is_init_done(tgec->tgec_drv_param))
		return -EINVAL;

	tgec->addr = ENET_ADDR_TO_UINT64(*p_enet_addr);
	fman_tgec_set_mac_address(tgec->regs, (u8 *)(*p_enet_addr));

	return 0;
}

int tgec_add_hash_mac_address(struct fm_mac_dev *fm_mac_dev,
			      enet_addr_t *eth_addr)
{
	struct tgec_t *tgec = (struct tgec_t *)fm_mac_dev;
	struct eth_hash_entry_t *hash_entry;
	u32 crc;
	u32 hash;
	u64 addr;

	if (!is_init_done(tgec->tgec_drv_param))
		return -EINVAL;

	addr = ENET_ADDR_TO_UINT64(*eth_addr);

	if (!(addr & GROUP_ADDRESS)) {
		/* Unicast addresses not supported in hash */
		pr_err("Unicast Address\n");
		return -EINVAL;
	}
	/* CRC calculation */
	crc = get_mac_addr_hash_code(addr);

	/* Take 9 MSB bits */
	hash = (crc >> TGEC_HASH_MCAST_SHIFT) & TGEC_HASH_ADR_MSK;

	/* Create element to be added to the driver hash table */
	hash_entry = kmalloc(sizeof(*hash_entry), GFP_KERNEL);
	if (!hash_entry)
		return -ENOMEM;
	hash_entry->addr = addr;
	INIT_LIST_HEAD(&hash_entry->node);

	list_add_tail(&hash_entry->node,
		      &tgec->multicast_addr_hash->lsts[hash]);
	fman_tgec_set_hash_table(tgec->regs, (hash | TGEC_HASH_MCAST_EN));

	return 0;
}

int tgec_del_hash_mac_address(struct fm_mac_dev *fm_mac_dev,
			      enet_addr_t *eth_addr)
{
	struct tgec_t *tgec = (struct tgec_t *)fm_mac_dev;
	struct eth_hash_entry_t *hash_entry = NULL;
	struct list_head *p_pos;
	u32 crc;
	u32 hash;
	u64 addr;

	if (!is_init_done(tgec->tgec_drv_param))
		return -EINVAL;

	addr = ((*(u64 *)eth_addr) >> 16);

	/* CRC calculation */
	crc = get_mac_addr_hash_code(addr);
	/* Take 9 MSB bits */
	hash = (crc >> TGEC_HASH_MCAST_SHIFT) & TGEC_HASH_ADR_MSK;

	list_for_each(p_pos, &tgec->multicast_addr_hash->lsts[hash]) {
		hash_entry = ETH_HASH_ENTRY_OBJ(p_pos);
		if (hash_entry->addr == addr) {
			list_del_init(&hash_entry->node);
			kfree(hash_entry);
			break;
		}
	}
	if (list_empty(&tgec->multicast_addr_hash->lsts[hash]))
		fman_tgec_set_hash_table(tgec->regs,
					 (hash & ~TGEC_HASH_MCAST_EN));

	return 0;
}

int tgec_get_version(struct fm_mac_dev *fm_mac_dev, u32 *mac_version)
{
	struct tgec_t *tgec = (struct tgec_t *)fm_mac_dev;

	if (!is_init_done(tgec->tgec_drv_param))
		return -EINVAL;

	*mac_version = fman_tgec_get_revision(tgec->regs);

	return 0;
}

int tgec_set_exception(struct fm_mac_dev *fm_mac_dev,
		       enum fm_mac_exceptions exception,
		       bool enable)
{
	struct tgec_t *tgec = (struct tgec_t *)fm_mac_dev;
	u32 bit_mask = 0;

	if (!is_init_done(tgec->tgec_drv_param))
		return -EINVAL;

	bit_mask = get_exception_flag(exception);
	if (bit_mask) {
		if (enable)
			tgec->exceptions |= bit_mask;
		else
			tgec->exceptions &= ~bit_mask;
	} else {
		pr_err("Undefined exception\n");
		return -EINVAL;
	}
	if (enable)
		fman_tgec_enable_interrupt(tgec->regs, bit_mask);
	else
		fman_tgec_disable_interrupt(tgec->regs, bit_mask);

	return 0;
}

int tgec_init(struct fm_mac_dev *fm_mac_dev)
{
	struct tgec_t *tgec = (struct tgec_t *)fm_mac_dev;
	struct tgec_cfg *tgec_drv_param;
	enet_addr_t eth_addr;
	int err;

	if (is_init_done(tgec->tgec_drv_param))
		return -EINVAL;

	if (DEFAULT_RESET_ON_INIT &&
	    (fm_reset_mac(tgec->fm, tgec->mac_id) != 0)) {
		pr_err("Can't reset MAC!\n");
		return -EINVAL;
	}

	fm_get_revision(tgec->fm, &tgec->fm_rev_info);
	err = check_init_parameters(tgec);
	if (err)
		return err;

	tgec_drv_param = tgec->tgec_drv_param;

	MAKE_ENET_ADDR_FROM_UINT64(tgec->addr, eth_addr);
	fman_tgec_set_mac_address(tgec->regs, (u8 *)eth_addr);

	/* interrupts */
	/* FM_10G_REM_N_LCL_FLT_EX_10GMAC_ERRATA_SW005 Errata workaround */
	if (tgec->fm_rev_info.major_rev <= 2)
		tgec->exceptions &= ~(TGEC_IMASK_REM_FAULT |
				      TGEC_IMASK_LOC_FAULT);

	err = fman_tgec_init(tgec->regs, tgec_drv_param, tgec->exceptions);
	if (err) {
		free_init_resources(tgec);
		pr_err("TGEC version doesn't support this i/f mode\n");
		return err;
	}

	/* Max Frame Length */
	err = fm_set_mac_max_frame(tgec->fm, FM_MAC_10G, tgec->mac_id,
				   tgec_drv_param->max_frame_length);
	if (err) {
		pr_err("Setting max frame length FAILED\n");
		free_init_resources(tgec);
		return -EINVAL;
	}

	/* FM_TX_FIFO_CORRUPTION_ERRATA_10GMAC_A007 Errata workaround */
	if (tgec->fm_rev_info.major_rev == 2)
		fman_tgec_set_erratum_tx_fifo_corruption_10gmac_a007(tgec->
								     regs);

	tgec->multicast_addr_hash = alloc_hash_table(TGEC_HASH_TABLE_SIZE);
	if (!tgec->multicast_addr_hash) {
		free_init_resources(tgec);
		pr_err("allocation hash table is FAILED\n");
		return -ENOMEM;
	}

	tgec->unicast_addr_hash = alloc_hash_table(TGEC_HASH_TABLE_SIZE);
	if (!tgec->unicast_addr_hash) {
		free_init_resources(tgec);
		pr_err("allocation hash table is FAILED\n");
		return -ENOMEM;
	}

	fm_register_intr(tgec->fm, FM_MOD_MAC, tgec->mac_id,
			 FM_INTR_TYPE_ERR, tgec_err_exception, tgec);

	kfree(tgec_drv_param);
	tgec->tgec_drv_param = NULL;

	return 0;
}

int tgec_free(struct fm_mac_dev *fm_mac_dev)
{
	struct tgec_t *tgec = (struct tgec_t *)fm_mac_dev;

	free_init_resources(tgec);

	if (tgec->tgec_drv_param)
		tgec->tgec_drv_param = NULL;

	kfree(tgec->tgec_drv_param);
	kfree(tgec);

	return 0;
}

void *tgec_config(struct fm_mac_params_t *fm_mac_param)
{
	struct tgec_t *tgec;
	struct tgec_cfg *tgec_drv_param;
	void __iomem *base_addr;

	base_addr = fm_mac_param->base_addr;
	/* allocate memory for the UCC GETH data structure. */
	tgec = kzalloc(sizeof(*tgec), GFP_KERNEL);
	if (!tgec)
		return NULL;

	/* allocate memory for the 10G MAC driver parameters data structure. */
	tgec_drv_param = kzalloc(sizeof(*tgec_drv_param), GFP_KERNEL);
	if (!tgec_drv_param) {
		tgec_free((struct fm_mac_dev *)tgec);
		return NULL;
	}

	/* Plant parameter structure pointer */
	tgec->tgec_drv_param = tgec_drv_param;

	fman_tgec_defconfig(tgec_drv_param);

	tgec->regs = (struct tgec_regs __iomem *)(base_addr);
	tgec->addr = ENET_ADDR_TO_UINT64(fm_mac_param->addr);
	tgec->enet_mode = fm_mac_param->enet_mode;
	tgec->mac_id = fm_mac_param->mac_id;
	tgec->exceptions = TGEC_DEFAULT_EXCEPTIONS;
	tgec->exception_cb = fm_mac_param->exception_cb;
	tgec->event_cb = fm_mac_param->event_cb;
	tgec->dev_id = fm_mac_param->dev_id;
	tgec->fm = fm_mac_param->fm;

	/* Save FMan revision */
	fm_get_revision(tgec->fm, &tgec->fm_rev_info);

	return tgec;
}
