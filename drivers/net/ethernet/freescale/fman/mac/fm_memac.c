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

#include "fm_common.h"
#include "fm_memac.h"

#include <linux/string.h>
#include <linux/slab.h>

static u32 get_mac_addr_hash_code(u64 eth_addr)
{
	u64 mask1, mask2;
	u32 xor_val = 0;
	u8 i, j;

	for (i = 0; i < 6; i++) {
		mask1 = eth_addr & (u64)0x01;
		eth_addr >>= 1;

		for (j = 0; j < 7; j++) {
			mask2 = eth_addr & (u64)0x01;
			mask1 ^= mask2;
			eth_addr >>= 1;
		}

		xor_val |= (mask1 << (5 - i));
	}

	return xor_val;
}

static int memac_mii_write_phy_reg(struct memac_t *memac, u8 phy_addr,
				   u8 reg, u16 data)
{
	return fman_memac_mii_write_phy_reg(memac->mii_regs,
					    phy_addr, reg, data,
					    (enum enet_speed)
					    ENET_SPEED_FROM_MODE(memac->
								 enet_mode));
}

static void setup_sgmii_internal_phy(struct memac_t *memac, u8 phy_addr,
				     bool fixed_link)
{
	u16 tmp_reg16;
	enum e_enet_mode enet_mode;

	/* In case the higher MACs are used (i.e. the MACs that should
	 * support 10G), speed=10000 is provided for SGMII ports.
	 * Temporary modify enet mode to 1G one, so MII functions can
	 * work correctly.
	 */
	enet_mode = memac->enet_mode;
	memac->enet_mode =
	    MAKE_ENET_MODE(ENET_INTERFACE_FROM_MODE(memac->enet_mode),
			   ENET_SPEED_1000);

	/* SGMII mode */
	tmp_reg16 = PHY_SGMII_IF_MODE_SGMII;
	if (!fixed_link)
		/* AN enable */
		tmp_reg16 |= PHY_SGMII_IF_MODE_AN;
	else
		/* Fixed link 1Gb FD */
		tmp_reg16 |= PHY_SGMII_IF_MODE_SPEED_GB |
			     PHY_SGMII_IF_MODE_DUPLEX_FULL;
	memac_mii_write_phy_reg(memac, phy_addr, 0x14, tmp_reg16);

	/* Device ability according to SGMII specification */
	tmp_reg16 = PHY_SGMII_DEV_ABILITY_SGMII;
	memac_mii_write_phy_reg(memac, phy_addr, 0x4, tmp_reg16);

	/* Adjust link timer for SGMII  -
	 * According to Cisco SGMII specification the timer should be 1.6 ms.
	 * The link_timer register is configured in units of the clock.
	 * - When running as 1G SGMII, Serdes clock is 125 MHz, so
	 * unit = 1 / (125*10^6 Hz) = 8 ns.
	 * 1.6 ms in units of 8 ns = 1.6ms / 8ns = 2*10^5 = 0x30d40
	 * - When running as 2.5G SGMII, Serdes clock is 312.5 MHz, so
	 * unit = 1 / (312.5*10^6 Hz) = 3.2 ns.
	 * 1.6 ms in units of 3.2 ns = 1.6ms / 3.2ns = 5*10^5 = 0x7a120.
	 * Since link_timer value of 1G SGMII will be too short for 2.5 SGMII,
	 * we always set up here a value of 2.5 SGMII.
	 */
	memac_mii_write_phy_reg(memac, phy_addr, 0x13, 0x0007);
	memac_mii_write_phy_reg(memac, phy_addr, 0x12, 0xa120);

	if (!fixed_link)
		/* Restart AN */
		tmp_reg16 = PHY_SGMII_CR_DEF_VAL | PHY_SGMII_CR_RESET_AN;
	else
		tmp_reg16 = PHY_SGMII_CR_DEF_VAL & ~PHY_SGMII_CR_AN_ENABLE;
	memac_mii_write_phy_reg(memac, phy_addr, 0x0, tmp_reg16);

	/* Restore original enet mode */
	memac->enet_mode = enet_mode;
}

static void setup_sgmii_internal_phy_base_x(struct memac_t *memac,
					    u8 phy_addr)
{
	u16 tmp_reg16;
	enum e_enet_mode enet_mode;

	/* In case the higher MACs are used (i.e. the MACs that
	 * should support 10G), speed=10000 is provided for SGMII ports.
	 * Temporary modify enet mode to 1G one, so MII functions can
	 * work correctly.
	 */
	enet_mode = memac->enet_mode;
	memac->enet_mode =
	    MAKE_ENET_MODE(ENET_INTERFACE_FROM_MODE(memac->enet_mode),
			   ENET_SPEED_1000);

	/* 1000BaseX mode */
	tmp_reg16 = PHY_SGMII_IF_MODE_1000X;
	memac_mii_write_phy_reg(memac, phy_addr, 0x14, tmp_reg16);

	/* AN Device capability  */
	tmp_reg16 = PHY_SGMII_DEV_ABILITY_1000X;
	memac_mii_write_phy_reg(memac, phy_addr, 0x4, tmp_reg16);

	/* Adjust link timer for SGMII  -
	 * For Serdes 1000BaseX auto-negotiation the timer should be 10 ms.
	 * The link_timer register is configured in units of the clock.
	 * - When running as 1G SGMII, Serdes clock is 125 MHz, so
	 * unit = 1 / (125*10^6 Hz) = 8 ns.
	 * 10 ms in units of 8 ns = 10ms / 8ns = 1250000 = 0x1312d0
	 * - When running as 2.5G SGMII, Serdes clock is 312.5 MHz, so
	 * unit = 1 / (312.5*10^6 Hz) = 3.2 ns.
	 * 10 ms in units of 3.2 ns = 10ms / 3.2ns = 3125000 = 0x2faf08.
	 * Since link_timer value of 1G SGMII will be too short for 2.5 SGMII,
	 * we always set up here a value of 2.5 SGMII.
	 */
	memac_mii_write_phy_reg(memac, phy_addr, 0x13, 0x002f);
	memac_mii_write_phy_reg(memac, phy_addr, 0x12, 0xaf08);

	/* Restart AN */
	tmp_reg16 = PHY_SGMII_CR_DEF_VAL | PHY_SGMII_CR_RESET_AN;
	memac_mii_write_phy_reg(memac, phy_addr, 0x0, tmp_reg16);

	/* Restore original enet mode */
	memac->enet_mode = enet_mode;
}

static int check_init_parameters(struct memac_t *memac)
{
	if (memac->addr == 0) {
		pr_err("Ethernet MAC must have a valid MAC address\n");
		return -EINVAL;
	}
	if (!memac->exception_cb) {
		pr_err("Uninitialized exception handler\n");
		return -EINVAL;
	}
	if (!memac->event_cb) {
		pr_warn("Uninitialize event handler\n");
		return -EINVAL;
	}

	/* FM_LEN_CHECK_ERRATA_FMAN_SW002 Errata workaround */
	if (!memac->memac_drv_param->no_length_check_enable) {
		pr_err("Length Check!\n");
		return -EINVAL;
	}

	return 0;
}

static int get_exception_flag(enum fm_mac_exceptions exception)
{
	u32 bit_mask;

	switch (exception) {
	case FM_MAC_EX_10G_TX_ECC_ER:
		bit_mask = MEMAC_IMASK_TECC_ER;
		break;
	case FM_MAC_EX_10G_RX_ECC_ER:
		bit_mask = MEMAC_IMASK_RECC_ER;
		break;
	case FM_MAC_EX_TS_FIFO_ECC_ERR:
		bit_mask = MEMAC_IMASK_TSECC_ER;
		break;
	case FM_MAC_EX_MAGIC_PACKET_INDICATION:
		bit_mask = MEMAC_IMASK_MGI;
		break;
	default:
		bit_mask = 0;
		break;
	}

	return bit_mask;
}

static void memac_err_exception(void *handle)
{
	struct memac_t *memac = (struct memac_t *)handle;
	u32 event, imask;

	event = fman_memac_get_event(memac->regs, 0xffffffff);
	imask = fman_memac_get_interrupt_mask(memac->regs);

	/* Imask include both error and notification/event bits.
	 * Leaving only error bits enabled by imask.
	 * The imask error bits are shifted by 16 bits offset from
	 * their corresponding location in the ievent - hence the >> 16
	 */
	event &= ((imask & MEMAC_ALL_ERRS_IMASK) >> 16);

	fman_memac_ack_event(memac->regs, event);

	if (event & MEMAC_IEVNT_TS_ECC_ER)
		memac->exception_cb(memac->dev_id, FM_MAC_EX_TS_FIFO_ECC_ERR);
	if (event & MEMAC_IEVNT_TX_ECC_ER)
		memac->exception_cb(memac->dev_id, FM_MAC_EX_10G_TX_ECC_ER);
	if (event & MEMAC_IEVNT_RX_ECC_ER)
		memac->exception_cb(memac->dev_id, FM_MAC_EX_10G_RX_ECC_ER);
}

static void memac_exception(void *handle)
{
	struct memac_t *memac = (struct memac_t *)handle;
	u32 event, imask;

	event = fman_memac_get_event(memac->regs, 0xffffffff);
	imask = fman_memac_get_interrupt_mask(memac->regs);

	/* Imask include both error and notification/event bits.
	 * Leaving only error bits enabled by imask.
	 * The imask error bits are shifted by 16 bits offset from
	 * their corresponding location in the ievent - hence the >> 16
	 */
	event &= ((imask & MEMAC_ALL_ERRS_IMASK) >> 16);

	fman_memac_ack_event(memac->regs, event);

	if (event & MEMAC_IEVNT_MGI)
		memac->exception_cb(memac->dev_id,
				    FM_MAC_EX_MAGIC_PACKET_INDICATION);
}

static void free_init_resources(struct memac_t *memac)
{
	fm_unregister_intr(memac->fm, FM_MOD_MAC, memac->mac_id,
			   FM_INTR_TYPE_ERR);

	/* release the driver's group hash table */
	free_hash_table(memac->multicast_addr_hash);
	memac->multicast_addr_hash = NULL;

	/* release the driver's individual hash table */
	free_hash_table(memac->unicast_addr_hash);
	memac->unicast_addr_hash = NULL;
}

static bool is_init_done(struct memac_cfg *memac_drv_params)
{
	/* Checks if mEMAC driver parameters were initialized */
	if (!memac_drv_params)
		return true;

	return false;
}

int memac_enable(struct fm_mac_dev *fm_mac_dev, enum comm_mode mode)
{
	struct memac_t *memac = (struct memac_t *)fm_mac_dev;

	if (!is_init_done(memac->memac_drv_param))
		return -EINVAL;

	fman_memac_enable(memac->regs,
			  (mode & COMM_MODE_RX), (mode & COMM_MODE_TX));

	return 0;
}

int memac_disable(struct fm_mac_dev *fm_mac_dev, enum comm_mode mode)
{
	struct memac_t *memac = (struct memac_t *)fm_mac_dev;

	if (!is_init_done(memac->memac_drv_param))
		return -EINVAL;

	fman_memac_disable(memac->regs,
			   (mode & COMM_MODE_RX), (mode & COMM_MODE_TX));

	return 0;
}

int memac_set_promiscuous(struct fm_mac_dev *fm_mac_dev, bool new_val)
{
	struct memac_t *memac = (struct memac_t *)fm_mac_dev;

	if (!is_init_done(memac->memac_drv_param))
		return -EINVAL;

	fman_memac_set_promiscuous(memac->regs, new_val);

	return 0;
}

int memac_adjust_link(struct fm_mac_dev *fm_mac_dev, enum ethernet_speed speed)
{
	struct memac_t *memac = (struct memac_t *)fm_mac_dev;
	int full_duplex = true;

	if (!is_init_done(memac->memac_drv_param))
		return -EINVAL;

	fman_memac_adjust_link(memac->regs,
			       (enum enet_interface)
			       ENET_INTERFACE_FROM_MODE(memac->enet_mode),
			       (enum enet_speed)speed, full_duplex);
	return 0;
}

int memac_cfg_max_frame_len(struct fm_mac_dev *fm_mac_dev, u16 new_val)
{
	struct memac_t *memac = (struct memac_t *)fm_mac_dev;

	if (is_init_done(memac->memac_drv_param))
		return -EINVAL;

	memac->memac_drv_param->max_frame_length = new_val;

	return 0;
}

int memac_cfg_reset_on_init(struct fm_mac_dev *fm_mac_dev, bool enable)
{
	struct memac_t *memac = (struct memac_t *)fm_mac_dev;

	if (is_init_done(memac->memac_drv_param))
		return -EINVAL;

	memac->memac_drv_param->reset_on_init = enable;

	return 0;
}

int memac_cfg_fixed_link(struct fm_mac_dev *fm_mac_dev, bool enable)
{
	struct memac_t *memac = (struct memac_t *)fm_mac_dev;

	if (is_init_done(memac->memac_drv_param))
		return -EINVAL;

	memac->memac_drv_param->fixed_link = enable;

	return 0;
}

int memac_set_tx_pause_frames(struct fm_mac_dev *fm_mac_dev,
			      u8 __maybe_unused priority,
			      u16 pause_time,
			      u16 __maybe_unused thresh_time)
{
	struct memac_t *memac = (struct memac_t *)fm_mac_dev;

	if (!is_init_done(memac->memac_drv_param))
		return -EINVAL;

	fman_memac_set_tx_pause_frames(memac->regs, FM_MAC_NO_PFC,
				       pause_time, 0);

	return 0;
}

int memac_accept_rx_pause_frames(struct fm_mac_dev *fm_mac_dev, bool en)
{
	struct memac_t *memac = (struct memac_t *)fm_mac_dev;

	if (!is_init_done(memac->memac_drv_param))
		return -EINVAL;

	fman_memac_set_rx_ignore_pause_frames(memac->regs, !en);

	return 0;
}

int memac_modify_mac_address(struct fm_mac_dev *fm_mac_dev,
			     enet_addr_t *p_enet_addr)
{
	struct memac_t *memac = (struct memac_t *)fm_mac_dev;

	if (!is_init_done(memac->memac_drv_param))
		return -EINVAL;

	fman_memac_add_addr_in_paddr(memac->regs, (u8 *)(*p_enet_addr), 0);

	return 0;
}

int memac_add_hash_mac_address(struct fm_mac_dev *fm_mac_dev,
			       enet_addr_t *eth_addr)
{
	struct memac_t *memac = (struct memac_t *)fm_mac_dev;
	struct eth_hash_entry_t *hash_entry;
	u32 hash;
	u64 addr;

	if (!is_init_done(memac->memac_drv_param))
		return -EINVAL;

	addr = ENET_ADDR_TO_UINT64(*eth_addr);

	if (!(addr & GROUP_ADDRESS)) {
		/* Unicast addresses not supported in hash */
		pr_err("Unicast Address\n");
		return -EINVAL;
	}
	hash = get_mac_addr_hash_code(addr) & HASH_CTRL_ADDR_MASK;

	/* Create element to be added to the driver hash table */
	hash_entry = kmalloc(sizeof(*hash_entry), GFP_KERNEL);
	if (!hash_entry)
		return -ENOMEM;
	hash_entry->addr = addr;
	INIT_LIST_HEAD(&hash_entry->node);

	list_add_tail(&hash_entry->node,
		      &memac->multicast_addr_hash->lsts[hash]);
	fman_memac_set_hash_table(memac->regs,
				  (hash | HASH_CTRL_MCAST_EN));

	return 0;
}

int memac_del_hash_mac_address(struct fm_mac_dev *fm_mac_dev,
			       enet_addr_t *eth_addr)
{
	struct memac_t *memac = (struct memac_t *)fm_mac_dev;
	struct eth_hash_entry_t *hash_entry = NULL;
	struct list_head *p_pos;
	u32 hash;
	u64 addr;

	if (!is_init_done(memac->memac_drv_param))
		return -EINVAL;

	addr = ENET_ADDR_TO_UINT64(*eth_addr);

	hash = get_mac_addr_hash_code(addr) & HASH_CTRL_ADDR_MASK;

	list_for_each(p_pos, &memac->multicast_addr_hash->lsts[hash]) {
		hash_entry = ETH_HASH_ENTRY_OBJ(p_pos);
		if (hash_entry->addr == addr) {
			list_del_init(&hash_entry->node);
			kfree(hash_entry);
			break;
		}
	}
	if (list_empty(&memac->multicast_addr_hash->lsts[hash]))
		fman_memac_set_hash_table(memac->regs,
					  (hash & ~HASH_CTRL_MCAST_EN));

	return 0;
}

int memac_set_exception(struct fm_mac_dev *fm_mac_dev,
			enum fm_mac_exceptions exception,
			bool enable)
{
	struct memac_t *memac = (struct memac_t *)fm_mac_dev;
	u32 bit_mask = 0;

	if (!is_init_done(memac->memac_drv_param))
		return -EINVAL;

	bit_mask = get_exception_flag(exception);
	if (bit_mask) {
		if (enable)
			memac->exceptions |= bit_mask;
		else
			memac->exceptions &= ~bit_mask;
	} else {
		pr_err("Undefined exception\n");
		return -EINVAL;
	}
	fman_memac_set_exception(memac->regs, bit_mask, enable);

	return 0;
}

int memac_init(struct fm_mac_dev *fm_mac_dev)
{
	struct memac_t *memac = (struct memac_t *)fm_mac_dev;
	struct memac_cfg *memac_drv_param;
	enum enet_interface enet_interface;
	enum enet_speed enet_speed;
	u8 i, phy_addr;
	enet_addr_t eth_addr;
	enum fm_mac_type port_type;
	bool slow_10g_if = false;
	bool fixed_link;
	int err;
	u32 reg32 = 0;

	if (is_init_done(memac->memac_drv_param))
		return -EINVAL;

	err = check_init_parameters(memac);
	if (err)
		return err;

	memac_drv_param = memac->memac_drv_param;

	if (memac->fm_rev_info.major_rev == 6 &&
	    memac->fm_rev_info.minor_rev == 4)
		slow_10g_if = true;

	port_type =
	    ((ENET_SPEED_FROM_MODE(memac->enet_mode) < ENET_SPEED_10000) ?
	    FM_MAC_1G : FM_MAC_10G);

	/* First, reset the MAC if desired. */
	if (memac_drv_param->reset_on_init) {
		err = fman_memac_reset(memac->regs);
		if (err) {
			pr_err("fman_memac_reset() failed\n");
			return err;
		}
	}

	/* MAC Address */
	MAKE_ENET_ADDR_FROM_UINT64(memac->addr, eth_addr);
	fman_memac_add_addr_in_paddr(memac->regs, (u8 *)eth_addr, 0);

	enet_interface =
	    (enum enet_interface)ENET_INTERFACE_FROM_MODE(memac->enet_mode);
	enet_speed = (enum enet_speed)ENET_SPEED_FROM_MODE(memac->enet_mode);
	fixed_link = memac_drv_param->fixed_link;

	fman_memac_init(memac->regs,
			memac->memac_drv_param,
			enet_interface,
			enet_speed,
			slow_10g_if,
			memac->exceptions);

	/* FM_RX_FIFO_CORRUPT_ERRATA_10GMAC_A006320 errata workaround
	 * Exists only in FMan 6.0 and 6.3.
	 */
	if ((memac->fm_rev_info.major_rev == 6) &&
	    ((memac->fm_rev_info.minor_rev == 0) ||
	    (memac->fm_rev_info.minor_rev == 3))) {
		/* MAC strips CRC from received frames - this workaround
		 * should decrease the likelihood of bug appearance
		 */
		reg32 = in_be32(&memac->regs->command_config);
		reg32 &= ~CMD_CFG_CRC_FWD;
		out_be32(&memac->regs->command_config, reg32);
	}

	if (ENET_INTERFACE_FROM_MODE(memac->enet_mode) == ENET_IF_SGMII) {
		/* Configure internal SGMII PHY */
		if (memac->enet_mode & ENET_IF_SGMII_BASEX)
			setup_sgmii_internal_phy_base_x(memac, PHY_MDIO_ADDR);
		else
			setup_sgmii_internal_phy(memac, PHY_MDIO_ADDR,
						 fixed_link);
	} else if (ENET_INTERFACE_FROM_MODE(memac->enet_mode) ==
		   ENET_IF_QSGMII) {
		/* Configure 4 internal SGMII PHYs */
		for (i = 0; i < 4; i++) {
			/* QSGMII PHY address occupies 3 upper bits of 5-bit
			 * phy_address; the lower 2 bits are used to extend
			 * register address space and access each one of 4
			 * ports inside QSGMII.
			 */
			phy_addr = (u8)((PHY_MDIO_ADDR << 2) | i);
			if (memac->enet_mode & ENET_IF_SGMII_BASEX)
				setup_sgmii_internal_phy_base_x(memac,
								phy_addr);
			else
				setup_sgmii_internal_phy(memac, phy_addr,
							 fixed_link);
		}
	}

	/* Max Frame Length */
	err = fm_set_mac_max_frame(memac->fm, port_type, memac->mac_id,
				   memac_drv_param->max_frame_length);
	if (err) {
		pr_err("settings Mac max frame length is FAILED\n");
		return err;
	}

	memac->multicast_addr_hash = alloc_hash_table(HASH_TABLE_SIZE);
	if (!memac->multicast_addr_hash) {
		free_init_resources(memac);
		pr_err("allocation hash table is FAILED\n");
		return -ENOMEM;
	}

	memac->unicast_addr_hash = alloc_hash_table(HASH_TABLE_SIZE);
	if (!memac->unicast_addr_hash) {
		free_init_resources(memac);
		pr_err("allocation hash table is FAILED\n");
		return -ENOMEM;
	}

	fm_register_intr(memac->fm, FM_MOD_MAC, memac->mac_id,
			 FM_INTR_TYPE_ERR, memac_err_exception, memac);

	fm_register_intr(memac->fm, FM_MOD_MAC, memac->mac_id,
			 FM_INTR_TYPE_NORMAL, memac_exception, memac);

	kfree(memac_drv_param);
	memac->memac_drv_param = NULL;

	return 0;
}

int memac_free(struct fm_mac_dev *fm_mac_dev)
{
	struct memac_t *memac = (struct memac_t *)fm_mac_dev;

	free_init_resources(memac);

	kfree(memac->memac_drv_param);
	kfree(memac);

	return 0;
}

void *memac_config(struct fm_mac_params_t *fm_mac_param)
{
	struct memac_t *memac;
	struct memac_cfg *memac_drv_param;
	void __iomem *base_addr;

	base_addr = fm_mac_param->base_addr;
	/* allocate memory for the m_emac data structure */
	memac = kzalloc(sizeof(*memac), GFP_KERNEL);
	if (!memac)
		return NULL;

	/* allocate memory for the m_emac driver parameters data structure */
	memac_drv_param = kzalloc(sizeof(*memac_drv_param), GFP_KERNEL);
	if (!memac_drv_param) {
		memac_free((struct fm_mac_dev *)memac);
		return NULL;
	}

	/* Plant parameter structure pointer */
	memac->memac_drv_param = memac_drv_param;

	fman_memac_defconfig(memac_drv_param);

	memac->addr = ENET_ADDR_TO_UINT64(fm_mac_param->addr);

	memac->regs = (struct memac_regs __iomem *)(base_addr);
	memac->mii_regs = (struct memac_mii_access_mem_map __iomem *)
		(base_addr + MEMAC_TO_MII_OFFSET);
	memac->enet_mode = fm_mac_param->enet_mode;
	memac->mac_id = fm_mac_param->mac_id;
	memac->exceptions = MEMAC_DEFAULT_EXCEPTIONS;
	memac->exception_cb = fm_mac_param->exception_cb;
	memac->event_cb = fm_mac_param->event_cb;
	memac->dev_id = fm_mac_param->dev_id;
	memac->fm = fm_mac_param->fm;

	/* Save FMan revision */
	fm_get_revision(memac->fm, &memac->fm_rev_info);

	return memac;
}
