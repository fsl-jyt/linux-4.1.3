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
#include "fm_dtsec.h"
#include "fsl_fman_dtsec.h"
#include "fsl_fman_dtsec_mii_acc.h"

#include <linux/string.h>
#include <linux/slab.h>
#include <linux/bitrev.h>

static int check_init_parameters(struct dtsec_t *dtsec)
{
	if (ENET_SPEED_FROM_MODE(dtsec->enet_mode) >= ENET_SPEED_10000) {
		pr_err("1G MAC driver supports 1G or lower speeds\n");
		return -EINVAL;
	}
	if (dtsec->addr == 0) {
		pr_err("Ethernet MAC Must have a valid MAC Address\n");
		return -EINVAL;
	}
	if ((ENET_SPEED_FROM_MODE(dtsec->enet_mode) >= ENET_SPEED_1000) &&
	    dtsec->dtsec_drv_param->halfdup_on) {
		pr_err("Ethernet MAC 1G can't work in half duplex\n");
		return -EINVAL;
	}

	/* FM_RX_PREAM_4_ERRATA_DTSEC_A001 Errata workaround */
	if (dtsec->dtsec_drv_param->rx_preamble) {
		pr_err("preamble_rx_en\n");
		return -EINVAL;
	}

	if (((dtsec->dtsec_drv_param)->tx_preamble ||
	     (dtsec->dtsec_drv_param)->rx_preamble) &&
	    ((dtsec->dtsec_drv_param)->preamble_len != 0x7)) {
		pr_err("Preamble length should be 0x7 bytes\n");
		return -EINVAL;
	}
	if ((dtsec->dtsec_drv_param)->halfdup_on &&
	    (dtsec->dtsec_drv_param->tx_time_stamp_en ||
	     dtsec->dtsec_drv_param->rx_time_stamp_en)) {
		pr_err("1588 timeStamp disabled in half duplex mode\n");
		return -EINVAL;
	}
	if ((dtsec->dtsec_drv_param)->rx_flow &&
	    (dtsec->dtsec_drv_param)->rx_ctrl_acc) {
		pr_err("Receive control frame can not be accepted\n");
		return -EINVAL;
	}
	if ((dtsec->dtsec_drv_param)->rx_prepend >
	    MAX_PACKET_ALIGNMENT) {
		pr_err("packetAlignmentPadding can't be > than %d\n",
		       MAX_PACKET_ALIGNMENT);
		return -EINVAL;
	}
	if (((dtsec->dtsec_drv_param)->non_back_to_back_ipg1 >
	     MAX_INTER_PACKET_GAP) ||
	    ((dtsec->dtsec_drv_param)->non_back_to_back_ipg2 >
	     MAX_INTER_PACKET_GAP) ||
	     ((dtsec->dtsec_drv_param)->back_to_back_ipg >
	      MAX_INTER_PACKET_GAP)) {
		pr_err("Inter packet gap can't be greater than %d\n",
		       MAX_INTER_PACKET_GAP);
		return -EINVAL;
	}
	if ((dtsec->dtsec_drv_param)->halfdup_alt_backoff_val >
	    MAX_INTER_PALTERNATE_BEB) {
		pr_err("alternateBackoffVal can't be greater than %d\n",
		       MAX_INTER_PALTERNATE_BEB);
		return -EINVAL;
	}
	if ((dtsec->dtsec_drv_param)->halfdup_retransmit >
	    MAX_RETRANSMISSION) {
		pr_err("maxRetransmission can't be greater than %d\n",
		       MAX_RETRANSMISSION);
		return -EINVAL;
	}
	if ((dtsec->dtsec_drv_param)->halfdup_coll_window >
	    MAX_COLLISION_WINDOW) {
		pr_err("collisionWindow can't be greater than %d\n",
		       MAX_COLLISION_WINDOW);
		return -EINVAL;
	/* If Auto negotiation process is disabled, need to set up the PHY
	 * using the MII Management Interface
	 */
	}
	if (dtsec->dtsec_drv_param->tbipa > MAX_PHYS) {
		pr_err("PHY address (should be 0-%d)\n", MAX_PHYS);
		return -ERANGE;
	}
	if (!dtsec->exception_cb) {
		pr_err("uninitialized exception_cb\n");
		return -EINVAL;
	}
	if (!dtsec->event_cb) {
		pr_err("uninitialized event_cb\n");
		return -EINVAL;
	}

	/* FM_LEN_CHECK_ERRATA_FMAN_SW002 Errata workaround */
	if (dtsec->dtsec_drv_param->rx_len_check) {
		pr_warn("Length Check!\n");
		return -EINVAL;
	}

	return 0;
}

static int get_exception_flag(enum fm_mac_exceptions exception)
{
	u32 bit_mask;

	switch (exception) {
	case FM_MAC_EX_1G_BAB_RX:
		bit_mask = DTSEC_IMASK_BREN;
		break;
	case FM_MAC_EX_1G_RX_CTL:
		bit_mask = DTSEC_IMASK_RXCEN;
		break;
	case FM_MAC_EX_1G_GRATEFUL_TX_STP_COMPLET:
		bit_mask = DTSEC_IMASK_GTSCEN;
		break;
	case FM_MAC_EX_1G_BAB_TX:
		bit_mask = DTSEC_IMASK_BTEN;
		break;
	case FM_MAC_EX_1G_TX_CTL:
		bit_mask = DTSEC_IMASK_TXCEN;
		break;
	case FM_MAC_EX_1G_TX_ERR:
		bit_mask = DTSEC_IMASK_TXEEN;
		break;
	case FM_MAC_EX_1G_LATE_COL:
		bit_mask = DTSEC_IMASK_LCEN;
		break;
	case FM_MAC_EX_1G_COL_RET_LMT:
		bit_mask = DTSEC_IMASK_CRLEN;
		break;
	case FM_MAC_EX_1G_TX_FIFO_UNDRN:
		bit_mask = DTSEC_IMASK_XFUNEN;
		break;
	case FM_MAC_EX_1G_MAG_PCKT:
		bit_mask = DTSEC_IMASK_MAGEN;
		break;
	case FM_MAC_EX_1G_MII_MNG_RD_COMPLET:
		bit_mask = DTSEC_IMASK_MMRDEN;
		break;
	case FM_MAC_EX_1G_MII_MNG_WR_COMPLET:
		bit_mask = DTSEC_IMASK_MMWREN;
		break;
	case FM_MAC_EX_1G_GRATEFUL_RX_STP_COMPLET:
		bit_mask = DTSEC_IMASK_GRSCEN;
		break;
	case FM_MAC_EX_1G_DATA_ERR:
		bit_mask = DTSEC_IMASK_TDPEEN;
		break;
	case FM_MAC_EX_1G_RX_MIB_CNT_OVFL:
		bit_mask = DTSEC_IMASK_MSROEN;
		break;
	default:
		bit_mask = 0;
		break;
	}

	return bit_mask;
}

static bool is_init_done(struct dtsec_cfg *dtsec_drv_params)
{
	/* Checks if dTSEC driver parameters were initialized */
	if (!dtsec_drv_params)
		return true;

	return false;
}

static u32 get_mac_addr_hash_code(u64 eth_addr)
{
	u32 crc;

	/* CRC calculation */
	GET_MAC_ADDR_CRC(eth_addr, crc);

	crc = bitrev32(crc);

	return crc;
}

static u16 dtsec_get_max_frame_length(struct dtsec_t *dtsec)
{
	if (is_init_done(dtsec->dtsec_drv_param))
		return 0;

	return fman_dtsec_get_max_frame_len(dtsec->regs);
}

static void dtsec_isr(void *handle)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)handle;
	u32 event;
	struct dtsec_regs __iomem *dtsec_regs = dtsec->regs;

	/* do not handle MDIO events */
	event = fman_dtsec_get_event(dtsec_regs,
				     (u32)(~(DTSEC_IMASK_MMRDEN |
					     DTSEC_IMASK_MMWREN)));

	event &= fman_dtsec_get_interrupt_mask(dtsec_regs);

	fman_dtsec_ack_event(dtsec_regs, event);

	if (event & DTSEC_IMASK_BREN)
		dtsec->exception_cb(dtsec->dev_id, FM_MAC_EX_1G_BAB_RX);
	if (event & DTSEC_IMASK_RXCEN)
		dtsec->exception_cb(dtsec->dev_id, FM_MAC_EX_1G_RX_CTL);
	if (event & DTSEC_IMASK_GTSCEN)
		dtsec->exception_cb(dtsec->dev_id,
				    FM_MAC_EX_1G_GRATEFUL_TX_STP_COMPLET);
	if (event & DTSEC_IMASK_BTEN)
		dtsec->exception_cb(dtsec->dev_id, FM_MAC_EX_1G_BAB_TX);
	if (event & DTSEC_IMASK_TXCEN)
		dtsec->exception_cb(dtsec->dev_id, FM_MAC_EX_1G_TX_CTL);
	if (event & DTSEC_IMASK_TXEEN)
		dtsec->exception_cb(dtsec->dev_id, FM_MAC_EX_1G_TX_ERR);
	if (event & DTSEC_IMASK_LCEN)
		dtsec->exception_cb(dtsec->dev_id, FM_MAC_EX_1G_LATE_COL);
	if (event & DTSEC_IMASK_CRLEN)
		dtsec->exception_cb(dtsec->dev_id, FM_MAC_EX_1G_COL_RET_LMT);
	if (event & DTSEC_IMASK_XFUNEN) {
		/* FM_TX_LOCKUP_ERRATA_DTSEC6 Errata workaround */
		if (dtsec->fm_rev_info.major_rev == 2) {
			u32 tpkt1, tmp_reg1, tpkt2, tmp_reg2, i;
			/* a. Write 0x00E0_0C00 to DTSEC_ID
			 *	This is a read only register
			 * b. Read and save the value of TPKT
			 */
			tpkt1 = in_be32(&dtsec_regs->tpkt);

			/* c. Read the register at dTSEC address offset 0x32C */
			tmp_reg1 = in_be32(&dtsec_regs->reserved02c0[27]);

			/* d. Compare bits [9:15] to bits [25:31] of the
			 * register at address offset 0x32C.
			 */
			if ((tmp_reg1 & 0x007F0000) !=
				(tmp_reg1 & 0x0000007F)) {
				/* If they are not equal, save the value of
				 * this register and wait for at least
				 * MAXFRM*16 ns
				 */
				usleep_range((u32)(min
					(dtsec_get_max_frame_length(dtsec) *
					16 / 1000, 1)), (u32)
					(min(dtsec_get_max_frame_length
					(dtsec) * 16 / 1000, 1) + 1));
			}

			/* e. Read and save TPKT again and read the register
			 * at dTSEC address offset 0x32C again
			 */
			tpkt2 = in_be32(&dtsec_regs->tpkt);
			tmp_reg2 = in_be32(&dtsec_regs->reserved02c0[27]);

			/* f. Compare the value of TPKT saved in step b to
			 * value read in step e. Also compare bits [9:15] of
			 * the register at offset 0x32C saved in step d to the
			 * value of bits [9:15] saved in step e. If the two
			 * registers values are unchanged, then the transmit
			 * portion of the dTSEC controller is locked up and
			 * the user should proceed to the recover sequence.
			 */
			if ((tpkt1 == tpkt2) && ((tmp_reg1 & 0x007F0000) ==
				(tmp_reg2 & 0x007F0000))) {
				/* recover sequence */

				/* a.Write a 1 to RCTRL[GRS] */

				out_be32(&dtsec_regs->rctrl,
					 in_be32(&dtsec_regs->rctrl) |
					 RCTRL_GRS);

				/* b.Wait until IEVENT[GRSC]=1, or at least
				 * 100 us has elapsed.
				 */
				for (i = 0; i < 100; i++) {
					if (in_be32(&dtsec_regs->ievent) &
					    DTSEC_IMASK_GRSCEN)
						break;
					udelay(1);
				}
				if (in_be32(&dtsec_regs->ievent) &
				    DTSEC_IMASK_GRSCEN)
					out_be32(&dtsec_regs->ievent,
						 DTSEC_IMASK_GRSCEN);
				else
					pr_debug("Rx lockup due to Tx lockup\n");

				/* c.Write a 1 to bit n of FM_RSTC
				 * (offset 0x0CC of FPM)
				 */
				fm_reset_mac(dtsec->fm, dtsec->mac_id);

				/* d.Wait 4 Tx clocks (32 ns) */
				udelay(1);

				/* e.Write a 0 to bit n of FM_RSTC. */
				/* cleared by FMAN
				 */
			}
		}

		dtsec->exception_cb(dtsec->dev_id, FM_MAC_EX_1G_TX_FIFO_UNDRN);
	}
	if (event & DTSEC_IMASK_MAGEN)
		dtsec->exception_cb(dtsec->dev_id, FM_MAC_EX_1G_MAG_PCKT);
	if (event & DTSEC_IMASK_GRSCEN)
		dtsec->exception_cb(dtsec->dev_id,
				    FM_MAC_EX_1G_GRATEFUL_RX_STP_COMPLET);
	if (event & DTSEC_IMASK_TDPEEN)
		dtsec->exception_cb(dtsec->dev_id, FM_MAC_EX_1G_DATA_ERR);
	if (event & DTSEC_IMASK_RDPEEN)
		dtsec->exception_cb(dtsec->dev_id, FM_MAC_1G_RX_DATA_ERR);

	/* masked interrupts */
	WARN_ON(event & DTSEC_IMASK_ABRTEN);
	WARN_ON(event & DTSEC_IMASK_IFERREN);
}

static void dtsec_1588_isr(void *handle)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)handle;
	u32 event;
	struct dtsec_regs __iomem *dtsec_regs = dtsec->regs;

	if (dtsec->ptp_tsu_enabled) {
		event = fman_dtsec_check_and_clear_tmr_event(dtsec_regs);

		if (event) {
			WARN_ON(event & TMR_PEVENT_TSRE);
			dtsec->exception_cb(dtsec->dev_id,
					     FM_MAC_EX_1G_1588_TS_RX_ERR);
		}
	}
}

static void free_init_resources(struct dtsec_t *dtsec)
{
	fm_unregister_intr(dtsec->fm, FM_MOD_MAC, dtsec->mac_id,
			   FM_INTR_TYPE_ERR);
	fm_unregister_intr(dtsec->fm, FM_MOD_MAC, dtsec->mac_id,
			   FM_INTR_TYPE_NORMAL);

	/* release the driver's group hash table */
	free_hash_table(dtsec->multicast_addr_hash);
	dtsec->multicast_addr_hash = NULL;

	/* release the driver's individual hash table */
	free_hash_table(dtsec->unicast_addr_hash);
	dtsec->unicast_addr_hash = NULL;
}

static int graceful_stop(struct dtsec_t *dtsec, enum comm_mode mode)
{
	struct dtsec_regs __iomem *regs;

	regs = dtsec->regs;

	/* Assert the graceful transmit stop bit */
	if (mode & COMM_MODE_RX) {
		fman_dtsec_stop_rx(regs);

		if (dtsec->fm_rev_info.major_rev == 2)
			usleep_range(100, 200);
		else
			udelay(10);
	}

	if (mode & COMM_MODE_TX) {
		if (dtsec->fm_rev_info.major_rev == 2)
			pr_debug("GTS not supported due to DTSEC_A004 errata.\n");
		else
			pr_debug("GTS not supported due to DTSEC_A0014 errata.\n");
	}
	return 0;
}

static int graceful_restart(struct dtsec_t *dtsec, enum comm_mode mode)
{
	struct dtsec_regs __iomem *regs;

	regs = dtsec->regs;
	/* clear the graceful receive stop bit */
	if (mode & COMM_MODE_TX)
		fman_dtsec_start_tx(regs);

	if (mode & COMM_MODE_RX)
		fman_dtsec_start_rx(regs);

	return 0;
}

static int dtsec_mii_write_phy_reg(struct dtsec_t *dtsec, u8 phy_addr,
				   u8 reg, u16 data)
{
	u16 dtsec_freq;

	dtsec_freq = (u16)(dtsec->clk_freq >> 1);

	return fman_dtsec_mii_write_reg(dtsec->mii_regs, phy_addr, reg,
					data, dtsec_freq);
}

static int dtsec_mii_read_phy_reg(struct dtsec_t *dtsec, u8 phy_addr,
				  u8 reg, u16 *data)
{
	u16 dtsec_freq;
	int err;

	dtsec_freq = (u16)(dtsec->clk_freq >> 1);

	err = fman_dtsec_mii_read_reg(dtsec->mii_regs, phy_addr, reg,
				      data, dtsec_freq);

	if (*data == 0xffff) {
		pr_warn("Read wrong data(0xffff):phy_addr 0x%x,reg 0x%x",
			phy_addr, reg);
		return -ENXIO;
	}
	if (err)
		return err;

	return err;
}

int dtsec_cfg_max_frame_len(struct fm_mac_dev *fm_mac_dev, u16 new_val)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;

	if (is_init_done(dtsec->dtsec_drv_param))
		return -EINVAL;

	dtsec->dtsec_drv_param->maximum_frame = new_val;

	return 0;
}

int dtsec_cfg_pad_and_crc(struct fm_mac_dev *fm_mac_dev, bool new_val)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;

	if (is_init_done(dtsec->dtsec_drv_param))
		return -EINVAL;

	dtsec->dtsec_drv_param->tx_pad_crc = new_val;

	return 0;
}

int dtsec_enable(struct fm_mac_dev *fm_mac_dev, enum comm_mode mode)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;

	if (!is_init_done(dtsec->dtsec_drv_param))
		return -EINVAL;

	fman_dtsec_enable(dtsec->regs,
			  (bool)!!(mode & COMM_MODE_RX),
			  (bool)!!(mode & COMM_MODE_TX));

	graceful_restart(dtsec, mode);

	return 0;
}

int dtsec_disable(struct fm_mac_dev *fm_mac_dev, enum comm_mode mode)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;

	if (!is_init_done(dtsec->dtsec_drv_param))
		return -EINVAL;

	graceful_stop(dtsec, mode);

	fman_dtsec_disable(dtsec->regs,
			   (bool)!!(mode & COMM_MODE_RX),
			   (bool)!!(mode & COMM_MODE_TX));

	return 0;
}

int dtsec_set_tx_pause_frames(struct fm_mac_dev *fm_mac_dev,
			      u8 __maybe_unused priority,
			      u16 pause_time, u16 __maybe_unused thresh_time)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;

	if (!is_init_done(dtsec->dtsec_drv_param))
		return -EINVAL;

	/* FM_BAD_TX_TS_IN_B_2_B_ERRATA_DTSEC_A003 Errata workaround */
	if (dtsec->fm_rev_info.major_rev == 2)
		if (0 < pause_time && pause_time <= 320) {
			pr_warn("pause-time: %d illegal.Should be > 320\n",
				pause_time);
			return -EINVAL;
		}

	fman_dtsec_set_tx_pause_frames(dtsec->regs, pause_time);
	return 0;
}

int dtsec_accept_rx_pause_frames(struct fm_mac_dev *fm_mac_dev, bool en)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;

	if (!is_init_done(dtsec->dtsec_drv_param))
		return -EINVAL;

	fman_dtsec_handle_rx_pause(dtsec->regs, en);

	return 0;
}

int dtsec_modify_mac_address(struct fm_mac_dev *fm_mac_dev,
			     enet_addr_t *enet_addr)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;

	if (!is_init_done(dtsec->dtsec_drv_param))
		return -EINVAL;

	/* Initialize MAC Station Address registers (1 & 2)
	 * Station address have to be swapped (big endian to little endian
	 */
	dtsec->addr = ENET_ADDR_TO_UINT64(*enet_addr);
	fman_dtsec_set_mac_address(dtsec->regs, (u8 *)(*enet_addr));

	return 0;
}

int dtsec_add_hash_mac_address(struct fm_mac_dev *fm_mac_dev,
			       enet_addr_t *eth_addr)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;
	struct eth_hash_entry_t *hash_entry;
	u64 addr;
	s32 bucket;
	u32 crc;
	bool mcast, ghtx;

	if (!is_init_done(dtsec->dtsec_drv_param))
		return -EINVAL;

	addr = ENET_ADDR_TO_UINT64(*eth_addr);

	ghtx = (bool)((fman_dtsec_get_rctrl(dtsec->regs) &
			RCTRL_GHTX) ? true : false);
	mcast = (bool)((addr & MAC_GROUP_ADDRESS) ? true : false);

	/* Cannot handle unicast mac addr when GHTX is on */
	if (ghtx && !mcast) {
		pr_err("Could not compute hash bucket\n");
		return -EINVAL;
	}
	crc = get_mac_addr_hash_code(addr);

	/* considering the 9 highest order bits in crc H[8:0]:
	 *if ghtx = 0 H[8:6] (highest order 3 bits) identify the hash register
	 *and H[5:1] (next 5 bits) identify the hash bit
	 *if ghts = 1 H[8:5] (highest order 4 bits) identify the hash register
	 *and H[4:0] (next 5 bits) identify the hash bit.
	 *
	 *In bucket index output the low 5 bits identify the hash register
	 *bit, while the higher 4 bits identify the hash register
	 */

	if (ghtx) {
		bucket = (s32)((crc >> 23) & 0x1ff);
	} else {
		bucket = (s32)((crc >> 24) & 0xff);
		/* if !ghtx and mcast the bit must be set in gaddr instead of
		 *igaddr.
		 */
		if (mcast)
			bucket += 0x100;
	}

	fman_dtsec_set_bucket(dtsec->regs, bucket, true);

	/* Create element to be added to the driver hash table */
	hash_entry = kmalloc(sizeof(*hash_entry), GFP_KERNEL);
	if (!hash_entry)
		return -ENOMEM;
	hash_entry->addr = addr;
	INIT_LIST_HEAD(&hash_entry->node);

	if (addr & MAC_GROUP_ADDRESS)
		/* Group Address */
		list_add_tail(&hash_entry->node,
			      &dtsec->multicast_addr_hash->lsts[bucket]);
	else
		list_add_tail(&hash_entry->node,
			      &dtsec->unicast_addr_hash->lsts[bucket]);

	return 0;
}

int dtsec_del_hash_mac_address(struct fm_mac_dev *fm_mac_dev,
			       enet_addr_t *eth_addr)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;
	struct list_head *pos;
	struct eth_hash_entry_t *hash_entry = NULL;
	u64 addr;
	s32 bucket;
	u32 crc;
	bool mcast, ghtx;

	if (!is_init_done(dtsec->dtsec_drv_param))
		return -EINVAL;

	addr = ENET_ADDR_TO_UINT64(*eth_addr);

	ghtx = (bool)((fman_dtsec_get_rctrl(dtsec->regs) &
			RCTRL_GHTX) ? true : false);
	mcast = (bool)((addr & MAC_GROUP_ADDRESS) ? true : false);

	/* Cannot handle unicast mac addr when GHTX is on */
	if (ghtx && !mcast) {
		pr_err("Could not compute hash bucket\n");
		return -EINVAL;
	}
	crc = get_mac_addr_hash_code(addr);

	if (ghtx) {
		bucket = (s32)((crc >> 23) & 0x1ff);
	} else {
		bucket = (s32)((crc >> 24) & 0xff);
		/* if !ghtx and mcast the bit must be set
		 * in gaddr instead of igaddr.
		 */
		if (mcast)
			bucket += 0x100;
	}

	if (addr & MAC_GROUP_ADDRESS) {
		/* Group Address */
		list_for_each(pos,
			      &dtsec->multicast_addr_hash->lsts[bucket]) {
			hash_entry = ETH_HASH_ENTRY_OBJ(pos);
			if (hash_entry->addr == addr) {
				list_del_init(&hash_entry->node);
				kfree(hash_entry);
				break;
			}
		}
		if (list_empty(&dtsec->multicast_addr_hash->lsts[bucket]))
			fman_dtsec_set_bucket(dtsec->regs, bucket, false);
	} else {
		/* Individual Address */
		list_for_each(pos,
			      &dtsec->unicast_addr_hash->lsts[bucket]) {
			hash_entry = ETH_HASH_ENTRY_OBJ(pos);
			if (hash_entry->addr == addr) {
				list_del_init(&hash_entry->node);
				kfree(hash_entry);
				break;
			}
		}
		if (list_empty(&dtsec->unicast_addr_hash->lsts[bucket]))
			fman_dtsec_set_bucket(dtsec->regs, bucket, false);
	}

	/* address does not exist */
	WARN_ON(!hash_entry);

	return 0;
}

int dtsec_set_promiscuous(struct fm_mac_dev *fm_mac_dev, bool new_val)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;

	if (!is_init_done(dtsec->dtsec_drv_param))
		return -EINVAL;

	fman_dtsec_set_uc_promisc(dtsec->regs, new_val);
	fman_dtsec_set_mc_promisc(dtsec->regs, new_val);

	return 0;
}

int dtsec_adjust_link(struct fm_mac_dev *fm_mac_dev, enum ethernet_speed speed)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;
	int err;
	enum enet_interface enet_interface;
	enum enet_speed enet_speed;
	int full_duplex = true;

	if (!is_init_done(dtsec->dtsec_drv_param))
		return -EINVAL;

	dtsec->enet_mode =
	    MAKE_ENET_MODE(ENET_INTERFACE_FROM_MODE(dtsec->enet_mode), speed);
	enet_interface =
	    (enum enet_interface)ENET_INTERFACE_FROM_MODE(dtsec->enet_mode);
	enet_speed = (enum enet_speed)ENET_SPEED_FROM_MODE(dtsec->enet_mode);

	err = fman_dtsec_adjust_link(dtsec->regs, enet_interface,
				     enet_speed, full_duplex);

	if (err == -EINVAL) {
		pr_err("Ethernet doesn't support Half Duplex mode\n");
		return -EINVAL;
	}

	return err;
}

int dtsec_restart_autoneg(struct fm_mac_dev *fm_mac_dev)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;
	u16 tmp_reg16;

	if (!is_init_done(dtsec->dtsec_drv_param))
		return -EINVAL;

	dtsec_mii_read_phy_reg(dtsec, dtsec->tbi_phy_addr, 0, &tmp_reg16);

	tmp_reg16 &= ~(PHY_CR_SPEED0 | PHY_CR_SPEED1);
	tmp_reg16 |=
	    (PHY_CR_ANE | PHY_CR_RESET_AN | PHY_CR_FULLDUPLEX | PHY_CR_SPEED1);

	dtsec_mii_write_phy_reg(dtsec, dtsec->tbi_phy_addr, 0, tmp_reg16);

	return 0;
}

int dtsec_get_version(struct fm_mac_dev *fm_mac_dev, u32 *mac_version)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;

	if (!is_init_done(dtsec->dtsec_drv_param))
		return -EINVAL;

	*mac_version = fman_dtsec_get_revision(dtsec->regs);

	return 0;
}

int dtsec_set_exception(struct fm_mac_dev *fm_mac_dev,
			enum fm_mac_exceptions exception,
			bool enable)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;
	u32 bit_mask = 0;

	if (!is_init_done(dtsec->dtsec_drv_param))
		return -EINVAL;

	if (exception != FM_MAC_EX_1G_1588_TS_RX_ERR) {
		bit_mask = get_exception_flag(exception);
		if (bit_mask) {
			if (enable)
				dtsec->exceptions |= bit_mask;
			else
				dtsec->exceptions &= ~bit_mask;
		} else {
			pr_err("Undefined exception\n");
			return -EINVAL;
		}
		if (enable)
			fman_dtsec_enable_interrupt(dtsec->regs, bit_mask);
		else
			fman_dtsec_disable_interrupt(dtsec->regs, bit_mask);
	} else {
		if (!dtsec->ptp_tsu_enabled) {
			pr_err("Exception valid for 1588 only\n");
			return -EINVAL;
		}
		switch (exception) {
		case FM_MAC_EX_1G_1588_TS_RX_ERR:
			if (enable) {
				dtsec->en_tsu_err_exeption = true;
				fman_dtsec_enable_tmr_interrupt(dtsec->regs);
			} else {
				dtsec->en_tsu_err_exeption = false;
				fman_dtsec_disable_tmr_interrupt(dtsec->regs);
			}
			break;
		default:
			pr_err("Undefined exception\n");
			return -EINVAL;
		}
	}

	return 0;
}

int dtsec_init(struct fm_mac_dev *fm_mac_dev)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;
	struct dtsec_cfg *dtsec_drv_param;
	int err;
	u16 max_frm_ln;
	enum enet_interface enet_interface;
	enum enet_speed enet_speed;
	enet_addr_t eth_addr;

	if (is_init_done(dtsec->dtsec_drv_param))
		return -EINVAL;

	if (DEFAULT_RESET_ON_INIT &&
	    (fm_reset_mac(dtsec->fm, dtsec->mac_id) != 0)) {
		pr_err("Can't reset MAC!\n");
		return -EINVAL;
	}

	err = check_init_parameters(dtsec);
	if (err)
		return err;

	dtsec_drv_param = dtsec->dtsec_drv_param;

	enet_interface =
	    (enum enet_interface)ENET_INTERFACE_FROM_MODE(dtsec->enet_mode);
	enet_speed = (enum enet_speed)ENET_SPEED_FROM_MODE(dtsec->enet_mode);
	MAKE_ENET_ADDR_FROM_UINT64(dtsec->addr, eth_addr);

	err = fman_dtsec_init(dtsec->regs, dtsec_drv_param, enet_interface,
			      enet_speed, (u8 *)eth_addr,
			      dtsec->fm_rev_info.major_rev,
			      dtsec->fm_rev_info.minor_rev,
			      dtsec->exceptions);
	if (err) {
		free_init_resources(dtsec);
		pr_err("DTSEC version doesn't support this i/f mode\n");
		return err;
	}

	if (ENET_INTERFACE_FROM_MODE(dtsec->enet_mode) == ENET_IF_SGMII) {
		u16 tmp_reg16;

		/* Configure the TBI PHY Control Register */
		tmp_reg16 = PHY_TBICON_CLK_SEL | PHY_TBICON_SRESET;
		dtsec_mii_write_phy_reg(dtsec, (u8)dtsec_drv_param->tbipa,
					17, tmp_reg16);

		tmp_reg16 = PHY_TBICON_CLK_SEL;
		dtsec_mii_write_phy_reg(dtsec, (u8)dtsec_drv_param->tbipa,
					17, tmp_reg16);

		tmp_reg16 =
		    (PHY_CR_PHY_RESET | PHY_CR_ANE | PHY_CR_FULLDUPLEX |
		     PHY_CR_SPEED1);
		dtsec_mii_write_phy_reg(dtsec, (u8)dtsec_drv_param->tbipa,
					0, tmp_reg16);

		if (dtsec->enet_mode & ENET_IF_SGMII_BASEX)
			tmp_reg16 = PHY_TBIANA_1000X;
		else
			tmp_reg16 = PHY_TBIANA_SGMII;
		dtsec_mii_write_phy_reg(dtsec, (u8)dtsec_drv_param->tbipa,
					4, tmp_reg16);

		tmp_reg16 =
		    (PHY_CR_ANE | PHY_CR_RESET_AN | PHY_CR_FULLDUPLEX |
		     PHY_CR_SPEED1);

		dtsec_mii_write_phy_reg(dtsec, (u8)dtsec_drv_param->tbipa,
					0, tmp_reg16);
	}

	/* Max Frame Length */
	max_frm_ln = fman_dtsec_get_max_frame_len(dtsec->regs);
	err = fm_set_mac_max_frame(dtsec->fm, FM_MAC_1G, dtsec->mac_id,
				   max_frm_ln);
	if (err) {
		pr_err("Setting max frame length failed\n");
		free_init_resources(dtsec);
		return -EINVAL;
	}

	dtsec->multicast_addr_hash =
	alloc_hash_table(EXTENDED_HASH_TABLE_SIZE);
	if (!dtsec->multicast_addr_hash) {
		free_init_resources(dtsec);
		pr_err("MC hash table is failed\n");
		return -ENOMEM;
	}

	dtsec->unicast_addr_hash = alloc_hash_table(DTSEC_HASH_TABLE_SIZE);
	if (!dtsec->unicast_addr_hash) {
		free_init_resources(dtsec);
		pr_err("UC hash table is failed\n");
		return -ENOMEM;
	}

	/* register err intr handler for dtsec to FPM (err) */
	fm_register_intr(dtsec->fm, FM_MOD_MAC, dtsec->mac_id,
			 FM_INTR_TYPE_ERR, dtsec_isr, dtsec);
	/* register 1588 intr handler for TMR to FPM (normal) */
	fm_register_intr(dtsec->fm, FM_MOD_MAC, dtsec->mac_id,
			 FM_INTR_TYPE_NORMAL, dtsec_1588_isr, dtsec);

	kfree(dtsec_drv_param);
	dtsec->dtsec_drv_param = NULL;

	return 0;
}

int dtsec_free(struct fm_mac_dev *fm_mac_dev)
{
	struct dtsec_t *dtsec = (struct dtsec_t *)fm_mac_dev;

	free_init_resources(dtsec);

	kfree(dtsec->dtsec_drv_param);
	dtsec->dtsec_drv_param = NULL;
	kfree(dtsec);

	return 0;
}

void *dtsec_config(struct fm_mac_params_t *fm_mac_param)
{
	struct dtsec_t *dtsec;
	struct dtsec_cfg *dtsec_drv_param;
	void __iomem *base_addr;

	base_addr = fm_mac_param->base_addr;

	/* allocate memory for the UCC GETH data structure. */
	dtsec = kzalloc(sizeof(*dtsec), GFP_KERNEL);
	if (!dtsec)
		return NULL;

	/* allocate memory for the d_tsec driver parameters data structure. */
	dtsec_drv_param = kzalloc(sizeof(*dtsec_drv_param), GFP_KERNEL);
	if (!dtsec_drv_param)
		goto err_dtsec;

	/* Plant parameter structure pointer */
	dtsec->dtsec_drv_param = dtsec_drv_param;

	fman_dtsec_defconfig(dtsec_drv_param);

	dtsec->regs = (struct dtsec_regs __iomem *)(base_addr);
	dtsec->mii_regs = (struct dtsec_mii_reg __iomem *)
		(base_addr + DTSEC_TO_MII_OFFSET);
	dtsec->addr = ENET_ADDR_TO_UINT64(fm_mac_param->addr);
	dtsec->enet_mode = fm_mac_param->enet_mode;
	dtsec->mac_id = fm_mac_param->mac_id;
	dtsec->exceptions = DTSEC_DEFAULT_EXCEPTIONS;
	dtsec->exception_cb = fm_mac_param->exception_cb;
	dtsec->event_cb = fm_mac_param->event_cb;
	dtsec->dev_id = fm_mac_param->dev_id;
	dtsec->ptp_tsu_enabled = dtsec->dtsec_drv_param->ptp_tsu_en;
	dtsec->en_tsu_err_exeption = dtsec->dtsec_drv_param->ptp_exception_en;
	dtsec->tbi_phy_addr = dtsec->dtsec_drv_param->tbi_phy_addr;

	dtsec->fm = fm_mac_param->fm;
	dtsec->clk_freq = fm_get_clock_freq(dtsec->fm);
	if (dtsec->clk_freq  == 0) {
		pr_err("Can't get clock for MAC!\n");
		goto err_dtsec;
	}

	/* Save FMan revision */
	fm_get_revision(dtsec->fm, &dtsec->fm_rev_info);

	return dtsec;

err_dtsec:
	kfree(dtsec);
	return NULL;
}
