/* Copyright 2008-2015 Freescale Semiconductor, Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>

#include "mac.h"
#include "fsl_fman_drv.h"
#include "fm_mac.h"
#include "fm_dtsec.h"
#include "fm_tgec.h"
#include "fm_memac.h"

#define MAC_DESCRIPTION "FSL FMan MAC API based driver"

MODULE_LICENSE("Dual BSD/GPL");

MODULE_AUTHOR("Emil Medve <Emilian.Medve@Freescale.com>");

MODULE_DESCRIPTION(MAC_DESCRIPTION);

struct mac_priv_s {
	struct fm_mac_dev *fm_mac;
};

const char	*mac_driver_description __initconst = MAC_DESCRIPTION;
const size_t	 mac_sizeof_priv[] = {
	[DTSEC] = sizeof(struct mac_priv_s),
	[XGMAC] = sizeof(struct mac_priv_s),
	[MEMAC] = sizeof(struct mac_priv_s)
};

static const enum e_enet_mode _100[] = {
	[PHY_INTERFACE_MODE_MII]	= ENET_MODE_MII_100,
	[PHY_INTERFACE_MODE_RMII]	= ENET_MODE_RMII_100
};

static const enum e_enet_mode _1000[] = {
	[PHY_INTERFACE_MODE_GMII]	= ENET_MODE_GMII_1000,
	[PHY_INTERFACE_MODE_SGMII]	= ENET_MODE_SGMII_1000,
	[PHY_INTERFACE_MODE_TBI]	= ENET_MODE_TBI_1000,
	[PHY_INTERFACE_MODE_RGMII]	= ENET_MODE_RGMII_1000,
	[PHY_INTERFACE_MODE_RGMII_ID]	= ENET_MODE_RGMII_1000,
	[PHY_INTERFACE_MODE_RGMII_RXID]	= ENET_MODE_RGMII_1000,
	[PHY_INTERFACE_MODE_RGMII_TXID]	= ENET_MODE_RGMII_1000,
	[PHY_INTERFACE_MODE_RTBI]	= ENET_MODE_RTBI_1000
};

static enum e_enet_mode __attribute__((nonnull))
macdev2enetinterface(const struct mac_device *mac_dev)
{
	switch (mac_dev->max_speed) {
	case SPEED_100:
		return _100[mac_dev->phy_if];
	case SPEED_1000:
		return _1000[mac_dev->phy_if];
	case SPEED_10000:
		return ENET_MODE_XGMII_10000;
	default:
		return ENET_MODE_MII_100;
	}
}

struct fm_mac_dev *get_mac_handle(struct mac_device *mac_dev)
{
	const struct mac_priv_s	*priv;

	priv = macdev_priv(mac_dev);

	return priv->fm_mac;
}
EXPORT_SYMBOL(get_mac_handle);

static void mac_exception(void *_mac_dev, enum fm_mac_exceptions ex)
{
	struct mac_device	*mac_dev;

	mac_dev = (struct mac_device *)_mac_dev;

	if (FM_MAC_EX_10G_RX_FIFO_OVFL == ex) {
		/* don't flag RX FIFO after the first */
		mac_dev->set_exception(get_mac_handle(mac_dev),
				       FM_MAC_EX_10G_RX_FIFO_OVFL, false);
		dev_err(mac_dev->dev, "10G MAC got RX FIFO Error = %x\n",
			ex);
	}

	dev_dbg(mac_dev->dev, "%s:%s() -> %d\n", KBUILD_BASENAME ".c",
		__func__, ex);
}

static int tgec_initialization(struct mac_device *mac_dev)
{
	int err;
	struct mac_priv_s	*priv;
	struct fm_mac_params_t		param;
	u32			version;

	priv = macdev_priv(mac_dev);

	param.base_addr = (typeof(param.base_addr))
		devm_ioremap(mac_dev->dev, mac_dev->res->start, 0x2000);
	param.enet_mode	= macdev2enetinterface(mac_dev);
	memcpy(&param.addr, mac_dev->addr,
	       min(sizeof(param.addr), sizeof(mac_dev->addr)));
	param.mac_id		= mac_dev->cell_index;
	param.fm		= (void *)mac_dev->fm;
	param.exception_cb	= mac_exception;
	param.event_cb		= mac_exception;
	param.dev_id		= mac_dev;

	priv->fm_mac = tgec_config(&param);
	if (!priv->fm_mac) {
		err = -EINVAL;
		goto _return;
	}

	err = tgec_cfg_max_frame_len(priv->fm_mac, fm_get_max_frm());
	if (err < 0)
		goto _return_fm_mac_free;

	err = tgec_init(priv->fm_mac);
	if (err < 0)
		goto _return_fm_mac_free;

	/* For 1G MAC, disable by default the MIB counters overflow interrupt */
	if (macdev2enetinterface(mac_dev) != ENET_MODE_XGMII_10000) {
		err = mac_dev->set_exception(get_mac_handle(mac_dev),
					     FM_MAC_EX_1G_RX_MIB_CNT_OVFL,
					     false);
		if (err < 0)
			goto _return_fm_mac_free;
	}

	/* For 10G MAC, disable Tx ECC exception */
	if (macdev2enetinterface(mac_dev) == ENET_MODE_XGMII_10000) {
		err = mac_dev->set_exception(get_mac_handle(mac_dev),
					     FM_MAC_EX_10G_TX_ECC_ER, false);
		if (err < 0)
			goto _return_fm_mac_free;
	}

	err = tgec_get_version(priv->fm_mac, &version);
	if (err < 0)
		goto _return_fm_mac_free;

	dev_info(mac_dev->dev, "FMan %s version: 0x%08x\n",
		 ((macdev2enetinterface(mac_dev) != ENET_MODE_XGMII_10000) ?
		  "dTSEC" : "XGEC"), version);

	goto _return;

_return_fm_mac_free:
	tgec_free(get_mac_handle(mac_dev));

_return:
	return err;
}

static int dtsec_initialization(struct mac_device *mac_dev)
{
	int					err;
	struct mac_priv_s	*priv;
	struct fm_mac_params_t		param;
	u32			version;

	priv = macdev_priv(mac_dev);

	param.base_addr = (typeof(param.base_addr))
		devm_ioremap(mac_dev->dev, mac_dev->res->start, 0x2000);
	param.enet_mode	= macdev2enetinterface(mac_dev);
	memcpy(&param.addr, mac_dev->addr,
	       min(sizeof(param.addr), sizeof(mac_dev->addr)));
	param.mac_id		= mac_dev->cell_index;
	param.fm		= (void *)mac_dev->fm;
	param.exception_cb	= mac_exception;
	param.event_cb		= mac_exception;
	param.dev_id		= mac_dev;

	priv->fm_mac = dtsec_config(&param);
	if (!priv->fm_mac) {
		err = -EINVAL;
		goto _return;
	}

	err = dtsec_cfg_max_frame_len(priv->fm_mac, fm_get_max_frm());
	if (err < 0)
		goto _return_fm_mac_free;

	err = dtsec_cfg_pad_and_crc(priv->fm_mac, true);
	if (err < 0)
		goto _return_fm_mac_free;

	err = dtsec_init(priv->fm_mac);
	if (err < 0)
		goto _return_fm_mac_free;

	/* For 1G MAC, disable by default the MIB counters overflow interrupt */
	if (macdev2enetinterface(mac_dev) != ENET_MODE_XGMII_10000) {
		err = mac_dev->set_exception(get_mac_handle(mac_dev),
					     FM_MAC_EX_1G_RX_MIB_CNT_OVFL,
					     false);
		if (err < 0)
			goto _return_fm_mac_free;
	}

	/* For 10G MAC, disable Tx ECC exception */
	if (macdev2enetinterface(mac_dev) == ENET_MODE_XGMII_10000) {
		err = mac_dev->set_exception(get_mac_handle(mac_dev),
					     FM_MAC_EX_10G_TX_ECC_ER, false);
		if (err < 0)
			goto _return_fm_mac_free;
	}

	err = dtsec_get_version(priv->fm_mac, &version);
	if (err < 0)
		goto _return_fm_mac_free;

	dev_info(mac_dev->dev, "FMan %s version: 0x%08x\n",
		 ((macdev2enetinterface(mac_dev) != ENET_MODE_XGMII_10000) ?
		  "dTSEC" : "XGEC"), version);

	goto _return;

_return_fm_mac_free:
	dtsec_free(get_mac_handle(mac_dev));

_return:
	return err;
}

static int memac_initialization(struct mac_device *mac_dev)
{
	int			 err;
	struct mac_priv_s	*priv;
	struct fm_mac_params_t	 param;

	priv = macdev_priv(mac_dev);

	param.base_addr = (typeof(param.base_addr))
		devm_ioremap(mac_dev->dev, mac_dev->res->start, 0x2000);
	param.enet_mode	= macdev2enetinterface(mac_dev);
	memcpy(&param.addr, mac_dev->addr, sizeof(mac_dev->addr));
	param.mac_id		= mac_dev->cell_index;
	param.fm		= (void *)mac_dev->fm;
	param.exception_cb	= mac_exception;
	param.event_cb		= mac_exception;
	param.dev_id		= mac_dev;

	priv->fm_mac = memac_config(&param);
	if (!priv->fm_mac) {
		err = -EINVAL;
		goto _return;
	}

	err = memac_cfg_max_frame_len(priv->fm_mac, fm_get_max_frm());
	if (err < 0)
		goto _return_fm_mac_free;

	err = memac_cfg_reset_on_init(priv->fm_mac, true);
	if (err < 0)
		goto _return_fm_mac_free;

	err = memac_cfg_fixed_link(priv->fm_mac, mac_dev->fixed_link);
	if (err < 0)
		goto _return_fm_mac_free;

	err = memac_init(priv->fm_mac);
	if (err < 0)
		goto _return_fm_mac_free;

	dev_info(mac_dev->dev, "FMan MEMAC\n");

	goto _return;

_return_fm_mac_free:
	memac_free(priv->fm_mac);

_return:
	return err;
}

static int dtsec_start(struct mac_device *mac_dev)
{
	int	 err;
	struct phy_device *phy_dev = mac_dev->phy_dev;

	err = dtsec_enable(get_mac_handle(mac_dev),
			   COMM_MODE_RX_AND_TX);

	if (!err && phy_dev)
		phy_start(phy_dev);

	return err;
}

static int tgec_start(struct mac_device *mac_dev)
{
	int	 err;
	struct phy_device *phy_dev = mac_dev->phy_dev;

	err = tgec_enable(get_mac_handle(mac_dev),
			  COMM_MODE_RX_AND_TX);

	if (!err && phy_dev)
		phy_start(phy_dev);

	return err;
}

static int memac_start(struct mac_device *mac_dev)
{
	int	 err;
	struct phy_device *phy_dev = mac_dev->phy_dev;

	err = memac_enable(get_mac_handle(mac_dev),
			   COMM_MODE_RX_AND_TX);

	if (!err && phy_dev)
		phy_start(phy_dev);

	return err;
}

static int dtsec_stop(struct mac_device *mac_dev)
{
	if (mac_dev->phy_dev)
		phy_stop(mac_dev->phy_dev);

	return dtsec_disable(get_mac_handle(mac_dev),
			      COMM_MODE_RX_AND_TX);
}

static int tgec_stop(struct mac_device *mac_dev)
{
	return tgec_disable(get_mac_handle(mac_dev),
			      COMM_MODE_RX_AND_TX);
}

static int memac_stop(struct mac_device *mac_dev)
{
	if (mac_dev->phy_dev && (macdev2enetinterface(mac_dev) !=
				 ENET_MODE_XGMII_10000))
		phy_stop(mac_dev->phy_dev);

	return memac_disable(get_mac_handle(mac_dev),
			      COMM_MODE_RX_AND_TX);
}

static int set_multi(struct net_device *net_dev, struct mac_device *mac_dev)
{
	struct mac_priv_s	*mac_priv;
	struct mac_address	*old_addr, *tmp;
	struct netdev_hw_addr	*ha;
	int			err;
	enet_addr_t		*addr;

	mac_priv = macdev_priv(mac_dev);

	/* Clear previous address list */
	list_for_each_entry_safe(old_addr, tmp, &mac_dev->mc_addr_list, list) {
		addr = (enet_addr_t *)old_addr->addr;
		err = mac_dev->remove_hash_mac_addr(mac_priv->fm_mac, addr);
		if (err < 0)
			return err;

		list_del(&old_addr->list);
		kfree(old_addr);
	}

	/* Add all the addresses from the new list */
	netdev_for_each_mc_addr(ha, net_dev) {
		addr = (enet_addr_t *)ha->addr;
		err = mac_dev->add_hash_mac_addr(mac_priv->fm_mac, addr);
		if (err < 0)
			return err;

		tmp = kmalloc(sizeof(*tmp), GFP_ATOMIC);
		if (!tmp)
			return -ENOMEM;

		ether_addr_copy(tmp->addr, ha->addr);
		list_add(&tmp->list, &mac_dev->mc_addr_list);
	}
	return 0;
}

/* Avoid redundant calls to FMD, if the MAC driver already contains the desired
 * active PAUSE settings. Otherwise, the new active settings should be reflected
 * in FMan.
 */
int set_mac_active_pause(struct mac_device *mac_dev, bool rx, bool tx)
{
	struct fm_mac_dev *fm_mac_dev = get_mac_handle(mac_dev);
	int err = 0;

	if (rx != mac_dev->rx_pause_active) {
		err = mac_dev->set_rx_pause(fm_mac_dev, rx);
		if (likely(err == 0))
			mac_dev->rx_pause_active = rx;
	}

	if (tx != mac_dev->tx_pause_active) {
		u16 pause_time = (tx ? FSL_FM_PAUSE_TIME_ENABLE :
					 FSL_FM_PAUSE_TIME_DISABLE);

		err = mac_dev->set_tx_pause(fm_mac_dev, 0,
						    pause_time, 0);

		if (likely(err == 0))
			mac_dev->tx_pause_active = tx;
	}

	return err;
}
EXPORT_SYMBOL(set_mac_active_pause);

/* Determine the MAC RX/TX PAUSE frames settings based on PHY
 * autonegotiation or values set by eththool.
 */
void get_pause_cfg(struct mac_device *mac_dev, bool *rx_pause, bool *tx_pause)
{
	struct phy_device *phy_dev = mac_dev->phy_dev;
	u16 lcl_adv, rmt_adv;
	u8 flowctrl;

	*rx_pause = *tx_pause = false;

	if (!phy_dev->duplex)
		return;

	/* If PAUSE autonegotiation is disabled, the TX/RX PAUSE settings
	 * are those set by ethtool.
	 */
	if (!mac_dev->autoneg_pause) {
		*rx_pause = mac_dev->rx_pause_req;
		*tx_pause = mac_dev->tx_pause_req;
		return;
	}

	/* Else if PAUSE autonegotiation is enabled, the TX/RX PAUSE
	 * settings depend on the result of the link negotiation.
	 */

	/* get local capabilities */
	lcl_adv = 0;
	if (phy_dev->advertising & ADVERTISED_Pause)
		lcl_adv |= ADVERTISE_PAUSE_CAP;
	if (phy_dev->advertising & ADVERTISED_Asym_Pause)
		lcl_adv |= ADVERTISE_PAUSE_ASYM;

	/* get link partner capabilities */
	rmt_adv = 0;
	if (phy_dev->pause)
		rmt_adv |= LPA_PAUSE_CAP;
	if (phy_dev->asym_pause)
		rmt_adv |= LPA_PAUSE_ASYM;

	/* Calculate TX/RX settings based on local and peer advertised
	 * symmetric/asymmetric PAUSE capabilities.
	 */
	flowctrl = mii_resolve_flowctrl_fdx(lcl_adv, rmt_adv);
	if (flowctrl & FLOW_CTRL_RX)
		*rx_pause = true;
	if (flowctrl & FLOW_CTRL_TX)
		*tx_pause = true;
}
EXPORT_SYMBOL(get_pause_cfg);

static void adjust_link_void(struct net_device *net_dev)
{
}

static void adjust_link_dtsec(struct net_device *net_dev)
{
	struct device *dev = net_dev->dev.parent;
	struct dpaa_eth_data *eth_data = dev->platform_data;
	struct mac_device *mac_dev = eth_data->mac_dev;
	struct phy_device *phy_dev = mac_dev->phy_dev;
	struct fm_mac_dev *fm_mac_dev;
	bool rx_pause, tx_pause;
	int err;

	fm_mac_dev = get_mac_handle(mac_dev);
	if (!phy_dev->link) {
		/* TODO: move in dtsec_an_errata() */
		dtsec_restart_autoneg(fm_mac_dev);

		return;
	}

	dtsec_adjust_link(fm_mac_dev, phy_dev->speed);
	get_pause_cfg(mac_dev, &rx_pause, &tx_pause);
	err = set_mac_active_pause(mac_dev, rx_pause, tx_pause);
	if (err < 0)
		netdev_err(net_dev, "set_mac_active_pause() = %d\n", err);
}

static void adjust_link_memac(struct net_device *net_dev)
{
	struct device *dev = net_dev->dev.parent;
	struct dpaa_eth_data *eth_data = dev->platform_data;
	struct mac_device *mac_dev = eth_data->mac_dev;
	struct phy_device *phy_dev = mac_dev->phy_dev;
	struct fm_mac_dev *fm_mac_dev;
	bool rx_pause, tx_pause;
	int err;

	fm_mac_dev = get_mac_handle(mac_dev);
	memac_adjust_link(fm_mac_dev, phy_dev->speed);

	get_pause_cfg(mac_dev, &rx_pause, &tx_pause);
	err = set_mac_active_pause(mac_dev, rx_pause, tx_pause);
	if (err < 0)
		netdev_err(net_dev, "set_mac_active_pause() = %d\n", err);
}

/* Initializes driver's PHY state, and attaches to the PHY.
 * Returns 0 on success.
 */
static int dtsec_init_phy(struct net_device *net_dev,
			  struct mac_device *mac_dev)
{
	struct phy_device	*phy_dev;

	phy_dev = of_phy_connect(net_dev, mac_dev->phy_node,
				 &adjust_link_dtsec, 0,
				 mac_dev->phy_if);
	if (!phy_dev) {
		netdev_err(net_dev, "Could not connect to PHY %s\n",
			   mac_dev->phy_node->full_name);
		return -ENODEV;
	}

	/* Remove any features not supported by the controller */
	phy_dev->supported &= mac_dev->if_support;
	/* Enable the symmetric and asymmetric PAUSE frame advertisements,
	 * as most of the PHY drivers do not enable them by default.
	 */
	phy_dev->supported |= (SUPPORTED_Pause | SUPPORTED_Asym_Pause);
	phy_dev->advertising = phy_dev->supported;

	mac_dev->phy_dev = phy_dev;

	return 0;
}

static int xgmac_init_phy(struct net_device *net_dev,
			  struct mac_device *mac_dev)
{
	struct phy_device *phy_dev;

	phy_dev = of_phy_connect(net_dev, mac_dev->phy_node, &adjust_link_void,
				 0, mac_dev->phy_if);
	if (!phy_dev) {
		netdev_err(net_dev, "Could not attach to PHY %s\n",
			   mac_dev->phy_node->full_name);
		return -ENODEV;
	}

	phy_dev->supported &= mac_dev->if_support;
	/* Enable the symmetric and asymmetric PAUSE frame advertisements,
	 * as most of the PHY drivers do not enable them by default.
	 */
	phy_dev->supported |= (SUPPORTED_Pause | SUPPORTED_Asym_Pause);
	phy_dev->advertising = phy_dev->supported;

	mac_dev->phy_dev = phy_dev;

	return 0;
}

static int memac_init_phy(struct net_device *net_dev,
			  struct mac_device *mac_dev)
{
	struct phy_device       *phy_dev;

	phy_dev = of_phy_connect(net_dev, mac_dev->phy_node,
				 &adjust_link_memac, 0,
				 mac_dev->phy_if);

	if (!phy_dev) {
		netdev_err(net_dev, "Could not connect to PHY %s\n",
			   mac_dev->phy_node->full_name);
		return -ENODEV;
	}

	/* Remove any features not supported by the controller */
	phy_dev->supported &= mac_dev->if_support;
	/* Enable the symmetric and asymmetric PAUSE frame advertisements,
	 * as most of the PHY drivers do not enable them by default.
	 */
	phy_dev->supported |= (SUPPORTED_Pause | SUPPORTED_Asym_Pause);
	phy_dev->advertising = phy_dev->supported;

	mac_dev->phy_dev = phy_dev;

	return 0;
}

static void setup_dtsec(struct mac_device *mac_dev)
{
	mac_dev->init_phy	= dtsec_init_phy;
	mac_dev->init		= dtsec_initialization;
	mac_dev->start		= dtsec_start;
	mac_dev->stop		= dtsec_stop;
	mac_dev->set_promisc	= dtsec_set_promiscuous;
	mac_dev->change_addr    = dtsec_modify_mac_address;
	mac_dev->add_hash_mac_addr	= dtsec_add_hash_mac_address;
	mac_dev->remove_hash_mac_addr	= dtsec_del_hash_mac_address;
	mac_dev->set_multi      = set_multi;
	mac_dev->set_tx_pause	= dtsec_set_tx_pause_frames;
	mac_dev->set_rx_pause	= dtsec_accept_rx_pause_frames;
	mac_dev->set_exception	= dtsec_set_exception;
}

static void setup_xgmac(struct mac_device *mac_dev)
{
	mac_dev->init_phy	= xgmac_init_phy;
	mac_dev->init		= tgec_initialization;
	mac_dev->start		= tgec_start;
	mac_dev->stop		= tgec_stop;
	mac_dev->set_promisc	= tgec_set_promiscuous;
	mac_dev->change_addr    = tgec_modify_mac_address;
	mac_dev->add_hash_mac_addr	= tgec_add_hash_mac_address;
	mac_dev->remove_hash_mac_addr	= tgec_del_hash_mac_address;
	mac_dev->set_multi      = set_multi;
	mac_dev->set_tx_pause	= tgec_set_tx_pause_frames;
	mac_dev->set_rx_pause	= tgec_accept_rx_pause_frames;
	mac_dev->set_exception  = tgec_set_exception;
}

static void setup_memac(struct mac_device *mac_dev)
{
	mac_dev->init_phy	= memac_init_phy;
	mac_dev->init		= memac_initialization;
	mac_dev->start		= memac_start;
	mac_dev->stop		= memac_stop;
	mac_dev->set_promisc	= memac_set_promiscuous;
	mac_dev->change_addr    = memac_modify_mac_address;
	mac_dev->add_hash_mac_addr	= memac_add_hash_mac_address;
	mac_dev->remove_hash_mac_addr	= memac_del_hash_mac_address;
	mac_dev->set_multi      = set_multi;
	mac_dev->set_tx_pause	= memac_set_tx_pause_frames;
	mac_dev->set_rx_pause	= memac_accept_rx_pause_frames;
	mac_dev->set_exception  = memac_set_exception;
}

void (*const mac_setup[])(struct mac_device *mac_dev) = {
	[DTSEC] = setup_dtsec,
	[XGMAC] = setup_xgmac,
	[MEMAC] = setup_memac
};
