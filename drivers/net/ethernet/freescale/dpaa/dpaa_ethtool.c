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

#include <linux/string.h>

#include "dpaa_eth.h"
#include "mac.h"
#include "dpaa_eth_common.h"

static int dpa_get_settings(struct net_device *net_dev,
			    struct ethtool_cmd *et_cmd)
{
	int err;
	struct dpa_priv_s *priv;

	priv = netdev_priv(net_dev);

	if (!priv->mac_dev->phy_dev) {
		netdev_dbg(net_dev, "phy device not initialized\n");
		return 0;
	}

	err = phy_ethtool_gset(priv->mac_dev->phy_dev, et_cmd);

	return err;
}

static int dpa_set_settings(struct net_device *net_dev,
			    struct ethtool_cmd *et_cmd)
{
	int err;
	struct dpa_priv_s *priv;

	priv = netdev_priv(net_dev);

	if (!priv->mac_dev->phy_dev) {
		netdev_err(net_dev, "phy device not initialized\n");
		return -ENODEV;
	}

	err = phy_ethtool_sset(priv->mac_dev->phy_dev, et_cmd);
	if (err < 0)
		netdev_err(net_dev, "phy_ethtool_sset() = %d\n", err);

	return err;
}

static void dpa_get_drvinfo(struct net_device *net_dev,
			    struct ethtool_drvinfo *drvinfo)
{
	int len;

	strlcpy(drvinfo->driver, KBUILD_MODNAME,
		sizeof(drvinfo->driver));
	len = snprintf(drvinfo->version, sizeof(drvinfo->version),
		       "%X", 0);
	len = snprintf(drvinfo->fw_version, sizeof(drvinfo->fw_version),
		       "%X", 0);

	if (len >= sizeof(drvinfo->fw_version)) {
		/* Truncated output */
		netdev_notice(net_dev, "snprintf() = %d\n", len);
	}
	strlcpy(drvinfo->bus_info, dev_name(net_dev->dev.parent->parent),
		sizeof(drvinfo->bus_info));
}

static u32 dpa_get_msglevel(struct net_device *net_dev)
{
	return ((struct dpa_priv_s *)netdev_priv(net_dev))->msg_enable;
}

static void dpa_set_msglevel(struct net_device *net_dev,
			     u32 msg_enable)
{
	((struct dpa_priv_s *)netdev_priv(net_dev))->msg_enable = msg_enable;
}

static int dpa_nway_reset(struct net_device *net_dev)
{
	int err;
	struct dpa_priv_s *priv;

	priv = netdev_priv(net_dev);

	if (!priv->mac_dev->phy_dev) {
		netdev_err(net_dev, "phy device not initialized\n");
		return -ENODEV;
	}

	err = 0;
	if (priv->mac_dev->phy_dev->autoneg) {
		err = phy_start_aneg(priv->mac_dev->phy_dev);
		if (err < 0)
			netdev_err(net_dev, "phy_start_aneg() = %d\n",
				   err);
	}

	return err;
}

static void dpa_get_pauseparam(struct net_device *net_dev,
			       struct ethtool_pauseparam *epause)
{
	struct dpa_priv_s *priv;
	struct mac_device *mac_dev;
	struct phy_device *phy_dev;

	priv = netdev_priv(net_dev);
	mac_dev = priv->mac_dev;

	phy_dev = mac_dev->phy_dev;
	if (!phy_dev) {
		netdev_err(net_dev, "phy device not initialized\n");
		return;
	}

	epause->autoneg = mac_dev->autoneg_pause;
	epause->rx_pause = mac_dev->rx_pause_active;
	epause->tx_pause = mac_dev->tx_pause_active;
}

static int dpa_set_pauseparam(struct net_device *net_dev,
			      struct ethtool_pauseparam *epause)
{
	struct dpa_priv_s *priv;
	struct mac_device *mac_dev;
	struct phy_device *phy_dev;
	int err;
	u32 newadv, oldadv;
	bool rx_pause, tx_pause;

	priv = netdev_priv(net_dev);
	mac_dev = priv->mac_dev;

	phy_dev = mac_dev->phy_dev;
	if (!phy_dev) {
		netdev_err(net_dev, "phy device not initialized\n");
		return -ENODEV;
	}

	if (!(phy_dev->supported & SUPPORTED_Pause) ||
	    (!(phy_dev->supported & SUPPORTED_Asym_Pause) &&
	    (epause->rx_pause != epause->tx_pause)))
		return -EINVAL;

	/* The MAC should know how to handle PAUSE frame autonegotiation before
	 * adjust_link is triggered by a forced renegotiation of sym/asym PAUSE
	 * settings.
	 */
	mac_dev->autoneg_pause = !!epause->autoneg;
	mac_dev->rx_pause_req = !!epause->rx_pause;
	mac_dev->tx_pause_req = !!epause->tx_pause;

	/* Determine the sym/asym advertised PAUSE capabilities from the desired
	 * rx/tx pause settings.
	 */
	newadv = 0;
	if (epause->rx_pause)
		newadv = ADVERTISED_Pause | ADVERTISED_Asym_Pause;
	if (epause->tx_pause)
		newadv |= ADVERTISED_Asym_Pause;

	oldadv = phy_dev->advertising &
			(ADVERTISED_Pause | ADVERTISED_Asym_Pause);

	/* If there are differences between the old and the new advertised
	 * values, restart PHY autonegotiation and advertise the new values.
	 */
	if (oldadv != newadv) {
		phy_dev->advertising &= ~(ADVERTISED_Pause
				| ADVERTISED_Asym_Pause);
		phy_dev->advertising |= newadv;
		if (phy_dev->autoneg) {
			err = phy_start_aneg(phy_dev);
			if (err < 0)
				netdev_err(net_dev, "phy_start_aneg() = %d\n",
					   err);
		}
	}

	get_pause_cfg(mac_dev, &rx_pause, &tx_pause);
	err = set_mac_active_pause(mac_dev, rx_pause, tx_pause);
	if (err < 0)
		netdev_err(net_dev, "set_mac_active_pause() = %d\n", err);

	return err;
}

const struct ethtool_ops dpa_ethtool_ops = {
	.get_settings = dpa_get_settings,
	.set_settings = dpa_set_settings,
	.get_drvinfo = dpa_get_drvinfo,
	.get_msglevel = dpa_get_msglevel,
	.set_msglevel = dpa_set_msglevel,
	.nway_reset = dpa_nway_reset,
	.get_pauseparam = dpa_get_pauseparam,
	.set_pauseparam = dpa_set_pauseparam,
	.get_link = ethtool_op_get_link,
};
