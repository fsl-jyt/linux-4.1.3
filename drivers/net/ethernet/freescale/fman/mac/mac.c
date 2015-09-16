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
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/device.h>
#include <linux/phy.h>

#include "mac.h"

#define DTSEC_SUPPORTED \
	(SUPPORTED_10baseT_Half \
	| SUPPORTED_10baseT_Full \
	| SUPPORTED_100baseT_Half \
	| SUPPORTED_100baseT_Full \
	| SUPPORTED_Autoneg \
	| SUPPORTED_Pause \
	| SUPPORTED_Asym_Pause \
	| SUPPORTED_MII)

static DEFINE_MUTEX(eth_lock);

static const char phy_str[][11] = {
	[PHY_INTERFACE_MODE_MII]	= "mii",
	[PHY_INTERFACE_MODE_GMII]	= "gmii",
	[PHY_INTERFACE_MODE_SGMII]	= "sgmii",
	[PHY_INTERFACE_MODE_TBI]	= "tbi",
	[PHY_INTERFACE_MODE_RMII]	= "rmii",
	[PHY_INTERFACE_MODE_RGMII]	= "rgmii",
	[PHY_INTERFACE_MODE_RGMII_ID]	= "rgmii-id",
	[PHY_INTERFACE_MODE_RGMII_RXID]	= "rgmii-rxid",
	[PHY_INTERFACE_MODE_RGMII_TXID]	= "rgmii-txid",
	[PHY_INTERFACE_MODE_RTBI]	= "rtbi",
	[PHY_INTERFACE_MODE_XGMII]	= "xgmii"
};

static phy_interface_t __pure __attribute__((nonnull)) str2phy(const char *str)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(phy_str); i++)
		if (strcmp(str, phy_str[i]) == 0)
			return (phy_interface_t)i;

	return PHY_INTERFACE_MODE_MII;
}

static const u16 phy2speed[] = {
	[PHY_INTERFACE_MODE_MII]	= SPEED_100,
	[PHY_INTERFACE_MODE_GMII]	= SPEED_1000,
	[PHY_INTERFACE_MODE_SGMII]	= SPEED_1000,
	[PHY_INTERFACE_MODE_TBI]	= SPEED_1000,
	[PHY_INTERFACE_MODE_RMII]	= SPEED_100,
	[PHY_INTERFACE_MODE_RGMII]	= SPEED_1000,
	[PHY_INTERFACE_MODE_RGMII_ID]	= SPEED_1000,
	[PHY_INTERFACE_MODE_RGMII_RXID]	= SPEED_1000,
	[PHY_INTERFACE_MODE_RGMII_TXID]	= SPEED_1000,
	[PHY_INTERFACE_MODE_RTBI]	= SPEED_1000,
	[PHY_INTERFACE_MODE_XGMII]	= SPEED_10000
};

static struct mac_device *
alloc_macdev(struct device *dev, size_t sizeof_priv,
	     void (*setup)(struct mac_device *mac_dev))
{
	struct mac_device	*mac_dev;

	mac_dev = devm_kzalloc(dev, sizeof(*mac_dev) + sizeof_priv, GFP_KERNEL);
	if (!mac_dev) {
		mac_dev = ERR_PTR(-ENOMEM);
	} else {
		mac_dev->dev = dev;
		dev_set_drvdata(dev, mac_dev);
		setup(mac_dev);
	}

	return mac_dev;
}

static struct platform_device *dpaa_eth_add_device(int fman_id,
						   struct mac_device *mac_dev,
						   struct device_node *node)
{
	struct platform_device *pdev;
	struct dpaa_eth_data data;
	static int dpaa_eth_dev_cnt;
	int ret;

	data.mac_dev = mac_dev;
	data.mac_hw_id = mac_dev->cell_index;
	data.fman_hw_id = fman_id;
	data.mac_node = node;

	mutex_lock(&eth_lock);

	pdev = platform_device_alloc("dpaa-ethernet", dpaa_eth_dev_cnt);
	if (!pdev) {
		ret = -ENOMEM;
		goto no_mem;
	}

	ret = platform_device_add_data(pdev, &data, sizeof(data));
	if (ret)
		goto err;

	ret = platform_device_add(pdev);
	if (ret)
		goto err;

	dpaa_eth_dev_cnt++;
	mutex_unlock(&eth_lock);

	return pdev;

err:
	platform_device_put(pdev);
no_mem:
	mutex_unlock(&eth_lock);

	return ERR_PTR(ret);
}

static const struct of_device_id mac_match[] = {
	[DTSEC] = {
		.compatible	= "fsl,fman-dtsec"
	},
	[XGMAC] = {
		.compatible	= "fsl,fman-xgec"
	},
	[MEMAC] = {
		.compatible	= "fsl,fman-memac"
	},
	{}
};
MODULE_DEVICE_TABLE(of, mac_match);

static int mac_probe(struct platform_device *_of_dev)
{
	int			 err, i, lenp;
	struct device		*dev;
	struct device_node	*mac_node, *dev_node;
	struct mac_device	*mac_dev;
	struct platform_device	*of_dev;
	struct resource		 res;
	const u8		*mac_addr;
	const char		*char_prop;
	const u32		*u32_prop;
	const struct of_device_id *match;
	u8			 fman_id;

	const phandle           *phandle_prop;

	dev = &_of_dev->dev;
	mac_node = dev->of_node;

	match = of_match_device(mac_match, dev);
	if (!match)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(mac_match) - 1 && match != mac_match + i;
			i++)
		;
	BUG_ON(i >= ARRAY_SIZE(mac_match) - 1);

	mac_dev = alloc_macdev(dev, mac_sizeof_priv[i], mac_setup[i]);
	if (IS_ERR(mac_dev)) {
		err = PTR_ERR(mac_dev);
		dev_err(dev, "alloc_macdev() = %d\n", err);
		goto _return;
	}

	INIT_LIST_HEAD(&mac_dev->mc_addr_list);

	/* Get the FM node */
	dev_node = of_get_parent(mac_node);
	if (!dev_node) {
		dev_err(dev, "of_get_parent(%s) failed\n",
			mac_node->full_name);
		err = -EINVAL;
		goto _return_dev_set_drvdata;
	}

	of_dev = of_find_device_by_node(dev_node);
	if (!of_dev) {
		dev_err(dev, "of_find_device_by_node(%s) failed\n",
			dev_node->full_name);
		err = -EINVAL;
		goto _return_of_node_put;
	}

	/* Get the FMan cell-index */
	u32_prop = of_get_property(dev_node, "cell-index", &lenp);
	if (!u32_prop) {
		dev_err(dev, "of_get_property(%s, cell-index) failed\n",
			dev_node->full_name);
		err = -EINVAL;
		goto _return_of_node_put;
	}
	BUG_ON(lenp != sizeof(u32));
	fman_id = (u8)*u32_prop + 1; /* cell-index 0 => FMan id 1 */

	mac_dev->fm_dev = fm_bind(&of_dev->dev);
	if (!mac_dev->fm_dev) {
		dev_err(dev, "fm_bind(%s) failed\n", dev_node->full_name);
		err = -ENODEV;
		goto _return_of_node_put;
	}

	mac_dev->fm = (void *)fm_get_handle(mac_dev->fm_dev);
	of_node_put(dev_node);

	/* Get the address of the memory mapped registers */
	err = of_address_to_resource(mac_node, 0, &res);
	if (err < 0) {
		dev_err(dev, "of_address_to_resource(%s) = %d\n",
			mac_node->full_name, err);
		goto _return_dev_set_drvdata;
	}

	mac_dev->res =
		__devm_request_region(dev, fm_get_mem_region(mac_dev->fm_dev),
				      res.start, res.end + 1 - res.start,
				      "mac");
	if (!mac_dev->res) {
		dev_err(dev, "__devm_request_mem_region(mac) failed\n");
		err = -EBUSY;
		goto _return_dev_set_drvdata;
	}

	mac_dev->vaddr = devm_ioremap(dev, mac_dev->res->start,
				      mac_dev->res->end + 1
				      - mac_dev->res->start);
	if (!mac_dev->vaddr) {
		dev_err(dev, "devm_ioremap() failed\n");
		err = -EIO;
		goto _return_dev_set_drvdata;
	}

#define TBIPA_OFFSET		0x1c
#define TBIPA_DEFAULT_ADDR	5 /* override if used as external PHY addr. */
	mac_dev->tbi_node = of_parse_phandle(mac_node, "tbi-handle", 0);
	if (mac_dev->tbi_node) {
		u32 tbiaddr = TBIPA_DEFAULT_ADDR;

		u32_prop = of_get_property(mac_dev->tbi_node, "reg", NULL);
		if (u32_prop)
			tbiaddr = *u32_prop;
		out_be32(mac_dev->vaddr + TBIPA_OFFSET, tbiaddr);
	}

	if (!of_device_is_available(mac_node)) {
		devm_iounmap(dev, mac_dev->vaddr);
		__devm_release_region(dev, fm_get_mem_region(mac_dev->fm_dev),
				      res.start, res.end + 1 - res.start);
		fm_unbind(mac_dev->fm_dev);
		devm_kfree(dev, mac_dev);
		dev_set_drvdata(dev, NULL);
		return -ENODEV;
	}

	/* Get the cell-index */
	u32_prop = of_get_property(mac_node, "cell-index", &lenp);
	if (!u32_prop) {
		dev_err(dev, "of_get_property(%s, cell-index) failed\n",
			mac_node->full_name);
		err = -EINVAL;
		goto _return_dev_set_drvdata;
	}
	BUG_ON(lenp != sizeof(u32));
	mac_dev->cell_index = (u8)*u32_prop;

	/* Get the MAC address */
	mac_addr = of_get_mac_address(mac_node);
	if (!mac_addr) {
		dev_err(dev, "of_get_mac_address(%s) failed\n",
			mac_node->full_name);
		err = -EINVAL;
		goto _return_dev_set_drvdata;
	}
	memcpy(mac_dev->addr, mac_addr, sizeof(mac_dev->addr));

	/* Get the port handles */
	phandle_prop = of_get_property(mac_node, "fsl,fman-ports", &lenp);
	if (!phandle_prop) {
		dev_err(dev, "of_get_property(%s, fsl,fman-ports) failed\n",
			mac_node->full_name);
		err = -EINVAL;
		goto _return_dev_set_drvdata;
	}
	BUG_ON(lenp != sizeof(phandle) * ARRAY_SIZE(mac_dev->port_dev));

	for_each_port_device(i, mac_dev->port_dev) {
		/* Find the port node */
		dev_node = of_find_node_by_phandle(phandle_prop[i]);
		if (!dev_node) {
			dev_err(dev, "of_find_node_by_phandle() failed\n");
			err = -EINVAL;
			goto _return_of_node_put;
		}

		of_dev = of_find_device_by_node(dev_node);
		if (!of_dev) {
			dev_err(dev, "of_find_device_by_node(%s) failed\n",
				dev_node->full_name);
			err = -EINVAL;
			goto _return_of_node_put;
		}

		mac_dev->port_dev[i] = fm_port_bind(&of_dev->dev);
		if (!mac_dev->port_dev[i]) {
			dev_err(dev, "dev_get_drvdata(%s) failed\n",
				dev_node->full_name);
			err = -EINVAL;
			goto _return_of_node_put;
		}
		of_node_put(dev_node);
	}

	/* Get the PHY connection type */
	char_prop = (const char *)of_get_property(mac_node,
						"phy-connection-type", NULL);
	if (!char_prop) {
		dev_warn(dev,
			 "of_get_property(%s, phy-connection-type) failed. Defaulting to MII\n",
			 mac_node->full_name);
		mac_dev->phy_if = PHY_INTERFACE_MODE_MII;
	} else {
		mac_dev->phy_if = str2phy(char_prop);
	}

	mac_dev->link		= false;
	mac_dev->fixed_link	= false;
	mac_dev->speed		= phy2speed[mac_dev->phy_if];
	mac_dev->max_speed	= mac_dev->speed;
	mac_dev->if_support = DTSEC_SUPPORTED;
	/* We don't support half-duplex in SGMII mode */
	if (strstr(char_prop, "sgmii"))
		mac_dev->if_support &= ~(SUPPORTED_10baseT_Half |
					SUPPORTED_100baseT_Half);

	/* Gigabit support (no half-duplex) */
	if (mac_dev->max_speed == 1000)
		mac_dev->if_support |= SUPPORTED_1000baseT_Full;

	/* The 10G interface only supports one mode */
	if (strstr(char_prop, "xgmii"))
		mac_dev->if_support = SUPPORTED_10000baseT_Full;

	/* Get the rest of the PHY information */
	mac_dev->phy_node = of_parse_phandle(mac_node, "phy-handle", 0);
	if (!mac_dev->phy_node && of_phy_is_fixed_link(mac_node)) {
		err = of_phy_register_fixed_link(mac_node);
		if (err)
			goto _return_dev_set_drvdata;
		mac_dev->fixed_link = true;
		mac_dev->phy_node = of_node_get(mac_node);
	}

	err = mac_dev->init(mac_dev);
	if (err < 0) {
		dev_err(dev, "mac_dev->init() = %d\n", err);
		of_node_put(mac_dev->phy_node);
		goto _return_dev_set_drvdata;
	}

	/* pause frame autonegotiation enabled */
	mac_dev->autoneg_pause = true;

	/* by intializing the values to false, force FMD to enable PAUSE frames
	 * on RX and TX
	 */
	mac_dev->rx_pause_req = true;
	mac_dev->tx_pause_req = true;
	mac_dev->rx_pause_active = false;
	mac_dev->tx_pause_active = false;
	err = set_mac_active_pause(mac_dev, true, true);
	if (err < 0)
		dev_err(dev, "set_mac_active_pause() = %d\n", err);

	dev_info(dev,
		 "FMan MAC address: %02hx:%02hx:%02hx:%02hx:%02hx:%02hx\n",
		     mac_dev->addr[0], mac_dev->addr[1], mac_dev->addr[2],
		     mac_dev->addr[3], mac_dev->addr[4], mac_dev->addr[5]);

	mac_dev->eth_dev = dpaa_eth_add_device(fman_id, mac_dev,
					       mac_node);
	if (IS_ERR(mac_dev->eth_dev)) {
		dev_err(dev, "failed to add Ethernet platform device for MAC %d\n",
			mac_dev->cell_index);
		mac_dev->eth_dev = NULL;
	}

	goto _return;

_return_of_node_put:
	of_node_put(dev_node);
_return_dev_set_drvdata:
	dev_set_drvdata(dev, NULL);
_return:
	return err;
}

static struct platform_driver mac_driver = {
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= mac_match,
	},
	.probe		= mac_probe,
};

module_platform_driver(mac_driver);
