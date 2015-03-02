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

#ifndef __MAC_H
#define __MAC_H

#include <linux/device.h>	/* struct device, BUS_ID_SIZE */
#include <linux/if_ether.h>	/* ETH_ALEN */
#include <linux/phy.h>		/* phy_interface_t, struct phy_device */
#include <linux/list.h>

#include "enet_ext.h"

#include "fsl_fman_drv.h"	/* struct port_device */
#include "fm_port_ext.h"

struct fm_mac_dev;
enum fm_mac_exceptions;

enum {DTSEC, XGMAC, MEMAC};

struct mac_device {
	struct device		*dev;
	void			*priv;
	u8			 cell_index;
	struct resource		*res;
	void __iomem		*vaddr;
	u8			 addr[ETH_ALEN];
	bool			 promisc;

	struct fm		*fm_dev;
	struct fm_port_drv_t	*port_dev[2];

	phy_interface_t		 phy_if;
	u32			 if_support;
	bool			 link;
	bool			 fixed_link;
	u16		 speed;
	u16		 max_speed;
	struct device_node	*phy_node;
	struct device_node	*tbi_node;
	struct phy_device	*phy_dev;
	void			*fm;
	/* List of multicast addresses */
	struct list_head	 mc_addr_list;
	struct platform_device	*eth_dev;

	bool autoneg_pause;
	bool rx_pause_req;
	bool tx_pause_req;
	bool rx_pause_active;
	bool tx_pause_active;

	int (*init_phy)(struct net_device *net_dev, struct mac_device *mac_dev);
	int (*init)(struct mac_device *mac_dev);
	int (*start)(struct mac_device *mac_dev);
	int (*stop)(struct mac_device *mac_dev);
	int (*set_promisc)(struct fm_mac_dev *fm_mac_dev, bool enable);
	int (*change_addr)(struct fm_mac_dev *fm_mac_dev,
			   enet_addr_t *p_enet_addr);
	int (*set_multi)(struct net_device *net_dev,
			 struct mac_device *mac_dev);
	int (*set_rx_pause)(struct fm_mac_dev *fm_mac_dev, bool en);
	int (*set_tx_pause)(struct fm_mac_dev *fm_mac_dev, u8 priority,
			    u16 pause_time, u16 thresh_time);
	int (*set_exception)(struct fm_mac_dev *fm_mac_dev,
			     enum fm_mac_exceptions exception,
			     bool enable);
	int (*add_hash_mac_addr)(struct fm_mac_dev *fm_mac_dev,
				 enet_addr_t *p_eth_addr);
	int (*remove_hash_mac_addr)(struct fm_mac_dev *fm_mac_dev,
				    enet_addr_t *p_eth_addr);
};

struct mac_address {
	u8 addr[ETH_ALEN];
	struct list_head list;
};

struct dpaa_eth_data {
	struct device_node *mac_node;
	struct mac_device *mac_dev;
	int mac_hw_id;
	int fman_hw_id;
};

#define get_fm_handle(net_dev) \
	(((struct dpa_priv_s *)netdev_priv(net_dev))->mac_dev->fm_dev)

#define for_each_port_device(i, port_dev)	\
	for (i = 0; i < ARRAY_SIZE(port_dev); i++)

static inline __attribute((nonnull)) void *macdev_priv(
		const struct mac_device *mac_dev)
{
	return (void *)mac_dev + sizeof(*mac_dev);
}

extern const char	*mac_driver_description;
extern const size_t	 mac_sizeof_priv[];
extern void (*const mac_setup[])(struct mac_device *mac_dev);

int set_mac_active_pause(struct mac_device *mac_dev, bool rx, bool tx);
void get_pause_cfg(struct mac_device *mac_dev, bool *rx_pause, bool *tx_pause);
struct fm_mac_dev *get_mac_handle(struct mac_device *mac_dev);

#endif	/* __MAC_H */
