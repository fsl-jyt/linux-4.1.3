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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/io.h>

#include "fm_common.h"
#include "fsl_fman_drv.h"
#include "fm_port_ext.h"
#include "fm_drv.h"

static struct fm_port_drv_t
*read_fm_port_dev_tree_node(struct platform_device *of_dev)
{
	struct fm_drv_t *fm_drv;
	struct fm_port_drv_t *port;
	struct device_node *fm_node, *port_node;
	struct resource res;
	const u32 *u32_prop;
	int err = 0, lenp;
	enum fm_port_type port_type;
	enum fm_port_speed port_speed = FM_PORT_SPEED_DUMMY;
	u8 cell_index;

	port_node = of_node_get(of_dev->dev.of_node);

	/* Get the FM node */
	fm_node = of_get_parent(port_node);
	if (!fm_node) {
		pr_err("of_get_parent() = %d\n", err);
		return NULL;
	}

	fm_drv = dev_get_drvdata(&of_find_device_by_node(fm_node)->dev);
	of_node_put(fm_node);

	/* if fm_probe() failed, no point in going further with port probing */
	if (!fm_drv)
		return NULL;

	u32_prop = (u32 *)of_get_property(port_node, "cell-index", &lenp);
	if (!u32_prop) {
		pr_err("of_get_property(%s, cell-index) failed\n",
		       port_node->full_name);
		return NULL;
	}
	if (WARN_ON(lenp != sizeof(u32)))
		return NULL;
	cell_index = (u8)*u32_prop;

	port = &fm_drv->ports[cell_index];
	port->id = cell_index;
	port->port_params.port_id = port->id;

	if (of_device_is_compatible(port_node, "fsl,fman-v3-port-tx")) {
		port_type = FM_PORT_TYPE_TX;
		port_speed = FM_PORT_SPEED_1G;
		u32_prop = (u32 *)of_get_property(port_node,
						  "fsl,fman-10g-port", &lenp);
		if (u32_prop)
			port_speed = FM_PORT_SPEED_10G;

	} else if (of_device_is_compatible(port_node, "fsl,fman-v2-port-tx")) {
		if (cell_index >= TX_10G_PORT_BASE)
			port_speed = FM_PORT_SPEED_10G;
		else
			port_speed = FM_PORT_SPEED_1G;
		port_type = FM_PORT_TYPE_TX;

	} else if (of_device_is_compatible(port_node, "fsl,fman-v3-port-rx")) {
		port_type = FM_PORT_TYPE_RX;
		port_speed = FM_PORT_SPEED_1G;
		u32_prop = (u32 *)of_get_property(port_node,
						  "fsl,fman-10g-port", &lenp);
		if (u32_prop)
			port_speed = FM_PORT_SPEED_10G;

	} else if (of_device_is_compatible(port_node, "fsl,fman-v2-port-rx")) {
		if (cell_index >= RX_10G_PORT_BASE)
			port_speed = FM_PORT_SPEED_10G;
		else
			port_speed = FM_PORT_SPEED_1G;
		port_type = FM_PORT_TYPE_RX;

	}  else {
		pr_err("Illegal port type\n");
		return NULL;
	}

	port->port_params.port_type = port_type;
	port->port_params.port_speed = port_speed;

	if (port_type == FM_PORT_TYPE_TX) {
		u32 qman_channel_id;

		qman_channel_id = get_qman_channel_id(fm_drv, cell_index);

		if (qman_channel_id == 0) {
			pr_err("incorrect qman-channel-id\n");
			return NULL;
		}
		port->tx_ch = qman_channel_id;
		port->port_params.specific_params.non_rx_params.qm_channel =
			qman_channel_id;
	}

	err = of_address_to_resource(port_node, 0, &res);
	if (err < 0) {
		pr_err("of_address_to_resource() = %d\n", err);
		return NULL;
	}

	port->dev = &of_dev->dev;
	port->base_addr = 0;
	port->phys_base_addr = res.start;
	port->mem_size = res.end + 1 - res.start;
	port->port_params.fm = fm_drv->fm_dev;
	port->fm = (void *)fm_drv;

	of_node_put(port_node);

	port->active = true;

	return port;
}

static int configure_fm_port_dev(struct fm_port_drv_t *port)
{
	struct fm_drv_t *fm_drv = (struct fm_drv_t *)port->fm;
	struct resource *dev_res;

	if (!port->active) {
		pr_err("FM port not configured!!!\n");
		return -EINVAL;
	}

	dev_res = __devm_request_region(fm_drv->dev, fm_drv->res,
					port->phys_base_addr,
					port->mem_size,
					"fman-port-hc");
	if (!dev_res) {
		pr_err("__devm_request_region() failed\n");
		return -EINVAL;
	}
	port->base_addr = devm_ioremap(fm_drv->dev, port->phys_base_addr,
				       port->mem_size);
	if (port->base_addr == 0)
		pr_err("devm_ioremap() failed\n");

	port->port_params.base_addr = port->base_addr;

	return 0;
}

static int init_fm_port_dev(struct fm_port_drv_t *port)
{
	struct fm_drv_t *fm_drv = (struct fm_drv_t *)port->fm;

	if (!port->active || port->fm_port)
		return -EINVAL;

	port->fm_port = fm_port_config(&port->port_params);
	if (!port->fm_port) {
		pr_err("fm_port_config() failed\n");
		return -EINVAL;
	}

	fm_get_revision(fm_drv->fm_dev, &port->fm_rev_info);

	/* FM_QMI_NO_DEQ_OPTIONS_SUPPORT Errata workaround */
	if (port->port_params.port_type == FM_PORT_TYPE_TX) {
		int err = 0;

		err = fm_port_cfg_deq_high_priority(port->fm_port, true);
		if (err)
			return err;
		err =
		    fm_port_cfg_deq_prefetch_option(port->fm_port,
						    FM_PORT_DEQ_FULL_PREFETCH);
		if (err)
			return err;
	}

	/* Configure BCB workaround on Rx ports, only for B4860 rev1 */
	if ((port->fm_rev_info.major_rev >= 6) &&
	    (port->port_params.port_type == FM_PORT_TYPE_RX)) {
		unsigned int svr;

		svr = mfspr(SPRN_SVR);
		if ((SVR_SOC_VER(svr) == SVR_B4860) && (SVR_MAJ(svr) == 1))
			fm_port_cfg_bcb_wa(port->fm_port);
	}

	fm_port_cfg_buf_prefix_content(port->fm_port,
				       &port->buff_prefix_content);

	if (fm_port_init(port->fm_port) != 0) {
		pr_err("fm_port_init() failed\n");
		return -EINVAL;
	}

/* FMan Fifo sizes "behind the scene":
 * Using the following formulae (*), under a set of simplifying assumptions (.):
 * . all ports are configured in Normal Mode (rather than Independent Mode)
 * . the DPAA Eth driver allocates buffers of size:
 *     . MAXFRM + NET_IP_ALIGN + DPA_PRIV_DATA_SIZE + DPA_PARSE_RESULTS_SIZE
 *		 + DPA_HASH_RESULTS_SIZE, i.e.:
 *       MAXFRM + 2 + 16 + sizeof(fm_prs_result_t) + 16, i.e.:
 *       MAXFRM + 66
 * . excessive buffer pools not accounted for
 *
 * for Rx ports on P4080:
 *     . IFSZ = ceil(max(FMBM_EBMPI[PBS]) / 256)*256 + 7*256
 *     . no internal frame offset (FMBM_RIM[FOF] == 0) - otherwise,
 *     add up to 256 to the above
 *
 * for Tx ports:
 *     . IFSZ = ceil(frame_size / 256)*256 + 3*256
 *			+ FMBM_TFP[DPDE]*256, i.e.:
 *       IFSZ = ceil(MAXFRM / 256)*256 + 3 x 256 + FMBM_TFP[DPDE]*256
 *
 * for P4080:
 *     . (conservative decisions, assuming that BMI must bring the entire
 *     frame, not only the frame header)
 *     . no internal frame offset (FMBM_OIM[FOF] == 0) - otherwise,
 *     add up to 256 to the above
 *
 * . for P4080/P5020/P3041/P2040, DPDE is:
 *             > 0 or 1, for 1Gb ports, HW default: 0
 *             > 2..7 (recommended: 3..7) for 10Gb ports, HW default: 3
 *
 * . for P4080, MXT is in range (0..63)
 *
 */
	return 0;
}

void fm_set_rx_port_params(struct fm_port_drv_t *port,
			   struct fm_port_params *params)
{
	int i;

	port->port_params.specific_params.rx_params.err_fqid = params->errq;
	port->port_params.specific_params.rx_params.dflt_fqid = params->defq;
	port->port_params.specific_params.rx_params.
	    ext_buf_pools.num_of_pools_used = params->num_pools;
	for (i = 0; i < params->num_pools; i++) {
		port->port_params.specific_params.rx_params.ext_buf_pools.
			ext_buf_pool[i].id = params->pool_param[i].id;
		port->port_params.specific_params.rx_params.ext_buf_pools.
			ext_buf_pool[i].size = params->pool_param[i].size;
	}

	port->buff_prefix_content.priv_data_size = params->priv_data_size;
	port->buff_prefix_content.pass_prs_result = params->parse_results;
	port->buff_prefix_content.pass_hash_result = params->hash_results;
	port->buff_prefix_content.pass_time_stamp = params->time_stamp;
	port->buff_prefix_content.data_align = params->data_align;

	init_fm_port_dev(port);
}
EXPORT_SYMBOL(fm_set_rx_port_params);

void fm_set_tx_port_params(struct fm_port_drv_t *port,
			   struct fm_port_params *params)
{
	port->port_params.specific_params.non_rx_params.err_fqid =
		params->errq;
	port->port_params.specific_params.non_rx_params.dflt_fqid =
		params->defq;

	port->buff_prefix_content.priv_data_size = params->priv_data_size;
	port->buff_prefix_content.pass_prs_result = params->parse_results;
	port->buff_prefix_content.pass_hash_result = params->hash_results;
	port->buff_prefix_content.pass_time_stamp = params->time_stamp;
	port->buff_prefix_content.data_align = params->data_align;

	init_fm_port_dev(port);
}
EXPORT_SYMBOL(fm_set_tx_port_params);

static int fm_port_probe(struct platform_device *of_dev)
{
	struct fm_port_drv_t *port;
	struct fm_drv_t *fm_drv;
	struct device *dev;

	dev = &of_dev->dev;

	port = read_fm_port_dev_tree_node(of_dev);
	if (!port)
		return -EIO;

	if (!port->active)
		return 0;

	if (configure_fm_port_dev(port) != 0)
		return -EIO;

	dev_set_drvdata(dev, port);

	fm_drv = (struct fm_drv_t *)port->fm;

	if (port->port_params.port_type == FM_PORT_TYPE_RX) {
		snprintf(port->name, sizeof(port->name),
			 "%s-port-rx%d", fm_drv->name, port->id);
	} else if (port->port_params.port_type == FM_PORT_TYPE_TX) {
		snprintf(port->name, sizeof(port->name),
			 "%s-port-tx%d", fm_drv->name, port->id);
	}

	/* FM_TX_INVALID_ECC_ERRATA_10GMAC_A009 Errata workaround */
	if (port->fm_rev_info.major_rev < 6)
		fm_disable_rams_ecc(fm_drv->fm_dev);

	pr_debug("%s probed\n", port->name);

	return 0;
}

static const struct of_device_id fm_port_match[] = {
	{.compatible = "fsl,fman-v3-port-rx"},
	{.compatible = "fsl,fman-v2-port-rx"},
	{.compatible = "fsl,fman-v3-port-tx"},
	{.compatible = "fsl,fman-v2-port-tx"},
	{}
};

MODULE_DEVICE_TABLE(of, fm_port_match);

static struct platform_driver fm_port_driver = {
	.driver = {
		   .name = "fsl-fman-port",
		   .of_match_table = fm_port_match,
		   },
	.probe = fm_port_probe,
};

builtin_platform_driver(fm_port_driver);
