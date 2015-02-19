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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/clk-provider.h>
#include <linux/errno.h>
#include <sysdev/fsl_soc.h>

#include "fm_ext.h"
#include "fm_drv.h"
#include "fm_muram_ext.h"

/* Bootargs defines */
/* Extra headroom for RX buffers - Default, min and max */
#define FSL_FM_RX_EXTRA_HEADROOM	64
#define FSL_FM_RX_EXTRA_HEADROOM_MIN	16
#define FSL_FM_RX_EXTRA_HEADROOM_MAX	384

/* Maximum frame length */
#define FSL_FM_MAX_FRAME_SIZE			1522
#define FSL_FM_MAX_POSSIBLE_FRAME_SIZE		9600
#define FSL_FM_MIN_POSSIBLE_FRAME_SIZE		64

/* Extra headroom for Rx buffers.
 * FMan is instructed to allocate, on the Rx path, this amount of
 * space at the beginning of a data buffer, beside the DPA private
 * data area and the IC fields.
 * Does not impact Tx buffer layout.
 * Configurable from bootargs. 64 by default, it's needed on
 * particular forwarding scenarios that add extra headers to the
 * forwarded frame.
 */
int fsl_fm_rx_extra_headroom = FSL_FM_RX_EXTRA_HEADROOM;
module_param(fsl_fm_rx_extra_headroom, int, 0);
MODULE_PARM_DESC(fsl_fm_rx_extra_headroom, "Extra headroom for Rx buffers");

/* Max frame size, across all interfaces.
 * Configurable from bootargs, to avoid allocating oversized (socket)
 * buffers when not using jumbo frames.
 * Must be large enough to accommodate the network MTU, but small enough
 * to avoid wasting skb memory.
 *
 * Could be overridden once, at boot-time, via the
 * fm_set_max_frm() callback.
 */
int fsl_fm_max_frm = FSL_FM_MAX_FRAME_SIZE;
module_param(fsl_fm_max_frm, int, 0);
MODULE_PARM_DESC(fsl_fm_max_frm, "Maximum frame size, across all interfaces");

u16 fm_get_max_frm(void)
{
	static bool fm_check_mfl;

	if (!fm_check_mfl) {
		if (fsl_fm_max_frm > FSL_FM_MAX_POSSIBLE_FRAME_SIZE ||
		    fsl_fm_max_frm < FSL_FM_MIN_POSSIBLE_FRAME_SIZE) {
			pr_warn("Invalid fsl_fm_max_frm value (%d) in bootargs, valid range is %d-%d. Falling back to the default (%d)\n",
				fsl_fm_max_frm,
				FSL_FM_MIN_POSSIBLE_FRAME_SIZE,
				FSL_FM_MAX_POSSIBLE_FRAME_SIZE,
				FSL_FM_MAX_FRAME_SIZE);
			fsl_fm_max_frm = FSL_FM_MAX_FRAME_SIZE;
		}
		fm_check_mfl = true;
	}

	return fsl_fm_max_frm;
}
EXPORT_SYMBOL(fm_get_max_frm);

int fm_get_rx_extra_headroom(void)
{
	static bool fm_check_rx_extra_headroom;

	if (!fm_check_rx_extra_headroom) {
		if (fsl_fm_rx_extra_headroom > FSL_FM_RX_EXTRA_HEADROOM_MAX ||
		    fsl_fm_rx_extra_headroom < FSL_FM_RX_EXTRA_HEADROOM_MIN) {
			pr_warn("Invalid fsl_fm_rx_extra_headroom value (%d) in bootargs, valid range is %d-%d. Falling back to the default (%d)\n",
				fsl_fm_rx_extra_headroom,
				FSL_FM_RX_EXTRA_HEADROOM_MIN,
				FSL_FM_RX_EXTRA_HEADROOM_MAX,
				FSL_FM_RX_EXTRA_HEADROOM);
			fsl_fm_rx_extra_headroom = FSL_FM_RX_EXTRA_HEADROOM;
		}

		fsl_fm_rx_extra_headroom = true;
		fsl_fm_rx_extra_headroom = ALIGN(fsl_fm_rx_extra_headroom, 16);
	}

	return fsl_fm_rx_extra_headroom;
}
EXPORT_SYMBOL(fm_get_rx_extra_headroom);

static irqreturn_t fm_irq(int irq, void *fm_dev)
{
	struct fm_drv_t *fm_drv = (struct fm_drv_t *)fm_dev;

	fm_event_isr(fm_drv->fm_dev);

	return IRQ_HANDLED;
}

static irqreturn_t fm_err_irq(int irq, void *fm_dev)
{
	struct fm_drv_t *fm_drv = (struct fm_drv_t *)fm_dev;

	if (fm_error_isr(fm_drv->fm_dev) == 0)
		return IRQ_HANDLED;

	return IRQ_NONE;
}

static int fill_qman_channels_info(struct fm_drv_t *fm_drv)
{
	fm_drv->qman_channels = kcalloc(fm_drv->num_of_qman_channels,
					sizeof(u32), GFP_KERNEL);
	if (!fm_drv->qman_channels)
		return -ENOMEM;

	if (fm_drv->fm_rev_info.major_rev >= 6) {
		fm_drv->qman_channels[0] = 0x30;
		fm_drv->qman_channels[1] = 0x31;
		fm_drv->qman_channels[2] = 0x28;
		fm_drv->qman_channels[3] = 0x29;
		fm_drv->qman_channels[4] = 0x2a;
		fm_drv->qman_channels[5] = 0x2b;
		fm_drv->qman_channels[6] = 0x2c;
		fm_drv->qman_channels[7] = 0x2d;
		fm_drv->qman_channels[8] = 0x2;
		fm_drv->qman_channels[9] = 0x3;
		fm_drv->qman_channels[10] = 0x4;
		fm_drv->qman_channels[11] = 0x5;
		fm_drv->qman_channels[12] = 0x6;
		fm_drv->qman_channels[13] = 0x7;
	} else {
		fm_drv->qman_channels[0] = 0x30;
		fm_drv->qman_channels[1] = 0x28;
		fm_drv->qman_channels[2] = 0x29;
		fm_drv->qman_channels[3] = 0x2a;
		fm_drv->qman_channels[4] = 0x2b;
		fm_drv->qman_channels[5] = 0x2c;
		fm_drv->qman_channels[6] = 0x1;
		fm_drv->qman_channels[7] = 0x2;
		fm_drv->qman_channels[8] = 0x3;
		fm_drv->qman_channels[9] = 0x4;
		fm_drv->qman_channels[10] = 0x5;
		fm_drv->qman_channels[11] = 0x6;
	}

	return 0;
}

static const struct of_device_id fm_muram_match[] = {
	{
	 .compatible = "fsl,fman-muram"},
	{}
};
MODULE_DEVICE_TABLE(of, fm_muram_match);

static struct fm_drv_t *read_fm_dev_tree_node(struct platform_device *of_dev)
{
	struct fm_drv_t *fm_drv;
	struct device_node *fm_node, *muram_node;
	struct resource *res;
	const u32 *u32_prop;
	int lenp, err;
	struct clk *clk;
	u32 clk_rate;

	fm_node = of_node_get(of_dev->dev.of_node);

	u32_prop = (const u32 *)of_get_property(fm_node, "cell-index", &lenp);
	if (!u32_prop) {
		pr_err("of_get_property(%s, cell-index) failed\n",
		       fm_node->full_name);
		goto _return_null;
	}
	if (WARN_ON(lenp != sizeof(u32)))
		return NULL;

	fm_drv = kzalloc(sizeof(*fm_drv), GFP_KERNEL);
	if (!fm_drv)
		goto _return_null;

	fm_drv->dev = &of_dev->dev;
	fm_drv->id = (u8)*u32_prop;

	/* Get the FM interrupt */
	res = platform_get_resource(of_dev, IORESOURCE_IRQ, 0);
	if (!res) {
		pr_err("Can't get FMan IRQ resource\n");
		goto _return_null;
	}
	fm_drv->irq = res->start;

	/* Get the FM error interrupt */
	res = platform_get_resource(of_dev, IORESOURCE_IRQ, 1);
	if (!res) {
		pr_err("Can't get FMan Error IRQ resource\n");
		goto _return_null;
	}
	fm_drv->err_irq = res->start;

	/* Get the FM address */
	res = platform_get_resource(of_dev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("Can't get FMan memory resouce\n");
		goto _return_null;
	}

	fm_drv->fm_base_addr = 0;
	fm_drv->fm_phys_base_addr = res->start;
	fm_drv->fm_mem_size = res->end + 1 - res->start;

	clk = of_clk_get_by_name(fm_node, NULL);
	if (IS_ERR(clk)) {
		pr_err("Failed to get FM%d clock structure\n", fm_drv->id);
		goto _return_null;
	}

	clk_rate = clk_get_rate(clk);
	if (!clk_rate) {
		pr_err("Failed to determine FM%d clock rate\n", fm_drv->id);
		goto _return_null;
	}
	/* Rounding to MHz */
	clk_rate = (clk_rate + 500000) / 1000000;
	fm_drv->params.fm_clk_freq = (u16)clk_rate;

	u32_prop = (const u32 *)of_get_property(fm_node,
						"fsl,qman-channel-range",
						&lenp);
	if (!u32_prop) {
		pr_err("of_get_property(%s, fsl,qman-channel-range) failed\n",
		       fm_node->full_name);
		goto _return_null;
	}
	if (WARN_ON(lenp != sizeof(u32) * 2))
		goto _return_null;
	fm_drv->qman_channel_base = u32_prop[0];
	fm_drv->num_of_qman_channels = u32_prop[1];

	/* Get the MURAM base address and size */
	muram_node = of_find_matching_node(fm_node, fm_muram_match);
	if (!muram_node) {
		pr_err("could not find MURAM node\n");
		goto _return_null;
	}

	err = of_address_to_resource(muram_node, 0, res);
	if (err) {
		of_node_put(muram_node);
		pr_err("of_address_to_resource() = %d\n", err);
		goto _return_null;
	}

	fm_drv->fm_muram_phys_base_addr = res->start;
	fm_drv->fm_muram_mem_size = res->end + 1 - res->start;

	{
		/* In B4 rev 2.0 (and above) the MURAM size is 512KB.
		 * Check the SVR and update MURAM size if required.
		 */
		u32 svr;

		svr = mfspr(SPRN_SVR);

		if ((SVR_SOC_VER(svr) == SVR_B4860) && (SVR_MAJ(svr) >= 2))
			fm_drv->fm_muram_mem_size = 0x80000;
	}

	of_node_put(muram_node);
	of_node_put(fm_node);

	fm_drv->active = true;

	goto _return;

_return_null:
	of_node_put(fm_node);
	return NULL;
_return:
	return fm_drv;
}

static void fm_drv_exceptions_cb(void *dev_id,
				 enum fm_exceptions __maybe_unused exception)
{
	struct fm_drv_t *fm_drv = (struct fm_drv_t *)dev_id;

	WARN_ON(!fm_drv);

	pr_debug("got fm exception %d\n", exception);
}

static void fm_drv_bus_error_cb(void *dev_id,
				enum fm_port_type __maybe_unused port_type,
				u8 __maybe_unused port_id,
				u64 __maybe_unused addr,
				u8 __maybe_unused tnum,
				u16 __maybe_unused liodn)
{
	struct fm_drv_t *fm_drv = (struct fm_drv_t *)dev_id;

	WARN_ON(!fm_drv);

	pr_debug("got fm bus error: port_id[%d]\n", port_id);
}

u32 get_qman_channel_id(struct fm_drv_t *fm_drv, u32 port_id)
{
	u32 qman_channel = 0;
	int i;

	for (i = 0; i < fm_drv->num_of_qman_channels; i++) {
		if (fm_drv->qman_channels[i] == port_id)
			break;
	}

	if (i == fm_drv->num_of_qman_channels)
		return 0;

	qman_channel = fm_drv->qman_channel_base + i;

	return qman_channel;
}

static int configure_fm_dev(struct fm_drv_t *fm_drv)
{
	int err;

	if (!fm_drv->active) {
		pr_err("FMan not configured\n");
		return -EINVAL;
	}

	err = devm_request_irq(fm_drv->dev, fm_drv->irq, fm_irq,
			       IRQF_NO_SUSPEND, "fman", fm_drv);
	if (err < 0) {
		pr_err("Error: allocating irq %d (error = %d)\n",
		       fm_drv->irq, err);
		return -EINVAL;
	}

	if (fm_drv->err_irq != 0) {
		err = devm_request_irq(fm_drv->dev, fm_drv->err_irq,
				       fm_err_irq,
				       IRQF_SHARED | IRQF_NO_SUSPEND,
				       "fman-err", fm_drv);
		if (err < 0) {
			pr_err("Error: allocating irq %d (error = %d)\n",
			       fm_drv->err_irq, err);
			return -EINVAL;
		}
	}

	fm_drv->res = devm_request_mem_region(fm_drv->dev,
					      fm_drv->fm_phys_base_addr,
					      fm_drv->fm_mem_size, "fman");
	if (!fm_drv->res) {
		pr_err("request_mem_region() failed\n");
		return -EINVAL;
	}

	fm_drv->fm_base_addr = devm_ioremap(fm_drv->dev,
					    fm_drv->fm_phys_base_addr,
					    fm_drv->fm_mem_size);
	if (fm_drv->fm_base_addr == 0) {
		pr_err("devm_ioremap() failed\n");
		return -EINVAL;
	}

	fm_drv->params.base_addr = fm_drv->fm_base_addr;
	fm_drv->params.fm_id = fm_drv->id;
	fm_drv->params.exception_cb = fm_drv_exceptions_cb;
	fm_drv->params.bus_error_cb = fm_drv_bus_error_cb;
	fm_drv->params.dev_id = fm_drv;

	return 0;
}

static int init_fm_dev(struct fm_drv_t *fm_drv)
{
	if (!fm_drv->active) {
		pr_err("FMan not configured\n");
		return -EINVAL;
	}

	fm_drv->muram = fm_muram_init(fm_drv->fm_muram_phys_base_addr,
				      fm_drv->fm_muram_mem_size);
	if (!fm_drv->muram) {
		pr_err("FMan MURAM initalization failed\n");
		return -EINVAL;
	}

	fm_drv->params.muram = fm_drv->muram;

	fm_drv->fm_dev = fm_config(&fm_drv->params);
	if (!fm_drv->fm_dev) {
		pr_err("FMan config failed\n");
		return -EINVAL;
	}

	fm_get_revision(fm_drv->fm_dev, &fm_drv->fm_rev_info);

	if (fm_cfg_reset_on_init(fm_drv->fm_dev, true) != 0) {
		pr_err("fm_cfg_reset_on_init() failed\n");
		return -EINVAL;
	}

	/* Config total fifo size for FManV3H */
	if ((fm_drv->fm_rev_info.major_rev >= 6) &&
	    (fm_drv->fm_rev_info.minor_rev != 1 &&
	     fm_drv->fm_rev_info.minor_rev != 4))
			fm_cfg_total_fifo_size(fm_drv->fm_dev, 295 * 1024);

	if (fm_init(fm_drv->fm_dev) != 0) {
		pr_err("fm_init() failed\n");
		return -EINVAL;
	}

	if (fm_drv->err_irq == 0) {
		fm_set_exception(fm_drv->fm_dev, FM_EX_DMA_BUS_ERROR, false);
		fm_set_exception(fm_drv->fm_dev, FM_EX_DMA_READ_ECC, false);
		fm_set_exception(fm_drv->fm_dev,
				 FM_EX_DMA_SYSTEM_WRITE_ECC, false);
		fm_set_exception(fm_drv->fm_dev, FM_EX_DMA_FM_WRITE_ECC, false);
		fm_set_exception(fm_drv->fm_dev,
				 FM_EX_DMA_SINGLE_PORT_ECC, false);
		fm_set_exception(fm_drv->fm_dev,
				 FM_EX_FPM_STALL_ON_TASKS, false);
		fm_set_exception(fm_drv->fm_dev, FM_EX_FPM_SINGLE_ECC, false);
		fm_set_exception(fm_drv->fm_dev, FM_EX_FPM_DOUBLE_ECC, false);
		fm_set_exception(fm_drv->fm_dev, FM_EX_QMI_SINGLE_ECC, false);
		fm_set_exception(fm_drv->fm_dev,
				 FM_EX_QMI_DOUBLE_ECC, false);
		fm_set_exception(fm_drv->fm_dev,
				 FM_EX_QMI_DEQ_FROM_UNKNOWN_PORTID, false);
		fm_set_exception(fm_drv->fm_dev, FM_EX_BMI_LIST_RAM_ECC, false);
		fm_set_exception(fm_drv->fm_dev,
				 FM_EX_BMI_STORAGE_PROFILE_ECC, false);
		fm_set_exception(fm_drv->fm_dev,
				 FM_EX_BMI_STATISTICS_RAM_ECC, false);
		fm_set_exception(fm_drv->fm_dev,
				 FM_EX_BMI_DISPATCH_RAM_ECC, false);
	}

	if (fill_qman_channels_info(fm_drv) < 0) {
		pr_err("can't fill qman channel info\n");
		return -EINVAL;
	}

	return 0;
}

static int fm_probe(struct platform_device *of_dev)
{
	struct fm_drv_t *fm_drv;

	fm_drv = read_fm_dev_tree_node(of_dev);
	if (!fm_drv)
		return -EIO;
	if (configure_fm_dev(fm_drv) != 0)
		return -EIO;
	if (init_fm_dev(fm_drv) != 0)
		return -EIO;

	dev_set_drvdata(fm_drv->dev, fm_drv);

	pr_debug("FM%d probed\n", fm_drv->id);

	return 0;
}

struct fm *fm_bind(struct device *fm_dev)
{
	return (struct fm *)(dev_get_drvdata(get_device(fm_dev)));
}

void fm_unbind(struct fm *fm)
{
	struct fm_drv_t *fm_drv = (struct fm_drv_t *)fm;

	put_device(fm_drv->dev);
}

struct resource *fm_get_mem_region(struct fm *fm)
{
	struct fm_drv_t *fm_drv = (struct fm_drv_t *)fm;

	return fm_drv->res;
}

void *fm_get_handle(struct fm *fm)
{
	struct fm_drv_t *fm_drv = (struct fm_drv_t *)fm;

	return (void *)fm_drv->fm_dev;
}

static const struct of_device_id fm_match[] = {
	{
	 .compatible = "fsl,fman"},
	{}
};

MODULE_DEVICE_TABLE(of, fm_match);

static struct platform_driver fm_driver = {
	.driver = {
		   .name = "fsl-fman",
		   .of_match_table = fm_match,
		   },
	.probe = fm_probe,
};

builtin_platform_driver(fm_driver);
