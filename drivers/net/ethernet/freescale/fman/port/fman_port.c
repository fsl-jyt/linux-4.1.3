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

#include "fsl_fman_port.h"

static u32 get_no_pcd_nia_bmi_ac_enc_frame(struct fman_port_cfg *cfg)
{
	if (cfg->errata_A006675)
		return NIA_ENG_FM_CTL |
		    NIA_FM_CTL_AC_NO_IPACC_PRE_BMI_ENQ_FRAME;
	else
		return NIA_ENG_BMI | NIA_BMI_AC_ENQ_FRAME;
}

static int init_bmi_rx(struct fman_port *port,
		       struct fman_port_cfg *cfg,
		       struct fman_port_params *params)
{
	struct fman_port_rx_bmi_regs __iomem *regs = &port->bmi_regs->rx;
	u32 tmp;

	/* Rx Configuration register */
	tmp = 0;
	if (cfg->discard_override)
		tmp |= BMI_PORT_CFG_FDOVR;
	iowrite32be(tmp, &regs->fmbm_rcfg);

	/* DMA attributes */
	tmp = (u32)cfg->dma_swap_data << BMI_DMA_ATTR_SWP_SHIFT;
	if (cfg->dma_ic_stash_on)
		tmp |= BMI_DMA_ATTR_IC_STASH_ON;
	if (cfg->dma_header_stash_on)
		tmp |= BMI_DMA_ATTR_HDR_STASH_ON;
	if (cfg->dma_sg_stash_on)
		tmp |= BMI_DMA_ATTR_SG_STASH_ON;
	if (cfg->dma_write_optimize)
		tmp |= BMI_DMA_ATTR_WRITE_OPTIMIZE;
	iowrite32be(tmp, &regs->fmbm_rda);

	/* Rx FIFO parameters */
	tmp = (cfg->rx_pri_elevation / FMAN_PORT_BMI_FIFO_UNITS - 1) <<
	    BMI_RX_FIFO_PRI_ELEVATION_SHIFT;
	tmp |= cfg->rx_fifo_thr / FMAN_PORT_BMI_FIFO_UNITS - 1;
	iowrite32be(tmp, &regs->fmbm_rfp);

	if (cfg->excessive_threshold_register)
		/* always allow access to the extra resources */
		iowrite32be(BMI_RX_FIFO_THRESHOLD_ETHE, &regs->fmbm_reth);

	/* Frame end data */
	tmp = (cfg->checksum_bytes_ignore & BMI_FRAME_END_CS_IGNORE_MASK) <<
		BMI_FRAME_END_CS_IGNORE_SHIFT;
	tmp |= (cfg->rx_cut_end_bytes & BMI_RX_FRAME_END_CUT_MASK) <<
		BMI_RX_FRAME_END_CUT_SHIFT;
	if (cfg->errata_A006320)
		tmp &= 0xffe0ffff;
	iowrite32be(tmp, &regs->fmbm_rfed);

	/* Internal context parameters */
	tmp = ((cfg->ic_ext_offset / FMAN_PORT_IC_OFFSET_UNITS) &
		BMI_IC_TO_EXT_MASK) << BMI_IC_TO_EXT_SHIFT;
	tmp |= ((cfg->ic_int_offset / FMAN_PORT_IC_OFFSET_UNITS) &
		BMI_IC_FROM_INT_MASK) << BMI_IC_FROM_INT_SHIFT;
	tmp |= (cfg->ic_size / FMAN_PORT_IC_OFFSET_UNITS) & BMI_IC_SIZE_MASK;
	iowrite32be(tmp, &regs->fmbm_ricp);

	/* Internal buffer offset */
	tmp = ((cfg->int_buf_start_margin / FMAN_PORT_IC_OFFSET_UNITS) &
		BMI_INT_BUF_MARG_MASK) << BMI_INT_BUF_MARG_SHIFT;
	iowrite32be(tmp, &regs->fmbm_rim);

	/* External buffer margins */
	tmp = (cfg->ext_buf_start_margin & BMI_EXT_BUF_MARG_START_MASK) <<
		BMI_EXT_BUF_MARG_START_SHIFT;
	tmp |= cfg->ext_buf_end_margin & BMI_EXT_BUF_MARG_END_MASK;
	if (cfg->fmbm_rebm_has_sgd && cfg->no_scatter_gather)
		tmp |= BMI_SG_DISABLE;
	iowrite32be(tmp, &regs->fmbm_rebm);

	/* Frame attributes */
	tmp = BMI_CMD_RX_MR_DEF;
	tmp |= BMI_CMD_ATTR_ORDER;
	tmp |= (u32)cfg->color << BMI_CMD_ATTR_COLOR_SHIFT;
	if (cfg->sync_req)
		tmp |= BMI_CMD_ATTR_SYNC;

	iowrite32be(tmp, &regs->fmbm_rfca);

	/* NIA */
	tmp = (u32)cfg->rx_fd_bits << BMI_NEXT_ENG_FD_BITS_SHIFT;
	tmp |= get_no_pcd_nia_bmi_ac_enc_frame(cfg);
	iowrite32be(tmp, &regs->fmbm_rfne);

	/* Enqueue NIA */
	iowrite32be(NIA_ENG_QMI_ENQ | NIA_ORDER_RESTOR, &regs->fmbm_rfene);

	/* Default/error queues */
	iowrite32be((params->dflt_fqid & DEFAULT_FRAME_QUEUE_ID),
		    &regs->fmbm_rfqid);
	iowrite32be((params->err_fqid & ERROR_FRAME_QUEUE_ID),
		    &regs->fmbm_refqid);

	/* Discard/error masks */
	iowrite32be(params->discard_mask, &regs->fmbm_rfsdm);
	iowrite32be(params->err_mask, &regs->fmbm_rfsem);

	return 0;
}

static int init_bmi_tx(struct fman_port *port,
		       struct fman_port_cfg *cfg,
		       struct fman_port_params *params)
{
	struct fman_port_tx_bmi_regs __iomem *regs = &port->bmi_regs->tx;
	u32 tmp;

	/* Tx Configuration register */
	tmp = 0;
	iowrite32be(tmp, &regs->fmbm_tcfg);

	/* DMA attributes */
	tmp = (u32)cfg->dma_swap_data << BMI_DMA_ATTR_SWP_SHIFT;
	if (cfg->dma_ic_stash_on)
		tmp |= BMI_DMA_ATTR_IC_STASH_ON;
	if (cfg->dma_header_stash_on)
		tmp |= BMI_DMA_ATTR_HDR_STASH_ON;
	if (cfg->dma_sg_stash_on)
		tmp |= BMI_DMA_ATTR_SG_STASH_ON;
	iowrite32be(tmp, &regs->fmbm_tda);

	/* Tx FIFO parameters */
	tmp = (cfg->tx_fifo_min_level / FMAN_PORT_BMI_FIFO_UNITS) <<
	    BMI_TX_FIFO_MIN_FILL_SHIFT;
	tmp |= ((cfg->tx_fifo_deq_pipeline_depth - 1) &
		BMI_FIFO_PIPELINE_DEPTH_MASK) << BMI_FIFO_PIPELINE_DEPTH_SHIFT;
	tmp |= (cfg->tx_fifo_low_comf_level / FMAN_PORT_BMI_FIFO_UNITS) - 1;
	iowrite32be(tmp, &regs->fmbm_tfp);

	/* Frame end data */
	tmp = (cfg->checksum_bytes_ignore & BMI_FRAME_END_CS_IGNORE_MASK) <<
		BMI_FRAME_END_CS_IGNORE_SHIFT;
	iowrite32be(tmp, &regs->fmbm_tfed);

	/* Internal context parameters */
	tmp = ((cfg->ic_ext_offset / FMAN_PORT_IC_OFFSET_UNITS) &
		BMI_IC_TO_EXT_MASK) << BMI_IC_TO_EXT_SHIFT;
	tmp |= ((cfg->ic_int_offset / FMAN_PORT_IC_OFFSET_UNITS) &
		BMI_IC_FROM_INT_MASK) << BMI_IC_FROM_INT_SHIFT;
	tmp |= (cfg->ic_size / FMAN_PORT_IC_OFFSET_UNITS) & BMI_IC_SIZE_MASK;
	iowrite32be(tmp, &regs->fmbm_ticp);

	/* Frame attributes */
	tmp = BMI_CMD_TX_MR_DEF;
	tmp |= BMI_CMD_ATTR_ORDER;
	tmp |= (u32)cfg->color << BMI_CMD_ATTR_COLOR_SHIFT;
	iowrite32be(tmp, &regs->fmbm_tfca);

	/* Dequeue NIA + enqueue NIA */
	iowrite32be(NIA_ENG_QMI_DEQ, &regs->fmbm_tfdne);
	iowrite32be(NIA_ENG_QMI_ENQ | NIA_ORDER_RESTOR, &regs->fmbm_tfene);
	if (cfg->fmbm_tfne_has_features)
		iowrite32be(!params->dflt_fqid ?
			    BMI_EBD_EN | NIA_BMI_AC_FETCH_ALL_FRAME :
			    NIA_BMI_AC_FETCH_ALL_FRAME, &regs->fmbm_tfne);
	if (!params->dflt_fqid && params->dont_release_buf) {
		iowrite32be(DEFAULT_CONF_FRAME_QUEUE_ID, &regs->fmbm_tcfqid);
		iowrite32be(NIA_ENG_BMI | NIA_BMI_AC_TX_RELEASE,
			    &regs->fmbm_tfene);
		if (cfg->fmbm_tfne_has_features)
			iowrite32be(ioread32be(&regs->fmbm_tfne) & ~BMI_EBD_EN,
				    &regs->fmbm_tfne);
	}

	/* Confirmation/error queues */
	if (params->dflt_fqid || !params->dont_release_buf)
		iowrite32be(params->dflt_fqid & DEFAULT_CONF_FRAME_QUEUE_ID,
			    &regs->fmbm_tcfqid);
	iowrite32be((params->err_fqid & ERROR_FRAME_QUEUE_ID),
		    &regs->fmbm_tefqid);

	return 0;
}

static int init_qmi(struct fman_port *port,
		    struct fman_port_cfg *cfg, struct fman_port_params *params)
{
	struct fman_port_qmi_regs __iomem *regs = port->qmi_regs;
	u32 tmp;

	/* Rx port configuration */
	if ((port->type == E_FMAN_PORT_TYPE_RX) ||
	    (port->type == E_FMAN_PORT_TYPE_RX_10G)) {
		/* Enqueue NIA */
		iowrite32be(NIA_ENG_BMI | NIA_BMI_AC_RELEASE, &regs->fmqm_pnen);
		return 0;
	}

	/* Continue with Tx and O/H port configuration */
	if ((port->type == E_FMAN_PORT_TYPE_TX) ||
	    (port->type == E_FMAN_PORT_TYPE_TX_10G)) {
		/* Enqueue NIA */
		iowrite32be(NIA_ENG_BMI | NIA_BMI_AC_TX_RELEASE,
			    &regs->fmqm_pnen);
		/* Dequeue NIA */
		iowrite32be(NIA_ENG_BMI | NIA_BMI_AC_TX, &regs->fmqm_pndn);
	} else {
		/* Enqueue NIA */
		iowrite32be(NIA_ENG_BMI | NIA_BMI_AC_RELEASE, &regs->fmqm_pnen);
		/* Dequeue NIA */
		iowrite32be(NIA_ENG_BMI | NIA_BMI_AC_FETCH, &regs->fmqm_pndn);
	}

	/* Dequeue Configuration register */
	tmp = 0;
	if (cfg->deq_high_pri)
		tmp |= QMI_DEQ_CFG_PRI;

	switch (cfg->deq_type) {
	case E_FMAN_PORT_DEQ_BY_PRI:
		tmp |= QMI_DEQ_CFG_TYPE1;
		break;
	case E_FMAN_PORT_DEQ_ACTIVE_FQ:
		tmp |= QMI_DEQ_CFG_TYPE2;
		break;
	case E_FMAN_PORT_DEQ_ACTIVE_FQ_NO_ICS:
		tmp |= QMI_DEQ_CFG_TYPE3;
		break;
	default:
		return -EINVAL;
	}

	if (cfg->qmi_deq_options_support) {
		switch (cfg->deq_prefetch_opt) {
		case E_FMAN_PORT_DEQ_NO_PREFETCH:
			break;
		case E_FMAN_PORT_DEQ_PART_PREFETCH:
			tmp |= QMI_DEQ_CFG_PREFETCH_PARTIAL;
			break;
		case E_FMAN_PORT_DEQ_FULL_PREFETCH:
			tmp |= QMI_DEQ_CFG_PREFETCH_FULL;
			break;
		default:
			return -EINVAL;
		}
	}
	tmp |= (params->deq_sp & QMI_DEQ_CFG_SP_MASK) << QMI_DEQ_CFG_SP_SHIFT;
	tmp |= cfg->deq_byte_cnt;
	iowrite32be(tmp, &regs->fmqm_pndc);

	return 0;
}

void fman_port_defconfig(struct fman_port_cfg *cfg, enum fman_port_type type)
{
	cfg->dma_swap_data = E_FMAN_PORT_DMA_NO_SWAP;
	cfg->dma_ic_stash_on = false;
	cfg->dma_header_stash_on = false;
	cfg->dma_sg_stash_on = false;
	cfg->dma_write_optimize = true;
	cfg->color = E_FMAN_PORT_COLOR_GREEN;
	cfg->discard_override = false;
	cfg->checksum_bytes_ignore = 0;
	cfg->rx_cut_end_bytes = 4;
	cfg->rx_pri_elevation = BMI_PRIORITY_ELEVATION_LEVEL;
	cfg->rx_fifo_thr = BMI_FIFO_THRESHOLD;
	cfg->rx_fd_bits = 0;
	cfg->ic_ext_offset = 0;
	cfg->ic_int_offset = 0;
	cfg->ic_size = 0;
	cfg->int_buf_start_margin = 0;
	cfg->ext_buf_start_margin = 0;
	cfg->ext_buf_end_margin = 0;
	cfg->tx_fifo_min_level = 0;
	cfg->tx_fifo_low_comf_level = (5 * 1024);
	cfg->deq_type = E_FMAN_PORT_DEQ_BY_PRI;

	cfg->sync_req = true;
	cfg->deq_prefetch_opt = E_FMAN_PORT_DEQ_FULL_PREFETCH;

	cfg->tx_fifo_deq_pipeline_depth = BMI_DEQUEUE_PIPELINE_DEPTH(type);
	cfg->deq_high_pri = QMI_HIGH_PRIORITY(type);
	cfg->deq_byte_cnt = QMI_BYTE_COUNT_LEVEL_CONTROL(type);
}

int fman_port_init(struct fman_port *port,
		   struct fman_port_cfg *cfg, struct fman_port_params *params)
{
	int err;

	/* Init BMI registers */
	switch (port->type) {
	case E_FMAN_PORT_TYPE_RX:
	case E_FMAN_PORT_TYPE_RX_10G:
		err = init_bmi_rx(port, cfg, params);
		break;
	case E_FMAN_PORT_TYPE_TX:
	case E_FMAN_PORT_TYPE_TX_10G:
		err = init_bmi_tx(port, cfg, params);
		break;
	default:
		return -EINVAL;
	}

	if (err)
		return err;

	/* Init QMI registers */
	err = init_qmi(port, cfg, params);
	return err;

	return 0;
}

int fman_port_enable(struct fman_port *port)
{
	u32 __iomem *bmi_cfg_reg;
	u32 tmp;
	bool rx_port;

	switch (port->type) {
	case E_FMAN_PORT_TYPE_RX:
	case E_FMAN_PORT_TYPE_RX_10G:
		bmi_cfg_reg = &port->bmi_regs->rx.fmbm_rcfg;
		rx_port = true;
		break;
	case E_FMAN_PORT_TYPE_TX:
	case E_FMAN_PORT_TYPE_TX_10G:
		bmi_cfg_reg = &port->bmi_regs->tx.fmbm_tcfg;
		rx_port = false;
		break;
	default:
		return -EINVAL;
	}

	/* Enable QMI */
	if (!rx_port) {
		tmp = ioread32be(&port->qmi_regs->fmqm_pnc) | QMI_PORT_CFG_EN;
		iowrite32be(tmp, &port->qmi_regs->fmqm_pnc);
	}

	/* Enable BMI */
	tmp = ioread32be(bmi_cfg_reg) | BMI_PORT_CFG_EN;
	iowrite32be(tmp, bmi_cfg_reg);

	return 0;
}

int fman_port_disable(const struct fman_port *port)
{
	u32 __iomem *bmi_cfg_reg, *bmi_status_reg;
	u32 tmp;
	bool rx_port, failure = false;
	int count;

	switch (port->type) {
	case E_FMAN_PORT_TYPE_RX:
	case E_FMAN_PORT_TYPE_RX_10G:
		bmi_cfg_reg = &port->bmi_regs->rx.fmbm_rcfg;
		bmi_status_reg = &port->bmi_regs->rx.fmbm_rst;
		rx_port = true;
		break;
	case E_FMAN_PORT_TYPE_TX:
	case E_FMAN_PORT_TYPE_TX_10G:
		bmi_cfg_reg = &port->bmi_regs->tx.fmbm_tcfg;
		bmi_status_reg = &port->bmi_regs->tx.fmbm_tst;
		rx_port = false;
		break;
	default:
		return -EINVAL;
	}

	/* Disable QMI */
	if (!rx_port) {
		tmp = ioread32be(&port->qmi_regs->fmqm_pnc) & ~QMI_PORT_CFG_EN;
		iowrite32be(tmp, &port->qmi_regs->fmqm_pnc);

		/* Wait for QMI to finish FD handling */
		count = 100;
		do {
			udelay(10);
			tmp = ioread32be(&port->qmi_regs->fmqm_pns);
		} while ((tmp & QMI_PORT_STATUS_DEQ_FD_BSY) && --count);

		if (count == 0) {
			/* Timeout */
			failure = true;
		}
	}

	/* Disable BMI */
	tmp = ioread32be(bmi_cfg_reg) & ~BMI_PORT_CFG_EN;
	iowrite32be(tmp, bmi_cfg_reg);

	/* Wait for graceful stop end */
	count = 500;
	do {
		udelay(10);
		tmp = ioread32be(bmi_status_reg);
	} while ((tmp & BMI_PORT_STATUS_BSY) && --count);

	if (count == 0) {
		/* Timeout */
		failure = true;
	}

	if (failure)
		return -EBUSY;

	return 0;
}

int fman_port_set_bpools(const struct fman_port *port,
			 const struct fman_port_bpools *bp)
{
	u32 __iomem *bp_reg, *bp_depl_reg;
	u32 tmp;
	u8 i, max_bp_num;
	bool grp_depl_used = false, rx_port;

	switch (port->type) {
	case E_FMAN_PORT_TYPE_RX:
	case E_FMAN_PORT_TYPE_RX_10G:
		max_bp_num = port->ext_pools_num;
		rx_port = true;
		bp_reg = port->bmi_regs->rx.fmbm_ebmpi;
		bp_depl_reg = &port->bmi_regs->rx.fmbm_mpd;
		break;
	default:
		return -EINVAL;
	}

	if (rx_port) {
		/* Check buffers are provided in ascending order */
		for (i = 0; (i < (bp->count - 1) &&
			     (i < FMAN_PORT_MAX_EXT_POOLS_NUM - 1)); i++) {
			if (bp->bpool[i].size > bp->bpool[i + 1].size)
				return -EINVAL;
		}
	}

	/* Set up external buffers pools */
	for (i = 0; i < bp->count; i++) {
		tmp = BMI_EXT_BUF_POOL_VALID;
		tmp |= ((u32)bp->bpool[i].bpid <<
			BMI_EXT_BUF_POOL_ID_SHIFT) & BMI_EXT_BUF_POOL_ID_MASK;

		if (rx_port) {
			if (bp->counters_enable)
				tmp |= BMI_EXT_BUF_POOL_EN_COUNTER;

			if (bp->bpool[i].is_backup)
				tmp |= BMI_EXT_BUF_POOL_BACKUP;

			tmp |= (u32)bp->bpool[i].size;
		}

		iowrite32be(tmp, &bp_reg[i]);
	}

	/* Clear unused pools */
	for (i = bp->count; i < max_bp_num; i++)
		iowrite32be(0, &bp_reg[i]);

	/* Pools depletion */
	tmp = 0;
	for (i = 0; i < FMAN_PORT_MAX_EXT_POOLS_NUM; i++) {
		if (bp->bpool[i].grp_bp_depleted) {
			grp_depl_used = true;
			tmp |= 0x80000000 >> i;
		}

		if (bp->bpool[i].single_bp_depleted)
			tmp |= 0x80 >> i;
	}

	if (grp_depl_used)
		tmp |= ((u32)bp->grp_bp_depleted_num - 1) <<
		    BMI_POOL_DEP_NUM_OF_POOLS_SHIFT;

	iowrite32be(tmp, bp_depl_reg);
	return 0;
}
