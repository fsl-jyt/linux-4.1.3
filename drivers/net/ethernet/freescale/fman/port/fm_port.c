/*
 * Copyright 2008 - 2015 Freescale Semiconductor Inc.
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

#include "fm_muram_ext.h"

#include "fm_port.h"

#include <linux/string.h>
#include <linux/slab.h>

static int check_init_parameters(struct fm_port_t *fm_port)
{
	struct fm_port_drv_param_t *params = fm_port->fm_port_drv_param;
	struct fman_port_cfg *dflt_config = &params->dflt_cfg;
	u32 unused_mask;

	/* Rx only */
	if (fm_port->port_type == FM_PORT_TYPE_RX) {
		/* external buffer pools */
		if (!params->ext_buf_pools.num_of_pools_used) {
			pr_err("ext_buf_pools.num_of_pools_used=0. At least one buffer pool must be defined\n");
			return -EINVAL;
		}

		if (fm_sp_check_buf_pools_params(&params->ext_buf_pools,
						 params->backup_bm_pools,
			&params->buf_pool_depletion,
			fm_port->port_intg->max_num_of_ext_pools,
			fm_port->port_intg->bm_max_num_of_pools) != 0)
			return -EINVAL;
		/* Check that part of IC that needs copying is small enough
		 * to enter start margin
		 */
		if (params->int_context.size &&
		    (params->int_context.size +
		     params->int_context.ext_buf_offset >
		     params->buf_margins.start_margins)) {
			pr_err("int_context.size is larger than start margins\n");
			return -EINVAL;
		}

		if ((params->liodn_offset != LIODN_DONT_OVERRIDE) &&
		    (params->liodn_offset & ~FM_LIODN_OFFSET_MASK)) {
			pr_err("liodn_offset is larger than %d\n",
			       FM_LIODN_OFFSET_MASK + 1);
		}

		if (fm_port->fm_rev_info.major_rev < 6)
			if (fm_port->fm_port_drv_param->backup_bm_pools) {
				pr_err("Backup Bm Pools\n");
				return -EINVAL;
			}
	} else {
		/* Non Rx ports */
		if (params->deq_sub_portal >=
		    fm_port->port_intg->fm_max_num_of_sub_portals) {
			pr_err("deq_sub_portal has to be in the range of 0 - %d\n",
			       fm_port->port_intg->fm_max_num_of_sub_portals);
			return -EINVAL;
		}

		/* to protect HW internal-context from overwrite */
		if ((params->int_context.size) &&
		    (params->int_context.int_context_offset <
		     MIN_TX_INT_OFFSET)) {
			pr_err("non-Rx int_context.int_context_offset can't be smaller than %d\n",
			       MIN_TX_INT_OFFSET);
			return -EINVAL;
		}

		if ((fm_port->port_type == FM_PORT_TYPE_TX) ||
		    /* in O/H DFLT_NOT_SUPPORTED indicates that
		     * it is not supported and should not be checked
		     */
		    (fm_port->fm_port_drv_param->dflt_cfg.
		     tx_fifo_deq_pipeline_depth != DFLT_NOT_SUPPORTED)) {
			/* Check that not larger than 8 */
			if ((!fm_port->fm_port_drv_param->dflt_cfg.
			     tx_fifo_deq_pipeline_depth) ||
				(fm_port->fm_port_drv_param->dflt_cfg.
				tx_fifo_deq_pipeline_depth >
				MAX_FIFO_PIPELINE_DEPTH)) {
				pr_err("fifo_deq_pipeline_depth can't be larger than %d\n",
				       MAX_FIFO_PIPELINE_DEPTH);
				return -EINVAL;
			}
		}
	}

	/* Rx */
	if (fm_port->port_type == FM_PORT_TYPE_RX) {
		if (!params->dflt_fqid) {
			pr_err("dflt_fqid must be between 1 and 2^24-1\n");
			return -EINVAL;
		}
	}

	/* All ports */
	/* common BMI registers values */
	/* Check that Queue Id is not larger than 2^24, and is not 0 */
	if ((params->err_fqid & ~0x00FFFFFF) || !params->err_fqid) {
		pr_err("err_fqid must be between 1 and 2^24-1\n");
		return -EINVAL;
	}
	if (params->dflt_fqid & ~0x00FFFFFF) {
		pr_err("dflt_fqid must be between 1 and 2^24-1\n");
		return -EINVAL;
	}

	/* Rx only */
	if (fm_port->port_type == FM_PORT_TYPE_RX) {
		if (dflt_config->rx_pri_elevation % BMI_FIFO_UNITS) {
			pr_err("rx_fifo_pri_elevation_level has to be divisible by %d\n",
			       BMI_FIFO_UNITS);
			return -EINVAL;
		}
		if ((dflt_config->rx_pri_elevation < BMI_FIFO_UNITS) ||
		    (dflt_config->rx_pri_elevation >
		     fm_port->port_intg->max_port_fifo_size)) {
			pr_err("rx_fifo_pri_elevation_level not in range of 256 - %d\n",
			       fm_port->port_intg->max_port_fifo_size);
			return -EINVAL;
		}
		if (dflt_config->rx_fifo_thr % BMI_FIFO_UNITS) {
			pr_err("rx_fifo_threshold must be div by %d\n",
			       BMI_FIFO_UNITS);
			return -EINVAL;
		}
		if ((dflt_config->rx_fifo_thr < BMI_FIFO_UNITS) ||
		    (dflt_config->rx_fifo_thr >
		     fm_port->port_intg->max_port_fifo_size)) {
			pr_err("rx_fifo_threshold has to be in the range of 256 - %d\n",
			       fm_port->port_intg->max_port_fifo_size);
			return -EINVAL;
		}

		/* Check that not larger than 16 */
		if (dflt_config->rx_cut_end_bytes > FRAME_END_DATA_SIZE) {
			pr_err("cut_bytes_from_end can't be larger than %d\n",
			       FRAME_END_DATA_SIZE);
			return -EINVAL;
		}

		if (fm_sp_check_buf_margins(&params->buf_margins) != 0)
			return -EINVAL;

		/* extra FIFO size (allowed only to Rx ports) */
		if (params->set_size_of_fifo &&
		    (fm_port->fifo_bufs.extra % BMI_FIFO_UNITS)) {
			pr_err("fifo_bufs.extra has to be divisible by %d\n",
			       BMI_FIFO_UNITS);
			return -EINVAL;
		}

		if (params->buf_pool_depletion.pools_grp_mode_enable &&
		    !params->buf_pool_depletion.num_of_pools) {
			pr_err("buf_pool_depletion.num_of_pools can not be 0 when pools_grp_mode_enable=true\n");
			return -EINVAL;
		}
	}

	/* Non Rx ports */
	/* extra FIFO size (allowed only to Rx ports) */
	else if (fm_port->fifo_bufs.extra) {
		pr_err(" No fifo_bufs.extra for non Rx ports\n");
		return -EINVAL;
	}

	/* Tx only */
	if (fm_port->port_type == FM_PORT_TYPE_TX) {
		if (dflt_config->tx_fifo_min_level % BMI_FIFO_UNITS) {
			pr_err("tx_fifo_min_fill_level has to be divisible by %d\n",
			       BMI_FIFO_UNITS);
			return -EINVAL;
		}
		if (dflt_config->tx_fifo_min_level >
		    (fm_port->port_intg->max_port_fifo_size - 256)) {
			pr_err("tx_fifo_min_fill_level has to be in the range of 0 - %d\n",
			       (fm_port->port_intg->max_port_fifo_size - 256));
			return -EINVAL;
		}
		if (dflt_config->tx_fifo_low_comf_level % BMI_FIFO_UNITS) {
			pr_err("tx_fifo_low_comf_level has to be divisible by %d\n",
			       BMI_FIFO_UNITS);
			return -EINVAL;
		}
		if ((dflt_config->tx_fifo_low_comf_level < BMI_FIFO_UNITS) ||
		    (dflt_config->tx_fifo_low_comf_level >
		     fm_port->port_intg->max_port_fifo_size)) {
			pr_err("tx_fifo_low_comf_level has to be in the range of 256 - %d\n",
			       fm_port->port_intg->max_port_fifo_size);
			return -EINVAL;
		}
		if (fm_port->port_speed == FM_PORT_SPEED_1G)
			if (fm_port->fm_port_drv_param->dflt_cfg.
			    tx_fifo_deq_pipeline_depth > 2) {
				pr_err("fifoDeqPipelineDepth for 1G can't be larger than 2\n");
				return -EINVAL;
			}
	}

	/* Non Tx Ports */
	/* If discard override was selected , no frames may be discarded. */
	else if (dflt_config->discard_override && params->errors_to_discard) {
		pr_err("errors_to_discard is not empty, but frm_discard_override selected (all discarded frames to be enqueued to error queue).\n");
		return -EINVAL;
	}

	/* Rx */
	if (fm_port->port_type == FM_PORT_TYPE_RX) {
		unused_mask = BMI_STATUS_RX_MASK_UNUSED;

		/* Check that no common bits with BMI_STATUS_MASK_UNUSED */
		if (params->errors_to_discard & unused_mask) {
			pr_err("errors_to_discard contains undefined bits\n");
			return -EINVAL;
		}
	}

	/* All ports */
	/* Check that not larger than 16 */
	if ((params->cheksum_last_bytes_ignore > FRAME_END_DATA_SIZE) &&
	    ((params->cheksum_last_bytes_ignore != DFLT_NOT_SUPPORTED))) {
		pr_err("cheksum_last_bytes_ignore can't be larger than %d\n",
		       FRAME_END_DATA_SIZE);
		return -EINVAL;
	}

	if (fm_sp_check_int_context_params(&params->int_context) != 0)
		return -EINVAL;

	/* common BMI registers values */
	if (params->set_num_of_tasks &&
	    ((!fm_port->tasks.num) ||
	     (fm_port->tasks.num > MAX_NUM_OF_TASKS))) {
		pr_err("tasks.num can't be larger than %d\n",
		       MAX_NUM_OF_TASKS);
		return -EINVAL;
	}
	if (params->set_num_of_tasks &&
	    (fm_port->tasks.extra > MAX_NUM_OF_EXTRA_TASKS)) {
		pr_err("tasks.extra can't be larger than %d\n",
		       MAX_NUM_OF_EXTRA_TASKS);
		return -EINVAL;
	}
	if (params->set_num_of_open_dmas &&
	    ((!fm_port->open_dmas.num) ||
	     (fm_port->open_dmas.num > MAX_NUM_OF_DMAS))) {
		pr_err("open_dmas.num can't be larger than %d\n",
		       MAX_NUM_OF_DMAS);
		return -EINVAL;
	}
	if (params->set_num_of_open_dmas &&
	    (fm_port->open_dmas.extra > MAX_NUM_OF_EXTRA_DMAS)) {
		pr_err("open_dmas.extra can't be larger than %d\n",
		       MAX_NUM_OF_EXTRA_DMAS);
		return -EINVAL;
	}
	if (params->set_size_of_fifo &&
	    (!fm_port->fifo_bufs.num || (fm_port->fifo_bufs.num >
	     fm_port->port_intg->max_port_fifo_size))) {
		pr_err("fifo_bufs.num has to be in the range of 256 - %d\n",
		       fm_port->port_intg->max_port_fifo_size);
		return -EINVAL;
	}
	if (params->set_size_of_fifo &&
	    (fm_port->fifo_bufs.num % BMI_FIFO_UNITS)) {
		pr_err("fifo_bufs.num has to be divisible by %d\n",
		       BMI_FIFO_UNITS);
		return -EINVAL;
	}

	return 0;
}

static bool is_init_done(struct fm_port_drv_param_t *fm_port_drv_params)
{
	/* Checks if FMan port driver parameters were initialized */
	if (!fm_port_drv_params)
		return true;

	return false;
}

static int verify_size_of_fifo(struct fm_port_t *fm_port)
{
	u32 min_fifo_size_required = 0, opt_fifo_size_for_b2b = 0;

	/* TX Ports */
	if (fm_port->port_type == FM_PORT_TYPE_TX) {
		min_fifo_size_required = (u32)
		    (roundup(fm_port->max_frame_length,
			     BMI_FIFO_UNITS) + (3 * BMI_FIFO_UNITS));

		min_fifo_size_required +=
		    fm_port->fm_port_drv_param->
		    dflt_cfg.tx_fifo_deq_pipeline_depth * BMI_FIFO_UNITS;

		opt_fifo_size_for_b2b = min_fifo_size_required;

		/* Add some margin for back-to-back capability to improve
		 * performance, allows the hardware to pipeline new frame dma
		 * while the previous frame not yet transmitted.
		 */
		if (fm_port->port_speed == FM_PORT_SPEED_10G)
			opt_fifo_size_for_b2b += 3 * BMI_FIFO_UNITS;
		else
			opt_fifo_size_for_b2b += 2 * BMI_FIFO_UNITS;
	}

	/* RX Ports */
	else if (fm_port->port_type == FM_PORT_TYPE_RX) {
		if (fm_port->fm_rev_info.major_rev >= 6)
			min_fifo_size_required = (u32)
			(roundup(fm_port->max_frame_length,
				 BMI_FIFO_UNITS) + (5 * BMI_FIFO_UNITS));
			/* 4 according to spec + 1 for FOF>0 */
		else
			min_fifo_size_required = (u32)
			(roundup(min(fm_port->max_frame_length,
				     fm_port->rx_pools_params.largest_buf_size),
				     BMI_FIFO_UNITS) + (7 * BMI_FIFO_UNITS));

		opt_fifo_size_for_b2b = min_fifo_size_required;

		/* Add some margin for back-to-back capability to improve
		 * performance,allows the hardware to pipeline new frame dma
		 * while the previous frame not yet transmitted.
		 */
		if (fm_port->port_speed == FM_PORT_SPEED_10G)
			opt_fifo_size_for_b2b += 8 * BMI_FIFO_UNITS;
		else
			opt_fifo_size_for_b2b += 3 * BMI_FIFO_UNITS;
	}

	BUG_ON(min_fifo_size_required <= 0);
	BUG_ON(opt_fifo_size_for_b2b < min_fifo_size_required);

	/* Verify the size  */
	if (fm_port->fifo_bufs.num < min_fifo_size_required)
		pr_debug("FIFO size should be enlarged to %d bytes\n",
			 min_fifo_size_required);
	else if (fm_port->fifo_bufs.num < opt_fifo_size_for_b2b)
		pr_debug("For b2b processing,FIFO may be enlarged to %d bytes\n",
			 opt_fifo_size_for_b2b);

	return 0;
}

static void fm_port_drv_param_free(struct fm_port_t *fm_port)
{
	kfree(fm_port->fm_port_drv_param);
	fm_port->fm_port_drv_param = NULL;
}

static int set_ext_buffer_pools(struct fm_port_t *fm_port)
{
	struct fm_ext_pools_t *ext_buf_pools =
	&fm_port->fm_port_drv_param->ext_buf_pools;
	struct fm_buf_pool_depletion_t *buf_pool_depletion =
	&fm_port->fm_port_drv_param->buf_pool_depletion;
	u8 ordered_array[FM_PORT_MAX_NUM_OF_EXT_POOLS];
	u16 sizes_array[BM_MAX_NUM_OF_POOLS];
	int i = 0, j = 0, err;
	struct fman_port_bpools bpools;

	memset(&ordered_array, 0, sizeof(u8) * FM_PORT_MAX_NUM_OF_EXT_POOLS);
	memset(&sizes_array, 0, sizeof(u16) * BM_MAX_NUM_OF_POOLS);
	memcpy(&fm_port->ext_buf_pools, ext_buf_pools,
	       sizeof(struct fm_ext_pools_t));

	fm_sp_set_buf_pools_in_asc_order_of_buf_sizes(ext_buf_pools,
						      ordered_array,
						      sizes_array);

	/* Prepare flibs bpools structure */
	memset(&bpools, 0, sizeof(struct fman_port_bpools));
	bpools.count = ext_buf_pools->num_of_pools_used;
	bpools.counters_enable = true;
	for (i = 0; i < ext_buf_pools->num_of_pools_used; i++) {
		bpools.bpool[i].bpid = ordered_array[i];
		bpools.bpool[i].size = sizes_array[ordered_array[i]];
		/* functionality available only for some derivatives
		 * (limited by config)
		 */
		if (fm_port->fm_port_drv_param->backup_bm_pools)
			for (j = 0; j < fm_port->fm_port_drv_param->
			     backup_bm_pools->num_of_backup_pools; j++)
				if (ordered_array[i] ==
				    fm_port->fm_port_drv_param->
				    backup_bm_pools->pool_ids[j]) {
					bpools.bpool[i].is_backup = true;
					break;
				}
	}

	/* save pools parameters for later use */
	fm_port->rx_pools_params.num_of_pools =
	    ext_buf_pools->num_of_pools_used;
	fm_port->rx_pools_params.largest_buf_size =
	    sizes_array[ordered_array[ext_buf_pools->num_of_pools_used - 1]];
	fm_port->rx_pools_params.second_largest_buf_size =
	    sizes_array[ordered_array[ext_buf_pools->num_of_pools_used - 2]];

	/* FMBM_RMPD reg. - pool depletion */
	if (buf_pool_depletion->pools_grp_mode_enable) {
		bpools.grp_bp_depleted_num = buf_pool_depletion->num_of_pools;
		for (i = 0; i < fm_port->port_intg->bm_max_num_of_pools;
		     i++) {
			if (buf_pool_depletion->pools_to_consider[i]) {
				for (j = 0; j < ext_buf_pools->
				     num_of_pools_used; j++) {
					if (i == ordered_array[j]) {
						bpools.bpool[j].
						    grp_bp_depleted = true;
						break;
					}
				}
			}
		}
	}

	if (buf_pool_depletion->single_pool_mode_enable) {
		for (i = 0; i < fm_port->port_intg->bm_max_num_of_pools; i++) {
			if (buf_pool_depletion->
			    pools_to_consider_for_single_mode[i]) {
				for (j = 0; j < ext_buf_pools->
				     num_of_pools_used; j++) {
					if (i == ordered_array[j]) {
						bpools.bpool[j].
						    single_bp_depleted = true;
						break;
					}
				}
			}
		}
	}

	/* Issue flibs function */
	err = fman_port_set_bpools(&fm_port->port, &bpools);
	if (err != 0) {
		pr_err("fman_port_set_bpools\n");
		return -EINVAL;
	}

	kfree(fm_port->fm_port_drv_param->backup_bm_pools);

	return 0;
}

static int init_low_level_driver(struct fm_port_t *fm_port)
{
	struct fm_port_drv_param_t *drv_params = fm_port->fm_port_drv_param;
	struct fman_port_params port_params;
	u32 tmp_val;

	/* Set up flibs parameters and issue init function */
	memset(&port_params, 0, sizeof(struct fman_port_params));
	port_params.discard_mask = drv_params->errors_to_discard;
	port_params.dflt_fqid = drv_params->dflt_fqid;
	port_params.err_fqid = drv_params->err_fqid;
	port_params.deq_sp = drv_params->deq_sub_portal;
	port_params.dont_release_buf = drv_params->dont_release_buf;
	switch (fm_port->port_type) {
	case FM_PORT_TYPE_RX:
		port_params.err_mask =
			(RX_ERRS_TO_ENQ & ~port_params.discard_mask);
		if (drv_params->forward_reuse_int_context)
			drv_params->dflt_cfg.rx_fd_bits =
			    (u8)(BMI_PORT_RFNE_FRWD_RPD >> 24);
		break;
	default:
		break;
	}

	tmp_val = (u32)((fm_port->internal_buf_offset % OFFSET_UNITS) ?
		(fm_port->internal_buf_offset / OFFSET_UNITS + 1) :
		(fm_port->internal_buf_offset / OFFSET_UNITS));
	fm_port->internal_buf_offset = (u8)(tmp_val * OFFSET_UNITS);
	drv_params->dflt_cfg.int_buf_start_margin =
	    fm_port->internal_buf_offset;
	drv_params->dflt_cfg.ext_buf_start_margin =
	    drv_params->buf_margins.start_margins;
	drv_params->dflt_cfg.ext_buf_end_margin =
	    drv_params->buf_margins.end_margins;

	drv_params->dflt_cfg.ic_ext_offset =
	    drv_params->int_context.ext_buf_offset;
	drv_params->dflt_cfg.ic_int_offset =
	    drv_params->int_context.int_context_offset;
	drv_params->dflt_cfg.ic_size = drv_params->int_context.size;

	if (0 !=
	    fman_port_init(&fm_port->port, &drv_params->dflt_cfg,
			   &port_params)) {
		pr_err("fman_port_init\n");
		return -ENODEV;
	}

	/* The code bellow is a trick so the FM will not release the buffer
	 * to BM nor will try to enqueue the frame to QM
	 */
	if (fm_port->port_type == FM_PORT_TYPE_TX) {
		if (!drv_params->dflt_fqid && drv_params->dont_release_buf) {
			/* override fmbm_tcfqid 0 with a false non-0 value.
			 * This will force FM to act according to tfene.
			 * Otherwise, if fmbm_tcfqid is 0 the FM will release
			 * buffers to BM regardless of fmbm_tfene
			 */
			out_be32(&fm_port->port.bmi_regs->tx.fmbm_tcfqid,
				 0xFFFFFF);
			out_be32(&fm_port->port.bmi_regs->tx.fmbm_tfene,
				 NIA_ENG_BMI | NIA_BMI_AC_TX_RELEASE);
		}
	}

	return 0;
}

static struct fm_port_intg_t *set_port_intg_params(struct fm_port_t *fm_port)
{
	struct fm_port_intg_t *intg;
	u32 bmi_max_fifo_size;

	intg = kzalloc(sizeof(*intg), GFP_KERNEL);
	if (!intg)
		return NULL;

	bmi_max_fifo_size = fm_get_bmi_max_fifo_size(fm_port->fm);

	intg->max_port_fifo_size =
				MAX_PORT_FIFO_SIZE(bmi_max_fifo_size);

	switch (fm_port->fm_rev_info.major_rev) {
	case FM_IP_BLOCK_P2_P3_P5:
	case FM_IP_BLOCK_P4:
		intg->max_num_of_ext_pools		= 4;
		intg->fm_max_num_of_sub_portals	= 12;
		intg->bm_max_num_of_pools		= 64;
		break;

	case FM_IP_BLOCK_B_T:
		intg->max_num_of_ext_pools		= 8;
		intg->fm_max_num_of_sub_portals	= 16;
		intg->bm_max_num_of_pools		= 64;
		break;

	default:
		pr_err("Unsupported FMan version\n");
		kfree(intg);
		return NULL;
	}

	return intg;
}

struct fm_port_t *fm_port_config(struct fm_port_params_t *fm_port_params)
{
	struct fm_port_t *fm_port;
	void __iomem *base_addr = fm_port_params->base_addr;
	enum fman_port_type fman_port_type = E_FMAN_PORT_TYPE_DUMMY;
	u32 tmp_reg;

	/* Allocate FM structure */
	fm_port = kzalloc(sizeof(*fm_port), GFP_KERNEL);
	if (!fm_port)
		return NULL;

	/* Allocate the FM driver's parameters structure */
	fm_port->fm_port_drv_param =
	kzalloc(sizeof(*fm_port->fm_port_drv_param), GFP_KERNEL);
	if (!fm_port->fm_port_drv_param)
		goto err_fm_port_params;

	/* Initialize FM port parameters which will be kept by the driver */
	fm_port->port_type = fm_port_params->port_type;
	fm_port->port_speed = fm_port_params->port_speed;
	fm_port->port_id = fm_port_params->port_id;
	fm_port->fm = fm_port_params->fm;

	/* get FM revision */
	fm_get_revision(fm_port->fm, &fm_port->fm_rev_info);

	fm_port->port_intg = set_port_intg_params(fm_port);
	if (!fm_port->port_intg)
			goto err_fm_port_intg;

	/* Set up FM port parameters for initialization phase only */

	/* In order to be aligned with flib port types, we need to translate
	 * the port type and speed to fman_port_type
	 */
	if (fm_port->port_type == FM_PORT_TYPE_TX) {
		if (fm_port->port_speed == FM_PORT_SPEED_10G)
			fman_port_type = E_FMAN_PORT_TYPE_TX_10G;
		else
			fman_port_type = E_FMAN_PORT_TYPE_TX;
	} else if (fm_port->port_type == FM_PORT_TYPE_RX) {
		if (fm_port->port_speed == FM_PORT_SPEED_10G)
			fman_port_type = E_FMAN_PORT_TYPE_RX_10G;
		else
			fman_port_type = E_FMAN_PORT_TYPE_RX;
	}
	fman_port_defconfig(&fm_port->fm_port_drv_param->dflt_cfg,
			    fman_port_type);
	/* Overwrite some integration specific parameters */
	fm_port->fm_port_drv_param->dflt_cfg.rx_pri_elevation =
	DFLT_PORT_RX_FIFO_PRI_ELEVATION_LEV(
				fm_port->port_intg->max_port_fifo_size);
	fm_port->fm_port_drv_param->dflt_cfg.rx_fifo_thr =
	fm_port->fm_port_drv_param->rx_fifo_threshold =
	DFLT_PORT_RX_FIFO_THRESHOLD(fm_port->fm_rev_info.major_rev,
				    fm_port->port_intg->max_port_fifo_size);

	fm_port->fm_port_drv_param->dflt_cfg.errata_A006675 = false;

	if ((fm_port->fm_rev_info.major_rev == 6) &&
	    ((fm_port->fm_rev_info.minor_rev == 0) ||
	     (fm_port->fm_rev_info.minor_rev == 3)))
		fm_port->fm_port_drv_param->dflt_cfg.errata_A006320 = true;
	else
		fm_port->fm_port_drv_param->dflt_cfg.errata_A006320 = false;

	/* Excessive Threshold register - exists for pre-FMv3 chips only */
	if (fm_port->fm_rev_info.major_rev < 6) {
		fm_port->fm_port_drv_param->dflt_cfg.
		excessive_threshold_register = true;

		fm_port->fm_port_drv_param->dflt_cfg.fmbm_rebm_has_sgd =
		    false;
		fm_port->fm_port_drv_param->dflt_cfg.fmbm_tfne_has_features =
		    false;
	} else {
		fm_port->fm_port_drv_param->dflt_cfg.
		excessive_threshold_register = false;
		fm_port->fm_port_drv_param->dflt_cfg.fmbm_rebm_has_sgd =
		true;
		fm_port->fm_port_drv_param->dflt_cfg.fmbm_tfne_has_features =
		true;
	}

	fm_port->fm_port_drv_param->dflt_cfg.qmi_deq_options_support = true;

	/* Continue with other parameters */
	fm_port->fm_port_drv_param->base_addr = base_addr;
	/* set memory map pointers */
	fm_port->fm_port_bmi_regs = (union fm_port_bmi_regs_u __iomem *)
		(base_addr + BMI_PORT_REGS_OFFSET);

	fm_port->fm_port_drv_param->buffer_prefix_content.priv_data_size =
	DFLT_PORT_BUFFER_PREFIX_CONTENT_PRIV_DATA_SIZE;
	fm_port->fm_port_drv_param->buffer_prefix_content.pass_prs_result =
	DFLT_PORT_BUFFER_PREFIX_CONTENT_PASS_PRS_RESULT;
	fm_port->fm_port_drv_param->buffer_prefix_content.pass_time_stamp =
	DFLT_PORT_BUFFER_PREFIX_CONTENT_PASS_TIME_STAMP;
	fm_port->fm_port_drv_param->buffer_prefix_content.data_align =
	DFLT_PORT_BUFFER_PREFIX_CONTEXT_DATA_ALIGN;
	fm_port->fm_port_drv_param->liodn_base = fm_port_params->liodn_base;
	fm_port->fm_port_drv_param->cheksum_last_bytes_ignore =
	DFLT_PORT_CHECKSUM_LAST_BYTES_IGNORE;

	fm_port->max_frame_length = DFLT_PORT_MAX_FRAME_LENGTH;
	/* resource distribution. */

	fm_port->fifo_bufs.num =
	fm_port_dflt_num_of_fifo_bufs(fm_port->fm_rev_info.major_rev,
				      fm_port->port_type,
				      fm_port->port_speed) * BMI_FIFO_UNITS;
	fm_port->fifo_bufs.extra =
	DFLT_PORT_EXTRA_NUM_OF_FIFO_BUFS * BMI_FIFO_UNITS;

	fm_port->open_dmas.num =
	fm_port_dflt_num_of_open_dmas(fm_port->fm_rev_info.major_rev,
				      fm_port->port_type,
				      fm_port->port_speed);
	fm_port->open_dmas.extra =
	fm_port_dflt_extra_num_of_open_dmas(fm_port->fm_rev_info.major_rev,
					    fm_port->port_type,
					    fm_port->port_speed);
	fm_port->tasks.num =
	fm_port_dflt_num_of_tasks(fm_port->fm_rev_info.major_rev,
				  fm_port->port_type,
				  fm_port->port_speed);
	fm_port->tasks.extra =
	fm_port_dflt_extra_num_of_tasks(fm_port->fm_rev_info.major_rev,
					fm_port->port_type,
					fm_port->port_speed);

	/* FM_HEAVY_TRAFFIC_SEQUENCER_HANG_ERRATA_FMAN_A006981 errata
	 * workaround
	 */
	if ((fm_port->fm_rev_info.major_rev == 6) &&
	    (fm_port->fm_rev_info.minor_rev == 0) &&
	    (((fm_port->port_type == FM_PORT_TYPE_TX) &&
	    (fm_port->port_speed == FM_PORT_SPEED_1G)))) {
		fm_port->open_dmas.num = 16;
		fm_port->open_dmas.extra = 0;
	}

	/* Port type specific initialization: */
	switch (fm_port->port_type) {
	case FM_PORT_TYPE_RX:
		/* Initialize FM port parameters for initialization
		 * phase only
		 */
		fm_port->fm_port_drv_param->cut_bytes_from_end =
		DFLT_PORT_CUT_BYTES_FROM_END;
		fm_port->fm_port_drv_param->en_buf_pool_depletion = false;
		fm_port->fm_port_drv_param->frm_discard_override =
		DFLT_PORT_FRM_DISCARD_OVERRIDE;

		fm_port->fm_port_drv_param->rx_fifo_pri_elevation_level =
		DFLT_PORT_RX_FIFO_PRI_ELEVATION_LEV(fm_port->port_intg->
						    max_port_fifo_size);
		fm_port->fm_port_drv_param->rx_fifo_threshold =
		DFLT_PORT_RX_FIFO_THRESHOLD(fm_port->fm_rev_info.major_rev,
					    fm_port->port_intg->
					    max_port_fifo_size);

		fm_port->fm_port_drv_param->buf_margins.end_margins =
		DFLT_PORT_BUF_MARGINS_END_MAARGINS;
		fm_port->fm_port_drv_param->errors_to_discard =
		DFLT_PORT_ERRORS_TO_DISCARD;
		fm_port->fm_port_drv_param->forward_reuse_int_context =
		DFLT_PORT_FORWARD_INT_CONTENT_REUSE;
		break;

	case FM_PORT_TYPE_TX:
		if (fm_port->port_speed == FM_PORT_SPEED_1G) {
			fm_port->fm_port_drv_param->dont_release_buf = false;
			/* FM_WRONG_RESET_VALUES_ERRATA_FMAN_A005127 Errata
			 * workaround
			 */
			if (fm_port->fm_rev_info.major_rev >= 6) {
				tmp_reg = 0x00001013;
				out_be32(&fm_port->fm_port_bmi_regs->
					 tx_port_bmi_regs.fmbm_tfp,
					 tmp_reg);
			}
		}
		if (fm_port->port_speed == FM_PORT_SPEED_10G) {
			fm_port->fm_port_drv_param->tx_fifo_min_fill_level =
			DFLT_PORT_TX_FIFO_MIN_FILL_LEVEL;
			fm_port->fm_port_drv_param->tx_fifo_low_comf_level =
			DFLT_PORT_TX_FIFO_LOW_COMF_LEVEL;

			fm_port->fm_port_drv_param->deq_type =
			DFLT_PORT_DEQ_TYPE;
			fm_port->fm_port_drv_param->deq_prefetch_option =
			DFLT_PORT_DEQ_PREFETCH_OPT;
		}
		fm_port->fm_port_drv_param->deq_high_priority =
		DFLT_PORT_DEQ_HIGH_PRIORITY(fm_port->port_speed);
		fm_port->fm_port_drv_param->deq_byte_cnt =
		DFLT_PORT_DEQ_BYTE_CNT(fm_port->port_speed);
		fm_port->fm_port_drv_param->dflt_cfg.
			tx_fifo_deq_pipeline_depth =
			fm_port_dflt_fifo_deq_pipeline_depth(
				     fm_port->fm_rev_info.major_rev,
				     fm_port->port_type,
				     fm_port->port_speed);
		break;
	default:
		pr_err("Invalid port type\n");
		goto err_fm_port;
	}

	switch (fm_port->port_type) {
	case FM_PORT_TYPE_RX:
		/* Initialize FM port parameters for initialization
		 * phase only
		 */
		memcpy(&fm_port->fm_port_drv_param->ext_buf_pools,
		       &fm_port_params->
			specific_params.rx_params.ext_buf_pools,
		       sizeof(struct fm_ext_pools_t));
		fm_port->fm_port_drv_param->err_fqid =
		fm_port_params->specific_params.rx_params.err_fqid;
		fm_port->fm_port_drv_param->dflt_fqid =
		fm_port_params->specific_params.rx_params.dflt_fqid;
		fm_port->fm_port_drv_param->liodn_offset =
		fm_port_params->specific_params.rx_params.liodn_offset;
		break;
	case FM_PORT_TYPE_TX:
		fm_port->fm_port_drv_param->err_fqid =
		fm_port_params->specific_params.non_rx_params.err_fqid;
		fm_port->fm_port_drv_param->deq_sub_portal =
		(u8)(fm_port_params->specific_params.non_rx_params.
		qm_channel & QMI_DEQ_CFG_SUBPORTAL_MASK);
		fm_port->fm_port_drv_param->dflt_fqid =
		fm_port_params->specific_params.non_rx_params.dflt_fqid;
		break;
	default:
		pr_err("Invalid port type\n");
		goto err_fm_port;
	}

	memset(fm_port->name, 0, (sizeof(char)) * MODULE_NAME_SIZE);
	if (snprintf(fm_port->name, MODULE_NAME_SIZE, "FM-%d-port-%s-%d",
		     fm_get_id(fm_port->fm),
		     ((fm_port->port_type == FM_PORT_TYPE_TX) ?
		     ((fm_port->port_speed == FM_PORT_SPEED_10G) ? "10g-TX" :
		     "1g-TX") :
		     ((fm_port->port_speed == FM_PORT_SPEED_10G) ? "10g-RX" :
		     "1g-RX")),
		     fm_port->port_id) == 0) {
		pr_err("sprintf failed\n");
		goto err_fm_port;
	}

	return fm_port;

err_fm_port:
	kfree(fm_port->port_intg);
err_fm_port_intg:
	kfree(fm_port->fm_port_drv_param);
err_fm_port_params:
	kfree(fm_port);
	return NULL;
}

int fm_port_init(struct fm_port_t *fm_port)
{
	struct fm_port_drv_param_t *drv_params;
	int err;
	struct fm_inter_module_port_init_params_t fm_params;
	struct fm_revision_info_t rev_info;
	enum fman_port_type fman_port_type = E_FMAN_PORT_TYPE_DUMMY;

	if (is_init_done(fm_port->fm_port_drv_param))
		return -EINVAL;

	err = fm_sp_build_buffer_structure(&fm_port->fm_port_drv_param->
					   int_context,
					   &fm_port->fm_port_drv_param->
					   buffer_prefix_content,
					   &fm_port->fm_port_drv_param->
					   buf_margins,
					   &fm_port->buffer_offsets,
					   &fm_port->internal_buf_offset);
	if (err)
		return err;

	/* FM_HEAVY_TRAFFIC_HANG_ERRATA_FMAN_A005669 Errata workaround */
	if (fm_port->fm_rev_info.major_rev >= 6 &&
	    (fm_port->fm_port_drv_param->bcb_workaround) &&
	    ((fm_port->port_type == FM_PORT_TYPE_RX) &&
	    (fm_port->port_speed == FM_PORT_SPEED_1G))) {
		fm_port->fm_port_drv_param->errors_to_discard |=
		    FM_PORT_FRM_ERR_PHYSICAL;
		if (!fm_port->fifo_bufs.num)
			fm_port->fifo_bufs.num =
				fm_port_dflt_num_of_fifo_bufs(
					fm_port->fm_rev_info.major_rev,
					fm_port->port_type,
					fm_port->port_speed) *
					BMI_FIFO_UNITS;
		fm_port->fifo_bufs.num += 4 * 1024;
	}

	err = check_init_parameters(fm_port);
	if (err)
		return err;

	drv_params = fm_port->fm_port_drv_param;

	memset(&fm_port->port, 0, sizeof(struct fman_port));
	/* In order to be aligned with flib port types, we need to translate
	 * the port type and speed to fman_port_type
	 */
	if (fm_port->port_type == FM_PORT_TYPE_TX) {
		if (fm_port->port_speed == FM_PORT_SPEED_10G)
			fman_port_type = E_FMAN_PORT_TYPE_TX_10G;
		else
			fman_port_type = E_FMAN_PORT_TYPE_TX;
	} else if (fm_port->port_type == FM_PORT_TYPE_RX) {
		if (fm_port->port_speed == FM_PORT_SPEED_10G)
			fman_port_type = E_FMAN_PORT_TYPE_RX_10G;
		else
			fman_port_type = E_FMAN_PORT_TYPE_RX;
	}
	fm_port->port.type = fman_port_type;
	fm_get_revision(fm_port->fm, &rev_info);
	fm_port->port.fm_rev_maj = rev_info.major_rev;
	fm_port->port.fm_rev_min = rev_info.minor_rev;
	fm_port->port.bmi_regs = (union fman_port_bmi_regs __iomem *)
		(drv_params->base_addr + BMI_PORT_REGS_OFFSET);
	fm_port->port.qmi_regs = (struct fman_port_qmi_regs __iomem *)
		(drv_params->base_addr + QMI_PORT_REGS_OFFSET);
	fm_port->port.ext_pools_num = (u8)8;

	if (fm_port->port_type == FM_PORT_TYPE_RX) {
		/* Call the external Buffer routine which also checks fifo
		 * size and updates it if necessary
		 */
		/* define external buffer pools and pool depletion */
		err = set_ext_buffer_pools(fm_port);
		if (err)
			return err;
		/* check if the largest external buffer pool is large enough */
		if (drv_params->buf_margins.start_margins + MIN_EXT_BUF_SIZE +
		    drv_params->buf_margins.end_margins >
		    fm_port->rx_pools_params.largest_buf_size) {
			pr_err("buf_margins.start_margins (%d) + minimum buf size (64) + buf_margins.end_margins (%d) is larger than maximum external buffer size (%d)\n",
			       drv_params->buf_margins.start_margins,
			       drv_params->buf_margins.end_margins,
			       fm_port->rx_pools_params.largest_buf_size);
			return -EINVAL;
		}
	}

	/* Call FM module routine for communicating parameters */
	memset(&fm_params, 0, sizeof(fm_params));
	fm_params.port_id = fm_port->port_id;
	fm_params.port_type = (enum fm_port_type)fm_port->port_type;
	fm_params.port_speed = (enum fm_port_speed)fm_port->port_speed;
	fm_params.num_of_tasks = (u8)fm_port->tasks.num;
	fm_params.num_of_extra_tasks = (u8)fm_port->tasks.extra;
	fm_params.num_of_open_dmas = (u8)fm_port->open_dmas.num;
	fm_params.num_of_extra_open_dmas = (u8)fm_port->open_dmas.extra;

	if (fm_port->fifo_bufs.num) {
		err = verify_size_of_fifo(fm_port);
		if (err != 0)
			return -err;
	}
	fm_params.size_of_fifo = fm_port->fifo_bufs.num;
	fm_params.extra_size_of_fifo = fm_port->fifo_bufs.extra;
	fm_params.liodn_offset = drv_params->liodn_offset;
	fm_params.liodn_base = drv_params->liodn_base;
	fm_params.deq_pipeline_depth =
	fm_port->fm_port_drv_param->dflt_cfg.tx_fifo_deq_pipeline_depth;
	fm_params.max_frame_length = fm_port->max_frame_length;

	err = fm_get_set_port_params(fm_port->fm, &fm_params);
	if (err)
		return -err;

	err = init_low_level_driver(fm_port);
	if (err != 0)
		return -err;

	fm_port_drv_param_free(fm_port);

	return 0;
}

int fm_port_cfg_deq_high_priority(struct fm_port_t *fm_port, bool high_pri)
{
	if (is_init_done(fm_port->fm_port_drv_param))
		return -EINVAL;

	if (fm_port->port_type == FM_PORT_TYPE_RX) {
		pr_err("cfg_deq_high_priority() not available for Rx ports\n");
		return -ENOMEM;
	}

	fm_port->fm_port_drv_param->dflt_cfg.deq_high_pri = high_pri;

	return 0;
}

int fm_port_cfg_deq_prefetch_option(struct fm_port_t *fm_port,
				    enum fm_port_deq_prefetch_option
				    deq_prefetch_option)
{
	if (is_init_done(fm_port->fm_port_drv_param))
		return -EINVAL;

	if (fm_port->port_type == FM_PORT_TYPE_RX) {
		pr_err("fm_port_cfg_deq_prefetch_option not available for Rx ports\n");
		return -ENODEV;
	}
	fm_port->fm_port_drv_param->dflt_cfg.deq_prefetch_opt =
	(enum fman_port_deq_prefetch)deq_prefetch_option;

	return 0;
}

int fm_port_cfg_buf_prefix_content(struct fm_port_t *fm_port,
				   struct fm_buffer_prefix_content_t *
				   fm_buffer_prefix_content)
{
	if (is_init_done(fm_port->fm_port_drv_param))
		return -EINVAL;

	memcpy(&fm_port->fm_port_drv_param->buffer_prefix_content,
	       fm_buffer_prefix_content,
	       sizeof(struct fm_buffer_prefix_content_t));
	/* if data_align was not initialized by user,
	 * we return to driver's default
	 */
	if (!fm_port->fm_port_drv_param->buffer_prefix_content.data_align)
		fm_port->fm_port_drv_param->buffer_prefix_content.data_align =
		DFLT_PORT_BUFFER_PREFIX_CONTEXT_DATA_ALIGN;

	return 0;
}

int fm_port_cfg_bcb_wa(struct fm_port_t *fm_port)
{
	if (is_init_done(fm_port->fm_port_drv_param))
		return -EINVAL;

	fm_port->fm_port_drv_param->bcb_workaround = true;

	return 0;
}

int fm_port_disable(struct fm_port_t *fm_port)
{
	int err;

	if (!is_init_done(fm_port->fm_port_drv_param))
		return -EINVAL;

	err = fman_port_disable(&fm_port->port);
	if (err == -EBUSY) {
		pr_debug("%s: BMI or QMI is Busy. Port forced down\n",
			 fm_port->name);
		err = 0;
	}

	fm_port->enabled = false;

	return err;
}
EXPORT_SYMBOL(fm_port_disable);

int fm_port_enable(struct fm_port_t *fm_port)
{
	int err;

	if (!is_init_done(fm_port->fm_port_drv_param))
		return -EINVAL;

	/* Used by fm_port_free routine as indicationif to disable port.
	 * Thus set it to true prior to enabling itself.
	 * This way if part of enable process fails there will be still
	 * things to disable during Free.
	 * For example, if BMI enable succeeded but QMI failed, still BMI
	 * needs to be disabled by Free.
	 */
	fm_port->enabled = true;

	err = fman_port_enable(&fm_port->port);

	return err;
}
EXPORT_SYMBOL(fm_port_enable);
