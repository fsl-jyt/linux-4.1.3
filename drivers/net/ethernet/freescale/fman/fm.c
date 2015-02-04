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
#include "fm.h"
#include "fm_muram_ext.h"
#include <asm/mpc85xx.h>
#include "fsl_fman.h"

#include <linux/string.h>
#include <linux/slab.h>

static struct fm_intg_t *fill_intg_params(u8 major, u8 minor)
{
	struct fm_intg_t *intg;

	intg = kzalloc(sizeof(*intg), GFP_KERNEL);
	if (!intg)
		return NULL;

	/* P1023 - Major 4
	 * P4080 - Major 2
	 * P2041/P3041/P5020/P5040 - Major 3
	 * Tx/Bx - Major 6
	 */

	switch (major) {
	case FM_IP_BLOCK_P2_P3_P5:
		intg->fm_muram_size		= 160 * 1024;
		intg->fm_iram_size		= 64 * 1024;
		intg->fm_num_of_ctrl		= 2;

		intg->dma_thresh_max_commq	= 31;
		intg->dma_thresh_max_buf	= 127;

		intg->qmi_max_num_of_tnums	= 64;
		intg->qmi_def_tnums_thresh	= 48;

		intg->bmi_max_num_of_tasks	= 128;
		intg->bmi_max_num_of_dmas	= 32;
		intg->port_max_weight		= 16;

		intg->fm_port_num_of_cg		= 256;

		intg->num_of_rx_ports		= 6;
		break;

	case FM_IP_BLOCK_P4:

		intg->fm_muram_size		= 160 * 1024;
		intg->fm_iram_size		= 64 * 1024;
		intg->fm_num_of_ctrl		= 2;

		intg->dma_thresh_max_commq	= 31;
		intg->dma_thresh_max_buf	= 127;

		intg->qmi_max_num_of_tnums	= 64;
		intg->qmi_def_tnums_thresh	= 48;

		intg->bmi_max_num_of_tasks	= 128;
		intg->bmi_max_num_of_dmas	= 32;
		intg->port_max_weight		= 16;

		intg->fm_port_num_of_cg		= 256;

		intg->num_of_rx_ports		= 5;
		break;

	case FM_IP_BLOCK_B_T:
		intg->dma_thresh_max_commq	= 83;
		intg->dma_thresh_max_buf	= 127;

		intg->qmi_max_num_of_tnums	= 64;
		intg->qmi_def_tnums_thresh	= 32;

		intg->port_max_weight		= 16;
		intg->fm_port_num_of_cg		= 256;

		/* FManV3L */
		if (minor == 1 || minor == 4) {
			intg->fm_muram_size		= 192 * 1024;
			intg->fm_num_of_ctrl		= 2;

			intg->bmi_max_num_of_tasks	= 64;
			intg->bmi_max_num_of_dmas	= 32;

			intg->num_of_rx_ports		= 5;

			if (minor == 1)
				intg->fm_iram_size	= 32 * 1024;
			else
				intg->fm_iram_size	= 64 * 1024;
		}
		/* FManV3H */
		else if (minor == 0 || minor == 2 || minor == 3) {
			intg->fm_muram_size		= 384 * 1024;
			intg->fm_iram_size		= 64 * 1024;
			intg->fm_num_of_ctrl		= 4;

			intg->bmi_max_num_of_tasks	= 128;
			intg->bmi_max_num_of_dmas	= 84;

			intg->num_of_rx_ports		= 8;
		} else {
			pr_err("Unsupported FManv3 version\n");
			goto not_supported;
		}

		break;
	default:
		pr_err("Unsupported FMan version\n");
		goto not_supported;
	}

	intg->bmi_max_fifo_size = intg->fm_muram_size;

	return intg;

not_supported:
	kfree(intg);
	return NULL;
}

static bool is_init_done(struct fman_cfg *fm_drv_params)
{
	/* Checks if FMan driver parameters were initialized */
	if (!fm_drv_params)
		return true;

	return false;
}

static void free_init_resources(struct fm_t *fm)
{
	if (fm->cam_offset)
		fm_muram_free_mem(fm->muram, fm->cam_offset, fm->cam_size);
	if (fm->fifo_offset)
		fm_muram_free_mem(fm->muram, fm->fifo_offset, fm->fifo_size);
}

static int check_fm_parameters(struct fm_t *fm)
{
	if (fm->fm_state->rev_info.major_rev < 6) {
		if (!fm->fm_drv_param->dma_axi_dbg_num_of_beats ||
		    (fm->fm_drv_param->dma_axi_dbg_num_of_beats >
			DMA_MODE_MAX_AXI_DBG_NUM_OF_BEATS)) {
			pr_err("axi_dbg_num_of_beats has to be in the range 1 - %d\n",
			       DMA_MODE_MAX_AXI_DBG_NUM_OF_BEATS);
			return -EINVAL;
		}
	}
	if (fm->fm_drv_param->dma_cam_num_of_entries % DMA_CAM_UNITS) {
		pr_err("dma_cam_num_of_entries has to be divisble by %d\n",
		       DMA_CAM_UNITS);
		return -EINVAL;
	}
	if (fm->fm_drv_param->dma_comm_qtsh_asrt_emer >
	    fm->intg->dma_thresh_max_commq) {
		pr_err("dma_comm_qtsh_asrt_emer can not be larger than %d\n",
		       fm->intg->dma_thresh_max_commq);
		return -EINVAL;
	}
	if (fm->fm_drv_param->dma_comm_qtsh_clr_emer >
	    fm->intg->dma_thresh_max_commq) {
		pr_err("dma_comm_qtsh_clr_emer can not be larger than %d\n",
		       fm->intg->dma_thresh_max_commq);
		return -EINVAL;
	}
	if (fm->fm_drv_param->dma_comm_qtsh_clr_emer >=
	    fm->fm_drv_param->dma_comm_qtsh_asrt_emer) {
		pr_err("dma_comm_qtsh_clr_emer must be smaller than dma_comm_qtsh_asrt_emer\n");
		return -EINVAL;
	}
	if (fm->fm_state->rev_info.major_rev < 6) {
		if (fm->fm_drv_param->dma_read_buf_tsh_asrt_emer >
		    fm->intg->dma_thresh_max_buf) {
			pr_err("dma_read_buf_tsh_asrt_emer can not be larger than %d\n",
			       fm->intg->dma_thresh_max_buf);
			return -EINVAL;
		}
		if (fm->fm_drv_param->dma_read_buf_tsh_clr_emer >
		      fm->intg->dma_thresh_max_buf) {
			pr_err("dma_read_buf_tsh_clr_emer can not be larger than %d\n",
			       fm->intg->dma_thresh_max_buf);
			return -EINVAL;
		}
		if (fm->fm_drv_param->dma_read_buf_tsh_clr_emer >=
		      fm->fm_drv_param->dma_read_buf_tsh_asrt_emer) {
			pr_err("dma_read_buf_tsh_clr_emer must be < dma_read_buf_tsh_asrt_emer\n");
			return -EINVAL;
		}
		if (fm->fm_drv_param->dma_write_buf_tsh_asrt_emer >
		      fm->intg->dma_thresh_max_buf) {
			pr_err("dma_write_buf_tsh_asrt_emer can not be larger than %d\n",
			       fm->intg->dma_thresh_max_buf);
			return -EINVAL;
		}
		if (fm->fm_drv_param->dma_write_buf_tsh_clr_emer >
		      fm->intg->dma_thresh_max_buf) {
			pr_err("dma_write_buf_tsh_clr_emer can not be larger than %d\n",
			       fm->intg->dma_thresh_max_buf);
			return -EINVAL;
		}
		if (fm->fm_drv_param->dma_write_buf_tsh_clr_emer >=
		      fm->fm_drv_param->dma_write_buf_tsh_asrt_emer) {
			pr_err("dma_write_buf_tsh_clr_emer has to be less than dma_write_buf_tsh_asrt_emer\n");
			return -EINVAL;
		}
	} else {
		if ((fm->fm_drv_param->dma_dbg_cnt_mode ==
		    E_FMAN_DMA_DBG_CNT_INT_READ_EM) ||
		    (fm->fm_drv_param->dma_dbg_cnt_mode ==
		    E_FMAN_DMA_DBG_CNT_INT_WRITE_EM) ||
		    (fm->fm_drv_param->dma_dbg_cnt_mode ==
		    E_FMAN_DMA_DBG_CNT_RAW_WAR_PROT)) {
			pr_err("dma_dbg_cnt_mode value not supported by this SoC.\n");
			return -EINVAL;
		}
		if ((fm->fm_drv_param->dma_emergency_bus_select ==
		       FM_DMA_MURAM_READ_EMERGENCY) ||
		      (fm->fm_drv_param->dma_emergency_bus_select ==
		       FM_DMA_MURAM_WRITE_EMERGENCY)) {
			pr_err("emergency_bus_select value not supported by this SoC.\n");
			return -EINVAL;
		}
		if (fm->fm_drv_param->dma_stop_on_bus_error) {
			pr_err("dma_stop_on_bus_error not supported by this SoC.\n");
			return -EINVAL;
		}
		/* FM_AID_MODE_NO_TNUM_SW005 Errata workaround */
		if (fm->fm_state->rev_info.major_rev >= 6 &&
		    fm->fm_drv_param->dma_aid_mode !=
		    E_FMAN_DMA_AID_OUT_PORT_ID) {
			pr_err("dma_aid_mode not supported by this SoC.\n");
			return -EINVAL;
		}
		if (fm->fm_drv_param->dma_axi_dbg_num_of_beats) {
			pr_err("dma_axi_dbg_num_of_beats not supported by this SoC.\n");
			return -EINVAL;
		}
	}

	if (!fm->fm_state->fm_clk_freq) {
		pr_err("fm_clk_freq must be set.\n");
		return -EINVAL;
	}
	if ((fm->fm_drv_param->dma_watchdog *
	    fm->fm_state->fm_clk_freq) > DMA_MAX_WATCHDOG) {
		pr_err("dma_watchdog depends on FM clock. dma_watchdog(in microseconds)*clk (in Mhz), may not exceed 0x08%x\n",
		       DMA_MAX_WATCHDOG);
		return -EINVAL;
	}
	if (fm->fm_state->total_fifo_size % BMI_FIFO_UNITS) {
		pr_err("total_fifo_size number has to be divisible by %d\n",
		       BMI_FIFO_UNITS);
	}
	if (!fm->fm_state->total_fifo_size ||
	    (fm->fm_state->total_fifo_size > fm->intg->bmi_max_fifo_size)) {
		pr_err("total_fifo_size (curr - %d) has to be in the range 256 - %d\n",
		       fm->fm_state->total_fifo_size,
		       fm->intg->bmi_max_fifo_size);
		return -EINVAL;
	}
	if (!fm->fm_state->total_num_of_tasks ||
	    (fm->fm_state->total_num_of_tasks >
	    fm->intg->bmi_max_num_of_tasks)) {
		pr_err("total_num_of_tasks number has to be in the range 1 - %d\n",
		       fm->intg->bmi_max_num_of_tasks);
		return -EINVAL;
	}

	if ((fm->fm_state->rev_info.major_rev < 6) &&
	    (!fm->fm_state->max_num_of_open_dmas ||
	     (fm->fm_state->max_num_of_open_dmas >
	     fm->intg->bmi_max_num_of_dmas))) {
		pr_err("max_num_of_open_dmas number has to be in the range 1 - %d\n",
		       fm->intg->bmi_max_num_of_dmas);
		return -EINVAL;
	}

	if (fm->fm_drv_param->disp_limit_tsh > FPM_MAX_DISP_LIMIT) {
		pr_err("disp_limit_tsh can't be greater than %d\n",
		       FPM_MAX_DISP_LIMIT);
		return -EINVAL;
	}
	if (!fm->exception_cb) {
		pr_err("Exceptions callback not provided\n");
		return -EINVAL;
	}
	if (!fm->bus_error_cb) {
		pr_err("Error exceptions callback not provided\n");
		return -EINVAL;
	}
	if ((fm->fm_state->rev_info.major_rev == 2) &&
	    (fm->fm_drv_param->dma_watchdog)) {
		pr_err("watchdog not supported\n");
		return -EINVAL;
	}

	/* FM_ECC_HALT_NO_SYNC_ERRATA_10GMAC_A008 Errata workaround */
	if ((fm->fm_state->rev_info.major_rev < 6) &&
	    (fm->fm_drv_param->halt_on_unrecov_ecc_err)) {
		pr_err("Halt on ecc error not supported\n");
		return -EINVAL;
	}

	if (fm->fm_state->rev_info.major_rev < 6)
		if (fm->fm_drv_param->tnum_aging_period) {
			pr_err("Tnum aging not supported\n");
			return -EINVAL;
		}

	return 0;
}

static void bmi_err_event(struct fm_t *fm)
{
	u32 event;
	struct fman_bmi_regs __iomem *bmi_rg = fm->bmi_regs;

	event = fman_get_bmi_err_event(bmi_rg);

	if (event & BMI_ERR_INTR_EN_STORAGE_PROFILE_ECC)
		fm->exception_cb(fm->dev_id, FM_EX_BMI_STORAGE_PROFILE_ECC);
	if (event & BMI_ERR_INTR_EN_LIST_RAM_ECC)
		fm->exception_cb(fm->dev_id, FM_EX_BMI_LIST_RAM_ECC);
	if (event & BMI_ERR_INTR_EN_STATISTICS_RAM_ECC)
		fm->exception_cb(fm->dev_id, FM_EX_BMI_STATISTICS_RAM_ECC);
	if (event & BMI_ERR_INTR_EN_DISPATCH_RAM_ECC)
		fm->exception_cb(fm->dev_id, FM_EX_BMI_DISPATCH_RAM_ECC);
}

static void qmi_err_event(struct fm_t *fm)
{
	u32 event;
	struct fman_qmi_regs __iomem *qmi_rg = fm->qmi_regs;

	event = fman_get_qmi_err_event(qmi_rg);

	if (event & QMI_ERR_INTR_EN_DOUBLE_ECC)
		fm->exception_cb(fm->dev_id, FM_EX_QMI_DOUBLE_ECC);
	if (event & QMI_ERR_INTR_EN_DEQ_FROM_DEF)
		fm->exception_cb(fm->dev_id, FM_EX_QMI_DEQ_FROM_UNKNOWN_PORTID);
}

static void dma_err_event(struct fm_t *fm)
{
	u32 status;
	struct fman_dma_regs __iomem *dma_rg = fm->dma_regs;

	status = fman_get_dma_err_event(dma_rg);

	if (status & DMA_STATUS_FM_SPDAT_ECC)
		fm->exception_cb(fm->dev_id, FM_EX_DMA_SINGLE_PORT_ECC);
	if (status & DMA_STATUS_READ_ECC)
		fm->exception_cb(fm->dev_id, FM_EX_DMA_READ_ECC);
	if (status & DMA_STATUS_SYSTEM_WRITE_ECC)
		fm->exception_cb(fm->dev_id, FM_EX_DMA_SYSTEM_WRITE_ECC);
	if (status & DMA_STATUS_FM_WRITE_ECC)
		fm->exception_cb(fm->dev_id, FM_EX_DMA_FM_WRITE_ECC);
}

static void fpm_err_event(struct fm_t *fm)
{
	u32 event;
	struct fman_fpm_regs __iomem *fpm_rg = fm->fpm_regs;

	event = fman_get_fpm_err_event(fpm_rg);

	if ((event & FPM_EV_MASK_DOUBLE_ECC) &&
	    (event & FPM_EV_MASK_DOUBLE_ECC_EN))
		fm->exception_cb(fm->dev_id, FM_EX_FPM_DOUBLE_ECC);
	if ((event & FPM_EV_MASK_STALL) && (event & FPM_EV_MASK_STALL_EN))
		fm->exception_cb(fm->dev_id, FM_EX_FPM_STALL_ON_TASKS);
	if ((event & FPM_EV_MASK_SINGLE_ECC) &&
	    (event & FPM_EV_MASK_SINGLE_ECC_EN))
		fm->exception_cb(fm->dev_id, FM_EX_FPM_SINGLE_ECC);
}

static void muram_err_intr(struct fm_t *fm)
{
	u32 event;
	struct fman_fpm_regs __iomem *fpm_rg = fm->fpm_regs;

	event = fman_get_muram_err_event(fpm_rg);

	if (event & FPM_RAM_MURAM_ECC)
		fm->exception_cb(fm->dev_id, FM_EX_MURAM_ECC);
}

static void qmi_event(struct fm_t *fm)
{
	u32 event;
	struct fman_qmi_regs __iomem *qmi_rg = fm->qmi_regs;

	event = fman_get_qmi_event(qmi_rg);

	if (event & QMI_INTR_EN_SINGLE_ECC)
		fm->exception_cb(fm->dev_id, FM_EX_QMI_SINGLE_ECC);
}

static void enable_time_stamp(struct fm_t *fm)
{
	struct fman_fpm_regs __iomem *fpm_rg = fm->fpm_regs;

	WARN_ON(!fm->fm_state->count1_micro_bit);

	fman_enable_time_stamp(fpm_rg,
			       fm->fm_state->count1_micro_bit,
			       fm->fm_state->fm_clk_freq);

	fm->fm_state->enabled_time_stamp = true;
}

static int clear_iram(struct fm_t *fm)
{
	struct fm_iram_regs_t __iomem *iram;
	int i;

	iram = (struct fm_iram_regs_t __iomem *)(fm->base_addr + FM_MM_IMEM);

	/* Enable the auto-increment */
	out_be32(&iram->iadd, IRAM_IADD_AIE);
	while (in_be32(&iram->iadd) != IRAM_IADD_AIE)
		;

	for (i = 0; i < (fm->intg->fm_iram_size / 4); i++)
		out_be32(&iram->idata, 0xffffffff);

	out_be32(&iram->iadd, fm->intg->fm_iram_size - 4);
	/* Memory barrier */
	mb();
	while (in_be32(&iram->idata) != 0xffffffff)
		;

	return 0;
}

static u32 fm_get_exception_flag(enum fm_exceptions exception)
{
	u32 bit_mask;

	switch (exception) {
	case FM_EX_DMA_BUS_ERROR:
		bit_mask = FM_EX_DMA_BUS_ERROR;
		break;
	case FM_EX_DMA_SINGLE_PORT_ECC:
		bit_mask = FM_EX_DMA_SINGLE_PORT_ECC;
		break;
	case FM_EX_DMA_READ_ECC:
		bit_mask = FM_EX_DMA_READ_ECC;
		break;
	case FM_EX_DMA_SYSTEM_WRITE_ECC:
		bit_mask = FM_EX_DMA_SYSTEM_WRITE_ECC;
		break;
	case FM_EX_DMA_FM_WRITE_ECC:
		bit_mask = FM_EX_DMA_FM_WRITE_ECC;
		break;
	case FM_EX_FPM_STALL_ON_TASKS:
		bit_mask = FM_EX_FPM_STALL_ON_TASKS;
		break;
	case FM_EX_FPM_SINGLE_ECC:
		bit_mask = FM_EX_FPM_SINGLE_ECC;
		break;
	case FM_EX_FPM_DOUBLE_ECC:
		bit_mask = FM_EX_FPM_DOUBLE_ECC;
		break;
	case FM_EX_QMI_SINGLE_ECC:
		bit_mask = FM_EX_QMI_SINGLE_ECC;
		break;
	case FM_EX_QMI_DOUBLE_ECC:
		bit_mask = FM_EX_QMI_DOUBLE_ECC;
		break;
	case FM_EX_QMI_DEQ_FROM_UNKNOWN_PORTID:
		bit_mask = FM_EX_QMI_DEQ_FROM_UNKNOWN_PORTID;
		break;
	case FM_EX_BMI_LIST_RAM_ECC:
		bit_mask = FM_EX_BMI_LIST_RAM_ECC;
		break;
	case FM_EX_BMI_STORAGE_PROFILE_ECC:
		bit_mask = FM_EX_BMI_STORAGE_PROFILE_ECC;
		break;
	case FM_EX_BMI_STATISTICS_RAM_ECC:
		bit_mask = FM_EX_BMI_STATISTICS_RAM_ECC;
		break;
	case FM_EX_BMI_DISPATCH_RAM_ECC:
		bit_mask = FM_EX_BMI_DISPATCH_RAM_ECC;
		break;
	case FM_EX_MURAM_ECC:
		bit_mask = FM_EX_MURAM_ECC;
		break;
	default:
		bit_mask = 0;
		break;
	}

	return bit_mask;
}

static int fm_get_module_event(enum fm_event_modules module, u8 mod_id,
			       enum fm_intr_type intr_type)
{
	int event;

	switch (module) {
	case FM_MOD_MAC:
			event = (intr_type == FM_INTR_TYPE_ERR) ?
			(FM_EV_ERR_MAC0 + mod_id) :
			(FM_EV_MAC0 + mod_id);
		break;
	case FM_MOD_FMAN_CTRL:
		if (intr_type == FM_INTR_TYPE_ERR)
			event = FM_EV_DUMMY_LAST;
		else
			event = (FM_EV_FMAN_CTRL_0 + mod_id);
		break;
	case FM_MOD_DUMMY_LAST:
		event = FM_EV_DUMMY_LAST;
		break;
	default:
		event = FM_EV_DUMMY_LAST;
		break;
	}

	return event;
}

void fm_register_intr(struct fm_t *fm, enum fm_event_modules module,
		      u8 mod_id, enum fm_intr_type intr_type,
		      void (*isr_cb)(void *src_arg), void *src_arg)
{
	int event = 0;

	event = fm_get_module_event(module, mod_id, intr_type);
	WARN_ON(!(event < FM_EV_DUMMY_LAST));

	/* register in local FM structure */
	fm->intr_mng[event].isr_cb = isr_cb;
	fm->intr_mng[event].src_handle = src_arg;
}

void fm_unregister_intr(struct fm_t *fm, enum fm_event_modules module,
			u8 mod_id, enum fm_intr_type intr_type)
{
	int event = 0;

	event = fm_get_module_event(module, mod_id, intr_type);
	WARN_ON(!(event < FM_EV_DUMMY_LAST));

	fm->intr_mng[event].isr_cb = NULL;
	fm->intr_mng[event].src_handle = NULL;
}

u8 fm_get_id(struct fm_t *fm)
{
	return fm->fm_state->fm_id;
}

int fm_reset_mac(struct fm_t *fm, u8 mac_id)
{
	int err;
	struct fman_fpm_regs __iomem *fpm_rg = fm->fpm_regs;

	if (fm->fm_state->rev_info.major_rev >= 6) {
		pr_warn("FMan MAC reset!\n");
		return -EINVAL;
	}
	if (!fm->base_addr) {
		pr_warn("'base_address' is required!\n");
		return -EINVAL;
	}
	err = fman_reset_mac(fpm_rg, mac_id);

	if (err == -EINVAL) {
		pr_warn("Illegal MAC Id\n");
		return -EINVAL;
	} else if (err == EINVAL) {
		return -EINVAL;
	}
	return 0;
}

int fm_set_mac_max_frame(struct fm_t *fm, enum fm_mac_type type,
			 u8 mac_id, u16 mtu)
{
	/* if port is already initialized, check that MaxFrameLength is smaller
	 * or equal to the port's max
	 */
	if ((!fm->fm_state->port_mfl[mac_id]) ||
	    (fm->fm_state->port_mfl[mac_id] &&
	    (mtu <= fm->fm_state->port_mfl[mac_id]))) {
		fm->fm_state->mac_mfl[mac_id] = mtu;
	} else {
		pr_warn("MAC max_frame_length is larger than Port max_frame_length\n");
		return -EINVAL;
	}
	return 0;
}

u16 fm_get_clock_freq(struct fm_t *fm)
{
	return fm->fm_state->fm_clk_freq;
}

u32 fm_get_bmi_max_fifo_size(struct fm_t *fm)
{
	return fm->intg->bmi_max_fifo_size;
}

static int init_fm_dma(struct fm_t *fm)
{
	int err;

	err = fman_dma_init(fm->dma_regs, fm->fm_drv_param);
	if (err != 0)
		return err;

	/* Allocate MURAM for CAM */
	fm->cam_size = (u32)(fm->fm_drv_param->dma_cam_num_of_entries *
			     DMA_CAM_SIZEOF_ENTRY);
	fm->cam_offset = fm_muram_alloc(fm->muram, fm->cam_size);
	if (IS_ERR_VALUE(fm->cam_offset)) {
		pr_err("MURAM alloc for DMA CAM failed\n");
		return -ENOMEM;
	}

	if (fm->fm_state->rev_info.major_rev == 2) {
		u32 __iomem *cam_base_addr;

		fm_muram_free_mem(fm->muram, fm->cam_offset, fm->cam_size);

		fm->cam_size =
			fm->fm_drv_param->dma_cam_num_of_entries * 72 + 128;
		fm->cam_offset = fm_muram_alloc(fm->muram, fm->cam_size);
		if (IS_ERR_VALUE(fm->cam_offset)) {
			pr_err("MURAM alloc for DMA CAM failed\n");
			return -ENOMEM;
		}

		if (fm->fm_drv_param->dma_cam_num_of_entries % 8 ||
		    fm->fm_drv_param->dma_cam_num_of_entries > 32) {
			pr_err("wrong dma_cam_num_of_entries\n");
			return -EINVAL;
		}

		cam_base_addr = (u32 __iomem *)
				fm_muram_offset_to_vbase(fm->muram,
							 fm->cam_offset);
		out_be32(cam_base_addr,
			 ~((1 <<
			 (32 - fm->fm_drv_param->dma_cam_num_of_entries)) - 1));
	}

	fm->fm_drv_param->cam_base_addr = fm->cam_offset;

	return 0;
}

void *fm_config(struct fm_params_t *fm_param)
{
	struct fm_t *fm;
	void __iomem *base_addr;

	base_addr = fm_param->base_addr;

	/* Allocate FM structure */
	fm = kzalloc(sizeof(*fm), GFP_KERNEL);
	if (!fm)
		return NULL;

	fm->fm_state = kzalloc(sizeof(*fm->fm_state), GFP_KERNEL);
	if (!fm->fm_state)
		goto err_fm_state;

	/* Initialize FM parameters which will be kept by the driver */
	fm->fm_state->fm_id = fm_param->fm_id;

	/* Allocate the FM driver's parameters structure */
	fm->fm_drv_param = kzalloc(sizeof(*fm->fm_drv_param), GFP_KERNEL);
	if (!fm->fm_drv_param)
		goto err_fm_drv;

	/* Initialize FM parameters which will be kept by the driver */
	fm->fm_state->fm_id = fm_param->fm_id;
	fm->muram = fm_param->muram;
	fm->dev_id = fm_param->dev_id;
	fm->fm_state->fm_clk_freq = fm_param->fm_clk_freq;
	fm->exception_cb = fm_param->exception_cb;
	fm->bus_error_cb = fm_param->bus_error_cb;
	fm->fpm_regs = (struct fman_fpm_regs __iomem *)(base_addr + FM_MM_FPM);
	fm->bmi_regs = (struct fman_bmi_regs __iomem *)(base_addr + FM_MM_BMI);
	fm->qmi_regs = (struct fman_qmi_regs __iomem *)(base_addr + FM_MM_QMI);
	fm->dma_regs = (struct fman_dma_regs __iomem *)(base_addr + FM_MM_DMA);
	fm->base_addr = base_addr;

	spin_lock_init(&fm->spinlock);
	fman_defconfig(fm->fm_drv_param);

	fm->fm_drv_param->qmi_deq_option_support = true;

	fm->fm_state->rams_ecc_enable = false;
	fm->fm_state->extra_fifo_pool_size = 0;
	fm->fm_state->exceptions = DFLT_EXCEPTIONS;
	fm->reset_on_init = DFLT_RESET_ON_INIT;

	/* read revision */
	/* Chip dependent, will be configured in Init */
	fman_get_revision(fm->fpm_regs, &fm->fm_state->rev_info.major_rev,
			  &fm->fm_state->rev_info.minor_rev);

	fm->intg = fill_intg_params(fm->fm_state->rev_info.major_rev,
				      fm->fm_state->rev_info.minor_rev);
	if (!fm->intg)
		goto err_fm_intg;

	/* FM_AID_MODE_NO_TNUM_SW005 Errata workaround */
	if (fm->fm_state->rev_info.major_rev >= 6)
		fm->fm_drv_param->dma_aid_mode = FM_DMA_AID_OUT_PORT_ID;

	fm->fm_drv_param->qmi_def_tnums_thresh =
		fm->intg->qmi_def_tnums_thresh;

	fm->fm_state->total_fifo_size = 0;
	fm->fm_state->total_num_of_tasks =
	DFLT_TOTAL_NUM_OF_TASKS(fm->fm_state->rev_info.major_rev,
				fm->fm_state->rev_info.minor_rev,
				fm->intg->bmi_max_num_of_tasks);

	if (fm->fm_state->rev_info.major_rev < 6) {
		fm->fm_state->max_num_of_open_dmas =
		fm->intg->bmi_max_num_of_dmas;
		fm->fm_drv_param->dma_comm_qtsh_clr_emer =
		(u8)DFLT_DMA_COMM_Q_LOW(fm->fm_state->rev_info.major_rev,
					fm->intg->dma_thresh_max_commq);

		fm->fm_drv_param->dma_comm_qtsh_asrt_emer =
		(u8)DFLT_DMA_COMM_Q_HIGH(fm->fm_state->rev_info.major_rev,
					 fm->intg->dma_thresh_max_commq);

		fm->fm_drv_param->dma_cam_num_of_entries =
		DFLT_DMA_CAM_NUM_OF_ENTRIES(fm->fm_state->rev_info.major_rev);

		fm->fm_drv_param->dma_read_buf_tsh_clr_emer =
		DFLT_DMA_READ_INT_BUF_LOW(fm->intg->dma_thresh_max_buf);

		fm->fm_drv_param->dma_read_buf_tsh_asrt_emer =
		DFLT_DMA_READ_INT_BUF_HIGH(fm->intg->dma_thresh_max_buf);

		fm->fm_drv_param->dma_write_buf_tsh_clr_emer =
		DFLT_DMA_WRITE_INT_BUF_LOW(fm->intg->dma_thresh_max_buf);

		fm->fm_drv_param->dma_write_buf_tsh_asrt_emer =
		DFLT_DMA_WRITE_INT_BUF_HIGH(fm->intg->dma_thresh_max_buf);

		fm->fm_drv_param->dma_axi_dbg_num_of_beats =
		DFLT_AXI_DBG_NUM_OF_BEATS;
	}

	fm->fm_drv_param->tnum_aging_period = 0;
	fm->tnum_aging_period = fm->fm_drv_param->tnum_aging_period;

	return fm;

err_fm_intg:
	kfree(fm->fm_drv_param);
err_fm_drv:
	kfree(fm->fm_state);
err_fm_state:
	kfree(fm);
	return NULL;
}

int fm_init(struct fm_t *fm)
{
	struct fman_cfg *fm_drv_param = NULL;
	int err = 0;
	struct fm_revision_info_t rev_info;
	struct fman_rg fman_rg;

	if (is_init_done(fm->fm_drv_param))
		return -EINVAL;

	fman_rg.bmi_rg = fm->bmi_regs;
	fman_rg.qmi_rg = fm->qmi_regs;
	fman_rg.fpm_rg = fm->fpm_regs;
	fman_rg.dma_rg = fm->dma_regs;

	fm->fm_state->count1_micro_bit = FM_TIMESTAMP_1_USEC_BIT;
	fm->fm_drv_param->num_of_fman_ctrl_evnt_regs =
		FM_NUM_OF_FMAN_CTRL_EVENT_REGS;

	/* if user didn't configured total_fifo_size -
	 * (total_fifo_size=0) we configure default
	 * according to chip. otherwise, we use user's configuration.
	 */
	if (fm->fm_state->total_fifo_size == 0) {
		fm->fm_state->total_fifo_size =
		fm_dflt_total_fifo_size(fm->fm_state->rev_info.major_rev,
					fm->fm_state->rev_info.minor_rev);
		if (fm->fm_state->total_fifo_size == 0)
			return -EINVAL;
	}

	err = check_fm_parameters(fm);
	if (err)
		return err;

	fm_drv_param = fm->fm_drv_param;

	fm_get_revision(fm, &rev_info);

	/* clear revision-dependent non existing exception */
	if (rev_info.major_rev < 6)
		fm->fm_state->exceptions &= ~FM_EX_BMI_DISPATCH_RAM_ECC;

	if (rev_info.major_rev >= 6)
		fm->fm_state->exceptions &= ~FM_EX_QMI_SINGLE_ECC;

	/* clear CPG */
	memset_io((void __iomem *)(fm->base_addr + FM_MM_CGP), 0,
		  fm->intg->fm_port_num_of_cg);

	/* Reset the FM if required. */
	if (fm->reset_on_init) {
		if (rev_info.major_rev >= 6) {
			/* Errata A007273 */
			pr_debug("FManV3 reset is not supported!\n");
		} else {
			out_be32(&fm->fpm_regs->fm_rstc, FPM_RSTC_FM_RESET);
			/* Memory barrier */
			mb();
			usleep_range(100, 300);
		}

		if (fman_is_qmi_halt_not_busy_state(fm->qmi_regs)) {
			fman_resume(fm->fpm_regs);
			usleep_range(100, 300);
		}
	}

	if (clear_iram(fm) != 0)
		return -EINVAL;

	fm_drv_param->exceptions = fm->fm_state->exceptions;

	/* Init DMA Registers */

	err = init_fm_dma(fm);
	if (err != 0) {
		free_init_resources(fm);
		return err;
	}

	/* Init FPM Registers */

	err = fman_fpm_init(fm->fpm_regs, fm->fm_drv_param);
	if (err != 0) {
		free_init_resources(fm);
		return err;
	}

	/* define common resources */
	/* allocate MURAM for FIFO according to total size */
	fm->fifo_offset = fm_muram_alloc(fm->muram,
					 fm->fm_state->total_fifo_size);
	if (IS_ERR_VALUE(fm->cam_offset)) {
		free_init_resources(fm);
		pr_err("MURAM alloc for BMI FIFO failed\n");
		return -ENOMEM;
	}

	fm_drv_param->fifo_base_addr = fm->fifo_offset;
	fm_drv_param->total_fifo_size = fm->fm_state->total_fifo_size;
	fm_drv_param->total_num_of_tasks = fm->fm_state->total_num_of_tasks;
	fm_drv_param->clk_freq = fm->fm_state->fm_clk_freq;

	/* Init BMI Registers */
	err = fman_bmi_init(fm->bmi_regs, fm->fm_drv_param);
	if (err != 0) {
		free_init_resources(fm);
		return err;
	}

	/* Init QMI Registers */
	err = fman_qmi_init(fm->qmi_regs, fm->fm_drv_param);
	if (err != 0) {
		free_init_resources(fm);
		return err;
	}

	err = fman_enable(&fman_rg, fm_drv_param);
	if (err != 0)
		return err;

	enable_time_stamp(fm);

	kfree(fm->fm_drv_param);
	fm->fm_drv_param = NULL;

	return 0;
}

int fm_cfg_reset_on_init(struct fm_t *fm, bool enable)
{
	if (is_init_done(fm->fm_drv_param))
		return -EINVAL;

	fm->reset_on_init = enable;

	return 0;
}

int fm_cfg_total_fifo_size(struct fm_t *fm, u32 total_fifo_size)
{
	if (is_init_done(fm->fm_drv_param))
		return -EINVAL;

	fm->fm_state->total_fifo_size = total_fifo_size;

	return 0;
}

void fm_event_isr(struct fm_t *fm)
{
	u32 pending;
	struct fman_fpm_regs __iomem *fpm_rg;

	if (!is_init_done(fm->fm_drv_param))
		return;

	fpm_rg = fm->fpm_regs;

	/* normal interrupts */
	pending = fman_get_normal_pending(fpm_rg);
	if (!pending)
		return;

	if (pending & INTR_EN_QMI)
		qmi_event(fm);

	/* MAC interrupts */
	if (pending & INTR_EN_MAC0)
		fm_call_mac_isr(fm, FM_EV_MAC0 + 0);
	if (pending & INTR_EN_MAC1)
		fm_call_mac_isr(fm, FM_EV_MAC0 + 1);
	if (pending & INTR_EN_MAC2)
		fm_call_mac_isr(fm, FM_EV_MAC0 + 2);
	if (pending & INTR_EN_MAC3)
		fm_call_mac_isr(fm, FM_EV_MAC0 + 3);
	if (pending & INTR_EN_MAC4)
		fm_call_mac_isr(fm, FM_EV_MAC0 + 4);
	if (pending & INTR_EN_MAC5)
		fm_call_mac_isr(fm, FM_EV_MAC0 + 5);
	if (pending & INTR_EN_MAC6)
		fm_call_mac_isr(fm, FM_EV_MAC0 + 6);
	if (pending & INTR_EN_MAC7)
		fm_call_mac_isr(fm, FM_EV_MAC0 + 7);
	if (pending & INTR_EN_MAC8)
		fm_call_mac_isr(fm, FM_EV_MAC0 + 8);
	if (pending & INTR_EN_MAC9)
		fm_call_mac_isr(fm, FM_EV_MAC0 + 9);
}

int fm_error_isr(struct fm_t *fm)
{
	u32 pending;
	struct fman_fpm_regs __iomem *fpm_rg;

	if (!is_init_done(fm->fm_drv_param))
		return -EINVAL;

	fpm_rg = fm->fpm_regs;

	/* error interrupts */
	pending = fman_get_fpm_error_interrupts(fpm_rg);
	if (!pending)
		return -EINVAL;

	if (pending & ERR_INTR_EN_BMI)
		bmi_err_event(fm);
	if (pending & ERR_INTR_EN_QMI)
		qmi_err_event(fm);
	if (pending & ERR_INTR_EN_FPM)
		fpm_err_event(fm);
	if (pending & ERR_INTR_EN_DMA)
		dma_err_event(fm);
	if (pending & ERR_INTR_EN_MURAM)
		muram_err_intr(fm);

	/* MAC error interrupts */
	if (pending & ERR_INTR_EN_MAC0)
		fm_call_mac_isr(fm, FM_EV_ERR_MAC0 + 0);
	if (pending & ERR_INTR_EN_MAC1)
		fm_call_mac_isr(fm, FM_EV_ERR_MAC0 + 1);
	if (pending & ERR_INTR_EN_MAC2)
		fm_call_mac_isr(fm, FM_EV_ERR_MAC0 + 2);
	if (pending & ERR_INTR_EN_MAC3)
		fm_call_mac_isr(fm, FM_EV_ERR_MAC0 + 3);
	if (pending & ERR_INTR_EN_MAC4)
		fm_call_mac_isr(fm, FM_EV_ERR_MAC0 + 4);
	if (pending & ERR_INTR_EN_MAC5)
		fm_call_mac_isr(fm, FM_EV_ERR_MAC0 + 5);
	if (pending & ERR_INTR_EN_MAC6)
		fm_call_mac_isr(fm, FM_EV_ERR_MAC0 + 6);
	if (pending & ERR_INTR_EN_MAC7)
		fm_call_mac_isr(fm, FM_EV_ERR_MAC0 + 7);
	if (pending & ERR_INTR_EN_MAC8)
		fm_call_mac_isr(fm, FM_EV_ERR_MAC0 + 8);
	if (pending & ERR_INTR_EN_MAC9)
		fm_call_mac_isr(fm, FM_EV_ERR_MAC0 + 9);

	return 0;
}

int fm_disable_rams_ecc(struct fm_t *fm)
{
	bool explicit_disable = false;
	struct fman_fpm_regs __iomem *fpm_rg;
	int ret;

	if (!is_init_done(fm->fm_drv_param))
		return ret;

	fpm_rg = fm->fpm_regs;

	if (!fm->fm_state->internal_call)
		explicit_disable = true;
	fm->fm_state->internal_call = false;

	/* if rams are already disabled, or if rams were explicitly enabled and
	 *  are currently called indirectly (not explicitly), ignore this call.
	 */
	if (!fm->fm_state->rams_ecc_enable ||
	    (fm->fm_state->explicit_enable && !explicit_disable))
		return 0;
	if (fm->fm_state->explicit_enable)
		/* This is the case were both explicit are true.
		 * Turn off this flag for cases were following
		 * ramsEnable routines are called
		 */
		fm->fm_state->explicit_enable = false;

	fman_enable_rams_ecc(fpm_rg);
	fm->fm_state->rams_ecc_enable = false;

	return 0;
}

int fm_set_exception(struct fm_t *fm, enum fm_exceptions exception,
		     bool enable)
{
	u32 bit_mask = 0;
	struct fman_rg fman_rg;

	if (!is_init_done(fm->fm_drv_param))
		return -EINVAL;

	fman_rg.bmi_rg = fm->bmi_regs;
	fman_rg.qmi_rg = fm->qmi_regs;
	fman_rg.fpm_rg = fm->fpm_regs;
	fman_rg.dma_rg = fm->dma_regs;

	bit_mask = fm_get_exception_flag(exception);
	if (bit_mask) {
		if (enable)
			fm->fm_state->exceptions |= bit_mask;
		else
			fm->fm_state->exceptions &= ~bit_mask;

		return fman_set_exception(&fman_rg,
					  (enum fman_exceptions)exception,
					  enable);
	} else {
		pr_err("Undefined exception\n");
		return -EINVAL;
	}

	return 0;
}

void fm_get_revision(struct fm_t *fm,
		     struct fm_revision_info_t *fm_rev)
{
	fm_rev->major_rev = fm->fm_state->rev_info.major_rev;
	fm_rev->minor_rev = fm->fm_state->rev_info.minor_rev;
}
