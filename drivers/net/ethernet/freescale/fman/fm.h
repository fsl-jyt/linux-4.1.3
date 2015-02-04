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

#ifndef __FM_H
#define __FM_H

#include "fm_ext.h"

#include "fsl_fman.h"

#include <linux/io.h>

/* Hardware defines */
#define FM_MAX_NUM_OF_HW_PORT_IDS           64
#define FM_MAX_NUM_OF_MACS	10

#define FM_NUM_OF_FMAN_CTRL_EVENT_REGS		4

/* defaults */
#define DFLT_EXCEPTIONS	\
	((FM_EX_DMA_BUS_ERROR)            | \
	(FM_EX_DMA_READ_ECC)              | \
	(FM_EX_DMA_SYSTEM_WRITE_ECC)      | \
	(FM_EX_DMA_FM_WRITE_ECC)          | \
	(FM_EX_FPM_STALL_ON_TASKS)        | \
	(FM_EX_FPM_SINGLE_ECC)            | \
	(FM_EX_FPM_DOUBLE_ECC)            | \
	(FM_EX_QMI_DEQ_FROM_UNKNOWN_PORTID) | \
	(FM_EX_BMI_LIST_RAM_ECC)          | \
	(FM_EX_BMI_STORAGE_PROFILE_ECC)   | \
	(FM_EX_BMI_STATISTICS_RAM_ECC)    | \
	(FM_EX_MURAM_ECC)                 | \
	(FM_EX_BMI_DISPATCH_RAM_ECC)      | \
	(FM_EX_QMI_DOUBLE_ECC)            | \
	(FM_EX_QMI_SINGLE_ECC))

#define DFLT_AXI_DBG_NUM_OF_BEATS            1
#define DFLT_RESET_ON_INIT                 false

#define DFLT_DMA_READ_INT_BUF_LOW(dma_thresh_max_buf)	\
	((dma_thresh_max_buf + 1) / 2)
#define DFLT_DMA_READ_INT_BUF_HIGH(dma_thresh_max_buf)	\
	((dma_thresh_max_buf + 1) * 3 / 4)
#define DFLT_DMA_WRITE_INT_BUF_LOW(dma_thresh_max_buf)	\
	((dma_thresh_max_buf + 1) / 2)
#define DFLT_DMA_WRITE_INT_BUF_HIGH(dma_thresh_max_buf)\
	((dma_thresh_max_buf + 1) * 3 / 4)

#define DMA_COMM_Q_LOW_FMAN_V3		0x2A
#define DMA_COMM_Q_LOW_FMAN_V2(dma_thresh_max_commq)		\
	((dma_thresh_max_commq + 1) / 2)
#define DFLT_DMA_COMM_Q_LOW(major, dma_thresh_max_commq)	\
	((major == 6) ? DMA_COMM_Q_LOW_FMAN_V3 :		\
	DMA_COMM_Q_LOW_FMAN_V2(dma_thresh_max_commq))

#define DMA_COMM_Q_HIGH_FMAN_V3	0x3f
#define DMA_COMM_Q_HIGH_FMAN_V2(dma_thresh_max_commq)		\
	((dma_thresh_max_commq + 1) * 3 / 4)
#define DFLT_DMA_COMM_Q_HIGH(major, dma_thresh_max_commq)	\
	((major == 6) ? DMA_COMM_Q_HIGH_FMAN_V3 :		\
	DMA_COMM_Q_HIGH_FMAN_V2(dma_thresh_max_commq))

#define TOTAL_NUM_OF_TASKS_FMAN_V3L	59
#define TOTAL_NUM_OF_TASKS_FMAN_V3H	124
#define DFLT_TOTAL_NUM_OF_TASKS(major, minor, bmi_max_num_of_tasks)	\
	((major == 6) ? ((minor == 1 || minor == 4) ?			\
	TOTAL_NUM_OF_TASKS_FMAN_V3L : TOTAL_NUM_OF_TASKS_FMAN_V3H) :	\
	bmi_max_num_of_tasks)

#define DMA_CAM_NUM_OF_ENTRIES_FMAN_V3		64
#define DMA_CAM_NUM_OF_ENTRIES_FMAN_V2		32
#define DFLT_DMA_CAM_NUM_OF_ENTRIES(major)			\
	(major == 6 ? DMA_CAM_NUM_OF_ENTRIES_FMAN_V3 :		\
	DMA_CAM_NUM_OF_ENTRIES_FMAN_V2)

#define FM_TIMESTAMP_1_USEC_BIT             8

/* Defines used for enabling/disabling FM interrupts */
#define ERR_INTR_EN_DMA         0x00010000
#define ERR_INTR_EN_FPM         0x80000000
#define ERR_INTR_EN_BMI         0x00800000
#define ERR_INTR_EN_QMI         0x00400000
#define ERR_INTR_EN_MURAM       0x00040000
#define ERR_INTR_EN_MAC0        0x00004000
#define ERR_INTR_EN_MAC1        0x00002000
#define ERR_INTR_EN_MAC2        0x00001000
#define ERR_INTR_EN_MAC3        0x00000800
#define ERR_INTR_EN_MAC4        0x00000400
#define ERR_INTR_EN_MAC5        0x00000200
#define ERR_INTR_EN_MAC6        0x00000100
#define ERR_INTR_EN_MAC7        0x00000080
#define ERR_INTR_EN_MAC8        0x00008000
#define ERR_INTR_EN_MAC9        0x00000040

#define INTR_EN_QMI             0x40000000
#define INTR_EN_MAC0            0x00080000
#define INTR_EN_MAC1            0x00040000
#define INTR_EN_MAC2            0x00020000
#define INTR_EN_MAC3            0x00010000
#define INTR_EN_MAC4            0x00000040
#define INTR_EN_MAC5            0x00000020
#define INTR_EN_MAC6            0x00000008
#define INTR_EN_MAC7            0x00000002
#define INTR_EN_MAC8            0x00200000
#define INTR_EN_MAC9            0x00100000
#define INTR_EN_REV0            0x00008000
#define INTR_EN_REV1            0x00004000
#define INTR_EN_REV2            0x00002000
#define INTR_EN_REV3            0x00001000
#define INTR_EN_TMR             0x01000000

/* Modules registers offsets */
#define FM_MM_MURAM             0x00000000
#define FM_MM_BMI               0x00080000
#define FM_MM_QMI               0x00080400
#define FM_MM_PRS               0x000c7000
#define FM_MM_DMA               0x000C2000
#define FM_MM_FPM               0x000C3000
#define FM_MM_IMEM              0x000C4000
#define FM_MM_CGP               0x000DB000
#define FM_MM_TRB(i)            (0x000D0200 + 0x400 * (i))
#define FM_MM_SP                0x000dc000

/* Memory Mapped Registers */

struct fm_iram_regs_t {
	u32 iadd;	/* FM IRAM instruction address register */
	u32 idata;/* FM IRAM instruction data register */
	u32 itcfg;/* FM IRAM timing config register */
	u32 iready;/* FM IRAM ready register */
};

/* General defines */
#define FM_FW_DEBUG_INSTRUCTION             0x6ffff805UL

struct fm_state_struct_t {
	u8 fm_id;
	enum fm_port_type ports_types[FM_MAX_NUM_OF_HW_PORT_IDS];
	u16 fm_clk_freq;
	struct fm_revision_info_t rev_info;
	bool enabled_time_stamp;
	u8 count1_micro_bit;
	u8 total_num_of_tasks;
	u32 total_fifo_size;
	u8 max_num_of_open_dmas;
	u8 accumulated_num_of_tasks;
	u32 accumulated_fifo_size;
	u8 accumulated_num_of_open_dmas;
	u8 accumulated_num_of_deq_tnums;
	bool low_end_restriction;
	u32 exceptions;
	bool rams_ecc_enable;
	bool explicit_enable;
	bool internal_call;
	u32 extra_fifo_pool_size;
	u8 extra_tasks_pool_size;
	u8 extra_open_dmas_pool_size;
	u16 port_mfl[FM_MAX_NUM_OF_MACS];
	u16 mac_mfl[FM_MAX_NUM_OF_MACS];
};

struct fm_intg_t {
	/* Ram defines */
	u32 fm_muram_size;
	u32 fm_iram_size;
	u32 fm_num_of_ctrl;

	/* DMA defines */
	u32 dma_thresh_max_commq;
	u32 dma_thresh_max_buf;

	/* QMI defines */
	u32 qmi_max_num_of_tnums;
	u32 qmi_def_tnums_thresh;

	/* BMI defines */
	u32 bmi_max_num_of_tasks;
	u32 bmi_max_num_of_dmas;
	u32 bmi_max_fifo_size;
	u32 port_max_weight;

	u32 fm_port_num_of_cg;
	u32 num_of_rx_ports;
};

struct fm_t {
	void __iomem *base_addr;
	char fm_module_name[MODULE_NAME_SIZE];
	struct fm_intr_src_t intr_mng[FM_EV_DUMMY_LAST];

	struct fman_fpm_regs __iomem *fpm_regs;
	struct fman_bmi_regs __iomem *bmi_regs;
	struct fman_qmi_regs __iomem *qmi_regs;
	struct fman_dma_regs __iomem *dma_regs;
	fm_exceptions_cb *exception_cb;
	fm_bus_error_cb *bus_error_cb;
	void *dev_id;
	/* Spinlock for FMan use */
	spinlock_t spinlock;
	struct fm_state_struct_t *fm_state;
	u16 tnum_aging_period;

	struct fman_cfg *fm_drv_param;
	struct muram_info *muram;
	/* cam section in muram */
	int cam_offset;
	size_t cam_size;
	/* Fifo in MURAM */
	int fifo_offset;
	size_t fifo_size;
	bool reset_on_init;

	struct fm_intg_t *intg;
};

static inline int fm_dflt_total_fifo_size(u8 major, u8 minor)
{
	/* The total FIFO size values are calculation for each FMan version,
	 * taking into account the available resources.
	 */
	switch (major) {
	case 2:
		/* P4080 */
		return 100 * 1024;
	case 3:
		/* P2, P3, P5 */
		return 122 * 1024;
	case 4:
		/* P1023 */
		return 46 * 1024;
	case 6:
		/* FMan V3L (T1024, T1040) */
		if (minor == 1 || minor == 4)
			return 156 * 1024;
		/* FMan V3H (B4, T4, T2080) */
		else
			return 295 * 1024;
	default:
		pr_err("Default total fifo size calculation failed\n");
		return 0;
	}
}

static inline void fm_call_mac_isr(struct fm_t *fm, u8 id)
{
	if (fm->intr_mng[id].isr_cb)
		fm->intr_mng[id].isr_cb(fm->intr_mng[id].src_handle);
}

#endif /* __FM_H */
