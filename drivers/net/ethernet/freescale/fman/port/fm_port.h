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

#ifndef __FM_PORT_H
#define __FM_PORT_H

#include "fm_common.h"
#include "fm_sp_common.h"
#include "fsl_fman_sp.h"
#include "fm_port_ext.h"
#include "fsl_fman_port.h"

#define LIODN_DONT_OVERRIDE    (-1)

#define MIN_EXT_BUF_SIZE				64

#define MAX_PORT_FIFO_SIZE(bmi_max_fifo_size)	\
	min((u32)bmi_max_fifo_size, (u32)1024 * BMI_FIFO_UNITS)

/* Memory Map defines */
#define BMI_PORT_REGS_OFFSET				0
#define QMI_PORT_REGS_OFFSET				0x400

/* defaults */
#define DFLT_PORT_DEQ_HIGH_PRIORITY(speed)	\
	((speed == FM_PORT_SPEED_10G) ? true : false)
#define DFLT_PORT_DEQ_TYPE			FM_PORT_DEQ_TYPE1
#define DFLT_PORT_DEQ_PREFETCH_OPT		FM_PORT_DEQ_FULL_PREFETCH
#define DFLT_PORT_DEQ_BYTE_CNT(speed)		\
	((speed == FM_PORT_SPEED_10G) ? 0x1400 : 0x400)
#define DFLT_PORT_BUFFER_PREFIX_CONTENT_PRIV_DATA_SIZE	\
	DEFAULT_FM_SP_BUFFER_PREFIX_CONTENT_PRIV_DATA_SIZE
#define DFLT_PORT_BUFFER_PREFIX_CONTENT_PASS_PRS_RESULT	\
	DEFAULT_FM_SP_BUFFER_PREFIX_CONTENT_PRIV_PASS_PRS_RESULT
#define DFLT_PORT_BUFFER_PREFIX_CONTENT_PASS_TIME_STAMP	\
	DEFAULT_FM_SP_BUFFER_PREFIX_CONTEXT_PASS_TIME_STAMP
#define DFLT_PORT_BUFFER_PREFIX_CONTEXT_DATA_ALIGN		\
	DEFAULT_FM_SP_BUFFER_PREFIX_CONTEXT_DATA_ALIGN
#define DFLT_PORT_CHECKSUM_LAST_BYTES_IGNORE	0
#define DFLT_PORT_CUT_BYTES_FROM_END		4

#define DFLT_PORT_FRM_DISCARD_OVERRIDE		false

#define DFLT_PORT_FORWARD_INT_CONTENT_REUSE	false
#define DFLT_PORT_BUF_MARGINS_END_MAARGINS	0
#define DFLT_PORT_ERRORS_TO_DISCARD		FM_PORT_FRM_ERR_CLS_DISCARD
#define DFLT_PORT_MAX_FRAME_LENGTH		9600

#define DFLT_NOT_SUPPORTED			0xff

#define DFLT_PORT_RX_FIFO_PRI_ELEVATION_LEV(bmi_max_fifo_size)	\
	MAX_PORT_FIFO_SIZE(bmi_max_fifo_size)

#define DFLT_PORT_RX_FIFO_THRESHOLD(major, bmi_max_fifo_size)	\
	(major == 6 ?						\
	MAX_PORT_FIFO_SIZE(bmi_max_fifo_size) :		\
	(MAX_PORT_FIFO_SIZE(bmi_max_fifo_size) * 3 / 4))	\

#define DFLT_PORT_TX_FIFO_MIN_FILL_LEVEL		0
#define DFLT_PORT_TX_FIFO_LOW_COMF_LEVEL		(5 * 1024)

#define DFLT_PORT_EXTRA_NUM_OF_FIFO_BUFS		0

#define FM_PORT_MAX_NUM_OF_CONGESTION_GRPS_ALL_INTEGRATIONS 256

/* Memory Mapped Registers */
struct fm_port_rx_bmi_regs_t {
	u32 fmbm_rcfg;	/* Rx Configuration */
	u32 fmbm_rst;	/* Rx Status */
	u32 fmbm_rda;	/* Rx DMA attributes */
	u32 fmbm_rfp;	/* Rx FIFO Parameters */
	u32 fmbm_rfed;	/* Rx Frame End Data */
	u32 fmbm_ricp;	/* Rx Internal Context Parameters */
	u32 fmbm_rim;	/* Rx Internal Buffer Margins */
	u32 fmbm_rebm;	/* Rx External Buffer Margins */
	u32 fmbm_rfne;	/* Rx Frame Next Engine */
	u32 fmbm_rfca;	/* Rx Frame Command Attributes. */
	u32 fmbm_rfpne;	/* Rx Frame Parser Next Engine */
	u32 fmbm_rpso;	/* Rx Parse Start Offset */
	u32 fmbm_rpp;	/* Rx Policer Profile  */
	u32 fmbm_rccb;	/* Rx Coarse Classification Base */
	u32 fmbm_reth;	/* Rx Excessive Threshold */
	u32 reserved1[0x01]; /* (0x03C) */
	u32 fmbm_rprai[FM_PORT_PRS_RESULT_NUM_OF_WORDS];
	/* Rx Parse Results Array Initialization */
	u32 fmbm_rfqid;	/* Rx Frame Queue ID */
	u32 fmbm_refqid;	/* Rx Error Frame Queue ID */
	u32 fmbm_rfsdm;	/* Rx Frame Status Discard Mask */
	u32 fmbm_rfsem;	/* Rx Frame Status Error Mask */
	u32 fmbm_rfene;	/* Rx Frame Enqueue Next Engine */
	u32 reserved2[0x02]; /* (0x074-0x078) */
	/* Rx Frame Continuous Mode Next Engine */
	u32 fmbm_rcmne;
	u32 reserved3[0x20]; /* (0x080 0x0FF)  */
	u32 fmbm_ebmpi[FM_PORT_MAX_NUM_OF_EXT_POOLS];
	/* Buffer Manager pool Information- */
	u32 fmbm_acnt[FM_PORT_MAX_NUM_OF_EXT_POOLS]; /* Allocate Counter- */
	u32 reserved4[0x08]; /* 0x130/0x140 - 0x15F reserved - */
	u32 fmbm_rcgm[FM_PORT_MAX_NUM_OF_CONGESTION_GRPS_ALL_INTEGRATIONS / 32];
	/* Congestion Group Map */
	u32 fmbm_rmpd;	/* BM Pool Depletion  */
	u32 reserved5[0x1F]; /* (0x184 0x1FF) */
	u32 fmbm_rstc;	/* Rx Statistics Counters */
	u32 fmbm_rfrc;	/* Rx Frame Counter */
	u32 fmbm_rfbc;	/* Rx Bad Frames Counter */
	u32 fmbm_rlfc;	/* Rx Large Frames Counter */
	u32 fmbm_rffc;	/* Rx Filter Frames Counter */
	u32 fmbm_rfcd;	/* Rx Frame Discard Counter */
	u32 fmbm_rfldec;	/* Rx Frames List DMA Error Counter */
	u32 fmbm_rodc;/* Rx Out of Buffers Discard Counter- */
	u32 fmbm_rbdc;	/* Rx Buffers Deallocate Counter- */
	u32 fmbm_rpec;	/* Rx RX Prepare to enqueue Counter- */
	u32 reserved6[0x16]; /* (0x228 0x27F) */
	u32 fmbm_rpc;	/* Rx Performance Counters */
	u32 fmbm_rpcp;	/* Rx Performance Count Parameters */
	u32 fmbm_rccn;	/* Rx Cycle Counter */
	u32 fmbm_rtuc;	/* Rx Tasks Utilization Counter */
	u32 fmbm_rrquc;/* Rx Receive Queue Utilization Counter */
	u32 fmbm_rduc;	/* Rx DMA Utilization Counter */
	u32 fmbm_rfuc;	/* Rx FIFO Utilization Counter */
	u32 fmbm_rpac;	/* Rx Pause Activation Counter */
	u32 reserved7[0x18]; /* (0x2A0-0x2FF) */
	u32 fmbm_rdcfg[0x3]; /* Rx Debug- */
	u32 fmbm_rgpr;	/* Rx General Purpose Register. */
	u32 reserved8[0x3a];
	/* (0x310-0x3FF) */
};

struct fm_port_tx_bmi_regs_t {
	u32 fmbm_tcfg;	/* Tx Configuration */
	u32 fmbm_tst;	/* Tx Status */
	u32 fmbm_tda;	/* Tx DMA attributes */
	u32 fmbm_tfp;	/* Tx FIFO Parameters */
	u32 fmbm_tfed;	/* Tx Frame End Data */
	u32 fmbm_ticp;	/* Tx Internal Context Parameters */
	u32 fmbm_tfdne;	/* Tx Frame Dequeue Next Engine. */
	u32 fmbm_tfca;	/* Tx Frame Command attribute. */
	u32 fmbm_tcfqid;	/* Tx Confirmation Frame Queue ID. */
	u32 fmbm_tfeqid;	/* Tx Frame Error Queue ID */
	u32 fmbm_tfene;	/* Tx Frame Enqueue Next Engine */
	u32 fmbm_trlmts;	/* Tx Rate Limiter Scale */
	u32 fmbm_trlmt;	/* Tx Rate Limiter */
	u32 fmbm_tccb;	/* Tx Coarse Classification Base */
	u32 reserved0[0x0e]; /* (0x038-0x070) */
	u32 fmbm_tfne;	/* Tx Frame Next Engine */
	u32 fmbm_tpfcm[0x02];
	/* Tx Priority based Flow Control (PFC) Mapping */
	u32 fmbm_tcmne; /* Tx Frame Continuous Mode Next Engine */
	u32 reserved2[0x60]; /* (0x080-0x200) */
	u32 fmbm_tstc;	/* Tx Statistics Counters */
	u32 fmbm_tfrc;	/* Tx Frame Counter */
	u32 fmbm_tfdc;	/* Tx Frames Discard Counter */
	u32 fmbm_tfledc;	/* Tx Frame Length error discard counter */
	/* Tx Frame unsupported format discard Counter */
	u32 fmbm_tfufdc;
	u32 fmbm_tbdc;	/* Tx Buffers Deallocate Counter */
	u32 reserved3[0x1A]; /* (0x218-0x280) */
	u32 fmbm_tpc;	/* Tx Performance Counters */
	u32 fmbm_tpcp;	/* Tx Performance Count Parameters */
	u32 fmbm_tccn;	/* Tx Cycle Counter */
	u32 fmbm_ttuc;	/* Tx Tasks Utilization Counter */
	u32 fmbm_ttcquc; /* Tx Transmit Confirm Queue Utilization Counter */
	u32 fmbm_tduc;	/* Tx DMA Utilization Counter */
	u32 fmbm_tfuc;	/* Tx FIFO Utilization Counter */
	u32 reserved4[16];/* (0x29C-0x2FF) */
	u32 fmbm_tdcfg[0x3];
					/* Tx Debug- */
	u32 fmbm_tgpr;	/* O/H General Purpose Register */
	u32 reserved5[0x3a]; /* (0x310-0x3FF) */
};

union fm_port_bmi_regs_u {
	struct fm_port_rx_bmi_regs_t rx_port_bmi_regs;
	struct fm_port_tx_bmi_regs_t tx_port_bmi_regs;
};

/* BMI defines */
#define BMI_PORT_RFNE_FRWD_RPD                  0x40000000

#define BMI_STATUS_RX_MASK_UNUSED	\
(u32)(~(FM_PORT_FRM_ERR_DMA                    | \
FM_PORT_FRM_ERR_PHYSICAL               | \
FM_PORT_FRM_ERR_SIZE                   | \
FM_PORT_FRM_ERR_CLS_DISCARD            | \
FM_PORT_FRM_ERR_EXTRACTION             | \
FM_PORT_FRM_ERR_NO_SCHEME              | \
FM_PORT_FRM_ERR_COLOR_RED              | \
FM_PORT_FRM_ERR_COLOR_YELLOW           | \
FM_PORT_FRM_ERR_PRS_TIMEOUT            | \
FM_PORT_FRM_ERR_PRS_ILL_INSTRUCT       | \
FM_PORT_FRM_ERR_BLOCK_LIMIT_EXCEEDED   | \
FM_PORT_FRM_ERR_PRS_HDR_ERR            | \
FM_PORT_FRM_ERR_IPRE                   | \
FM_PORT_FRM_ERR_IPR_NCSP               | \
FM_PORT_FRM_ERR_KEYSIZE_OVERFLOW))

#define RX_ERRS_TO_ENQ	\
(FM_PORT_FRM_ERR_DMA                    | \
FM_PORT_FRM_ERR_PHYSICAL               | \
FM_PORT_FRM_ERR_SIZE                   | \
FM_PORT_FRM_ERR_EXTRACTION             | \
FM_PORT_FRM_ERR_NO_SCHEME              | \
FM_PORT_FRM_ERR_PRS_TIMEOUT            | \
FM_PORT_FRM_ERR_PRS_ILL_INSTRUCT       | \
FM_PORT_FRM_ERR_BLOCK_LIMIT_EXCEEDED   | \
FM_PORT_FRM_ERR_PRS_HDR_ERR            | \
FM_PORT_FRM_ERR_KEYSIZE_OVERFLOW       | \
FM_PORT_FRM_ERR_IPRE)

/* sizes */
#define FRAME_END_DATA_SIZE                     16
#define MIN_TX_INT_OFFSET                       16
#define MAX_FIFO_PIPELINE_DEPTH                 8
#define MAX_NUM_OF_TASKS                        64
#define MAX_NUM_OF_EXTRA_TASKS                  8
#define MAX_NUM_OF_DMAS                         16
#define MAX_NUM_OF_EXTRA_DMAS                   8

/* QMI defines */
#define QMI_DEQ_CFG_SUBPORTAL_MASK              0x1f

struct fm_port_drv_param_t {
	struct fman_port_cfg dflt_cfg;
	u32 dflt_fqid;
	u32 err_fqid;
	void __iomem *base_addr;
	u8 deq_sub_portal;
	bool deq_high_priority;
	enum fm_port_deq_type deq_type;
	enum fm_port_deq_prefetch_option deq_prefetch_option;
	u16 deq_byte_cnt;
	u8 cheksum_last_bytes_ignore;
	u8 cut_bytes_from_end;
	struct fm_buf_pool_depletion_t buf_pool_depletion;
	bool frm_discard_override;
	bool en_buf_pool_depletion;
	u16 liodn_offset;
	u16 liodn_base;
	struct fm_ext_pools_t ext_buf_pools;
	u32 tx_fifo_min_fill_level;
	u32 tx_fifo_low_comf_level;
	u32 rx_fifo_pri_elevation_level;
	u32 rx_fifo_threshold;
	struct fm_sp_buf_margins_t buf_margins;
	struct fm_sp_int_context_data_copy_t int_context;
	u32 errors_to_discard;
	bool forward_reuse_int_context;
	struct fm_buffer_prefix_content_t buffer_prefix_content;
	struct fm_backup_bm_pools_t *backup_bm_pools;
	bool dont_release_buf;
	bool set_num_of_tasks;
	bool set_num_of_open_dmas;
	bool set_size_of_fifo;
	bool bcb_workaround;
};

struct fm_port_rx_pools_params_t {
	u8 num_of_pools;
	u16 second_largest_buf_size;
	u16 largest_buf_size;
};

struct fm_port_intg_t {
	u32 max_port_fifo_size;
	u32 max_num_of_ext_pools;
	u32 fm_max_num_of_sub_portals;
	u32 bm_max_num_of_pools;
};

/* No PCD Engine indicated */
#define FM_PCD_NONE			0

struct fm_port_t {
	struct fman_port port;
	void *fm;
	struct fm_revision_info_t fm_rev_info;
	u8 port_id;
	enum fm_port_type port_type;
	enum fm_port_speed port_speed;
	int enabled;
	char name[MODULE_NAME_SIZE];

	union fm_port_bmi_regs_u __iomem *fm_port_bmi_regs;
	/* The optional engines are devined avobe */
	struct fm_sp_buffer_offsets_t buffer_offsets;

	u8 internal_buf_offset;
	struct fm_ext_pools_t ext_buf_pools;

	u16 max_frame_length;
	struct fm_port_rsrc_t open_dmas;
	struct fm_port_rsrc_t tasks;
	struct fm_port_rsrc_t fifo_bufs;
	struct fm_port_rx_pools_params_t rx_pools_params;

	struct fm_port_drv_param_t *fm_port_drv_param;
	struct fm_port_intg_t *port_intg;
};

static inline int fm_port_dflt_fifo_deq_pipeline_depth(u8 major,
						       enum fm_port_type type,
						       enum fm_port_speed speed)
{
	switch (type) {
	case FM_PORT_TYPE_RX:
	case FM_PORT_TYPE_TX:
		switch (speed) {
		case FM_PORT_SPEED_10G:
			return 4;
		case FM_PORT_SPEED_1G:
			if (major >= 6)
				return 2;
			else
				return 1;
		default:
			return 0;
		}
	default:
		return 0;
	}
}

static inline int fm_port_dflt_num_of_tasks(u8 major,
					    enum fm_port_type type,
					    enum fm_port_speed speed)
{
	switch (type) {
	case FM_PORT_TYPE_RX:
	case FM_PORT_TYPE_TX:
		switch (speed) {
		case FM_PORT_SPEED_10G:
			return 16;
		case FM_PORT_SPEED_1G:
			if (major >= 6)
				return 4;
			else
				return 3;
		default:
			return 0;
		}
	default:
		return 0;
	}
}

static inline int fm_port_dflt_extra_num_of_tasks(u8 major,
						  enum fm_port_type type,
						  enum fm_port_speed speed)
{
	switch (type) {
	case FM_PORT_TYPE_RX:
		/* FMan V3 */
		if (major >= 6)
			return 0;

		/* FMan V2 */
		if (speed == FM_PORT_SPEED_10G)
			return 8;
		else
			return 2;
	case FM_PORT_TYPE_TX:
	default:
		return 0;
	}
}

static inline int fm_port_dflt_num_of_open_dmas(u8 major,
						enum fm_port_type type,
						enum fm_port_speed speed)
{
	int val;

	if (major >= 6) {
		switch (type) {
		case FM_PORT_TYPE_TX:
			if (speed == FM_PORT_SPEED_10G)
				val = 12;
			else
				val = 3;
			break;
		case FM_PORT_TYPE_RX:
			if (speed == FM_PORT_SPEED_10G)
				val = 8;
			else
				val = 2;
			break;
		default:
			return 0;
		}
	} else {
		switch (type) {
		case FM_PORT_TYPE_TX:
		case FM_PORT_TYPE_RX:
			if (speed == FM_PORT_SPEED_10G)
				val = 8;
			else
				val = 1;
			break;
		default:
			val = 0;
		}
	}

	return val;
}

static inline int fm_port_dflt_extra_num_of_open_dmas(u8 major,
						      enum fm_port_type type,
						      enum fm_port_speed speed)
{
	/* FMan V3 */
	if (major >= 6)
		return 0;

	/* FMan V2 */
	switch (type) {
	case FM_PORT_TYPE_RX:
	case FM_PORT_TYPE_TX:
		if (speed == FM_PORT_SPEED_10G)
			return 8;
		else
			return 1;
	default:
		return 0;
	}
}

static inline int fm_port_dflt_num_of_fifo_bufs(u8 major,
						enum fm_port_type type,
						enum fm_port_speed speed)
{
	int val;

	if (major >= 6) {
		switch (type) {
		case FM_PORT_TYPE_TX:
			if (speed == FM_PORT_SPEED_10G)
				val = 64;
			else
				val = 50;
			break;
		case FM_PORT_TYPE_RX:
			if (speed == FM_PORT_SPEED_10G)
				val = 96;
			else
				val = 50;
			break;
		default:
			val = 0;
		}
	} else {
		switch (type) {
		case FM_PORT_TYPE_TX:
			if (speed == FM_PORT_SPEED_10G)
				val = 48;
			else
				val = 44;
			break;
		case FM_PORT_TYPE_RX:
			if (speed == FM_PORT_SPEED_10G)
				val = 48;
			else
				val = 45;
			break;
		default:
			val = 0;
		}
	}

	return val;
}

#endif /* __FM_PORT_H */
