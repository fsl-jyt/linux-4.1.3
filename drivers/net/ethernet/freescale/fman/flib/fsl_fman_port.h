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

#ifndef __FSL_FMAN_PORT_H
#define __FSL_FMAN_PORT_H

#include <linux/io.h>

#include "fsl_fman_sp.h"

/* Registers bit fields */

/* BMI defines */
#define BMI_EBD_EN				0x80000000

#define BMI_PORT_CFG_EN				0x80000000
#define BMI_PORT_CFG_FDOVR			0x02000000

#define BMI_PORT_STATUS_BSY			0x80000000

#define BMI_DMA_ATTR_SWP_SHIFT			FMAN_SP_DMA_ATTR_SWP_SHIFT
#define BMI_DMA_ATTR_IC_STASH_ON		0x10000000
#define BMI_DMA_ATTR_HDR_STASH_ON		0x04000000
#define BMI_DMA_ATTR_SG_STASH_ON		0x01000000
#define BMI_DMA_ATTR_WRITE_OPTIMIZE		FMAN_SP_DMA_ATTR_WRITE_OPTIMIZE

#define BMI_RX_FIFO_PRI_ELEVATION_SHIFT	16
#define BMI_RX_FIFO_THRESHOLD_ETHE		0x80000000

#define BMI_FRAME_END_CS_IGNORE_SHIFT		24
#define BMI_FRAME_END_CS_IGNORE_MASK		0x0000001f

#define BMI_RX_FRAME_END_CUT_SHIFT		16
#define BMI_RX_FRAME_END_CUT_MASK		0x0000001f

#define BMI_IC_TO_EXT_SHIFT			FMAN_SP_IC_TO_EXT_SHIFT
#define BMI_IC_TO_EXT_MASK			0x0000001f
#define BMI_IC_FROM_INT_SHIFT			FMAN_SP_IC_FROM_INT_SHIFT
#define BMI_IC_FROM_INT_MASK			0x0000000f
#define BMI_IC_SIZE_MASK			0x0000001f

#define BMI_INT_BUF_MARG_SHIFT			28
#define BMI_INT_BUF_MARG_MASK			0x0000000f
#define BMI_EXT_BUF_MARG_START_SHIFT		FMAN_SP_EXT_BUF_MARG_START_SHIFT
#define BMI_EXT_BUF_MARG_START_MASK		0x000001ff
#define BMI_EXT_BUF_MARG_END_MASK		0x000001ff

#define BMI_CMD_MR_LEAC				0x00200000
#define BMI_CMD_MR_SLEAC			0x00100000
#define BMI_CMD_MR_MA				0x00080000
#define BMI_CMD_MR_DEAS				0x00040000
#define BMI_CMD_RX_MR_DEF			(BMI_CMD_MR_LEAC | \
						BMI_CMD_MR_SLEAC | \
						BMI_CMD_MR_MA | \
						BMI_CMD_MR_DEAS)
#define BMI_CMD_TX_MR_DEF			0

#define BMI_CMD_ATTR_ORDER			0x80000000
#define BMI_CMD_ATTR_SYNC			0x02000000
#define BMI_CMD_ATTR_COLOR_SHIFT		26

#define BMI_FIFO_PIPELINE_DEPTH_SHIFT		12
#define BMI_FIFO_PIPELINE_DEPTH_MASK		0x0000000f
#define BMI_NEXT_ENG_FD_BITS_SHIFT		24

#define BMI_COUNTERS_EN				0x80000000

#define BMI_EXT_BUF_POOL_VALID			FMAN_SP_EXT_BUF_POOL_VALID
#define BMI_EXT_BUF_POOL_EN_COUNTER		FMAN_SP_EXT_BUF_POOL_EN_COUNTER
#define BMI_EXT_BUF_POOL_BACKUP		FMAN_SP_EXT_BUF_POOL_BACKUP
#define BMI_EXT_BUF_POOL_ID_SHIFT		16
#define BMI_EXT_BUF_POOL_ID_MASK		0x003F0000
#define BMI_POOL_DEP_NUM_OF_POOLS_SHIFT	16

#define BMI_TX_FIFO_MIN_FILL_SHIFT		16

#define BMI_SG_DISABLE				FMAN_SP_SG_DISABLE

#define BMI_PRIORITY_ELEVATION_LEVEL	\
	((0x3FF + 1) *	FMAN_PORT_BMI_FIFO_UNITS)
#define BMI_FIFO_THRESHOLD		\
	((0x3FF + 1) *	FMAN_PORT_BMI_FIFO_UNITS)

#define BMI_DEQUEUE_PIPELINE_DEPTH(_type)		\
	(_type == E_FMAN_PORT_TYPE_TX_10G ? 4 : 1)

/* QMI defines */
#define QMI_PORT_CFG_EN				0x80000000
#define QMI_PORT_CFG_EN_COUNTERS		0x10000000
#define QMI_PORT_STATUS_DEQ_FD_BSY		0x20000000

#define QMI_DEQ_CFG_PRI				0x80000000
#define QMI_DEQ_CFG_TYPE1			0x10000000
#define QMI_DEQ_CFG_TYPE2			0x20000000
#define QMI_DEQ_CFG_TYPE3			0x30000000
#define QMI_DEQ_CFG_PREFETCH_PARTIAL		0x01000000
#define QMI_DEQ_CFG_PREFETCH_FULL		0x03000000
#define QMI_DEQ_CFG_SP_MASK			0xf
#define QMI_DEQ_CFG_SP_SHIFT			20

#define QMI_HIGH_PRIORITY(_type)		\
	(_type == E_FMAN_PORT_TYPE_TX_10G ? true : false)
#define QMI_BYTE_COUNT_LEVEL_CONTROL(_type)	\
	(_type == E_FMAN_PORT_TYPE_TX_10G ? 0x1400 : 0x400)

/* General port defines */
#define FMAN_PORT_MAX_EXT_POOLS_NUM	8
#define FMAN_PORT_OBS_EXT_POOLS_NUM	2
#define FMAN_PORT_CG_MAP_NUM		8
#define FMAN_PORT_PRS_RESULT_WORDS_NUM	8
#define FMAN_PORT_BMI_FIFO_UNITS	0x100
#define FMAN_PORT_IC_OFFSET_UNITS	0x10

/* NIA Description */
#define NIA_ORDER_RESTOR			0x00800000
#define NIA_ENG_FM_CTL				0x00000000
#define NIA_ENG_BMI				0x00500000
#define NIA_ENG_QMI_ENQ				0x00540000
#define NIA_ENG_QMI_DEQ				0x00580000

#define NIA_FM_CTL_AC_NO_IPACC_PRE_BMI_ENQ_FRAME	0x00000028
#define NIA_BMI_AC_ENQ_FRAME				0x00000002
#define NIA_BMI_AC_TX_RELEASE				0x000002C0
#define NIA_BMI_AC_RELEASE				0x000000C0
#define NIA_BMI_AC_TX					0x00000274
#define NIA_BMI_AC_FETCH				0x00000208
#define NIA_BMI_AC_FETCH_ALL_FRAME			0x0000020c

#define DEFAULT_FRAME_QUEUE_ID		0x00FFFFFF
#define ERROR_FRAME_QUEUE_ID		0x00FFFFFF
#define DEFAULT_CONF_FRAME_QUEUE_ID	0x00FFFFFF

/* FM Port Register Map */

/* BMI Rx port register map */
struct fman_port_rx_bmi_regs {
	u32 fmbm_rcfg;		/* Rx Configuration */
	u32 fmbm_rst;		/* Rx Status */
	u32 fmbm_rda;		/* Rx DMA attributes */
	u32 fmbm_rfp;		/* Rx FIFO Parameters */
	u32 fmbm_rfed;		/* Rx Frame End Data */
	u32 fmbm_ricp;		/* Rx Internal Context Parameters */
	u32 fmbm_rim;		/* Rx Internal Buffer Margins */
	u32 fmbm_rebm;		/* Rx External Buffer Margins */
	u32 fmbm_rfne;		/* Rx Frame Next Engine */
	u32 fmbm_rfca;		/* Rx Frame Command Attributes. */
	u32 fmbm_rfpne;		/* Rx Frame Parser Next Engine */
	u32 fmbm_rpso;		/* Rx Parse Start Offset */
	u32 fmbm_rpp;		/* Rx Policer Profile  */
	u32 fmbm_rccb;		/* Rx Coarse Classification Base */
	u32 fmbm_reth;		/* Rx Excessive Threshold */
	u32 reserved003c[1];	/* (0x03C 0x03F) */
	u32 fmbm_rprai[FMAN_PORT_PRS_RESULT_WORDS_NUM];
	/* Rx Parse Results Array Init */
	u32 fmbm_rfqid;		/* Rx Frame Queue ID */
	u32 fmbm_refqid;		/* Rx Error Frame Queue ID */
	u32 fmbm_rfsdm;		/* Rx Frame Status Discard Mask */
	u32 fmbm_rfsem;		/* Rx Frame Status Error Mask */
	u32 fmbm_rfene;		/* Rx Frame Enqueue Next Engine */
	u32 reserved0074[0x2];	/* (0x074-0x07C)  */
	u32 fmbm_rcmne; /* Rx Frame Continuous Mode Next Engine */
	u32 reserved0080[0x20];/* (0x080 0x0FF)  */
	u32 fmbm_ebmpi[FMAN_PORT_MAX_EXT_POOLS_NUM];
	/* Buffer Manager pool Information- */
	u32 fmbm_acnt[FMAN_PORT_MAX_EXT_POOLS_NUM];	/* Allocate Counter- */
	u32 reserved0130[8];	/* 0x130/0x140 - 0x15F reserved - */
	u32 fmbm_rcgm[FMAN_PORT_CG_MAP_NUM];	/* Congestion Group Map */
	u32 fmbm_mpd;		/* BM Pool Depletion  */
	u32 reserved0184[0x1F];	/* (0x184 0x1FF) */
	u32 fmbm_rstc;		/* Rx Statistics Counters */
	u32 fmbm_rfrc;		/* Rx Frame Counter */
	u32 fmbm_rfbc;		/* Rx Bad Frames Counter */
	u32 fmbm_rlfc;		/* Rx Large Frames Counter */
	u32 fmbm_rffc;		/* Rx Filter Frames Counter */
	u32 fmbm_rfdc;		/* Rx Frame Discard Counter */
	u32 fmbm_rfldec;		/* Rx Frames List DMA Error Counter */
	u32 fmbm_rodc;		/* Rx Out of Buffers Discard nntr */
	u32 fmbm_rbdc;		/* Rx Buffers Deallocate Counter */
	u32 reserved0224[0x17];	/* (0x224 0x27F) */
	u32 fmbm_rpc;		/* Rx Performance Counters */
	u32 fmbm_rpcp;		/* Rx Performance Count Parameters */
	u32 fmbm_rccn;		/* Rx Cycle Counter */
	u32 fmbm_rtuc;		/* Rx Tasks Utilization Counter */
	u32 fmbm_rrquc;		/* Rx Receive Queue Utilization cntr */
	u32 fmbm_rduc;		/* Rx DMA Utilization Counter */
	u32 fmbm_rfuc;		/* Rx FIFO Utilization Counter */
	u32 fmbm_rpac;		/* Rx Pause Activation Counter */
	u32 reserved02a0[0x18];	/* (0x2A0 0x2FF) */
	u32 fmbm_rdbg;		/* Rx Debug- */
};

/* BMI Tx port register map */
struct fman_port_tx_bmi_regs {
	u32 fmbm_tcfg;		/* Tx Configuration */
	u32 fmbm_tst;		/* Tx Status */
	u32 fmbm_tda;		/* Tx DMA attributes */
	u32 fmbm_tfp;		/* Tx FIFO Parameters */
	u32 fmbm_tfed;		/* Tx Frame End Data */
	u32 fmbm_ticp;		/* Tx Internal Context Parameters */
	u32 fmbm_tfdne;		/* Tx Frame Dequeue Next Engine. */
	u32 fmbm_tfca;		/* Tx Frame Command attribute. */
	u32 fmbm_tcfqid;	/* Tx Confirmation Frame Queue ID. */
	u32 fmbm_tefqid;	/* Tx Frame Error Queue ID */
	u32 fmbm_tfene;		/* Tx Frame Enqueue Next Engine */
	u32 fmbm_trlmts;	/* Tx Rate Limiter Scale */
	u32 fmbm_trlmt;		/* Tx Rate Limiter */
	u32 reserved0034[0x0e];	/* (0x034-0x6c) */
	u32 fmbm_tccb;		/* Tx Coarse Classification base */
	u32 fmbm_tfne;		/* Tx Frame Next Engine */
	u32 fmbm_tpfcm[0x02];
	/* Tx Priority based Flow Control (PFC) Mapping */
	u32 fmbm_tcmne;		/* Tx Frame Continuous Mode Next Engine */
	u32 reserved0080[0x60];	/* (0x080-0x200) */
	u32 fmbm_tstc;		/* Tx Statistics Counters */
	u32 fmbm_tfrc;		/* Tx Frame Counter */
	u32 fmbm_tfdc;		/* Tx Frames Discard Counter */
	u32 fmbm_tfledc;	/* Tx Frame len error discard cntr */
	u32 fmbm_tfufdc;	/* Tx Frame unsprt frmt discard cntr */
	u32 fmbm_tbdc;		/* Tx Buffers Deallocate Counter */
	u32 reserved0218[0x1A];	/* (0x218-0x280) */
	u32 fmbm_tpc;		/* Tx Performance Counters */
	u32 fmbm_tpcp;		/* Tx Performance Count Parameters */
	u32 fmbm_tccn;		/* Tx Cycle Counter */
	u32 fmbm_ttuc;		/* Tx Tasks Utilization Counter */
	u32 fmbm_ttcquc;	/* Tx Transmit conf Q util Counter */
	u32 fmbm_tduc;		/* Tx DMA Utilization Counter */
	u32 fmbm_tfuc;		/* Tx FIFO Utilization Counter */
};

/* BMI port register map */
union fman_port_bmi_regs {
	struct fman_port_rx_bmi_regs rx;
	struct fman_port_tx_bmi_regs tx;
};

/* QMI port register map */
struct fman_port_qmi_regs {
	u32 fmqm_pnc;		/* PortID n Configuration Register */
	u32 fmqm_pns;		/* PortID n Status Register */
	u32 fmqm_pnts;		/* PortID n Task Status Register */
	u32 reserved00c[4];	/* 0xn00C - 0xn01B */
	u32 fmqm_pnen;		/* PortID n Enqueue NIA Register */
	u32 fmqm_pnetfc;		/* PortID n Enq Total Frame Counter */
	u32 reserved024[2];	/* 0xn024 - 0x02B */
	u32 fmqm_pndn;		/* PortID n Dequeue NIA Register */
	u32 fmqm_pndc;		/* PortID n Dequeue Config Register */
	u32 fmqm_pndtfc;		/* PortID n Dequeue tot Frame cntr */
	u32 fmqm_pndfdc;		/* PortID n Dequeue FQID Dflt Cntr */
	u32 fmqm_pndcc;		/* PortID n Dequeue Confirm Counter */
};

enum fman_port_dma_swap {
	E_FMAN_PORT_DMA_NO_SWAP,	/* No swap, transfer data as is */
	E_FMAN_PORT_DMA_SWAP_LE,
	/* The transferred data should be swapped in PPC Little Endian mode */
	E_FMAN_PORT_DMA_SWAP_BE
	/* The transferred data should be swapped in Big Endian mode */
};

/* Default port color */
enum fman_port_color {
	E_FMAN_PORT_COLOR_GREEN,	/* Default port color is green */
	E_FMAN_PORT_COLOR_YELLOW,	/* Default port color is yellow */
	E_FMAN_PORT_COLOR_RED,		/* Default port color is red */
	E_FMAN_PORT_COLOR_OVERRIDE	/* Ignore color */
};

/* QMI dequeue from the SP channel - types */
enum fman_port_deq_type {
	E_FMAN_PORT_DEQ_BY_PRI,
	/* Priority precedence and Intra-Class scheduling */
	E_FMAN_PORT_DEQ_ACTIVE_FQ,
	/* Active FQ precedence and Intra-Class scheduling */
	E_FMAN_PORT_DEQ_ACTIVE_FQ_NO_ICS
	/* Active FQ precedence and override Intra-Class scheduling */
};

/* QMI dequeue prefetch modes */
enum fman_port_deq_prefetch {
	E_FMAN_PORT_DEQ_NO_PREFETCH, /* No prefetch mode */
	E_FMAN_PORT_DEQ_PART_PREFETCH, /* Partial prefetch mode */
	E_FMAN_PORT_DEQ_FULL_PREFETCH /* Full prefetch mode */
};

/* FM Port configuration structure, used at init */
struct fman_port_cfg {
	/* BMI parameters */
	enum fman_port_dma_swap dma_swap_data;
	bool dma_ic_stash_on;
	bool dma_header_stash_on;
	bool dma_sg_stash_on;
	bool dma_write_optimize;
	u32 ic_ext_offset;
	u32 ic_int_offset;
	u32 ic_size;
	enum fman_port_color color;
	bool sync_req;
	bool discard_override;
	u32 checksum_bytes_ignore;
	u32 rx_cut_end_bytes;
	u32 rx_pri_elevation;
	u32 rx_fifo_thr;
	u8 rx_fd_bits;
	u32 int_buf_start_margin;
	u32 ext_buf_start_margin;
	u32 ext_buf_end_margin;
	u32 tx_fifo_min_level;
	u32 tx_fifo_low_comf_level;
	u32 tx_fifo_deq_pipeline_depth;
	/* QMI parameters */
	bool deq_high_pri;
	enum fman_port_deq_type deq_type;
	enum fman_port_deq_prefetch deq_prefetch_opt;
	u16 deq_byte_cnt;
	bool no_scatter_gather;
	int errata_A006675;
	int errata_A006320;
	int excessive_threshold_register;
	int fmbm_rebm_has_sgd;
	int fmbm_tfne_has_features;
	int qmi_deq_options_support;
};

enum fman_port_type {
	E_FMAN_PORT_TYPE_RX = 0,	/* 1G Rx port */
	E_FMAN_PORT_TYPE_RX_10G,	/* 10G Rx port */
	E_FMAN_PORT_TYPE_TX,		/* 1G Tx port */
	E_FMAN_PORT_TYPE_TX_10G,	/* 10G Tx port */
	E_FMAN_PORT_TYPE_DUMMY
};

struct fman_port_params {
	u32 discard_mask;
	u32 err_mask;
	u32 dflt_fqid;
	u32 err_fqid;
	u32 deq_sp;
	bool dont_release_buf;
};

/* Port context - used by most API functions */
struct fman_port {
	enum fman_port_type type;
	u8 fm_rev_maj;
	u8 fm_rev_min;
	union fman_port_bmi_regs __iomem *bmi_regs;
	struct fman_port_qmi_regs __iomem *qmi_regs;
	u8 ext_pools_num;
};

/* External buffer pools configuration */
struct fman_port_bpools {
	u8 count;			/* Num of pools to set up */
	bool counters_enable;		/* Enable allocate counters */
	u8 grp_bp_depleted_num;
	/* Number of depleted pools - if reached the BMI indicates
	 * the MAC to send a pause frame
	 */
	struct {
		u8 bpid;		/* BM pool ID */
		u16 size;
		/* Pool's size - must be in ascending order */
		bool is_backup;
		/* If this is a backup pool */
		bool grp_bp_depleted;
		/* Consider this buffer in multiple pools depletion criteria */
		bool single_bp_depleted;
		/* Consider this buffer in single pool depletion criteria */
	} bpool[FMAN_PORT_MAX_EXT_POOLS_NUM];
};

/*   FM Port API */
void fman_port_defconfig(struct fman_port_cfg *cfg, enum fman_port_type type);
int fman_port_init(struct fman_port *port,
		   struct fman_port_cfg *cfg, struct fman_port_params *params);
int fman_port_enable(struct fman_port *port);
int fman_port_disable(const struct fman_port *port);
int fman_port_set_bpools(const struct fman_port *port,
			 const struct fman_port_bpools *bp);

#endif	/* __FSL_FMAN_PORT_H */
