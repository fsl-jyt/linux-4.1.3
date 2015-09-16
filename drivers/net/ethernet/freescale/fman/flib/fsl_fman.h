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

#ifndef __FSL_FMAN_H
#define __FSL_FMAN_H

#include <linux/delay.h>

struct fman_revision_info {
	u8 major_rev;			/* Major revision */
	u8 minor_rev;			/* Minor revision */
};

/* sizes */
#define OFFSET_UNITS				16
#define MAX_INT_OFFSET				240
#define MAX_IC_SIZE				256
#define MAX_EXT_OFFSET				496
#define MAX_EXT_BUFFER_OFFSET			511

/* Memory Mapped Registers */
#define FMAN_LIODN_TBL	64	/* size of LIODN table */

struct fman_fpm_regs {
	u32 fmfp_tnc;		/* FPM TNUM Control 0x00 */
	u32 fmfp_prc;		/* FPM Port_ID FmCtl Association 0x04 */
	u32 fmfp_brkc;		/* FPM Breakpoint Control 0x08 */
	u32 fmfp_mxd;		/* FPM Flush Control 0x0c */
	u32 fmfp_dist1;		/* FPM Dispatch Thresholds1 0x10 */
	u32 fmfp_dist2;		/* FPM Dispatch Thresholds2 0x14 */
	u32 fm_epi;		/* FM Error Pending Interrupts 0x18 */
	u32 fm_rie;		/* FM Error Interrupt Enable 0x1c */
	u32 fmfp_fcev[4];	/* FPM FMan-Controller Event 1-4 0x20-0x2f */
	u32 res0030[4];		/* res 0x30 - 0x3f */
	u32 fmfp_cee[4];	/* PM FMan-Controller Event 1-4 0x40-0x4f */
	u32 res0050[4];		/* res 0x50-0x5f */
	u32 fmfp_tsc1;		/* FPM TimeStamp Control1 0x60 */
	u32 fmfp_tsc2;		/* FPM TimeStamp Control2 0x64 */
	u32 fmfp_tsp;		/* FPM Time Stamp 0x68 */
	u32 fmfp_tsf;		/* FPM Time Stamp Fraction 0x6c */
	u32 fm_rcr;		/* FM Rams Control 0x70 */
	u32 fmfp_extc;		/* FPM External Requests Control 0x74 */
	u32 fmfp_ext1;		/* FPM External Requests Config1 0x78 */
	u32 fmfp_ext2;		/* FPM External Requests Config2 0x7c */
	u32 fmfp_drd[16];	/* FPM Data_Ram Data 0-15 0x80 - 0xbf */
	u32 fmfp_dra;		/* FPM Data Ram Access 0xc0 */
	u32 fm_ip_rev_1;	/* FM IP Block Revision 1 0xc4 */
	u32 fm_ip_rev_2;	/* FM IP Block Revision 2 0xc8 */
	u32 fm_rstc;		/* FM Reset Command 0xcc */
	u32 fm_cld;		/* FM Classifier Debug 0xd0 */
	u32 fm_npi;		/* FM Normal Pending Interrupts 0xd4 */
	u32 fmfp_exte;		/* FPM External Requests Enable 0xd8 */
	u32 fmfp_ee;		/* FPM Event&Mask 0xdc */
	u32 fmfp_cev[4];	/* FPM CPU Event 1-4 0xe0-0xef */
	u32 res00f0[4];		/* res 0xf0-0xff */
	u32 fmfp_ps[50];	/* FPM Port Status 0x100-0x1c7 */
	u32 res01c8[14];	/* res 0x1c8-0x1ff */
	u32 fmfp_clfabc;	/* FPM CLFABC 0x200 */
	u32 fmfp_clfcc;		/* FPM CLFCC 0x204 */
	u32 fmfp_clfaval;	/* FPM CLFAVAL 0x208 */
	u32 fmfp_clfbval;	/* FPM CLFBVAL 0x20c */
	u32 fmfp_clfcval;	/* FPM CLFCVAL 0x210 */
	u32 fmfp_clfamsk;	/* FPM CLFAMSK 0x214 */
	u32 fmfp_clfbmsk;	/* FPM CLFBMSK 0x218 */
	u32 fmfp_clfcmsk;	/* FPM CLFCMSK 0x21c */
	u32 fmfp_clfamc;	/* FPM CLFAMC 0x220 */
	u32 fmfp_clfbmc;	/* FPM CLFBMC 0x224 */
	u32 fmfp_clfcmc;	/* FPM CLFCMC 0x228 */
	u32 fmfp_decceh;	/* FPM DECCEH 0x22c */
	u32 res0230[116];	/* res 0x230 - 0x3ff */
	u32 fmfp_ts[128];	/* 0x400: FPM Task Status 0x400 - 0x5ff */
	u32 res0600[0x400 - 384];
};

struct fman_bmi_regs {
	u32 fmbm_init;		/* BMI Initialization 0x00 */
	u32 fmbm_cfg1;		/* BMI Configuration 1 0x04 */
	u32 fmbm_cfg2;		/* BMI Configuration 2 0x08 */
	u32 res000c[5];		/* 0x0c - 0x1f */
	u32 fmbm_ievr;		/* Interrupt Event Register 0x20 */
	u32 fmbm_ier;		/* Interrupt Enable Register 0x24 */
	u32 fmbm_ifr;		/* Interrupt Force Register 0x28 */
	u32 res002c[5];		/* 0x2c - 0x3f */
	u32 fmbm_arb[8];	/* BMI Arbitration 0x40 - 0x5f */
	u32 res0060[12];	/* 0x60 - 0x8f */
	u32 fmbm_dtc[3];	/* Debug Trap Counter 0x90 - 0x9b */
	u32 res009c;		/* 0x9c */
	u32 fmbm_dcv[3][4];	/* Debug Compare val 0xa0-0xcf */
	u32 fmbm_dcm[3][4];	/* Debug Compare Mask 0xd0-0xff */
	u32 fmbm_gde;		/* BMI Global Debug Enable 0x100 */
	u32 fmbm_pp[63];	/* BMI Port Parameters 0x104 - 0x1ff */
	u32 res0200;		/* 0x200 */
	u32 fmbm_pfs[63];	/* BMI Port FIFO Size 0x204 - 0x2ff */
	u32 res0300;		/* 0x300 */
	u32 fmbm_spliodn[63];	/* Port Partition ID 0x304 - 0x3ff */
};

struct fman_qmi_regs {
	u32 fmqm_gc;		/* General Configuration Register 0x00 */
	u32 res0004;		/* 0x04 */
	u32 fmqm_eie;		/* Error Interrupt Event Register 0x08 */
	u32 fmqm_eien;		/* Error Interrupt Enable Register 0x0c */
	u32 fmqm_eif;		/* Error Interrupt Force Register 0x10 */
	u32 fmqm_ie;		/* Interrupt Event Register 0x14 */
	u32 fmqm_ien;		/* Interrupt Enable Register 0x18 */
	u32 fmqm_if;		/* Interrupt Force Register 0x1c */
	u32 fmqm_gs;		/* Global Status Register 0x20 */
	u32 fmqm_ts;		/* Task Status Register 0x24 */
	u32 fmqm_etfc;		/* Enqueue Total Frame Counter 0x28 */
	u32 fmqm_dtfc;		/* Dequeue Total Frame Counter 0x2c */
	u32 fmqm_dc0;		/* Dequeue Counter 0 0x30 */
	u32 fmqm_dc1;		/* Dequeue Counter 1 0x34 */
	u32 fmqm_dc2;		/* Dequeue Counter 2 0x38 */
	u32 fmqm_dc3;		/* Dequeue Counter 3 0x3c */
	u32 fmqm_dfdc;		/* Dequeue FQID from Default Counter 0x40 */
	u32 fmqm_dfcc;		/* Dequeue FQID from Context Counter 0x44 */
	u32 fmqm_dffc;		/* Dequeue FQID from FD Counter 0x48 */
	u32 fmqm_dcc;		/* Dequeue Confirm Counter 0x4c */
	u32 res0050[7];		/* 0x50 - 0x6b */
	u32 fmqm_tapc;		/* Tnum Aging Period Control 0x6c */
	u32 fmqm_dmcvc;		/* Dequeue MAC Command Valid Counter 0x70 */
	u32 fmqm_difdcc;	/* Dequeue Invalid FD Command Counter 0x74 */
	u32 fmqm_da1v;		/* Dequeue A1 Valid Counter 0x78 */
	u32 res007c;		/* 0x7c */
	u32 fmqm_dtc;		/* 0x80 Debug Trap Counter 0x80 */
	u32 fmqm_efddd;		/* 0x84 Enqueue Frame desc Dynamic dbg 0x84 */
	u32 res0088[2];		/* 0x88 - 0x8f */
	struct {
		u32 fmqm_dtcfg1;	/* 0x90 dbg trap cfg 1 Register 0x00 */
		u32 fmqm_dtval1;	/* Debug Trap Value 1 Register 0x04 */
		u32 fmqm_dtm1;		/* Debug Trap Mask 1 Register 0x08 */
		u32 fmqm_dtc1;		/* Debug Trap Counter 1 Register 0x0c */
		u32 fmqm_dtcfg2;	/* dbg Trap cfg 2 Register 0x10 */
		u32 fmqm_dtval2;	/* Debug Trap Value 2 Register 0x14 */
		u32 fmqm_dtm2;		/* Debug Trap Mask 2 Register 0x18 */
		u32 res001c;		/* 0x1c */
	} dbg_traps[3];			/* 0x90 - 0xef */
	u8 res00f0[0x400 - 0xf0];	/* 0xf0 - 0x3ff */
};

struct fman_dma_regs {
	u32 fmdmsr;	/* FM DMA status register 0x00 */
	u32 fmdmmr;	/* FM DMA mode register 0x04 */
	u32 fmdmtr;	/* FM DMA bus threshold register 0x08 */
	u32 fmdmhy;	/* FM DMA bus hysteresis register 0x0c */
	u32 fmdmsetr;	/* FM DMA SOS emergency Threshold Register 0x10 */
	u32 fmdmtah;	/* FM DMA transfer bus address high reg 0x14 */
	u32 fmdmtal;	/* FM DMA transfer bus address low reg 0x18 */
	u32 fmdmtcid;	/* FM DMA transfer bus communication ID reg 0x1c */
	u32 fmdmra;	/* FM DMA bus internal ram address register 0x20 */
	u32 fmdmrd;	/* FM DMA bus internal ram data register 0x24 */
	u32 fmdmwcr;	/* FM DMA CAM watchdog counter value 0x28 */
	u32 fmdmebcr;	/* FM DMA CAM base in MURAM register 0x2c */
	u32 fmdmccqdr;	/* FM DMA CAM and CMD Queue Debug reg 0x30 */
	u32 fmdmccqvr1;	/* FM DMA CAM and CMD Queue Value reg #1 0x34 */
	u32 fmdmccqvr2;	/* FM DMA CAM and CMD Queue Value reg #2 0x38 */
	u32 fmdmcqvr3;	/* FM DMA CMD Queue Value register #3 0x3c */
	u32 fmdmcqvr4;	/* FM DMA CMD Queue Value register #4 0x40 */
	u32 fmdmcqvr5;	/* FM DMA CMD Queue Value register #5 0x44 */
	u32 fmdmsefrc;	/* FM DMA Semaphore Entry Full Reject Cntr 0x48 */
	u32 fmdmsqfrc;	/* FM DMA Semaphore Queue Full Reject Cntr 0x4c */
	u32 fmdmssrc;	/* FM DMA Semaphore SYNC Reject Counter 0x50 */
	u32 fmdmdcr;	/* FM DMA Debug Counter 0x54 */
	u32 fmdmemsr;	/* FM DMA Emergency Smoother Register 0x58 */
	u32 res005c;	/* 0x5c */
	u32 fmdmplr[FMAN_LIODN_TBL / 2];	/* DMA LIODN regs 0x60-0xdf */
	u32 res00e0[0x400 - 56];
};

struct fman_rg {
	struct fman_fpm_regs __iomem *fpm_rg;
	struct fman_dma_regs __iomem *dma_rg;
	struct fman_bmi_regs __iomem *bmi_rg;
	struct fman_qmi_regs __iomem *qmi_rg;
};

enum fman_dma_cache_override {
	E_FMAN_DMA_NO_CACHE_OR = 0,/* No override of the Cache field */
	E_FMAN_DMA_NO_STASH_DATA, /* No data stashing in system level cache */
	E_FMAN_DMA_MAY_STASH_DATA, /* Stashing allowed in sys level cache */
	E_FMAN_DMA_STASH_DATA /* Stashing performed in system level cache */
};

enum fman_dma_aid_mode {
	E_FMAN_DMA_AID_OUT_PORT_ID = 0,		  /* 4 LSB of PORT_ID */
	E_FMAN_DMA_AID_OUT_TNUM			  /* 4 LSB of TNUM */
};

enum fman_dma_dbg_cnt_mode {
	E_FMAN_DMA_DBG_NO_CNT = 0, /* No counting */
	E_FMAN_DMA_DBG_CNT_DONE, /* Count DONE commands */
	E_FMAN_DMA_DBG_CNT_COMM_Q_EM, /* command Q emergency signal */
	E_FMAN_DMA_DBG_CNT_INT_READ_EM,	/* Read buf emergency signal */
	E_FMAN_DMA_DBG_CNT_INT_WRITE_EM, /* Write buf emergency signal */
	E_FMAN_DMA_DBG_CNT_FPM_WAIT, /* FPM WAIT signal */
	E_FMAN_DMA_DBG_CNT_SIGLE_BIT_ECC, /* Single bit ECC errors */
	E_FMAN_DMA_DBG_CNT_RAW_WAR_PROT	/* RAW&WAR protection counter */
};

enum fman_dma_emergency_level {
	E_FMAN_DMA_EM_EBS = 0, /* EBS emergency */
	E_FMAN_DMA_EM_SOS /* SOS emergency */
};

enum fman_catastrophic_err {
	E_FMAN_CATAST_ERR_STALL_PORT = 0, /* Port_ID stalled reset required */
	E_FMAN_CATAST_ERR_STALL_TASK /* Only erroneous task is stalled */
};

enum fman_dma_err {
	E_FMAN_DMA_ERR_CATASTROPHIC = 0, /* Catastrophic DMA error */
	E_FMAN_DMA_ERR_REPORT /* Reported DMA error */
};

struct fman_cfg {
	u8 disp_limit_tsh;
	u8 prs_disp_tsh;
	u8 plcr_disp_tsh;
	u8 kg_disp_tsh;
	u8 bmi_disp_tsh;
	u8 qmi_enq_disp_tsh;
	u8 qmi_deq_disp_tsh;
	u8 fm_ctl1_disp_tsh;
	u8 fm_ctl2_disp_tsh;
	enum fman_dma_cache_override dma_cache_override;
	enum fman_dma_aid_mode dma_aid_mode;
	bool dma_aid_override;
	u32 dma_axi_dbg_num_of_beats;
	u32 dma_cam_num_of_entries;
	u32 dma_watchdog;
	u8 dma_comm_qtsh_asrt_emer;
	u32 dma_write_buf_tsh_asrt_emer;
	u32 dma_read_buf_tsh_asrt_emer;
	u8 dma_comm_qtsh_clr_emer;
	u32 dma_write_buf_tsh_clr_emer;
	u32 dma_read_buf_tsh_clr_emer;
	u32 dma_sos_emergency;
	enum fman_dma_dbg_cnt_mode dma_dbg_cnt_mode;
	bool dma_stop_on_bus_error;
	bool dma_en_emergency;
	u32 dma_emergency_bus_select;
	enum fman_dma_emergency_level dma_emergency_level;
	bool dma_en_emergency_smoother;
	u32 dma_emergency_switch_counter;
	bool halt_on_external_activ;
	bool halt_on_unrecov_ecc_err;
	enum fman_catastrophic_err catastrophic_err;
	enum fman_dma_err dma_err;
	bool en_muram_test_mode;
	bool en_iram_test_mode;
	bool external_ecc_rams_enable;
	u16 tnum_aging_period;
	u32 exceptions;
	u16 clk_freq;
	bool pedantic_dma;
	u32 cam_base_addr;
	u32 fifo_base_addr;
	u32 total_fifo_size;
	u32 total_num_of_tasks;
	bool qmi_deq_option_support;
	u32 qmi_def_tnums_thresh;
	u8 num_of_fman_ctrl_evnt_regs;
};

/* Exceptions */
#define FMAN_EX_DMA_BUS_ERROR			0x80000000
#define FMAN_EX_DMA_READ_ECC			0x40000000
#define FMAN_EX_DMA_SYSTEM_WRITE_ECC		0x20000000
#define FMAN_EX_DMA_FM_WRITE_ECC		0x10000000
#define FMAN_EX_FPM_STALL_ON_TASKS		0x08000000
#define FMAN_EX_FPM_SINGLE_ECC			0x04000000
#define FMAN_EX_FPM_DOUBLE_ECC			0x02000000
#define FMAN_EX_QMI_SINGLE_ECC			0x01000000
#define FMAN_EX_QMI_DEQ_FROM_UNKNOWN_PORTID	0x00800000
#define FMAN_EX_QMI_DOUBLE_ECC			0x00400000
#define FMAN_EX_BMI_LIST_RAM_ECC		0x00200000
#define FMAN_EX_BMI_PIPELINE_ECC		0x00100000
#define FMAN_EX_BMI_STATISTICS_RAM_ECC		0x00080000
#define FMAN_EX_IRAM_ECC			0x00040000
#define FMAN_EX_NURAM_ECC			0x00020000
#define FMAN_EX_BMI_DISPATCH_RAM_ECC		0x00010000

enum fman_exceptions {
	E_FMAN_EX_DMA_BUS_ERROR = 0, /* DMA bus error. */
	E_FMAN_EX_DMA_READ_ECC,	/* Read Buffer ECC error */
	E_FMAN_EX_DMA_SYSTEM_WRITE_ECC,	/* Write Buffer ECC err on sys side */
	E_FMAN_EX_DMA_FM_WRITE_ECC, /* Write Buffer ECC error on FM side */
	E_FMAN_EX_FPM_STALL_ON_TASKS, /* Stall of tasks on FPM */
	E_FMAN_EX_FPM_SINGLE_ECC, /* Single ECC on FPM. */
	E_FMAN_EX_FPM_DOUBLE_ECC, /* Double ECC error on FPM ram access */
	E_FMAN_EX_QMI_SINGLE_ECC, /* Single ECC on QMI. */
	E_FMAN_EX_QMI_DOUBLE_ECC, /* Double bit ECC occurred on QMI */
	E_FMAN_EX_QMI_DEQ_FROM_UNKNOWN_PORTID,/* DeQ from unknown port id */
	E_FMAN_EX_BMI_LIST_RAM_ECC, /* Linked List RAM ECC error */
	E_FMAN_EX_BMI_STORAGE_PROFILE_ECC, /* storage profile */
	E_FMAN_EX_BMI_STATISTICS_RAM_ECC, /* Statistics RAM ECC Err Enable */
	E_FMAN_EX_BMI_DISPATCH_RAM_ECC,	/* Dispatch RAM ECC Error Enable */
	E_FMAN_EX_IRAM_ECC, /* Double bit ECC occurred on IRAM */
	E_FMAN_EX_MURAM_ECC /* Double bit ECC occurred on MURAM */
};

#define FPM_PRT_FM_CTL1			0x00000001
#define FPM_PRT_FM_CTL2			0x00000002

/* DMA definitions */

/* masks */
#define DMA_MODE_AID_OR			0x20000000
#define DMA_MODE_SBER			0x10000000
#define DMA_MODE_BER			0x00200000
#define DMA_MODE_ECC			0x00000020
#define DMA_MODE_SECURE_PROT		0x00000800
#define DMA_MODE_EMER_READ		0x00080000
#define DMA_MODE_AXI_DBG_MASK		0x0F000000

#define DMA_TRANSFER_PORTID_MASK	0xFF000000
#define DMA_TRANSFER_TNUM_MASK		0x00FF0000
#define DMA_TRANSFER_LIODN_MASK	0x00000FFF

#define DMA_STATUS_BUS_ERR		0x08000000
#define DMA_STATUS_READ_ECC		0x04000000
#define DMA_STATUS_SYSTEM_WRITE_ECC	0x02000000
#define DMA_STATUS_FM_WRITE_ECC	0x01000000
#define DMA_STATUS_FM_SPDAT_ECC	0x00080000

#define FM_LIODN_BASE_MASK		0x00000FFF

/* DMA mask and shifts */
#define DMA_MODE_CACHE_OR_SHIFT		30
#define DMA_MODE_AXI_DBG_SHIFT			24
#define DMA_MODE_CEN_SHIFT			13
#define DMA_MODE_CEN_MASK			0x00000007
#define DMA_MODE_DBG_SHIFT			7
#define DMA_MODE_EMER_LVL_SHIFT		6
#define DMA_MODE_AID_MODE_SHIFT		4
#define DMA_MODE_MAX_AXI_DBG_NUM_OF_BEATS	16

#define DMA_THRESH_COMMQ_SHIFT			24
#define DMA_THRESH_READ_INT_BUF_SHIFT		16
#define DMA_THRESH_READ_INT_BUF_MASK		0x0000003f
#define DMA_THRESH_WRITE_INT_BUF_MASK		0x0000003f

#define DMA_LIODN_SHIFT				16

#define DMA_TRANSFER_PORTID_SHIFT		24
#define DMA_TRANSFER_TNUM_SHIFT		16

/* sizes */
#define DMA_MAX_WATCHDOG			0xffffffff

/* others */
#define DMA_CAM_SIZEOF_ENTRY			0x40
#define DMA_CAM_ALIGN				0x40
#define DMA_CAM_UNITS				8

/* General defines */
#define FM_DEBUG_STATUS_REGISTER_OFFSET	0x000d1084UL

/* FPM defines */

/* masks */
#define FPM_EV_MASK_DOUBLE_ECC		0x80000000
#define FPM_EV_MASK_STALL		0x40000000
#define FPM_EV_MASK_SINGLE_ECC		0x20000000
#define FPM_EV_MASK_RELEASE_FM		0x00010000
#define FPM_EV_MASK_DOUBLE_ECC_EN	0x00008000
#define FPM_EV_MASK_STALL_EN		0x00004000
#define FPM_EV_MASK_SINGLE_ECC_EN	0x00002000
#define FPM_EV_MASK_EXTERNAL_HALT	0x00000008
#define FPM_EV_MASK_ECC_ERR_HALT	0x00000004

#define FPM_RAM_RAMS_ECC_EN		0x80000000
#define FPM_RAM_IRAM_ECC_EN		0x40000000
#define FPM_RAM_MURAM_ECC		0x00008000
#define FPM_RAM_IRAM_ECC		0x00004000
#define FPM_RAM_MURAM_TEST_ECC		0x20000000
#define FPM_RAM_IRAM_TEST_ECC		0x10000000
#define FPM_RAM_RAMS_ECC_EN_SRC_SEL	0x08000000

#define FPM_IRAM_ECC_ERR_EX_EN		0x00020000
#define FPM_MURAM_ECC_ERR_EX_EN	0x00040000

#define FPM_REV1_MAJOR_MASK		0x0000FF00
#define FPM_REV1_MINOR_MASK		0x000000FF

#define FPM_TS_CTL_EN			0x80000000

#define FPM_RSTC_FM_RESET		0x80000000

/* shifts */
#define FPM_DISP_LIMIT_SHIFT		24

#define FPM_THR1_PRS_SHIFT		24
#define FPM_THR1_KG_SHIFT		16
#define FPM_THR1_PLCR_SHIFT		8
#define FPM_THR1_BMI_SHIFT		0

#define FPM_THR2_QMI_ENQ_SHIFT		24
#define FPM_THR2_QMI_DEQ_SHIFT		0
#define FPM_THR2_FM_CTL1_SHIFT		16
#define FPM_THR2_FM_CTL2_SHIFT		8

#define FPM_EV_MASK_CAT_ERR_SHIFT	1
#define FPM_EV_MASK_DMA_ERR_SHIFT	0

#define FPM_REV1_MAJOR_SHIFT		8
#define FPM_REV1_MINOR_SHIFT		0

#define FPM_TS_INT_SHIFT		16

#define FPM_PORT_FM_CTL_PORTID_SHIFT	24

#define FPM_PRC_ORA_FM_CTL_SEL_SHIFT	16

/* others */
#define FPM_MAX_DISP_LIMIT		31
#define FPM_RSTC_FM_RESET		0x80000000
#define FPM_RSTC_MAC0_RESET		0x40000000
#define FPM_RSTC_MAC1_RESET		0x20000000
#define FPM_RSTC_MAC2_RESET		0x10000000
#define FPM_RSTC_MAC3_RESET		0x08000000
#define FPM_RSTC_MAC8_RESET		0x04000000
#define FPM_RSTC_MAC4_RESET		0x02000000
#define FPM_RSTC_MAC5_RESET		0x01000000
#define FPM_RSTC_MAC6_RESET		0x00800000
#define FPM_RSTC_MAC7_RESET		0x00400000
#define FPM_RSTC_MAC9_RESET		0x00200000
/* BMI defines */
/* masks */
#define BMI_INIT_START				0x80000000
#define BMI_ERR_INTR_EN_STORAGE_PROFILE_ECC	0x80000000
#define BMI_ERR_INTR_EN_LIST_RAM_ECC		0x40000000
#define BMI_ERR_INTR_EN_STATISTICS_RAM_ECC	0x20000000
#define BMI_ERR_INTR_EN_DISPATCH_RAM_ECC	0x10000000
#define BMI_NUM_OF_TASKS_MASK			0x3F000000
#define BMI_NUM_OF_EXTRA_TASKS_MASK		0x000F0000
#define BMI_NUM_OF_DMAS_MASK			0x00000F00
#define BMI_NUM_OF_EXTRA_DMAS_MASK		0x0000000F
#define BMI_FIFO_SIZE_MASK			0x000003FF
#define BMI_EXTRA_FIFO_SIZE_MASK		0x03FF0000
#define BMI_CFG2_DMAS_MASK			0x0000003F
#define BMI_CFG2_TASKS_MASK			0x0000003F

/* shifts */
#define BMI_CFG2_TASKS_SHIFT		16
#define BMI_CFG2_DMAS_SHIFT		0
#define BMI_CFG1_FIFO_SIZE_SHIFT	16
#define BMI_EXTRA_FIFO_SIZE_SHIFT	16
#define BMI_NUM_OF_TASKS_SHIFT		24
#define BMI_EXTRA_NUM_OF_TASKS_SHIFT	16
#define BMI_NUM_OF_DMAS_SHIFT		8
#define BMI_EXTRA_NUM_OF_DMAS_SHIFT	0

/* others */
#define BMI_FIFO_ALIGN			0x100
#define FMAN_BMI_FIFO_UNITS		0x100

/* QMI defines */
/* masks */
#define QMI_CFG_ENQ_EN			0x80000000
#define QMI_CFG_DEQ_EN			0x40000000
#define QMI_CFG_EN_COUNTERS		0x10000000
#define QMI_CFG_DEQ_MASK		0x0000003F
#define QMI_CFG_ENQ_MASK		0x00003F00
#define QMI_CFG_ENQ_SHIFT		8

#define QMI_ERR_INTR_EN_DOUBLE_ECC	0x80000000
#define QMI_ERR_INTR_EN_DEQ_FROM_DEF	0x40000000
#define QMI_INTR_EN_SINGLE_ECC		0x80000000

/* shifts */
#define QMI_TAPC_TAP			22

#define QMI_GS_HALT_NOT_BUSY		0x00000002

/* IRAM defines */
/* masks */
#define IRAM_IADD_AIE			0x80000000
#define IRAM_READY			0x80000000

u32 fman_get_bmi_err_event(struct fman_bmi_regs __iomem *bmi_rg);
u32 fman_get_qmi_err_event(struct fman_qmi_regs __iomem *qmi_rg);
u32 fman_get_dma_com_id(struct fman_dma_regs __iomem *dma_rg);
u64 fman_get_dma_addr(struct fman_dma_regs __iomem *dma_rg);
u32 fman_get_dma_err_event(struct fman_dma_regs __iomem *dma_rg);
u32 fman_get_fpm_err_event(struct fman_fpm_regs __iomem *fpm_rg);
u32 fman_get_muram_err_event(struct fman_fpm_regs __iomem *fpm_rg);
u32 fman_get_iram_err_event(struct fman_fpm_regs __iomem *fpm_rg);
u32 fman_get_qmi_event(struct fman_qmi_regs __iomem *qmi_rg);
u32 fman_get_fpm_error_interrupts(struct fman_fpm_regs __iomem *fpm_rg);
u32 fman_get_qmi_deq_th(struct fman_qmi_regs __iomem *qmi_rg);
u32 fman_get_qmi_enq_th(struct fman_qmi_regs __iomem *qmi_rg);
u8 fman_get_num_of_dmas(struct fman_bmi_regs __iomem *bmi_rg, u8 port_id);
u8 fman_get_num_extra_dmas(struct fman_bmi_regs __iomem *bmi_rg, u8 port_id);
u32 fman_get_normal_pending(struct fman_fpm_regs __iomem *fpm_rg);
u32 fman_get_controller_event(struct fman_fpm_regs __iomem *fpm_rg, u8 reg_id);
void fman_get_revision(struct fman_fpm_regs __iomem *fpm_rg, u8 *major,
		       u8 *minor);

void fman_set_order_restoration_per_port(struct fman_fpm_regs __iomem *fpm_rg,
					 u8 port_id, bool is_rx_port);
void fman_set_qmi_enq_th(struct fman_qmi_regs __iomem *qmi_rg, u32 val);
void fman_set_qmi_deq_th(struct fman_qmi_regs __iomem *qmi_rg, u32 val);
void fman_set_liodn_per_port(struct fman_rg *fman_rg, u8 port_id,
			     u16 liodn_base, u16 liodn_offset);
void fman_set_size_of_fifo(struct fman_bmi_regs __iomem *bmi_rg, u8 port_id,
			   u32 size_of_fifo, u32 extra_size_of_fifo);
void fman_set_num_of_tasks(struct fman_bmi_regs __iomem *bmi_rg,
			   u8 port_id,
			   u8 num_of_tasks, u8 num_of_extra_tasks);
void fman_set_num_of_open_dmas(struct fman_bmi_regs __iomem *bmi_rg,
			       u8 port_id, u8 num_of_open_dmas,
			       u8 num_of_extra_open_dmas, u8 total_num_of_dmas);
int fman_set_exception(struct fman_rg *fman_rg,
		       enum fman_exceptions exception, bool enable);

void fman_defconfig(struct fman_cfg *cfg);
int fman_fpm_init(struct fman_fpm_regs __iomem *fpm_rg, struct fman_cfg *cfg);
int fman_bmi_init(struct fman_bmi_regs __iomem *bmi_rg, struct fman_cfg *cfg);
int fman_qmi_init(struct fman_qmi_regs __iomem *qmi_rg, struct fman_cfg *cfg);
int fman_dma_init(struct fman_dma_regs __iomem *dma_rg, struct fman_cfg *cfg);
int fman_enable(struct fman_rg *fman_rg, struct fman_cfg *cfg);
void fman_resume(struct fman_fpm_regs __iomem *fpm_rg);

void fman_enable_time_stamp(struct fman_fpm_regs __iomem *fpm_rg,
			    u8 count1ubit, u16 fm_clk_freq);
void fman_enable_rams_ecc(struct fman_fpm_regs __iomem *fpm_rg);
void fman_disable_rams_ecc(struct fman_fpm_regs __iomem *fpm_rg);
int fman_reset_mac(struct fman_fpm_regs __iomem *fpm_rg, u8 mac_id);
bool fman_rams_ecc_is_external_ctl(struct fman_fpm_regs __iomem *fpm_rg);
bool fman_is_qmi_halt_not_busy_state(struct fman_qmi_regs __iomem *qmi_rg);

/* default values */
#define DEFAULT_CATASTROPHIC_ERR		E_FMAN_CATAST_ERR_STALL_PORT
#define DEFAULT_DMA_ERR				E_FMAN_DMA_ERR_CATASTROPHIC
#define DEFAULT_HALT_ON_EXTERNAL_ACTIVATION	false
#define DEFAULT_HALT_ON_UNRECOVERABLE_ECC_ERROR false
#define DEFAULT_EXTERNAL_ECC_RAMS_ENABLE	false
#define DEFAULT_AID_OVERRIDE			false
#define DEFAULT_AID_MODE			E_FMAN_DMA_AID_OUT_TNUM
#define DEFAULT_DMA_COMM_Q_LOW			0x2A
#define DEFAULT_DMA_COMM_Q_HIGH		0x3F
#define DEFAULT_CACHE_OVERRIDE			E_FMAN_DMA_NO_CACHE_OR
#define DEFAULT_DMA_CAM_NUM_OF_ENTRIES		64
#define DEFAULT_DMA_DBG_CNT_MODE		E_FMAN_DMA_DBG_NO_CNT
#define DEFAULT_DMA_EN_EMERGENCY		false
#define DEFAULT_DMA_SOS_EMERGENCY		0
#define DEFAULT_DMA_WATCHDOG			0	/* disabled */
#define DEFAULT_DMA_EN_EMERGENCY_SMOOTHER	false
#define DEFAULT_DMA_EMERGENCY_SWITCH_COUNTER	0
#define DEFAULT_DISP_LIMIT			0
#define DEFAULT_PRS_DISP_TH			16
#define DEFAULT_PLCR_DISP_TH			16
#define DEFAULT_KG_DISP_TH			16
#define DEFAULT_BMI_DISP_TH			16
#define DEFAULT_QMI_ENQ_DISP_TH		16
#define DEFAULT_QMI_DEQ_DISP_TH		16
#define DEFAULT_FM_CTL1_DISP_TH		16
#define DEFAULT_FM_CTL2_DISP_TH		16
#define DEFAULT_TNUM_AGING_PERIOD		4

#endif	/* __FSL_FMAN_H */
