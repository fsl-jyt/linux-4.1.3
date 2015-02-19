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

/* FM Application Programming Interface. */
#ifndef __FM_EXT
#define __FM_EXT

#include "fsl_fman_sp.h"

/* Enum for defining port types */
enum fm_port_type {
	FM_PORT_TYPE_TX = 0,	/* TX Port */
	FM_PORT_TYPE_RX,	/* RX Port */
	FM_PORT_TYPE_DUMMY
};

/* Enum for defining port speed */
enum fm_port_speed {
	FM_PORT_SPEED_1G = 0,		/* 1G port */
	FM_PORT_SPEED_10G,		/* 10G port */
	FM_PORT_SPEED_DUMMY
};

/* BMan defines */
#define BM_MAX_NUM_OF_POOLS		64 /* Buffers pools */
#define FM_PORT_MAX_NUM_OF_EXT_POOLS	8  /* External BM pools per Rx port */

/* General FM defines */
#define FM_MAX_NUM_OF_PARTITIONS    64	 /* Maximum number of partitions */

/* FM Frame descriptor macros  */
/* Frame queue Context Override */
#define FM_FD_CMD_FCO                   0x80000000
#define FM_FD_CMD_RPD                   0x40000000  /* Read Prepended Data */
#define FM_FD_CMD_DTC                   0x10000000  /* Do L4 Checksum */

/* Not for Rx-Port! Unsupported Format */
#define FM_FD_ERR_UNSUPPORTED_FORMAT    0x04000000
/* Not for Rx-Port! Length Error */
#define FM_FD_ERR_LENGTH                0x02000000
#define FM_FD_ERR_DMA                   0x01000000  /* DMA Data error */

/* IPR frame (not error) */
#define FM_FD_IPR                       0x00000001
/* IPR non-consistent-sp */
#define FM_FD_ERR_IPR_NCSP              (0x00100000 | FM_FD_IPR)
/* IPR error */
#define FM_FD_ERR_IPR                   (0x00200000 | FM_FD_IPR)
/* IPR timeout */
#define FM_FD_ERR_IPR_TO                (0x00300000 | FM_FD_IPR)

/* Rx FIFO overflow, FCS error, code error, running disparity error
 * (SGMII and TBI modes), FIFO parity error. PHY Sequence error,
 * PHY error control character detected.
 */
#define FM_FD_ERR_PHYSICAL              0x00080000
/* Frame too long OR Frame size exceeds max_length_frame  */
#define FM_FD_ERR_SIZE                  0x00040000
/* classification discard */
#define FM_FD_ERR_CLS_DISCARD           0x00020000
/* Extract Out of Frame */
#define FM_FD_ERR_EXTRACTION            0x00008000
/* No Scheme Selected */
#define FM_FD_ERR_NO_SCHEME             0x00004000
/* Keysize Overflow */
#define FM_FD_ERR_KEYSIZE_OVERFLOW      0x00002000
/* Frame color is red */
#define FM_FD_ERR_COLOR_RED             0x00000800
/* Frame color is yellow */
#define FM_FD_ERR_COLOR_YELLOW          0x00000400
/* Parser Time out Exceed */
#define FM_FD_ERR_PRS_TIMEOUT           0x00000080
/* Invalid Soft Parser instruction */
#define FM_FD_ERR_PRS_ILL_INSTRUCT      0x00000040
/* Header error was identified during parsing */
#define FM_FD_ERR_PRS_HDR_ERR           0x00000020
/* Frame parsed beyind 256 first bytes */
#define FM_FD_ERR_BLOCK_LIMIT_EXCEEDED  0x00000008

/* non Frame-Manager error */
#define FM_FD_RX_STATUS_ERR_NON_FM      0x00400000

/* Parse results memory layout */
struct fm_prs_result_t {
	u8 lpid;		/* Logical port id */
	u8 shimr;		/* Shim header result  */
	u16 l2r;		/* Layer 2 result */
	u16 l3r;		/* Layer 3 result */
	u8 l4r;		/* Layer 4 result */
	u8 cplan;		/* Classification plan id */
	u16 nxthdr;		/* Next Header  */
	u16 cksum;		/* Running-sum */
	/* Flags&fragment-offset field of the last IP-header */
	u16 flags_frag_off;
	/* Routing type field of a IPV6 routing extension header */
	u8 route_type;
	/* Routing Extension Header Present; last bit is IP valid */
	u8 rhp_ip_valid;
	u8 shim_off[2];		/* Shim offset */
	u8 ip_pid_off;		/* IP PID (last IP-proto) offset */
	u8 eth_off;		/* ETH offset */
	u8 llc_snap_off;	/* LLC_SNAP offset */
	u8 vlan_off[2];		/* VLAN offset */
	u8 etype_off;		/* ETYPE offset */
	u8 pppoe_off;		/* PPP offset */
	u8 mpls_off[2];		/* MPLS offset */
	u8 ip_off[2];		/* IP offset */
	u8 gre_off;		/* GRE offset */
	u8 l4_off;		/* Layer 4 offset */
	u8 nxthdr_off;		/** Parser end point */
} __attribute__((__packed__));

/* FM Exceptions */
enum fm_exceptions {
	FM_EX_DMA_BUS_ERROR = 0,	/* DMA bus error. */
	/* Read Buffer ECC error (Valid for FM rev < 6) */
	FM_EX_DMA_READ_ECC,
	/* Write Buffer ECC error on system side (Valid for FM rev < 6) */
	FM_EX_DMA_SYSTEM_WRITE_ECC,
	/* Write Buffer ECC error on FM side (Valid for FM rev < 6) */
	FM_EX_DMA_FM_WRITE_ECC,
	/* Single Port ECC error on FM side (Valid for FM rev > 6) */
	FM_EX_DMA_SINGLE_PORT_ECC,
	FM_EX_FPM_STALL_ON_TASKS,	/* Stall of tasks on FPM */
	FM_EX_FPM_SINGLE_ECC,		/* Single ECC on FPM. */
	/* Double ECC error on FPM ram access */
	FM_EX_FPM_DOUBLE_ECC,
	FM_EX_QMI_SINGLE_ECC,		/* Single ECC on QMI. */
	FM_EX_QMI_DOUBLE_ECC,		/* Double bit ECC occurred on QMI */
	FM_EX_QMI_DEQ_FROM_UNKNOWN_PORTID,
					/* Dequeue from unknown port id */
	FM_EX_BMI_LIST_RAM_ECC,	/* Linked List RAM ECC error */
	FM_EX_BMI_STORAGE_PROFILE_ECC,/* Storage Profile ECC Error */
	/* Statistics Count RAM ECC Error Enable */
	FM_EX_BMI_STATISTICS_RAM_ECC,
	FM_EX_BMI_DISPATCH_RAM_ECC,	/* Dispatch RAM ECC Error Enable */
	FM_EX_MURAM_ECC		/* Double bit ECC occurred on MURAM */
};

/** fm_exceptions_cb
 * dev_id	- Pointer to the device
 * exception	- The exception.
 *
 * Exceptions user callback routine, will be called upon an exception
 * passing the exception identification.
 */
typedef void (fm_exceptions_cb)(void *dev_id,
				 enum fm_exceptions exception);

/** fm_bus_error_cb
 * dev_id	- Pointer to the device
 * port_type	- Port type (enum fm_port_type)
 * port_id	- Port id
 * addr		- Address that caused the error
 * tnum		- Owner of error
 * liodn	- Logical IO device number
 *
 * Bus error user callback routine, will be called upon bus error,
 * passing parameters describing the errors and the owner.
 */
typedef void (fm_bus_error_cb)(void *dev_id, enum fm_port_type port_type,
				u8 port_id, u64 addr, u8 tnum, u16 liodn);

/* A structure for defining buffer prefix area content. */
struct fm_buffer_prefix_content_t {
	/* Number of bytes to be left at the beginning of the external
	 * buffer; Note that the private-area will start from the base
	 * of the buffer address.
	 */
	u16 priv_data_size;
	/* true to pass the parse result to/from the FM;
	 * User may use FM_PORT_GetBufferPrsResult() in
	 * order to get the parser-result from a buffer.
	 */
	bool pass_prs_result;
	/* true to pass the timeStamp to/from the FM User */
	bool pass_time_stamp;
	/* true to pass the KG hash result to/from the FM User may
	 * use FM_PORT_GetBufferHashResult() in order to get the
	 * parser-result from a buffer.
	 */
	bool pass_hash_result;
	/* Add all other Internal-Context information: AD,
	 * hash-result, key, etc.
	 */
	u16 data_align;
};

/* A structure of information about each of the external
 * buffer pools used by a port or storage-profile.
 */
struct fm_ext_pool_params_t {
	u8 id;		    /* External buffer pool id */
	u16 size;		    /* External buffer pool buffer size */
};

/* A structure for informing the driver about the external
 * buffer pools allocated in the BM and used by a port or a
 * storage-profile.
 */
struct fm_ext_pools_t {
	u8 num_of_pools_used; /* Number of pools use by this port */
	struct fm_ext_pool_params_t ext_buf_pool[FM_PORT_MAX_NUM_OF_EXT_POOLS];
					/* Parameters for each port */
};

/* A structure for defining backup BM Pools. */
struct fm_backup_bm_pools_t {
	/* Number of BM backup pools -  must be smaller
	 * than the total number of pools defined for the specified port.
	 */
	u8 num_of_backup_pools;
	/* num_of_backup_pools pool id's, specifying which pools should
	 * be used only as backup. Pool id's specified here must be a
	 * subset of the pools used by the specified port.
	 */
	u8 pool_ids[FM_PORT_MAX_NUM_OF_EXT_POOLS];
};

/* A structure for defining BM pool depletion criteria */
struct fm_buf_pool_depletion_t {
	/* select mode in which pause frames will be sent after a
	 * number of pools (all together!) are depleted
	 */
	bool pools_grp_mode_enable;
	/* the number of depleted pools that will invoke pause
	 * frames transmission.
	 */
	u8 num_of_pools;
	/* For each pool, true if it should be considered for
	 * depletion (Note - this pool must be used by this port!).
	 */
	bool pools_to_consider[BM_MAX_NUM_OF_POOLS];
	/* select mode in which pause frames will be sent
	 * after a single-pool is depleted;
	 */
	bool single_pool_mode_enable;
	/* For each pool, true if it should be considered
	 * for depletion (Note - this pool must be used by this port!)
	 */
	bool pools_to_consider_for_single_mode[BM_MAX_NUM_OF_POOLS];
};

/* A Structure for defining FM initialization parameters */
struct fm_params_t {
	u8 fm_id;
	/* Index of the FM */
	void __iomem *base_addr;
	/* A pointer to base of memory mapped FM registers (virtual);
	 * NOTE that this should include ALL common registers of the FM
	 * including the PCD registers area.
	 */
	struct muram_info *muram;
	/* A pointer to an initialized MURAM object, to be used by the FM. */
	u16 fm_clk_freq;
	/* In Mhz; */
	fm_exceptions_cb *exception_cb;
	/* An application callback routine to handle exceptions; */
	fm_bus_error_cb *bus_error_cb;
	/* An application callback routine to handle exceptions; */
	void *dev_id;
	/* Device cookie used by the exception cbs */
};

struct fm_t; /* FMan data */

/**
 * fm_config
 * @fm_params		A pointer to a data structure of mandatory FM
 *			parameters
 *
 * Creates the FM module and returns its handle (descriptor).
 * This descriptor must be passed as first parameter to all other
 * FM function calls.
 * No actual initialization or configuration of FM hardware is
 * done by this routine. All FM parameters get default values that
 * may be changed by calling one or more of the advance config
 * routines.
 *
 * Return: A handle to the FM object, or NULL for Failure.
 */
void *fm_config(struct fm_params_t *fm_params);

/**
 * fm_init
 * @fm		Pointer to the FMan module
 *
 * Initializes the FM module by defining the software structure
 * and configuring the hardware registers.
 *
 * Return 0 on success; Error code otherwise.
 */
int fm_init(struct fm_t *fm);

/**
 * fm_free
 * @fm		Pointer to the FMan module
 *
 * Frees all resources that were assigned to FM module.
 * Calling this routine invalidates the descriptor.
 *
 * Return: 0 on success; Error code otherwise.
 */
int fm_free(struct fm_t *fm);

/* Enum for choosing the field that will be output on AID */
enum fm_dma_aid_mode {
	FM_DMA_AID_OUT_PORT_ID = 0,	    /* 4 LSB of PORT_ID */
	FM_DMA_AID_OUT_TNUM		    /* 4 LSB of TNUM */
};

/* Enum for selecting DMA Emergency options */
/* Enable emergency for MURAM1 */
#define  FM_DMA_MURAM_READ_EMERGENCY        0x00800000
/* Enable emergency for MURAM2 */
#define  FM_DMA_MURAM_WRITE_EMERGENCY       0x00400000
/* Enable emergency for external bus */
#define  FM_DMA_EXT_BUS_EMERGENCY           0x00100000

/**
 * fm_cfg_reset_on_init
 * @fm	Pointer to the FMan module
 * @enable	When true, FM will be reset before any
 *
 * Define whether to reset the FM before initialization.
 * Change the default configuration [DEFAULT_RESET_ON_INIT].
 *
 * Allowed only following fm_config() and before fm_init().
 *
 * Return: 0 on success; Error code otherwise.
 */
int fm_cfg_reset_on_init(struct fm_t *fm, bool enable);

/**
 * fm_cfg_total_fifo_size
 * @fm		Pointer to the FMan module
 * @total_fifo_size	The selected new value.
 *
 * Define Total FIFO size for the whole FM.
 * Calling this routine changes the total Fifo size in the internal driver
 * data base from its default configuration [DEFAULT_total_fifo_size]
 *
 * Allowed only following fm_config() and before fm_init().
 *
 * Return: 0 on success; Error code otherwise.
 */
int fm_cfg_total_fifo_size(struct fm_t *fm, u32 total_fifo_size);

/* A Structure for returning FM revision information */
struct fm_revision_info_t {
	u8 major_rev;		    /* Major revision */
	u8 minor_rev;		    /* Minor revision */
};

/**
 * fm_set_exception
 * fm		Pointer to the FMan module
 * exception	The exception to be selected.
 * enable	True to enable interrupt, false to mask it.
 *
 * Calling this routine enables/disables the specified exception.
 *
 * Allowed only following fm_init().
 *
 * Return: 0 on success; Error code otherwise.
 */
int fm_set_exception(struct fm_t *fm, enum fm_exceptions exception,
		     bool enable);

/**
 * fm_disable_rams_ecc
 * @fm		Pointer to the FMan module
 * Disables ECC mechanism for all the different FM RAM's; E.g. IRAM,
 * MURAM, etc. Note:
 *	In opposed to fm_enable_rams_ecc, this routine must be called
 *	explicitly to disable all Rams ECC.
 *
 * Allowed only following fm_config() and before fm_init().
 *
 * Return: 0 on success; Error code otherwise.
 */
int fm_disable_rams_ecc(struct fm_t *fm);

/**
 * fm_get_revision
 * @fm			- Pointer to the FMan module
 * fm_rev		- A structure of revision information parameters.
 * Returns the FM revision
 *
 * Allowed only following fm_init().
 *
 * Return: 0 on success; Error code otherwise.
 */
void fm_get_revision(struct fm_t *fm, struct fm_revision_info_t *fm_rev);

/**
 * fm_error_isr
 * @fm		Pointer to the FMan module
 * FM interrupt-service-routine for errors.
 *
 * Allowed only following fm_init().
 *
 * Return: 0 on success; EMPTY if no errors found in register, other
 *	   error code otherwise.
 */
int fm_error_isr(struct fm_t *fm);

/**
 * fm_event_isr
 * @fm	Pointer to the FMan module
 *
 * FM interrupt-service-routine for normal events.
 * Allowed only following fm_init().
 */
void fm_event_isr(struct fm_t *fm);

#endif /* __FM_EXT */
