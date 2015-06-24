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
#ifndef __FM_PORT_EXT
#define __FM_PORT_EXT

#include "fm_ext.h"

/* FM Port API
 * The FM uses a general module called "port" to represent a Tx port (MAC),
 * an Rx port (MAC) or Offline Parsing port.
 * The number of ports in an FM varies between SOCs.
 * The SW driver manages these ports as sub-modules of the FM,i.e. after an
 * FM is initialized, its ports may be initialized and operated upon.
 * The port is initialized aware of its type, but other functions on a port
 * may be indifferent to its type. When necessary, the driver verifies
 * coherence and returns error if applicable.
 * On initialization, user specifies the port type and it's index (relative
 * to the port's type) - always starting at 0.
 */

/* General FM Port defines */
/* Number of 4 bytes words in parser result */
#define FM_PORT_PRS_RESULT_NUM_OF_WORDS     8

/* FM Frame error */
/* Frame Descriptor errors */
/* Not for Rx-Port! Unsupported Format */
#define FM_PORT_FRM_ERR_UNSUPPORTED_FORMAT	FM_FD_ERR_UNSUPPORTED_FORMAT
/* Not for Rx-Port! Length Error */
#define FM_PORT_FRM_ERR_LENGTH			FM_FD_ERR_LENGTH
/* DMA Data error */
#define FM_PORT_FRM_ERR_DMA			FM_FD_ERR_DMA
/* non Frame-Manager error; probably come from SEC that was chained to FM */
#define FM_PORT_FRM_ERR_NON_FM			FM_FD_RX_STATUS_ERR_NON_FM
 /* IPR error */
#define FM_PORT_FRM_ERR_IPRE			(FM_FD_ERR_IPR & ~FM_FD_IPR)
/* IPR non-consistent-sp */
#define FM_PORT_FRM_ERR_IPR_NCSP		(FM_FD_ERR_IPR_NCSP &	\
						~FM_FD_IPR)

/* Rx FIFO overflow, FCS error, code error, running disparity
 * error (SGMII and TBI modes), FIFO parity error.
 * PHY Sequence error, PHY error control character detected.
 */
#define FM_PORT_FRM_ERR_PHYSICAL                FM_FD_ERR_PHYSICAL
/* Frame too long OR Frame size exceeds max_length_frame  */
#define FM_PORT_FRM_ERR_SIZE                    FM_FD_ERR_SIZE
/* indicates a classifier "drop" operation */
#define FM_PORT_FRM_ERR_CLS_DISCARD             FM_FD_ERR_CLS_DISCARD
/* Extract Out of Frame */
#define FM_PORT_FRM_ERR_EXTRACTION              FM_FD_ERR_EXTRACTION
/* No Scheme Selected */
#define FM_PORT_FRM_ERR_NO_SCHEME               FM_FD_ERR_NO_SCHEME
/* Keysize Overflow */
#define FM_PORT_FRM_ERR_KEYSIZE_OVERFLOW        FM_FD_ERR_KEYSIZE_OVERFLOW
/* Frame color is red */
#define FM_PORT_FRM_ERR_COLOR_RED               FM_FD_ERR_COLOR_RED
/* Frame color is yellow */
#define FM_PORT_FRM_ERR_COLOR_YELLOW            FM_FD_ERR_COLOR_YELLOW
/* Parser Time out Exceed */
#define FM_PORT_FRM_ERR_PRS_TIMEOUT             FM_FD_ERR_PRS_TIMEOUT
/* Invalid Soft Parser instruction */
#define FM_PORT_FRM_ERR_PRS_ILL_INSTRUCT        FM_FD_ERR_PRS_ILL_INSTRUCT
/* Header error was identified during parsing */
#define FM_PORT_FRM_ERR_PRS_HDR_ERR             FM_FD_ERR_PRS_HDR_ERR
/* Frame parsed beyind 256 first bytes */
#define FM_PORT_FRM_ERR_BLOCK_LIMIT_EXCEEDED    FM_FD_ERR_BLOCK_LIMIT_EXCEEDED
/* FPM Frame Processing Timeout Exceeded */
#define FM_PORT_FRM_ERR_PROCESS_TIMEOUT         0x00000001

/* FM Port Initialization Unit */
struct fm_port_t;

/* A structure for additional Rx port parameters */
struct fm_port_rx_params_t {
	u32 err_fqid;			/* Error Queue Id. */
	u32 dflt_fqid;			/* Default Queue Id. */
	u16 liodn_offset;			/* Port's LIODN offset. */
	/* Which external buffer pools are used
	 * (up to FM_PORT_MAX_NUM_OF_EXT_POOLS), and their sizes.
	 */
	struct fm_ext_pools_t ext_buf_pools;
};

/* A structure for additional non-Rx port parameters */
struct fm_port_non_rx_params_t {
	/* Error Queue Id. */
	u32 err_fqid;
	/* For Tx - Default Confirmation queue, 0 means no Tx confirmation
	 * for processed frames. For OP port - default Rx queue.
	 */
	u32 dflt_fqid;
	/* QM-channel dedicated to this port; will be used
	 * by the FM for dequeue.
	 */
	u32 qm_channel;
};

/* A union for additional parameters depending on port type */
union fm_port_specific_params_u {
	/* Rx port parameters structure */
	struct fm_port_rx_params_t rx_params;
	/* Non-Rx port parameters structure */
	struct fm_port_non_rx_params_t non_rx_params;
};

/* A structure representing FM initialization parameters */
struct fm_port_params_t {
	void __iomem *base_addr;
	/* Virtual Address of memory mapped FM Port registers. */
	void *fm;
	/* A handle to the FM object this port related to */
	enum fm_port_type port_type;
	/* Port type */
	enum fm_port_speed port_speed;
	/* Port speed */
	u8 port_id;
	/* Port Id - relative to type;
	 * NOTE: When configuring Offline Parsing port for FMANv3 devices,
	 * it is highly recommended NOT to use port_id=0 due to lack of HW
	 * resources on port_id=0.
	 */
	u16 liodn_base;
	/* Irrelevant for P4080 rev 1. LIODN base for this port, to be
	 * used together with LIODN offset.
	 */
	union fm_port_specific_params_u specific_params;
	/* Additional parameters depending on port type. */
};

struct fm_port;

/**
 * fm_port_config
 * @fm_port_params:	Pointer to data structure of parameters
 *
 * Creates a descriptor for the FM PORT module.
 * The routine returns a handle (descriptor) to the FM PORT object.
 * This descriptor must be passed as first parameter to all other FM PORT
 * function calls.
 * No actual initialization or configuration of FM hardware is done by this
 * routine.
 *
 * Return: Pointer to FM Port object, or NULL for Failure.
 */
struct fm_port_t *fm_port_config(struct fm_port_params_t *fm_port_params);

/**
 * fm_port_init
 * fm_port:	A pointer to a FM Port module.
 * Initializes the FM PORT module by defining the software structure and
 * configuring the hardware registers.
 *
 * Return: 0 on success; Error code otherwise.
 */
int fm_port_init(struct fm_port_t *fm_port);

/* enum for defining QM frame dequeue */
enum fm_port_deq_type {
	/* Dequeue from the SP channel - with priority precedence,
	 * and Intra-Class Scheduling respected.
	 */
	FM_PORT_DEQ_TYPE1,
	/* Dequeue from the SP channel - with active FQ precedence,
	 * and Intra-Class Scheduling respected.
	 */
	FM_PORT_DEQ_TYPE2,
	/* Dequeue from the SP channel - with active FQ precedence,
	 * and override Intra-Class Scheduling
	 */
	FM_PORT_DEQ_TYPE3
};

/* enum for defining QM frame dequeue */
enum fm_port_deq_prefetch_option {
	/* QMI preforms a dequeue action for a single frame only
	 * when a dedicated portID Tnum is waiting.
	 */
	FM_PORT_DEQ_NO_PREFETCH,
	/* QMI preforms a dequeue action for 3 frames when one
	 * dedicated port_id tnum is waiting.
	 */
	FM_PORT_DEQ_PARTIAL_PREFETCH,
	/* QMI preforms a dequeue action for 3 frames when
	 * no dedicated port_id tnums are waiting.
	 */
	FM_PORT_DEQ_FULL_PREFETCH
};

/* enum for defining port default color */
enum fm_port_color {
	FM_PORT_COLOR_GREEN,	    /* Default port color is green */
	FM_PORT_COLOR_YELLOW,	    /* Default port color is yellow */
	FM_PORT_COLOR_RED,	    /* Default port color is red */
	FM_PORT_COLOR_OVERRIDE    /* Ignore color */
};

/* A structure for defining FM port resources */
struct fm_port_rsrc_t {
	u32 num; /* Committed required resource */
	u32 extra; /* Extra (not committed) required resource */
};

/**
 * fm_port_cfg_deq_high_priority
 * fm_port:	A pointer to a FM Port module.
 * @high_pri:	True to select high priority, false for normal operation.
 *
 * Calling this routine changes the dequeue priority in the internal driver
 * data base from its default configuration (DFLT_PORT_DEQ_HIGH_PRIORITY)
 * May be used for Non-Rx ports only
 *
 * Allowed only following fm_port_config() and before fm_port_init().
 *
 * Return: 0 on success; Error code otherwise.
 */
int fm_port_cfg_deq_high_priority(struct fm_port_t *fm_port, bool high_pri);

/**
 * fm_port_cfg_deq_prefetch_option
 * fm_port:			A pointer to a FM Port module.
 * @deq_prefetch_option:	New option
 * Calling this routine changes the dequeue prefetch option parameter in the
 * internal driver data base from its default configuration:
 * [DEFAULT_PORT_DEQ_PREFETCH_OPT]
 * Note: Available for some chips only May be used for Non-Rx ports only
 *
 * Allowed only following fm_port_config() and before fm_port_init().
 *
 * Return: 0 on success; Error code otherwise.
 */
int fm_port_cfg_deq_prefetch_option(struct fm_port_t *fm_port,
				    enum fm_port_deq_prefetch_option
				    deq_prefetch_option);

/**
 * fm_port_cfg_buf_prefix_content
 * fm_port				A pointer to a FM Port module.
 * @fm_buffer_prefix_content		A structure of parameters describing
 *					the structure of the buffer.
 *					Out parameter:
 *					Start margin - offset of data from
 *					start of external buffer.
 * Defines the structure, size and content of the application buffer.
 * The prefix, in Tx ports, if 'pass_prs_result', the application should set
 * a value to their offsets in the prefix of the FM will save the first
 * 'priv_data_size', than, depending on 'pass_prs_result' and
 * 'pass_time_stamp', copy parse result and timeStamp, and the packet itself
 * (in this order), to the application buffer, and to offset.
 * Calling this routine changes the buffer margins definitions in the internal
 * driver data base from its default configuration:
 * Data size:  [DEFAULT_PORT_BUFFER_PREFIX_CONTENT_PRIV_DATA_SIZE]
 * Pass Parser result: [DEFAULT_PORT_BUFFER_PREFIX_CONTENT_PASS_PRS_RESULT].
 * Pass timestamp: [DEFAULT_PORT_BUFFER_PREFIX_CONTENT_PASS_TIME_STAMP].
 * May be used for all ports
 *
 * Allowed only following fm_port_config() and before fm_port_init().
 *
 * Return: 0 on success; Error code otherwise.
 */
int fm_port_cfg_buf_prefix_content(struct fm_port_t *fm_port,
				   struct fm_buffer_prefix_content_t
				   *fm_buffer_prefix_content);

/**
 * fm_port_cfg_bcb_wa
 * fm_port	A pointer to a FM Port module.
 *
 * BCB errata workaround.
 * When BCB errata is applicable, the workaround is always performed by FM
 * Controller. Thus, this functions doesn't actually enable errata workaround
 * but rather allows driver to perform adjustments required due to errata
 * workaround execution in FM controller.
 * Applying BCB workaround also configures FM_PORT_FRM_ERR_PHYSICAL errors
 * to be discarded.
 *
 * Allowed only following fm_port_config() and before fm_port_init().
 *
 * Return: 0 on success; Error code otherwise.
 */
int fm_port_cfg_bcb_wa(struct fm_port_t *fm_port);

/**
 * fm_port_disable
 * fm_port	A pointer to a FM Port module.
 *
 * Gracefully disable an FM port. The port will not start new	tasks after all
 * tasks associated with the port are terminated.
 *
 * This is a blocking routine, it returns after port is gracefully stopped,
 * i.e. the port will not except new frames, but it will finish all frames
 * or tasks which were already began.
 * Allowed only following fm_port_init().
 *
 * Return: 0 on success; Error code otherwise.
 */
int fm_port_disable(struct fm_port_t *fm_port);

/**
 * fm_port_enable
 * fm_port: A pointer to a FM Port module.
 *
 * A runtime routine provided to allow disable/enable of port.
 *
 * Allowed only following fm_port_init().
 *
 * Return: 0 on success; Error code otherwise.
 */
int fm_port_enable(struct fm_port_t *fm_port);

#endif /* __FM_PORT_EXT */
