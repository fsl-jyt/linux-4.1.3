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

#ifndef __FSL_FMAN_DRV_H
#define __FSL_FMAN_DRV_H

#include <linux/types.h>
#include <linux/device.h>	/* struct device */
#include "fm_ext.h"

/* FM device opaque structure used for type checking */
struct fm;

/* FMan Port structure .., */
struct fm_port_t;

/* A structure of information about each of the external
 * buffer pools used by the port,
 */
struct fm_port_pool_param {
	u8 id;			/* External buffer pool id */
	u16 size;		/* External buffer pool buffer size */
};

/* structure for additional port parameters */
struct fm_port_params {
	u32 errq;	/* Error Queue Id. */
	u32 defq;	/* For Tx and HC - Default Confirmation queue,
			 * 0 means no Tx conf for processed frames.
			 * For Rx and OP - default Rx queue.
			 */
	u8 num_pools;	/* Number of pools use by this port */
	struct fm_port_pool_param pool_param[FM_PORT_MAX_NUM_OF_EXT_POOLS];
	/* Parameters for each pool */
	u16 priv_data_size;
	/* Area that user may save for his own
	 * need (E.g. save the SKB)
	 */
	bool parse_results;	/* Put the parser-results in the Rx/Tx buffer */
	bool hash_results;	/* Put the hash-results in the Rx/Tx buffer */
	bool time_stamp;	/* Put the time-stamp in the Rx/Tx buffer */
	u16 data_align;
	/* value for selecting a data alignment (must be a power of 2);
	 * if write optimization is used, must be >= 16.
	 */
};

/**
 * fm_bind
 * @fm_dev:	the OF handle of the FM device.
 *
 * Bind to a specific FM device.
 *
 * Allowed only after the port was created.
 *
 * Return: A handle of the FM device.
 */
struct fm *fm_bind(struct device *fm_dev);

/**
 * fm_unbind
 * @fm:		A handle of the FM device.
 *
 * Un-bind from a specific FM device.
 *
 * Allowed only after the port was created.
 */
void fm_unbind(struct fm *fm);

/**
 * fm_get_handle
 * @fm:		A handle of the FM device
 *
 * Get pointer to internal FM strcuture
 *
 * Return: A pointer to internal FM structure
 */
void *fm_get_handle(struct fm *fm);

/**
 * fm_get_mem_region
 * @fm:		A handle of the FM device
 *
 * Get FM memory region
 *
 * Return: A structure with FM memory region information
 */

struct resource *fm_get_mem_region(struct fm *fm);

/**
 * fm_port_bind
 * @fm_port_dev:	The OF handle of the FM port device.
 *
 * Bind to a specific FM-port device (may be Rx or Tx port).
 *
 * Allowed only after the port was created.
 *
 * Return: A handle of the FM port device.
 */
struct fm_port_drv_t *fm_port_bind(struct device *fm_port_dev);

/**
 * fm_set_rx_port_params
 * @port:	A handle of the FM port device.
 * @params:	Rx port parameters
 *
 * Configure parameters for a specific Rx FM-port device.
 *
 * Allowed only after the port is binded.
 */
void fm_set_rx_port_params(struct fm_port_drv_t *port,
			   struct fm_port_params *params);

/**
 * fm_port_get_buff_layout_ext_params
 * @port:	A handle of the FM port device.
 * @params:	PCD port parameters
 *
 * Get data_align from the device tree chosen node if applied.
 * This function will only update these two parameters.
 * When this port has no such parameters in the device tree
 * values will be set to 0.
 *
 * Allowed only after the port is binded.
 */
void fm_port_get_buff_layout_ext_params(struct fm_port_drv_t *port,
					struct fm_port_params *params);

/**
 * fm_get_tx_port_channel
 * @port:	A handle of the FM port device.
 *
 * Get qman-channel number for this Tx port.
 * Allowed only after the port is binded.
 *
 * Return: qman-channel number for this Tx port.
 */
int fm_get_tx_port_channel(struct fm_port_drv_t *port);

/**
 * fm_set_tx_port_params
 * @port:	A handle of the FM port device.
 * @params:	Tx port parameters
 *
 * Configure parameters for a specific Tx FM-port device
 *
 * Allowed only after the port is binded.
 */
void fm_set_tx_port_params(struct fm_port_drv_t *port,
			   struct fm_port_params *params);
/**
 * fm_port_drv_handle
 * @port:	A handle of the FM port device.
 *
 * Return: A pointer to the internal FM Port structure
 */
struct fm_port_t *fm_port_drv_handle(const struct fm_port_drv_t *port);

/**
 * fm_get_max_frm
 *
 * Return: Max frame length configured in the FM driver
 */
u16 fm_get_max_frm(void);

/**
 * fm_get_rx_extra_headroom
 *
 * Return: Extra headroom size configured in the FM driver
 */
int fm_get_rx_extra_headroom(void);

#endif /* __FSL_FMAN_DRV_H */
