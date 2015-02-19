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
