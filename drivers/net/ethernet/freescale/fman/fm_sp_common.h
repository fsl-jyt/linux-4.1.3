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

/* FM SP  ... */
#ifndef __FM_SP_COMMON_H
#define __FM_SP_COMMON_H

#include "fm_ext.h"
#include "fsl_fman.h"

#define ILLEGAL_BASE    (~0)

/* defaults */
#define DEFAULT_FM_SP_BUFFER_PREFIX_CONTENT_PRIV_DATA_SIZE      0
#define DEFAULT_FM_SP_BUFFER_PREFIX_CONTENT_PRIV_PASS_PRS_RESULT     false
#define DEFAULT_FM_SP_BUFFER_PREFIX_CONTEXT_PASS_TIME_STAMP     false
#define DEFAULT_FM_SP_BUFFER_PREFIX_CONTEXT_DATA_ALIGN         64

/* structure for defining internal context copying */
struct fm_sp_int_context_data_copy_t {
	/* < Offset in External buffer to which internal
	 *  context is copied to (Rx) or taken from (Tx, Op).
	 */
	u16 ext_buf_offset;
	/* Offset within internal context to copy from
	 * (Rx) or to copy to (Tx, Op).
	 */
	u8 int_context_offset;
	/* Internal offset size to be copied */
	u16 size;
};

/*  struct for defining external buffer margins */
struct fm_sp_buf_margins_t {
	/* Number of bytes to be left at the beginning
	 * of the external buffer (must be divisible by 16)
	 */
	u16 start_margins;
	/* number of bytes to be left at the end
	 * of the external buffer(must be divisible by 16)
	 */
	u16 end_margins;
};

struct fm_sp_buffer_offsets_t {
	u32 data_offset;
	u32 prs_result_offset;
	u32 time_stamp_offset;
	u32 hash_result_offset;
};

int fm_sp_build_buffer_structure(struct fm_sp_int_context_data_copy_t
				 *fm_port_int_context_data_copy,
				 struct fm_buffer_prefix_content_t
				 *buffer_prefix_content,
				 struct fm_sp_buf_margins_t
				 *fm_port_buf_margins,
				 struct fm_sp_buffer_offsets_t
				 *fm_port_buffer_offsets,
				 u8 *internal_buf_offset);

int fm_sp_check_int_context_params(struct fm_sp_int_context_data_copy_t *
				   fm_sp_int_context_data_copy);
int fm_sp_check_buf_pools_params(struct fm_ext_pools_t *fm_ext_pools,
				 struct fm_backup_bm_pools_t
				 *fm_backup_bm_pools,
				 struct fm_buf_pool_depletion_t
				 *fm_buf_pool_depletion,
				 u32 max_num_of_ext_pools,
				 u32 bm_max_num_of_pools);
int fm_sp_check_buf_margins(struct fm_sp_buf_margins_t *fm_sp_buf_margins);
void fm_sp_set_buf_pools_in_asc_order_of_buf_sizes(struct fm_ext_pools_t
						   *fm_ext_pools,
						   u8 *ordered_array,
						   u16 *sizes_array);
#endif /* __FM_SP_COMMON_H */
