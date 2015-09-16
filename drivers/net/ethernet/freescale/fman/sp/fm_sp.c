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

#include "fm_sp_common.h"
#include "fm_common.h"
#include "fsl_fman_sp.h"

#include <linux/string.h>

void fm_sp_set_buf_pools_in_asc_order_of_buf_sizes(struct fm_ext_pools_t
						   *fm_ext_pools,
						   u8 *ordered_array,
						   u16 *sizes_array)
{
	u16 buf_size = 0;
	int i = 0, j = 0, k = 0;

	/* First we copy the external buffers pools information
	 * to an ordered local array
	 */
	for (i = 0; i < fm_ext_pools->num_of_pools_used; i++) {
		/* get pool size */
		buf_size = fm_ext_pools->ext_buf_pool[i].size;

		/* keep sizes in an array according to poolId
		 * for direct access
		 */
		sizes_array[fm_ext_pools->ext_buf_pool[i].id] = buf_size;

		/* save poolId in an ordered array according to size */
		for (j = 0; j <= i; j++) {
			/* this is the next free place in the array */
			if (j == i)
				ordered_array[i] =
				    fm_ext_pools->ext_buf_pool[i].id;
			else {
				/* find the right place for this poolId */
				if (buf_size < sizes_array[ordered_array[j]]) {
					/* move the pool_ids one place ahead
					 * to make room for this poolId
					 */
					for (k = i; k > j; k--)
						ordered_array[k] =
						    ordered_array[k - 1];

					/* now k==j, this is the place for
					 * the new size
					 */
					ordered_array[k] =
					    fm_ext_pools->ext_buf_pool[i].id;
					break;
				}
			}
		}
	}
}

int fm_sp_check_buf_pools_params(struct fm_ext_pools_t *fm_ext_pools,
				 struct fm_backup_bm_pools_t
				 *fm_backup_bm_pools,
				 struct fm_buf_pool_depletion_t
				 *fm_buf_pool_depletion,
				 u32 max_num_of_ext_pools,
				 u32 bm_max_num_of_pools)
{
	int i = 0, j = 0;
	bool found;
	u8 count = 0;

	if (fm_ext_pools) {
		if (fm_ext_pools->num_of_pools_used > max_num_of_ext_pools) {
			pr_err("num_of_pools_used can't be larger than %d\n",
			       max_num_of_ext_pools);
			return -EINVAL;
		}
		for (i = 0; i < fm_ext_pools->num_of_pools_used; i++) {
			if (fm_ext_pools->ext_buf_pool[i].id >=
			    bm_max_num_of_pools) {
				pr_err("ext_buf_pools.ext_buf_pool[%d].id can't be larger than %d\n",
				       i, bm_max_num_of_pools);
				return -EINVAL;
			}
			if (!fm_ext_pools->ext_buf_pool[i].size) {
				pr_err("ext_buf_pools.ext_buf_pool[%d].size is 0\n",
				       i);
				return -EINVAL;
			}
		}
	}
	if (!fm_ext_pools && (fm_backup_bm_pools || fm_buf_pool_depletion)) {
		pr_err("backup_bm_pools ot buf_pool_depletion can not be defined without external pools\n");
		return -EINVAL;
	}
	/* backup BM pools indication is valid only for some chip derivatives
	 * (limited by the config routine)
	 */
	if (fm_backup_bm_pools) {
		if (fm_backup_bm_pools->num_of_backup_pools >=
		    fm_ext_pools->num_of_pools_used) {
			pr_err("backup_bm_pools must be smaller than ext_buf_pools.num_of_pools_used\n");
			return -EINVAL;
		}
		found = false;
		for (i = 0; i < fm_backup_bm_pools->num_of_backup_pools; i++) {
			for (j = 0; j < fm_ext_pools->num_of_pools_used; j++) {
				if (fm_backup_bm_pools->pool_ids[i] ==
				    fm_ext_pools->ext_buf_pool[j].id) {
					found = true;
					break;
				}
			}
			if (!found) {
				pr_err("All backup_bm_pools.pool_ids must be included in ext_buf_pools.ext_buf_pool[n].id\n");
				return -EINVAL;
			}
			found = false;
		}
	}

	/* up to ext_buf_pools.num_of_pools_used pools may be defined */
	if (fm_buf_pool_depletion && fm_buf_pool_depletion->
	    pools_grp_mode_enable) {
		if ((fm_buf_pool_depletion->num_of_pools >
		     fm_ext_pools->num_of_pools_used)) {
			pr_err("buf_pool_depletion.num_of_pools can't be larger than %d and can't be larger than num_of_pools_used\n",
			       max_num_of_ext_pools);
			return -EINVAL;
		}
		if (!fm_buf_pool_depletion->num_of_pools) {
			pr_err("buf_pool_depletion.num_of_pools_to_consider can not be 0 when pools_grp_mode_enable=true\n");
			return -EINVAL;
		}
		found = false;
		count = 0;
		/* for each pool that is in pools_to_consider, check if it
		 * is defined in ext_buf_pool
		 */
		for (i = 0; i < bm_max_num_of_pools; i++) {
			if (fm_buf_pool_depletion->pools_to_consider[i]) {
				for (j = 0; j < fm_ext_pools->num_of_pools_used;
				     j++) {
					if (i == fm_ext_pools->
					    ext_buf_pool[j].id) {
						found = true;
						count++;
						break;
					}
				}
				if (!found) {
					pr_err("Pools selected for depletion are not used.\n");
					return -EINVAL;
				}
				found = false;
			}
		}
		/* check that the number of pools that we have checked is
		 * equal to the number announced by the user
		 */
		if (count != fm_buf_pool_depletion->num_of_pools) {
			pr_err("buf_pool_depletion.num_of_pools is larger than the number of pools defined.\n");
			return -EINVAL;
		}
	}

	if (fm_buf_pool_depletion && fm_buf_pool_depletion->
	    single_pool_mode_enable) {
		/* calculate vector for number of pools depletion */
		found = false;
		count = 0;
		for (i = 0; i < bm_max_num_of_pools; i++) {
			if (fm_buf_pool_depletion->
			    pools_to_consider_for_single_mode[i]) {
				for (j = 0; j < fm_ext_pools->num_of_pools_used;
				     j++) {
					if (i == fm_ext_pools->
					    ext_buf_pool[j].id) {
						found = true;
						count++;
						break;
					}
				}
				if (!found) {
					pr_err("Pools selected for depletion are not used.\n");
					return -EINVAL;
				}
				found = false;
			}
		}
		if (!count) {
			pr_err("No pools defined for single buffer mode pool depletion.\n");
			return -EINVAL;
		}
	}

	return 0;
}

int fm_sp_check_int_context_params(struct fm_sp_int_context_data_copy_t *
				  fm_sp_int_context_data_copy)
{
	/* Check that divisible by 16 and not larger than 240 */
	if (fm_sp_int_context_data_copy->int_context_offset > MAX_INT_OFFSET) {
		pr_err("int_context.int_context_offset can't be larger than %d\n",
		       MAX_INT_OFFSET);
		return -EINVAL;
	}
	if (fm_sp_int_context_data_copy->int_context_offset % OFFSET_UNITS) {
		pr_err("int_context.int_context_offset has to be divisible by %d\n",
		       OFFSET_UNITS);
		return -EINVAL;
	}
	/* check that ic size+ic internal offset, does not exceed
	 * ic block size
	 */
	if (fm_sp_int_context_data_copy->size +
	    fm_sp_int_context_data_copy->int_context_offset > MAX_IC_SIZE) {
		pr_err("int_context.size + int_context.int_context_offset has to be smaller than %d\n",
		       MAX_IC_SIZE);
		return -EINVAL;
	}
	/* Check that divisible by 16 and not larger than 256 */
	if (fm_sp_int_context_data_copy->size % OFFSET_UNITS) {
		pr_err("int_context.size  has to be divisible by %d\n",
		       OFFSET_UNITS);
		return -EINVAL;
	}
	/* Check that divisible by 16 and not larger than 4K */
	if (fm_sp_int_context_data_copy->ext_buf_offset > MAX_EXT_OFFSET) {
		pr_err("int_context.ext_buf_offset can't be larger than %d\n",
		       MAX_EXT_OFFSET);
		return -EINVAL;
	}
	if (fm_sp_int_context_data_copy->ext_buf_offset % OFFSET_UNITS) {
		pr_err("int_context.ext_buf_offset has to be divisible by %d\n",
		       OFFSET_UNITS);
		return -EINVAL;
	}
	return 0;
}

int fm_sp_check_buf_margins(struct fm_sp_buf_margins_t *fm_sp_buf_margins)
{
	/* Check the margin definition */
	if (fm_sp_buf_margins->start_margins > MAX_EXT_BUFFER_OFFSET) {
		pr_err("buf_margins.start_margins can't be larger than %d\n",
		       MAX_EXT_BUFFER_OFFSET);
		return -EINVAL;
	}
	if (fm_sp_buf_margins->end_margins > MAX_EXT_BUFFER_OFFSET) {
		pr_err("buf_margins.end_margins can't be larger than %d\n",
		       MAX_EXT_BUFFER_OFFSET);
		return -EINVAL;
	}
	return 0;
}

int fm_sp_build_buffer_structure(struct fm_sp_int_context_data_copy_t *
				 fm_sp_int_context_data_copy,
				 struct fm_buffer_prefix_content_t *
				 buffer_prefix_content,
				 struct fm_sp_buf_margins_t
				 *fm_sp_buf_margins,
				 struct fm_sp_buffer_offsets_t
				 *fm_sp_buffer_offsets,
				 u8 *internal_buf_offset)
{
	u32 tmp;

	/* Align start of internal context data to 16 byte */
	fm_sp_int_context_data_copy->ext_buf_offset = (u16)
		((buffer_prefix_content->priv_data_size & (OFFSET_UNITS - 1)) ?
		((buffer_prefix_content->priv_data_size + OFFSET_UNITS) &
			~(u16)(OFFSET_UNITS - 1)) :
		buffer_prefix_content->priv_data_size);

	/* Translate margin and int_context params to FM parameters */
	/* Initialize with illegal value. Later we'll set legal values. */
	fm_sp_buffer_offsets->prs_result_offset = (u32)ILLEGAL_BASE;
	fm_sp_buffer_offsets->time_stamp_offset = (u32)ILLEGAL_BASE;
	fm_sp_buffer_offsets->hash_result_offset = (u32)ILLEGAL_BASE;

	/* Internally the driver supports 4 options
	 * 1. prsResult/timestamp/hashResult selection (in fact 8 options,
	 * but for simplicity we'll
	 * relate to it as 1).
	 * 2. All IC context (from AD) not including debug.
	 */

	/* This case covers the options under 1 */
	/* Copy size must be in 16-byte granularity. */
	fm_sp_int_context_data_copy->size =
	    (u16)((buffer_prefix_content->pass_prs_result ? 32 : 0) +
		  ((buffer_prefix_content->pass_time_stamp ||
		  buffer_prefix_content->pass_hash_result) ? 16 : 0));

	/* Align start of internal context data to 16 byte */
	fm_sp_int_context_data_copy->int_context_offset =
	    (u8)(buffer_prefix_content->pass_prs_result ? 32 :
		 ((buffer_prefix_content->pass_time_stamp ||
		 buffer_prefix_content->pass_hash_result) ? 64 : 0));

	if (buffer_prefix_content->pass_prs_result)
		fm_sp_buffer_offsets->prs_result_offset =
		    fm_sp_int_context_data_copy->ext_buf_offset;
	if (buffer_prefix_content->pass_time_stamp)
		fm_sp_buffer_offsets->time_stamp_offset =
		    buffer_prefix_content->pass_prs_result ?
		    (fm_sp_int_context_data_copy->ext_buf_offset +
			sizeof(struct fm_prs_result_t)) :
		    fm_sp_int_context_data_copy->ext_buf_offset;
	if (buffer_prefix_content->pass_hash_result)
		/* If PR is not requested, whether TS is
		 * requested or not, IC will be copied from TS
			 */
		fm_sp_buffer_offsets->hash_result_offset =
		buffer_prefix_content->pass_prs_result ?
			(fm_sp_int_context_data_copy->ext_buf_offset +
				sizeof(struct fm_prs_result_t) + 8) :
			fm_sp_int_context_data_copy->ext_buf_offset + 8;

	if (fm_sp_int_context_data_copy->size)
		fm_sp_buf_margins->start_margins =
		    (u16)(fm_sp_int_context_data_copy->ext_buf_offset +
			  fm_sp_int_context_data_copy->size);
	else
		/* No Internal Context passing, STartMargin is
		 * immediately after private_info
		 */
		fm_sp_buf_margins->start_margins =
		    buffer_prefix_content->priv_data_size;

	/* align data start */
	tmp = (u32)(fm_sp_buf_margins->start_margins %
		    buffer_prefix_content->data_align);
	if (tmp)
		fm_sp_buf_margins->start_margins +=
		    (buffer_prefix_content->data_align - tmp);
	fm_sp_buffer_offsets->data_offset = fm_sp_buf_margins->start_margins;

	return 0;
}
