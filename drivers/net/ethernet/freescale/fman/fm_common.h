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
#ifndef __FM_COMMON_H
#define __FM_COMMON_H

#include "fm_ext.h"

/* Enum for inter-module interrupts registration */
enum fm_event_modules {
	FM_MOD_MAC = 0,		/* MAC event */
	FM_MOD_FMAN_CTRL,	/* FMAN Controller */
	FM_MOD_DUMMY_LAST
};

/* Enum for interrupts types */
enum fm_intr_type {
	FM_INTR_TYPE_ERR,
	FM_INTR_TYPE_NORMAL
};

/* Enum for inter-module interrupts registration */
enum fm_inter_module_event {
	FM_EV_ERR_MAC0 = 0,	/* MAC 0 error event */
	FM_EV_ERR_MAC1,		/* MAC 1 error event */
	FM_EV_ERR_MAC2,		/* MAC 2 error event */
	FM_EV_ERR_MAC3,		/* MAC 3 error event */
	FM_EV_ERR_MAC4,		/* MAC 4 error event */
	FM_EV_ERR_MAC5,		/* MAC 5 error event */
	FM_EV_ERR_MAC6,		/* MAC 6 error event */
	FM_EV_ERR_MAC7,		/* MAC 7 error event */
	FM_EV_ERR_MAC8,		/* MAC 8 error event */
	FM_EV_ERR_MAC9,		/* MAC 9 error event */
	FM_EV_MAC0,		/* MAC 0 event (Magic packet detection) */
	FM_EV_MAC1,		/* MAC 1 event (Magic packet detection) */
	FM_EV_MAC2,		/* MAC 2 (Magic packet detection) */
	FM_EV_MAC3,		/* MAC 3 (Magic packet detection) */
	FM_EV_MAC4,		/* MAC 4 (Magic packet detection) */
	FM_EV_MAC5,		/* MAC 5 (Magic packet detection) */
	FM_EV_MAC6,		/* MAC 6 (Magic packet detection) */
	FM_EV_MAC7,		/* MAC 7 (Magic packet detection) */
	FM_EV_MAC8,		/* MAC 8 event (Magic packet detection) */
	FM_EV_MAC9,		/* MAC 9 event (Magic packet detection) */
	FM_EV_FMAN_CTRL_0,	/* Fman controller event 0 */
	FM_EV_FMAN_CTRL_1,	/* Fman controller event 1 */
	FM_EV_FMAN_CTRL_2,	/* Fman controller event 2 */
	FM_EV_FMAN_CTRL_3,	/* Fman controller event 3 */
	FM_EV_DUMMY_LAST
};

/* FM IP BLOCK versions */
#define FM_IP_BLOCK_P2_P3_P5		3
#define FM_IP_BLOCK_P4			2
#define FM_IP_BLOCK_B_T			6

#define MODULE_NAME_SIZE        30
#define DUMMY_PORT_ID           0

#define FM_LIODN_OFFSET_MASK    0x3FF

/* Port Id defines */
#define BASE_RX_PORTID			0x08
#define BASE_TX_PORTID			0x28

static inline u8 hw_port_id_to_sw_port_id(u8 major, u8 hw_port_id)
{
	u8 sw_port_id = 0;

	if (hw_port_id >= BASE_TX_PORTID) {
		sw_port_id = hw_port_id - BASE_TX_PORTID;
	} else if (hw_port_id >= BASE_RX_PORTID) {
		sw_port_id = hw_port_id - BASE_RX_PORTID;
	} else {
		sw_port_id = DUMMY_PORT_ID;
		BUG_ON(false);
	}

	return sw_port_id;
}

#define BMI_MAX_FIFO_SIZE                   (FM_MURAM_SIZE)
#define BMI_FIFO_UNITS                      0x100

struct fm_intr_src_t {
	void (*isr_cb)(void *src_arg);
	void *src_handle;
};

/* enum for defining MAC types */
enum fm_mac_type {
	FM_MAC_10G = 0,	    /* 10G MAC */
	FM_MAC_1G	    /* 1G MAC */
};

/* Structure for port-FM communication
 * during fm_port_init. Fields commented 'IN' are passed
 * by the port module to be used by the FM module.
 * Fields commented 'OUT' will be filled by FM before returning to port.
 * Some fields are optional (depending on configuration) and
 * will be analized by the port and FM modules accordingly.
 */
struct fm_inter_module_port_init_params_t {
	u8 port_id;			/* IN. port Id */
	enum fm_port_type port_type;	/* IN. Port type */
	enum fm_port_speed port_speed;	/* IN. Port speed */
	u16 liodn_offset;		/* IN. Port's requested resource */
	u8 num_of_tasks;		/* IN. Port's requested resource */
	u8 num_of_extra_tasks;		/* IN. Port's requested resource */
	u8 num_of_open_dmas;		/* IN. Port's requested resource */
	u8 num_of_extra_open_dmas;	/* IN. Port's requested resource */
	u32 size_of_fifo;		/* IN. Port's requested resource */
	u32 extra_size_of_fifo;		/* IN. Port's requested resource */
	u8 deq_pipeline_depth;		/* IN. Port's requested resource */
	u16 max_frame_length;		/* IN. Port's max frame length. */
	u16 liodn_base;
	/* LIODN base for this port, to be used together with LIODN offset. */
};

void fm_register_intr(struct fm_t *fm, enum fm_event_modules mod, u8 mod_id,
		      enum fm_intr_type intr_type,
		      void (*f_isr)(void *h_src_arg), void *h_src_arg);

void fm_unregister_intr(struct fm_t *fm, enum fm_event_modules mod, u8 mod_id,
			enum fm_intr_type intr_type);

struct muram_info *fm_get_muram_pointer(struct fm_t *fm);

int fm_reset_mac(struct fm_t *fm, u8 mac_id);

u16 fm_get_clock_freq(struct fm_t *fm);

u8 fm_get_id(struct fm_t *fm);

int fm_get_set_port_params(struct fm_t *fm,
			   struct fm_inter_module_port_init_params_t
			   *port_params);

int fm_set_num_of_open_dmas(struct fm_t *fm, u8 port_id, u8 *num_of_open_dmas,
			    u8 *num_of_extra_open_dmas);
int fm_set_num_of_tasks(struct fm_t *fm, u8 port_id, u8 *num_of_tasks,
			u8 *num_of_extra_tasks);
int fm_set_size_of_fifo(struct fm_t *fm, u8 port_id, u32 *size_of_fifo,
			u32 *extra_size_of_fifo);

u32 fm_get_bmi_max_fifo_size(struct fm_t *fm);

int fm_set_mac_max_frame(struct fm_t *fm, enum fm_mac_type type, u8 mac_id,
			 u16 mtu);

#endif /* __FM_COMMON_H */
