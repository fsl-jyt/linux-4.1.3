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

/* Ethernet generic definitions and enums. */

#ifndef __ENET_EXT_H
#define __ENET_EXT_H

#include "fsl_enet.h"

/* Number of octets (8-bit bytes) in an ethernet address */
#define ENET_NUM_OCTETS_PER_ADDRESS 6
/* Group address mask for ethernet addresses */
#define ENET_GROUP_ADDR	    0x01

/* Ethernet Address */
typedef u8 enet_addr_t[ENET_NUM_OCTETS_PER_ADDRESS];

/* Ethernet MAC-PHY Interface */
enum ethernet_interface {
	ENET_IF_MII = E_ENET_IF_MII,	 /* MII interface */
	ENET_IF_RMII = E_ENET_IF_RMII, /* RMII interface */
	ENET_IF_SMII = E_ENET_IF_SMII, /* SMII interface */
	ENET_IF_GMII = E_ENET_IF_GMII, /* GMII interface */
	ENET_IF_RGMII = E_ENET_IF_RGMII,
					 /* RGMII interface */
	ENET_IF_TBI = E_ENET_IF_TBI,	 /* TBI interface */
	ENET_IF_RTBI = E_ENET_IF_RTBI, /* RTBI interface */
	ENET_IF_SGMII = E_ENET_IF_SGMII,
					 /* SGMII interface */
	ENET_IF_XGMII = E_ENET_IF_XGMII,
					 /* XGMII interface */
	ENET_IF_QSGMII = E_ENET_IF_QSGMII,
					 /* QSGMII interface */
	ENET_IF_XFI = E_ENET_IF_XFI	 /* XFI interface */
};

/* SGMII/QSGII interface with 1000BaseX auto-negotiation between MAC and phy
 * or backplane; Note: 1000BaseX auto-negotiation relates only to interface
 * between MAC and phy/backplane, SGMII phy can still synchronize with far-end
 * phy at 10Mbps, 100Mbps or 1000Mbps
 */
#define ENET_IF_SGMII_BASEX       0x80000000

/* Ethernet Speed (nominal data rate) */
enum ethernet_speed {
	ENET_SPEED_10 = E_ENET_SPEED_10,	 /* 10 Mbps */
	ENET_SPEED_100 = E_ENET_SPEED_100,	 /* 100 Mbps */
	ENET_SPEED_1000 = E_ENET_SPEED_1000,	 /* 1000 Mbps = 1 Gbps */
	ENET_SPEED_10000 = E_ENET_SPEED_10000	 /* 10000 Mbps = 10 Gbps */
};

/* Ethernet mode (combination of MAC-PHY interface and speed) */
enum e_enet_mode {
	ENET_MODE_INVALID = 0,	/* Invalid Ethernet mode */
	/*    10 Mbps MII   */
	ENET_MODE_MII_10 = (ENET_IF_MII | ENET_SPEED_10),
	/*   100 Mbps MII   */
	ENET_MODE_MII_100 = (ENET_IF_MII | ENET_SPEED_100),
	/*    10 Mbps RMII  */
	ENET_MODE_RMII_10 = (ENET_IF_RMII | ENET_SPEED_10),
	/*   100 Mbps RMII  */
	ENET_MODE_RMII_100 = (ENET_IF_RMII | ENET_SPEED_100),
	/*    10 Mbps SMII  */
	ENET_MODE_SMII_10 = (ENET_IF_SMII | ENET_SPEED_10),
	/*   100 Mbps SMII  */
	ENET_MODE_SMII_100 = (ENET_IF_SMII | ENET_SPEED_100),
	/*  1000 Mbps GMII  */
	ENET_MODE_GMII_1000 = (ENET_IF_GMII | ENET_SPEED_1000),
	/*    10 Mbps RGMII */
	ENET_MODE_RGMII_10 = (ENET_IF_RGMII | ENET_SPEED_10),
	/*   100 Mbps RGMII */
	ENET_MODE_RGMII_100 = (ENET_IF_RGMII | ENET_SPEED_100),
	/*  1000 Mbps RGMII */
	ENET_MODE_RGMII_1000 = (ENET_IF_RGMII | ENET_SPEED_1000),
	/*  1000 Mbps TBI   */
	ENET_MODE_TBI_1000 = (ENET_IF_TBI | ENET_SPEED_1000),
	/*  1000 Mbps RTBI  */
	ENET_MODE_RTBI_1000 = (ENET_IF_RTBI | ENET_SPEED_1000),
	/* 10 Mbps SGMII with auto-negotiation between MAC and
	 * SGMII phy according to Cisco SGMII specification
	 */
	ENET_MODE_SGMII_10 = (ENET_IF_SGMII | ENET_SPEED_10),
	/* 100 Mbps SGMII with auto-negotiation between MAC and
	 * SGMII phy according to Cisco SGMII specification
	 */
	ENET_MODE_SGMII_100 = (ENET_IF_SGMII | ENET_SPEED_100),
	/* 1000 Mbps SGMII with auto-negotiation between MAC and
	 * SGMII phy according to Cisco SGMII specification
	 */
	ENET_MODE_SGMII_1000 = (ENET_IF_SGMII | ENET_SPEED_1000),
	/* 10 Mbps SGMII with 1000BaseX auto-negotiation between
	 * MAC and SGMII phy or backplane
	 */
	ENET_MODE_SGMII_BASEX_10 =
	    (ENET_IF_SGMII_BASEX | ENET_IF_SGMII | ENET_SPEED_10),
	/* 100 Mbps SGMII with 1000BaseX auto-negotiation between
	 * MAC and SGMII phy or backplane
	 */
	ENET_MODE_SGMII_BASEX_100 =
	    (ENET_IF_SGMII_BASEX | ENET_IF_SGMII | ENET_SPEED_100),
	/* 1000 Mbps SGMII with 1000BaseX auto-negotiation between
	 * MAC and SGMII phy or backplane
	 */
	ENET_MODE_SGMII_BASEX_1000 =
	    (ENET_IF_SGMII_BASEX | ENET_IF_SGMII | ENET_SPEED_1000),
	/* 1000 Mbps QSGMII with auto-negotiation between MAC and
	 * QSGMII phy according to Cisco QSGMII specification
	 */
	ENET_MODE_QSGMII_1000 = (ENET_IF_QSGMII | ENET_SPEED_1000),
	/* 1000 Mbps QSGMII with 1000BaseX auto-negotiation between
	 * MAC and QSGMII phy or backplane
	 */
	ENET_MODE_QSGMII_BASEX_1000 =
	    (ENET_IF_SGMII_BASEX | ENET_IF_QSGMII | ENET_SPEED_1000),
	/* 10000 Mbps XGMII */
	ENET_MODE_XGMII_10000 = (ENET_IF_XGMII | ENET_SPEED_10000),
	/* 10000 Mbps XFI */
	ENET_MODE_XFI_10000 = (ENET_IF_XFI | ENET_SPEED_10000)
};

#define IS_ENET_MODE_VALID(mode)			\
	(((mode) == ENET_MODE_MII_10) ||		\
	((mode) == ENET_MODE_MII_100) ||		\
	((mode) == ENET_MODE_RMII_10) ||		\
	((mode) == ENET_MODE_RMII_100) ||		\
	((mode) == ENET_MODE_SMII_10) ||		\
	((mode) == ENET_MODE_SMII_100) ||		\
	((mode) == ENET_MODE_GMII_1000) ||		\
	((mode) == ENET_MODE_RGMII_10) ||		\
	((mode) == ENET_MODE_RGMII_100) ||		\
	((mode) == ENET_MODE_RGMII_1000) ||		\
	((mode) == ENET_MODE_TBI_1000) ||		\
	((mode) == ENET_MODE_RTBI_1000) ||		\
	((mode) == ENET_MODE_SGMII_10) ||		\
	((mode) == ENET_MODE_SGMII_100) ||		\
	((mode) == ENET_MODE_SGMII_1000) ||		\
	((mode) == ENET_MODE_SGMII_BASEX_10) ||	\
	((mode) == ENET_MODE_SGMII_BASEX_100) ||	\
	((mode) == ENET_MODE_SGMII_BASEX_1000) ||	\
	((mode) == ENET_MODE_XGMII_10000) ||		\
	((mode) == ENET_MODE_QSGMII_1000) ||		\
	((mode) == ENET_MODE_QSGMII_BASEX_1000) ||	\
	((mode) == ENET_MODE_XFI_10000))

#define MAKE_ENET_MODE(_interface, _speed) \
		      (enum e_enet_mode)((_interface) | (_speed))

#define ENET_INTERFACE_FROM_MODE(mode) \
				(enum ethernet_interface)((mode) & 0x0FFF0000)
#define ENET_SPEED_FROM_MODE(mode) \
			    (enum ethernet_speed)((mode) & 0x0000FFFF)

#define ENET_ADDR_TO_UINT64(_enet_addr)		\
	(u64)(((u64)(_enet_addr)[0] << 40) |		\
	      ((u64)(_enet_addr)[1] << 32) |		\
	      ((u64)(_enet_addr)[2] << 24) |		\
	      ((u64)(_enet_addr)[3] << 16) |		\
	      ((u64)(_enet_addr)[4] << 8) |		\
	      ((u64)(_enet_addr)[5]))

#define MAKE_ENET_ADDR_FROM_UINT64(_addr64, _enet_addr) \
	do { \
		int i; \
		for (i = 0; i < ENET_NUM_OCTETS_PER_ADDRESS; i++) \
			(_enet_addr)[i] = \
			(u8)((_addr64) >> ((5 - i) * 8)); \
	} while (0)

#endif /* __ENET_EXT_H */
