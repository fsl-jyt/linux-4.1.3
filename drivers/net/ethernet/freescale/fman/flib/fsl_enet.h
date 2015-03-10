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

#ifndef __FSL_ENET_H
#define __FSL_ENET_H

/* Ethernet MAC-PHY Interface */
enum enet_interface {
	E_ENET_IF_MII = 0x00010000,	      /* MII interface */
	E_ENET_IF_RMII = 0x00020000,	      /* RMII interface */
	E_ENET_IF_SMII = 0x00030000,	      /* SMII interface */
	E_ENET_IF_GMII = 0x00040000,	      /* GMII interface */
	E_ENET_IF_RGMII = 0x00050000,	      /* RGMII interface */
	E_ENET_IF_TBI = 0x00060000,	      /* TBI interface */
	E_ENET_IF_RTBI = 0x00070000,	      /* RTBI interface */
	E_ENET_IF_SGMII = 0x00080000,	      /* SGMII interface */
	E_ENET_IF_XGMII = 0x00090000,	      /* XGMII interface */
	E_ENET_IF_QSGMII = 0x000a0000,	      /* QSGMII interface */
	E_ENET_IF_XFI = 0x000b0000	      /* XFI interface */
};

/* Ethernet Speed (nominal data rate) */
enum enet_speed {
	E_ENET_SPEED_10 = 10,		/* 10 Mbps */
	E_ENET_SPEED_100 = 100,		/* 100 Mbps */
	E_ENET_SPEED_1000 = 1000,	/* 1000 Mbps = 1 Gbps */
	E_ENET_SPEED_10000 = 10000	/* 10000 Mbps = 10 Gbps */
};

/* SGMII/QSGII interface with 1000BaseX auto-negotiation between MAC
 * and phy or backplane;
 * Note: 1000BaseX auto-negotiation relates only to interface between MAC
 * and phy/backplane, SGMII phy can still synchronize with far-end phy at
 * 10Mbps, 100Mbps or 1000Mbps
 */
#define ENET_IF_SGMII_BASEX 0x80000000

enum enet_mode {
	E_ENET_MODE_INVALID = 0,
	/* Invalid Ethernet mode */
	E_ENET_MODE_MII_10 = (E_ENET_IF_MII | E_ENET_SPEED_10),
	/* 10 Mbps MII */
	E_ENET_MODE_MII_100 = (E_ENET_IF_MII | E_ENET_SPEED_100),
	/* 100 Mbps MII */
	E_ENET_MODE_RMII_10 = (E_ENET_IF_RMII | E_ENET_SPEED_10),
	/* 10 Mbps RMII */
	E_ENET_MODE_RMII_100 = (E_ENET_IF_RMII | E_ENET_SPEED_100),
	/* 100 Mbps RMII */
	E_ENET_MODE_SMII_10 = (E_ENET_IF_SMII | E_ENET_SPEED_10),
	/* 10 Mbps SMII */
	E_ENET_MODE_SMII_100 = (E_ENET_IF_SMII | E_ENET_SPEED_100),
	/* 100 Mbps SMII */
	E_ENET_MODE_GMII_1000 = (E_ENET_IF_GMII | E_ENET_SPEED_1000),
	/* 1000 Mbps GMII */
	E_ENET_MODE_RGMII_10 = (E_ENET_IF_RGMII | E_ENET_SPEED_10),
	/* 10 Mbps RGMII */
	E_ENET_MODE_RGMII_100 = (E_ENET_IF_RGMII | E_ENET_SPEED_100),
	/* 100 Mbps RGMII */
	E_ENET_MODE_RGMII_1000 = (E_ENET_IF_RGMII | E_ENET_SPEED_1000),
	/* 1000 Mbps RGMII */
	E_ENET_MODE_TBI_1000 = (E_ENET_IF_TBI | E_ENET_SPEED_1000),
	/* 1000 Mbps TBI */
	E_ENET_MODE_RTBI_1000 = (E_ENET_IF_RTBI | E_ENET_SPEED_1000),
	/* 1000 Mbps RTBI */
	E_ENET_MODE_SGMII_10 = (E_ENET_IF_SGMII | E_ENET_SPEED_10),
	/* 10 Mbps SGMII with auto-negotiation between MAC and
	 * SGMII phy according to Cisco SGMII specification
	 */
	E_ENET_MODE_SGMII_100 = (E_ENET_IF_SGMII | E_ENET_SPEED_100),
	/* 100 Mbps SGMII with auto-negotiation between MAC and
	 * SGMII phy according to Cisco SGMII specification
	 */
	E_ENET_MODE_SGMII_1000 = (E_ENET_IF_SGMII | E_ENET_SPEED_1000),
	/* 1000 Mbps SGMII with auto-negotiation between MAC and
	 * SGMII phy according to Cisco SGMII specification
	 */
	E_ENET_MODE_SGMII_BASEX_10 = (ENET_IF_SGMII_BASEX | E_ENET_IF_SGMII
				      | E_ENET_SPEED_10),
	/* 10 Mbps SGMII with 1000BaseX auto-negotiation between
	 * MAC and SGMII phy or backplane
	 */
	E_ENET_MODE_SGMII_BASEX_100 = (ENET_IF_SGMII_BASEX | E_ENET_IF_SGMII
				       | E_ENET_SPEED_100),
	/* 100 Mbps SGMII with 1000BaseX auto-negotiation between
	 * MAC and SGMII phy or backplane
	 */
	E_ENET_MODE_SGMII_BASEX_1000 = (ENET_IF_SGMII_BASEX | E_ENET_IF_SGMII
					| E_ENET_SPEED_1000),
	/* 1000 Mbps SGMII with 1000BaseX auto-negotiation between
	 * MAC and SGMII phy or backplane
	 */
	E_ENET_MODE_QSGMII_1000 = (E_ENET_IF_QSGMII | E_ENET_SPEED_1000),
	/* 1000 Mbps QSGMII with auto-negotiation between MAC and
	 * QSGMII phy according to Cisco QSGMII specification
	 */
	E_ENET_MODE_QSGMII_BASEX_1000 = (ENET_IF_SGMII_BASEX | E_ENET_IF_QSGMII
					 | E_ENET_SPEED_1000),
	/* 1000 Mbps QSGMII with 1000BaseX auto-negotiation between
	 * MAC and QSGMII phy or backplane
	 */
	E_ENET_MODE_XGMII_10000 = (E_ENET_IF_XGMII | E_ENET_SPEED_10000),
	/* 10000 Mbps XGMII */
	E_ENET_MODE_XFI_10000 = (E_ENET_IF_XFI | E_ENET_SPEED_10000)
	/* 10000 Mbps XFI */
};

#endif	/* __FSL_ENET_H */
