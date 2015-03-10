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

#ifndef __FSL_FMAN_DTSEC_MII_ACC_H
#define __FSL_FMAN_DTSEC_MII_ACC_H

#include <linux/io.h>

/* MII Management Configuration Register */
#define MIIMCFG_RESET_MGMT		0x80000000
#define MIIMCFG_MGNTCLK_MASK		0x00000007
#define MIIMCFG_MGNTCLK_SHIFT		0

/* MII	Management Command Register */
#define MIIMCOM_SCAN_CYCLE		0x00000002
#define MIIMCOM_READ_CYCLE		0x00000001

/* MII	Management Address Register */
#define MIIMADD_PHY_ADDR_SHIFT		8
#define MIIMADD_PHY_ADDR_MASK		0x00001f00

#define MIIMADD_REG_ADDR_SHIFT		0
#define MIIMADD_REG_ADDR_MASK		0x0000001f

/* MII Management Indicator Register */
#define MIIMIND_BUSY			0x00000001

/* PHY Control Register */
#define PHY_CR_PHY_RESET    0x8000
#define PHY_CR_LOOPBACK	    0x4000
#define PHY_CR_SPEED0	    0x2000
#define PHY_CR_ANE	    0x1000
#define PHY_CR_RESET_AN	    0x0200
#define PHY_CR_FULLDUPLEX   0x0100
#define PHY_CR_SPEED1	    0x0040

#define PHY_TBICON_SRESET   0x8000
#define PHY_TBICON_SPEED2   0x0020
#define PHY_TBICON_CLK_SEL  0x0020
#define PHY_TBIANA_SGMII    0x4001
#define PHY_TBIANA_1000X    0x01a0
/* register map */

/* MII Configuration Control Memory Map Registers */
struct dtsec_mii_reg {
	u32 reserved1[72];
	u32 miimcfg;	/* MII Mgmt:configuration */
	u32 miimcom;	/* MII Mgmt:command	  */
	u32 miimadd;	/* MII Mgmt:address	  */
	u32 miimcon;	/* MII Mgmt:control 3	  */
	u32 miimstat;	/* MII Mgmt:status	  */
	u32 miimind;	/* MII Mgmt:indicators	  */
};

void fman_dtsec_mii_init(struct dtsec_mii_reg __iomem *regs, u16 dtsec_freq);
int fman_dtsec_mii_write_reg(struct dtsec_mii_reg __iomem *regs, u8 addr,
			     u8 reg, u16 data, u16 dtsec_freq);
int fman_dtsec_mii_read_reg(struct dtsec_mii_reg __iomem *regs, u8 addr,
			    u8 reg, u16 *data, u16 dtsec_freq);

#endif	/* __FSL_FMAN_DTSEC_MII_ACC_H */
