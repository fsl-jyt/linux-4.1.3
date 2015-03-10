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

#include "fsl_fman_dtsec_mii_acc.h"

/**
 * dtsec_mii_get_div() - calculates the value of the dtsec mii divider
 * @dtsec_freq:		dtsec clock frequency (in Mhz)
 *
 * This function calculates the dtsec mii clock divider that determines
 * the MII MDC clock. MII MDC clock will be set to work in the range
 * of 1.5 to 2.5Mhz
 * The output of this function is the value of MIIMCFG[MgmtClk] which
 * implicitly determines the divider value.
 * Note: the dTSEC system clock is equal to 1/2 of the FMan clock.
 * The table below which reflects dtsec_mii_get_div() functionality
 * shows the relations among dtsec_freq, MgmtClk, actual divider
 * and the MII frequency:
 * dtsec freq   MgmtClk	   div	      MII freq Mhz
 * [0.....80]	 1	(1/4)(1/8)    [0   to 2.5]
 * [81...120]	 2	(1/6)(1/8)    [1.6 to 2.5]
 * [121..160]	 3	(1/8)(1/8)    [1.8 to 2.5]
 * [161..200]	 4	(1/10)(1/8)   [2.0 to 2.5]
 * [201..280]	 5	(1/14)(1/8)   [1.8 to 2.5]
 * [281..400]	 6	(1/20)(1/8)   [1.1 to 2.5]
 * [401..560]	 7	(1/28)(1/8)   [1.8 to 2.5]
 * [560..frq]	 7	(1/28)(1/8)   [frq/224]
 * Returns: the MIIMCFG[MgmtClk] appropriate value
 */
static u8 dtsec_mii_get_div(u16 dtsec_freq)
{
	u16 mgmt_clk;

	if (dtsec_freq < 80)
		mgmt_clk = 1;
	else if (dtsec_freq < 120)
		mgmt_clk = 2;
	else if (dtsec_freq < 160)
		mgmt_clk = 3;
	else if (dtsec_freq < 200)
		mgmt_clk = 4;
	else if (dtsec_freq < 280)
		mgmt_clk = 5;
	else if (dtsec_freq < 400)
		mgmt_clk = 6;
	else
		mgmt_clk = 7;

	return (u8)mgmt_clk;
}

int fman_dtsec_mii_write_reg(struct dtsec_mii_reg __iomem *regs, u8 addr,
			     u8 reg, u16 data, u16 dtsec_freq)
{
	u32 tmp;
	int count;

	/* Setup the MII Mgmt clock speed */
	iowrite32be((u32)dtsec_mii_get_div(dtsec_freq), &regs->miimcfg);

	/* Stop the MII management read cycle */
	iowrite32be(0, &regs->miimcom);
	/* Dummy read to make sure MIIMCOM is written */
	tmp = ioread32be(&regs->miimcom);

	/* Setting up MII Management Address Register */
	tmp = (u32)((addr << MIIMADD_PHY_ADDR_SHIFT) | reg);
	iowrite32be(tmp, &regs->miimadd);

	/* Setting up MII Management Control Register with data */
	iowrite32be((u32)data, &regs->miimcon);
	/* Dummy read to make sure MIIMCON is written */
	tmp = ioread32be(&regs->miimcon);

	/* Wait until MII management write is complete */
	count = 100;
	do {
		udelay(1);
	} while (((ioread32be(&regs->miimind)) & MIIMIND_BUSY) && count--);

	if (count == 0)
		return -EBUSY;

	return 0;
}

int fman_dtsec_mii_read_reg(struct dtsec_mii_reg __iomem *regs, u8 addr,
			    u8 reg, u16 *data, u16 dtsec_freq)
{
	u32 tmp;
	int count;

	/* Setup the MII Mgmt clock speed */
	iowrite32be((u32)dtsec_mii_get_div(dtsec_freq), &regs->miimcfg);

	/* Setting up the MII Management Address Register */
	tmp = (u32)((addr << MIIMADD_PHY_ADDR_SHIFT) | reg);
	iowrite32be(tmp, &regs->miimadd);

	/* Perform an MII management read cycle */
	iowrite32be(MIIMCOM_READ_CYCLE, &regs->miimcom);
	/* Dummy read to make sure MIIMCOM is written */
	tmp = ioread32be(&regs->miimcom);

	/* Wait until MII management write is complete */
	count = 100;
	do {
		udelay(1);
	} while (((ioread32be(&regs->miimind)) & MIIMIND_BUSY) && count--);

	if (count == 0)
		return -EBUSY;

	/* Read MII management status  */
	*data = (u16)ioread32be(&regs->miimstat);

	iowrite32be(0, &regs->miimcom);
	/* Dummy read to make sure MIIMCOM is written */
	tmp = ioread32be(&regs->miimcom);

	if (*data == 0xffff)
		return -ENXIO;

	return 0;
}
