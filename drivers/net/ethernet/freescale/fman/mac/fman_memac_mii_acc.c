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

#include "fsl_fman_memac_mii_acc.h"

static int write_phy_reg_10g(struct memac_mii_access_mem_map __iomem *mii_regs,
			     u8 phy_addr, u8 reg, u16 data)
{
	u32 tmp_reg;
	int count;

	tmp_reg = ioread32be(&mii_regs->mdio_cfg);
	/* Leave only MDIO_CLK_DIV bits set on */
	tmp_reg &= MDIO_CFG_CLK_DIV_MASK;
	/* Set maximum MDIO_HOLD value to allow phy to see
	 * change of data signal
	 */
	tmp_reg |= MDIO_CFG_HOLD_MASK;
	/* Add 10G interface mode */
	tmp_reg |= MDIO_CFG_ENC45;
	iowrite32be(tmp_reg, &mii_regs->mdio_cfg);

	/* Wait for command completion */
	count = 100;
	do {
		udelay(1);
	} while (((ioread32be(&mii_regs->mdio_cfg)) & MDIO_CFG_BSY) && --count);

	if (count == 0)
		return -EBUSY;

	/* Specify phy and register to be accessed */
	iowrite32be(phy_addr, &mii_regs->mdio_ctrl);
	iowrite32be(reg, &mii_regs->mdio_addr);

	count = 100;
	do {
		udelay(1);
	} while (((ioread32be(&mii_regs->mdio_cfg)) & MDIO_CFG_BSY) && --count);

	if (count == 0)
		return -EBUSY;

	/* Write data */
	iowrite32be(data, &mii_regs->mdio_data);

	/* Wait for write transaction end */
	count = 100;
	do {
		udelay(1);
	} while (((ioread32be(&mii_regs->mdio_data)) & MDIO_DATA_BSY) &&
		 --count);

	if (count == 0)
		return -EBUSY;

	return 0;
}

static int write_phy_reg_1g(struct memac_mii_access_mem_map __iomem *mii_regs,
			    u8 phy_addr, u8 reg, u16 data)
{
	u32 tmp_reg;
	int count;

	/* Leave only MDIO_CLK_DIV and MDIO_HOLD bits set on */
	tmp_reg = ioread32be(&mii_regs->mdio_cfg);
	tmp_reg &= (MDIO_CFG_CLK_DIV_MASK | MDIO_CFG_HOLD_MASK);
	iowrite32be(tmp_reg, &mii_regs->mdio_cfg);

	/* Wait for command completion */
	count = 100;
	do {
		udelay(1);
	} while (((ioread32be(&mii_regs->mdio_cfg)) & MDIO_CFG_BSY) && --count);

	if (count == 0)
		return -EBUSY;

	/* Write transaction */
	tmp_reg = (phy_addr << MDIO_CTL_PHY_ADDR_SHIFT);
	tmp_reg |= reg;
	iowrite32be(tmp_reg, &mii_regs->mdio_ctrl);

	/* Wait for command completion */
	count = 100;
	do {
		udelay(1);
	} while (((ioread32be(&mii_regs->mdio_cfg)) & MDIO_CFG_BSY) && --count);

	if (count == 0)
		return -EBUSY;

	iowrite32be(data, &mii_regs->mdio_data);

	/* Wait for write transaction to end */
	count = 100;
	do {
		udelay(1);
	} while (((ioread32be(&mii_regs->mdio_data)) & MDIO_DATA_BSY) &&
		 --count);

	if (count == 0)
		return -EBUSY;

	return 0;
}

int fman_memac_mii_write_phy_reg(struct memac_mii_access_mem_map __iomem
				 *mii_regs, u8 phy_addr, u8 reg, u16 data,
				 enum enet_speed enet_speed)
{
	int err = 0;
	/* Figure out interface type - 10G vs 1G.
	 * In 10G interface both phy_addr and devAddr present.
	 */
	if (enet_speed == E_ENET_SPEED_10000)
		err = write_phy_reg_10g(mii_regs, phy_addr, reg, data);
	else
		err = write_phy_reg_1g(mii_regs, phy_addr, reg, data);

	return err;
}
