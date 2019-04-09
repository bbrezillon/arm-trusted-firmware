/*
 * Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <dram_regs.h>
#include <m0_param.h>
#include <pmu_bits.h>
#include <pmu_regs.h>
#include "misc_regs.h"
#include "rk3399_mcu.h"

static uint32_t gatedis_con0;

#define PD_VOP_PWR_STAT		(1 << 20)

/* CRU_CLKGATE10_CON */
#define ACLK_VOP0_PRE_SRC_EN	(1 << 8)
#define HCLK_VOP0_PRE_EN	(1 << 9)
#define ACLK_VOP1_PRE_SRC_EN	(1 << 10)
#define HCLK_VOP1_PRE_EN	(1 << 11)
#define DCLK_VOP0_SRC_EN	(1 << 12)
#define DCLK_VOP1_SRC_EN	(1 << 13)

/* CRU_CLKGATE28_CON */
#define HCLK_VOP0_EN		(1 << 2)
#define ACLK_VOP0_EN		(1 << 3)
#define HCLK_VOP1_EN		(1 << 6)
#define ACLK_VOP1_EN		(1 << 7)

/* VOP */
#define VOP_LITE_BASE		0x478f0000
#define VOP_BIG_BASE		0x47900000
#define VOP_SYS_CTRL		0x8
#define VOP_SYS_CTRL1		0xc
#define VOP_WIN0_CTRL0		0x30
#define	VOP_INTR_CLEAR0		0x284
#define VOP_INTR_RAW_STATUS0	0x28c

/* VOP_SYS_CTRL */
#define VOP_DMA_STOP_EN		(1 << 21)
#define VOP_STANDBY_EN		(1 << 22)

/* VOP_WIN0_CTRL0 */
#define WB_ENABLE		(1 << 0)

/* VOP_INTR_CLEAR0 */
#define	INT_CLR_DMA_FINISH	(1 << 15)
#define INT_CLR_LINE_FLAG1	(1 << 4)
#define INT_CLR_LINE_FLAG0	(1 << 3)

/* VOP_INTR_RAW_STATUS0 */
#define	INT_RAW_STATUS_DMA_FINISH	(1 << 15)
#define INT_RAW_STATUS_LINE_FLAG1	(1 << 4)
#define INT_RAW_STATUS_LINE_FLAG0	(1 << 3)

static inline int check_dma_status(uint32_t vop_addr, uint32_t *clr_dma_flag)
{
	if (*clr_dma_flag) {
		mmio_write_32(vop_addr + VOP_INTR_CLEAR0, 0x80008000);
		*clr_dma_flag = 0;
	}

	if ((mmio_read_32(vop_addr + VOP_SYS_CTRL) &
	     (VOP_STANDBY_EN | VOP_DMA_STOP_EN)) ||
	    !(mmio_read_32(vop_addr + VOP_WIN0_CTRL0) & WB_ENABLE) ||
	    (mmio_read_32(vop_addr + VOP_INTR_RAW_STATUS0) &
	    INT_RAW_STATUS_DMA_FINISH))
		return 1;

	return 0;
}

static int wait_vop_dma_finish(void)
{
	uint32_t clr_dma_flag = 1;
	uint32_t ret = 0;

	stopwatch_init_usecs_expire(60000);
	while (((mmio_read_32(PMU_BASE + PMU_PWRDN_ST) &
		PD_VOP_PWR_STAT) == 0)) {
		/*
		 * VOPL case:
		 * CRU_CLKGATE10_CON(bit10): ACLK_VOP1_PRE_SRC_EN
		 * CRU_CLKGATE10_CON(bit11): HCLK_VOP1_PRE_EN
		 * CRU_CLKGATE10_CON(bit13): DCLK_VOP1_SRC_EN
		 * CRU_CLKGATE28_CON(bit7): ACLK_VOP1_EN
		 * CRU_CLKGATE28_CON(bit6): HCLK_VOP1_EN
		 *
		 * VOPB case:
		 * CRU_CLKGATE10_CON(bit8): ACLK_VOP0_PRE_SRC_EN
		 * CRU_CLKGATE10_CON(bit9): HCLK_VOP0_PRE_EN
		 * CRU_CLKGATE10_CON(bit12): DCLK_VOP0_SRC_EN
		 * CRU_CLKGATE28_CON(bit3): ACLK_VOP0_EN
		 * CRU_CLKGATE28_CON(bit2): HCLK_VOP0_EN
		 */
		if (((mmio_read_32(CRU_BASE + CRU_CLKGATE10_CON) &
		      0x2c00) == 0) &&
		    ((mmio_read_32(CRU_BASE + CRU_CLKGATE28_CON) &
		      0xc0) == 0)) {
			if (check_dma_status(VOP_LITE_BASE, &clr_dma_flag))
				return;
		} else if (((mmio_read_32(CRU_BASE + CRU_CLKGATE10_CON) &
			     0x1300) == 0) &&
			   ((mmio_read_32(CRU_BASE + CRU_CLKGATE28_CON) &
			     0x0c) == 0)) {
			if (check_dma_status(VOP_BIG_BASE, &clr_dma_flag))
				return;
		} else {
			/* No VOPs are enabled, so don't wait. */
			return;
		}

		if (stopwatch_expired()) {
			ret = 1;
			goto out;
		}
	}

out:
	stopwatch_reset();
	return ret;
}

static void idle_port(void)
{
	gatedis_con0 = mmio_read_32(PMUCRU_BASE + PMU_CRU_GATEDIS_CON0);
	mmio_write_32(PMUCRU_BASE + PMU_CRU_GATEDIS_CON0, 0x3fffffff);

	mmio_setbits_32(PMU_BASE + PMU_BUS_IDLE_REQ,
			(1 << PMU_IDLE_REQ_MSCH0) | (1 << PMU_IDLE_REQ_MSCH1));
	while ((mmio_read_32(PMU_BASE + PMU_BUS_IDLE_ST) &
		((1 << PMU_IDLE_ST_MSCH1) | (1 << PMU_IDLE_ST_MSCH0))) !=
		((1 << PMU_IDLE_ST_MSCH1) | (1 << PMU_IDLE_ST_MSCH0)))
		continue;
}

static void deidle_port(void)
{
	mmio_clrbits_32(PMU_BASE + PMU_BUS_IDLE_REQ,
			(1 << PMU_IDLE_REQ_MSCH0) | (1 << PMU_IDLE_REQ_MSCH1));
	while (mmio_read_32(PMU_BASE + PMU_BUS_IDLE_ST) &
	       ((1 << PMU_IDLE_ST_MSCH1) | (1 << PMU_IDLE_ST_MSCH0)))
		continue;

	/* document is wrong, PMU_CRU_GATEDIS_CON0 do not need set MASK BIT */
	mmio_write_32(PMUCRU_BASE + PMU_CRU_GATEDIS_CON0, gatedis_con0);
}

static void ddr_set_pll(void)
{
	mmio_write_32(CRU_BASE + CRU_DPLL_CON3, PLL_MODE(PLL_SLOW_MODE));

	mmio_write_32(CRU_BASE + CRU_DPLL_CON3, PLL_POWER_DOWN(1));
	mmio_write_32(CRU_BASE + CRU_DPLL_CON0,
		      mmio_read_32(PARAM_ADDR + PARAM_DPLL_CON0));
	mmio_write_32(CRU_BASE + CRU_DPLL_CON1,
		      mmio_read_32(PARAM_ADDR + PARAM_DPLL_CON1));
	mmio_write_32(CRU_BASE + CRU_DPLL_CON3, PLL_POWER_DOWN(0));

	while ((mmio_read_32(CRU_BASE + CRU_DPLL_CON2) & (1u << 31)) == 0)
		continue;

	mmio_write_32(CRU_BASE + CRU_DPLL_CON3, PLL_MODE(PLL_NORMAL_MODE));
}

__attribute__((noreturn)) void main(void)
{
	uint32_t freq_select;

	freq_select = mmio_read_32(PARAM_ADDR + PARAM_FREQ_SELECT);
	freq_select &= ~PARAM_FREQ_SELECT_FLAGS;

	if (freq_select & PARAM_FREQ_WAIT_VBLANK)
		wait_vop_dma_finish();

	mmio_setbits_32(PHY_REG(0, 927), (1 << 22));
	mmio_setbits_32(PHY_REG(1, 927), (1 << 22));
	idle_port();

	mmio_write_32(CIC_BASE + CIC_CTRL0,
		      (((0x3 << 4) | (1 << 2) | 1) << 16) |
		      (1 << 2) | 1 | freq_select);
	while ((mmio_read_32(CIC_BASE + CIC_STATUS0) & (1 << 2)) == 0)
		continue;

	ddr_set_pll();
	mmio_write_32(CIC_BASE + CIC_CTRL0, 0x20002);
	while ((mmio_read_32(CIC_BASE + CIC_STATUS0) & (1 << 0)) == 0)
		continue;

	deidle_port();
	mmio_clrbits_32(PHY_REG(0, 927), (1 << 22));
	mmio_clrbits_32(PHY_REG(1, 927), (1 << 22));

	mmio_write_32(PARAM_ADDR + PARAM_M0_DONE, M0_DONE_FLAG);

	for (;;)
		__asm__ volatile ("wfi");
}
