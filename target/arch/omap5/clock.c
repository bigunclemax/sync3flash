/*
* Copyright (C) 2012 Texas Instruments, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
* SUCH DAMAGE.
*/

#include <aboot.h>
#include <common_proc.h>
#include <io.h>
#include <hw.h>
#include <clock.h>
typedef struct dpll_param dpll_param;


/* OPP NOM */
static struct dpll_param core_dpll_params[3] = {
	{277, 4, 2, 5, 8, 4, 62,  4, -1,  5,  7, -1}, /* 19.2MHz ES1.0 */
	{277, 4, 2, 5, 8, 4, 62, 5,  6,  5,  7,  6}, /* 19.2MHz ES2.0 */
	{277, 9, 2, 5, 8, 8, 62, 10, -1, 10, 14, -1}  /* 38.4MHz ES1.0 */
};
#define CORE_VOLTAGE	1040000

/* OPP NOM */
static struct dpll_param usb_dpll_params[3] = {
	{400, 7,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1}, /* 19.2MHz ES1.0 */
	{400, 7,  2, -1, -1, -1, -1, -1, -1, -1, -1, -1}, /* 19.2MHz ES2.0 */
	{400, 15, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1}  /* 38.4MHz ES1.0*/
};

#ifndef CONFIG_FORCE_IVA_OPPNOM
/* OPP_LOW */
static struct dpll_param iva_dpll_params[3] = {
	{182, 2, -1, -1, 10, 12, -1, -1, -1, -1, -1, -1}, /* 19.2MHz ES1.0 */
	{182, 2, -1, -1, 5, 6, -1, -1, -1, -1, -1, -1},	  /* 19.2 MHz */
	{91,  1, -1, -1, 10, 12, -1, -1, -1, -1, -1, -1}  /* 38.4MHz ES1.0 */
};
#define IVA_VOLTAGE	880000
#else
/* OPP_NOM: in the customer specific need of having to go to OPP NOM */
static struct dpll_param iva_dpll_params[3] = {
	{182, 2, -1, -1,  5,  6,  -1, -1, -1, -1, -1, -1}, /* 19.2MHz ES1.0 */
	{182, 2, -1, -1,  5,  6,  -1, -1, -1, -1, -1, -1}, /* 19.2MHz ES2.0 */
	{91,  2, -1, -1,  5,  6,  -1, -1, -1, -1, -1, -1}  /* 38.4MHz ES1.0 */
};
#define IVA_VOLTAGE	950000
#endif

/* OPP NOM */
static struct dpll_param abe_dpll_params[2] = {
	{750, 0, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1}, /* 19.2MHz ES1.0 */
#ifdef ABE_SYSCLK /* Selection of the ABE_DPLL reference clock */
        {46, 8, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1},  /* 19.2 MHz ES2.0 */
#else
	{750, 0, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1}  /* 32KHz ES2.0 */
#endif
};

/* OPP NOM */
static struct dpll_param mpu_dpll_params[3] = {
	{375,  8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1}, /* 19.2MHz ES1.0 */
	{401,  6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1}, /* 19.2 MHz ES2.0 */
	{375, 17, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1}  /* 38.4MHz ES1.0 */
};
#define MPU_VOLTAGE	1060000

/* OPP NOM */
static struct dpll_param per_dpll_params[3] = {
	{20, 0, 4, 3, 6, 4, -1, 2, -1, -1, -1, -1},	/* 19.2 MHz ES1.0 */
	{20, 0, 4, 4, 3, 4, -1, 4, -1, -1, -1, -1},	/* 19.2 MHz */
	{10, 0, 4, 3, 6, 4, -1, 4, -1, -1, -1, -1}	/* 38.4 MHz ES1.0 */
};

static void setup_clocks(struct proc_specific_functions *proc_ops)
{
	u32 reg_addr_base;
	u32 l3init_cm_core_addr_base;
	if (proc_ops->proc_get_proc_id) {
		if (proc_ops->proc_get_proc_id() < OMAP_5430_ES2_DOT_0) {
			reg_addr_base = L4PER_CM_CORE_BASE;
			l3init_cm_core_addr_base = L3INIT_CM_CORE_BASE_ES1_0;
		} else {
			reg_addr_base = CORE_CM_CORE_L4PER_BASE;
			l3init_cm_core_addr_base = L3INIT_CM_CORE_BASE;
		}

		set_modify(reg_addr_base + CM_L4PER_CLKSTCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_SW_WKUP);
		set_modify(reg_addr_base + CM_L4PER_GPTIMER2_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_SW_WKUP);
		set_modify(reg_addr_base + CM_L4PER_GPIO2_CLKCTRL_OFFSET,
				GPIO_CTRL_FIELD_MASK, 0);
		set_modify(reg_addr_base + CM_L4PER_GPIO3_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_NO_SLEEP);
		set_modify(reg_addr_base + CM_L4PER_GPIO4_CLKCTRL_OFFSET,
				GPIO_CTRL_FIELD_MASK, 0);
		set_modify(reg_addr_base + CM_L4PER_GPIO4_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_NO_SLEEP);
		set_modify(reg_addr_base + CM_L4PER_GPIO5_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_NO_SLEEP);
		set_modify(reg_addr_base + CM_L4PER_GPIO6_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_NO_SLEEP);
		set_modify(reg_addr_base + CM_L4PER_I2C1_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_SW_WKUP);
		set_modify(reg_addr_base + CM_L4PER_UART1_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_SW_WKUP);
		set_modify(reg_addr_base + CM_L4PER_UART2_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_SW_WKUP);
		set_modify(reg_addr_base + CM_L4PER_UART3_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_SW_WKUP);
		set_modify(reg_addr_base + CM_L4PER_UART4_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_SW_WKUP);
		set_modify(reg_addr_base + CM_L4PER_CLKSTCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_HW_AUTO);
	} else
		return;


	set_modify(CORE_CM_CORE_BASE + CM_EMIF_CLKSTCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_SW_WKUP);
	set_modify(CORE_CM_CORE_BASE + CM_EMIF_EMIF1_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_NO_SLEEP);
	set_modify(CORE_CM_CORE_BASE + CM_EMIF_EMIF2_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_NO_SLEEP);
	set_modify(CORE_CM_CORE_BASE + CM_L4CFG_CLKSTCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_SW_WKUP);
	set_modify(CORE_CM_CORE_BASE + CM_L4CFG_L4_CFG_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_NO_SLEEP);
	set_modify(CORE_CM_CORE_BASE + CM_L4CFG_CLKSTCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_HW_AUTO);
	set_modify(l3init_cm_core_addr_base + CM_L3INIT_MMC1_CLKCTRL_OFFSET,
				L3INIT_CTRL_FIELD_MASK, MMC_CLK_192MHZ_DPLL);
	set_modify(l3init_cm_core_addr_base + CM_L3INIT_MMC2_CLKCTRL_OFFSET,
				L3INIT_CTRL_FIELD_MASK, MMC_CLK_192MHZ_DPLL);
	set_modify(l3init_cm_core_addr_base + CM_L3INIT_CLKSTCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_SW_WKUP);
	set_modify(l3init_cm_core_addr_base + CM_L3INIT_MMC1_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_SW_WKUP);
	set_modify(l3init_cm_core_addr_base + CM_L3INIT_MMC2_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_SW_WKUP);
	set_modify(l3init_cm_core_addr_base + CM_L3INIT_CLKSTCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_HW_AUTO);
	set_modify(WKUPAON_CM_BASE + CM_WKUPAON_TIMER1_CLKCTRL_OFFSET,
				L3INIT_CTRL_FIELD_MASK, 0);
	set_modify(WKUPAON_CM_BASE + CM_WKUPAON_GPIO1_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_NO_SLEEP);
	set_modify(WKUPAON_CM_BASE + CM_WKUPAON_TIMER1_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_SW_WKUP);
	set_modify(WKUPAON_CM_BASE + CM_WKUPAON_WD_TIMER2_CLKCTRL_OFFSET,
				CLKTRCTRL_FIELD_MASK, CLKTRCTRL_SW_WKUP);
}

static void configure_core_dpll(dpll_param *dpll_param_p)
{
#ifdef CONFIG_USE_CH_SETTINGS_CONFIG
	/* Use the Config header to configure this dpll */
	return;
#endif
	/* Unlock the CORE dpll */
	set_modify(CM_CLKMODE_DPLL_CORE, 0x0000000f, IDLE_BYPASS_FAST_RELOCK_MODE);
	if (!check_loop(BIT(0), 0, CM_IDLEST_DPLL_CORE)) {
		/* do nothing */
	}

	set_modify(CM_CLKSEL_DPLL_CORE, 0x0007ff00, dpll_param_p->m << 8);
	set_modify(CM_CLKSEL_DPLL_CORE, 0x0000007f, dpll_param_p->n);


	/* Setup post-dividers */
	if (dpll_param_p->m2 >= 0)
		writel(dpll_param_p->m2, CM_DIV_M2_DPLL_CORE);
	if (dpll_param_p->m3 >= 0)
		writel(dpll_param_p->m3, CM_DIV_M3_DPLL_CORE);
	if (dpll_param_p->h11 >= 0)
		writel(dpll_param_p->h11, CM_DIV_H11_DPLL_CORE);
	if (dpll_param_p->h12 >= 0)
		writel(dpll_param_p->h12, CM_DIV_H12_DPLL_CORE);
	if (dpll_param_p->h13 >= 0)
		writel(dpll_param_p->h13, CM_DIV_H13_DPLL_CORE);
	if (dpll_param_p->h14 >= 0)
		writel(dpll_param_p->h14, CM_DIV_H14_DPLL_CORE);
	if (dpll_param_p->h21 >= 0)
		writel(dpll_param_p->h21, CM_DIV_H21_DPLL_CORE);
	if (dpll_param_p->h22 >= 0)
		writel(dpll_param_p->h22, CM_DIV_H22_DPLL_CORE);
	if (dpll_param_p->h23 >= 0)
		writel(dpll_param_p->h23, CM_DIV_H23_DPLL_CORE);
	if (dpll_param_p->h24 >= 0)
		writel(dpll_param_p->h24, CM_DIV_H24_DPLL_CORE);

	return;
}

static void configure_per_dpll(dpll_param *dpll_param_p)
{
#ifdef CONFIG_USE_CH_SETTINGS_CONFIG
	/* Use the Config header to configure this dpll */
	return;
#endif
	/* Put DPLL into bypass mode */
	set_modify(CM_CLKMODE_DPLL_PER, 0x00000007, 0x00000005);
	if (!check_loop(BIT(0), 0, CM_IDLEST_DPLL_PER)) {
		/* do nothing */
	}

	/* Setup post-dividers */
	if (dpll_param_p->m2 >= 0)
		writel(dpll_param_p->m2, CM_DIV_M2_DPLL_PER);
	if (dpll_param_p->m3 >= 0)
		writel(dpll_param_p->m3, CM_DIV_M3_DPLL_PER);
	if (dpll_param_p->h11 >= 0)
		writel(dpll_param_p->h11, CM_DIV_H11_DPLL_PER);
	if (dpll_param_p->h12 >= 0)
		writel(dpll_param_p->h12, CM_DIV_H12_DPLL_PER);
	if (dpll_param_p->h13 >= 0)
		writel(dpll_param_p->h14, CM_DIV_H14_DPLL_PER);

	writel(0x00000002, CORE_CM_CORE_L4PER_BASE);

	/* Program DPLL frequency (M and N) */
	set_modify(CM_CLKSEL_DPLL_PER, 0x0007ff00, dpll_param_p->m << 8);
	set_modify(CM_CLKSEL_DPLL_PER, 0x0000007f, dpll_param_p->n);

	/* Put DPLL into lock mode */
	writel(0x00000007, CM_CLKMODE_DPLL_PER);

	/* Wait for DPLL to be locked */
	if (!check_loop(BIT(0), 1, CM_IDLEST_DPLL_PER)) {
		/* do nothing */
	}
       writel(0x00000001, CM_EMIF_EMIF1_CLKCTRL);
       writel(0x00000001, CM_EMIF_EMIF2_CLKCTRL);
       writel(0x2, (CORE_CM_CORE_L4PER_BASE + CM_L4PER_UART1_CLKCTRL_OFFSET));
       writel(0x2, (CORE_CM_CORE_L4PER_BASE + CM_L4PER_UART2_CLKCTRL_OFFSET));
       writel(0x2, (CORE_CM_CORE_L4PER_BASE + CM_L4PER_UART3_CLKCTRL_OFFSET));
       writel(0x2, (CORE_CM_CORE_L4PER_BASE + CM_L4PER_UART4_CLKCTRL_OFFSET));
	return;
}

static void configure_mpu_dpll(dpll_param *dpll_param_p)
{
#ifdef CONFIG_USE_CH_SETTINGS_CONFIG
	/* Use the Config header to configure this dpll */
	return;
#endif
	/* Put DPLL into bypass mode */
	set_modify(CM_CLKMODE_DPLL_MPU, 0x00000007, 0x00000005);

	if (!check_loop(BIT(0), 0, CM_IDLEST_DPLL_MPU)) {
		/* do nothing */
	}

	/* Program DPLL frequency (M and N) */
	set_modify(CM_CLKSEL_DPLL_MPU, 0x0007ff00, dpll_param_p->m << 8);
	set_modify(CM_CLKSEL_DPLL_MPU, 0x0000007f, dpll_param_p->n);

	/* Program DPLL_CLKOUT divider (M2 = 1) */
	if (dpll_param_p->m2 >= 0)
		writel(dpll_param_p->m2, CM_DIV_M2_DPLL_MPU);

	/* Put DPLL into lock mode */
	writel(0x00000007, CM_CLKMODE_DPLL_MPU);

	/* Wait for DPLL to be locked */
	if (!check_loop(BIT(0), 1, CM_IDLEST_DPLL_MPU)) {
		/* do nothing */
	}

	return;
}
/* The IVA DPLL is not part of the CH Header so we have to perform this
API */
static void configure_iva_dpll(dpll_param *dpll_param_p)
{
	/* Unlock the IVA dpll */
	set_modify(CM_CLKMODE_DPLL_IVA, 0x00000007, IDLE_BYPASS_FAST_RELOCK_MODE);
	if (!check_loop(BIT(0), 0, CM_IDLEST_DPLL_IVA)) {
		/* do nothing */
	}

	/* CM_BYPCLK_DPLL_IVA = CORE_X2_CLK/2 */
	set_modify(CM_BYPCLK_DPLL_IVA, 0x00000003, 0x1);
	/* Disable DPLL autoidle */
	set_modify(CM_AUTOIDLE_DPLL_IVA, 0x00000007, 0x0);
	set_modify(CM_CLKSEL_DPLL_IVA, 0x0007ff00, dpll_param_p->m << 8);
	set_modify(CM_CLKSEL_DPLL_IVA, 0x0000007f, dpll_param_p->n);
	set_modify(CM_DIV_H11_DPLL_IVA, 0x0000003f, dpll_param_p->h11);
	set_modify(CM_DIV_H11_DPLL_IVA, 0x00000200, 0x1 << 9);
	set_modify(CM_DIV_H12_DPLL_IVA, 0x0000003f, dpll_param_p->h12);
	set_modify(CM_DIV_H12_DPLL_IVA, 0x00000200, 0x1 << 9);

	/* Lock the iva dpll */
	set_modify(CM_CLKMODE_DPLL_IVA, 0x00000007, PLL_LOCK);
	if (!check_loop(BIT(0), 1, CM_IDLEST_DPLL_IVA)) {
		/* do nothing */
	}

	return;
}

/* The ABE DPLL is not part of the CH Header so we have to perform this
API */
static void configure_abe_dpll(dpll_param *dpll_param_p)
{
	set_modify(CM_CLKMODE_DPLL_ABE, 0x00000007,
				IDLE_BYPASS_FAST_RELOCK_MODE);
	check_loop(BIT(1), 0, CM_IDLEST_DPLL_ABE);

#ifdef ABE_SYSCLK
	set_modify(CM_CLKSEL_ABE_PLL_REF, 0x1, 0);
	set_modify(CM_CLKSEL_WKUPAON, 0x1, 0);
#else
	set_modify(CM_CLKSEL_ABE_PLL_REF, 0x1, 1);
	set_modify(CM_CLKSEL_WKUPAON, 0x1, 1);
#endif
	/* Disable DPLL autoidle */
	set_modify(CM_AUTOIDLE_DPLL_ABE, 0x7, 0x0);

	/* Set SW_WKUP explicitly */
	set_modify(CM_ABE_CLKSTCTRL, CLKTRCTRL_FIELD_MASK, CLKTRCTRL_SW_WKUP);

	/* Wait for the powerdomain to be up */
	check_loop(0x3, 0x3, PM_ABE_PWRSTST);

#ifndef ABE_SYSCLK
	/*
	* Enable higher frequencies when fed from 32KHz clk.
	*
	* BIT(5) - DPLL_RAMP_RATE == 4 REFCLKs
	* BIT(8) - DPLL_DRIFTGUARD_EN is enabled
	* BIT(9) - DPLL_RELOCK_RAMP_EN is enabled
	* BIT(10) - DPLL_LPMODE_EN is enabled
	* BIT(11) - REGM4XEN is enabled
	*
	* The DPLL_REGM4XEN bit provides a magic 4x multplier to existing MN
	* dividers.  This is how a DPLL driven from 32KHz clock can achieve
	* 196.608MHz.
	*/
	set_modify(CM_CLKMODE_DPLL_ABE, 0x1fff,
		(BIT(5) | BIT(8) | BIT(9) | BIT(10) | BIT(11)));
#endif

	set_modify(CM_CLKSEL_DPLL_ABE, 0x0007ff00, dpll_param_p->m << 8);
	set_modify(CM_CLKSEL_DPLL_ABE, 0x0000007f, dpll_param_p->n);
	set_modify(CM_DIV_M2_DPLL_ABE, 0x0000001f, dpll_param_p->m2);
	set_modify(CM_DIV_M3_DPLL_ABE, 0x0000001f, dpll_param_p->m3);

	set_modify(CM_CLKMODE_DPLL_ABE, 0x00000007, PLL_LOCK);

	check_loop(BIT(0), 1, CM_IDLEST_DPLL_ABE);

	return;
}

static void configure_usb_dpll(dpll_param *dpll_param_p)
{
	u32 num, den, sd_div;

	num = dpll_param_p->m * (19200000/1000);
	den = (dpll_param_p->n + 1) * 250 * 1000;
	num += den - 1;
	sd_div = num / den;

	set_modify(CM_CLKSEL_DPLL_USB, 0xff000000, sd_div << 24);

	/* Unlock the USB dpll */
	set_modify(CM_CLKMODE_DPLL_USB, 0x00000007, IDLE_BYPASS_LOW_POWER_MODE);
	if (!check_loop(BIT(0), 0, CM_IDLEST_DPLL_USB)) {
		/* do nothing */
	}

	set_modify(CM_CLKSEL_DPLL_USB, 0x000fff00, dpll_param_p->m << 8);
	set_modify(CM_CLKSEL_DPLL_USB, 0x0000007f, dpll_param_p->n);

	/* lock the dpll */
	writel(0x00000007, CM_CLKMODE_DPLL_USB);
	if (!check_loop(BIT(0), 1, CM_IDLEST_DPLL_USB)) {
		/* do nothing */
	}

	/* Setup post-dividers */
	if (dpll_param_p->m2 >= 0)
		writel(dpll_param_p->m2, CM_DIV_M2_DPLL_USB);

	return;
}

static u8 get_twl6035_voltage(u32 uv)
{
	/* format the desired voltage for palmas */
	return (u8)((uv - 500000) / 10000) + 6;
}

static u32 get_twl6035_slewdelay(u32 opp_uv)
{
	/*
	 * compute the delay for voltage to stabilize
	 * depending upon the voltage transition
	 * Current levels with TWL6035 palmas for ES1.0:
	 * opp_boot: 1050000 uv
	 * slew rate = 5 mv per ms
	 * Revisit when ES2.0 is out
	 */
	u32 slew_rate = 5;
	u32 opp_boot_uv = 1050000;
	u32 delta_uv;
	u32 sdelay;

	if (opp_boot_uv > opp_uv)
		delta_uv = opp_boot_uv - opp_uv;
	else
		delta_uv = opp_uv - opp_boot_uv;

	sdelay = (delta_uv / slew_rate) / 1000;

	return sdelay;
}

static void set_vcore(u8 regaddr, u8 slaveaddr, u32 voltage)
{
	u8 valid = 0x1;
	u8  data;
	u32 cmd;

	/* Compute and send i2c command */
	data = get_twl6035_voltage(voltage);

	cmd = (valid << 24) | (data << 16) | (regaddr << 8) | slaveaddr;
	writel(cmd, PRM_VC_VAL_BYPASS);

	/* check if voltage Controller did not send out command */
	if (!check_loop(0x01000000, 0, PRM_VC_VAL_BYPASS)) {
		/* do nothing, serial not setup yet */
	}

	/* make sure voltage stabilized */
	ldelay(TIME_LOOP_RATIO * get_twl6035_slewdelay(voltage));

	/* clean up irq status */
	set_modify(PRM_IRQSTATUS_MPU, 0x00000000, 0x00000000);
}

void scale_vcores(struct proc_specific_functions *proc_ops)
{
	u32 regaddr;
	u32 slaveaddr;

	/* Configure SR I2C Mode - FS mode */
	writel(0x00000000, PRM_VC_CFG_I2C_MODE);

	/* Configure SR I2C Clock */
	writel(0x0000150E, PRM_VC_CFG_I2C_CLK);

	/* Configure Palmas TWL6035 */
	slaveaddr = 0x12;

	/* VDD_CORE - SMPS8_VOLTAGE */
	regaddr = 0x37;
	set_vcore(regaddr, slaveaddr, CORE_VOLTAGE);

	/* VDD_MM:  SMPS45_VOLTAGE */
	regaddr = 0x2B;
	set_vcore(regaddr, slaveaddr, IVA_VOLTAGE);

	/* VDD_MPU:  SMPS12_VOLTAGE */
	regaddr = 0x23;
	set_vcore(regaddr, slaveaddr, MPU_VOLTAGE);

	return;
}

void prcm_init(struct proc_specific_functions *proc_ops)
{
	u32 temp;
	u8 n = 1;

	if (proc_ops->proc_get_proc_id) {
		if (proc_ops->proc_get_proc_id() < OMAP_5430_ES2_DOT_0)
			n = 0;
	}

	/* Configure CORE DPLL but don't lock it */
	configure_core_dpll(&core_dpll_params[n]);

	temp = (CLKSEL_CORE_X2_DIV_1 << CLKSEL_CORE_SHIFT) |
	(CLKSEL_L3_CORE_DIV_2 << CLKSEL_L3_SHIFT) |
	(CLKSEL_L4_L3_DIV_2 << CLKSEL_L4_SHIFT);
	writel(temp, CM_CLKSEL_CORE);

	/* Configure PER DPLL and LOCK it */
	configure_per_dpll(&per_dpll_params[n]);

	/* Configure MPU DPLL and LOCK it */
	configure_mpu_dpll(&mpu_dpll_params[n]);

	/* Configure IVA DPLL and LOCK it */
	configure_iva_dpll(&iva_dpll_params[n]);

	/* Configure ABE DPLL and LOCK it */
	configure_abe_dpll(&abe_dpll_params[n]);

#ifndef CONFIG_USE_CH_RAM_CONFIG
	/* Configure EMIF controller */
	setup_emif_config();
	writel(0x00000002, CM_EMIF_CLKSTCTRL);

#endif

#ifndef CONFIG_USE_CH_SETTINGS_CONFIG
	/* LOCK the CORE DPLL */
	writel(0x00001709, CM_SHADOW_FREQ_CONFIG1);
	if (!check_loop(BIT(0), 1, CM_SHADOW_FREQ_CONFIG1)) {
		/* do nothing */
	}
#endif

	/* Configure USB DPLL and LOCK it */
	configure_usb_dpll(&usb_dpll_params[n]);

	/* Configure CLOCKS */
	setup_clocks(proc_ops);

}
