/*
 * Copyright(c) 2012 Texas Instruments.
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

#include <types.h>
#include <io.h>
#include <hw.h>
#include <omap_rom.h>
#include <common_proc.h>
#include <string.h>
#include <usbboot_common.h>

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

int pmic_reg_access(hal_i2c i2c_id, u16 slave, u16 reg, u32 value, int read)
{
	int ret; u32 clk32; u16 cmd;

	ret = i2c_init(i2c_id);
	if (ret != 0) {
		printf("Failed to init I2C-%d, ret = %d\n", i2c_id, ret);
		goto fail;
	} else
		DBG("Initialized I2C-%d\n", i2c_id);

	clk32 = readl(CLK32K_COUNTER_REGISTER);

	if (read) {
		cmd = (reg & 0xFF);
		ret = i2c_read(i2c_id, slave, 1, &cmd, clk32, 0xFF);
		if (ret != 0) {
			printf("I2C read failed, ret = %d\n", ret);
			goto fail;
		}
	} else {
		cmd = (reg & 0xFF) | ((value & 0xFF) << 8);
		ret = i2c_write(i2c_id, slave, 2, &cmd, clk32, 0xFF);
		if (ret != 0) {
			printf("I2C write failed, ret = %d\n", ret);
			goto fail;
		}
	}

	ret = i2c_close(i2c_id);
	if (ret != 0) {
		printf("i2c close for module %d failed, ret = %d\n",
							i2c_id, ret);
		goto fail;
	} else
		DBG("I2C-%d has been disabled\n", i2c_id);

	return cmd;
fail:
	return 0;
}


static int palmas_read_silicon_revision(u32 *revision)
{
	int ret;

	/* read INTERNAL_DESIGNREV to get silicon revision */
	ret = pmic_reg_access(HAL_I2C1, 0x4A, 0x57, 0, 1);
	if (ret != 0) {
		*revision = ret;
		return 0;
	}

	return ret;
}

char *palmas_get_silicon_revision(void)
{
	u32 revision = 0;
	static char rev_id[8];
	enum {
		PALMAS_ES1_DOT_0,
		PALMAS_ES2_DOT_0,
		PALMAS_ES2_DOT_1
	};

	if (palmas_read_silicon_revision(&revision)) {
		printf("Could not get palmas silicon revision");
		return NULL;
	}

	switch (revision) {
	case PALMAS_ES1_DOT_0:
		strcpy(rev_id, "ES1.0");
		break;
	case PALMAS_ES2_DOT_0:
		strcpy(rev_id, "ES2.0");
		break;
	case PALMAS_ES2_DOT_1:
		strcpy(rev_id, "ES2.1");
		break;
	default:
		printf("unknown PALMAS revision\n");
		strcpy(rev_id, "unknown");
		break;
	}

	return rev_id;
}

static void palmas_read_reset_reason(void)
{
	int ret;

	printf("OMAP reset reason PRM_RSTST = 0x%04x\n", readl(PRM_RSTST));

	/* SWOFF_STATUS: qualify which switch off events generate a HW RESET */
	ret = pmic_reg_access(HAL_I2C1, 0x48, 0xB1, 0, 1);
	printf("PMIC reset reason SWOFF_STATUS = 0x%02x\n", ret);
}

static int palmas_read_sw_revision(void)
{
	int ret;

	ret = pmic_reg_access(HAL_I2C1, 0x48, 0xB7, 0, 1);
	if (ret != 0)
		return ret;

	return 0;
}

static int palmas_configure_pwm_mode(void)
{
	int ret;

	ret = pmic_reg_access(HAL_I2C1, 0x48, 0x30, 0x0F, 0);
	if (ret != 0) {
#ifdef DEBUG
		ret = pmic_reg_access(HAL_I2C1, 0x48, 0x30, 0x0F, 1);
		if (ret != 0)
			printf("SMPS7_CTRL = 0x%x\n", ret);
		else
			return ret;
#endif
		return 0;
	}

	return ret;
}

struct pmic_specific_functions pmic_funcs = {
	.pmic_configure_pwm_mode = palmas_configure_pwm_mode,
	.pmic_read_reset_reason = palmas_read_reset_reason,
	.pmic_read_sw_revision = palmas_read_sw_revision,
	.pmic_get_silicon_revision = palmas_get_silicon_revision,
};

void *init_pmic_funcs(void)
{
	return &pmic_funcs;
}
