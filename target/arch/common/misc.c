/*
 * Copyright (C) 2012, Texas Instruments, Inc.
 * Texas Instruments, <www.ti.com>
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
#include <io.h>
#include <omap_rom.h>
#include <string.h>

#define LOOP_MAX      2000
u32 check_loop(u32 mask, u32 match, u32 addr)
{
	u32 i = 0, val;
	while(1) {
		++i;
		val = readl(addr) & mask;
		if (val == match)
			return (1);
		if (i == LOOP_MAX)
			return (0);
	}
	return(0);
}

void set_modify(u32 reg, u32 mask, u32 value)
{
	u32 read = readl(reg);
	u32 reg_value = ((read & ~(mask)) | value);

	writel(reg_value, reg);
}

void dev_to_devstr(u8 dev, char *devstr)
{
	switch (dev) {
	case DEVICE_USB:
		strcpy(devstr, "USB");
		break;
	case DEVICE_EMMC:
		strcpy(devstr, "EMMC");
		break;
	case DEVICE_SDCARD:
		strcpy(devstr, "SD");
		break;
	case DEVICE_SATA:
		strcpy(devstr, "SATA");
		break;
	default:
		strcpy(devstr, "Unknown");
		break;
	}
}

int devstr_to_dev(const char *devstr, u8 *dev)
{
	int ret = 0;

	if (!strcmp(devstr, "USB"))
		*dev = DEVICE_USB;
	else if (!strcmp(devstr, "EMMC"))
		*dev = DEVICE_EMMC;
	else if (!strcmp(devstr, "SD"))
		*dev = DEVICE_SDCARD;
	else if (!strcmp(devstr, "SATA"))
		*dev = DEVICE_SATA;
	else
		ret = -1;

	return ret;
}
