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

#ifndef _ROM_MMC_H_
#define _ROM_MMC_H_

#include <rom_memory.h>

#define MMCSD_TYPE_MMC			(1)
#define MMCSD_TYPE_SD			(2)
#define MMCSD_MODE_RAW			(1)
#define MMCSD_MODE_FAT			(2)
#define MMCSD_ADDRESSING_BYTE		(1)
#define MMCSD_ADDRESSING_SECTOR		(2)

struct mmc_config {
	u32 configid;
	u32 value;
};

struct mmc {
	struct mem_device dread;
	struct mem_driver *io;
};

/* This is an approximation since all the fields are not defined here */
#define SIZEOF_MMC_DEVICE_DATA		2500
struct mmc_devicedata {
	u32 moduleid;
	u32 type;      /* memory type (MMC/SD)  */
	u32 mode;      /* opmode (BOOT/RAW/FAT) */
	u32 copy;
	u32 version;
	u32 addressing; /* sector or byte addressed */
	u32 buswidth;
	u32 size;
}; /* did not include the remaining fields */

#endif
