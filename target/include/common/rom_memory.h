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

#ifndef _ROM_MEMORY_H_
#define _ROM_MEMORY_H_

/* Memory (storage) device list */
#define DEVICE_TYPE_NULL	0x00
#define DEVICE_TYPE_XIP		0x01
#define DEVICE_TYPE_XIP_WAIT	0x02
#define DEVICE_TYPE_NAND	0x03
#define DEVICE_TYPE_ONENAND	0x04
#define DEVICE_SDCARD		0x05
#if defined CONFIG_IS_OMAP4
#define DEVICE_EMMC		0x06
#define DEVICE_EMMC_MUX5	0x07
#define DEVICE_SATA		DEVICE_TYPE_NULL
#elif defined CONFIG_IS_OMAP5
#define DEVICE_EMMC_BOOT	0x06
#define DEVICE_EMMC		0x07
#define DEVICE_SATA		0x09
#endif

/* Memory read descriptor */
struct read_desc {
	u32 sector_start;
	u32 sector_count;
	void *destination;
};

struct write_desc {
	u32 sector_start;
	u32 sector_count;
	void *source;
};

/* Memory device handle */
struct mem_device {
	u32 initialized;
	u8 device_type;
	u8 trials_count;
	u32 xip_device;
	u16 search_size;
	u32 base_address;
	u16 hs_toc_mask;
	u16 gp_toc_mask;
	void *device_data;
	u16 *boot_options;
};

/* Memory device driver */
struct mem_driver {
	int (*init)(struct mem_device *md);
	int (*read)(struct mem_device *md, struct read_desc *rd);
	int (*configure)(struct mem_device *md, void *config);
	int (*write)(struct mem_device *md, struct write_desc *wr);
};

#endif
