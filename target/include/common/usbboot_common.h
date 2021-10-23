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

#ifndef _USBBOOT_COMMON_H_
#define _USBBOOT_COMMON_H_

#include "../config_omap5uevm.h"

#include <types.h>
#include <version.h>
#include <rom_usb.h>

#define CEIL(a, b) (((a) / (b)) + ((a % b) > 0 ? 1 : 0))

#if defined DO_MEMORY_TEST_DURING_FIRST_STAGE_IN_EBOOT || \
defined DO_MEMORY_TEST_DURING_FIRST_STAGE_IN_IBOOT

#define USER_RQ_MEMTEST		0x00FF004F
#define USER_RQ_UMEMTEST	0x00FF002F
void memtest(void *x, unsigned count);
#endif

#define USER_RQ_FASTBOOT	0x00FF0048
#define USER_RQ_UFASTBOOT	0x00FF0028

struct partition {
	const char *name;
	u32 size_kb;
};

/* Use these functions to override the
 * default configuration for the processor */
struct proc_specific_functions {
	char* (*proc_get_rom_version)(void);
	u32 (*proc_get_api_base)(void);
	char* (*proc_get_serial_num)(void);
	char* (*proc_get_type)(void);
	char* (*proc_get_revision)(void);
	char* (*proc_get_version)(void);
	int (*proc_get_proc_id)(void);
	void (*proc_check_lpddr2_temp)(void);
};

struct storage_specific_functions {
	int (*init)(u8 device);
	int (*get_sector_size)(void);
	u64 (*get_total_sectors)(void);
	int (*read)(u64 start_sec, u64 sectors, void *data);
	int (*write)(u64 start_sec, u64 sectors, void *data);
	int (*erase)(u64 start_sec, u64 sectors);
};

struct usb_specific_functions {
	int (*usb_open)(struct usb *usb, int init,
				struct proc_specific_functions *proc_ops);
	void (*usb_init)(struct usb *usb);
	void (*usb_close)(struct usb *usb);
	int (*usb_queue_read)(struct usb *usb, void *data, unsigned len);
	int (*usb_wait_read)(struct usb *usb);
	int (*usb_queue_write)(struct usb *usb, void *data, unsigned len);
	int (*usb_wait_write)(struct usb *usb);
	int (*usb_read)(struct usb *usb, void *data, unsigned len);
	int (*usb_write)(struct usb *usb, void *data, unsigned len);
	struct usb* (*usb_enable)();
	void (*usb_configure)(struct usb *usb, unsigned mode);
	struct usb *usb;
};

/* Use these functions to override the
 * default configuration for the processor */
struct board_specific_functions {
	void (*board_scale_vcores)(struct proc_specific_functions *proc_ops);
	struct partition *(*board_get_part_tbl)(void);
	void (*board_prcm_init)(struct proc_specific_functions *proc_ops);
	void (*board_gpmc_init)(void);
	void (*board_late_init)(void);
	void (*board_mux_init)(void);
	void (*board_ddr_init)(struct proc_specific_functions *proc_ops);
	void (*board_signal_integrity_reg_init)
				(struct proc_specific_functions *proc_ops);
	int (*board_storage_init)(u8 dev,
				struct storage_specific_functions *storage_ops);
	int (*board_user_fastboot_request)(void);
	u8 (*board_get_flash_slot)(void);
	struct storage_specific_functions *(*board_set_flash_slot)(u8 dev,
				struct proc_specific_functions *proc_ops,
				struct storage_specific_functions *storage_ops);
	u32 (*board_get_board_rev)(void);
	int (*board_fastboot_size_request)(
					struct usb_specific_functions *usb_ops,
					void *data, unsigned len);
};

struct pmic_specific_functions {
	int (*pmic_enable)(void);
	int (*pmic_disable)(void);
	int (*pmic_configure_pwm_mode)(void);
	void (*pmic_read_reset_reason)(void);
	int (*pmic_read_sw_revision)(void);
	char* (*pmic_get_silicon_revision)(void);
};

struct bootloader_ops {
	struct board_specific_functions *board_ops;
	struct proc_specific_functions *proc_ops;
	struct storage_specific_functions *storage_ops;
	struct pmic_specific_functions *pmic_ops;
	struct usb_specific_functions *usb_ops;
};

void* init_board_funcs(void);
void* init_processor_id_funcs(void);
void *init_pmic_funcs(void);
void *init_usb_funcs(void);

unsigned long crc32(unsigned long crc, const unsigned char *buf,
						unsigned int len);

int get_downloadsize_from_string(int count, char *string);
struct bootloader_ops *boot_common(unsigned bootdevice);

/* Storage drivers function inits */
struct storage_specific_functions *init_rom_mmc_funcs(int proc_id, u8 device);
struct storage_specific_functions *init_rom_sata_funcs(int proc_id, u8 device);

#endif
