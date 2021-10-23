/*
 * Copyright (c) 2010, The Android Open Source Project.
 *
  * Copyright (C) 2012, Texas Instruments, Inc.
 * Texas Instruments, <www.ti.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Neither the name of The Android Open Source Project nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
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
 *
 */

#ifndef _FASTBOOT_H_
#define _FASTBOOT_H_

#include <usbboot_common.h>
#include <sparse_format.h>
#include <fastboot_common.h>
#include <omap_rom.h>

/* EFI defines */
#define EFI_VERSION 0x00010000
#define EFI_ENTRIES 128
#define EFI_NAMELEN 36

static const u8 partition_type[16] = {
	0xa2, 0xa0, 0xd0, 0xeb, 0xe5, 0xb9, 0x33, 0x44,
	0x87, 0xc0, 0x68, 0xb6, 0xb7, 0x26, 0x99, 0xc7,
};

static const u8 random_uuid[16] = {
	0xff, 0x1f, 0xf2, 0xf9, 0xd4, 0xa8, 0x0e, 0x5f,
	0x97, 0x46, 0x59, 0x48, 0x69, 0xae, 0xc3, 0x4e,
};

struct efi_header {
	u8 magic[8];

	u32 version;
	u32 header_sz;

	u32 crc32;
	u32 reserved;

	u64 header_lba;
	u64 backup_lba;
	u64 first_lba;
	u64 last_lba;

	u8 volume_uuid[16];

	u64 entries_lba;

	u32 entries_count;
	u32 entries_size;
	u32 entries_crc32;
} __packed;

struct ptable {
	u8 mbr[512];
	union {
		struct efi_header header;
		u8 block[512];
	};
	struct efi_entry entry[EFI_ENTRIES];
};

struct fastboot_data {
	struct board_specific_functions *board_ops;
	struct proc_specific_functions *proc_ops;
	struct storage_specific_functions *storage_ops;
	struct usb_specific_functions *usb_ops;
	struct pmic_specific_functions *pmic_ops;
	struct fastboot_ptentry *e;
	struct fastboot_ptentry fb_ptable[MAX_PTN];
	u32 getsize;
	u32 sector;
	sparse_header_t *sparse_header;
};

#if defined CONFIG_FASTBOOT

void do_fastboot(struct bootloader_ops *boot_ops);
char *get_serial_number(void);
void fastboot_flash_reset_ptn(void);
fastboot_ptentry *fastboot_flash_find_ptn(const char *name);
char *get_ptn_size(struct fastboot_data *fb_data, char *buf, const char *ptn) ;

#else

static inline void do_fastboot(struct bootloader_ops *boot_ops)
						 { return; };
static inline char *get_serial_number(void) { return 0; };
static inline void fastboot_flash_reset_ptn(void) { return; };
static inline fastboot_ptentry *fastboot_flash_find_ptn(const char *name) { return NULL; };
static inline char *get_ptn_size(struct fastboot_data *fb_data, char *buf,
					const char *ptn) { return 0; };

#endif /* CONFIG_FASTBOOT */

int do_gpt_format(struct fastboot_data *fb_data);

#endif /* FASTBOOT_H */
