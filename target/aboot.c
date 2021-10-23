/*
 * Copyright (C) 2010 The Android Open Source Project
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
#include <io.h>

#include <mux.h>
#include <hw.h>

#include <omap_rom.h>
#include <usbboot_common.h>

static unsigned MSG = 0xaabbccdd;


const u64 BOOT_INFO     = 0x2;
const u64 BOOT_MLO      = 0x100;
const u64 BOOT_BANK1    = 0x184;
const u64 BOOT_BANK2    = 0x7cd2;

static void print_sector()
{
    int i, k = 16;
    for (i = 0; i < (0x200/k); ++i ) {
        int j;
        for(j = 0; j < k; ++j)
            printf("%02x ", ((char *) CONFIG_ADDR_DOWNLOAD)[i*k+j]);
        printf("\n");
    }
}

static void dump_sectors(struct storage_specific_functions *storage_ops, u64 start_sec, u64 count)
{
    u64 idx;
    for(idx = start_sec; idx < (start_sec + count); ++idx) {
        int ret = storage_ops->read(idx, 1, (void *) CONFIG_ADDR_DOWNLOAD);
        print_sector();
    }
}

static void flash_image(struct storage_specific_functions *storage_ops, u64 start_sec, u64 img_sz)
{
    u64 img_blks = (img_sz / 512) + ((img_sz % 512) != 0);
    int ret = storage_ops->erase(start_sec, img_blks);
    printf("ERASE done\n");
    ret = storage_ops->write(start_sec, img_blks, (void *) CONFIG_ADDR_DOWNLOAD);
    printf("WRITE done\n");
}

static int load_from_usb(struct usb_specific_functions *usb_ops, unsigned *_len)
{
	unsigned len, n;
	enable_irqs();
	usb_ops->usb_queue_read(usb_ops->usb, &len, 4);
	usb_ops->usb_write(usb_ops->usb, &MSG, 4);
	n = usb_ops->usb_wait_read(usb_ops->usb);
	if (n)
		return -1;

	if (usb_ops->usb_read(usb_ops->usb, (void *)CONFIG_ADDR_DOWNLOAD, len))
		return -1;

//	usb_ops->usb_close(usb_ops->usb);

	disable_irqs();
	*_len = len;
	return 0;
}

void aboot(unsigned *info)
{
	unsigned n, len;
	unsigned bootdevice = -1;
	struct bootloader_ops *boot_ops;

	if (info)
		bootdevice = info[2] & 0xFF;
	else
		goto fail;

	boot_ops = boot_common(bootdevice);
	if (!boot_ops)
		goto fail;

    boot_ops->usb_ops->usb_write(boot_ops->usb_ops->usb, &MSG, 4);

    // erase boot bank info
    boot_ops->storage_ops->erase(0x2, 0x1);

    n = load_from_usb(boot_ops->usb_ops, &len);
    if (n) {
        serial_puts("*** IO ERROR ***\n");
        goto fail;
    }
    printf("Downloaded MLO. Size: %d (0x%x)\n", len, len);
    flash_image(boot_ops->storage_ops, BOOT_MLO, len);
    printf("Flashed MLO.\n");

    n = load_from_usb(boot_ops->usb_ops, &len);
    if (n) {
        serial_puts("*** IO ERROR ***\n");
        goto fail;
    }
    printf("Downloaded IFS. Size: %d (0x%x)\n", len, len);
    flash_image(boot_ops->storage_ops, BOOT_BANK1, len);
    printf("Flashed IFS.\n");

fail:
	for (;;) ;
}
