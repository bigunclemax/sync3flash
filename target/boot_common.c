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
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSE
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <aboot.h>
#include <io.h>
#include <common.h>
#include <common_proc.h>
#include <fastboot.h>
#include <usbboot_common.h>
#include <alloc.h>
#include <user_params.h>

__attribute__((__section__(".mram")))
static struct bootloader_ops boot_operations;
u32 public_rom_base;

#ifdef TWO_STAGE_OMAPBOOT

#if defined DO_MEMORY_TEST_DURING_FIRST_STAGE_IN_EBOOT || \
defined DO_MEMORY_TEST_DURING_FIRST_STAGE_IN_IBOOT
/* This sanity memory test taken from aboot.c has no
    value of its own, it is just here to demonstrate how
    and where to implement such feature if required */

void memtest(void *x, unsigned count)
{
	unsigned *w = x;
	unsigned n;
	count /= 4;

	printf("memtest write - %d\n", count);
	for (n = 0; n < count; n++) {
		unsigned chk = 0xa5a5a5a5 ^ n;
		w[n] = chk;
	}
	printf("memtest read\n");
	for (n = 0; n < count; n++) {
		unsigned chk = 0xa5a5a5a5 ^ n;
		if (w[n] != chk) {
			printf("ERROR @ %x (%x != %x)\n",
				(unsigned) (w+n), w[n], chk);
			return;
		}
	}
	printf("OK!\n");
}
#endif
#endif

struct bootloader_ops *boot_common(unsigned bootdevice)
{
	int ret = 0;
	struct bootloader_ops *boot_ops = &boot_operations;
	char buf[DEV_STR_LENGTH];

	boot_ops->board_ops = init_board_funcs();
	boot_ops->proc_ops = init_processor_id_funcs();
	boot_ops->storage_ops = NULL;
	boot_ops->pmic_ops = init_pmic_funcs();
	boot_ops->usb_ops = init_usb_funcs();

	if (boot_ops->proc_ops->proc_check_lpddr2_temp)
		boot_ops->proc_ops->proc_check_lpddr2_temp();

	if (boot_ops->proc_ops->proc_get_api_base)
		public_rom_base = boot_ops->proc_ops->proc_get_api_base();

	watchdog_disable();

	if (boot_ops->board_ops->board_mux_init)
		boot_ops->board_ops->board_mux_init();

	if (boot_ops->board_ops->board_ddr_init)
		boot_ops->board_ops->board_ddr_init(boot_ops->proc_ops);

	if (boot_ops->board_ops->board_signal_integrity_reg_init)
		boot_ops->board_ops->board_signal_integrity_reg_init
							(boot_ops->proc_ops);

	ldelay(100);

	if (boot_ops->board_ops->board_scale_vcores)
		boot_ops->board_ops->board_scale_vcores(boot_ops->proc_ops);

	if (boot_ops->board_ops->board_prcm_init)
		boot_ops->board_ops->board_prcm_init(boot_ops->proc_ops);

	init_memory_alloc();

	if (boot_ops->board_ops->board_gpmc_init)
		boot_ops->board_ops->board_gpmc_init();

	if (boot_ops->board_ops->board_late_init)
		boot_ops->board_ops->board_late_init();

	enable_irqs();

	serial_init();

	printf("%s\n", ABOOT_VERSION);
	printf("Build Info: "__DATE__ " - " __TIME__ "\n");

	if (boot_ops->pmic_ops->pmic_enable)
		boot_ops->pmic_ops->pmic_enable();

	if (boot_ops->pmic_ops->pmic_read_reset_reason)
		boot_ops->pmic_ops->pmic_read_reset_reason();

	if (boot_ops->pmic_ops->pmic_configure_pwm_mode) {
		ret = boot_ops->pmic_ops->pmic_configure_pwm_mode();
		if (ret != 0)
			printf("unable to configure PWM mode\n");
	}

	if (boot_ops->usb_ops->usb_enable) {
		boot_ops->usb_ops->usb = boot_ops->usb_ops->usb_enable();
		if (!boot_ops->usb_ops->usb)
			goto fail;
	} else
		goto fail;

	if (bootdevice == DEVICE_USB) {

		bootdevice = boot_ops->board_ops->board_get_flash_slot();
		ret = boot_ops->usb_ops->usb_open(boot_ops->usb_ops->usb,
					NO_INIT_USB, boot_ops->proc_ops);
		if (ret != 0) {
			printf("\nusb_open failed\n");
			goto fail;
		}
	}

	if (!boot_ops->board_ops->board_get_flash_slot ||
			!boot_ops->board_ops->board_set_flash_slot)
		goto fail;

	dev_to_devstr(bootdevice, buf);
	printf("sram: boot device: %s\n", buf);

	boot_ops->storage_ops = boot_ops->board_ops->board_set_flash_slot
			(bootdevice, boot_ops->proc_ops, boot_ops->storage_ops);
	if (!boot_ops->storage_ops) {
		printf("Unable to init storage\n");
		goto fail;
	}

	return boot_ops;

fail:
	printf("boot failed\n");
	while (1)
		;

	return NULL;
}

