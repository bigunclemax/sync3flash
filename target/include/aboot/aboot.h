/*
 * Copyright (C) 2010 The Android Open Source Project
 * All rights reserved.
 *
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

#ifndef _ABOOT_H_
#define _ABOOT_H_

#include <types.h>
#include <stdarg.h>
#include <usbboot_common.h>
#include <omap_rom.h>

void serial_init(void);
void serial_putc(char c);
void serial_puts(const char *s);

int printf(const char *fmt, ...);
int snprintf(char *str, size_t len, const char *fmt, ...);
int vsprintf(char *str, const char *fmt, va_list ap);
int vsnprintf(char *str, size_t len, const char *fmt, va_list ap);

int strlen(const char *s);
void memset(void *p, unsigned char c, unsigned len);

void enable_irqs(void);
void disable_irqs(void);

void reset_cpu(void);

u32 check_loop(u32 mask, u32 match, u32 addr);
void ldelay(unsigned long loops);
void set_modify(u32 reg, u32 mask, u32 value);
void dev_to_devstr(u8 dev, char *devstr);
int devstr_to_dev(const char *devstr, u8 *dev);

/* global configuration, changable by board file */
int do_booti(struct bootloader_ops *boot_ops, char *info,
				void *download_addr);

/* rev-id stuff */
typedef enum {
	OMAP_REV_INVALID,
	OMAP_4430_ES1_DOT_0,
	OMAP_4430_ES2_DOT_0,
	OMAP_4430_ES2_DOT_1,
	OMAP_4430_ES2_DOT_2,
	OMAP_4430_ES2_DOT_3,
	OMAP_4460_ES1_DOT_0,
	OMAP_4460_ES1_DOT_1,
	OMAP_4470_ES1_DOT_0,
	OMAP_5430_ES1_DOT_0,
	OMAP_5432_ES1_DOT_0,
	OMAP_5430_ES2_DOT_0,
	OMAP_5432_ES2_DOT_0,

} omap_rev;

/* omap-type */
typedef enum {
	OMAP_TYPE_TEST,
	OMAP_TYPE_EMU,
	OMAP_TYPE_SEC,
	OMAP_TYPE_GP,
	OMAP_TYPE_BAD,
} omap_type;

#endif
