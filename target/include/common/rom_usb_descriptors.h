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

#ifndef _ROM_USB_DESCRIPTORS_H_
#define _ROM_USB_DESCRIPTORS_H_

#ifndef VENDOR_ID
#define VENDOR_ID	0x451
#endif

#ifndef PRODUCT_ID
#define PRODUCT_ID	0xd022
#endif

#define ROM_USB_DESCRIPTOR_MANUFACTURER		0x2c
#define ROM_USB_DESCRIPTOR_PRODUCT		0x2d
#define ROM_USB_DESCRIPTOR_SERIAL		0x2e
#define ROM_USB_DESCRIPTOR_CONFIGURATION	0x2f
#define ROM_USB_DESCRIPTOR_INTERFACE		0x30

#define ROM_USB_DESCRIPTOR_CLASS		0xff
#define ROM_USB_DESCRIPTOR_SUBCLASS		0xff
#define ROM_USB_DESCRIPTOR_PROTOCOL		0xff

#define ROM_USB_DESCRIPTOR_INTERFACE_CLASS	0xff
#define ROM_USB_DESCRIPTOR_INTERFACE_SUBCLASS	0x42
#define ROM_USB_DESCRIPTOR_INTERFACE_PROTOCOL	0x03

#define ROM_USB_DESCRIPTOR_BCDDEVICE		0x0200

#endif
