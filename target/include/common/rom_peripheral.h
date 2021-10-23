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

#ifndef _ROM_PERIPHERAL_H_
#define _ROM_PERIPHERAL_H_

/* Peripheral device list */
#define DEVICE_NULL	0x40
#define DEVICE_UART1	0x41
#define DEVICE_UART2	0x42
#define DEVICE_UART3	0x43
#define DEVICE_UART4	0x44
#define DEVICE_USB	0x45
#define DEVICE_USBEXT	0x46

/* Tranfer mode */
#if defined(CONFIG_IS_OMAP4)
#define XFER_MODE_CPU 0
#define XFER_MODE_DMA 1
#endif

/* Peripheral Handle */
struct per_handle {
	void *config_object;
	void (*callback)(struct per_handle *rh);
	void *data;
	u32 length;
	u16 *options;
#if defined(CONFIG_IS_OMAP4)
	u32 xfer_mode;
#endif
	u32 device_type;
	volatile u32 status;
	u16 hs_toc_mask;
	u16 gp_toc_mask;
	u32 *device_data;
};

/* Peripheral Driver */
struct per_driver {
	int (*init)(struct per_handle *rh);
	int (*read)(struct per_handle *rh);
	int (*write)(struct per_handle *rh);
	int (*close)(struct per_handle *rh);
	int (*config)(struct per_handle *rh, void *x);
};

#endif
