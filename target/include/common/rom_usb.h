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

#ifndef _ROM_USB_H_
#define _ROM_USB_H_

#include <rom_peripheral.h>

#define LC_STRING_DESC_MAX_LENGTH	(48)
#define HAL_USB_STRING_DESC		(3)

/* USB String descriptor */
struct usb_string_desc {
	u8  blength;
	u8  bdescriptortype;
	u8  bstring[LC_STRING_DESC_MAX_LENGTH];
} __attribute__ ((packed));

/* USB Custom descriptor */
struct usb_custom_desc {
	/* Device descriptor */
	struct {
		u8  bdeviceclass;	/* class */
		u8  bdevicesubclass;	/* subclass */
		u8  bdeviceprotocol;	/* protocol */
		u16 idvendor;		/* vendor id */
		u16 idproduct;		/* product id */
		u16 bcddevice;		/* bcddevice */
		u8  imanufacturer;	/* imanufacturer(defined by rom apis) */
		u8  iproduct;		/* iproduct(defined by rom apis) */
		u8  iserialnumber;	/* iserialnumber(defined by rom apis) */
	} __attribute__ ((packed));

	/* Configuration descriptor */
	struct {
		u8 iConfiguration;	/*iconfiguration(defined by rom apis) */
		u8 binterfaceclass;	/* binterfaceclass */
		u8 binterfacesubclass;	/* binterfacesubclass */
		u8 binterfaceprotocol;	/* binterfaceprotocol */
		u8 iinterface;		/* iinterface(defined by rom apis) */
		u8 reserved[3];		/* reserved */
	} __attribute__ ((packed));

	/* String descriptors */
	struct usb_string_desc *str[5];

} __attribute__ ((packed));

struct per_usb_config {
	u32 configid;
	u32 value;
};

struct usb {
	struct per_handle dread;
	struct per_handle dwrite;
	struct per_driver *io;
};

#define USB_SETCONFIGDESC_ATTRIBUTES      (0)
#define USB_SETCONFIGDESC_MAXPOWER        (1)
#define USB_SETSUSPEND_CALLBACK           (2)

#if defined(CONFIG_IS_OMAP5)
#define HAL_USB_TRB_HWO_HW_OWNED		(1)
#define HAL_USB_TRB_CHN_NO_CHAIN		(0)
#define HAL_USB_TRB_LST_LAST			(1)
#define HAL_USB_TRB_CSP_NO_CONT_SHORT_PACKET	(0)
#define HAL_USB_TRB_TRBCTL_TYPE_NORMAL		(1)

struct usb_trb {
	u32 ptrlo;       /* buffer ptr low */
	u32 ptrhi;       /* buffer ptr high */
	u32 bufsiz:24;   /* buffer size */
	u32 pcm1:2;      /* packet count */
	u32 rsvd1:2;     /* reserved */
	u32 trbsts:4;    /* trb status */
	u32 hwo:1;       /* hardware owned */
	u32 lst:1;       /* last trb */
	u32 chn:1;       /* chained */
	u32 csp:1;       /* continue on short pkt */
	u32 trbctl:6;    /* trb control */
	u32 ispimi:1;    /* interrupt on short pkt */
	u32 ioc:1;       /* interrupt on completion */
	u32 rsvd3:2;     /* reserved */
	u32 streamid:16; /* streamid */
	u32 rsvd2:2;     /* reserved */
} __attribute__ ((packed));

struct usb_ioconf {
	u32 mode;                  /* Data transfer mode (deprecated) */
	u32 conf_timeout;          /* Redefines the USB timeout       */
	struct usb_trb *trb_pool;  /* User defined Trb pool           */
	struct usb_custom_desc *usr_desc; /* User defined usb descriptors */
};
#endif

#endif
