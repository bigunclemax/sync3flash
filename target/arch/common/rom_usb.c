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
#include <omap_rom.h>
#include <string.h>
#include <usbboot_common.h>
#if defined(CONFIG_IS_OMAP5)
#include <rom_usb_descriptors.h>
#endif

static struct proc_specific_functions *local_proc_ops;

#if defined(CONFIG_IS_OMAP5)
static struct usb_ioconf ioconf_read;
static struct usb_ioconf ioconf_write;
volatile struct usb_trb trbout;

static struct usb_string_desc usb_fastboot_manufacturer_string_desc;
static struct usb_string_desc usb_fastboot_product_string_desc;
static struct usb_string_desc usb_fastboot_serial_string_desc;
static struct usb_string_desc usb_fastboot_configuration_string_desc;
static struct usb_string_desc usb_fastboot_interface_string_desc;

static struct usb_custom_desc usb_fastboot_desc = {
	{
		/* Device descriptor */
		ROM_USB_DESCRIPTOR_CLASS,		/* class */
		ROM_USB_DESCRIPTOR_SUBCLASS,		/* subclass */
		ROM_USB_DESCRIPTOR_PROTOCOL,		/* protocol */
		VENDOR_ID,				/* vendor id */
		PRODUCT_ID,				/* product id */
		ROM_USB_DESCRIPTOR_BCDDEVICE,		/* bcddevice */
		ROM_USB_DESCRIPTOR_MANUFACTURER,	/* imanufacturer */
		ROM_USB_DESCRIPTOR_PRODUCT,		/* iproduct */
		ROM_USB_DESCRIPTOR_SERIAL,		/* iserialnumber */
	},
	{
		/* Configuration descriptor */
		ROM_USB_DESCRIPTOR_CONFIGURATION,	/* iconfiguration */
		ROM_USB_DESCRIPTOR_INTERFACE_CLASS,	/* binterfaceclass */
		ROM_USB_DESCRIPTOR_INTERFACE_SUBCLASS,	/* binterfacesubclass */
		ROM_USB_DESCRIPTOR_INTERFACE_PROTOCOL,	/* binterfaceprotocol */
		ROM_USB_DESCRIPTOR_INTERFACE,		/* iinterface */
		{0x00, 0x00, 0x00}			/* reserved */
	},
	{
		/* String descriptors */
		&usb_fastboot_manufacturer_string_desc,
		&usb_fastboot_product_string_desc,
		&usb_fastboot_serial_string_desc,
		&usb_fastboot_configuration_string_desc,
		&usb_fastboot_interface_string_desc
	}
};

static void usb_desc_unicode(char *pout, char *pin)
{
	u32 i = 0;

	while (i++ < 24 && *pin != '\0') {
		*pout++ = *pin++;
		*pout++ = '\0';
	}
}

static void usb_desc_configure(u8 desc_string)
{
	char str[48];

	switch (desc_string) {
	case ROM_USB_DESCRIPTOR_MANUFACTURER:
		usb_fastboot_manufacturer_string_desc.blength =
						2+2*strlen(MANUFACTURER_NAME);
		usb_fastboot_manufacturer_string_desc.bdescriptortype =
							HAL_USB_STRING_DESC;
		usb_desc_unicode(
			(char *)usb_fastboot_manufacturer_string_desc.bstring,
			MANUFACTURER_NAME);
		break;
	case ROM_USB_DESCRIPTOR_PRODUCT:
		usb_fastboot_product_string_desc.blength =
					2+2*strlen(PRODUCT_NAME);
		usb_fastboot_product_string_desc.bdescriptortype =
							HAL_USB_STRING_DESC;
		usb_desc_unicode(
			(char *)usb_fastboot_product_string_desc.bstring,
			PRODUCT_NAME);
		break;
	case ROM_USB_DESCRIPTOR_SERIAL:
		strcpy(str, local_proc_ops->proc_get_serial_num());
		usb_fastboot_serial_string_desc.blength = 2+2*strlen(str);
		usb_fastboot_serial_string_desc.bdescriptortype =
							HAL_USB_STRING_DESC;
		usb_desc_unicode(
			(char *)usb_fastboot_serial_string_desc.bstring, str);
		break;
	case ROM_USB_DESCRIPTOR_CONFIGURATION:
		strcpy(str, "Android Fastboot");
		usb_fastboot_configuration_string_desc.blength =
						2+2*strlen(str);
		usb_fastboot_configuration_string_desc.bdescriptortype =
							HAL_USB_STRING_DESC;
		usb_desc_unicode(
			(char *)usb_fastboot_configuration_string_desc.bstring,
			str);
		break;
	case ROM_USB_DESCRIPTOR_INTERFACE:
		strcpy(str, "Android Fastboot");
		usb_fastboot_interface_string_desc.blength = 2+2*strlen(str);
		usb_fastboot_interface_string_desc.bdescriptortype =
							HAL_USB_STRING_DESC;
		usb_desc_unicode(
			(char *)usb_fastboot_interface_string_desc.bstring,
			str);
		break;
	}

	return;
}
#endif

int usb_open(struct usb *usb, int init,
				struct proc_specific_functions *proc_ops)
{
	struct per_handle *boot;
	u32 device;
	u16 options = 1;
	int n;

	/*clear global usb structure*/
	memset(usb, 0, sizeof(*usb));

	local_proc_ops = proc_ops;

	if (init) {
		device = DEVICE_USB;
		usb->dread.config_object = NULL;
		usb->dread.options = &options;
		usb->dread.device_type = DEVICE_USB;

		usb->dwrite.config_object = NULL;
		usb->dwrite.options = &options;
		usb->dwrite.device_type = DEVICE_USB;

#if defined(CONFIG_IS_OMAP4)
		/* Bulk transfers default to DMA mode */
		usb->dread.xfer_mode = XFER_MODE_DMA;
		usb->dwrite.xfer_mode = XFER_MODE_DMA;
#endif

		n = rom_get_device_data((void *)&usb->dread.device_data);
		if (n)
			return n;

		usb->dwrite.device_data = usb->dread.device_data;
	} else {
		/* get peripheral device descriptor
		that was used during rom usb boot */
		n = rom_get_per_device(&boot);
		if (n)
			return n;

		device = boot->device_type;
		memcpy(&usb->dread, boot, sizeof(struct per_handle));
		memcpy(&usb->dwrite, boot, sizeof(struct per_handle));
	}


	/* get rom usb driver */
	n = rom_get_per_driver(&usb->io, device);
	if (n)
		return n;

	if (init)
		usb_init(usb);

	return 0;
}

/* from omap5 ES2.0 onwards, close the connection
*  and reconnect using custom usb descriptors
*/
void usb_reopen(struct usb *usb)
{
#if defined(CONFIG_IS_OMAP5)
	usb_desc_configure(ROM_USB_DESCRIPTOR_MANUFACTURER);
	usb_desc_configure(ROM_USB_DESCRIPTOR_PRODUCT);
	usb_desc_configure(ROM_USB_DESCRIPTOR_SERIAL);
	usb_desc_configure(ROM_USB_DESCRIPTOR_CONFIGURATION);
	usb_desc_configure(ROM_USB_DESCRIPTOR_INTERFACE);

	ioconf_read.mode          = 0;
	ioconf_read.conf_timeout  = 0;
	ioconf_read.trb_pool      = NULL;
	ioconf_read.usr_desc      = &usb_fastboot_desc;
	ioconf_write.mode         = 0;
	ioconf_write.conf_timeout = 0;
	ioconf_write.trb_pool     = NULL;
	ioconf_write.usr_desc     = &usb_fastboot_desc;

	usb->dread.config_object  = &ioconf_read;
	usb->dwrite.config_object = &ioconf_write;
#endif
	usb_init(usb);

	return;
}

int usb_queue_read(struct usb *usb, void *data, unsigned len)
{
	int n;

#if defined(CONFIG_IS_OMAP5)
	/* only applicable to omap5 ES1.0 */
	if (local_proc_ops->proc_get_proc_id) {
		if (local_proc_ops->proc_get_proc_id() <
							OMAP_5430_ES2_DOT_0) {

			memset((void *)&trbout, 0, sizeof(trbout));
			ioconf_read.mode         = 0;
			ioconf_read.conf_timeout = 0;
			ioconf_read.trb_pool     = (struct usb_trb *) &trbout;
			usb->dread.config_object = &ioconf_read;

			trbout.ptrlo    = (u32)data;
			trbout.ptrhi    = 0;
			trbout.bufsiz   =
				((len >> 9) + ((len & 0x1FF) ? 1 : 0)) << 9;
			trbout.hwo      = HAL_USB_TRB_HWO_HW_OWNED;
			trbout.chn      = HAL_USB_TRB_CHN_NO_CHAIN;
			trbout.lst      = HAL_USB_TRB_LST_LAST;
			trbout.csp      = HAL_USB_TRB_CSP_NO_CONT_SHORT_PACKET;
			trbout.trbctl   = HAL_USB_TRB_TRBCTL_TYPE_NORMAL;

		}
	}
#endif

	usb->dread.data = data;
	usb->dread.length = len;
	usb->dread.status = -1;
	usb->dread.device_type = DEVICE_USB;
	usb->dread.callback = NULL;
	n = usb->io->read(&usb->dread);

	return n;
}

int usb_wait_read(struct usb *usb)
{
	for (;;) {
#if defined(CONFIG_IS_OMAP4)
		/* Below code works around an issue with omap4 rom driver for */
		/* handling transfer completion on short packet. It is quite  */
		/* tricky to fix the regular DMA mode. By using CPU mode, the */
		/* rom handler properly copies the received data but lacks    */
		/* signaling transfer completion in status field. We then     */
		/* detect it by inspecting the receive buffer and checking if */
		/* it has been filled up with the received data.              */
		if ((usb->dread.xfer_mode == XFER_MODE_CPU)
			&& (readl((u32)usb->dread.data) != 0x0))
			usb->dread.status = STATUS_OKAY;
#endif

		if (usb->dread.status == -1)
			continue;
		if (usb->dread.status == STATUS_WAITING)
			continue;

		return usb->dread.status;
	}
}

int usb_queue_write(struct usb *usb, void *data, unsigned len)
{
	int n;

#if defined(CONFIG_IS_OMAP5)
	/* only applicable to omap5 ES1.0 */
	if (local_proc_ops->proc_get_proc_id) {
		if (local_proc_ops->proc_get_proc_id() <
							OMAP_5430_ES2_DOT_0) {
			ioconf_write.mode         = 0;
			ioconf_write.conf_timeout = 0;
			ioconf_write.trb_pool     = 0;
			usb->dwrite.config_object = &ioconf_write;
		}
	}
#endif
	usb->dwrite.data = data;
	usb->dwrite.length = len;
	usb->dwrite.status = -1;
	usb->dwrite.device_type = DEVICE_USB;
	usb->dwrite.callback = NULL;

	n = usb->io->write(&usb->dwrite);

	return n;
}

int usb_wait_write(struct usb *usb)
{
	for (;;) {
		if (usb->dwrite.status == -1)
			continue;
		if (usb->dwrite.status == STATUS_WAITING)
			continue;
		return usb->dwrite.status;
	}
}

#define USB_MAX_IO 65536
int usb_read(struct usb *usb, void *data, unsigned len)
{
	unsigned xfer;
	unsigned char *x = data;
	int n;

	while (len > 0) {
		xfer = (len > USB_MAX_IO) ? USB_MAX_IO : len;

		n = usb_queue_read(usb, x, xfer);
		if (n)
			return n;

		n = usb_wait_read(usb);
		if (n)
			return n;

		x += xfer;
		len -= xfer;
	}
	return 0;
}

int usb_write(struct usb *usb, void *data, unsigned len)
{
	int n;

	n = usb_queue_write(usb, data, len);
	if (n)
		return n;

	return usb_wait_write(usb);
}

void usb_init(struct usb *usb)
{
	usb->io->init(&usb->dread);
}

void usb_close(struct usb *usb)
{
	usb->io->close(&usb->dread);
}

void usb_configure(struct usb *usb, unsigned mode)
{
#if defined(CONFIG_IS_OMAP4)
    /* On OMAP4, this function adds ability to configure    */
	/* bulk transfers to operate either in DMA or CPU mode. */
	/* It is not applicable to OMAP5 (use DMA always).      */
	usb->dread.xfer_mode = mode;
	usb->dwrite.xfer_mode = mode;
#endif
}
