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

#include <aboot.h>
#include <io.h>
#include <omap_rom.h>
#include <string.h>
#include <alloc.h>

static struct usb *usb_enable(void)
{
	struct usb *usb;

	usb = (struct usb *) alloc_memory(sizeof(struct usb));
	if (usb != NULL)
		return usb;

	printf("unable to allocate memory usb structure\n");
	return NULL;
}

struct usb_specific_functions usb_funcs = {
	.usb_open = usb_open,
	.usb_init = usb_init,
	.usb_close = usb_close,
	.usb_queue_read = usb_queue_read,
	.usb_wait_read = usb_wait_read,
	.usb_queue_write = usb_queue_write,
	.usb_wait_write = usb_wait_write,
	.usb_write = usb_write,
	.usb_read = usb_read,
	.usb_enable = usb_enable,
	.usb_configure = usb_configure
};

void *init_usb_funcs(void)
{
	return &usb_funcs;
}
