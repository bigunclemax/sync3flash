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

#ifndef __DEVICE_TREE_H__
#define __DEVICE_TREE_H__

#include <usbboot_common.h>

#define ENVIRO_MAGIC "ANDROID_ENV!"
#define ENVIRO_MAGIC_SIZE 12
#define ENVIRO_NAME_SIZE 16
#define ENVIRO_ARGS_SIZE 512

#define DEVICE_TREE	0x825f0000

typedef struct enviro_img_hdr enviro_img_hdr;

/* Probably should move this */
struct enviro_img_hdr
{
    unsigned char magic[ENVIRO_MAGIC_SIZE];

    unsigned dev_tree_size;  /* size in bytes */
    unsigned dev_tree_addr;  /* physical load addr */

    unsigned enviroment_size; /* size in bytes */

    unsigned splash_img_size;  /* size in bytes */
    unsigned splash_img_addr;  /* physical load addr */

    unsigned fastboot_img_size;  /* size in bytes */
    unsigned fastboot_img_addr;  /* physical load addr */

    unsigned charger_img_size;  /* size in bytes */
    unsigned charger_img_addr;  /* physical load addr */

    unsigned page_size;    /* flash page size we assume */

    unsigned char name[ENVIRO_NAME_SIZE]; /* asciiz product name */

    unsigned id[8]; /* timestamp / checksum / sha1 / etc */
};

u32 load_dev_tree(struct bootloader_ops *boot_ops, u32 atag_load_addr);

#endif
