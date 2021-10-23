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

#ifndef _SMARTIO_H_
#define _SMARTIO_H_

#include <types.h>

/* smartio impedances and slew rates */

struct smartio {
	u32 base;
	u32 mask;
	u32 value;
};

void configure_smartio(struct smartio *smartio_config);

#define CONTROL_SMART1IO_PADCONF_0	0x4A002DA8
#define CONTROL_SMART1IO_PADCONF_1	0x4A002DAC
#define CONTROL_SMART1IO_PADCONF_2	0x4A002DB0
#define CONTROL_SMART2IO_PADCONF_0	0x4A002DB4
#define CONTROL_SMART2IO_PADCONF_1	0x4A002DB8
#define CONTROL_SMART2IO_PADCONF_2	0x4A002DBC
#define CONTROL_SMART3IO_PADCONF_0	0x4A002DC0
#define CONTROL_SMART3IO_PADCONF_1	0x4A002DC4

/* CONTROL_SMART1IO_PADCONF_0 FIELDS */
#define MCSPI1_DS	0
#define HSI2_DS	2
#define HSI1_DS	4
#define EMMC_DS	8
#define C2C_PART2_DS	16
#define C2C_PART1_DS	18

/* CONTROL_SMART1IO_PADCONF_1 FIELDS */
#define MCSPI2_DS	30

/* CONTROL_SMART1IO_PADCONF_2 FIELDS */
#define C2C_PART4_DS	14
#define C2C_PART3_DS	16
#define WLSDIO_DS	22

/* CONTROL_SMART2IO_PADCONF_0 FIELDS */
#define EMMC_SC	8
#define C2C_PART2_SC	16
#define C2C_PART1_SC	18

/* CONTROL_SMART2IO_PADCONF_1 FIELDS */
#define MCSPI2_SC	24
#define HSI2_SC	28

/* CONTROL_SMART2IO_PADCONF_2 FIELDS */
#define C2C_PART4_SC	16
#define C2C_PART3_SC	18
#define WLSDIO_SC	20

/* CONTROL_SMART3IO_PADCONF_1 FIELDS */
#define USBB3_SR	14
#define USBB2_SR	17
#define USBB1_SR	20
#define USBB3_I	23
#define USBB2_I	26
#define USBB1_I	29

/* mask */
#define SMARTIO_FIELD_MASK	0x3

/* impedance */
#define IM_120_OHM	0
#define IM_60_OHM	0x1
#define IM_45_OHM	0x2
#define IM_30_OHM	0x3

/* slew rate */
#define SR_FAST	0x2
#define SR_NA	0x3

#endif
