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

MEMORY
	{
	sram (A): ORIGIN = 0x40309000, LENGTH = 53K
	mram (Arw) : ORIGIN = 0x40309000 + 53k, LENGTH = 5k
	sdram (Arw) : ORIGIN = 0xA0BFFFFF, LENGTH = 500m
	}


ENTRY(_start)
SECTIONS {
	.text : { *(.text) } > sram

	. = ALIGN(4);
	.got : { *(.got) }

	. = ALIGN(4);
	.rodata : { *(.rodata) } > sram

	. = ALIGN(4);
	.data : { *(.data) } >sram

	. = ALIGN(4);
	__bss_start = .;
	.bss : { *(.bss) }>sram
	. = ALIGN(4);
	__bss_end = .;

	. = ALIGN(4);
	.mram (NOLOAD): { *(.mram) } >mram

	. = ALIGN(4);
	.sdram (NOLOAD): { *(.sdram) } > sdram
}
