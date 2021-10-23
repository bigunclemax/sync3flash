/*
 * Copyright (C) 2012, Texas Instruments, Inc.
 * Texas Instruments, <www.ti.com>
 *
 * Copyright (C) 2012 The Android Open Source Project
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


#define PRM_RSTCTRL                     0x4AE07B00
#define PRM_RSTCTRL_RESET_COLD_BIT      (1<<1)

.text
.globl _start

_start:
		/* obtain address of where _start was loaded */
		adr r7, _start
		adr r8, size
		ldr r8, [r8]

		/* image end (source) */
		add r7, r8, r7

		/* image start and end (destination) */
		ldr r8, =_start
		ldr r9, =__bss_end

copy:
		ldr r6, [r7], #-4
		str r6, [r9], #-4
		cmp r9, r8
		bne copy

clear_bss:
		ldr r7, =__bss_start
		ldr r8, =__bss_end
		mov r9, #0

clrbss_loop:
		str r9, [r7]
		cmp r7, r8
		add r7, r7, #4
		bne clrbss_loop

start_aboot:
		ldr r8, =aboot
		bx r8
		b .

.global ldelay
ldelay:
       subs r0, r0, #1
       bne ldelay
       bx lr

size:
		.word __bss_end - _start

.global enable_irqs
.global disable_irqs

enable_irqs:
	cpsie I
	bx lr

disable_irqs:
	cpsid I
	bx lr

.globl reset_cpu
reset_cpu:
        ldr     r1, rstctl      /* get addr for global reset reg */
        ldr     r3, rstbit      /*  sw reset bit */
        str     r3, [r1]        /* force reset */
        mov     r0, r0
_loop_forever:
        b       _loop_forever
rstctl:
        .word   PRM_RSTCTRL
rstbit:
        .word   PRM_RSTCTRL_RESET_COLD_BIT
