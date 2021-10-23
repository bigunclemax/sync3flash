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

#include <serial.h>

void serial_init(void)
{
	unsigned divisor = CONFIG_SERIAL_CLK_HZ / 16 / CONFIG_BAUDRATE;

	WR(0x00, IER);
	WR(0x07, MDR1); /* reset */
	WR(0x83, LCR);  /* 8N1 + banksel */
	WR(divisor & 0xFF, DLL);
	WR(divisor >> 8, DLM);
	WR(0x03, LCR);  /* 8N1 */
	WR(0x03, MCR);  /* DTR, RTS */
	WR(0x07, FCR);  /* reset and enable FIFO */
	WR(0x00, MDR1); /* run */
}

static inline void _serial_putc(char c)
{
	while (!(RD(LSR) & 0x20))
		;
	WR(c, THR);
}

void serial_putc(char c)
{
	if (c == '\n')
		_serial_putc('\r');
	_serial_putc(c);
}

void serial_puts(const char *s)
{
	while (*s)
		serial_putc(*s++);
}

