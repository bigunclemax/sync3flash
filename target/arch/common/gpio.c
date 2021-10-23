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

#include <io.h>

#include <gpio.h>

#define GPIO_CTRL    0x130
#define GPIO_OE      0x134
#define GPIO_DATAIN  0x138
#define GPIO_DATAOUT 0x13C
#define GPIO_CLEAR   0x190
#define GPIO_SET     0x194

void gpio_write(unsigned gpio, unsigned set)
{
	unsigned base = gpio_base[gpio / 32];
	unsigned bit = 1 << (gpio % 32);

	/* ensure that this GPIO bank is enabled */
	writel(0, base + GPIO_CTRL);

	/* enable output for this gpio */
	writel(readl(base + GPIO_OE) & (~bit), base + GPIO_OE);

	/* set or clear the bit */
	writel(bit, base + (set ? GPIO_SET : GPIO_CLEAR));
}

unsigned gpio_read(unsigned gpio)
{
	unsigned base = gpio_base[gpio / 32];
	unsigned bit = 1 << (gpio % 32);

	/* determine if this gpio is configured as an input or output */
	if (bit & readl(base + GPIO_OE)) {
		/* pin is configured as an input */
		return (bit & readl(base + GPIO_DATAIN)) ? 1 : 0;
	} else {
		/* pin is configured as an output */
		return (bit & readl(base + GPIO_SET)) ? 1 : 0;
	}
}
