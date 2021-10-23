/*
* Copyright (C) 2012 Texas Instruments, Inc.
* All rights reserved.
* Author: Christina Warren <cawarren@ti.com>
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

/* Watchdog Enable : Enables the WD2 timer */
int watchdog_enable(void)
{
	rom_hal_wdtimer_enable();

	return 0;
}

/* Watchdog Disable : Disables the WD2 timer */
int watchdog_disable(void)
{
	rom_hal_wdtimer_disable();

	return 0;
}

/* Watchdog Reset : Resets the WD2 timer */
int watchdog_reset(void)
{
	rom_hal_wdtimer_reset();

	return 0;
}

/* Watchdog Set timeout : Set a watchdog timeout
 * The timeout value cannot be <1ms or > 36h 24 min
 * Timeout value should be in ms
 */
int watchdog_set_timeout(u32 timeout)
{
	int n;

	n = rom_hal_wdtimer_set_timeout(timeout);
	if (n)
		return n; /* returns when n!=STATUS_OKAY */

	return 0;
}
