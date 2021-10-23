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

int i2c_init(hal_i2c i2c_id)
{
	int n;

	/* configure mux and pull ups */
	n = rom_hal_ctrl_configurepads(HAL_MODULE_I2C, i2c_id);
	if (n)
		return n; /* returns when n!=STATUS_OKAY */

	/* enables all interface and functional clocks */
	n = rom_hal_cm_enablemoduleclocks(HAL_MODULE_I2C, i2c_id);
	if (n)
		return n; /* returns when n!=STATUS_OKAY */

	/* enables interrupt handler and interrupt controller */
	rom_irq_initialize();

	/* initializes the selected I2C module */
	n = rom_hal_i2c_initialize(i2c_id);
	if (n)
		return n; /* returns when n!=STATUS_OKAY */

	return 0;
}

int i2c_write(hal_i2c i2c_id, u16 slave, u32 count, void *data, u32 start_time,
								u32 timeout)
{
	int n;

	/* i2c - write */
	n = rom_hal_i2c_write(i2c_id, slave, count, data, start_time,
								timeout);
	if (n)
		return n; /* returns when n!=STATUS_OKAY */

	return 0;
}

int i2c_read(hal_i2c i2c_id, u16 slave, u32 count, void *data, u32 start_time,
								u32 timeout)
{
	int n;

	/* i2c - write */
	n = rom_hal_i2c_write(i2c_id, slave, 1, data, start_time,
								timeout);
	if (n)
		return n; /* returns when n!=STATUS_OKAY */

	/* i2c - read */
	n = rom_hal_i2c_read(i2c_id, slave, count, data, start_time,
								timeout);
	if (n)
		return n; /* returns when n!=STATUS_OKAY */

	return 0;
}

int i2c_close(hal_i2c i2c_id)
{
	int n;

	/* i2c - close */
	n = rom_hal_i2c_close(i2c_id);
	if (n)
		return n; /* returns when n!=STATUS_OKAY */

	/* disables all interface and functional clocks */
	n = rom_hal_cm_disablemoduleclocks(HAL_MODULE_I2C, i2c_id);
	if (n)
		return n; /* returns when n!=STATUS_OKAY */

	return 0;
}
