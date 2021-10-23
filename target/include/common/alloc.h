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

#define HEAP_SIZE	0x1F400000

struct mem_alloc_header;

struct mem_alloc_header {
	u32 status;	/* free =0, used =1 */
	u32 section_size;
	u8 *data;
	struct mem_alloc_header *next;
};

#define MALLOC_HDR_SIZE (sizeof(struct mem_alloc_header))

/* Need to align to 512 bytes as buffers might be passed to MMC or SATA
 * and for them, the data needs to be aligned to 512 bytes
 */
#define ALLOC_ALIGN_SIZE	512
#define ALLOC_ALIGN_SHIFT	9

/* Threshold for breaking a segment */
#define FRAGMENT_THRESH (MALLOC_HDR_SIZE + ALLOC_ALIGN_SIZE)

void init_memory_alloc(void);
int *alloc_memory(int size);
int *zalloc_memory(int size);
int free_memory(void *ptr_to_data);
