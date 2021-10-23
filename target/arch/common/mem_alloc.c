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
#include <usbboot_common.h>
#include <alloc.h>

#ifdef DEBUG
#define DBG(x...) printf(x)
#else
#define DBG(x...)
#endif /* DEBUG */

__attribute__((__section__(".sdram")))
static struct mem_alloc_header head;

void init_memory_alloc(void)
{
	u8 *next;

	head.status = 0;
	head.section_size = HEAP_SIZE - (2 * MALLOC_HDR_SIZE);
	head.data = (u8 *)&head;
	head.data += MALLOC_HDR_SIZE;
	next = (u8 *)(head.data);
	next += head.section_size;
	head.next = (struct mem_alloc_header *)next;

	head.next->status = 0;
	head.next->section_size = 0;
	head.next->data = NULL;
	head.next->next = NULL;

	return;
}

static int align_size_requested(int size)
{
	if (size % ALLOC_ALIGN_SIZE)
		size = ((size >> ALLOC_ALIGN_SHIFT) + 1) << ALLOC_ALIGN_SHIFT;

	return size;
}

int *alloc_memory(int size)
{
	struct mem_alloc_header *hdr = &head;
	struct mem_alloc_header *next;
	u8 *tptr;
#ifdef DEBUG
	u32 max_available = 0;
	while (hdr->next) {
		printf("hdr = 0x%08x, data = 0x%08x, next = 0x%08x\n",
			hdr, hdr->data, hdr->next);
		if (hdr->section_size > max_available)
			max_available = hdr->section_size;
		hdr = hdr->next;
	}
	hdr = &head;
	printf("Maximum section available = 0x%08x bytes. Req=0x%08x\n",
				max_available, size);
#endif

	if (size < 1) {
		printf("user has requested an invalid amount of memory\n");
		return NULL;
	}
	size = align_size_requested(size);

	/*
	* Traverse the heap to find an available section:
	* If a section is unused and can fit requested size
	* then allocated the section and break that section
	* if necessary. If you reached end of linked list
	* you have no space left.
	*/
	while (hdr->next) {
		if ((!hdr->status) && (hdr->section_size >= size)) {
			DBG("Section at 0x%08x can be used\n\n\n",
							(u32)hdr);
			break;
		}
		hdr = hdr->next;
	}

	if (!hdr->next) {
		printf("Reached end of heap. Requested size not available\n");
		return NULL;
	}

	/* Break segment if segment is too big */
	if (hdr->section_size > (size + FRAGMENT_THRESH)) {
		tptr = hdr->data + size;
		next = (struct mem_alloc_header *) tptr;
		next->status = 0;
		next->section_size = hdr->section_size -
					(size + MALLOC_HDR_SIZE);
		next->next = hdr->next;
		next->data = (u8 *)next + MALLOC_HDR_SIZE;

		hdr->next = next;
		hdr->section_size = size;
	}
	hdr->status = 1;
	DBG("allocated: %p data %p size %d\n",
		hdr, hdr->data, hdr->section_size);
	return (int *) hdr->data;
}

int *zalloc_memory(int size)
{
	/* call alloc_memory() and then memset the
	*  allocated memory to zero
	*  This is an expensive operation to do, so use this judiciously!!!
	*/

	void *ptr_mem;

	ptr_mem = alloc_memory(size);
	if (ptr_mem != NULL) {
		memset(ptr_mem, 0, size);
		return ptr_mem;
	} else {
		printf("invalid memory section\n");
		return NULL;
	}
}

int free_memory(void *ptr_to_data)
{
	struct mem_alloc_header *hdr = &head;
	u8 *free;
	struct mem_alloc_header *curr, *tofree = NULL;
	free = (u8 *) (ptr_to_data - MALLOC_HDR_SIZE);
	curr = (struct mem_alloc_header *)free;

	while (hdr->next) {
		if (hdr == curr)
			/* found the segment to free */
			tofree = hdr;

		/* if the next segment is not the last one in the heap
		   and both consecutive segments are free, concatenate them */
		if ((hdr->next->next) && ((hdr->status == 0) &&
			(hdr->next->status == 0))) {
			/* the new segment size is the size of both segments
			   added to the size of the discarded header */
			hdr->section_size += hdr->next->section_size + MALLOC_HDR_SIZE;
			hdr->next = hdr->next->next;
		}
		hdr = hdr->next;
	}

	if ((!tofree) || (!tofree->next)) {
		printf("Invalid pointer: 0x%08x\n", ptr_to_data);
		return -1;
	}

	tofree->status = 0;
	DBG("user has freed memory section 0x%x of size 0x%x\n",
				tofree, tofree->section_size);
	return 0;
}
