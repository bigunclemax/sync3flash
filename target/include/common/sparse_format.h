/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _SPARSE_FORMAT_H_
#define _SPARSE_FORMAT_H_

#define SPARSE_HEADER_MAGIC	0xed26ff3a

#define CHUNK_TYPE_RAW		0xCAC1
#define CHUNK_TYPE_FILL		0xCAC2
#define CHUNK_TYPE_DONT_CARE	0xCAC3
#define CHUNK_TYPE_CRC          0xCAC4


typedef struct sparse_header {
	u32	magic;		/* 0xed26ff3a */
	u16	major_version;	/* (0x1) - reject images with higher major
								versions */
	u16	minor_version;	/* (0x0) - allow images with higer minor
								versions */
	u16	file_hdr_sz;	/* 28 bytes for first revision of the file
								format */
	u16	chunk_hdr_sz;	/* 12 bytes for first revision of the file
								format */
	u32	blk_sz;		/* block size in bytes, must be a multiple
							of 4 (4096) */
	u32	total_blks;	/* total blocks in the non-sparse output
								image */
	u32	total_chunks;	/* total chunks in the sparse input image */
	u32	image_checksum;	/* CRC32 checksum of the original data,
						counting "don't care" */
				/* as 0. Standard 802.3 polynomial, use a
							Public Domain */
				/* table implementation */
} sparse_header_t;

typedef struct chunk_header {
	u16	chunk_type;	/* 0xCAC1 -> raw; 0xCAC2 -> fill;
				0xCAC3 -> don't care */
	u16	reserved1;
	u32	chunk_sz;	/* in blocks in output image */
	u32	total_sz;	/* in bytes of chunk input file
				including chunk header and data */
} chunk_header_t;
#endif
