/*
 * Copyright (C) 2010 The Android Open Source Project
 * All rights reserved.
 *
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

#ifndef __STRING_H_
#define __STRING_H_

#include <types.h>
#include <stdarg.h>

/* memcmp - compare memory areas
 * @cs: One area of memory
 * @ct: Another area of memory
 * @count: The size of the area.
 *
 * Compares "count" number of bytes of memory areas cs and ct.
 *
 * Returns an integer less than, equal to, or greater than 0
 * if cs is found, respectively to be less than, equal to
 * or greater than ct.
 */
int memcmp(const void *cs, const void *ct, size_t count);

/* memcpy - copy memory area
 * @_dst: Where to copy to
 * @_src: Where to copy from
 * @count: The size of the area.
 *
 * Copies the memory area from specified by _src to that
 * specified by _dst. The number of bytes copied is decided
 * by count.
 *
 * Returns a pointer to the destination.
 */
void *memcpy(void *_dst, const void *_src, unsigned count);

/* memset - fill the memory with a constant byte
 * @s: Pointer to the start of the area.
 * @c: The byte to fill the area with
 * @count: The size of the area.
 *
 * Fill "size" number of bytes of the memory area
 * pointed to by _ptr with the value contained in c.
 *
 * Returns a pointer to the memory area pointed to by _ptr.
 */
void memset(void *_ptr, unsigned char c, unsigned size);

/* memmove - copy memory area
 * @dest: Where to copy to
 * @src: Where to copy from
 * @count: The size of the area.
 *
 * Copied "count" number of bytes from memory area src to
 * memory area dest.
 *
 * Returns a pointer to a dest.
 */
void *memmove(void *dest, const void *src, size_t count);

/* strcat - concatenates two strings
 * @dest: The string to be appended to
 * @src: The string to append to it
 *
 * The src string is appended to the dest string.
 *
 * Returns a pointer to the newly created dest string.
 */
char *strcat(char *dest, const char *src);

/* strcmp - compares two strings
 * @cs: One string
 * @ct: Another string
 *
 * Compares string cs with string ct.
 *
 * Returns an integer less than, equal to or greater than 0
 * if cs is found, respectively to be less than, equal to, or
 * greater than ct.
 */
int strcmp(const char *cs, const char *ct);

/* strncmp - compares a certain number of bytes in two strings
 * @cs: One string
 * @ct: Another string
 * @count: The maximum number of bytes to compare
 *
 * Compares the first "count" number of bytes in string cs with
 * string ct.
 *
 * Returns an integer less than, equal to or greater than 0
 * if cs is found, respectively to be less than, equal to, or
 * greater than ct.
 */
int strncmp(const char *cs, const char *ct, size_t count);

/* strcpy - copies a string
 * @dest: Where to copy the string to
 * @src: Where to copy the string from
 *
 * Copies the string pointed to by src to the buffer pointed
 * to by dest.
 *
 * Returns a pointer to the string dest.
 */
char *strcpy(char *dest, const char *src);

/* strncpy - copies a certain number of bytes from a string
 * @dest: Where to copy the string to
 * @src: Where to copy the string from
 * @count: The maximum number of bytes to copy
 *
 * Copies "count" number of bytes from the string pointed to
 * by src to the buffer pointed to by dest.
 *
 * Returns a pointer to the string dest.
 */
char *strncpy(char *dest, const char *src, size_t count);

/* strlen - calculates the length of string
 * @s: The string to be sized
 *
 * Calculates the length of a string excluding the terminating
 * null byte at the end of the string.
 *
 * Returns the number of characters that s contains.
 */
int strlen(const char *s);

/* printf - print formatted output
 *
 * Produce output according to the format passed to it.
 * Returns the number of characters printed
 * (excluding the null byte used to end output to strings).
 * If an output error is encountered, a negative value is returned.
 */
int printf(const char *fmt, ...);

int snprintf(char *str, size_t len, const char *fmt, ...);

int vsprintf(char *str, const char *fmt, va_list ap);

int vsnprintf(char *str, size_t len, const char *fmt, va_list ap);

int sprintf(char *buf, const char *fmt, ...);

void raise(void);

unsigned long strtoul(const char *cp, char **endp, unsigned int base);

#endif  /* __STRING_H_ */
