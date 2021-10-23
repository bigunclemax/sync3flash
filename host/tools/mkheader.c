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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>

/* CH Header defines */
#define CONFIG_USE_CH_SETTINGS_CONFIG	1
#define CONFIG_USE_CH_RAM_CONFIG	1

#include "ch-header.h"

void file2h(void* buff, unsigned buff_sz)
{
    const char *name = "aboot";

    printf("/* auto generated file */\n\n");
    printf("unsigned char %s_data[] = {", name);
    int i;
    for (i = 0; i < buff_sz; ++i ) {
        if (!(i % 16))
            printf("\n");

        printf("0x%02x,", ((u_int8_t *)buff)[i]);
    }

    printf("\n};\n");
    printf("unsigned %s_size = sizeof(%s_data);\n", name, name);
}

int main(int argc, char **argv)
{
    unsigned data_sz;
    void *data;
    int fd;
    struct stat s;
	unsigned x[2];

	if (argc < 3)
		return -1;

	x[1] = strtoul(argv[1], NULL, 0);

    fd = open(argv[2], O_RDONLY);
    if (fd < 0)
        return -1;

    if (fstat(fd, &s))
        return -1;

	x[0] = data_sz = s.st_size;

    data_sz += sizeof(basic_header);
    data = malloc(data_sz);

    memcpy(data, basic_header, sizeof(basic_header));

    ssize_t ret = read(fd, data + sizeof(basic_header), s.st_size);
    if (ret != s.st_size)
        return -1;

    close(fd);

    file2h(data, data_sz);

	return 0;
}
