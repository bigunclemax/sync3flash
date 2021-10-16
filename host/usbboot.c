#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <memory.h>
#include <netinet/in.h>

#include "usb.h"

extern unsigned char aboot_data[];
extern unsigned aboot_size;

static int usage(void)
{
    fprintf(stderr, "\nsync3flash usage:\n\n");
    fprintf(stderr, "  sync3flash -m <MLO> -i <IFS>\n\n");

    return 0;
}

static void *load_file(const char *file, unsigned *sz)
{
    void *data;
    struct stat s;
    int fd;

    fd = open(file, O_RDONLY);
    if (fd < 0)
        return 0;

    if (fstat(fd, &s))
        goto fail;

    data = malloc(s.st_size);
    if (!data)
        goto fail;

    if (read(fd, data, s.st_size) != s.st_size) {
        free(data);
        goto fail;
    }

    close(fd);
    *sz = s.st_size;
    return data;

    fail:
    fprintf(stderr, "failed loading file\n");
    close(fd);
    return 0;
}

#define OFF_CHIP	0x04
#define OFF_ROM_REV	0x07
#define OFF_ID		0x0F
#define OFF_MPKH	0x26

/*
 * See TRM for OMAP4, OMAP5 ASIC ID Structure
 */
struct chip_info {
    uint16_t chip;
    char rom_rev;
    char IDEN[20];
    char MPKH[32];
    uint32_t crc0;
    uint32_t crc1;
    char proc_type[8];
};
static struct chip_info chip;

static char *usb_boot_read_chip_info(usb_handle *usb)
{
    static char proc_type[8];
    uint32_t msg_getid = 0xF0030003;
    uint8_t id[81];
    uint8_t *crc1;
    uint8_t gp_device_crc1[4] = {0, 0, 0, 0};
    int i;

    memset(id, 0xee, 81);
    fprintf(stderr,"reading ASIC ID\n");
    usb_write(usb, &msg_getid, sizeof(msg_getid));
    usb_read(usb, id, sizeof(id));

    memcpy(&chip.chip, &id[OFF_CHIP+0], 2);
    chip.chip = ntohs(chip.chip);
    chip.rom_rev = id[OFF_ROM_REV];
    memcpy(chip.IDEN, &id[OFF_ID], 20);
    memcpy(chip.MPKH, &id[OFF_MPKH], 32);
    chip.crc0 = ntohl(*(uint32_t *)&id[73]);
    chip.crc1 = ntohl(*(uint32_t *)&id[77]);

    fprintf(stderr,"CHIP: %02x%02x\n", id[OFF_CHIP+0], id[OFF_CHIP+1]);
    fprintf(stderr, "rom minor version: %02X\n", id[OFF_ROM_REV]);
    fprintf(stderr,"IDEN: ");
    for (i = 0; i < 20; i++)
        fprintf(stderr,"%02x", id[OFF_ID+i]);
    fprintf(stderr,"\nMPKH: ");
    for (i = 0; i < 32; i++)
        fprintf(stderr,"%02x", id[OFF_MPKH+i]);
    fprintf(stderr,"\nCRC0: %02x%02x%02x%02x\n",
            id[73], id[74], id[75], id[76]);
    fprintf(stderr,"CRC1: %02x%02x%02x%02x\n",
            id[77], id[78], id[79], id[80]);

    crc1 = &id[77];
    if (memcmp(crc1, &gp_device_crc1, 4 * sizeof(uint8_t))) {
        fprintf(stderr, "device is ED/HD (EMU/HS)\n");
        strcpy(proc_type, "EMU");
    } else {
        fprintf(stderr, "device is GP\n");
        strcpy(proc_type, "GP");
    }

    strcpy(chip.proc_type, proc_type);
    return proc_type;
}

static int match_omap_bootloader(usb_ifc_info *ifc)
{
    if (ifc->dev_vendor != 0x0451)
        return -1;
    if ((ifc->dev_product != 0xd010) && (ifc->dev_product != 0xd00f) &&
        (ifc->dev_product != 0xd011) &&  (ifc->dev_product != 0xd012))
        return -1;
    return 0;
}

static int usb_boot(usb_handle *usb, void *data, unsigned sz,
                    void *data2, unsigned sz2)
{
    void *msg = NULL;
    uint32_t msg_boot = 0xF0030002;
    uint32_t msg_size = sz;
    uint32_t param = 0;

    fprintf(stderr, "sending 2ndstage to target...\n");
    usb_write(usb, &msg_boot, sizeof(msg_boot));
    usb_write(usb, &aboot_size, sizeof(aboot_size));
    usb_write(usb, aboot_data, (int)aboot_size);

    fprintf(stderr,"waiting for 2ndstage response...\n");
    usb_read(usb, &msg_size, sizeof(msg_size));
    if (msg_size != 0xaabbccdd) {
        fprintf(stderr, "unexpected 2ndstage response\n");
        return -1;
    }

    for (int i = 0; i < 2; i++) {

        msg = i ? data2 : data;
        msg_size = i ? sz2 : sz;

        usb_write(usb, &msg_size, sizeof(msg_size));
        usb_read(usb, &param, sizeof(param));
        if (param != 0xaabbccdd) {
            fprintf(stderr, "unexpected 2ndstage response\n");
            return -1;
        }

        fprintf(stderr, "sending image to target...size "
                        "(%d-B/%d-KB/%d-MB)\n", msg_size,
                msg_size / 1024, msg_size / (1024 * 1024));

        if (msg_size)
            usb_write(usb, msg, (int) msg_size);
        else
            usb_write(usb, &msg_boot, sizeof(msg_boot));

    }

    return 0;
}

int main(int argc, char **argv) {

    int once = 1;
    usb_handle *usb = NULL;
    void *data = NULL, *data2 = NULL; //data1 - MLO, data2 - IFS
    unsigned sz = 0, sz2 = 0;

    int opt;
    while ((opt = getopt(argc, argv, "m:i:h")) != -1) {
        switch (opt) {
            case 'm':
                data = load_file(optarg, &sz);
                if (data == 0) {
                    fprintf(stderr, "cannot load MLO '%s'\n", optarg);
                    goto error;
                }
                break;
            case 'i':
                data2 = load_file(optarg, &sz2);
                if (data2 == 0) {
                    fprintf(stderr, "cannot load IFS '%s'\n", optarg);
                    goto error;
                }
                break;
            case 'h':
            default:
                goto error;
        }
    }

    if(!sz || !sz2) {
        goto error;
    }

    for (;;) {
        if (usb == NULL)
            usb = usb_open(match_omap_bootloader);

        if (usb) {
            usb_boot_read_chip_info(usb);
            return usb_boot(usb, data, sz, data2, sz2);
        }

        if (once) {
            once = 0;
            fprintf(stderr, "waiting for device...\n");
        }
        usleep(250);
    }

error:
    usage();
    return -1;
}