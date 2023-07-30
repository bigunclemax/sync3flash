# Sync3flash
allows to flash eMMC on Ford Sync3 APIM module via USB cable directly connected to PC.

This tool is suitable for bring up bricked APIM modules or for performing clean/factory installation on it.

**Requirements**:
- PC with Linux.  
If you don't have it than you can use an any Linux livecd on a flash drive, for example: `puppy linux`.  
**Working on virtual machine is not guaranteed.** (But you can try, see method at the end :)
- `MLO` and `QNX-IFS-REFORMAT` files from REFORMAT package
- miniUSB cable
- `sync3flash` (this) tool

## Main idea


## Let's start:

0. Disassembly Sync 3 APIM module to get physical access to main board with Omap SoC.
1. Connect Sync module directly to PC through miniUSB cable.  
(**Don't try to connect PC through Sync 3 USB hub. This will not work**)
2. Start waking up Sync module by sending CAN ignition packages.
3. Run `sync3flash` tool:  
`sudo ./sync3flash -i QNX-IFS-REFORMAT -m MLO`
4. Short [contacts](https://gist.githubusercontent.com/bigunclemax/7117d6f506200224156e093f7d5a16be/raw/cfeeddb606323f26ac0c2957e0f38127a26cbf71/v31.jpg) circled in red
5. Enable `12V` power supply to turn on Sync module.

If everything goes smooth, you should see output like that:
```
user@PC:/$ sudo ./sync3flash -m MLO -i QNX-IFS-REFORMAT 
waiting for device...
reading ASIC ID
CHIP: 5430
rom minor version: 02
IDEN: 0000000000000000000000000000000000000000
MPKH: 0000000000000000000000000000000000000000000000000000000000000000
CRC0: 071a9a31
CRC1: 00000000
device is GP
sending 2ndstage to target...
waiting for 2ndstage response...
sending image to target...size (22696-B/22-KB/0-MB)
sending image to target...size (9279956-B/9062-KB/8-MB)
```

After 30 seconds Sync 3 APIM will reboot and reformat will start.  
That's it 😎

## Troubleshooting

- **No response from bootloader on `1Gb RAM` board version**  
If `sync3flash` stuck at `waiting for 2ndstage response...` make sure that you use
`sync3flash` version `1.1` or later. (Many thanks to **Tadeusz** for this fix 🙏)

---  

Using `sync3flash` on Virtual Box with Linux.  
**This method is from OmapWiki and has not been tested.**

```
NOTE:
You can use virtualized Linux on Windows to make it work. Below is example for VirtualBox:
Settings -> USB -> Add Empty Filter
Name: OMAP5 sEVM
Vendor Id: 0451
Product Id: d011

Then sudo ./usbboot -f and do as usual with your board:
* first time should fail as drivers will be installed
* next time will work OK.

Note: We don't support this, so if you have any issues please direct it to the omap5 mailing list. 
```

---

[Here is](https://gist.github.com/bigunclemax/c566ecfc2f6d92e76e68446e46bdd944#file-sync3usbbootreformat_en-md) full article with detailed explanation of this method.
