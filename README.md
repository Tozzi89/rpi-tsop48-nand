It might be compiled on Raspberry Pi by command like
`g++ rpi-tsop48-nand.cpp -o rpi-tsop48-nand`

Tested with Raspi 1 B V1 with 26 pin GPIO. For newer models the GPIO mapping needs to be changed.

Modified for flashing MR33 NAND Spansion S34ML01G200TFV00 with old bootloader to make OpenWRT work again, see https://github.com/riptidewave93/LEDE-MR33/issues/13 and https://openwrt.org/toh/meraki/mr33

Flashing was possible with NAND still soldered into MR33, 3V3 supplied through UART header. MR33 will only boot with Raspi disconnected!

Download ubootmr332012.bin from https://github.com/riptidewave93/LEDE-MR33/issues/13#issuecomment-733942631

```
Short instructions:

Read full flash
rpi-tsop48-nand 150 read_full 0 65536 mr33_full.dmp

Add padding for flashing
dd if=mr33_full.dmp bs=$((0x738000)) count=1 > newflash-mr33.bin
dd if=ubootmr332012.bin bs=132k count=5 >> newflash-mr33.bin

Erase blocks
rpi-tsop48-nand 150 erase_blocks 56 5

Flash "new" bootloader
rpi-tsop48-nand 150 write_full 3584 320 newflash-mr33.bin
```

Then follow the instructions in the OpenWRT Wiki. 22.03.3 is working and can be installed directly. 

Inspired by https://github.com/riptidewave93/LEDE-MR33/issues/13#issuecomment-802309974
