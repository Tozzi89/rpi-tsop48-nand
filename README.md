It might be compiled on Raspberry Pi by command like
g++ rpi-tsop48-nand.cpp -o rpi-tsop48-nand

Modified for flashing MR33 NAND Spansion S34ML01G200TFV00 with old bootloader to make OpenWRT work again, see https://github.com/riptidewave93/LEDE-MR33/issues/13 and https://openwrt.org/toh/meraki/mr33

Flashing was possible with NAND still soldered into MR33, 3V3 supplied through UART header. MR33 will only boot with Raspi disconnected!

Short instructions:

Read full flash
rpi-tsop48-nand-b3 150 read_full 0 65536 mr33_full.dmp

Download ubootmr332012.bin https://github.com/riptidewave93/LEDE-MR33/issues/13#issuecomment-733942631

Add padding for flashing
dd if=mr33_full.dmp bs=$((0x738000)) count=1 > newflash-mr33.bin
dd if=ubootmr332012.bin bs=132k count=5 >> newflash-mr33.bin

Erase blocks
rpi-tsop48-nand-b3 150 erase_blocks 56 5

Flash "new" bootloader
rpi-tsop48-nand-b3 150 write_full 3584 320 newflash-mr33.bin

Then follow instructions in Openwrt Wiki.

Inspired by https://github.com/riptidewave93/LEDE-MR33/issues/13#issuecomment-802309974
