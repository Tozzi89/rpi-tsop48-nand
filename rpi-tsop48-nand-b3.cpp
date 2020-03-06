/*
    Raspberry Pi / 

    RasPS3 : GPIO NAND flasher
    (made out of "360-Clip based 8-bit NAND reader" by pharos)

    Copyright (C) 2016  littlebalup

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <math.h>

#include <sys/types.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/mman.h>

// #define DEBUG 1

#define PAGE_SIZE 2112 // (2K + 64)Byte
#define BLOCK_SIZE 135168 // (2K + 64)Byte
#define MAX_WAIT_READ_BUSY	1000000

#define PAGES_PER_BLOCK 64

/* For Raspberry B+ :*/
#define BCM2708_PERI_BASE	0x20000000
#define GPIO_BASE	 	(BCM2708_PERI_BASE + 0x200000)

/* For Raspberry 2B and 3B :*/
//#define BCM2736_PERI_BASE        0x3F000000
//#define GPIO_BASE                (BCM2736_PERI_BASE + 0x200000) /* GPIO controller */

#define N_WRITE_PROTECT		2 // pulled up by RPi, this is useful
#define N_READ_BUSY		3 // pulled up by RPi, this is also useful

// rest of GPIOs have been chose arbitrarily, with the only constraint of not using
// GPIO 14 (TXD)/GPIO 15 (RXD)/06 (GND) on P1. instead I use GND on P2 header, pin 8

// IMPORTANT: BE VERY CAREFUL TO CONNECT VCC TO P1-01 (3.3V) AND *NOT* P1-02 (5V) !!

#define N_WRITE_ENABLE 		27
#define ADDRESS_LATCH_ENABLE	4
#define COMMAND_LATCH_ENABLE	17
#define N_READ_ENABLE		18
#define N_CHIP_ENABLE		22

#define INLINE inline __attribute__((always_inline))

int data_to_gpio_map[8] = { 23, 24, 25, 8, 7, 10, 9, 11 }; // 23 is NAND IO0, etc.

volatile unsigned int *gpio;

INLINE void init_perfcounters (int32_t do_reset, int32_t enable_divider){
	// in general enable all counters (including cycle counter)
	int32_t value = 1;

	// peform reset:  
	if (do_reset) {
		value |= 2;     // reset all counters to zero.
		value |= 4;     // reset cycle counter to zero.
	} 

	if (enable_divider){
		value |= 8;     // enable "by 64" divider for CCNT.
	}

	value |= 16;

	// program the performance-counter control-register:
	asm volatile ("MCR p15, 0, %0, c9, c12, 0\t\n" :: "r"(value)); 

	// enable all counters:  
	asm volatile ("MCR p15, 0, %0, c9, c12, 1\t\n" :: "r"(0x8000000f));  

	// clear overflows:
	asm volatile ("MCR p15, 0, %0, c9, c12, 3\t\n" :: "r"(0x8000000f));
}

INLINE long ccnt_read(){
	long cc = 0;
	__asm__ volatile ("mrc p15, 0, %0, c9, c13, 0":"=r" (cc));
	return cc;
}


template<int X> class NopEmitter {
	public:
	INLINE void nop() {
		asm volatile("nop");
	}
};
template<int X> INLINE void Nop() {
	Nop<X - 1>();
	NopEmitter<X-1> nop;
	nop.nop();
}
template<> INLINE void Nop<1>() {
	NopEmitter<0> nop;
	nop.nop();
}

#define CPU_MHZ 700

constexpr int insns_per_nsec(int nsec){
	double div = (double)CPU_MHZ / 1000;
	return ceil(nsec / div);
}

#define NSLEEP(x) Nop<insns_per_nsec(x)>()


#define tCH() NSLEEP(5);
#define tWC() NSLEEP(25)
#define tWB() NSLEEP(100)

//#define CLE_1()

static INLINE int read_id(unsigned char id[5]);
static INLINE int read_pages(int first_page_number, int number_of_pages, char *outfile, int write_spare);
static INLINE int write_pages(int first_page_number, int number_of_pages, char *infile);
static INLINE int erase_blocks(int first_block_number, int number_of_blocks);

static INLINE void INP_GPIO(int g)
{
#ifdef DEBUG
	printf("setting direction of GPIO#%d to input\n", g);
#endif
	(*(gpio+((g)/10)) &= ~(7<<(((g)%10)*3)));
}

static INLINE void OUT_GPIO(int g)
{
	INP_GPIO(g);
#ifdef DEBUG
	printf("setting direction of GPIO#%d to output\n", g);
#endif
	*(gpio+((g)/10)) |= (1<<(((g)%10)*3));
}

static INLINE void GPIO_SET_1(int g)
{
#ifdef DEBUG
	printf("setting GPIO#%d to 1\n", g);
#endif
	*(gpio +  7)  = 1 << g;
}

static INLINE void GPIO_SET_0(int g)
{
#ifdef DEBUG
	printf("setting GPIO#%d to 0\n", g);
#endif
	*(gpio + 10)  = 1 << g;
}

static INLINE int GPIO_READ(int g)
{
	int x = (*(gpio + 13) & (1 << g)) >> g;
#ifdef DEBUG
	printf("GPIO#%d reads as %d\n", g, x);
#endif
	return x;
}

static INLINE void set_data_direction_in(void)
{
	int i;
#ifdef DEBUG
	printf("data direction => IN\n");
#endif
	for (i = 0; i < 8; i++)
		INP_GPIO(data_to_gpio_map[i]);
}

static INLINE void set_data_direction_out(void)
{
	int i;
#ifdef DEBUG
	printf("data direction => OUT\n");
#endif
	for (i = 0; i < 8; i++)
		OUT_GPIO(data_to_gpio_map[i]);
}

static INLINE int GPIO_DATA8_IN(void)
{
	int i, data;
	for (i = data = 0; i < 8; i++, data = data << 1) {
		data |= GPIO_READ(data_to_gpio_map[7 - i]);
	}
	data >>= 1;
#ifdef DEBUG
	printf("GPIO_DATA8_IN: data=%02x\n", data);
#endif
	return data;
}

static INLINE void GPIO_DATA8_OUT(int data)
{
	int i;
#ifdef DEBUG
	printf("GPIO_DATA8_OUT: data=%02x\n", data);
#endif
	for (i = 0; i < 8; i++, data >>= 1) {
		if (data & 1)
			GPIO_SET_1(data_to_gpio_map[i]);
		else
			GPIO_SET_0(data_to_gpio_map[i]);
	}
}

int delay = 1;
static INLINE void shortpause()
{
	int i;
	static volatile int dontcare = 0;
	for (i = 0; i < delay; i++) {
		dontcare++;
	}
}


#define DEBUG_STATUS_LED_GPIO 16

static INLINE void debug_status(bool value){
	if(value){
		GPIO_SET_0(DEBUG_STATUS_LED_GPIO);
	} else {
		GPIO_SET_1(DEBUG_STATUS_LED_GPIO);
	}
}

static INLINE void debug_blink(){
	GPIO_SET_1(DEBUG_STATUS_LED_GPIO);
	GPIO_SET_0(DEBUG_STATUS_LED_GPIO);
}

/*
INLINE void shortpause()
{
    struct timespec ts;
    ts.tv_sec = delay / 1000;
    ts.tv_nsec = (delay % 1000) * 1000000;
    nanosleep(&ts, NULL);
}*/

int main(int argc, char **argv)
{ 
	int mem_fd;

	printf("\nRasPS3 (b3)\na Raspberry GPIO flasher for PS3 NANDs, by littlebalup\n\n");

	if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC)) < 0) {
		perror("open /dev/mem, are you root?");
		return -1;
	}

	if(setpriority(PRIO_PROCESS, getpid(), -20) < 0){
		perror("renice failed, are you root?");
		return -1;
	}

	if ((gpio = (volatile unsigned int *) mmap((caddr_t) 0x13370000, 4096, PROT_READ|PROT_WRITE,
						MAP_SHARED|MAP_FIXED, mem_fd, GPIO_BASE)) == MAP_FAILED) {
		perror("mmap GPIO_BASE");
		close(mem_fd);
		return -1;
	}

	OUT_GPIO(DEBUG_STATUS_LED_GPIO);
	
	INP_GPIO(N_READ_BUSY);

	OUT_GPIO(N_WRITE_PROTECT);
	GPIO_SET_1(N_WRITE_PROTECT);

	OUT_GPIO(N_READ_ENABLE);
	GPIO_SET_1(N_READ_ENABLE);

	OUT_GPIO(N_WRITE_ENABLE);
	GPIO_SET_1(N_WRITE_ENABLE);

	OUT_GPIO(COMMAND_LATCH_ENABLE);
	GPIO_SET_0(COMMAND_LATCH_ENABLE);

	OUT_GPIO(ADDRESS_LATCH_ENABLE);
	GPIO_SET_0(ADDRESS_LATCH_ENABLE);

	OUT_GPIO(N_CHIP_ENABLE);
	GPIO_SET_0(N_CHIP_ENABLE);

	if (argc < 3) {
usage:
		GPIO_SET_1(N_CHIP_ENABLE);
		printf("usage: sudo %s <delay> <command> ...\n\n" \
		    " <delay> used to slow down operations (50 should work, increase if bad reads)\n\n" \
		    "Commands:\n" \
		    " read_id (no arguments)                        : read and decrypt chip ID\n" \
		    " read_full <page #> <# of pages> <output file> : read N pages including spare\n" \
		    " read_data <page #> <# of pages> <output file> : read N pages, discard spare\n" \
		    " write_full <page #> <# of pages> <input file> : write N pages, including spare\n" \
		    " write_data <page #> <# of pages> <input file> : write N pages, discard spare\n" \
		    " erase_blocks <block number> <# of blocks>     : erase N blocks\n\n" \
		    "Notes:\n" \
		    " This program assumes PAGE_SIZE == %d\n" \
		    " Run as root (sudo) required (for /dev/mem access)\n\n",
			argv[0], PAGE_SIZE);
		close(mem_fd);
		return -1;
	}

	//hide cursor
	// printf("\e[?25l");
	// fflush(stdout);

	delay = atoi(argv[1]);
	// if (delay < 20) {
	// 	printf("delay must be >= 20\n");
	// 	return -1;
	// }

	if (strcmp(argv[2], "read_id") == 0) {
		return read_id(NULL);
	}

	if (strcmp(argv[2], "read_full") == 0) {
		if (argc != 6) goto usage;
		if (atoi(argv[4]) <= 0) {
			printf("# of pages must be > 0\n");
			return -1;
		}
		return read_pages(atoi(argv[3]), atoi(argv[4]), argv[5], 1);
	}

	if (strcmp(argv[2], "read_data") == 0) {
		if (argc != 6) goto usage;
		if (atoi(argv[4]) <= 0) {
			printf("# of pages must be > 0\n");
			return -1;
		}
		return read_pages(atoi(argv[3]), atoi(argv[4]), argv[5], 0);
	}

	if (strcmp(argv[2], "write_full") == 0) {
		if (argc != 6) goto usage;
		if (atoi(argv[4]) <= 0) {
			printf("# of pages must be > 0\n");
			return -1;
		}
		return write_pages(atoi(argv[3]), atoi(argv[4]), argv[5]);
	}

	if (strcmp(argv[2], "erase_blocks") == 0) {
		if (argc != 5) goto usage;
		if (atoi(argv[4]) <= 0) {
			printf("# of blocks must be > 0\n");
			return -1;
		}
		return erase_blocks(atoi(argv[3]), atoi(argv[4]));
	}

	printf("unknown command '%s'\n", argv[2]);
	goto usage;
	return 0;
}

void error_msg(const char *msg)
{
	printf("%s\nBe sure to check wiring, and check that pressure is applied on both sides of 360 Clip\n" \
		"sometimes it is required to move slightly the 360 Clip in case of a false contact\n", msg);
}

void print_id(unsigned char id[5])
{
	unsigned int i, bit, page_size, ras_size, orga, plane_number;
	unsigned long block_size, plane_size, nand_size, nandras_size;
	char maker[16], device[16], serial_access[20];
	unsigned *thirdbits = (unsigned*)malloc(sizeof(unsigned) * 8);
	unsigned *fourthbits = (unsigned*)malloc(sizeof(unsigned) * 8);
	unsigned *fifthbits = (unsigned*)malloc(sizeof(unsigned) * 8);

	printf("Raw ID data: ");
	for (i = 0; i < 5; i++)
		printf("0x%02X ", id[i]);
	printf("\n");

 	switch(id[0]) {
 		case 0xEC: {
 			strcpy(maker, "Samsung");
 			switch(id[1]) {
 				case 0xA1: strcpy(device, "K9F1G08R0A"); break;
 				case 0xD5: strcpy(device, "K9GAG08U0M"); break;
 				case 0xF1: strcpy(device, "K9F1G08U0A/B"); break;
 				default: strcpy(device, "unknown");
 			}
 			break;
 		}
 		case 0xAD: {
 			strcpy(maker, "Hynix");
 			switch(id[1]) {
 				case 0x73: strcpy(device, "HY27US08281A"); break;
 				case 0xD7: strcpy(device, "H27UBG8T2A"); break;
 				case 0xDA: strcpy(device, "HY27UF082G2B"); break;
 				case 0xDC: strcpy(device, "H27U4G8F2D"); break;
 				default: strcpy(device, "unknown");
 			}
 			break;
 		}
 		default: strcpy(maker, "unknown"); strcpy(device, "unknown");
 	}

/* all sizes in bytes */
	for(bit = 0; bit < 8; ++bit)
		thirdbits[bit] = (id[2] >> bit) & 1;

	for(bit = 0; bit < 8; ++bit)
		fourthbits[bit] = (id[3] >> bit) & 1;
	switch(fourthbits[1] * 10 + fourthbits[0]) {
		case 00: page_size = 1024; break;
		case 01: page_size = 2048; break;
		case 10: page_size = 4096; break;
		case 11: page_size = 8192; break;
	}
	switch(fourthbits[5] * 10 + fourthbits[4]) {
		case 00: block_size = 64 * 1024; break;
		case 01: block_size = 128 * 1024; break;
		case 10: block_size = 256 * 1024; break;
		case 11: block_size = 521 * 1024; break;
	}
	switch(fourthbits[2]) {
		case 0: ras_size = 8; break; // for 512 bytes
		case 1: ras_size = 16; break; // for 512 bytes
	}
	switch(fourthbits[6]) {
		case 0: orga = 8; break; // bits
		case 1: orga = 16; break; // bits
	}
	switch(fourthbits[7] * 10 + fourthbits[3]) {
		case 00: strcpy(serial_access, "50ns/30ns minimum"); break;
		case 10: strcpy(serial_access, "25ns minimum"); break;
		case 01: strcpy(serial_access, "unknown (reserved)"); break;
		case 11: strcpy(serial_access, "unknown (reserved)"); break;
	}

	for(bit = 0; bit < 8; ++bit)
		fifthbits[bit] = (id[4] >> bit) & 1;
	switch(fifthbits[3] * 10 + fifthbits[2]) {
		case 00: plane_number = 1; break;
		case 01: plane_number = 2; break;
		case 10: plane_number = 4; break;
		case 11: plane_number = 8; break;
	}
	switch(fifthbits[6] * 100 + fifthbits[5] * 10 + fifthbits[4]) {
		case 000: plane_size = 64 / 8 * 1024 * 1024; break; // 64 megabits
		case 001: plane_size = 128 / 8 * 1024 * 1024; break; // 128 megabits
		case 010: plane_size = 256 / 8 * 1024 * 1024; break; // 256 megabits
		case 011: plane_size = 512 / 8 * 1024 * 1024; break; // 512 megabits
		case 100: plane_size = 1024 / 8 * 1024 * 1024; break; // 1 gigabit
		case 101: plane_size = 2048 / 8 * 1024 * 1024; break; // 2 gigabits
		case 110: plane_size = 4096 / 8 * 1024 * 1024; break; // 4 gigabits
		case 111: plane_size = 8192 / 8 * 1024 * 1024; break; // 8 gigabits
	}

	nand_size = plane_number * plane_size;
	nandras_size = nand_size + ras_size * nand_size / 512;


	printf("\n");
	printf("NAND manufacturer:  %s (0x%02X)\n", maker, id[0]);
	printf("NAND model:         %s (0x%02X)\n", device, id[1]);
	printf("\n");

	printf("              I/O|7|6|5|4|3|2|1|0|\n");
	printf("3rd ID data:     |");
	for(bit = 8; bit--;)
        printf("%u|", thirdbits[bit]);
    printf(" (0x%02X)\n", id[2]);
	printf("4th ID data:     |");
	for(bit = 8; bit--;)
        printf("%u|", fourthbits[bit]);
    printf(" (0x%02X)\n", id[3]);
	printf("5th ID data:     |");
	for(bit = 8; bit--;)
        printf("%u|", fifthbits[bit]);
    printf(" (0x%02X)\n", id[4]);

	printf("\n");
	printf("Page size:          %d bytes\n", page_size);
	printf("Block size:         %zd bytes\n", block_size);
	printf("RAS (/512 bytes):   %d bytes\n", ras_size);
	// printf("RAS (per page):  %d bytes\n", ras_size * page_size / 512);
	// printf("RAS (per block): %d bytes\n", ras_size * block_size / 512);
	printf("Organisation:       %d bit\n", orga);
	printf("Serial access:      %s\n", serial_access);
	printf("Number of planes:   %u\n", plane_number);
	printf("Plane size:         %zd bytes\n", plane_size);
	printf("\n");
	printf("NAND size:          %zd MB\n", nand_size / (1024 * 1024));
	printf("NAND size + RAS:    %zd MB\n", nandras_size / (1024 * 1024));
	printf("Number of blocks:   %zd\n", nand_size / block_size);
	printf("Number of pages:    %zd\n", nand_size / page_size);
}

static int read_id(unsigned char id[5])
{
	int i;
	unsigned char buf[5];

	GPIO_SET_1(COMMAND_LATCH_ENABLE); shortpause();
	GPIO_SET_0(N_WRITE_ENABLE);
	set_data_direction_out(); GPIO_DATA8_OUT(0x90); // Read ID byte 1
	shortpause();
	GPIO_SET_1(N_WRITE_ENABLE);
	shortpause();set_data_direction_in();
	GPIO_SET_0(COMMAND_LATCH_ENABLE);
	shortpause();

	GPIO_SET_1(ADDRESS_LATCH_ENABLE);
	GPIO_SET_0(N_WRITE_ENABLE);
	set_data_direction_out(); GPIO_DATA8_OUT(0x00); // Read ID byte 2
	shortpause();
	GPIO_SET_1(N_WRITE_ENABLE);
	shortpause(); set_data_direction_in();
	GPIO_SET_0(ADDRESS_LATCH_ENABLE);
	shortpause();

	for (i = 0; i < 5; i++) {
		GPIO_SET_0(N_READ_ENABLE);
		shortpause();
		buf[i] = GPIO_DATA8_IN(); //
		GPIO_SET_1(N_READ_ENABLE);
		shortpause();
	}
	if (id != NULL)
		memcpy(id, buf, 5);
	else
		print_id(buf);
	if (buf[0] == buf[1] && buf[1] == buf[2] && buf[2] == buf[3] && buf[3] == buf[4]) {
		error_msg("all five ID bytes are identical, this is not normal");
		return -1;
	}
	return 0;
}

static INLINE int page_to_address(int page, int address_byte_index)
{
	switch(address_byte_index) {
	//3rd: A12 A13 A14 A15 A16 A17 A18 A19
	case 2:
		return page & 0xff;
	//4th: A20 A21 A22 A23 A24 A25 A26 A27
	case 3:
		return (page >>  8) & 0xff;
	//5th: A28 A29 A30 L (1) L (1) L (1) L (1) L (1)
	case 4:
		return (page >> 16) & 0xff;
	default:
		return 0;
	}
}

static INLINE int send_read_command(int page)
{
	int i;

	set_data_direction_out();

	GPIO_SET_1(COMMAND_LATCH_ENABLE);
	shortpause();
	GPIO_SET_0(N_WRITE_ENABLE);
	shortpause();
	GPIO_DATA8_OUT(0x00);
	shortpause();
	GPIO_SET_1(N_WRITE_ENABLE);
	shortpause();
	GPIO_SET_0(COMMAND_LATCH_ENABLE);
	shortpause();

	GPIO_SET_1(ADDRESS_LATCH_ENABLE);
	for (i = 0; i < 5; i++) {
		GPIO_SET_0(N_WRITE_ENABLE);
		shortpause();


		// if (i < 2) {
		// 	printf("Col Add%d = %d\n", i + 1, page_to_address(page, i));
		// }
		// else {
		// 	printf("Row Add%d = %d\n", i - 1, page_to_address(page, i));
		// }

		GPIO_DATA8_OUT(page_to_address(page, i));
		shortpause();
		GPIO_SET_1(N_WRITE_ENABLE);
		shortpause();
	}
	GPIO_SET_0(ADDRESS_LATCH_ENABLE);
	shortpause();

	GPIO_SET_1(COMMAND_LATCH_ENABLE);
	shortpause();
	GPIO_SET_0(N_WRITE_ENABLE);
	shortpause();
	GPIO_DATA8_OUT(0x30);
	shortpause();
	GPIO_SET_1(N_WRITE_ENABLE);
	shortpause();
	GPIO_SET_0(COMMAND_LATCH_ENABLE);
	shortpause();

	return 0;
}

static INLINE int send_write_command(int page, unsigned char data[PAGE_SIZE])
{
	int i;

	set_data_direction_out();

	GPIO_SET_1(COMMAND_LATCH_ENABLE);
	shortpause();
	GPIO_SET_0(N_WRITE_ENABLE);
	shortpause();
	GPIO_DATA8_OUT(0x80);
	shortpause();
	GPIO_SET_1(N_WRITE_ENABLE);
	shortpause();
	GPIO_SET_0(COMMAND_LATCH_ENABLE);
	shortpause();

	GPIO_SET_1(ADDRESS_LATCH_ENABLE);
	for (i = 0; i < 5; i++) {
		GPIO_SET_0(N_WRITE_ENABLE);

		// if (i < 2) {
		// 	printf("Col Add%d = %d\n", i + 1, page_to_address(page, i));
		// }
		// else {
		// 	printf("Row Add%d = %d\n", i - 1, page_to_address(page, i));
		// }

		GPIO_DATA8_OUT(page_to_address(page, i));
		shortpause();
		GPIO_SET_1(N_WRITE_ENABLE);
		shortpause();
	}
	GPIO_SET_0(ADDRESS_LATCH_ENABLE);
	shortpause();

	for (i = 0; i < PAGE_SIZE; i++) {
		GPIO_SET_0(N_WRITE_ENABLE);
		shortpause();
		GPIO_DATA8_OUT(data[i]); //
		shortpause();
		GPIO_SET_1(N_WRITE_ENABLE);
		shortpause();
	}

	GPIO_SET_1(COMMAND_LATCH_ENABLE);
	shortpause(); GPIO_SET_0(N_WRITE_ENABLE);
	GPIO_DATA8_OUT(0x10);
	shortpause(); GPIO_SET_1(N_WRITE_ENABLE);
	shortpause(); GPIO_SET_0(COMMAND_LATCH_ENABLE);
	shortpause();

	return 0;
}

static INLINE void write_cmd(uint8_t cmd){
	GPIO_SET_1(COMMAND_LATCH_ENABLE); shortpause();
	GPIO_SET_0(N_WRITE_ENABLE);	shortpause();

	GPIO_DATA8_OUT(cmd); shortpause();
	
	GPIO_SET_1(N_WRITE_ENABLE);	shortpause();
	GPIO_SET_0(COMMAND_LATCH_ENABLE); shortpause();
}

static INLINE int send_eraseblock_command(int block)
{
	int i;

	set_data_direction_out();

	GPIO_SET_1(COMMAND_LATCH_ENABLE); shortpause();
	GPIO_SET_0(N_WRITE_ENABLE);	shortpause();
	GPIO_DATA8_OUT(0x60); shortpause();
	GPIO_SET_1(N_WRITE_ENABLE);	shortpause();
	GPIO_SET_0(COMMAND_LATCH_ENABLE); shortpause();

	GPIO_SET_1(ADDRESS_LATCH_ENABLE);
	for (i = 2; i < 5; i++) {
		GPIO_SET_0(N_WRITE_ENABLE);
		shortpause();

		// printf("Row Add%d = %d\n", i - 1, page_to_address(block, i));

		GPIO_DATA8_OUT(page_to_address(block, i));
		shortpause();
		GPIO_SET_1(N_WRITE_ENABLE);
		shortpause();
	}
	GPIO_SET_0(ADDRESS_LATCH_ENABLE); shortpause();

	GPIO_SET_1(COMMAND_LATCH_ENABLE); shortpause();
	GPIO_SET_0(N_WRITE_ENABLE); shortpause();
	GPIO_DATA8_OUT(0xD0); shortpause();
	GPIO_SET_1(N_WRITE_ENABLE); shortpause();
	GPIO_SET_0(COMMAND_LATCH_ENABLE); shortpause();

	return 0;
}

static INLINE int read_status()
{
	int data;
	set_data_direction_out();

	GPIO_SET_1(COMMAND_LATCH_ENABLE); shortpause();
	GPIO_SET_0(N_WRITE_ENABLE); shortpause();
	GPIO_DATA8_OUT(0x70); shortpause();
	GPIO_SET_1(N_WRITE_ENABLE);	shortpause();
	GPIO_SET_0(COMMAND_LATCH_ENABLE); shortpause();

	set_data_direction_in();

	GPIO_SET_0(N_READ_ENABLE); shortpause();
	data = GPIO_DATA8_IN(); shortpause();
	GPIO_SET_1(N_READ_ENABLE); shortpause();

	// printf("Status data = %d\n", data);

	return data & 1; // I/O0=0 success , I/O0=1 error
}


static int read_pages(int first_page_number, int number_of_pages, char *outfile, int write_spare)
{
	int page, page_no, block_no, page_nbr, percent, i, n, retry_count;
	unsigned char id[5], id2[5];
	unsigned char buf[PAGE_SIZE * 2];
	FILE *badlog, *f = fopen(outfile, "w+");
	if (f == NULL) {
		perror("fopen output file");
		return -1;
	}
	if ((badlog = fopen("bad.log", "w+")) == NULL) {
		perror("fopen bad.log");
		return -1;
	}
	if (GPIO_READ(N_READ_BUSY) == 0) {
		error_msg("N_READ_BUSY should be 1 (pulled up), but reads as 0. make sure the NAND is powered on");
		return -1;
	}

	if (read_id(id) < 0)
		return -1;
	print_id(id);
	printf("if this ID is incorrect, press Ctrl-C NOW to abort (3s timeout)\n");
	sleep(3);

	printf("\nStart reading...\n");
	clock_t start = clock();


	for (retry_count = 0, page = first_page_number*2; page < (first_page_number + number_of_pages)*2; page++) {

	  retry_all:
		page_no = page >> 1;

		//printf("page = %d, n = %d\n",page, n);

		if (page % 2 == 0 && retry_count == 0) {
			// page_no = page / 2;
			page_nbr = page_no - first_page_number + 1;
			percent = (100 * page_nbr) / number_of_pages;
			block_no = page_no / PAGES_PER_BLOCK;
			printf("Reading page n° %d in block n° %d (page %d of %d), %d%%\r", page_no, block_no, page_nbr, number_of_pages, percent);
			fflush(stdout);
		}
		// else {
		// 	printf("Reading the page again to ensure correct operation\n");
		// }

	  retry:
		read_id(id2);
		if (memcmp(id, id2, 5) != 0) {
			printf("\nNAND ID has changed! retrying");
			goto retry;
		}
		send_read_command(page_no);
		//for (i = 0; i < MAX_WAIT_READ_BUSY; i++) {
		//	if (GPIO_READ(N_READ_BUSY) == 0)
		//		break;
		//}
		while (GPIO_READ(N_READ_BUSY) == 0) {
			shortpause();
		}
		// if (i == MAX_WAIT_READ_BUSY) {
		// 	// #ifdef DEBUG
		// 		printf("N_READ_BUSY was not brought to 0 by NAND in time, retrying\n");
		// 	// #endif
		// 	goto retry;
		// }
		set_data_direction_in();
		// for (i = 0; i < MAX_WAIT_READ_BUSY; i++) {
		// 	if (GPIO_READ(N_READ_BUSY) == 1)
		// 		break;
		// }
		// if (i == MAX_WAIT_READ_BUSY) {
		// 	// #ifdef DEBUG
		// 		printf("N_READ_BUSY was not brought to 1 by NAND in time, retrying\n");
		// 	// #endif
		// 	goto retry;
		// }
		n = PAGE_SIZE*(page & 1);
		for (i = 0; i < PAGE_SIZE; i++) {
			GPIO_SET_0(N_READ_ENABLE);
			shortpause();
			buf[i + n] = GPIO_DATA8_IN(); //
			GPIO_SET_1(N_READ_ENABLE);
			shortpause();
		}
		if (!n) // read the page again to ensure correct operation, bit 0 in page used for this purpose
			// printf("RE LOOP    | page = %d, n = %d\n",page, n);
			// printf("Reading the page n° %d again to ensure correct operation\n", page_no);
			continue;

		if (memcmp(buf, buf + PAGE_SIZE, PAGE_SIZE) != 0) {
			if (retry_count == 0) printf("\n");
			if (retry_count < 5) {
				printf("Page failed to read correctly! retrying\n");
				retry_count++;
				page = page & ~1;
				goto retry_all;
			}
			printf("Too many retries. Perhaps bad block?\n");
			fprintf(badlog, "Page %d seems to be bad\n", page_no);
		}
		if (write_spare) {
			if (fwrite(buf, PAGE_SIZE, 1, f) != 1) {
				perror("fwrite");
				return -1;
			}
		}
		else {
			if (fwrite(buf, 512 * (PAGE_SIZE / 512), 1, f) != 1) {
				perror("fwrite");
				return -1;
			}
		}
		retry_count = 0;
	}
	fcloseall();
	clock_t end = clock();
	printf("\n\nReading done in %f seconds\n", (float)(end - start) / CLOCKS_PER_SEC);

	//show cursor
	// printf("\e[?25h");
	// fflush(stdout) ;
	return 0;
}


/*int read_pages(int first_page_number, int number_of_pages, char *outfile, int write_spare)
{
	int page, block_no, page_nbr, percent, i;
	unsigned char buf[PAGE_SIZE], id[5], id2[5];;
	FILE *f = fopen(outfile, "w+");
	if (f == NULL) {
		perror("fopen output file");
		return -1;
	}
	if (GPIO_READ(N_READ_BUSY) == 0) {
		error_msg("N_READ_BUSY should be 1 (pulled up), but reads as 0. make sure the NAND is powered on");
		return -1;
	}

	if (read_id(id) < 0)
		return -1;
	print_id(id);
	printf("if this ID is incorrect, press Ctrl-C NOW to abort (3s timeout)\n");
	sleep(3);

	printf("\nStart reading...\n\n");
	clock_t start = clock();


	for (page = first_page_number; page < first_page_number + number_of_pages; page++) {

		// printf("page = %d, n = %d\n",page, n);

		// page_nbr = page - first_page_number + 1;
		// percent = (100 * page_nbr) / number_of_pages;
		// block_no = page / 64;
		// printf("Reading page n° %d in block n° %d (page %d of %d), %d%%\n", page, block_no, page_nbr, number_of_pages, percent);
		printf("\nReading page n° %d\n", page);

		send_read_command(page);
		while (GPIO_READ(N_READ_BUSY) == 0) {
			// printf("Busy\n");
			shortpause();
		}
		set_data_direction_in();
		for (i = 0; i < PAGE_SIZE; i++) {
			GPIO_SET_0(N_READ_ENABLE);
			shortpause();
			buf[i] = GPIO_DATA8_IN(); //
			shortpause();
			GPIO_SET_1(N_READ_ENABLE);
			shortpause();
		}
		if (write_spare) {
			if (fwrite(buf, PAGE_SIZE, 1, f) != 1) {
				perror("fwrite");
				return -1;
			}
		}
		else {
			if (fwrite(buf, 512 * (PAGE_SIZE / 512), 1, f) != 1) {
				perror("fwrite");
				return -1;
			}
		}
	}
	fcloseall();
	clock_t end = clock();
	printf("\nReading done in %f seconds\n", (float)(end - start) / CLOCKS_PER_SEC);
}
*/
static INLINE int write_pages(int first_page_number, int number_of_pages, char *infile)
{
	int page, block_no, page_nbr, percent, retry_count;
	unsigned char buf[PAGE_SIZE], id[5], id2[5];;

	if (read_id(id) < 0)
		return -1;
	print_id(id);
	printf("if this ID is incorrect, press Ctrl-C NOW to abort (3s timeout)\n");
	sleep(3);

	printf("\nStart writing...\n");
	clock_t start = clock();


	FILE *f = fopen(infile, "rb");
	if (f == NULL) {
		perror("fopen input file");
		return -1;
	}

	// printf("first_page_number = %d\n", first_page_number);
	// printf("number of pages = %d\n", number_of_pages);


	for (retry_count = 0, page = first_page_number; page < first_page_number + number_of_pages; page++) {

	  retry_all:

		if (retry_count == 0) {
			// page_no = page / 2;
			page_nbr = page - first_page_number + 1;
			percent = (100 * page_nbr) / number_of_pages;
			block_no = page / PAGES_PER_BLOCK;
			printf("Writing page n° %d in block n° %d (page %d of %d), %d%%\r", page, block_no, page_nbr, number_of_pages, percent);
			fflush(stdout);
		}

		fseek(f, page * PAGE_SIZE, SEEK_SET);
		fread(buf, PAGE_SIZE, 1, f);

		// printf("\nwriting page n°%d\n", page);

	  retry:
		read_id(id2);
		if (memcmp(id, id2, 5) != 0) {
			printf("\nNAND ID has changed! retrying");
			goto retry;
		}

		send_write_command(page, buf);
		while (GPIO_READ(N_READ_BUSY) == 0) {
			// printf("Busy\n");
			shortpause();
		}
		// read_status();
		if (read_status()) {
			if (retry_count == 0) printf("\n");
			if (retry_count < 5) {
				printf("Failed to write page correctly! retrying\n");
				retry_count++;
				goto retry_all;
			}
			printf("Too many retries. Perhaps bad block?\n");
			// retry_count = 0;
		}
		retry_count = 0;
	}





	fcloseall();
	clock_t end = clock();
	printf("\nWrite done in %f seconds\n", (float)(end - start) / CLOCKS_PER_SEC);
	return 0;
}

static INLINE int erase_blocks(int first_block_number, int number_of_blocks)
{
	int block, block_nbr, percent, retry_count;
	unsigned char id[5], id2[5];

	if (read_id(id) < 0)
		return -1;
	print_id(id);
	printf("if this ID is incorrect, press Ctrl-C NOW to abort (3s timeout)\n");
	sleep(3);

	printf("\nStart erasing...\n");
	clock_t start = clock();

	for (retry_count = 0, block = first_block_number; block < (first_block_number + number_of_blocks); block++) {

	  retry_all:
			
		block_nbr = block - first_block_number + 1;
		percent = (100 * block_nbr) / number_of_blocks;

		if (retry_count == 0) {
			printf("Erasing block n° %d at adress 0x%02X (block %d of %d), %d%%\r", block, block * BLOCK_SIZE, block_nbr, number_of_blocks, percent);
			fflush(stdout);
			// printf("Block address : %d (0x%02X)\n", block * BLOCK_SIZE, block * BLOCK_SIZE);
		}

	  retry:
		read_id(id2);
		if (memcmp(id, id2, 5) != 0) {
			printf("\nNAND ID has changed! retrying");
			goto retry;
		}

		send_eraseblock_command(block * PAGES_PER_BLOCK);
		while (GPIO_READ(N_READ_BUSY) == 0) {
			// printf("Busy\n");
			//shortpause();
		}

		if (read_status()) {
			if (retry_count == 0) printf("\n");
			if (retry_count < 5) {
				printf("Failed to erase block correctly! retrying\n");
				retry_count++;
				goto retry_all;
			}
			printf("Too many retries. Perhaps bad block?\n");
			// retry_count = 0;
		}
		retry_count = 0;
	}

	clock_t end = clock();
	printf("\nErasing done in %f seconds\n", (float)(end - start) / CLOCKS_PER_SEC);
	return 0;
}