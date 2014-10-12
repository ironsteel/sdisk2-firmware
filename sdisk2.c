/*------------------------------------------------------------

	DISK II Emulator Farmware (1 of 2) for ATMEGA328P

	version 2.3	2013.10.30 by Koichi Nishida
	require version 2.0 hardware

	Copyright 2013 Koichi NISHIDA
	email to Koichi NISHIDA: tulip-house@msf.biglobe.ne.jp
	
------------------------------------------------------------*/

/*
hardware information:

use ATMEGA328P AVR.
connect 27MHz (overclock...) crystal to the AVR.
supply 3.3V power.

fuse setting : LOW 11011110

connection:
	<connection to SD card using spi>
	B2: CS (SS)
	B3: DI (MOSI)
	B4: DO (MISO)
	B5: SCLK (SCK)
	D6: LOCK
	
	<connections to APPLE II disk IF>
	B1: WRITE
	C0: PHASE-0
	C1: PHASE-1
	C2: PHASE-2
	C3: PHASE-3
	C4: READ PULSE (through 74HC125 3state)
	D2(INT0): WRITE REQUEST (10K ohm pull up for open collector)
	D3: EJECT
	D4: WRITE PROTECT (through 74HC125 3state)
	D7: DRIVE ENABLE
	
	<others>
	D5: LED
	C5: connect to C6 (RESET) for software reset
	
	Note that the enable input of the 3state buffer 74HC125,
	should be connected with DRIVE ENABLE.
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#define nop() __asm__ __volatile__ ("nop")
#define WAIT 1

// write buffer number
#define BUF_NUM 5

// fat buffer size
#define FAT_DSK_SIZE 18
#define FAT_NIC_SIZE 35

// for selecting SD card
#define SD_CS	2	// PB2 SS	CS

#define MAX_FILES_PER_DIR 512

// C prototypes

// ===== low level SD card functions =====

// cancel read
void cancelRead(void);
// write a byte data to the SD card
void writeByte(unsigned char c);
// read data from the SD card
unsigned char readByte(void);
// wait until finish a command
void waitFinish(void);
// issue SD card command without getting response
void cmd_(unsigned char cmd, unsigned long adr);
// issue SD card command  and wait normal response
void cmd(unsigned char cmd, unsigned long adr);
// get command response from the SD card
unsigned char getResp(void);
// issue command 17 and get ready for reading
void cmd17(unsigned long adr);
// find a file extension

// ===== file manipulate functions =====

int findExt(char *str, unsigned char *protect, unsigned char *name, uint16_t file_index_offset);
// prepare the FAT table on memory
void prepareFat(int i, unsigned short *fat, unsigned short len,
	unsigned char fatNum, unsigned char fatElemNum);
// memory copy	
void memcp(unsigned char *dst, unsigned char *src, const unsigned short len);
// duplicate FAT for FAT16
void duplicateFat(void);
// write to the SD cart one by one
void writeSD(unsigned long adr, unsigned char *data, unsigned short len);
// create a NIC image file
int createNic(unsigned char *name);
// translate a NIC image into a DSK image
void nic2Dsk(void);
// translate a DSK image into a NIC image
void dsk2Nic(void);

// ===== SDISK II main functions =====

// initialization SD
int SDinit(void);
// write data back to a NIC image
// set write pointer and write back if need
void buffering(void);
void writeBack(void);
void writeBackSub(unsigned char bn, unsigned char sc, unsigned char track);
// buffer clear
void buffClear(void);

// assembler functions
void wait5(unsigned short time);

// ===== data area =====

// SD card information
unsigned long bpbAddr, rootAddr;
unsigned long fatAddr;									// the beginning of FAT
unsigned char sectorsPerCluster, sectorsPerCluster2;	// sectors per cluster
unsigned short sectorsPerFat;	
unsigned long userAddr;									// the beginning of user data
// unsigned short fatDsk[FAT_DSK_SIZE];					// use writeData instead
unsigned short fatNic[FAT_NIC_SIZE];
unsigned char prevFatNumDsk, prevFatNumNic;
unsigned short nicDir, dskDir;

// DISK II status
volatile unsigned char ph_track;		// 0 - 139
unsigned char sector;					// 0 - 15
unsigned short bitbyte;					// 0 - (8*512-1)
unsigned char prepare;
unsigned char readPulse;
unsigned char magState;
unsigned char protect;
unsigned char formatting;
const unsigned char volume = 0xfe;
// track number is stored on eeprom.
#define EEP_PH_TRACK (uint8_t *)0x01

/* Track the current selected disk image on eeprom */
#define EEP_CURRENT_DISK_IMAGE (uint8_t *) 0x02

// write data buffer
unsigned char writeData[BUF_NUM][350];
unsigned char sectors[BUF_NUM], tracks[BUF_NUM];
unsigned char buffNum;
unsigned char *writePtr;
unsigned char doBuffering;

/* nic file index */
volatile  uint16_t current_disk_image = 0;

// a table for head stepper moter movement 
PROGMEM const prog_uchar stepper_table[4] = {0x0f,0xed,0x03,0x21};

// encode / decode table for a nib image
PROGMEM const prog_uchar encTable[] = {
	0x96,0x97,0x9A,0x9B,0x9D,0x9E,0x9F,0xA6,
	0xA7,0xAB,0xAC,0xAD,0xAE,0xAF,0xB2,0xB3,
	0xB4,0xB5,0xB6,0xB7,0xB9,0xBA,0xBB,0xBC,
	0xBD,0xBE,0xBF,0xCB,0xCD,0xCE,0xCF,0xD3,
	0xD6,0xD7,0xD9,0xDA,0xDB,0xDC,0xDD,0xDE,
	0xDF,0xE5,0xE6,0xE7,0xE9,0xEA,0xEB,0xEC,
	0xED,0xEE,0xEF,0xF2,0xF3,0xF4,0xF5,0xF6,
	0xF7,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF
};
PROGMEM const prog_uchar decTable[] = {
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x02,0x03,0x00,0x04,0x05,0x06,
	0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x08,0x00,0x00,0x00,0x09,0x0a,0x0b,0x0c,0x0d,
	0x00,0x00,0x0e,0x0f,0x10,0x11,0x12,0x13,0x00,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1b,0x00,0x1c,0x1d,0x1e,
	0x00,0x00,0x00,0x1f,0x00,0x00,0x20,0x21,0x00,0x22,0x23,0x24,0x25,0x26,0x27,0x28,
	0x00,0x00,0x00,0x00,0x00,0x29,0x2a,0x2b,0x00,0x2c,0x2d,0x2e,0x2f,0x30,0x31,0x32,
	0x00,0x00,0x33,0x34,0x35,0x36,0x37,0x38,0x00,0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f
};

// a table for translating logical sectors into physical sectors
PROGMEM const prog_uchar physicalSector[] = {
		0,13,11,9,7,5,3,1,14,12,10,8,6,4,2,15};

// for bit flip
PROGMEM const prog_uchar FlipBit[] = { 0,  2,  1,  3  };
PROGMEM const prog_uchar FlipBit1[] = { 0, 2,  1,  3  };
PROGMEM const prog_uchar FlipBit2[] = { 0, 8,  4,  12 };
PROGMEM const prog_uchar FlipBit3[] = { 0, 32, 16, 48 };

/* This is for debugging purposes */
void blink_led(unsigned char blink)
{
	unsigned short i = 0;
	for (i = 0; i < blink; i++) {
		PORTD |= 0b00100000;		// LED on
		_delay_ms(20);
		PORTD &= ~(0b00100000);	// LED off
		_delay_ms(20);
		PORTD |= 0b00100000;		// LED on
		_delay_ms(20);
		PORTD &= ~(0b00100000);	// LED off
	}
}

// buffer clear
void buffClear(void)
{
	unsigned char i;
	unsigned short j;
	
	for (i=0; i<BUF_NUM; i++)
		for (j=0; j<350; j++)
			writeData[i][j]=0;
	for (i=0; i<BUF_NUM; i++)
		sectors[i]=tracks[i]=0xff;
}

// cancel read from the SD card
void cancelRead(void)
{
	unsigned short i;
	if (bitbyte<(402*8)) {
		PORTB = 0b00011000;
		for (i=bitbyte; i<(514*8); i++) {
			PORTB = 0b00111000;
			PORTB = 0b00011000;
		}
		bitbyte = 402*8;
	}
}

// write a byte data to the SD card
void writeByte(unsigned char c)
{
	SPDR = c;
	// Wait for transmission complete
	while (!(SPSR & (1<<SPIF)))
		if (bit_is_set(PIND,3)) PORTC &= (~0b00100000);// if eject, reset
}

// read data from the SD card
unsigned char readByte(void)
{
	SPDR = 0xFF;
	// Wait for reception complete
	while (!(SPSR & (1<<SPIF)))
		if (bit_is_set(PIND,3)) PORTC &= (~0b00100000);// if eject, reset
	// Return Data Register
	return SPDR;
}

// wait until data is written to the SD card
void waitFinish(void)
{
	unsigned char ch;
	do {
		ch = readByte();
	} while (ch != 0xff);
}

// issue a SD card command without getting response
void cmd_(unsigned char cmd, unsigned long adr)
{
	writeByte(0xff);
	writeByte(0x40+cmd);
	writeByte(adr>>24);
	writeByte((adr>>16)&0xff);
	writeByte((adr>>8)&0xff);
	writeByte(adr&0xff);
	writeByte(0x95);
	writeByte(0xff);
}

// issue a SD card command and wait normal response
void cmd(unsigned char cmd, unsigned long adr)
{
	unsigned char res;
	do {
		writeByte(0xff);
		writeByte(0x40+cmd);
		writeByte(adr>>24);
		writeByte((adr>>16)&0xff);
		writeByte((adr>>8)&0xff);
		writeByte(adr&0xff);
		writeByte(0x95);
		writeByte(0xff);
	} while (((res=getResp())!=0) && (res!=0xff));
}

// get a command response from the SD card
unsigned char getResp(void)
{
	unsigned char ch;
	do {
		ch = readByte();
	} while ((ch&0x80) != 0);
	return ch;
}

// issue command 17 and get ready for reading
void cmd17(unsigned long adr)
{
	unsigned char ch;

	cmd(17, adr);
	do {	
		ch = readByte();
	} while (ch != 0xfe);
}

// find a file extension
int findExt(char *str, unsigned char *protect, unsigned char *name, uint16_t file_index_offset)
{
	short i;
	unsigned max_file = 512;

	// find NIC extension
	for (i = file_index_offset; i != MAX_FILES_PER_DIR; i++) {
		unsigned char ext[3], d;
		
		// check first char
		cmd(16, 1);
		cmd17(rootAddr+i*32);
		d = readByte();
		readByte(); readByte(); // discard CRC bytes
		if ((d==0x00)||(d==0x05)||(d==0x2e)||(d==0xe5)) continue;
		if (!(((d>='A')&&(d<='Z'))||((d>='0')&&(d<='9')))) continue;
		cmd17(rootAddr+i*32+11);
		d = readByte();
		readByte(); readByte(); // discard CRC bytes
		if (d&0x1e) continue;
		if (d==0xf) continue;
		// check extension
		cmd(16, 4);
		cmd17(rootAddr+i*32+8);
		ext[0] = readByte(); ext[1] = readByte(); ext[2] = readByte();
		if (protect) *protect = ((readByte()&1)<<4); else readByte();
		readByte(); readByte(); // discard CRC bytes
		if ((ext[0]==str[0])&&(ext[1]==str[1])&&(ext[2]==str[2])) {
			/* if we found a file entry,  store the index and break from the loop */
			max_file = i;
			break;
		}
	}
	if ((max_file != 512) && (name != 0)) {
		unsigned char j;
		cmd(16, 8);
		cmd17(rootAddr+max_file*32);
		for (j=0; j<8; j++) name[j] = readByte();
		readByte(); readByte();
		/* Write file name to eeprom */
		eeprom_busy_wait();
		eeprom_write_block((const void*)name, 0x10, 8);
		eeprom_busy_wait();

	}
	return max_file;
	// if 512 then not found...
}

// prepare a FAT table on memory
void prepareFat(int i, unsigned short *fat, unsigned short len,
	unsigned char fatNum, unsigned char fatElemNum)
{
	unsigned short ft;
	unsigned char fn;

	cmd(16, (unsigned long)2);
	cmd17(rootAddr+i*32+26);
	ft = readByte();
	ft += (unsigned short)readByte()*0x100;
	readByte(); readByte(); // discard CRC bytes
	if (0==fatNum) fat[0] = ft;
	for (i=0; i<len; i++) {
		fn = (i+1)/fatElemNum;
		cmd17((unsigned long)fatAddr+(unsigned long)ft*2);
		ft = readByte();
		ft += (unsigned short)readByte()*0x100;
		readByte(); readByte(); // discard CRC bytes
		if (fn==fatNum) fat[(i+1)%fatElemNum] = ft;
		if ((ft>0xfff6)||(fn>fatNum)) break;
	}
	cmd(16, (unsigned long)512);	
}

// memory copy
void memcp(unsigned char *dst, unsigned char *src, unsigned short len)
{
	unsigned short i;
	
	for (i=0; i<len; i++) dst[i]=src[i];
}

void writeSD(unsigned long adr, unsigned char *data, unsigned short len)
{
	unsigned int i;
	unsigned char *buf = &writeData[0][0];

	cmd(16, 512);
	cmd17(adr&0xfffffe00);
	for (i=0; i<512; i++) buf[i] = readByte();
	readByte(); readByte(); // discard CRC bytes
	memcp(&(buf[adr&0x1ff]), data, len);
	
	PORTB |= (1<<SD_CS);
	PORTB &= ~(1<<SD_CS);
				
	cmd(24,adr&0xfffffe00);		
	writeByte(0xff);
	writeByte(0xfe);
	for (i=0; i<512; i++) writeByte(buf[i]);
	writeByte(0xff);
	writeByte(0xff);
	readByte();
	waitFinish();
	
	PORTB |= (1<<SD_CS);
	PORTB &= ~(1<<SD_CS);	
}

void duplicateFat(void)
{
	unsigned short i, j;
	unsigned long adr = fatAddr;
	unsigned char *buf = &writeData[0][0];

	cmd(16, 512);
	for (j=0; j<sectorsPerFat; j++) {
		cmd17(adr);
		for (i=0; i<512; i++) buf[i] = readByte();
		readByte(); readByte(); // discard CRC bytes

		PORTB |= (1<<SD_CS);
		PORTB &= ~(1<<SD_CS);	
	
		cmd(24,adr+(unsigned long)sectorsPerFat*512);		
		writeByte(0xff);
		writeByte(0xfe);
		for (i=0; i<512; i++) writeByte(buf[i]);
		writeByte(0xff);
		writeByte(0xff);
		readByte();
		waitFinish();
		adr += 512;

		PORTB |= (1<<SD_CS);
		PORTB &= ~(1<<SD_CS);
	}
}

// create a NIC image file
int createNic(unsigned char *name)
{
	unsigned short re, clusterNum;
	unsigned long ft, adr;
	unsigned short d, i;
	unsigned char c, dirEntry[32], at;
	static unsigned char last[2] = {0xff, 0xff};
	
	for (i=0; i<32; i++) dirEntry[i]=0;
	memcp(dirEntry, name, 8);
	memcp(dirEntry+8, (unsigned char *)"NIC", 3);
	*(unsigned long *)(dirEntry+28) = (unsigned long)286720;
	
	// search a root directory entry
	for (re=0; re<512; re++) {
		cmd(16, 1);
		cmd17(rootAddr+re*32+0);
		c = readByte();
		readByte(); readByte(); // discard CRC bytes
		cmd17(rootAddr+re*32+11);
		at = readByte();
		readByte(); readByte(); // discard CRC bytes
		if (((c==0xe5)||(c==0x00))&&(at!=0xf)) break;  // find a RDE!
	}	
	if (re==512) return 0;
	// write a directory entry
	writeSD(rootAddr+re*32, dirEntry, 32);	
	// search the first fat entry
	adr = (rootAddr+re*32+26);
	clusterNum = 0;
	for (ft=2;
		(clusterNum<((560+sectorsPerCluster-1)>>sectorsPerCluster2)); ft++) {
		cmd(16, 2);
		cmd17(fatAddr+ft*2);
		d = readByte();
		d += (unsigned short)readByte()*0x100;
		readByte(); readByte(); // discard CRC bytes
		if (d==0) {
			clusterNum++;
			writeSD(adr, (unsigned char *)&ft, 2);
			adr = fatAddr+ft*2;
		}
	}
	writeSD(adr, last, 2);
	duplicateFat();
	return 1;
}

// translate a DSK image into a NIC image
void dsk2Nic(void)
{
	unsigned char trk, logic_sector;

	unsigned short i;
	unsigned char *dst = (&writeData[0][0]+512);
	unsigned short *fatDsk = (unsigned short *)(&writeData[0][0]+1024);

	PORTD |= 0b00100000;

	prevFatNumNic = prevFatNumDsk = 0xff;

	for (i=0; i<0x16; i++) dst[i]=0xff;

	// sync header
	dst[0x16]=0x03;
	dst[0x17]=0xfc;
	dst[0x18]=0xff;
	dst[0x19]=0x3f;
	dst[0x1a]=0xcf;
	dst[0x1b]=0xf3;
	dst[0x1c]=0xfc;
	dst[0x1d]=0xff;
	dst[0x1e]=0x3f;
	dst[0x1f]=0xcf;
	dst[0x20]=0xf3;
	dst[0x21]=0xfc;	
	
	// address header
	dst[0x22]=0xd5;
	dst[0x23]=0xaa;
	dst[0x24]=0x96;
	dst[0x2d]=0xde;
	dst[0x2e]=0xaa;
	dst[0x2f]=0xeb;
	
	// sync header
	for (i=0x30; i<0x35; i++) dst[i]=0xff;
	
	// data
	dst[0x35]=0xd5;
	dst[0x36]=0xaa;
	dst[0x37]=0xad;
	dst[0x18f]=	0xde;
	dst[0x190]=0xaa;
	dst[0x191]=0xeb;
	for (i=0x192; i<0x1a0; i++) dst[i]=0xff;
	for (i=0x1a0; i<0x200; i++) dst[i]=0x00;	

	cmd(16, (unsigned long)512);	
	for (trk = 0; trk < 35; trk++) {
		PORTD ^= 0b00100000;
		for (logic_sector = 0; logic_sector < 16; logic_sector++) {
			unsigned char *src;
			unsigned short ph_sector = (unsigned short)pgm_read_byte_near(physicalSector+logic_sector);

			if ((logic_sector&1)==0) {
				unsigned short long_sector = (unsigned short)trk*8+(logic_sector/2);
				unsigned short long_cluster = long_sector>>sectorsPerCluster2;
				unsigned char fatNum = long_cluster/FAT_DSK_SIZE;
				unsigned short ft;

				if (fatNum != prevFatNumDsk) {
					prevFatNumDsk = fatNum;						
					prepareFat(dskDir, fatDsk,
						(280+sectorsPerCluster-1)>>sectorsPerCluster2, fatNum, FAT_DSK_SIZE);	
				}
				ft = fatDsk[long_cluster%FAT_DSK_SIZE];
				cmd17((unsigned long)userAddr+(((unsigned long)(ft-2)<<sectorsPerCluster2)
					+ (long_sector&(sectorsPerCluster-1)))*(unsigned long)512);
				for (i=0; i<512; i++) {
					*(&writeData[0][0]+i)=readByte();
				}
				readByte(); readByte(); // discard CRC bytes				
				src = &writeData[0][0];
			} else {
				src = (&writeData[0][0]+256);
			}
			{
				unsigned char c, x, ox = 0;

				dst[0x25]=((volume>>1)|0xaa);
				dst[0x26]=(volume|0xaa);
				dst[0x27]=((trk>>1)|0xaa);
				dst[0x28]=(trk|0xaa);
				dst[0x29]=((ph_sector>>1)|0xaa);
				dst[0x2a]=(ph_sector|0xaa);
				c = (volume^trk^ph_sector);
				dst[0x2b]=((c>>1)|0xaa);
				dst[0x2c]=(c|0xaa);
				for (i = 0; i < 86; i++) {
					x = (pgm_read_byte_near(FlipBit1+(src[i]&3)) |
						pgm_read_byte_near(FlipBit2+(src[i+86]&3)) |
						((i<=83)?pgm_read_byte_near(FlipBit3+(src[i+172]&3)):0));
					dst[i+0x38] = pgm_read_byte_near(encTable+(x^ox));
					ox = x;
				}
				for (i = 0; i < 256; i++) {
					x = (src[i] >> 2);
					dst[i+0x8e] = pgm_read_byte_near(encTable+(x^ox));
					ox = x;
				}
				dst[0x18e]=pgm_read_byte_near(encTable+ox);
			}
			{
				unsigned short long_sector = (unsigned short)trk*16+ph_sector;
				unsigned short long_cluster = long_sector>>sectorsPerCluster2;
				unsigned char fatNum = long_cluster/FAT_NIC_SIZE;
				unsigned short ft;
			
				if (fatNum != prevFatNumNic) {
					prevFatNumNic = fatNum;
					prepareFat(nicDir, fatNic,
						(560+sectorsPerCluster-1)>>sectorsPerCluster2, fatNum, FAT_NIC_SIZE);
				}
				ft = fatNic[long_cluster%FAT_NIC_SIZE];

				PORTB |= (1<<SD_CS);
				PORTB &= ~(1<<SD_CS);	

				cmd(24, userAddr+(((unsigned long)(ft-2)<<sectorsPerCluster2)
					+ (long_sector&(sectorsPerCluster-1)))*(unsigned long)512);
				writeByte(0xff);
				writeByte(0xfe);
				for (i = 0; i < 512; i++) writeByte(dst[i]);
				PORTB = 0b00010000;
				writeByte(0xff);
				writeByte(0xff);
				readByte();
				waitFinish();

				PORTB |= (1<<SD_CS);
				PORTB &= ~(1<<SD_CS);	
			}
		}
	}
	buffClear();
	PORTD &= ~(0b00100000);
}

// initialization SD card
int SDinit(void)
{
	unsigned char ch;
	unsigned short i;
	char str[5];
	unsigned char filebase[8];

	PORTD |= 0b00100000;		// LED on

	//SPI enable, master mode, f/128
	SPCR = ((1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0));

	PORTB |= (1<<SD_CS);					// disable CS
	for(i=0; i < 10; i++) writeByte(0xFF);	// Send 10 * 8 = 80 clock
	PORTB &= ~(1<<SD_CS);					// enable CS
	for(i=0; i < 2; i++) writeByte(0xFF);	// Send 2 * 8 = 16 clock

	cmd_(0, 0);	// command 0
 	do {	
		ch = readByte();
	} while (ch != 0x01);
	
	PORTB |= (1<<SD_CS);		// disable CS

	while (1) {
		PORTB &= ~(1<<SD_CS);
		cmd_(55, 0);			// command 55
		ch = getResp();
		if (ch == 0xff) return 0;
		if (ch & 0xfe) continue;
		// if (ch == 0x00) break;
		PORTB |= (1<<SD_CS);
		PORTB &= ~(1<<SD_CS);
		cmd_(41, 0);			// command 41	
		if (!(ch=getResp())) break;
		if (ch == 0xff) return 0;
		PORTB |= (1<<SD_CS);
	}

	//SPI enable, master mode, f/2
	SPCR = ((1<<SPE)|(1<<MSTR));
	SPSR = (1<<SPI2X);

	// BPB address
	cmd(16,5);
	cmd17(54);
	for (i=0; i<5; i++) str[i] = readByte();
	readByte(); readByte();	// discard CRC
	if ((str[0]=='F')&&(str[1]=='A')&&(str[2]=='T')&&
		(str[3]=='1')&&(str[4]=='6')) {
		bpbAddr = 0;
	} else {
		cmd(16, 4);
		cmd17((unsigned long)0x1c6);
		bpbAddr = readByte();
		bpbAddr += (unsigned long)readByte()*0x100;
		bpbAddr += (unsigned long)readByte()*0x10000;
		bpbAddr += (unsigned long)readByte()*0x1000000;
		bpbAddr *= 512;
		readByte(); readByte(); // discard CRC bytes
	}

	// sectorsPerCluster and reservedSectors
	{
		unsigned short reservedSectors;
		volatile unsigned char k;
		cmd(16, 3);
		cmd17(bpbAddr+0xd);
		sectorsPerCluster = k = readByte();
		sectorsPerCluster2 = 0;
			while (k != 1) {
			sectorsPerCluster2++;
			k >>= 1;
		}
		reservedSectors = readByte();
		reservedSectors += (unsigned short)readByte()*0x100;
		readByte(); readByte(); // discard CRC bytes	
		// sectorsPerCluster = 0x40 at 2GB, 0x10 at 512MB
		// reservedSectors = 2 at 2GB
		fatAddr = bpbAddr + (unsigned long)512*reservedSectors;
	}

	{
		// sectorsPerFat and rootAddr
		cmd(16, 2);
		cmd17(bpbAddr+0x16);
		sectorsPerFat = readByte();
		sectorsPerFat += (unsigned short)readByte()*0x100;
		readByte(); readByte(); // discard CRC bytes		
		// sectorsPerFat =  at 512MB,  0xEF at 2GB
		rootAddr = fatAddr + ((unsigned long)sectorsPerFat*2*512);
		userAddr = rootAddr+(unsigned long)512*32;
	}

	// find "NIC" extension
	nicDir = findExt("NIC", &protect, filebase, current_disk_image);
	if (nicDir == 512) {		// create NIC file if not exists
		// find "DSK" extension
		dskDir = findExt("DSK", (unsigned char *)0, filebase, current_disk_image);
		if (dskDir == 512) return 0;
		if (!createNic(filebase)) return 0;
		nicDir = findExt("NIC", &protect, (unsigned char *)0, current_disk_image);
		if (nicDir == 512) return 0;
		// convert DSK image to NIC image
		dsk2Nic();
	}
	
	prevFatNumNic = 0xff;
	prevFatNumDsk = 0xff;
	cmd(16, (unsigned long)512);
	SPCR = 0;					// disable spi
	PORTD &= ~(0b00100000);	// LED off
	
	return 1;
}

// move head
ISR(PCINT1_vect)
{
	if (bit_is_set(PIND, 7)) return;
	
	unsigned char stp;
	static unsigned char prevStp = 0;

	stp = (PINC & 0b00001111);
	if (stp != prevStp) {
		prevStp = stp;
		unsigned char ofs =
			((stp==0b00001000)?2:
			((stp==0b00000100)?4:
			((stp==0b00000010)?6:
			((stp==0b00000001)?0:0xff))));
		if (ofs != 0xff) {
			ofs = ((ofs+ph_track)&7);
			unsigned char bt = pgm_read_byte_near(stepper_table + (ofs>>1));
			prevStp = stp;
			if (ofs&1) bt &= 0x0f; else bt >>= 4;
			ph_track += ((bt & 0x08) ? (0xf8 | bt) : bt);
			if (ph_track > 196) ph_track = 0;	
			if (ph_track > 139) ph_track = 139;
		}
	}
}

int main(void)
{
	PORTB = 0b00010000;
	PORTC = 0b00100000;
	PORTD = 0b01001000;

	DDRB = 0b00101100;	
	DDRC = 0b00110000;
	DDRD = 0b00110000;
	
	/* PORTD0 acts as an input */
	/* turn on the pull up on PD0 (pin2) */
	PORTD |= (1 << PORTD0);

	/* PORTD1 acts as an input */
	/* turn on the pull up on PD1 (pin3) */
	PORTD |= (1 << PORTD1);

	sei();

	eeprom_busy_wait();
	ph_track = eeprom_read_byte(EEP_PH_TRACK);
	eeprom_busy_wait();
	current_disk_image = eeprom_read_word(EEP_CURRENT_DISK_IMAGE);
	if (current_disk_image >= 512) {
		current_disk_image = 0;
	}
	if (ph_track > 196) ph_track = 0;	
	if (ph_track > 139) ph_track = 139;

	// pc interrupt for head move
	PCMSK1 = 0b00001111;
	PCICR = (1<<PCIE1);
    
	/* Turn pin change interrupt on PORD0 */
	PCICR |= (1 << PCIE2);    
	PCMSK2 |= (1 << PCINT16);  
	PCMSK2 |= (1 << PCINT17);  

	unsigned long i;

	// wait long low of eject
	for (i=0; i!=0x80000;i++) {	
		if (bit_is_set(PIND,3)) {
			i=0; continue;
		}
	}
	// now SD card has been inserted completely

	// timer0 (4u sec)
	OCR0A = 108;
	TCCR0A = (1<<WGM01);
	TCCR0B = (1<<CS00);
	
	MCUCR = 0b00000010;
	
	// int0 falling edge interrupt for write request
	// int1 rising edge interrupt for eject
	EICRA = 0b00001110;

	if (SDinit()) {
		// initialize variables
		readPulse = 0;
		magState = 0;
		prepare = 1;
		bitbyte = 0;
		sector = 0;
		buffNum = 0;
		formatting = 0;
		writePtr = &(writeData[buffNum][0]);
		buffClear();
		TIMSK0 |= (1<<OCIE0A);	// on timer
		EIMSK |= (1<<INT0);	// on int0
		EIMSK |= (1<<INT1);	// on int1
		SPCR = 0; 				// off spi
	} else while (1) ;
	
	while (1) {
		if (bit_is_set(PIND, 7)) { 				// disable drive
			PORTD &= ~(0b00100000);				// LED off
		} else { 									// enable drive                                                                                                                                                                   
			PORTD |= 0b00100000;					// LED on
			if (bit_is_set(PIND,6)||protect||(PINC&2))
				PORTD |= 0b00010000;
			else
				PORTD &= ~0b00010000;
			if (prepare) {
				TIMSK0 &= ~(1<<OCIE0A);			// disable timer0
				sector = ((sector+1)&0xf);
				unsigned char trk = (ph_track>>2);
				if (!(((sectors[0]^sector)|(tracks[0]^trk)) &
					((sectors[1]^sector)|(tracks[1]^trk)) &
					((sectors[2]^sector)|(tracks[2]^trk)) &
					((sectors[3]^sector)|(tracks[3]^trk)) &
					((sectors[4]^sector)|(tracks[4]^trk))))
					writeBack();
				SPCR = ((1<<SPE)|(1<<MSTR));		// enable spi
				unsigned short long_sector = (unsigned short)trk*16+sector;
				unsigned short long_cluster = long_sector>>sectorsPerCluster2;
				unsigned char fatNum = long_cluster/FAT_NIC_SIZE;
				unsigned short ft;
				if (fatNum != prevFatNumNic) {
					prevFatNumNic = fatNum;
					prepareFat(nicDir, fatNic, (560+sectorsPerCluster-1)>>sectorsPerCluster2, fatNum, FAT_NIC_SIZE);
				}
				ft = fatNic[long_cluster%FAT_NIC_SIZE];
				cmd17(userAddr+(((unsigned long)(ft-2)<<sectorsPerCluster2)
					+ (long_sector&(sectorsPerCluster-1)))*512);
				bitbyte = 0;
				prepare = 0;
				SPCR = 0;							// disable SPI
				TIMSK0 |= (1<<OCIE0A);				// enable timer0
			}
			if (doBuffering) {
				doBuffering = 0;
				buffering();
			}
		}
	}
}

void writeBackSub(unsigned char bn, unsigned char sc, unsigned char track)
{
	unsigned char c;
	unsigned short i;
	unsigned short long_sector = (unsigned short)track*16+sc;
	unsigned short long_cluster = long_sector>>sectorsPerCluster2;
	unsigned char fatNum = long_cluster/FAT_NIC_SIZE;
	unsigned short ft;

	TIMSK0 &= ~(1<<OCIE0A);		// disable timer0
	SPCR = ((1<<SPE)|(1<<MSTR));	// enable spi

	// BPB address
	if (fatNum != prevFatNumNic) {
		prevFatNumNic = fatNum;
		prepareFat(nicDir, fatNic,
			(560+sectorsPerCluster-1)>>sectorsPerCluster2, fatNum, FAT_NIC_SIZE);
	}
	ft = fatNic[long_cluster%FAT_NIC_SIZE];
	
	PORTB |= (1<<SD_CS);
	PORTB &= ~(1<<SD_CS);

	cmd(24, (unsigned long)userAddr+(((unsigned long)(ft-2)<<sectorsPerCluster2)
		+ (unsigned long)(long_sector&(sectorsPerCluster-1)))*512);

	writeByte(0xff);
	writeByte(0xfe);
	
	// 22 ffs
	for (i = 0; i < 22; i++) {
		writeByte(0xff);
	}

	// sync header
	writeByte(0x03);
	writeByte(0xfc);
	writeByte(0xff);
	writeByte(0x3f);
	writeByte(0xcf);
	writeByte(0xf3);
	writeByte(0xfc);
	writeByte(0xff);
	writeByte(0x3f);
	writeByte(0xcf);
	writeByte(0xf3);
	writeByte(0xfc);

	// address header
	writeByte(0xd5);
	writeByte(0xAA);
	writeByte(0x96);
	writeByte((volume>>1)|0xaa);
	writeByte(volume|0xaa);
	writeByte((track>>1)|0xaa);
	writeByte(track|0xaa);
	writeByte((sc>>1)|0xaa);
	writeByte(sc|0xaa);
	c = (volume^track^sc);
	writeByte((c>>1)|0xaa);
	writeByte(c|0xaa);
	writeByte(0xde);
	writeByte(0xAA);
	writeByte(0xeb);

	// sync header
	writeByte(0xff);	
	writeByte(0xff);
	writeByte(0xff);
	writeByte(0xff);
	writeByte(0xff);

	// data
	for (i = 0; i < 349; i++) {
		c = writeData[bn][i];
		writeByte(c);
	}
	for (i = 0; i < 14; i++) {
		writeByte(0xff);
	}
	for (i = 0; i < 96; i++) {
		writeByte(0);
	}
	writeByte(0xff);
	writeByte(0xff);
	readByte();
	waitFinish();

	PORTB |= (1<<SD_CS);
	PORTB &= ~(1<<SD_CS);	

	SPCR = 0;							// disable spi
	TIMSK0 |= (1<<OCIE0A);				// enable timer0
}

// write back into the SD card
void writeBack(void)
{
	unsigned char i, j;

	for (j=0; j<BUF_NUM; j++) {
		if (sectors[j]!=0xff) {
			for (i=0; i<BUF_NUM; i++) {
				if (sectors[i] != 0xff)
					writeBackSub(i, sectors[i], tracks[i]);
				sectors[i] = 0xff;
				tracks[i] = 0xff;
				writeData[i][2]=0;
			}
			buffNum = 0;
			writePtr = &(writeData[buffNum][0]);
			break;
		}
	}
}

// set write pointer and write back if need
void buffering(void)
{
	static unsigned char sec;

	if (writeData[buffNum][2]==0xAD) {
		if (!formatting) {
			sectors[buffNum]=sector;
			tracks[buffNum]=(ph_track>>2);
			sector=((((sector==0xf)||(sector==0xd))?(sector+2):(sector+1))&0xf);
			if (buffNum == (BUF_NUM-1)) {
				// cancel reading
				cancelRead();
				writeBack();
				prepare = 1;
			} else {
				buffNum++;
				writePtr = &(writeData[buffNum][0]);
			}
		} else {
			sector = sec;
			formatting = 0;
			if (sec == 0xf) {
				// cancel reading
				cancelRead();
				prepare = 1;
			}
		}
	} if (writeData[buffNum][2]==0x96) {
		sec = (((writeData[buffNum][7]&0x55)<<1) | (writeData[buffNum][8]&0x55));
		formatting = 1;
	}
}

// reset when eject
ISR(INT1_vect)
{
	unsigned long i;

	// wait long high of eject
	for (i=0; i!=0x80000;i++)
		if (bit_is_clear(PIND,3)) return;
	// now SD card has been removed completely

	// record track number on eeprom
	eeprom_busy_wait();
	eeprom_write_byte (EEP_PH_TRACK, ph_track); 

	// and reset!
	PORTC &= (~0b00100000);
}
	
/* Interrupt handler for PORTD0 (pin 2) */
ISR (PCINT2_vect)
{
	/* If button on PORTD0 is pressed */
	/* increment the disk image index and restart */
	if ((PIND & (1 << PIND0)) == 0) {
		PORTD &= ~(0b00100000);		
		++current_disk_image;
		eeprom_busy_wait();
		eeprom_write_word(EEP_CURRENT_DISK_IMAGE, current_disk_image);
		eeprom_busy_wait();
		PORTC &= (~0b00100000);
	}
	/* If button on PORTD1 is pressed */
	/* reset the disk image index to 0 and restart */
	if ((PIND & (1 << PIND1)) == 0) {
		PORTD |= 0b00100000;		// LED on
		current_disk_image = 0;
		eeprom_busy_wait();
		eeprom_write_word(EEP_CURRENT_DISK_IMAGE, current_disk_image);
		eeprom_busy_wait();
		PORTC &= (~0b00100000);
	}
}
