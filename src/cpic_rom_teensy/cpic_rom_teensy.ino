
/*
 * CPIC - An Amstrad CPC ROM/RAM Board in a Teensy 3.5 microcontroller board
 * 
 * (C) 2017 Revaldinho
 * 
 * Teensy3.5 is a micro controller with the following key features 
 * - 120 MHz ARM Cortex-M4 with Floating Point Unit
 * - 512K Flash, 192K RAM, 4K EEPROM
 * - Microcontroller Chip MK64FX512VMD12
 * - 1 CAN Bus Port
 * - 16 General Purpose DMA Channels
 * - 5 Volt Tolerance On All Digital I/O Pins
 *
 * CPIC uses the Teensy to expand an Amstrad CPC providing
 * - 16 Upper (Application) ROM slots
 * - Lower (Firmware) ROM replacement
 * - 128K Bytes of RAM expansion using a DK'tronics/CPC6128 compatible banking scheme
 * 
 * All ROM data is stored in flash memory to be persistent, but accessing it does 
 * incur some wait states on the MIPS CPU whereas the RAM runs at the full 120MHz 
 * core speed. Copying the ROM contents into RAM is possible to speed this up, 
 * but then limits the number of ROMs available. Copying will take a while so may 
 * need to reboot the CPC after the ROM board: if sharing the CPC PSU then need
 * to provide a CPC reset button on the shield.
 *
 * ROM Selection performed by writing the ROM number to an IO address with A13 low.
 * ROM is accessed whenever ROMEN_B goes low. Addresses with Address[15] set go to upper (foreground)
 * ROMs and those with Address[15] low go to lower ROM.
 *       
 */
#ifdef DEBUG
#include <string.h>
#endif

#define DEBUG            1
//#define TEENSY_ASM       0
//#define LOWER_ROM_ENABLE 0
//#define RAM_EXP_ENABLE   0

// ----- Some string processing definitions to enable use of C tokens in assembler statements
#define str(a) st(a)    
#define st(a) #a   

// --- Port allocations 
// Data direction is the opposite of ChipKit - on Teensy 0=input, 1=Output
#define  CTRLADRHI_IN   GPIOB_PDIR
#define  CTRLADRHI_MODE GPIOB_PDDR
#define  DATACTRL_IN    GPIOC_PDIR
#define  DATACTRL_OUT   GPIOC_PDOR
#define  DATACTRL_MODE  GPIOC_PDDR
#define  ADR_LO_IN      GPIOD_PDIR
#define  ADR_LO_MODE    GPIOD_PDDR

// --- Bit allocations
#define DATA            0x00FF
#define ROMDIS          0x0100
#define RAMDIS          0x0200
// #define WAIT_B          0x0400

#define MREQ_B          0x00000001
#define IOREQ_B         0x00000002
#define WR_B            0x00000004
#define RAMRD_B         0x00000008
#define ROMEN_B         0x00000010
#define M1_B            0x00000020

#define ADDR_HI         0x00FF0000   // bits 16-23 on control input data
#define ADDR_LO         0x000000FF   // bits  0-7 on address input
#define ADR_13_RAW      0x00200000   // bit 13 in raw position in ctrl/adr word - ie bit 21
#define ADR_15_RAW      0x00800000   // bit 15 in raw position in ctrl/adr word - ie bit 23

#define MASK            (MREQ_B|IOREQ_B|WR_B|RAMRD_B|ROMEN_B)
// ---- Constants and macros
#define MAXROMS         16
#define RAMBLKSIZE      16384

#ifdef DEBUG
#define ROMSIZE         128
#else
#define ROMSIZE         16384
#endif

#define ADR_HI_BYTE     ((ctrladr&ADDR_HI)>>24)&0xFF
#define ADDRESS_IN      ((ADR_HI_BYTE<<8)|((ADR_LO_IN)&0xFF))

#define ROMSEL          !(ctrladr&(IOREQ_B|ADR_13_RAW|WR_B))
#define HIROMRD         ((!(ctrladr&(ROMEN_B)))&&(ctrladr&ADR_15_RAW))
#define ROMRD           !(ctrladr&(ROMEN_B))


// Global variables
const char upperrom[MAXROMS*ROMSIZE] = { 
#ifdef DEBUG
0,0,0,0,0
#else
#include "/Users/richarde/Documents/Development/git/CPiC/src/BASIC_1.0.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/CWTA.CSV"
//#include "ROM1.csv"
//#include "ROM2.csv"
//#include "ROM3.csv"
#endif
};

const boolean valid_upperrom[MAXROMS] = { 
  true, true, false, false,
  false, false, false, false,
  false, false, false, false,
  false, false, false, false
};

void setup() {
  // Set all pins to input mode using pinMode instructions as easy way of setting up 
  // GPIO control - can use port registers after this initialization
  for (int i=0 ; i< 58 ; i++ ) {
    pinMode(i,INPUT);
  }
}

void loop() {
  boolean validrom = true;
  int romnum = 0;  
  register int ctrladr;
  register int address;
  int datain;

  while (true) {
    while ( ((CTRLADRHI_IN)&MASK) != MASK ) {}           // Wait for control signals to go inactive
    DATACTRL_OUT = 0x0;
    DATACTRL_MODE = 0x0;                                 // Tristate all data and control outputs
    while ( ((ctrladr = CTRLADRHI_IN)&MASK) == MASK ) {} // Now wait for control signals to become active
    
    address = ADDRESS_IN;
    if ( HIROMRD && validrom ) {
      DATACTRL_OUT = upperrom[(romnum<<14)|(address&0x3FFF)] | ROMDIS;
      //DATACTRL_OUT =  0x00 | ROMDIS;
      DATACTRL_MODE = DATA | ROMDIS ;
    } else if ( ROMSEL ) {
      datain  = DATACTRL_IN&DATA;
      romnum = ((datain&0xF)) ;     // only allow 16 ROMS
      validrom = valid_upperrom[romnum];
    }
  }
}
