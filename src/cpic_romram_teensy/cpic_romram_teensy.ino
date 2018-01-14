
/*
   CPIC - An Amstrad CPC ROM/RAM Board in a Teensy 3.5 microcontroller board

   (C) 2017 Revaldinho

   Teensy3.5 is a micro controller with the following key features
   - 120 MHz ARM Cortex-M4 with Floating Point Unitct
   - 512K Flash, 192K RAM, 4K EEPROM
   - Microcontroller Chip MK64FX512VMD12
   - 1 CAN Bus Port
   - 16 General Purpose DMA Channels
   - 5 Volt Tolerance On All Digital I/O Pins

   CPIC uses the Teensy to expand an Amstrad CPC providing
   - 16 Upper (Application) ROM slots
   - Lower (Firmware) ROM replacement
   - 128K Bytes of RAM expansion using a DK'tronics/CPC6128 compatible banking scheme

   All ROM data is stored in flash memory to be persistent, but accessing it does
   incur some wait states on the MIPS CPU whereas the RAM runs at the full 120MHz
   core speed. Copying the ROM contents into RAM is possible to speed this up,
   but then limits the number of ROMs available. Copying will take a while so may
   need to reboot the CPC after the ROM board: if sharing the CPC PSU then need
   to provide a CPC reset button on the shield.

   CPC Peripheral Operation
   ========================

   ROM Selection performed by writing the ROM number to an IO address with A13 low.

   RAM expansion models a 64KBytes DKtronics unit. See
   http://www.cpcwiki.eu/imgs/8/83/DK%27Tronics_Peripheral_-_Technical_Manual_%28Edition_1%29.pdf

   RAM is switched in blocks of 16KBytes by writing a byte with the top two bit set to
   an IO address with A15 low. The remaining 6 bits are then interpreted as follows:

        0b11 ccc bbb

        ccc - is the bank code
        bbb - is the block code

   The block to be accessed can be computed from the following table where ccc is only valid for 000 and 001 values (which select
   between bank 0 (built-in) and expansion bank 1 or between bank0 and expansion bank 2 respectively. '-1' in the table means
   that the RAM expansion is not used and the access goes to the internal RAM. The rows in the table are selected according to
   the upper two address bits of the memory access.

   -------------------------------------------------------------------------------------------------------------------------------:---------
   Address\cccbbb 000000 000001 000010 000011 000100 000101 000110 000011 001000 001001 001010 001011 001100 001101 001110 001111 : >001111
   -------------------------------------------------------------------------------------------------------------------------------:---------
   1100-1111       -1      3      3      3      -1     -1     -1     -1     -1     7       7      7     -1     -1     -1     -1   :     -
   1000-1011       -1      -1     2      -1     -1     -1     -1     -1     -1     -1      6      -1    -1     -1     -1     -1   :     -
   0100-0111       -1      -1     1      -1     0      1       2      3     -1     -1      5      -1    4      5      6      7    :     -
   0000-0011       -1      -1     0      -1     -1     -1     -1     -1     -1     -1      4      -1    -1     -1     -1     -1   :     -
   -------------------------------------------------------------------------------------------------------------------------------:---------

   ROM is accessed whenever ROMEN_B goes low. Addresses with Address[15] set go to upper (foreground)
   ROMs and those with Address[15] low go to lower ROM.

*/
#ifdef DEBUG
#include <string.h>
#endif

#define DEBUG            1
//#define TEENSY_ASM       1
#define LOWER_ROM_ENABLE 1
#define RAM_EXP_ENABLE   1

// ----- Some string processing definitions to enable use of C tokens in assembler statements
#define str(a) st(a)
#define st(a) #a

// --- Port allocations
// Data direction is the opposite of ChipKit - on Teensy 0=input, 1=Output

// Some absolute addresses for the assembler code
#define GPIOB_PDIR_ADDR 0x400FF050
#define GPIOD_PDIR_ADDR 0x400FF0D0
#define GPIOC_PDDR_ADDR 0x400FF094

// Arduino defined tokens (including unit32_t*) casts for C
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
#define MASK            0x001F         // All above control signals inactive

#define ADDR_HI         0x00FF0000   // bits 16-23 on control input data
#define ADDR_LO         0x000000FF   // bits  0-7 on address input
#define ADR_13_RAW      0x00200000   // bit 13 in raw position in ctrl/adr word - ie bit 21
#define ADR_15_RAW      0x00800000   // bit 15 in raw position in ctrl/adr word - ie bit 23

#define VALIDRAMSELMASK 0x00C0       // Top two bits of Data must be set for a valid RAM selection
#define RAMCODEMASK     0x0038       // Next 3 bits are the 'code'
#define RAMBANKMASK     0x0007       // Bottom 3 bits are the 'code'

// ---- Constants and macros
#define MAXROMS         16
#define RAMBLKSIZE      16384

#ifdef DEBUG
#define ROMSIZE         128
#else
#define ROMSIZE         16384
#endif

#define ADR_HI_BYTE     (((ctrladrhi&ADDR_HI)>>24)&0xFF)
#define ADDRESS_IN      ((ADR_HI_BYTE<<8)|((ADR_LO_IN)&0xFF))

#define ROMSEL          (!(ctrladrhi&(IOREQ_B|ADR_13_RAW|WR_B)))
#define RAMSEL          (!(ctrladrhi&(IOREQ_B|ADR_15_RAW|WR_B)))
#define HIROMRD         ((!(ctrladrhi&(ROMEN_B)))&&(ctrladrhi&ADR_15_RAW))
#define LOROMRD         (!(ctrladrhi&(ROMEN_B|ADR_15_RAW)))
#define ROMRD           (!(ctrladrhi&(ROMEN_B)))
#define RAMRD           (!(ctrladrhi&(RAMRD_B)))
#define RAMWR           (!(ctrladrhi&(MREQ_B|WR_B)))

// Global variables
const char upperrom[MAXROMS * ROMSIZE] = {
#ifdef DEBUG
  0, 0, 0, 0, 0
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

#ifdef LOWER_ROM_ENABLE
// This will be the content of the AMSTRAD Firmware ROM to allow replacement
// of a CPC464 or 664 ROM with that from a 6128.
const char lowerrom[ROMSIZE] = {
#ifdef DEBUG
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9
#else
#include "/Users/richarde/Documents/Development/git/CPiC/src/OS_464.CSV"
#endif
};
#endif

#ifdef RAM_EXP_ENABLE
char ram[8][RAMBLKSIZE] ;                   //  128K for D'ktronics RAM exp
const int ramblocklut[4][16] = {
  -1,  3, 3,  3, -1, -1, -1, -1, -1,  7,  7,  7, -1, -1, -1, -1,
  -1, -1, 2, -1, -1, -1, -1, -1, -1, -1,  6, -1, -1, -1, -1, -1,
  -1, -1, 1, -1,  0,  1,  2,  3, -1, -1,  5, -1,  4,  5,  6,  7,
  -1, -1, 0, -1, -1, -1, -1, -1, -1, -1,  4, -1, -1, -1, -1, -1,
};
#endif

void setup() {
  // Set all pins to input mode using pinMode instructions as easy way of setting up
  // GPIO control - can use port registers after this initialization
  for (int i = 0 ; i < 58 ; i++ ) {
    pinMode(i, INPUT);
  }
}

void loop() {
#ifdef DEBUG
  int ram_lut_col = 2;
  boolean validrom = true;
#else
  int ram_lut_col = 0;
  boolean validrom = false;
#endif
  int romnum = 0;
  register int ctrladrhi;
  register int address;
  int ramblock;
  int datain;
  int ram_lut_row = 0;
  while (true) {


#ifdef TEENSY_ASM
    asm volatile (
        "ldr     r4, =" str(GPIOB_PDIR_ADDR) "\n\t"     // ctrl/high address port
        "ldr     r5, =" str(GPIOD_PDIR_ADDR) "\n\t"     // Address low byte port
        "ldr     r7, =" str(GPIOC_PDDR_ADDR) "\n"       // data/ctrl output direction

        // Exit loop - wait for all ctrl to go inactive
"loop1:  ldr     %[ctrladrhi], [r4]\n\t"                // NB 1 cycle load-before-use penalty here
        "movw    r8, #0\n\t"                            // ... so repeatedly load const into r8 for use later in delay slot
        "and     r6, %[ctrladrhi], #" str(MASK) "\n\t"  // Mask off control bits
        "cmp     r6, #" str(MASK) "\n\t"
        "bne     loop1\n\t"
        "str     r8,[r7]\n\t"                           // tristate the data and ctrl outputs
        
        // Entry loop - wait for one active low signal to go active
"loop2:  ldr     %[ctrladrhi], [r4]\n\t"               // NB 1 cycle load-before-use penalty here
        "ldr     %[address], [r5]\n\t"                 // .. so sample address low bits to make use of the latency
        "and     r6, %[ctrladrhi], #" str(MASK) "\n\t"
        "teqs    r6, #" str(MASK) "\n\t"
        "beq     loop2\n\t"

        "lsr     r5, %[ctrladrhi], #16\n\t"             // Extract addr hi bits and save in byte 1 of r5
        "and     r5, r5, #0xFF00\n\t"                   // Isolate high order bits
        "and     %[address], %[address], #0x00FF\n\t"   // Clear all other bits in low order byte
        "orr     %[address], %[address], r5\n\t"        // Merge low and high bits of address into one in %[address]
        :   [address] "=r" (address), [ctrladrhi] "=r" (ctrladrhi)               // Outputs list (C variables in registers)
        :                                               // No inputs
        :   "r4", "r5", "r6", "r7", "r8"                // Register clobber list
      );
#else
    while ( ((ctrladrhi = CTRLADRHI_IN)&MASK) != MASK ) {} // Wait for control signals to go inactive
    DATACTRL_OUT = 0x0;                                  // actively deassert ROMDIS/RAMDIS before..
    DATACTRL_MODE = 0x0;                                 // tristating all data and control outputs
    while ( ((ctrladrhi = CTRLADRHI_IN)&MASK) == MASK ) {} // Now wait for control signals to become active
    address = ADDRESS_IN;
#endif


#ifdef RAM_EXP_ENABLE
    if ( RAMRD ) {
      if ( ram_lut_col > -1 ) {
        ram_lut_row = (address & 0xC000) >> 14;
        ramblock = ramblocklut[ram_lut_row][ram_lut_col];
        if (ramblock >= 0 ) {
          DATACTRL_OUT = ram[ramblock][address] | RAMDIS;
          DATACTRL_MODE = DATA | RAMDIS;
        }
      }
    } else
#endif
      if ( HIROMRD && validrom ) {
        DATACTRL_OUT = upperrom[(romnum << 14) | (address & 0x3FFF)] | ROMDIS;
        DATACTRL_MODE = DATA | ROMDIS;
#ifdef LOWER_ROM_ENABLE
      } else if ( LOROMRD ) {
        DATACTRL_OUT = lowerrom[address & 0x3FFF] | ROMDIS;
        DATACTRL_MODE = DATA | ROMDIS;
#endif
#ifdef RAM_EXP_ENABLE
      } else if ( RAMWR ) {
        if ( ram_lut_col > -1 ) {
          datain  = DATACTRL_IN & DATA;
          ram_lut_row = (address & 0xC000) >> 14                 ;
          ramblock = ramblocklut[ram_lut_row][ram_lut_col] ;
          if (ramblock >= 0 ) {
            DATACTRL_OUT =  RAMDIS;
            DATACTRL_MODE = RAMDIS;
            ram[ramblock][address] = datain;
          }
        }
#endif
      } else if ( ROMSEL ) {
        datain  = DATACTRL_IN & DATA;
        romnum = ((datain & 0xF)) ;   // only allow 16 ROMS
        validrom = valid_upperrom[romnum];
      }
#ifdef RAM_EXP_ENABLE
      else if ( RAMSEL ) {
        datain  = DATACTRL_IN & DATA;
        if ((datain & VALIDRAMSELMASK) == VALIDRAMSELMASK) {
          // Now validate that only codes 000xxx or 001xxx are used - other codes unsupported (default to machine built-in RAM)
          if ((datain & RAMCODEMASK) <= 0x08) {
            ram_lut_col   = datain & RAMBANKMASK; // Set column for the RAM block lookup
          } else {
            ram_lut_col   = -1;          // disable RAM expansion
          }
        }
      }
#endif
  }
}
