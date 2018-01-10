
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
 * CPC Peripheral Operation
 * ========================
 *
 * ROM Selection performed by writing the ROM number to an IO address with A13 low.
 * 
 * RAM expansion models a 64KBytes DKtronics unit. See 
 * http://www.cpcwiki.eu/imgs/8/83/DK%27Tronics_Peripheral_-_Technical_Manual_%28Edition_1%29.pdf
 * 
 * RAM is switched in blocks of 16KBytes by writing a byte with the top two bit set to
 * an IO address with A15 low. The remaining 6 bits are then interpreted as follows:
 * 
 *      0b11 ccc bbb
 *      
 *      ccc - is the bank code 
 *      bbb - is the block code
 *      
 * The block to be accessed can be computed from the following table where ccc is only valid for 000 and 001 values (which select
 * between bank 0 (built-in) and expansion bank 1 or between bank0 and expansion bank 2 respectively. '-1' in the table means
 * that the RAM expansion is not used and the access goes to the internal RAM. The rows in the table are selected according to
 * the upper two address bits of the memory access.
 * 
 * -------------------------------------------------------------------------------------------------------------------------------:---------
 * Address\cccbbb 000000 000001 000010 000011 000100 000101 000110 000011 001000 001001 001010 001011 001100 001101 001110 001111 : >001111
 * -------------------------------------------------------------------------------------------------------------------------------:---------
 * 1100-1111       -1      3      3      3      -1     -1     -1     -1     -1     7       7      7     -1     -1     -1     -1   :     -
 * 1000-1011       -1      -1     2      -1     -1     -1     -1     -1     -1     -1      6      -1    -1     -1     -1     -1   :     -
 * 0100-0111       -1      -1     1      -1     0      1       2      3     -1     -1      5      -1    4      5      6      7    :     -
 * 0000-0011       -1      -1     0      -1     -1     -1     -1     -1     -1     -1      4      -1    -1     -1     -1     -1   :     -
 * -------------------------------------------------------------------------------------------------------------------------------:---------
 * 
 * ROM is accessed whenever ROMEN_B goes low. Addresses with Address[15] set go to upper (foreground)
 * ROMs and those with Address[15] low go to lower ROM.
 *       
 */
#ifdef DEBUG
#include <string.h>
#endif

#define DEBUG            1
//#define TEENSY_ASM       0
#define LOWER_ROM_ENABLE 0
#define RAM_EXP_ENABLE   0

// ----- Some string processing definitions to enable use of C tokens in assembler statements
#define str(a) st(a)    
#define st(a) #a   

// --- Port allocations 
// Data direction is the opposite of ChipKit - on Teensy 0=input, 1=Output
#define  CTRLADRHI_IN   GPIOB_PDOR
#define  CTRLADRHI_MODE GPIOB_PDDR
#define  DATACTRL_IN    GPIOC_PDIR
#define  DATACTRL_OUT   GPIOC_PDOR
#define  DATACTRL_MODE  GPIOC_PDDR
#define  ADR_LO_IN      GPIOD_PDOR
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

#define ADDR_HI         0x00FF0000   // bits 16-23 on control input data
#define ADDR_LO         0x000000FF   // bits  0-7 on address input
#define ADR_13_RAW      0x00200000   // bit 13 in raw position in ctrl/adr word - ie bit 21
#define ADR_15_RAW      0x00800000   // bit 15 in raw position in ctrl/adr word - ie bit 23

#define MASK            (MREQ_B|IOREQ_B|WR_B|RAMRD_B|ROMEN_B )
#define VALIDRAMSELMASK 0x00C0       // Top two bits of Data must be set for a valid ROM selection
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

#define ADR_HI_BYTE     ((ctrladr&ADDR_HI)>>24)&0xFF
#define ADDRESS_IN      ((ADR_HI_BYTE<<8)|((ADR_LO_IN)&0xFF))

#define ROMSEL          !(ctrladr&(IOREQ_B|ADR_13_RAW|WR_B))
#define RAMSEL          !(ctrladr&(IOREQ_B|ADR_15_RAW|WR_B))
#define HIROMRD         (!(ctrladr&(ROMEN_B)))&&(ctrladr&ADR_15_RAW)
#define LOROMRD         !(ctrladr&(ROMEN_B|ADR_15_RAW))
#define ROMRD           !(ctrladr&(ROMEN_B))
#define RAMRD           !(ctrladr&(RAMRD_B))
#define RAMWR           !(ctrladr&(MREQ_B|WR_B))

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

#ifdef LOWER_ROM_ENABLE
// This will be the content of the AMSTRAD Firmware ROM to allow replacement
// of a CPC464 or 664 ROM with that from a 6128.
const char lowerrom[ROMSIZE] = {
#ifdef DEBUG
0,1,2,3,4,5,6,7,8,9
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
  // Set all pins to input mode
  CTRLADRHI_MODE = 0x00000000;
  DATACTRL_MODE  = 0x00000000;
  DATACTRL_OUT   = 0x00000000;
  ADR_LO_MODE    = 0x00000000;
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
  register int ctrladr;
  register int address;
  int ramblock;
  int datain;
  int ram_lut_row = 0;
  while (true) {


#ifdef TEENSY_ASM
    // FIXME - replace with ARM code !
    asm volatile (  
            ".set noreorder\n"                           // ensure assembler doesn't get optimized or put NOPS after branches etc   
            "lui  $11, " str(CTRLDATA_IN_HI) "\n\t"  
            "lui  $10, " str(ADDR_IN_HI) "\n\t"          
            "li   $9, " str(MASK) "\n\t"          
            "lw   %1, " str(CTRLDATA_IN_LO) "($11)\n"    // sample CTRLDATA directly into C register var
                         
  "exitloop: and  $8, %1, $9\n\t"                        // AND ctrldata with the signal mask 
            "bne  $8, $9, exitloop\n\t"                  // Loop again 'til outputs go active low
            "lw   %1, " str(CTRLDATA_IN_LO) "($11)\n\t"  // sample CTRLDATA again (this is actually inside the loop in the branch delay slot!)
               
            "li   $8, 0xFFFF\n\t"                        // Setup tristate mask 
            "sw   $8, " str(CTRLDATA_MODE) "\n"          // Apply tristate mask on exit of loop

 "entryloop: and  $8, %1, $9\n\t"                        // AND ctrldata with the signal mask
            "beq  $8, $9, entryloop\n\t"                 // if not active low signals then loop again
            "lw   %1, " str(CTRLDATA_IN_LO) " ($11)\n\t" // sample CTRLDATA again actually inside the loop (in branch delay slot)

            "lw   %0, " str(ADDR_IN_LO) "($10)\n\t"      // sample ADDR_IN 
        :   "=r" (address), "=r" (ctrldata)              // Outputs list (C variables in registers)
        :                                                // No inputs
        :   "$8", "$9", "$10", "$11"                     // Register clobber list
        );
#else    
    while ( ((ctrladr = CTRLADRHI_IN)&MASK) != MASK ) {} // Wait for control signals to go inactive
    DATACTRL_MODE = 0x0;                                 // Tristate all data and control outputs
    while ( ((ctrladr = CTRLADRHI_IN)&MASK) == MASK ) {} // Now wait for control signals to become active
#endif

    address = ADDRESS_IN;

#ifdef RAM_EXP_ENABLE
    if ( RAMRD ) {
      if ( ram_lut_col > -1 ) {
        ram_lut_row = (address&0xC000)>>14;
        ramblock = ramblocklut[ram_lut_row][ram_lut_col];
        if (ramblock>=0 ) {
          DATACTRL_OUT = ram[ramblock][address] | RAMDIS;
          DATACTRL_MODE = DATA | RAMDIS;
        }
      }
   } else
#endif
    if ( HIROMRD && validrom ) {
      DATACTRL_OUT = upperrom[(romnum<<14)|(address&0x3FFF)] | ROMDIS;
      DATACTRL_MODE = DATA | ROMDIS;
#ifdef LOWER_ROM_ENABLE
    } else if ( LOROMRD ) {
      DATACTRL_OUT = lowerrom[address&0x3FFF] | ROMDIS;
      DATACTRL_MODE = DATA | ROMDIS;
#endif
#ifdef RAM_EXP_ENABLE
    } else if ( RAMWR ) {      
      if ( ram_lut_col > -1 ) {
        datain  = DATACTRL_IN&DATA;
        ram_lut_row = (address&0xC000)>>14                 ;
        ramblock = ramblocklut[ram_lut_row][ram_lut_col] ;
        if (ramblock>=0 ) {
          DATACTRL_OUT =  RAMDIS;
          DATACTRL_MODE = RAMDIS;
          ram[ramblock][address] = datain;
          }
      }
#endif      
    } else if ( ROMSEL ) {
      datain  = DATACTRL_IN&DATA;
      romnum = ((datain&0xF)) ;     // only allow 16 ROMS
      validrom = valid_upperrom[romnum];
    } 
#ifdef RAM_EXP_ENABLE    
    else if ( RAMSEL ) {
      datain  = DATACTRL_IN&DATA;    
      if ((datain&VALIDRAMSELMASK)==VALIDRAMSELMASK) {
        // Now validate that only codes 000xxx or 001xxx are used - other codes unsupported (default to machine built-in RAM)
        if ((datain&RAMCODEMASK) <=0x08) {
          ram_lut_col   = datain&RAMBANKMASK; // Set column for the RAM block lookup
        } else {
          ram_lut_col   = -1;          // disable RAM expansion
        }
      }
    }
#endif    
  }
}
