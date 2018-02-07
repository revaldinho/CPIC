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
   o http://www.cpcwiki.eu/imgs/8/83/DK%27Tronics_Peripheral_-_Technical_Manual_%28Edition_1%29.pdf

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

 Per-pin control notes
 ---------------------
 
 val = 0b0000_0000_0000_0000_0000_0001_0100_0100;
                                   ---  -    -
                                    \    \    \_ Slew rate enable: 1=Fast, 0=Slow 
                                     \    \_____ Drive strength  : 1=High, 0=Low
                                      \_________ Pin control     : 001 = GPIO

 // Set ROMDIS/RAMDIS to have fast slew + high drive

 PORTB_PCR8 = 0x00000144;
 PORTB_PCR9 = 0x00000144;

 OR can use the PORTX_GPCLR to set bits globally for an entire port, ie

 PORTB_GPCLR = 0x0144FFFF;

 ...picks bits from lower 16 bits and writes to selected bits[15:0] identified in the upper 16bit field.
 e.g. set all bits in a port to GPIO control and default to slow slew, low drive

 PORTB_GPCLR = 0x01440100 ;

  Worst case timings from Zilog 8400A Datasheet
  ---------------------------------------------
  
                :<----- 250ns------>:<----- 250ns------>:<----- 250ns------>:
                :        T1         :       T2/3/4      :       T3/4/5
                :_________          :_________          :_________
  CLK __________/         \_________/         \_________/         \_________
                : max:    .                             :
                : 110:    .                             :
      __________:____:____._____________________________:____________________
  ADR __________:____X____._____________________________:____________________
                :         .                             :
                :max:     .                             :
                :100:     .                             :
      __________:___:     .                             :____________________
  M1*           :   \_____._____________________________/
                :         .                             :
                :         . max.                        :
                :         . 75 .                        :
      __________:_________.____.                        :___________________
  IOREQ*        :         .    \________________________/
                :         . max .                       :
                :         . 85  .                       :
      __________:_________._____.                       :___________________
  MREQ*         :         .     \_______________________/
                :         . max                         :
                :         . 95   .                      :
      __________:_________.______.                      :___________________
  RD*           :         .      \______________________/
                :         .                             :
                :         .                         :Min: 
                :         .                         :30 :  
                :         .                          ______
  DATA ---------:-----------------------------------<______>-----------------
  
  ie time from M1* -> other control signals value ~ (125-100)+95 = 120ns.
     If triggering on M1, need to resample control signals at least 120ns (~15 cycles @ 120MHz) later 
     to determine whether ROM or RAM is being accessed!
  
  Instr Fetch time: M1* (T1) -> clock rise (T3)
  
                = (250-100)+ (250-30) = 370ns
  
  Data Read time: MREQ* (T1) -> clock rise (T4)
  
                = (250-125-85) + 250 + (250-30) = 510ns
  
  IO Read time: IOREQ* (T1) -> clock rise (T5)  
  
                = (250-125-75) + 250 + 250 + (250-30) = 770ns

  while ((ctrladr=PORTX & MASK) == MASK ) {}
  get address_hi                          // ~7 instr after sample here = 58ns later but address already valid
  compute address = f(ctrladr,address_hi)
  pre-fetch ROM data (choose upper or lower) - longest latency
  resample ctrladr                        // resample ctrl signals at least 120ns after original trigger
  
  ... proceed as normal through if-then-else
  
  Reset oscillator, pullups on all control inputs signal, and pulldowns on ROMDIS, RAMDIS
  
  o M1* -> ROMDIS time   < 370ns   RD_B -> ROMDIS time < 120ns[1]    M1, RD_B    -> Osc  ROMEN* -> low  
  o M1* -> RAMDIS time   < 370ns   RD_B -> RAMDIS time < 120ns[1]    M1, RD_B    -> Osc  RAMRD* -> low
  o MREQ* -> ROMDIS time < 510ns   RD_B -> ROMDIS time < 120ns[1]    MREQ*, RD_B -> Osc  ROMEN* -> low
  o MREQ* -> RAMDIS time < 510ns   RD_B -> RAMDIS time < 120ns[1]    MREQ*, RD_B -> Osc  RAMRD* -> low
  
  [1] only for debug-> RD_B doesn't normally tristate these signals, but does to make the event easy
      to observe when DEBUG=1
  
  Osc @ 0.5MHz -> 1000ns high/low time
*/
#include <string.h>

// ----- Some string processing definitions to enable use of C tokens in assembler statements
#define str(a) st(a)
#define st(a) #a

//#define LOWER_ROM_ENABLE 1

// --- Port allocations
// Data direction is the opposite of ChipKit - on Teensy 0=input, 1=Output
// Some absolute addresses for the assembler code
#define GPIO_BASE          0x400FF000
#define GPIOB_PDIR_OFFSET  0x50
#define GPIOD_PDIR_OFFSET  0xD0
#define GPIOC_PDDR_OFFSET  0x94
#define GPIOC_PDOR_OFFSET  0x80

#define GPIOB_PDIR_ADDR    0x400FF050
#define GPIOD_PDIR_ADDR    0x400FF0D0
#define GPIOC_PDDR_ADDR    0x400FF094
#define GPIOC_PDOR_ADDR    0x400FF080

// Arduino defined tokens (including unit32_t*) casts for C
#define  CTRLADRHI_IN     GPIOB_PDIR
#define  CTRLADRHI_MODE   GPIOB_PDDR
#define  DATACTRL_IN      GPIOC_PDIR
#define  DATACTRL_OUT     GPIOC_PDOR
#define  DATACTRL_MODE    GPIOC_PDDR
#define  DATACTRL_CLEAR   GPIOC_PCOR
#define  ADRLO_DINLO_IN        GPIOD_PDIR
#define  ADRLO_DINLO_MODE      GPIOD_PDDR

// --- Bit allocations
#define DATA              0x000000FF
#define ROMVALID          0x00000100
#define WAIT_B            0x00000400

#define MREQ_B            0x00000001 // Port B0
#define IOREQ_B           0x00000002
#define WR_B              0x00000004
#define RAMRD_B           0x00000008
#define ROMEN_B           0x00000010
#define M1_B              0x00000020 // Port B5 
#define RD_B              0x00000400 // Port B10 (because 6-9 not available)  
#define CLK4              0x00000800
#define MASK              0x00000424 // Mask on M1,RD,WR

#define ADDR_HI           0x00FF0000  // bits 16-23 on control input data
#define ADDR_LO           0x000000FF  // bits  0-7 on address input
#define ADR_13_RAW        0x00200000  // bit 13 in raw position in ctrl/adr word - ie bit 21
#define ADR_14_RAW        0x00400000  // bit 14 in raw position in ctrl/adr word - ie bit 22
#define ADR_15_RAW        0x00800000  // bit 15 in raw position in ctrl/adr word - ie bit 23
#define DIN_LO            0x0000F000  // bits 12..15

#define VALIDRAMSELMASK   0x000000C0  // Top two bits of Data must be set for a valid RAM selection
#define RAMCODEMASK       0x00000038  // Next 3 bits are the 'code'
#define RAMBANKMASK       0x00000007  // Bottom 3 bits are the 'code'

// ---- Constants and macros
#define MAXROMS           8
#define RAMBLKSIZE        16384
#define ROMSIZE           16384

#define ROMSEL            (!(ctrladrhi&(IOREQ_B|ADR_13_RAW|WR_B)))
#define HIROMRD           ((ctrladrhi&(ROMEN_B|ADR_14_RAW))==(ADR_14_RAW))

// Global variables
const char upperrom[MAXROMS*ROMSIZE] = {
//#include "/Users/richarde/Documents/Development/git/CPiC/src/BASIC_1.0.CSV"
//#include "/Users/richarde/Documents/Development/git/CPiC/src/PROTEXT.CSV"
//#include "/Users/richarde/Documents/Development/git/CPiC/src/UTOPIA.CSV"
//#include "/Users/richarde/Documents/Development/git/CPiC/src/BCPL.CSV"
//#include "/Users/richarde/Documents/Development/git/CPiC/src/CWTA.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/ALL_ZEROS.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/CWTA.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/MAXAM114.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/PROTEXT.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/MAXAM15.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/UTOP107.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/EXBASIC.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/ALL_ZEROS.CSV"
};

char ram[MAXROMS*ROMSIZE] ;

const boolean valid_upperrom[MAXROMS] = {
  false, true, true, true,
  false, false, false, false
};

void setup() {
  // Set all pins to input mode using pinMode instructions as easy way of setting up
  // GPIO control - can use port registers after this initialization
  for (int i = 0 ; i < 58 ; i++ ) {
    pinMode(i, INPUT);
  }
  memcpy( ram, upperrom, MAXROMS*ROMSIZE);
  DATACTRL_OUT  = 0x00 |  0     ; 
  DATACTRL_MODE = 0xFF |  ROMVALID ; // Always drive out - OE taken care of externally
}
void loop() { 
  int ctrladrhi;       
  int address;         
  char *romptr = NULL;
  char romdata = 0;  
  int romnum;
  
  while (true) {
    while ( ((ctrladrhi=CTRLADRHI_IN)&(M1_B|RD_B|WR_B)) == (M1_B|RD_B|WR_B) ) { }
    address =((ctrladrhi>>8)&0x3F00)|(ADRLO_DINLO_IN&0x00FF);
    romdata = *(romptr+address) ;
    ctrladrhi=CTRLADRHI_IN;
    if (HIROMRD && romptr) {   
      DATACTRL_OUT  = romdata | ROMVALID   ;  
      while ( !(CTRLADRHI_IN&RD_B) ) {} 
    } else if ( ROMSEL ) {
      //int romnum = (DATACTRL_IN)&0x07;         // Limit to 8 ROMS
      romnum = (ADRLO_DINLO_IN&0x0F000)>>12; // Limit to 8 ROMS and kill LSB which appears stuck at zero
      if (valid_upperrom[romnum]) {
        romptr = (char *) &(ram[romnum<<14]) ;
        DATACTRL_OUT  = 0x00 | ROMVALID   ;  
      } else {
        romptr = NULL;
        DATACTRL_OUT  = 0x00 | 0   ;  
      }
      while ( ! ((CTRLADRHI_IN)&WR_B) ) {}
    } 
  }
}
            



