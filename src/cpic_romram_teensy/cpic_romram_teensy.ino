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

//#define TEENSY_ASM       1
//#define LOWER_ROM_ENABLE 1
//#define RAM_EXP_ENABLE   1

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
#define  ADR_LO_IN        GPIOD_PDIR
#define  ADR_LO_MODE      GPIOD_PDDR

// --- Bit allocations
#define DATA              0x000000FF
#define ROMDIS            0x00000100
#define RAMDIS            0x00000200
#define OE_B              0x00000400
#define DIR_CPCTOTS       0x00000800
#define DIR_TSTOCPC       0x00000000
#define DIR               0x00000800


#define MREQ_B            0x00000001 // Port B0
#define IOREQ_B           0x00000002
#define WR_B              0x00000004
#define RAMRD_B           0x00000008
#define ROMEN_B           0x00000010
#define M1_B              0x00000020 // Port B5 
#define RD_B              0x00000400 // Port B10 (because 6-9 not available)  
#define CLK4              0x00000800
#define MASK              0x00000013 // Mask on subset temporarily

#define ADDR_HI           0x00FF0000  // bits 16-23 on control input data
#define ADDR_LO           0x000000FF  // bits  0-7 on address input
#define ADR_13_RAW        0x00200000  // bit 13 in raw position in ctrl/adr word - ie bit 21
#define ADR_14_RAW        0x00400000  // bit 14 in raw position in ctrl/adr word - ie bit 22
#define ADR_15_RAW        0x00800000  // bit 15 in raw position in ctrl/adr word - ie bit 23

#define VALIDRAMSELMASK   0x000000C0  // Top two bits of Data must be set for a valid RAM selection
#define RAMCODEMASK       0x00000038  // Next 3 bits are the 'code'
#define RAMBANKMASK       0x00000007  // Bottom 3 bits are the 'code'

// ---- Constants and macros
#define MAXROMS           8
#define RAMBLKSIZE        16384
#define ROMSIZE           16384

#define ROMSEL            (!(ctrladrhi&(IOREQ_B|ADR_13_RAW|WR_B)))
#define RAMSEL            (!(ctrladrhi&(IOREQ_B|ADR_15_RAW|WR_B)))
#define HIROMRD           ((ctrladrhi&(ROMEN_B|ADR_14_RAW))==(ADR_14_RAW))
#ifdef  LOWER_ROM_ENABLE
#define LOROMRD           (!(ctrladrhi&(ROMEN_B|ADR_14_RAW)))
#else
#define LOROMRD         false
#endif
#define RAMRD             (!(ctrladrhi&(RAMRD_B)))
#define RAMWR             (!(ctrladrhi&(MREQ_B|WR_B)))

// Global variables
const char upperrom[MAXROMS*ROMSIZE] = {
//#include "/Users/richarde/Documents/Development/git/CPiC/src/BASIC_1.0.CSV"
//#include "/Users/richarde/Documents/Development/git/CPiC/src/PROTEXT.CSV"
//#include "/Users/richarde/Documents/Development/git/CPiC/src/UTOPIA.CSV"
//#include "/Users/richarde/Documents/Development/git/CPiC/src/BCPL.CSV"
//#include "/Users/richarde/Documents/Development/git/CPiC/src/CWTA.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/ALL_ZEROS.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/CWTA.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/MAXAM.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/UTOPIA.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/PROTEXT.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/ALL_ZEROS.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/ALL_ZEROS.CSV"
#include "/Users/richarde/Documents/Development/git/CPiC/src/ALL_ZEROS.CSV"
};

char ram[MAXROMS*ROMSIZE] ;

const boolean valid_upperrom[MAXROMS] = {
  false, true, false, false,
  false, false, false, false,
};

#ifdef LOWER_ROM_ENABLE
// This will be the content of the AMSTRAD Firmware ROM to allow replacement
// of a CPC464 or 664 ROM with that from a 6128.
const char lowerrom[] = {
#include "/Users/richarde/Documents/Development/git/CPiC/src/OS_464.CSV"
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
  for (int i = 0 ; i < 54 ; i++ ) {
    pinMode(i, INPUT);
  }


  memcpy( ram, upperrom, MAXROMS*ROMSIZE);
  // Initialize 245 to drive from CPC to TS, set ROM/RAMDIS low, disable 245 driver
  DATACTRL_OUT  = 0x00 |  0     |  0     | 0    | DIR_CPCTOTS ; 
  DATACTRL_MODE = 0x00 | ROMDIS |RAMDIS  | OE_B | DIR; 
}

void loop() {
#ifdef RAM_EXP_ENABLE
  int ramblock;
  int ram_lut_row = 0;
#endif  
  int romnum = 0;
  register int ctrladrhi;       // register for use with assembler code
  register int address;         // register for use with assembler code
  char *romptr = NULL;
  char romdata = 0;  
  while (true) {
#ifdef TEENSY_ASM
   asm volatile (

        // NB using %[adrortmp] register (ie C address variable) as a temp variable in the upper portion of the code to save clobbering
        // more registers. This will be hold a valid address on exit from the assembly code.

        "ldr     r9, =" str(GPIO_BASE) "\n"                           // Tristate loop - tristate databus as soon as rd_b goes high  
"loop0:  ldr     %[ctrladrhi], [r9, #" str(GPIOB_PDIR_OFFSET) "]\n"   // Sample control signals                           
        "lsls    %[adrortmp], %[ctrladrhi], #21\n\t"                  // Check READ bit only  by shifting 21 places into MSB to see if we can tristate databus (which will be earlier than ROMEN_B and other derived signals)
        "bpl.n   loop0\n\t"                                           // Loop again if rd_b is zero
        "strb    %[adrortmp],[r9, #"  str(GPIOC_PDDR_OFFSET) "]\n"    // tristate the data bits only (lowest byte) normally if RD_B is high                                                                                                                                
                                                                      // Exit loop - now wait for all ctrl signals to go inactive (presampled above)
"loop1:  and     %[adrortmp], %[ctrladrhi],  #" str(MASK) "\n\t"      // Mask off control bits 
        "subs    %[adrortmp], %[adrortmp], #" str(MASK) "\n\t"        // subtract mask from %[adrortmp]. %[adrortmp] will be all-zeroes on exit
        "ldr     %[ctrladrhi], [r9, #" str(GPIOB_PDIR_OFFSET) "]\n\t" // Sample control bits again (masking latency if looping)
        "bne     loop1\n\t"                                            
        "str     %[adrortmp],[r9, #"  str(GPIOC_PDDR_OFFSET) "]\n"    // tristate the data and ctrl outputs by writing the all-zeroes from %[adrortmp]      
        
                                                                      // Entry Loop - wait for clock to go low
"loop2:  ldr     %[ctrladrhi], [r9, #" str(GPIOB_PDIR_OFFSET) "]\n\t" // Sample control bits and address high byte
        "ldrb    %[adrortmp], [r9, #" str(GPIOD_PDIR_OFFSET) "]\n\t"  // sample adr low byte only
        "lsls    r10, %[ctrladrhi], #20\n\t"                          // check clock by shifting 20 places into MSB
        "bmi.n   loop2\n\t"                                           // If not zero then loop again

        "uxtb    %[adrortmp], %[adrortmp]\n\t"                        // clear all upper bits  of low byte (remember we used this reg as a temp var above)
        "ubfx    %[ctrladrhi], %[ctrladrhi], #16,#8\n\t"              // isolate the 8 high order address bits 
        "orr     %[adrortmp], %[adrortmp], %[ctrladrhi], lsr #8 \n\t" // Or high and low together simultaneously moving the high bits into place

                                                                      // speculatively prefetch ROM data from slow flash memory before resampling control signals
        "ubfx    %[ctrladrhi], %[adrortmp], #0,#14\n\t"               // Get 16K addr offset by clearing top two bits and using ctrladrhi as a temp because we are about to resample in a few instr time below
#ifdef LOWER_ROM_ENABLE
        "tst      %[adrortmp], #0x8000\n\t"                           // Check full address MSB to see if we are in upper or lower ROM
        "ldrbeq   %[romdata], [%[romptr], %[ctrladrhi]]\n\t"          // Read a byte from upper ROM if full address MSB is set
        "ldrbne   %[romdata], [%[lowerrom], %[ctrladrhi]]\n\t"        // Read a byte from lower ROM if full address MSB is not set
#else
        "ldrb     %[romdata], [%[romptr], +%[ctrladrhi]]\n\t"         // Read a byte from upper ROM
#endif
        "ldr     %[ctrladrhi], [r9, #" str(GPIOB_PDIR_OFFSET) "]\n\t" // Finally, resample ctrl bits again, assuming they are valid by this point (latest ~ 100ns after clock falling edge)
        :   [ctrladrhi] "=r" (ctrladrhi), [adrortmp] "=r" (address), [romdata] "=r" (romdata)  // Outputs list (C variables in registers)
#ifdef LOWER_ROM_ENABLE
        :   [romptr] "r" (romptr), [lowerrom] "r" (lowerrom)                                   // Inputs list
#else
        :   [romptr] "r" (romptr)                                                              // Inputs list
#endif
        :   "r9", "r10"                                                                        // Register clobber list
      );
#else
//
// Recipe 1 - trigger off CLK4
//
//    // Wait 'til CLK4 goes high and sample low address bits shortly after
//    while (  !((ctrladrhi=CTRLADRHI_IN)&(CLK4)) ) {}
//    address =((ctrladrhi>>8)&0xFF00)|(ADR_LO_IN&0x00FF);
//    // Now wait for falling edge of clock and resample control bits shortly after
//    while (  ((ctrladrhi=CTRLADRHI_IN)&(CLK4)) ) {}
//

// Recipe 2 - trigger off M1 or RD or WR
    while ( ((ctrladrhi=CTRLADRHI_IN)&(M1_B|RD_B|WR_B)) == (M1_B|RD_B|WR_B) ) {}
    address =((ctrladrhi>>8)&0xFF00)|(ADR_LO_IN&0x00FF);
#ifdef LOWER_ROM_ENABLE    
      romdata = (address & 0x4000)? *(romptr+(address&0x3FFF)) : lowerrom[address&0x3FFF]  ;
#else
      romdata = *(romptr+(address&0x3FFF)) ;
#endif
    ctrladrhi=CTRLADRHI_IN;
#endif

    if ( LOROMRD || (HIROMRD && romptr)) {   
      DATACTRL_OUT  = romdata | ROMDIS |RAMDIS  | 0    | 0 ; 
      DATACTRL_MODE = 0xFF    | ROMDIS |RAMDIS  | OE_B | DIR; 
      while ((CTRLADRHI_IN&(MREQ_B|RD_B)) !=(MREQ_B|RD_B) ) {}  
      DATACTRL_MODE = 0x00    | ROMDIS |RAMDIS  | OE_B | DIR; 
      DATACTRL_OUT  = romdata | 0      |  0     | 0    | DIR_CPCTOTS ;
    } else if ( ROMSEL ) {
      romnum = (DATACTRL_IN)&0x0F;
      if (valid_upperrom[romnum]) {
        //romptr = (char *) &(upperrom[romnum<<14]) ;
        romptr = (char *) &(ram[romnum<<14]) ;
      } else {
        romptr = NULL;
      }
     while ( ! ((CTRLADRHI_IN)&WR_B) ) {}
    }
    
#ifdef RAM_EXP_ENABLE
    else if ( RAMRD || RAMWR ) {
      if ( ram_lut_col > -1 ) {
         ram_lut_row = (address & 0xC000) >> 14;
         ramblock = ramblocklut[ram_lut_row][ram_lut_col];
         if (ramblock >= 0 ) {
           if (RAMWR) {
             DATACTRL_OUT =  RAMDIS;   
             DATACTRL_MODE = RAMDIS;
             ram[ramblock][address&0x3FFF] = DATACTRL_IN & DATA;
           } else {
             DATACTRL_OUT = ram[ramblock][address&0x3FFF] | RAMDIS;
             DATACTRL_MODE = DATA | RAMDIS;
           }
         }
      }
    } else if ( RAMSEL ) {
      int datain = DATACTRL_IN & DATA;
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
            



