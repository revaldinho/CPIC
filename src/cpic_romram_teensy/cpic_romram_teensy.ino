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
  
  
  => Need to sample M1 for instruction cycle, and then address and then all control signals again to
     to get valid settings for RAMRD*, ROMEN* etc   
  
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

// ----- Some string processing definitions to enable use of C tokens in assembler statements
#define str(a) st(a)
#define st(a) #a

#define DEBUG            1
//#define TEENSY_ASM       1
#define LOWER_ROM_ENABLE 1
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
#define WAIT_B            0x00000400

#define MREQ_B            0x00000001 // Port B0
#define IOREQ_B           0x00000002
#define WR_B              0x00000004
#define RAMRD_B           0x00000008
#define ROMEN_B           0x00000010
#define M1_B              0x00000020 // Port B5 
#define RD_B              0x00000400 // Port B10 (because 6-9 not available)  
#define MASK              0x0000003F // All above control signals inactive (but ignore RD_B in aggregate)

#define ADDR_HI           0x00FF0000  // bits 16-23 on control input data
#define ADDR_LO           0x000000FF  // bits  0-7 on address input
#define ADR_13_RAW        0x00200000  // bit 13 in raw position in ctrl/adr word - ie bit 21
#define ADR_15_RAW        0x00800000  // bit 15 in raw position in ctrl/adr word - ie bit 23

#define VALIDRAMSELMASK   0x000000C0  // Top two bits of Data must be set for a valid RAM selection
#define RAMCODEMASK       0x00000038  // Next 3 bits are the 'code'
#define RAMBANKMASK       0x00000007  // Bottom 3 bits are the 'code'

// ---- Constants and macros
#define MAXROMS           16
#define RAMBLKSIZE        16384

#ifdef DEBUG
#define ROMSIZE           32
#else
#define ROMSIZE           16384
#endif

#define ROMSEL            (!(ctrladrhi&(IOREQ_B|ADR_13_RAW|WR_B)))
#define RAMSEL            (!(ctrladrhi&(IOREQ_B|ADR_15_RAW|WR_B)))
#define HIROMRD           ((!(ctrladrhi&(ROMEN_B)))&&(ctrladrhi&ADR_15_RAW))
#ifdef  LOWER_ROM_ENABLE
#define LOROMRD           (!(ctrladrhi&(ROMEN_B|ADR_15_RAW)))
#else
#define LOROMRD         false
#endif
#define ROMRD             (!(ctrladrhi&(ROMEN_B)))
#define RAMRD             (!(ctrladrhi&(RAMRD_B)))
#define RAMWR             (!(ctrladrhi&(MREQ_B|WR_B)))

// Global variables
const char upperrom[MAXROMS][ROMSIZE] = {
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
  //PORTC_PCR8 = 0x0144; // switch on GPIO control, high drive, fast slew for ROMDIS
  //PORTC_PCR9 = 0x0144; // as above for RAMDIS
}

void loop() {
#ifdef RAM_EXP_ENABLE
  #ifdef DEBUG
  int ram_lut_col = 2;
  #endif
  int ramblock;
  int ram_lut_row = 0;
#endif  
  int romnum = 0;
  register int ctrladrhi;       // register for use with assembler code
  register int address;         // register for use with assembler code

  char *romptr = (char *) upperrom[0];
  char romdata;
  while (true) {
    while ( ((ctrladrhi = CTRLADRHI_IN)&RD_B) != RD_B ) {}                                         // Tristate databus as soon as RD_B is high 
#ifdef DEBUG    
    DATACTRL_OUT  = 0x0000;                                                                        // Drive ROM/RAMDIS and data signals low to observe time cleanly
#else
    DATACTRL_MODE &= ~DATA;                                                                        // tristating all data signals only normally on RD_B deassertion
#endif

#ifdef TEENSY_ASM
   asm volatile (

        // NB using %[adrortmp] register (ie C address variable) as a temp variable in the upper portion of the code to save clobbering
        // more registers. This will be hold a valid address on exit from the assembly code.

        "ldr     r9, =" str(GPIO_BASE) "\n\t"                         // Tristate loop - tristate databus as soon as rd_b goes high 
        "ldr     %[ctrladrhi], [r9, #" str(GPIOB_PDIR_OFFSET) "]\n"   // Sample control signals                           
"loop0:  ands    %[adrortmp], %[ctrladrhi], #" str(RD_B) "\n\t"       // Check READ bit only to see if we can tristate databus (which will be earlier than ROMEN_B and other derived signals)
        "ldr     %[ctrladrhi], [r9, #" str(GPIOB_PDIR_OFFSET) "]\n\t" // resample control signals - mask latency if looping, and use for next loop
        "beq     loop0\n\t"                                           // Loop again if not set (ie AND result is zero)
                                                                         
#ifdef DEBUG                                                             
        "str     %[adrortmp],[r9, #"  str(GPIOC_PDOR_OFFSET) "]\n"    // zero ROMDIS/RAMDIS for easy observation in debug mode
#else                                                                 
        "strb    %[adrortmp],[r9, #"  str(GPIOC_PDDR_OFFSET) "]\n"    // tristate the data bits only (lowest byte) normally if RD_B is high
#endif                                                                                                                                      
                                                                      // Exit loop - now wait for all ctrl signals to go inactive (presampled above)
"loop1:  and     %[adrortmp], %[ctrladrhi],  #" str(MASK) "\n\t"      // Mask off control bits 
        "subs    %[adrortmp], %[adrortmp], #" str(MASK) "\n\t"        // subtract mask from %[adrortmp]. %[adrortmp] will be all-zeroes on exit
        "ldr     %[ctrladrhi], [r9, #" str(GPIOB_PDIR_OFFSET) "]\n\t" // Sample control bits again (masking latency if looping)
        "bne     loop1\n\t"                                            
        "str     %[adrortmp],[r9, #"  str(GPIOC_PDDR_OFFSET) "]\n"    // tristate the data and ctrl outputs by writing the all-zeroes from %[adrortmp]      
        
                                                                      // Entry Loop - wait for any control signals to go active low
"loop2:  ldr     %[ctrladrhi], [r9, #" str(GPIOB_PDIR_OFFSET) "]\n\t" // Sample control bits
        "and     %[adrortmp], %[ctrladrhi], #" str(MASK) "\n\t"       // Mask off control bits
        "teq     %[adrortmp], #" str(MASK) "\n\t"                     // Check if any active low
        "beq     loop2\n\t"                                           // If not then loop again

        "ldr     %[ctrladrhi], [r9, #" str(GPIOB_PDIR_OFFSET) "]\n\t" // Resample address bits again in case triggered by M1_B
        "ldrb    %[adrortmp], [r9, #" str(GPIOD_PDIR_OFFSET) "]\n\t"  // sample adr low byte only
        "and     %[adrortmp], %[adrortmp], #0x00FF\n\t"               // clear all other bits (remember we used this reg as a temp var above)
        "lsr     %[ctrladrhi], %[ctrladrhi], #8\n\t"                  // Move high addr bits into correct location
        "and     %[ctrladrhi], %[ctrladrhi], #0x00FF00\n\t"            // mask off high address bits 
        "orr     %[adrortmp], %[adrortmp], %[ctrladrhi]\n\t"          // Or high and low together

                                                                      // speculatively prefetch ROM data from slow flash memory before resampling control signals
        "bic      %[ctrladrhi], %[adrortmp], #0xC000\n\t"             // Get 16K addr offset by clearing top two bits and using ctrladrhi as a temp because we are about to resample in a few instr time below
#ifdef LOWER_ROM_ENABLE
        "tst      %[adrortmp], #0x8000\n\t"                           // Check full address MSB to see if we are in upper or lower ROM
        "ldrbeq   %[romdata], [%[romptr], %[ctrladrhi]]\n\t"         // Read a byte from upper ROM if full address MSB is set
        "ldrbne   %[romdata], [%[lowerrom], %[ctrladrhi]]\n\t"       // Read a byte from lower ROM if full address MSB is not set
#else
        "ldrb     %[romdata], [%[romptr], +%[ctrladrhi]]\n\t"         // Read a byte from upper ROM
#endif
        "ldr     %[ctrladrhi], [r9, #" str(GPIOB_PDIR_OFFSET) "]\n\t" // Finally, resample ctrl bits again, assuming they are valid by this point if an early M1 was the original trigger
        :   [ctrladrhi] "=r" (ctrladrhi), [adrortmp] "=r" (address), [romdata] "=r" (romdata)  // Outputs list (C variables in registers)
#ifdef LOWER_ROM_ENABLE
        :   [romptr] "r" (romptr), [lowerrom] "r" (lowerrom)                                   // Inputs list
#else
        :   [romptr] "r" (romptr)                                                              // Inputs list
#endif
        :   "r9"                                                                               // Register clobber list
      );
#else
    // Tristate databus as soon as the RD_B signal goes high. 
    while ( ((ctrladrhi = CTRLADRHI_IN)&RD_B) != RD_B ) {}
#ifdef DEBUG
    DATACTRL_OUT = 0x0;      // Drive ROMDIS/RAMDIS low for clean timing observation on scope
#else
    DATACTRL_MODE &= ~DATA;  // tristate DATA bits only normally 
#endif
    // Wait for all control signals to go inactive (for read and write), tristating all data and control outputs 
    // incl. ROM/RAM disables. Now wait for any control signals to become active - M1 trigger most critical
    while (((ctrladrhi = CTRLADRHI_IN)&MASK)    != MASK ) {}                                     
    DATACTRL_MODE = 0x0;                                                                             
    while ( ((ctrladrhi = CTRLADRHI_IN)&MASK) == MASK ) {}                                           
    // Assume M1 trigger, so fetch address (again) in case M1 valid before address (see timing above) and then prefetch 
    // ROM data (which may be invalid) as flash has longest latency and resample control signals at least 120ns after an 
    // M1 trigger. Don't care about the delay if it was any other trigger as all non-fetch RAM cycles have an extra 250ns 
    // clock tick and IO cycles have two extra ticks.
    address = (((CTRLADRHI_IN>>8)&0xFF00)|((ADR_LO_IN)&0xFF)) ;
#ifdef LOWER_ROM_ENABLE    
    romdata = (address & 0x8000)? *(romptr+(address&0x3FFF)) : lowerrom[address&0x3FFF]  ;
#else
    romdata = *(romptr+(address&0x3FFF)) ;
#endif
    ctrladrhi = CTRLADRHI_IN;
#endif

    if ( LOROMRD || (HIROMRD && romptr) ) {
      DATACTRL_OUT = romdata | ROMDIS;
      DATACTRL_MODE = DATA | ROMDIS;
    } else if ( ROMSEL ) {
      romnum   = DATACTRL_IN &0x0F ;   // allow only 16 ROMs and assume ok to alias higher ROMs
      romptr = (valid_upperrom[romnum]) ? (char *) upperrom[romnum]: NULL;
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
            



