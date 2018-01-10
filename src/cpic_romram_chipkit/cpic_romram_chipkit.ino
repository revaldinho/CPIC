
/*
 * CPIC - An Amstrad CPC ROM/RAM Board in a ChipKit MAX32 microcontroller board
 * 
 * (C) 2017 Revaldinho
 * 
 * PIC32 has 128KBytes RAM available for program and data, plus a further
 * 512KBytes Flash memory for static data.  
 *
 * All ROM data is stored in flash memory to be persistent, but accessing it does 
 * incur some wait states on the MIPS CPU: flash memory has a 30MHz read rate
 * so reading directly would cause 3 wait states per access whereas the RAM runs at
 * the full 80MHz core speed. Copying the ROM contents into RAM is possible to speed
 * this up, but then limits the number of ROMs available. Also this will take a while so may 
 * need to reboot the CPC after the ROM board: if sharing the CPC PSU then need
 * to provide a CPC reset button on the shield.
 *
 * PIC32 has a 256byte instruction cache to minimise wait states on executing from
 * flash. Ideally loop() should fit in this cache...
 *
 * ROM Selection performed by writing the ROM number to an IO address with A13 low.
 * The 'romsel' computation could be done via an external NOR3 if necessary to gain
 * a little speed.
 * 
 * RAM expansion models a 64KBytes DKtronics unit. See 
 * http://www.cpcwiki.eu/imgs/8/83/DK%27Tronics_Peripheral_-_Technical_Manual_%28Edition_1%29.pdf
 * 
 * 
 * Lower ROM replacement is implemented also in this code. Replacing BASIC and Firmware
 * ROMS on a 664 or 464 with those from a 6128 will effectively turn those machines into
 * a 6128 in conjunction with the 64KBytes RAM expansion.
 *
 *
 * IO Register programming:
 *
 *     [TRISA                       A       6000] <- portModeRegister(PORTA) directs here
 *     [  controls input/           ACLR    6004]
 *     [  oputput/high impedance    ASET    6008]
 *     [                            AINV    600C]
 *     -----------------------------------------
 *     [PORTA                       A       6010] <- portInputRegister(PORTA) directs here
 *     [    input state (read-only) ACLR    6014]
 *     [    (attmpted changes       ASET    6018]
 *     [    redirected to LAT)      AINV    601C]
 *     -----------------------------------------
 *     [LATA                        A       6020] <-- portOutputRegister(PORTA) directs here
 *     [  output state              ACLR    6024]
 *     [                            ASET    6028]
 *     [                            AINV    601C]
 *     
 *     PIC32 uses 32-bit addresses pointer (e.g. PORTA)
 *     (uint32_t*) myPointer+1 ===> moves over 4 bytes
 *     (since what it is pointing is it 32-bits wide)
 *     
 *                 TRISA+1    == ACLR
 *                 TRISA+2    == ASET
 *                 (and so on)
 *       
 */
#ifdef DEBUG
#include <string.h>
#endif

#define DEBUG            1
#define CHIPKIT_ASM      1
#define LOWER_ROM_ENABLE 0
#define RAMEXP_ENABLE    0

// ----- Some string processing definitions to enable use of C tokens in assembler statements
#define str(a) st(a)    
#define st(a) #a   

// ---- Define positive bit masks for ctrl + data word
#define DATA            0x00FF
#define ROMEN_B         0x0100
#define IORQ_B          0x0200
#define MREQ_B          0x0400
#define WR_B            0x0800
#define RAMRD_B         0x1000
#define WAIT_B          0x2000
#define ROMDIS          0x4000
#define RAMDIS          0x8000

#define RAMBANKMASK     0x00C0   // Top two bits of DATA must be set for a valid RAM bank selection
#define MASK            0x1F00   // Control bit mask

// ---- Bit masks for address word
#define ADR13           0x2000
#define ADR15           0x8000

// Define negative TRISTATE masks: 0 = output, 1 = input
#define TRI_EN_DATA        ~DATA
#define TRI_EN_ROMDIS      ~ROMDIS
#define TRI_EN_WAIT_B      ~WAIT_B
#define TRI_EN_RAMDIS      ~RAMDIS

// Port Assignment - Breadboard friendly allocation
#define CTRLDATA_MODE TRISD          // use PORT D data bits [7:0], ctrl bits [8:15]
#define CTRLDATA_IN   PORTD
#define CTRLDATA_OUT  LATD
#define ADDR_MODE     TRISB          // use PORT B for address bits [15:0]
#define ADDR_IN       PORTB
#define ADDR_OUT      LATB
#define TEST_MODE     TRISG          // use PORT G for test bits [7:0]
#define TEST_IN       PORTG
#define TEST_OUT      LATG

// ---- Define some Hi/Lo 16b constants to allow fast offset addressing of memory
#ifdef CHIPKIT_ASM
#define CTRLDATA_IN_HI 0xbf88       // Port D for use in assembler must be simple absolute value
#define CTRLDATA_IN_LO 0x60d0   
#define ADDR_IN_HI     0xbf88       // Port B for use in assembler must be simple absolute value
#define ADDR_IN_LO     0x6050
#endif

// ---- Define some Test rig pins on Port G
#define TEST2           79
#define TEST1           78

// ---- Constants and macros
#define MAXROMS         16

#ifdef DEBUG
#define ROMSIZE         128
#else
#define ROMSIZE         16384
#endif

#define ROMACCESS       (!(ctrldata&ROMEN_B)) 
#define ROMSEL          !((ctrldata&IORQ_B) || (ctrldata&WR_B) || (address&ADR13))  
#define RAMBLKSIZE      16384
#define RAMRDACCESS     !(ctrldata&RAMRD_B)
#define RAMWRACCESS     !(ctrldata&MREQ_B || (ctrldata&WR_B))
#define RAMSEL          !((ctrldata&IORQ_B) || (ctrldata&WR_B) || (address&ADR15))
#define ROMRAMSEL       !((ctrldata&IORQ_B) || (ctrldata&WR_B) )

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
char ramdata[4][RAMBLKSIZE] ;              //  64K for D'ktronics RAM exp
int ramblocklut [4][8] = {                 // mapping for D'ktronics expansion RAM
  -1, 3, 3, 3, -1, -1, -1, -1,             //   -1 indicates use original CPC RAM bank
  -1,-1, 2,-1, -1, -1, -1, -1,             // 0..3 picks a 16K block from this expansion
  -1,-1, 1,-1,  0,  1,  2,  3,
  -1,-1, 0,-1, -1, -1, -1, -1,
};
#endif

void setup() {
  CTRLDATA_MODE = 0xFFFF ; // Tristate all ctrl/data outputs
  ADDR_MODE     = 0xFFFF ; // Tristate all address outputs
  CTRLDATA_OUT  = 0x0000 ; // Preset WAIT_B to zero before first assertion  
  TEST_MODE     = 0x0000 ; // Enable all test port outputs
  TEST_OUT      = 0x0000;
  // ADDR_MODE line above doesn't seem to put all address bits (PortB) in input mode...
  for ( int i=54 ; i<70; i++ ) {
    pinMode(i,INPUT);
  }
}

void loop() {
#ifdef DEBUG  
  int rambanknum = 1;
  boolean validrom = true;
#else
  int rambanknum = 0;
  boolean validrom = false;
#endif
  int romnum = 0;  
  register int ctrldata;
  register int address;
  int ramblock;
  int dataout;
  int rom_addr_hi = 0;
  
  while (true) {
    
#ifdef CHIPKIT_ASM
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
    // exit loop: Wait here until all signals go inactive
    while ((CTRLDATA_IN&MASK) != MASK) { }      
    CTRLDATA_MODE = 0xFFFF;    
    // entry loop: Wait here for a transaction to start
    while (((ctrldata=CTRLDATA_IN)&MASK) == MASK) {
       // Always sample the address which becomes valid before the control signals go active
       address  = ADDR_IN;
    } 
#endif    

    if ( ROMACCESS ) {        
      if ((address&ADR15) && validrom) {
        dataout = upperrom[rom_addr_hi|(address&0x3FFF)];      
        CTRLDATA_OUT  = ROMDIS | dataout;                              // Write new data with ROMDIS signal 
        CTRLDATA_MODE = TRI_EN_ROMDIS & TRI_EN_DATA;                   // Enable ROMDIS and DATA        
#ifdef LOWER_ROM_ENABLE
      } else if (!(address&ADR15)) {
        dataout = lowerrom[address&0x3FFF];        
        CTRLDATA_OUT  = ROMDIS | dataout;                              // Write new data with ROMDIS signal 
        CTRLDATA_MODE = TRI_EN_ROMDIS & TRI_EN_DATA;                   // Enable ROMDIS and DATA                  
      }// else DATABUS and ROMDIS state held here from previous action
#else
    }
#endif        

#ifdef RAM_EXP_ENABLE
    } else if ( RAMRDACCESS ) {
      ramblock = ramblocklut[(address>>14)&0x03][rambanknum];        
      if ( ramblock >= 0 ) {         
        CTRLDATA_OUT = RAMDIS | ramdata[ramblock][address&0x3FFF]; // Write new data to databus 
        CTRLDATA_MODE = TRI_EN_DATA & TRI_EN_RAMDIS ;              // Disable WAIT_B driver, enable RAMDIS and DATA drivers
      } // else DATABUS and RAMDIS state held here from previous action
    } else if ( RAMWRACCESS ) {        
      ramblock = ramblocklut[(address>>14)&0x03][rambanknum];    
      if (ramblock>=0) {
        ramdata[ramblock][address&0x3FFF] = ctrldata&DATA;         // Write to internal RAM
      }       
#endif     
    } else if ( ROMRAMSEL ) {
      if ( !(address&ADR13)) {                                    
        // Upper ROM selection if ADR13 low
        romnum = (ctrldata & 0x0F);  
        rom_addr_hi = romnum <<14;
        validrom = valid_upperrom[romnum];        
#ifdef RAM_EXP_ENABLE
      } else if ( (!(address&ADR15)) && ((ctrldata&RAMBANKMASK)==RAMBANKMASK)){                              
        // RAM bank selection if ADR15 low AND top two bits of data word are set
        rambanknum = (ctrldata & 0x07); 
#endif                      
      } 
    } 
  }
}
