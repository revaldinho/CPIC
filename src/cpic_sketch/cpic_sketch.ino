
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
 * NB ROMs are numbered 0-MAXROMS-1 here, but map to CPC numbers 1-MAXROMS because
 * slot 0 is for the firmware and unlikely to be replaced
 *
 * ROM Selection performed by writing the ROM number to an IO address with A13 low.
 * The 'romsel' computation could be done via an external NOR3 if necessary to gain
 * a little speed.
 * 
 * RAM expansion models a 64KBytes DKtronics unit
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
// ---- Define positive bit masks for ctrl + data word
#define DATA            0x00FF
#define ROMEN_B         0x0100
#define ROMDIS          0x0200
#define IORQ_B          0x0400
#define MREQ_B          0x0800
#define WR_B            0x1000
#define WAIT_B          0x2000
#define RAMRD_B         0x4000
#define RAMDIS          0x8000

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


// ---- Define some Test rig pins on Port G
#define TEST2           79
#define TEST1           78

// ---- Constants and macros
#define MAXROMS         1
#define ROMSIZE         16384
#define ROMACCESS       (!(ctrldata&ROMEN_B)) && (address&ADR15)
#define ROMSEL          !((ctrldata&IORQ_B) || (ctrldata&WR_B) || (address&ADR13))  

#define RAMBLKSIZE      16384
#define RAMRDACCESS     !(ctrldata&RAMRD_B)
#define RAMWRACCESS     !(ctrldata&MREQ_B || (ctrldata&WR_B))
#define RAMSEL          !((ctrldata&IORQ_B) || (ctrldata&WR_B) || (address&ADR15))

// Global variables
const char romdata[MAXROMS][ROMSIZE] = { 
#include "/Users/richarde/Documents/Development/git/CPiC/src/CWTA.CSV"
//#include "ROM1.csv"
//#include "ROM2.csv"
//#include "ROM3.csv"
};

char ramdata[4][RAMBLKSIZE] ;              //  64K for D'ktronics RAM exp

int ramblocklut [4][8] = {                 // mapping for D'ktronics expansion RAM
  -1, 3, 3, 3, -1, -1, -1, -1,             //   -1 indicates use original CPC RAM bank
  -1,-1, 2,-1, -1, -1, -1, -1,             // 0..3 picks a 16K block from this expansion
  -1,-1, 1,-1,  0,  1,  2,  3,
  -1,-1, 0,-1, -1, -1, -1, -1,
};

    
void setup() {
  CTRLDATA_MODE = 0xFFFF ; // Tristate all ctrl/data outputs
  ADDR_MODE     = 0xFFFF ; // Tristate all address outputs
  CTRLDATA_OUT  = 0x0000 ; // Preset WAIT_B to zero before first assertion  
  TEST_MODE     = 0x0000 ; // Enable all test port outputs
  TEST_OUT = 0x0000;
  // ADDR_MODE line above doesn't seem to put all address bits (PortB) in input mode...
  for ( int i=54 ; i<70; i++ ) {
    pinMode(i,INPUT);
  }
 
}

void loop() {
  int romnum = 1;    // Should be '0' but noodle for Breadboard testing
  int ramblknum = 1; // Should be '0' ... as above
  int ctrldata;
  int address;
  int ramblock;
  boolean validrom = true;

  while (true) {
    ctrldata = CTRLDATA_IN; 
    address  = ADDR_IN;
    if ( ROMACCESS ) {
      if ( validrom ) {
        CTRLDATA_OUT  = ROMDIS | romdata[romnum][address&0x3FFF];      // Write new data with ROMDIS signal 
        CTRLDATA_MODE = TRI_EN_ROMDIS & TRI_EN_DATA;                   // Enable ROMDIS and DATA        
      } // else DATABUS and ROMDIS state held here from previous action
    } else if ( RAMRDACCESS ) {
      ramblock = ramblocklut[(address>>14)&0x03][ramblknum];        
      if ( ramblock >=0 ) {         
        CTRLDATA_OUT = RAMDIS | ramdata[ramblock][address&0x3FFF]; // Write new data to databus 
        CTRLDATA_MODE = TRI_EN_DATA & TRI_EN_RAMDIS ;              // Disable WAIT_B driver, enable RAMDIS and DATA drivers
      } // else DATABUS and RAMDIS state held here from previous action
    } else if ( RAMWRACCESS ) {
      ramblock = ramblocklut[(address>>14)&0x03][ramblknum];    
      if (ramblock>=0) {
        ramdata[ramblock][address&0x3FFF] = ctrldata&DATA;         // Write to internal RAM
      }  
    } else if ( ROMSEL ) {
      romnum = (ctrldata & 0x07) - 1;  
      validrom = (romnum>=0) && (romnum <MAXROMS); 
    } else if ( RAMSEL ) { 
      ramblknum = (ctrldata & 0x07);                       
    } else {
      CTRLDATA_MODE = 0xFFFF; // Not a ROM/RAM access so disable all drivers              
    }
  }
}
