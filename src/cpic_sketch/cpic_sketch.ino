
/*
 * CPIC - An Amstrad CPC ROM Board in a ChipKit MAX32 microcontroller board
 * 
 * (C) 2017 Revaldinho
 * 
 * PIC32 has 128KBytes RAM available for program and data, plus a further
 * 512KBytes Flash memory for static data.  
 *
 * All ROM data is stored in flash memory to be persistent, but copied over into 
 * RAM on booting the board to get faster access: flash memory has a 30MHz read rate
 * so reading directly would cause 3 wait states per access whereas the RAM runs at
 * the full 80MHz core speed. Copying the ROM contents will take a while so may 
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
#include <string.h>

// Define only if not fast enough to serve ROM bytes in time available
#define USEWAITSTATE    1

// ---- Define positive bit masks for ctrl + data word
#define DATA            0x00FF
#define ROMEN_B         0x0001
#define ROMDIS          0x0002
#define IORQ_B          0x0004
#define MREQ_B          0x0008
#define WR_B            0x0010
#define WAIT_B          0x0020
#define RAMRD_B         0x0040
#define RAMDIS          0x0080

#define ADR13           1 << 13
#define ADR15           1 << 15

// Define negative TRISTATE masks: 0 = output, 1 = input
#define TRI_EN_DATA        ~DATA
#define TRI_EN_ROMDIS      ~ROMDIS
#define TRI_EN_WAIT_B      ~WAIT_B
#define TRI_EN_RAMDIS      ~RAMDIS

// Port Assignment - Breadboard friendly allocation
#define DATA_MODE   TRISE          // use PORT E data bits [7:0]
#define DATA_IN     PORTE
#define DATA_OUT    LATE
#define ADDR_MODE   TRISB          // use PORT B for address bits [15:0]
#define ADDR_IN     PORTB
#define ADDR_OUT    LATB
#define CTRL_MODE   TRISA          // use PORT A for ctrl bits [7:0]
#define CTRL_IN     PORTA
#define CTRL_OUT    LATA
#define TEST_MODE   TRISG          // use PORT G for test bits [7:0]
#define TEST_IN     PORTG
#define TEST_OUT    LATG


// ---- Define some Test rig pins on Port G
#define TEST2           79
#define TEST1           78

// ---- Constants and macros
#define MAXROMS         1
#define ROMSIZE         16384
#define RAMBLKSIZE      16384
//#define ROMACCESS       false //(!(ctrl&ROMEN_B) && (address&ADR15))
#define RAMRDACCESS     false //!(ctrl&RAMRD_B)
#define RAMWRACCESS     false //!(ctrl&MREQ_B || (ctrl&WR_B))
#define ROMSEL          false //!((ctrl&IORQ_B) || (ctrl&WR_B) || (address&ADR13))  
#define RAMSEL          false //!((ctrl&IORQ_B) || (ctrl&WR_B) || (address&ADR15))
#define ROMACCESS       !(ctrl&0x1) 

// Global variables
const char flashdata[MAXROMS][ROMSIZE] = { 
#include "/Users/richarde/Documents/Development/git/CPiC/src/CWTA.CSV"
//#include "ROM1.csv"
//#include "ROM2.csv"
//#include "ROM3.csv"
};

char romdata[MAXROMS][ROMSIZE] ;
                                           //  48K for ROM space
char ramdata[4][RAMBLKSIZE] ;              //  64K for D'ktronics RAM exp
                                           //  16K for this program to run in
                                           // ---- 
                                           // 128K total PIC32 RAM capacity
                                           // ----

int ramblocklut [4][8] = {                 // mapping for D'ktronics expansion RAM
  -1, 3, 3, 3, -1, -1, -1, -1,             //   -1 indicates use original CPC RAM bank
  -1,-1, 2,-1, -1, -1, -1, -1,             // 0..3 picks a 16K block from this expansion
  -1,-1, 1,-1,  0,  1,  2,  3,
  -1,-1, 0,-1, -1, -1, -1, -1,
};

int romnum = 0;
int ramblknum = 0;
boolean validrom = false;
boolean newaccess = true ;
int address;
int ctrl;
int data;
int block;
    
void setup() {
  DATA_MODE     = 0xFFFF ; // Tristate all data outputs
  CTRL_MODE     = 0xFFFF ; // Tristate all ctrl outputs
  ADDR_MODE     = 0xFFFF ; // Tristate all address outputs
  CTRL_OUT      = 0x0000 ; // Preset WAIT_B to zero before first assertion
  memcpy(romdata, flashdata,  MAXROMS*ROMSIZE) ; // Copy flash data to RAM on startup
  
  TEST_MODE     = 0x0000 ; // Enable all test port outputs
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  TEST_OUT = 0x0000;


  // Breadboard noodling
  ramblknum = 1;
  romnum = 1;
  validrom = true;
}

void loop() {
  ctrl     = CTRL_IN; 
                                   
  if ( (ctrl&0xFF) == 0x00FF ) {
    // Check if any control lines are active first and bail out as soon as possible
    // so that loop can be responsive to next control line change
    CTRL_MODE = 0xFFFF;
    DATA_MODE = 0xFFFF;
    newaccess = true;    
    TEST_OUT = 0x0000;                 
  } else {
    // Control lines are active so now need to go through the long if-else statement
    // to check which event to handle
    address  = ADDR_IN;
    
    if ( ROMSEL ) {
      data     = DATA_IN;  
      romnum = (data & 0x07) - 1;  
      validrom = (romnum>=0) && (romnum <MAXROMS); 
      TEST_OUT = 0x0001;                   
    } else if ( RAMSEL ) { 
      data     = DATA_IN ;
      ramblknum = (data & 0x07);                       
      TEST_OUT = 0x0020;                 
    } else if ( ROMACCESS ) {
      if ( newaccess && validrom ) {
#ifdef USEWAITSTATE
        CTRL_MODE = TRI_EN_WAIT_B & TRI_EN_ROMDIS;   // Enable output drivers, asserting WAIT_B
#endif
        CTRL_OUT  = ROMDIS;
        DATA_OUT  = romdata[romnum][address&0x3FFF]; // Write new data 
        DATA_MODE = TRI_EN_DATA;
        CTRL_MODE = TRI_EN_ROMDIS;                   // Disable WAIT_B driver (deassert WAIT_B) only, leave others enabled
        newaccess = false;         
      } // else DATABUS and ROMDIS state held here from previous action
      TEST_OUT = 0x0001;                   
    } else if ( RAMRDACCESS ) {
      block    = ramblocklut[(address>>14)&0x03][ramblknum];  
      
      if ( newaccess && (block >=0 )) {         
#ifdef USEWAITSTATE
        CTRL_MODE = TRI_EN_WAIT_B & TRI_EN_RAMDIS;// Enable output drivers, asserting WAIT_B
#endif
        CTRL_OUT = RAMDIS;                         // Set RAMDIS high
        DATA_OUT = ramdata[block][address&0x3FFF]; // Write new data to databus 
        DATA_MODE = TRI_EN_DATA ;                  // Enable data drivers
        CTRL_MODE = TRI_EN_RAMDIS ;                // Disable WAIT_B driver, enable RAMDIS
        newaccess = false;        
      } // else DATABUS and RAMDIS state held here from previous action
    } else if ( RAMWRACCESS ) {
      data     = DATA_IN;
      block    = ramblocklut[(address>>14)&0x03][ramblknum];  
  
      if (block>=0) {
        ramdata[block][address&0x3FFF] = data;         // Write to internal RAM
      }   
    } else {
      CTRL_MODE = 0xFFFF; // Not a ROM access so disable all drivers
      DATA_MODE = 0xFFFF;
      newaccess = true;    
      TEST_OUT = 0x0000;                 
    }
  }
}
