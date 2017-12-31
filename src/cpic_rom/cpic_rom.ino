
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
//#define USEWAITSTATE    1

// ---- Define positive bit masks for ctrl + data word
#define DATA            0x00FF
#define ROMEN_B         0x0100
#define ROMDIS          0x0200
#define IORQ_B          0x0400
#define MREQ_B          0x0800
#define WR_B            0x1000
#define WAIT_B          0x2000
#define ADR13           0x4000              // Read A13, A15 also in the control word so we can distinguish
#define ADR15           0x8000              // between ROMSEL and ROMACCESS without reading address port

// Define negative TRISTATE masks: 0 = output, 1 = input
#define TRI_EN_DATA        ~DATA
#define TRI_EN_ROMDIS      ~ROMDIS
#define TRI_EN_WAIT_B      ~WAIT_B
#define TRI_EN_RAMDIS      ~RAMDIS

// Port Assignment - Breadboard friendly allocation
#define CTRLDATA_MODE   TRISD          // use PORT D data bits [7:0], ctrl [15:8]
#define CTRLDATA_IN     PORTD
#define CTRLDATA_OUT    LATD
#define ADDR_MODE       TRISB          // use PORT B for address bits [15:0]
#define ADDR_IN         PORTB
#define ADDR_OUT        LATB
#define TEST_MODE       TRISG          // use PORT G for test bits [7:0]
#define TEST_IN         PORTG
#define TEST_OUT        LATG

// ---- Define some Test rig pins on Port G
#define TEST2           79
#define TEST1           78

// ---- Constants and macros
#define MAXROMS         1
#define ROMSIZE         16384
#define ROMACCESS       (!(ctrldata&ROMEN_B) && (ctrldata&ADR15))
#define ROMSEL          !((ctrldata&IORQ_B) || (ctrldata&WR_B) || (ctrldata&ADR13))  

// Global variables
const char flashdata[MAXROMS][ROMSIZE] = { 
#include "/Users/richarde/Documents/Development/git/CPiC/src/CWTA.CSV"
//#include "ROM1.csv"
//#include "ROM2.csv"
//#include "ROM3.csv"
};

char romdata[MAXROMS][ROMSIZE] ;
int romnum = 0;
int ramblknum = 0;
boolean validrom = false;

#ifdef USEWAITSTATE
boolean newaccess = true ;
#endif

int address;
int ctrldata;
    
void setup() {
  CTRLDATA_MODE = 0xFFFF ; // Tristate all ctrl outputs
  ADDR_MODE     = 0xFFFF ; // Tristate all address outputs
  CTRLDATA_OUT  = 0x0000 ; // Preset WAIT_B to zero before first assertion
  memcpy(romdata, flashdata,  MAXROMS*ROMSIZE) ; // Copy flash data to RAM on startup
  
  TEST_MODE     = 0x0000 ; // Enable all test port outputs
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  TEST_OUT = 0x0000;

  // Breadboard noodling
  romnum = 1;
  validrom = true;
}

void loop() {

//  while ( true ) {
    ctrldata     = CTRLDATA_IN; 
                                     
    if ( ROMACCESS ) {
      address  = ADDR_IN;
#ifdef USEWAITSTATE
      if ( newaccess && validrom ) {
        CTRLDATA_MODE = TRI_EN_WAIT_B & TRI_EN_ROMDIS & TRI_EN_DATA;   // Enable output drivers, asserting WAIT_B
        CTRLDATA_OUT  = ROMDIS | romdata[romnum][address&0x3FFF];      // Write new data with ROMDIS signal 
        CTRLDATA_MODE = TRI_EN_ROMDIS & TRI_EN_DATA;                   // Drop wait signal but keep ROMDIS and DATA asserted
        newaccess = false;         
      }  // else DATABUS and ROMDIS state held here from previous action
#else
      if (validrom) {
        CTRLDATA_OUT  = ROMDIS | romdata[romnum][address&0x3FFF];      // Write new data with ROMDIS signal 
        CTRLDATA_MODE = TRI_EN_ROMDIS & TRI_EN_DATA;                   // Enable ROMDIS and DATA 
      }
#endif
      TEST_OUT = 0x0001;                   
    } else if ( ROMSEL ) {
      romnum = (ctrldata & 0x07) - 1;  
      validrom = (romnum>=0) && (romnum <MAXROMS); 
      TEST_OUT = 0x0001;                   
    } else {
      CTRLDATA_MODE = 0xFFFF; // Not a ROM access so disable all drivers
#ifdef USEWAITSTATE
      newaccess = true;
#endif    
      TEST_OUT = 0x0000;                 
    }
//  }
}
