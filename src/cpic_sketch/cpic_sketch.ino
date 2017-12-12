
/*
 * PIC32 has 128KBytes RAM available for program and data, plus a further
 * 512KBytes Flash memory for static data.  
 *
 * All ROM data is stored in flash memory to be persistent, but copied over into 
 * RAM on booting the board to get faster access. This will take a while so may 
 * need to reboot the CPC after the ROM board: if sharing the CPC PSU then need
 * to provide a CPC reset button on the shield.
 *
 * NB ROMs are numbered 0-MAXROMS-1 here, but map to CPC numbers 1-MAXROMS because
 * slot 0 is for the firmware and unlikely to be replaced
 *
 * ROM Selection performed by writing the ROM number to an IO address with A13 low.
 * The 'romsel' computation could be done via an external NOR3 if necessary to gain
 * a little speed.
 *
 * ROM addresses are only 14bits long, so tying off the two top address bits externally
 * could save a little more computation needed to mask the 16bit port value down to
 * 14 value bits.
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

// Define positive bit masks for ctrl + data word
#define DATA            0x00FF
#define ROMSEL          0x0100
#define ROMEN_B         0x0200
#define ROMDIS          0x0400
#define IORQ_B          0x0800
#define WR_B            0x1000
#define WAIT_B          0x2000

// Define negative TRISTATE masks: 0 = output, 1 = input
#define TRI_DATA        ~DATA
#define TRI_ROMDIS      ~ROMDIS
#define TRI_WAIT_B      ~WAIT_B

// Only A13 special in address word
#define A13             0x1 << 13

// Port Assignments
#define CTRLDATA_MODE   TRISB          // use PORT B for ctrl + data bits
#define CTRLDATA_IN     PORTB
#define CTRLDATA_OUT    LATB
#define ADDR_MODE       TRISC          // use PORT C for address bits
#define ADDR_IN         PORTC
#define ADDR_OUT        LATC

// Constants and macros
#define MAXROMS         1
#define ROMSIZE         16384
#define ROMACCESS       ~(ctrldata & ROMEN_B)
#define ROMSEL          ~((ctrldata&IORQ_B) || (ctrldata&WR_B) || (address&A13))

// Global variables
const char flashdata[MAXROMS][ROMSIZE] = { 
#include "/Users/richarde/Documents/Development/git/CPiC/src/CWTA.CSV"
//#include "ROM1.csv"
//#include "ROM2.csv"
//#include "ROM3.csv"
};

char romdata[MAXROMS][ROMSIZE] ;

int romnum = 0;
boolean validrom = false;
boolean newaccess = 1 ;

void setup() {
  
  CTRLDATA_MODE = 0xFFFF ; // Tristate all ctrl/data outputs
  ADDR_MODE     = 0xFFFF ; // Tristate all address outputs
  CTRLDATA_OUT  = 0x0000 ; // Preset WAIT_B to zero before first assertion
  memcpy(romdata, flashdata,  MAXROMS*ROMSIZE) ; // Copy flash data to RAM on startup
  
}

void loop() {
  int address;
  int ctrldata;
  
  address   = ADDR_IN & 0x3FFF ;      
  ctrldata  = CTRLDATA_IN ;

  if ( ROMSEL ) {
    romnum = (ctrldata & 0x07) - 1;                       
    validrom = ((romnum>=0) && (romnum <MAXROMS)) ? true : false; 
    newaccess = true;    
  } else if ( ROMACCESS ) {
    if ( newaccess && validrom ) {
#ifdef USEWAITSTATE
      CTRLDATA_MODE = (TRI_WAIT_B & TRI_ROMDIS & TRI_DATA) ;               // Enable output drivers, asserting WAIT_B
#endif
      CTRLDATA_OUT  = (                 ROMDIS | romdata[romnum][address]);// Write new data 
      CTRLDATA_MODE = (             TRI_ROMDIS & TRI_DATA) ;               // Disable WAIT_B driver (deassert WAIT_B) only, leave others enabled
      newaccess = false;    
    } // else DATABUS and ROMDIS state held here from previous action
  } else {
    CTRLDATA_MODE = 0xFFFF; // Not a ROM access so disable all drivers
    newaccess = true;    
  }
  
}
