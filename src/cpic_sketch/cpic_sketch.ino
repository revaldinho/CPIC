
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
// Define only if measuring response times via GPIOs (slows down system)
#define TESTRIG         1

// ---- Define positive bit masks for ctrl + data word
#define DATA            0x00FF
#define ROMEN_B         0x0100
#define ROMDIS          0x0200
#define IORQ_B          0x0400
#define MREQ_B          0x0800
#define WR_B            0x1000
#define WAIT_B          0x2000
#define RAMRD_B         0x3000
#define RAMDIS          0x4000

#define ADR13           1 << 13
#define ADR15           1 << 15

// Define negative TRISTATE masks: 0 = output, 1 = input
#define TRI_DATA        ~DATA
#define TRI_ROMDIS      ~ROMDIS
#define TRI_WAIT_B      ~WAIT_B
#define TRI_RAMDIS      ~RAMDIS

// Port Assignments
#define CTRLDATA_MODE   TRISB          // use PORT B for ctrl + data bits
#define CTRLDATA_IN     PORTB
#define CTRLDATA_OUT    LATB
#define ADDR_MODE       TRISC          // use PORT C for address bits
#define ADDR_IN         PORTC
#define ADDR_OUT        LATC

// ---- Define some Test rig pins on Port G

#define TEST5           84
#define TEST4           83
#define TEST3           82
#define TEST2           79
#define TEST1           78

// ---- Constants and macros
#define MAXROMS         1
#define ROMSIZE         16384
#define RAMBLKSIZE      16384
#define ROMACCESS       (~(ctrldata&ROMEN_B) && (address&ADR15))
#define RAMRDACCESS     ~(ctrldata&RAMRD_B)
#define RAMWRACCESS     ~(ctrldata&MREQ_B || (ctrldata&WR_B))
#define ROMSEL          ~((ctrldata&IORQ_B) || (ctrldata&WR_B) || (address&ADR13))  
#define RAMSEL          ~((ctrldata&IORQ_B) || (ctrldata&WR_B) || (address&ADR15))



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

void setup() {
  CTRLDATA_MODE = 0xFFFF ; // Tristate all ctrl/data outputs
  ADDR_MODE     = 0xFFFF ; // Tristate all address outputs
  CTRLDATA_OUT  = 0x0000 ; // Preset WAIT_B to zero before first assertion
  memcpy(romdata, flashdata,  MAXROMS*ROMSIZE) ; // Copy flash data to RAM on startup

#ifdef TESTRIG
  pinMode(TEST1, OUTPUT);
  pinMode(TEST2, OUTPUT);
  pinMode(TEST3, OUTPUT);
  pinMode(TEST4, OUTPUT);
  pinMode(TEST5, OUTPUT);

  digitalWrite(TEST1, LOW);
  digitalWrite(TEST2, LOW);
  digitalWrite(TEST3, LOW);
  digitalWrite(TEST4, LOW);
  digitalWrite(TEST5, LOW);
#endif
  
}

void loop() {
  int address;
  int ctrldata;
  int block;
    
  ctrldata = CTRLDATA_IN ;                                  
  address  = ADDR_IN ;                                    
  block    = ramblocklut[(address>>14)&0x03][ramblknum];
  
  if ( ROMSEL ) {
    romnum = (ctrldata & 0x07) - 1;                       
    validrom = (romnum>=0) && (romnum <MAXROMS); 
#ifdef TESTRIG    
    digitalWrite(TEST5,!digitalRead(TEST5));
#endif    
  } else if ( RAMSEL ) { 
    ramblknum = (ctrldata & 0x07);                       
#ifdef TESTRIG    
    digitalWrite(TEST4,!digitalRead(TEST4));
#endif    
  } else if ( ROMACCESS ) {
    if ( newaccess && validrom ) {
#ifdef USEWAITSTATE
      CTRLDATA_MODE = (TRI_WAIT_B & TRI_ROMDIS & TRI_DATA); // Enable output drivers, asserting WAIT_B
#endif
      CTRLDATA_OUT  = (ROMDIS | romdata[romnum][address&0x3FFF]);  // Write new data 
      CTRLDATA_MODE = (TRI_ROMDIS & TRI_DATA) ;             // Disable WAIT_B driver (deassert WAIT_B) only, leave others enabled
      newaccess = false;    
#ifdef TESTRIG      
      digitalWrite(TEST3,!digitalRead(TEST3));
#endif      
    } // else DATABUS and ROMDIS state held here from previous action
  } else if ( RAMRDACCESS ) {
    if ( newaccess && (block >=0 )) {         
#ifdef USEWAITSTATE
      CTRLDATA_MODE = (TRI_WAIT_B & TRI_RAMDIS & TRI_DATA); // Enable output drivers, asserting WAIT_B
#endif
      CTRLDATA_OUT  = ( RAMDIS | ramdata[block][address&0x3FFF]); // Write new data to databus and set RAMDIS high
      CTRLDATA_MODE = ( TRI_RAMDIS & TRI_DATA) ;           // Disable WAIT_B driver, enable RAMDIS and data drivers
      newaccess = false;        
#ifdef TESTRIG        
      digitalWrite(TEST2,!digitalRead(TEST2));      
#endif      
    }
  } else if ( RAMWRACCESS ) {
    if (block>=0) {
      ramdata[block][address&0x3FFF] = (ctrldata & DATA);         // Write to internal RAM
    }
  } else {
    CTRLDATA_MODE = 0xFFFF; // Not a ROM access so disable all drivers
    newaccess = true;    
  }
#ifdef TESTRIG
  // toggle GPIO here to measure loop time
  digitalWrite(TEST1, !digitalRead(TEST1));
#endif  
}
