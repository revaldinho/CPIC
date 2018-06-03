        @@
        @@ In C use
        @@ 
        @@ #include <stdio.h>
        @@ extern void asmloop( int *a, char *n)
        @@ ...
        @@ asmloop( romvalid, romdata)
        @@ ...
        @@
        @@ Entry:
        @@      r0 = validrom[] array address
        @@           - valid entries = 256 (ie ROMVALID const)
        @@           - invalid entries = 0
        @@      r1 = romdata[] array address
        @@ Exit:
        @@      - routine never exits
        @@ 
        @@ Register usage:
        @@     r0       = validrom[] array address
        @@     r1       = romdata[] array address
        @@     r2       = latest ctrladrhi_in sample
        @@     r3       = temp and romdata
        @@     r4       = romptr
        @@     r5       = temp
        @@     r6       = constant ADR13|IOREQ*|WR*
        @@     r7       = constant port DATACTRL_OUT
        @@     r8       = constant ADR14|ROMEN_B        
        @@     r9       = constant port CTRLADRHI_IN
        @@     r10      = constant port DINLO_IN
        @@     r11      = constant port ADRLO_IN
        @@     ip [r12] = ROMVALID flag
        @@     sp [r13] = Stack pointer
        @@     lr [r14] = unused
        @@
        @@ Latencies   CPU    Bus     Mem
        @@ 168 MHz      1       3       6
        @@ 144 MHz      1       3       5
        @@ 120 MHz      1       2       5
        @@  96 MHz      1       2       4       
        @@ 
        @@  Port Address definitions
        .thumb
        .syntax unified
        .equ    DINLO_IN                0x400ff010 @ PORTA DIN
        .equ	CTRLADRHI_IN,           0x400ff050 @ PORTB DIN
        .equ	ADRLO_IN,	        0x400ff0d0 @ PORTD DIN
        .equ	DATACTRL_OUT,           0x400ff080 @ PORTC DOUT

        @@ Bit assignments in PortB
	.equ 	MREQ_B,                 0x00000001 // bit 0
	.equ 	IOREQ_B,                0x00000002
	.equ 	WR_B,                   0x00000004
	.equ 	RD_B,                   0x00000008
	.equ 	ROMEN_B,                0x00000400 // bit 10
	.equ 	M1_B,                   0x00000800 // bit 11 

        .equ    ADDR_HI,                0x00FF0000  // bits 16-23 on control input data        
        .equ    ADR_13,                 0x00200000  // bit 13 in raw position in ctrl/adr word - ie bit 21
        .equ    ADR_14,                 0x00400000  // bit 14 in raw position in ctrl/adr word - ie bit 22
        .equ    ADR_15,                 0x00800000  // bit 15 in raw position in ctrl/adr word - ie bit 23

        @@ Bit assignments in PortC
        .equ    DATAOUT,                0x000000FF 
        .equ    ROMVALID,               0x00000100

        @@ Bit assignments in Port D
        .equ    ADDR_LO,                0x000000FF  // bits  0-7 on address input

        @@ Bit assignments in Port E
        .equ 	CLK4,                   0x01000000  // bit 24
        .equ 	READY,                  0x02000000  // bit 25
        
        @@ Bit assignments in Port A
        .equ    DIN_LO,                 0x0003F000  // bits 12..17  
        
        @@ Composite masks
        .equ    M1B_RDB_WRB,            M1_B   | RD_B    | WR_B 
        .equ    RDB_WRB,                RD_B   | WR_B           
        .equ    ROMENB_WRB,             ROMEN_B | WR_B         
        .equ    ADR13_IOREQB_WRB,       ADR_13 | IOREQ_B | WR_B        
        .equ    ADR14_ROMENB,           ADR_14 | ROMEN_B       
        .equ    ADR14,                  ADR_14                 
        
        .global asmloop
        .p2align 2
        .type  asmloop,%function
asmloop:        
        ldr 	r9, =CTRLADRHI_IN
        mov	ip, #0                  @ ROM valid flag low to start with 
        mov	r4, #0          	@ r4 = romptr = initialized to 0
        ldr	r11, =ADRLO_IN 
        ldr	r6, =ADR13_IOREQB_WRB
        ldr	r7, =DATACTRL_OUT
        ldr	r8, =ADR14_ROMENB
        ldr     r10, =DINLO_IN
        str     r4, [r7, #0]    	@ clear ROMVALID output by writing all zeroes to data & romvalid bits

EXITLOOP:                               @ Loop until RD* and WR* both go inactive high before entering main loop
        ldr	r2, [r9, #0]            @ r2 = [CTRLADRHI_IN] sample control and address bits
        and.w	r5, r2, #RDB_WRB        
        cmp.w	r5, #RDB_WRB            
        bne     EXITLOOP
L0:     
        ldr	r2, [r9, #0]            @ r2 = [CTRLADRHI_IN] sample control and address bits
        ldrb	r3, [r11, #0]           @ r3 = [ADRLO_IN] sample address low bits only, zero extended to 32b                
        and	r5, r2, r6              @ isolate trigger bits IOREQB_ADR13_WRB
        cbz     r5, ROMSEL              @ jump to ROM SELECTION if all low
        and	r5, r2, r8              @ isolate trigger bits ADR14_ROMENB
        cmp	r5, #ADR14              @ ROM access = ROMENB low and ADR14 high        
        bne     L0                      @ loop again if not equal otherwise fall through to ROM ACCESS
        
ROMACCESS:
        and	r5, r2, #0x3f0000	@ mask off upper 6 high address bits (16K address range) in place [high adr byte orig at bits 23:16]        
        orr	r3, r3, r5, lsr #8      @ OR upper and lower address bytes together shifting upper address bits into correct location         
        ldrb	r3, [r4, r3]            @ get romdata from [romptr+address] - zero extended to 32 bits
        strb 	r3, [r7, #0]    	@ then [DATACTRL_OUT] = romdata (leaving upper bytes with ROMVALID intact)
        b       EXITLOOP

ROMSEL:                                 @ ROM SELECTION event
        ldr	r3, [r10, #0]           @ r3 = [DINLO_IN] sample data in low bits
        lsr	r3, r3, #10     	@ shift databits into bottom byte to be ROMNUM*4 (ie shifted 2 places left) 'cos valid data is ints which are 4 bits
        and	r3, r3, #0x3C     	@ mask off the ROM number with 00 in bottom two bits
        ldr	ip, [r0, r3]    	@ check if this ROM number is valid reading from validrom[] array (valid = ROMVALID @ invalid = 0)
        add	r4, r1, r3, lsl #12     @ LSL ROMNUM (r3) 14 places (ie 12 more) into MSBs and add to ram[] array address to build romptr
        str	ip, [r7]                @ write out ROMVALID flag to DATACTRL_OUT (data is dont care)
        b	EXITLOOP


  
