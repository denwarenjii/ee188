; control.asm
;
; Testing all control register operations
;
; This file tests the following instructions:
;   STC     SR, Rn
;   STC     GBR, Rn
;   STC     VBR, Rn
;   STC.L   SR, @-Rn
;   STC.L   GBR, @-Rn
;   STC.L   VBR, @-Rn
;   STS     MACH, Rn
;   STS     MACL, Rn
;   STS     PR, Rn
;   STS.L   MACH, @-Rn
;   STS.L   MACL, @-Rn
;   STS.L   PR, @-Rn
;
; Revision History:
;   25 May 2025     Zack Huang      Initial revision.

ProgramStart:
    ; Test LDS
    MOV #$F, R0;
    LDS R0, PR;     SR  <- 0x0000000F

    ADD #1, R0;
    LDS R0, MACH;    MACH <- 0x00000100

    ADD #1, R0;
    LDS R0, MACL;    MACL <- 0x00000101

    STS PR, R0;
    MOV #$0, R1;
    MOV R0, @R1;    Expect R0 = 0x0000000F

    STS MACH, R0;
    MOV #$4, R1;
    MOV R0, @R1;    Expect R0 = 0x00000010

    STS MACL, R0;
    MOV #$8, R1;
    MOV R0, @R1;    Expect R0 = 0x00000011

    ; Test LDS.L
    MOV #$70, R3;
    MOV #$1A, R2;
    MOV R2, @R3;    Set memory at 0x70 to be 0x1A

    MOV #$74, R3;
    MOV #$2B, R2;
    MOV R2, @R3;    Set memory at 0x74 to be 0x2B

    MOV #$78, R3;
    MOV #$3C, R2;
    MOV R2, @R3;    Set memory at 0x78 to be 0x3C
    
    MOV #$70, R2;
    LDS.L @R2+, PR;     Read SR from memory, increment R2
    STS PR, R0;
    MOV #$C, R1;
    MOV R0, @R1;        Expect R0 = 0x000000AA

    LDS.L @R2+, MACH;    Read MACH from memory, increment R2
    STS MACH, R0;
    MOV #$10, R1;
    MOV R0, @R1;        Expect R0 = 0x000000BB

    LDS.L @R2+, MACL;    read MACL from memory, increment R2
    STS MACL, R0;
    MOV #$14, R1;
    MOV R0, @R1;        Expect R0 = 0x000000CC

    ; Test STS.L
    MOV #$4C, R0;
    LDS R0, PR;     SR  <- 0x0000004C

    ADD #1, R0;
    LDS R0, MACH;    MACH <- 0x0000004D

    ADD #1, R0;
    LDS R0, MACL;    MACL <- 0x0000004E

    MOV #$24, R1;
    STS.L PR, @-R1;     Expect 0x0000004C at 0x20
    STS.L MACH, @-R1;    Expect 0x0000004D at 0x1C
    STS.L MACL, @-R1;    Expect 0x0000004E at 0x18

    ; Test CLRMAC
    CLRMAC;

    MOV #$30, R1;
    STS.L MACH, @-R1;    Expect 0x00000000 at 0x2C
    STS.L MACL, @-R1;    Expect 0x00000000 at 0x28

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;

