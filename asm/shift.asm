; shift.asm
;
; Testing ALU shift operations
;
; This file tests the following instructions:
;   ROTL	Rn
;   ROTR	Rn
;   ROTCL	Rn
;   ROTCR	Rn
;   SHAL	Rn
;   SHAR	Rn
;   SHLL	Rn
;   SHLR	Rn
;
; Revision History:
;   12 May 2025     Zack Huang      Initial revision.

ProgramStart:
    ; ROTL
    MOV #-1, R0;    R0 <- 0xFFFFFFFF
    XOR #1,  R0;    R0 <- 0xFFFFFFFE
    ROTL R0;
    ROTL R0;
    ROTL R0;
    ROTL R0;        Rotate R0 left one byte
    MOV #0, R1;
    MOV R0, @R1;    Expect: R0 = 0xFFFFFFEF

    STC SR, R0;
    AND #1, R0;
    MOV #4, R1;
    MOV R0, @R1;    Expect: T = 1

    ; ROTR
    MOV #-1, R0;    R0 <- 0xFFFFFFFF
    XOR #1,  R0;    R0 <- 0xFFFFFFFE
    ROTR R0;
    ROTR R0;
    ROTR R0;
    ROTR R0;        Rotate R0 right one byte
    MOV #8, R1;
    MOV R0, @R1;    Expect: R0 = 0xEFFFFFFF

    STC SR, R0;
    AND #1, R0;
    MOV #12, R1;
    MOV R0, @R1;    Expect: T = 1

    ; ROTCL
    MOV #-1, R0;    R0 <- 0xFFFFFFFF
    CLRT;
    ROTCL R0;
    ROTCL R0;
    ROTCL R0;
    ROTCL R0;
    MOV #16, R1;
    MOV R0, @R1;    Expect: R0 = 0xFFFFFFF7

    STC SR, R0;
    AND #1, R0;
    MOV #20, R1;
    MOV R0, @R1;    Expect: T = 1

    ; ROTCR
    MOV #-1, R0;    R0 <- 0xFFFFFFFF
    CLRT;
    ROTCR R0;
    ROTCR R0;
    ROTCR R0;
    ROTCR R0;
    MOV #24, R1;
    MOV R0, @R1;    Expect: R0 = 0xEFFFFFFF

    STC SR, R0;
    AND #1, R0;
    MOV #28, R1;
    MOV R0, @R1;    Expect: T = 1

    ; SHLL/SHAL
    MOV #1, R0;
    SHAL R0;
    SHLL R0;
    SHAL R0;
    SHLL R0;
    MOV #32, R1;
    MOV R0, @R1;    Expect: R0 = 0x00000010

    STC SR, R0;
    AND #1, R0;
    MOV #36, R1;
    MOV R0, @R1;    Expect: T = 0

    ; SHAR
    MOV #-1, R0;
    XOR #$80, R0;   R0 <- FFFFFF7F
    SHAR R0;
    SHAR R0;
    SHAR R0;
    SHAR R0;
    MOV #40, R1;
    MOV R0, @R1;    Expect: R0 = 0xFFFFFFF7

    STC SR, R0;
    AND #1, R0;
    MOV #44, R1;
    MOV R0, @R1;    Expect: T = 1

    ; SHLR
    MOV #-1, R0;
    AND #$F0, R0;   R0 <- 000000F0
    SHLR R0;
    SHLR R0;
    SHLR R0;
    SHLR R0;
    MOV #48, R1;
    MOV R0, @R1;    Expect: R0 = 0x0000000F

    STC SR, R0;
    AND #1, R0;
    MOV #52, R1;
    MOV R0, @R1;    Expect: T = 0

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;
    ; Extra NOPs to clear pipeline
    NOP
    NOP
    NOP
