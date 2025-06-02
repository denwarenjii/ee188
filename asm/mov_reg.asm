; mov_Reg.asm
;
; Tests simple register MOVs
;
; This file tests the following instructions:
;   MOV  Rm, Rn
;
; Revision History:
;   11 May 2025     Zack Huang      Initial revision.

ProgramStart:
    ; Move constant through each register
    MOV #15, R0;
    MOV R0, R1;
    MOV R1, R2;
    MOV R2, R3;
    MOV R3, R4;
    MOV R4, R5;
    MOV R5, R6;
    MOV R6, R7;
    MOV R7, R8;
    MOV R8, R9;
    MOV R9, R10;
    MOV R10, R11;
    MOV R11, R12;
    MOV R12, R13;
    MOV R13, R14;
    MOV R14, R15;

    ; Write contents of R15 to memory
    MOV #0, R0;
    MOV R15, @R0    ; expect R0 <- 15

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;
