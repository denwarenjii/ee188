; reg_indirect.asm
;
; Testing indirect register addressing
;
; This file tests the following instructions:
;     MOV.B Rm, @Rn
;     MOV.W Rm, @Rn
;     MOV.L Rm, @Rn
;
; Revision History:
;   11 May 2025     Zack Huang      Initial revision.
;   14 May 2025     Zack Huang      Ensure endianness is consistent

ProgramStart:
    ; Write 0A0B0C0D to 0x00000000 using byte mode addressing
    ; Note, for the purposes of testing, all longwords in memory are
    ; interpreted as being big-endian, which means that the CPU will read this
    ; as 0D0C0B0A.
    MOV #0, R0;
    MOV #$D, R1;
    MOV.B R1, @R0;

    MOV #1, R0;
    MOV #$C, R1;
    MOV.B R1, @R0;

    MOV #2, R0;
    MOV #$B, R1;
    MOV.B R1, @R0;

    MOV #3, R0;
    MOV #$A, R1;
    MOV.B R1, @R0;

    ; Write 00120034 to 0x00000004 using word mode
    ; Note, for the purposes of testing, all longwords in memory are
    ; interpreted as being big-endian, which means that the CPU will read this
    ; as 00340012.
    MOV #$F, R1;
    ADD #$3, R1;
    MOV #6, R0;
    MOV.W R1, @R0;

    ADD #$F, R1;
    ADD #$F, R1;
    ADD #$4, R1;
    MOV #4, R0;
    MOV.W R1, @R0;

    ; Write FFFFFFFF to 0x00000008 using longword mode
    MOV #8, R0;
    MOV #-1, R1;
    MOV.L R1, @R0;

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;
    ; Extra NOPs to clear pipeline
    NOP
    NOP
    NOP

