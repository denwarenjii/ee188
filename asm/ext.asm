; ext.asm
;
; Testing all sign/zero extend instructions
;
; This file tests the following instructions:
;   EXTS.B  Rm, Rn
;   EXTS.W  Rm, Rn
;   EXTU.B  Rm, Rn
;   EXTU.W  Rm, Rn
;
; Revision History:
;   25 May 2025     Zack Huang      Initial revision.

ProgramStart:
    ; Test EXTU.B
    MOV #-1, R0;
    EXTU.B R0, R1;
    MOV #$0, R2;
    MOV R1, @R2;    Expect 000000FF

    ; Test EXTU.W
    MOV #-1, R0;
    EXTU.W R0, R1;
    MOV #$4, R2;
    MOV R1, @R2;    Expect 0000FFFF

    ; Test EXTS.B
    MOV #-1, R0;
    AND #$FF, R0;
    EXTS.B R0, R1;
    MOV #$8, R2;
    MOV R1, @R2;    Expect FFFFFFFF

    ; Test EXTS.W
    MOV #-1, R0;
    AND #$FF, R0;
    EXTS.W R0, R1;
    MOV #$C, R2;
    MOV R1, @R2;    Expect 000000FF

    MOV #-1, R0;
    AND #$FF, R0;

    SETT
    ROTCL R0;
    SETT
    ROTCL R0;
    SETT
    ROTCL R0;
    SETT
    ROTCL R0;
    SETT
    ROTCL R0;
    SETT
    ROTCL R0;
    SETT
    ROTCL R0;
    SETT
    ROTCL R0;       Set R0 to 0000FFFF
    
    EXTS.W R0, R1;
    MOV #$10, R2;
    MOV R1, @R2;    Expect FFFFFFFF

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;

