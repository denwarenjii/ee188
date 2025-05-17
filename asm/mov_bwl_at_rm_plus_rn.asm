; mov_bwl_at_rm_plus_rn.asm
;
; This file tests the following instructions:
;    MOV.B @Rm+, Rn
;    MOV.W @Rm+, Rn
;    MOV.L @Rm+, Rn
;
; It assumes that moving into memory works. 
;
; Revision History:
;   16 May 2025  Chris M.  Initial revision.
Start:
    ; We set up the memory as follows:
    ;
    ; 78: 12 34 56 78
    ; 7C: 11 22 33 44

    MOV   #$12, R1  ;0x12 at 0x78
    MOV   #$78, R0
    MOV.B R1,   @R0

    MOV   #$34, R1  ;0x34 at 0x79
    MOV   #$79, R0
    MOV.B R1,   @R0

    MOV   #$56, R1  ;0x56 at 0x7A
    MOV   #$7A, R0
    MOV.B R1,   @R0

    MOV   #$78, R1  ;0x78 at 0x7B
    MOV   #$7B, R0
    MOV.B R1,   @R0

    MOV   #$11, R1  ;0x11 at 0x7C
    MOV   #$7C, R0
    MOV.B R1,   @R0

    MOV   #$11, R1  ;0x11 at 0x7C
    MOV   #$7C, R0
    MOV.B R1,   @R0


