; mov_b_at_r0_rm_rn.asm
;
; This file tests the following instructions:
;   MOV.B @(R0, Rm), Rn
;   MOV.W @(R0, Rm), Rn
;   MOV.L @(R0, Rm), Rn
;
; It assumes that moving into memory works. 
;
; Revision History:
;   19 May 2025  Chris M.  Initial revision.

Start:
    ; Write 0x1234567 at 0x14. Read byte, word, and longword from 0x14 using
    ; instructions under test and write the result at 0x00, 0x04, and 0x08 
    ; respectively. 


    MOV ZeroVar, R0

    MOV   #$00, R1    ; Zero out 0x00, 0x04, and 0x08
    MOV   R0,   @R1

    MOV   #$04, R1
    MOV   R0,   @R1

    MOV   #$08, R1
    MOV   R0,   @R1
    
    MOV   Var,  R0  ; Write 0x12345678 at 0x14
    MOV   #$14, R1
    MOV   R0,   @R1


    ; Test MOV.B @(R0, Rm), Rn

    MOV     #$0C,     R0  ; 0x0C + 0x8 = 0x14, our target address.
    MOV     #$08,     R1
    MOV.B   @(R0,R1), R2

    MOV   #$00, R3        ; We expect signExtend(0x12) = 0x00000012 at 0x00
    MOV   R2,   @R3

    ; Test MOV.W @(R0, Rm), Rn

    MOV     #$0A,     R0  ; 0x0A + 0x0A = 0x14, our target address.
    MOV     #$0A,     R1
    MOV.W   @(R0,R1), R2

    MOV   #$04, R3        ; We expect signExtend(0x1234) = 0x00001234 at 0x00
    MOV   R2,   @R3

    ; Test MOV.L @(R0, Rm), Rn

    MOV     #$10,     R0  ; 0x10 + 0x04 = 0x14, our target address.
    MOV     #$04,     R1
    MOV.L   @(R0,R1), R2

    MOV   #$08, R3        ; We expect 0x12345678 at 0x00
    MOV   R2,   @R3

    ; Expected memory layout:

    ;00000000 12000000
    ;00000004 12340000
    ;00000008 12345678

End:
    ; The test bench interprets a read of 0xFFFFFFFC (-4)
    ;as system exit.
    MOV #-4, R0;
    MOV.B R0, @R0;
    ; Extra NOPs to clear pipeline
    NOP
    NOP
    NOP

    align 4
    LTORG

ZeroVar:  dc.l $00000000
Var:      dc.l $12345678
