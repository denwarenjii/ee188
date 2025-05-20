; mov_rm_at_r0_rn.asm
;
; This file tests the following instructions:
;    MOV.B Rm, @(R0, Rn)
;    MOV.W Rm, @(R0, Rn)
;    MOV.L Rm, @(R0, Rn)
;
; Revision History:
;   19 May 2025  Chris M.  Initial revision.

Start:
    ; We will move a byte, word, and longword of 0x11223344 into 0x08, 0x0C,
    ; and 0x10 respectively. This range will be zeroed out to prevent "X"s.

    MOV ZeroVar, R2 ; Zero out 0x00, 0x04, and 0x08

    MOV   #$08, R0  
    MOV   R2,   @R0

    MOV   #$0C, R0
    MOV   R2,   @R0

    MOV   #$10, R0
    MOV   R2,   @R0


    MOV   Var, R2

    ; Test MOV.B Rm, @(R0, Rn)

    MOV #$02, R0    ; Our target address is 0x08 (8) and 0x02 + 0x06 = 0x08
    MOV #$06, R1

    MOV.B R2, @(R0,R1) ; We expect 0x44 at 0x08

    ; Test MOV.W Rm, @(R0, Rn)

    MOV #$08, R0    ; Our target address is 0x0C (12) and 0x08 + 0x04  = 0x0C
    MOV #$04, R1

    MOV.W R2, @(R0,R1) ; We expect 0x3344 at 0x0C

    ; Test MOV.L Rm, @(R0, Rn)

    MOV #$0C, R0    ; Our target address is 0x10 (16) and 0x0C + 0x04  = 0x10
    MOV #$04, R1

    MOV.L R2, @(R0,R1) ; We expect 0x11223344 at 0x10

    ; Expected memory layout
    ; 00000008  44000000
    ; 0000000C  33440000
    ; 00000010  11223344

End:
  ; The test bench interprets a read of 0xFFFFFFFC (-4)
  ;as system exit.
  MOV #-4, R0;
  MOV.B R0, @R0;

    align 4
    LTORG

ZeroVar:  dc.l $00000000
Var:      dc.l $11223344
