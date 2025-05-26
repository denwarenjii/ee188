; xtrct.asm
;
; This file tests the following instruction:
;     XTRCT Rm, Rn
;
; This instruction moves the center 32 bits of registers Rm and Rn into Rn.
; It takes the low word of Rm and moves it into the high word of Rn, and
; moves the high word of Rn into the low word of Rn.
;
; Revision History:
;   25 May 2025  Chris M.  Initial revision.
;




Start:
    ; XTRCT RO,R1   Before execution RO = H'01234567, R1 = H'89ABCDEF
    ;               After execution  R1 = H'456789AB

    MOV   Var1,   R0
    MOV   Var2,   R1

    XTRCT R0,     R1

    MOV   #$00,   R2
    MOV   R1,     @R2

    ; Expected memory:
    ; 00000000 456789AB

End:
  ; The test bench interprets a read of 0xFFFFFFFC (-4)
  ;as system exit.
  MOV #-4, R0;
  MOV.B R0, @R0;



    align 4 ; long-word align program memory.
    LTORG

; Long-word constants (in program memory).
;
Var1:      dc.l $01234567
Var2:      dc.l $89ABCDEF

