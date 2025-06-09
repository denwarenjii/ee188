; movt_rn.asm
;
; This file tests the following instruction:
;     MOVT Rn
;
; Revision History:
;   25 May 2025  Chris M.  Initial revision.
;
Start:

; Example:
;
;     XOR     R2,R2 ; R2 = 0
;     CMP/PZ  R2    ; T  = 1
;     MOVT    RO    ; RO = 1
;     CLRT          ; T  = 0
;     MOVT    R1    ; R1 = 0

    MOV     #0,   R2
    XOR     R2,   R2  ; Anything XORed with itself is zero
    CMP/PZ  R2        ; Compare to zero sets T=1 because the result is zero.

    MOVT    R0        ; Write T to 0x00. We expect 0x00000001 at 0x00
    MOV     #$00, R1
    MOV     R0,   @R1

    CLRT              ; Clear T flag.

    MOVT    R0        ; Write T to 0x04. We expect 0x00000000 at 0x04
    MOV     #$04, R1
    MOV     R0,  @R1

    ; Expected memory:
    ; 00000000 00000001
    ; 00000004 00000000

End:
    ; The test bench interprets a read of 0xFFFFFFFC (-4)
    ; as system exit.
    MOV #-4, R0;
    MOV.B R0, @R0;
    ; Extra NOPs to clear pipeline
    NOP
    NOP
    NOP


    align 4
    LTORG

ZeroVar:  dc.l $00000000
Var:      dc.l $DEADBEEF
