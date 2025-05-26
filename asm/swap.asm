; mov_at_disp_gbr_r0.asm
;
; This file tests the following instructions:
;     SWAP.B Rm, Rn
;     SWAP.W Rm, Rn
;
;   SWAP.B swaps the upper and lower halves of the lower two bytes of Rm and moves
;   them into Rn.
;
;   SWAP.W swaps the upper and lower words of Rm and moves them into Rn.
;
; Revision History:
;   25 May 2025  Chris M.  Initial revision.
;
Start:
    MOV     Var,  R0

    SWAP.B  R0,   R1    ; 0x12345678 -> 0x12347856

    MOV     #$00, R2    ; Write result at address 0x00
    MOV     R1,   @R2

    SWAP.W  R0,   R1    ; 0x12345678 -> 0x56781234

    MOV     #$04, R2    ; Write result at address 0x04
    MOV     R1,   @R2

    ; Expected memory:
    ; 00000000 12347856
    ; 00000004 56781234

End:
  ; The test bench interprets a read of 0xFFFFFFFC (-4)
  ;as system exit.
  MOV #-4, R0;
  MOV.B R0, @R0;



    align 4 ; long-word align program memory.
    LTORG

; Long-word constants (in program memory).
;
ZeroVar:  dc.l $00000000
Var:      dc.l $12345678
