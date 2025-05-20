; mov_bwl_at_disp_rm_r0_or_rn.asm
;
; This file tests the following instructions:
;    MOV.B @(disp, Rm), R0
;    MOV.W @(disp, Rm), R0
;    MOV.L @(disp, Rm), Rn
;
; It assumes that moving into memory works. We write 0xDEADBEEF at address
; 0x10 and then use the move instruction under test to move a byte, word, and
; longword of this into a register. The byte is written to 0x00, word to
; 0x04, and longword to 0x08.
;
; Revision History:
;   19 May 2025  Chris M.  Initial revision.

Start:

    MOV   Var,  R0  ; Write 0xDEADBEEF to 0x10
    MOV   #$10, R1
    MOV   R0,   @R1


    ; Test MOV.B @(disp, Rm), R0

    MOV     #$0C,      R1
    MOV.B   @(4,R1),   R0  ;0x0C + 0x4 = 0x10, our target address.

    MOV   #$00,   R1    ; The low byte of 0xDEADBEEF is 0xDE. Sign-extended
                        ; into a register this is 0xFFFFFFDE, which we expect
                        ; at 0x00.

    MOV   R0,   @R1   



    ; Test MOV.W @(disp, Rm), R0

    MOV     #$0A,      R1
    MOV.W   @(6,R1),   R0 ; 0x0A + 0x06 = 0x10, our target address.

    MOV   #$04, R1    ; The low word of 0xDEADBEEF is 0xDEAD. Sign-extended 
                      ; into a register this is is 0xFFFFDEAD, which we expect
                      ; at 0x04

    MOV   R0,   @R1



    ; Test MOV.L @(disp, Rm), Rn

    MOV     #$08,      R1
    MOV.L   @(8,R1),   R2 ; 0x08 + 0x08 = 0x10, our target address.

    MOV   #$08,   R1  ; We expect 0xDEADBEEF at 0x08.
    MOV   R2,     @R1
    

    ; Expected memory layout:

    ; 00000000 FFFFFFDE
    ; 00000004 FFFFDEAD
    ; 00000008 DEADBEEF

End:
  ; The test bench interprets a read of 0xFFFFFFFC (-4)
  ;as system exit.
  MOV #-4, R0;
  MOV.B R0, @R0;


    align 4
    LTORG

ZeroVar:  dc.l $00000000
Var:      dc.l $DEADBEEF
