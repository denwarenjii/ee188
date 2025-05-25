; mov_bwl_at_disp_gbr.asm
;
; This file tests the following instructions:
;   MOV.B R0, @(disp, GBR)
;   MOV.W R0, @(disp, GBR)
;   MOV.L R0, @(disp, GBR)
;
; Revision History:
;   24 May 2025  Chris M.  Initial revision.
;
Start:
    ; Use the instructions under test to move to 0x20, 0x24, and 0x28.
    ; Zero out locations that don't move longwords to avoid unitialized values.

    MOV     ZeroVar, R0

    MOV     #$20, R1      ; Zero out 0x20
    MOV     R0,   @R1

    MOV     #$24, R1      ; Zero out 0x24
    MOV     R0,   @R1

    MOV     #$28, R1      ; Zero out 0x28
    MOV     R0,   @R1


    MOV     #$10, R0       ; Load 0x10 into the GBR
    LDC     R0,   GBR
    
    MOV     Var, R0        ; Load 0xDEADBEEF into R0

    MOV.B   R0, @($10,GBR) ; Move 0xEF into 0x20.

    MOV.W   R0, @($14,GBR) ; Move 0xBEEF into 0x24.

    MOV.L   R0, @($18,GBR) ; Move 0xDEADBEEF into 0x28.

    ; Expected memory layout
    ;
    ; 00000020 FE000000
    ; 00000024 BEEF0000
    ; 00000028 DEADBEEF

End:
  ; The test bench interprets a read of 0xFFFFFFFC (-4)
  ;as system exit.
  MOV #-4, R0;
  MOV.B R0, @R0;

    align 4
    LTORG

ZeroVar:  dc.l $00000000
Var:      dc.l $DEADBEEF


