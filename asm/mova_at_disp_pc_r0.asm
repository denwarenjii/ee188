; mova_at_disp_pc_r0.asm;
;
; This file tests the following instruction:
;     MOVA @(disp, PC), R0
;
; Revision History:
;   25 May 2025  Chris M.  Initial revision.
Start:

    ; Usage: MOV <var name>, R0
    ; NOTE: Can't do MOVA @(disp, PC), R0 directly

    ; 00000000: c702  ..  MOVA   @(disp, PC), R0 
    ; 00000002: e100  ..  MOV    #00,         R1
    ; 00000004: 2102  !.  MOV    R0,         @R1
    ; 00000006: e0fc  ..  MOV    #-4,         R0
    ; 00000008: 2000   .  MOV.B  R0,         @R0
    ; 0000000a: ffff  ..  .align 4
    ; 0000000c: 0000  ..  ZeroVar
    ; 0000000e: 0000  ..  ZeroVar
    ; 00000010: 1122  ."  Var
    ; 00000012: 3344  3D  Var


    MOVA Var, R0    ; PC = 0x00. Since Var starts at ROM address 0x10, we expect
                    ; 0x10 in R0

    MOV   #00,  R1  ; PC = 0x02
    MOV   R0,   @R1 ; PC = 0x04

    ; Expected memory:
    ; 00000000 00000010

End:
  ; The test bench interprets a read of 0xFFFFFFFC (-4)
  ;as system exit.
  MOV #-4, R0;      ; PC = 0x06
  MOV.B R0, @R0;    ; PC = 0x08


    ; long-word align program memory.
    align 4         ; PC = 0x0A
    LTORG

; Long-word constants (in program memory).
;
ZeroVar:  dc.l $00000000  ; PC = 0x0C (for 0x0000), PC = 0x0E (for 0x0000)
Var:      dc.l $11223344  ; PC = 0x10 (for 0x1122), PC = 0x12 (for 0x3344)
