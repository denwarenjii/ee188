; mov_bwl_r0_or_rm_at_disp_rn.asm
;
; This file tests the following instructions:
;    MOV.B R0, @(disp, Rn)
;    MOV.W R0, @(disp, Rn)
;    MOV.L Rm, @(disp, Rn)
;
; It assumes that moving into memory and move with PC relative addressing 
; works. Move with PC relative addressing is used to load long-word constants
; into registers.
;
; Revision History:
;   16 May 2025  Chris M.  Initial revision.

Start:

    ; Zero out words at 0x08, 0x0C, and 0x14
    MOV ZeroVar, R0

    MOV #$08, R1
    MOV R0, @R1

    MOV #$0C, R1
    MOV R0, @R1

    MOV #$14, R1
    MOV R0, @R1

    MOV Var, R0 ; Move 0x12345678 into R0

    MOV #04, R1
    MOV.B R0, @(4,R1)   ; Target address = 0x04 + zeroExtend(0x04) = 0x08
                        ; We expected 0x78 at 0x08

    MOV #04, R1
    MOV.W R0, @(8,R1)   ; Target address = 0x04 + zeroExtend(0x08) = 0x0C
                        ; We expect 0x5678 at 0x0C

    MOV Var, R1 ; Move 0x12345678 into R1

    MOV #$04, R2
    MOV.L R1, @(16,R2)   ; Target address = 0x04 + zeroExtend(0x16) = 0x14
                         ; We expect 0x12345678 at 0x14

    ; 00000008 78000000
    ; 0000000A 78560000
    ; 00000014 12345678

Done:
    ; The test bench interprets a read of 0xFFFFFFFC (-4) 
    ; as system exit.
    MOV   #-4, R0;
    MOV.B R0, @R0;
    ; Extra NOPs to clear pipeline
    NOP
    NOP
    NOP

    align 4
    LTORG

Var: dc.w $1234
     dc.w $5678

ZeroVar: dc.w $0000
         dc.w $0000

