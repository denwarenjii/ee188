; mov_at_disp_gbr_r0.asm
;
; This file tests the following instructions:
;     MOV.B @(disp, GBR), R0
;     MOV.W @(disp, GBR), R0
;     MOV.L @(disp, GBR), R0
;
; This test writes 0x11223344 at address 0x10 and uses the instructions under
; test to read the contents into R0. The byte, word, and longword are written
; to 0x00, 0x04, and 0x08 respectively.
;
; Revision History:
;   25 May 2025  Chris M.  Initial revision.
;
Start:

    MOV   ZeroVar, R0 ; Zero out 0x00, 0x04, and 0x08

    MOV   #$00, R1
    MOV   R0,   @R1

    MOV   #$04, R1
    MOV   R0,   @R1

    MOV   #$08, R1
    MOV   R0,   @R1

    MOV   Var,  R0  ; Read 0x11223344 into R0. Note that this is converted to
                    ; a PC-relative move by the assembler.

    MOV   #$10, R1  ; Write at 0x10
    MOV   R0,   @R1

    
    MOV   #$4,  R0  ; Load 0x04 into GBR
    LDC   R0,   GBR

    MOV.B @($0C,GBR), R0 ; 0x0C + 0x04 = 0x10, our target address. 
                         ; We expect to read 0x11

    MOV   #$00, R1       ; Write read memory contents at 0x00
    MOV   R0,   @R1


    MOV.W @($0C,GBR), R0 ; We expect to read 0x1122

    MOV   #$04, R1       ; Write read memory contents at 0x04
    MOV   R0,   @R1

    MOV.L @($0C,GBR), R0 ; We expect to read 0x11223344

    MOV   #$08, R1       ; Write read memory contents at 0x08
    MOV   R0,   @R1

    ; Expected memory
    ; 00000000 00000011
    ; 00000004 00001122
    ; 00000008 11223344

End:
    ; The test bench interprets a read of 0xFFFFFFFC (-4)
    ;as system exit.
    MOV #-4, R0;
    MOV.B R0, @R0;
    ; Extra NOPs to clear pipeline
    NOP
    NOP
    NOP



    align 4 ; long-word align program memory.
    LTORG

; Long-word constants (in program memory).
;
ZeroVar:  dc.l $00000000
Var:      dc.l $11223344  
