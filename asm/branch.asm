;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                              ;
; branch.asm                                                                   ;
;                                                                              ;
; This file tests the following instructions:                                  ;
;                                                                              ;
; Revision History:                                                            ;
;   26 May 2025  Chris M.  Initial revision.                                   ;
;                                                                              ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Start:

    ; Example:
    ;   CLRT          T is always cleared to 0
    ;   BT TRGET_T    Does not branch, because T = 0
    ;   BF TRGET_F    Branches to TRGET_F, because T = 0
    ;   NOP           
    ;   NOP           <- The PC location is used to calculate the branch
    ;                    destination address of the BF instruction
    ;
    ;   TRGET_F:

    CLRT              ; PC = 0x00
    BT    TRGET_T     ; PC = 0x02
    BF    TRGET_F     ; PC = 0x04
    NOP               ; PC = 0x06
    NOP               ; PC = 0x08

TRGET_T:

    MOV   TrueVar,  R1  ; PC = 0x0A
    MOV   #$00,     R0  ; PC = 0x0C
    MOV   R1,       @R0 ; PC = 0x0E

End_T:

    ; The test bench interprets a read of 0xFFFFFFFC (-4)
    ; as system exit.
    MOV     #-4,  R0;   ; PC = 0x10
    MOV.B   R0,   @R0;  ; PC = 0x12


TRGET_F:

    MOV   FalseVar,  R1 ; PC = 0x14
    MOV   #$00,     R0  ; PC = 0x16
    MOV   R1,       @R0 ; PC = 0x18

End_F:

    ; The test bench interprets a read of 0xFFFFFFFC (-4)
    ; as system exit.
    MOV     #-4,  R0;   ; PC = 0x1A
    MOV.B   R0,   @R0;  ; PC = 0x1C

    ; Note that we expect the false branch to be taken.

    ; Expected memory if true branch is taken:
    ;
    ;   00000000  AAAAAAAA


    ; Expected memory if false branch is taken:
    ;
    ;   00000000  BBBBBBBB
    

    align 4
    LTORG

TrueVar:    dc.l $AAAAAAAA
FalseVar:   dc.l $BBBBBBBB
