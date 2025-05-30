;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                              ;
; branch.asm                                                                   ;
;                                                                              ;
; This file tests the following instructions:                                  ;
;   BF <label>                                                                 ;
; Revision History:                                                            ;
;   26 May 2025  Chris M.  Initial revision.                                   ;
;                                                                              ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Start:
    ; The branches will be taken in this order if their implementations are 
    ; correct:
    ;
    ;   BF    TRGET_F0
    ;   BF/S  TRGET_F1
    ;   BT    TRGET_T0
    ;   BT/S  TRGET_T1

    ; BF test
    ;
    ; Example:
    ;   CLRT          T is always cleared to 0
    ;   BT TRGET_T    Does not branch, because T = 0
    ;   BF TRGET_F    Branches to TRGET_F, because T = 0
    ;   NOP           
    ;   NOP           <- The PC location is used to calculate the branch
    ;                    destination address of the BF instruction
    ;
    ;   TRGET_F:

    CLRT
    BT    TRGET_T0
    BF    TRGET_F0
    NOP
    NOP

TRGET_T0:

    MOV   TrueVar,  R1
    MOV   #$00,     R0
    MOV   R1,       @R0

TRGET_T1:

    MOV   TrueVar,  R1
    MOV   #$08,     R0
    MOV   R1,       @R0

End_T0:

    ; The test bench interprets a read of 0xFFFFFFFC (-4)
    ; as system exit.
    MOV     #-4,  R0
    MOV.B   R0,   @R0


TRGET_F0:

    MOV   FalseVar,   R1
    MOV   #$00,       R0    ; Write FalseVar at 0x00
    MOV   R1,         @R0

    MOV   #6,   R2
    MOV   #7,   R3

    CLRT            ; Clear the T flag.
    
    BF/S  TRGET_F1  ; Branch to TRGET_F1 and exeucte the delay slot.
    ADD   R2, R3    ; 6 + 7 = 13 should be in R3 when we get to TRGET_F1.

    BT/S  TRGET_T1  ; This branch should not be taken.
    NOP

    NOP

TRGET_F1:

    MOV   #$04,  R0   ; If the delay slot was executed, then 13 is in R3.
    MOV   R3,    @R0  ; Write 13 (0x0D) to 0x04

    MOV   FalseVar,   R1  ; Write FalseVar at 0x08
    MOV   #$08,       R0
    MOV   R1,         @R0


End_F0:

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
