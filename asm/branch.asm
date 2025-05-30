;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                              
; branch.asm                                                                   
;                                                                              
; This file tests the following instructions:                                  
;
;   BF    <label>                                                             
;   BF/S  <label>
;   BT    <label>
;   BT/S  <label>
;
; Branches are tested by writing to memory based on address we jump to. Note 
; that if jumps do not work at all, we will encounter the "exit" signal and
; the expected memory will be incomplete. The execution of the instructions in
; the delay slot is tested by seeing if the contents of a register are altered
; or not when we arrive at the target address.
;
; Revision History:                                                            
;   26 May 2025  Chris M.  Initial revision.                                   
;                                                                             
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Start:

    ; Test BF. Write at 0x00

    CLRT              ; Clear the T flag.
    BT    TRGET_T0    ; This branch should not be taken.
    BF    TRGET_F0    ; This branch is taken. The PC should be changed to TRGET_F0
    NOP
    NOP

TRGET_T0:

    MOV   TrueVar,  R1  ; Write TrueVar @ 0x00 to detect wrong branch.
    MOV   #$00,     R0
    MOV   R1,       @R0

TRGET_T1:

    MOV   TrueVar,  R1  ; Write TrueVar @ 0x08 to detect wrong branch.
    MOV   #$08,     R0
    MOV   R1,       @R0

End_T0:

    ; The test bench interprets a read of 0xFFFFFFFC (-4)
    ; as system exit.
    MOV     #-4,  R0
    MOV.B   R0,   @R0


TRGET_F0:

    ; Test BF/S. Write at 0x04 and 0x08

    MOV   FalseVar,   R1    ; Write FalseVar at 0x00
    MOV   #$00,       R0    
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

    MOV   #$04,  R0   ; If the delay slot was executed, then 0x0D is in R3.
    MOV   R3,    @R0  ; Write 0x0D to 0x04

    MOV   FalseVar,   R1  ; Write FalseVar at 0x08
    MOV   #$08,       R0
    MOV   R1,         @R0

    ; Test BT. Write at 0x0C

    SETT              ; Set the T flag
    BF    TRGET_F2    ; This branch should not be taken.
    BT    TRGET_T2    ; This branch is taken.
    NOP
    NOP

TRGET_F2:
     
    MOV   FalseVar,   R1  ; Write FalseVar at 0x0C to detect wrong branch.
    MOV   #$0C,       R0
    MOV   R1,         @R0

TRGET_T2:

    MOV   TrueVar,  R1 ; Write TrueVar at 0x0C
    MOV   #$0C,     R0
    MOV   R1,       @R0

End:

    ; The test bench interprets a read of 0xFFFFFFFC (-4)
    ; as system exit.
    MOV     #-4,  R0;   ; PC = 0x1A
    MOV.B   R0,   @R0;  ; PC = 0x1C

   
    ; Expected memory:
    ; 00000000 BBBBBBBB  ; FalseVar is written to 0x00 if `BF` jumps to the correct target.
    ; 00000004 0000000D  ; 0x0D (13) is written to 0x04 if the delayed slot of a `BF/S` is executed.
    ; 00000008 BBBBBBBB  ; FalseVar is written to 0x08 if `BF/S` jumps to the correct target.
    ; 0000000C AAAAAAAA  ; TrueVar is writen to 0x0C if `BT` jumps to the correct target.

    align 4
    LTORG

TrueVar:    dc.l $AAAAAAAA
FalseVar:   dc.l $BBBBBBBB
