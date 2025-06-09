; branch_conditional_delay.asm
;
; Testing conditional branches (not delayed branch). As per the spec, condition
; verification is performed in the ID stage.
;
; This file tests the following instructions:
;     BT/S
;     BF/S
;
; Revision History:
;   11 May 2025     Zack Huang      Initial revision.

ProgramStart:
    ; Set memory at certain addresses
    MOV #$1, R0
    MOV #$0, R1
    MOV #$4, R2

    MOV R0, @R1         ; Writes 1 to 0x00
    MOV R0, @R2         ; Writes 1 to 0x04

    MOV #$A, R3
    MOV #$B, R4

    MOV #$F, R0
    CMP/EQ #$F, R0
    BT/S BranchIfTrue
    ; Test that branch slots are not executed
    MOV R3, @R1         ; Should set 0x00 to 0xA
    MOV R4, @R2         ; Should not set 0x04 to 0xB
    NOP

BranchIfFalse:
    MOV #$8, R1
    MOV #$C, R2
    MOV R2, @R1         ; This should not be executed

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;
    ; Extra NOPs to clear pipeline
    NOP
    NOP
    NOP

BranchIfTrue:
    MOV #$8, R1
    MOV #$D, R2
    MOV R2, @R1         ; Should set 0x08 to 0xD

    MOV #$E, R0
    CMP/EQ #$F, R0
    BF/S BranchIfFalseAgain
    NOP

    ; Quit test program (should not happen)
    MOV #-4, R0;
    MOV.B R0, @R0;
    ; Extra NOPs to clear pipeline
    NOP
    NOP
    NOP

BranchIfFalseAgain:
    MOV #$C, R1
    MOV #$2, R2
    MOV R2, @R1         ; Should set 0x0C to 0x2

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;
    ; Extra NOPs to clear pipeline
    NOP
    NOP
    NOP

