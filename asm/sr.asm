; sr.asm
;
; Testing status register commands
;
; This file tests the following instructions:
;   SETT
;   CLRT
;   STC Rm, SR
;
; Revision History:
;   11 May 2025     Zack Huang      Initial revision.

ProgramStart:
    ; Set T bit
    SETT            ; set T bit
    STC SR, R0      ; store SR to R0
    AND #1, R0      ; mask T bit
    MOV #0, R1
    MOV R0, @R1     ; expect R0 <- 1

    ; Clear T bit
    CLRT            ; clear T bit
    STC SR, R0      ; store SR to R0
    AND #1, R0      ; mask T bit
    MOV #4, R1
    MOV R0, @R1     ; expect R0 <- 0

    ; Load A into SR
    MOV #$A, R0
    LDC R0, SR      ; load 0xA into SR
    STC SR, R2      ; store SR to R2
    MOV #8, R1
    MOV R2, @R1     ; expect R2 <- 0xA

    ; Quit test program
    MOV #-4, R0
    MOV.B R0, @R0
