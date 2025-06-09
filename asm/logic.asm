; logic.asm
;
; Testing ALU logic operations
;
; This file tests the following instructions:
;   NOT  Rm, Rn
;   AND  Rm, Rn
;   AND  #imm, R0
;   OR   Rm, Rn
;   OR   #imm, R0
;   XOR  Rm, Rn
;   XOR  #imm, R0
;   TST  Rm, Rn
;   TST  #imm, R0
;
; Revision History:
;   11 May 2025     Zack Huang      Initial revision.

ProgramStart:
    ; Test AND
    MOV #$3, R0
    MOV #$5, R1
    AND R1, R0      ; set R0 <- 3 & 5
    MOV #0, R1
    MOV R0, @R1     ; expect R0 <- 1

    ; Test AND immediate
    MOV #$E, R0
    AND #$5, R0     ; set R0 <- 0xE & 5
    MOV #4, R1
    MOV R0, @R1     ; expect R0 <- 4

    ; Test OR
    MOV #$3, R0
    MOV #$5, R1
    OR R1, R0       ; set R0 <- 3 | 5
    MOV #8, R1
    MOV R0, @R1     ; expect R0 <- 7

    ; Test OR immediate
    MOV #$E, R0
    OR #$5, R0      ; set R0 <- 0xE | 5
    MOV #12, R1
    MOV R0, @R1     ; expect R0 <- 0xF

    ; Test XOR
    MOV #$3, R0
    MOV #$5, R1
    XOR R1, R0      ; set R0 <- 3 ^ 5
    MOV #16, R1
    MOV R0, @R1     ; expect R0 <- 6

    ; Test XOR immediate
    MOV #$E, R0
    XOR #$5, R0
    MOV #20, R1     ; set R0 <- 0xE ^ 5
    MOV R0, @R1     ; expect R0 <- 0xB

    ; Test TST
    MOV #$2, R0
    MOV #$1, R1
    TST R1, R0      ; test 2 & 1
    STC SR, R0      ; store SR to R0
    AND #1, R0      ; mask to get T flag
    MOV #24, R1
    MOV R0, @R1     ; expect R0 <- 1

    MOV #$2, R0
    MOV #$2, R1
    TST R1, R0      ; test 2 & 2
    STC SR, R0      ; store SR to R0
    AND #1, R0      ; mask to get T flag
    MOV #28, R1
    MOV R0, @R1     ; expect R0 <- 0

    ; Test TST immediate
    MOV #$2, R0
    TST #1, R0      ; test 2 & 1
    STC SR, R0      ; store SR to R0
    AND #1, R0      ; mask to get T flag
    MOV #32, R1
    MOV R0, @R1     ; expect R0 <- 1

    MOV #$2, R0
    TST #2, R0      ; test 2 & 2
    STC SR, R0      ; store SR to R0
    AND #1, R0      ; mask to get T flag
    MOV #36, R1
    MOV R0, @R1     ; expect R0 <- 0

    ; Test NOT
    MOV #-1, R0
    XOR #$FF, R0    ;   R0 <- 0xFFFFFF00
    NOT R0, R0
    MOV #40, R1
    MOV R0, @R1     ;   Expect R0 = 0x000000FF

    ; Quit test program
    MOV #-4, R0
    MOV.B R0, @R0
    ; Extra NOPs to clear pipeline
    NOP
    NOP
    NOP
