; arith.asm
;
; Testing basic ALU operations
;
; This file tests the following instructions:
;     NEG Rm, Rn
;     ADD Rm, Rn
;     SUB Rm, Rn
;     DT Rn
;
; Revision History:
;   11 May 2025     Zack Huang      Initial revision.

ProgramStart:
    ; Test negation
    MOV #1, R1      ; R1 <- 1
    NEG R1, R1      ; Negate R1
    MOV #$0, R0;
    MOV R1, @R0;    ; Output R1 to 0x00 (expect -1)

    NEG R1, R1      ; Negate R1
    MOV #$4, R0 
    MOV R1, @R0     ; Output R1 to 0x04 (expect 1)

    ; Test addition
    MOV #1, R1      ; R1 <- 1
    ADD #8, R1      ; Add 8 to R1
    MOV #$8, R0;
    MOV R1, @R0;    ; Output R1 to 0x08 (expect 9)

    ; Test subtraction
    MOV #1, R1      ; R1 <- 1
    MOV #8, R2      ; R2 <- 8
    SUB R2, R1      ; Subtract R2 from R1
    MOV #$C, R0
    MOV R1, @R0     ; Output R1 to 0x0C (expect -7)

    ; Test DT
    MOV #2, R2      ; R2 <- 2
    DT R2           ; Decrement R2
    MOV #$10, R1
    MOV R2, @R1     ; Output R2 (1) to 0x10

    STC SR, R0      ; Store SR to R0
    AND #1, R0      ; mask out T bit
    MOV #$14, R1
    MOV R0, @R1     ; Output R0 to 0x14 (expect 0)

    DT R2           ; Decrement R2
    MOV #$18, R1
    MOV R2, @R1     ; Output R2 to 0x18 (expect 0)

    STC SR, R0      ; Store SR to R0
    AND #1, R0      ; mask out T bit
    MOV #$1C, R1
    MOV R0, @R1     ; Output R0 to 0x1C (expect 1)

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;
