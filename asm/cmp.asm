; cmp.asm
;
; Testing all comparison operations
;
; This file tests the following instructions:
;   CMP/EQ  #imm,RO
;   CMP/EQ  Rm, Rn
;   CMP/HS  Rm, Rn
;   CMP/GE  Rm, Rn
;   CMP/HI  Rm, Rn
;   CMP/GT  Rm, Rn
;   CMP/PL  Rn
;   CMP/PZ  Rn
;   CMP/STR Rm, Rn
;
; Revision History:
;   25 May 2025     Zack Huang      Initial revision.

ProgramStart:
    ; Test CMP/EQ (when unequal)
    MOV #50, R1;
    MOV #40, R2;

    CMP/EQ R1, R2;  Should set T = 0
    STC SR, R0;
    AND #1, R0;
    MOV #$00, R3;
    MOV R0, @R3;    Expect R0 = 00000000

    ; Test CMP/EQ (when equal)
    MOV #41, R1;
    MOV #41, R2;

    CMP/EQ R1, R2;  Should set T = 1
    STC SR, R0;
    AND #1, R0;
    MOV #$04, R3;
    MOV R0, @R3;    Expect R0 = 00000001

    ; Test CMP/HS
    MOV #$10, R1;
    MOV #$30, R2;
    CMP/HS R1, R2;  Should set T = 1
    STC SR, R0;
    AND #1, R0;
    MOV #$08, R3;
    MOV R0, @R3;    Expect R0 = 00000001

    CMP/HS R2, R1;  Should set T = 0
    STC SR, R0;
    AND #1, R0;
    MOV #$0C, R3;
    MOV R0, @R3;    Expect R0 = 00000000

    ; Test CMP/GE
    MOV #-4, R1;
    MOV #10, R2;
    CMP/GE R1, R2;  Should set T = 1
    STC SR, R0;
    AND #1, R0;
    MOV #$10, R3;
    MOV R0, @R3;    Expect R0 = 00000001

    CMP/GE R2, R1;  Should set T = 0
    STC SR, R0;
    AND #1, R0;
    MOV #$14, R3;
    MOV R0, @R3;    Expect R0 = 00000000

    ; Test CMP/HI
    MOV #-1, R1;
    MOV #-1, R2;
    CMP/HI R1, R2;  Should set T = 0
    STC SR, R0;
    AND #1, R0;
    MOV #$18, R3;
    MOV R0, @R3;    Expect R0 = 00000000

    MOV #10, R1;
    MOV #-1, R2;
    CMP/HI R1, R2;  Should set T = 1
    STC SR, R0;
    AND #1, R0;
    MOV #$1C, R3;
    MOV R0, @R3;    Expect R0 = 00000001

    CMP/HI R2, R1;  Should set T = 0
    STC SR, R0;
    AND #1, R0;
    MOV #$20, R3;
    MOV R0, @R3;    Expect R0 = 00000000

    ; Test CMP/GT
    MOV #-1, R1;
    MOV #-1, R2;
    CMP/GT R1, R2;  Should set T = 0
    STC SR, R0;
    AND #1, R0;
    MOV #$24, R3;
    MOV R0, @R3;    Expect R0 = 00000000

    MOV #10, R1;
    MOV #-1, R2;
    CMP/GT R1, R2;  Should set T = 0
    STC SR, R0;
    AND #1, R0;
    MOV #$28, R3;
    MOV R0, @R3;    Expect R0 = 00000000

    CMP/GT R2, R1;  Should set T = 1
    STC SR, R0;
    AND #1, R0;
    MOV #$2C, R3;
    MOV R0, @R3;    Expect R0 = 00000001

    ; Test CMP/EQ #imm, Rn
    MOV #10, R0;
    CMP/EQ #8, R0;
    STC SR, R0;
    AND #1, R0;
    MOV #$30, R3;
    MOV R0, @R3;    Expect R0 = 00000000

    MOV #10, R0;
    CMP/EQ #10, R0;
    STC SR, R0;
    AND #1, R0;
    MOV #$34, R3;
    MOV R0, @R3;    Expect R0 = 00000001

    ; Test CMP/PL Rn
    MOV #0, R0;
    CMP/PL R0;
    STC SR, R0;
    AND #1, R0;
    MOV #$38, R3;
    MOV R0, @R3;    Expect R0 = 00000000

    MOV #10, R0;
    CMP/PL R0;
    STC SR, R0;
    AND #1, R0;
    MOV #$3C, R3;
    MOV R0, @R3;    Expect R0 = 00000001

    MOV #-10, R0;
    CMP/PL R0;
    STC SR, R0;
    AND #1, R0;
    MOV #$40, R3;
    MOV R0, @R3;    Expect R0 = 00000000

    ; Test CMP/PL Rn
    MOV #0, R0;
    CMP/PZ R0;
    STC SR, R0;
    AND #1, R0;
    MOV #$44, R3;
    MOV R0, @R3;    Expect R0 = 00000001

    MOV #10, R0;
    CMP/PZ R0;
    STC SR, R0;
    AND #1, R0;
    MOV #$48, R3;
    MOV R0, @R3;    Expect R0 = 00000001

    MOV #-10, R0;
    CMP/PZ R0;
    STC SR, R0;
    AND #1, R0;
    MOV #$4C, R3;
    MOV R0, @R3;    Expect R0 = 00000000

    ; Test CMP/STR
    MOV #$0, R1;
    MOV #-1, R2;
    CMP/STR R0, R2;
    STC SR, R0;
    AND #1, R0;
    MOV #$50, R3;
    MOV R0, @R3;    Expect R0 = 00000000

    MOV #$0, R1;
    MOV #$A, R2;
    CMP/STR R0, R2;
    STC SR, R0;
    AND #1, R0;
    MOV #$54, R3;
    MOV R0, @R3;    Expect R0 = 00000001

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;
