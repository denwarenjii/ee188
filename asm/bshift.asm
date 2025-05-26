ProgramStart:
    ; Test SHLL2
    MOV #$F, R0;
    SHLL2 R0;
    MOV #$0, R1;
    MOV R0, @R1;    Expect: R0 = 0x000000F0

    ; Test SHLR2
    MOV #$F, R0;
    SHLR2 R0;
    MOV #$4, R1;
    MOV R0, @R1;    Expect: R0 = 0x000000F0

    ; Test SHLL8
    MOV #$F, R0;
    SHLL8 R0;
    MOV #$8, R1;
    MOV R0, @R1;    Expect: R0 = 0x0000F000

    ; Test SHLR8
    SHLR8 R0;
    MOV #$C, R1;
    MOV R0, @R1;    Expect: R0 = 0x0000000F

    ; Test SHLL16
    MOV #$F, R0;
    SHLL16 R0;
    MOV #$10, R1;
    MOV R0, @R1;    Expect: R0 = 0x000F0000

    ; Test SHLR8
    SHLR16 R0;
    MOV #$14, R1;
    MOV R0, @R1;    Expect: R0 = 0x0000000F

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;
