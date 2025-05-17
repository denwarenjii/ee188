ProgramStart:
    ; Test LDC
    MOV #$F, R0;
    LDC R0, SR;     SR  <- 0x0000000F

    ADD #1, R0;
    LDC R0, GBR;    GBR <- 0x00000100

    ADD #1, R0;
    LDC R0, VBR;    VBR <- 0x00000101

    STC SR, R0;
    MOV #$0, R1;
    MOV R0, @R1;    Expect R0 = 0x0000000F

    STC GBR, R0;
    MOV #$4, R1;
    MOV R0, @R1;    Expect R0 = 0x00000010

    STC VBR, R0;
    MOV #$8, R1;
    MOV R0, @R1;    Expect R0 = 0x00000011

    ; Test LDC.L
    MOV #$70, R3;
    MOV #$1A, R2;
    MOV R2, @R3;    Set memory at 0x70 to be 0x1A

    MOV #$74, R3;
    MOV #$2B, R2;
    MOV R2, @R3;    Set memory at 0x74 to be 0x2B

    MOV #$78, R3;
    MOV #$3C, R2;
    MOV R2, @R3;    Set memory at 0x78 to be 0x3C
    
    MOV #$70, R2;
    LDC.L @R2+, SR;     Read SR from memory, increment R2
    STC SR, R0;
    MOV #$C, R1;
    MOV R0, @R1;        Expect R0 = 0x000000AA

    LDC.L @R2+, GBR;    Read GBR from memory, increment R2
    STC GBR, R0;
    MOV #$10, R1;
    MOV R0, @R1;        Expect R0 = 0x000000BB

    LDC.L @R2+, VBR;    read VBR from memory, increment R2
    STC VBR, R0;
    MOV #$14, R1;
    MOV R0, @R1;        Expect R0 = 0x000000CC

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;

