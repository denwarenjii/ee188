LTORG

ProgramStart:
    MOV #$F, R0;
    LDC R0, SR;     SR  <- 0x0000000F

    ADD #1, R0;
    LDC R0, GBR;    GBR <- 0x00000100

    ADD #1, R0;
    LDC R0, VBR;    VBR <- 0x00000101

    STC SR, R0;
    MOV #0, R1;
    MOV R0, @R1;    Expect R0 = 0x0000000F

    STC GBR, R0;
    MOV #4, R1;
    MOV R0, @R1;    Expect R0 = 0x00000010

    STC VBR, R0;
    MOV #8, R1;
    MOV R0, @R1;    Expect R0 = 0x00000011

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;

