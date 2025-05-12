ProgramStart:
    ; Set T bit
    SETT;
    STC SR, R0;
    AND #1, R0;
    MOV #1, R0;
    MOV R0, @R1;

    ; Clear T bit
    CLRT;
    STC SR, R0;
    AND #1, R0;
    MOV #4, R1;
    MOV R0, @R1;

    ; Load A into SR
    MOV #$A, R0;
    LDC R0, SR;
    STC SR, R2;
    MOV #8, R1;
    MOV R2, @R1;

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;
