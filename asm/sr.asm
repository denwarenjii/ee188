ProgramStart:
    ; Set T bit
    SETT;
    STC SR, R1;
    MOV #0, R0;
    MOV R1, @R0;

    ; Clear T bit
    CLRT;
    STC SR, R1;
    MOV #4, R0;
    MOV R1, @R0;

    ; Load A into SR
    MOV #$A, R1;
    LDC R1, SR;
    STC SR, R2;
    MOV #8, R0;
    MOV R2, @R0;

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;
