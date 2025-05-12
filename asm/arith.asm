ProgramStart:
    ; Test negation
    MOV #1, R1;
    NEG R1, R1;
    MOV #0, R0;
    MOV R1, @R0;

    NEG R1, R1;
    MOV #4, R0;
    MOV R1, @R0;

    ; Test addition
    MOV #1, R1;
    ADD #8, R1;
    MOV #8, R0;
    MOV R1, @R0;

    ; Test subtraction
    MOV #1, R1;
    MOV #8, R2;
    SUB R2, R1;
    MOV #12, R0;
    MOV R1, @R0;

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;
