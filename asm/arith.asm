ProgramStart:
    ; Test negation
    MOV #1, R1;
    NEG R1, R1;
    MOV #$0, R0;
    MOV R1, @R0;

    NEG R1, R1;
    MOV #$4, R0;
    MOV R1, @R0;

    ; Test addition
    MOV #1, R1;
    ADD #8, R1;
    MOV #$8, R0;
    MOV R1, @R0;

    ; Test subtraction
    MOV #1, R1;
    MOV #8, R2;
    SUB R2, R1;
    MOV #$C, R0;
    MOV R1, @R0;

    ; Test DT
    MOV #2, R2;
    DT R2;
    MOV #$10, R1;
    MOV R2, @R1;    Expect R1 = 00000001

    STC SR, R0;
    AND #1, R0;
    MOV #$14, R1;
    MOV R0, @R1;    Expect T flag is 0

    DT R2;
    MOV #$18, R1;
    MOV R2, @R1;    Expect R1 = 00000000

    STC SR, R0;
    AND #1, R0;
    MOV #$1C, R1;
    MOV R0, @R1;    Expect T flag is 1

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;
