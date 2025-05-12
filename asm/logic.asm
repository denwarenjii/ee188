ProgramStart:
    ; Test AND
    MOV #$3, R0;
    MOV #$5, R1;
    AND R1, R0;
    MOV #0, R1;
    MOV R0, @R1;

    ; Test AND immediate
    MOV #$E, R0;
    AND #$5, R0;
    MOV #4, R1;
    MOV R0, @R1;

    ; Test OR
    MOV #$3, R0;
    MOV #$5, R1;
    OR R1, R0;
    MOV #8, R1;
    MOV R0, @R1;

    ; Test OR immediate
    MOV #$E, R0;
    OR #$5, R0;
    MOV #12, R1;
    MOV R0, @R1;

    ; Test XOR
    MOV #$3, R0;
    MOV #$5, R1;
    XOR R1, R0;
    MOV #16, R1;
    MOV R0, @R1;

    ; Test XOR immediate
    MOV #$E, R0;
    XOR #$5, R0;
    MOV #20, R1;
    MOV R0, @R1;

    ; Test TST
    MOV #$2, R0;
    MOV #$1, R1;
    TST R1, R0;
    STC SR, R0;
    AND #1, R0;
    MOV #24, R1;
    MOV R0, @R1;

    MOV #$2, R0;
    MOV #$2, R1;
    TST R1, R0;
    STC SR, R0;
    AND #1, R0;
    MOV #28, R1;
    MOV R0, @R1;

    ; Test TST immediate
    MOV #$2, R0;
    TST #1, R0;
    STC SR, R0;
    AND #1, R0;
    MOV #32, R1;
    MOV R0, @R1;

    MOV #$2, R0;
    TST #2, R0;
    STC SR, R0;
    AND #1, R0;
    MOV #36, R1;
    MOV R0, @R1;

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;
