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

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;
