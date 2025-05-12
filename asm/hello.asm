ProgramStart:
    MOV #15, R0;
    MOV #0, R1;
    MOV R0, @R1;
    ; Quit test program
    MOV #-1, R0;
    MOV.B R0, @R0;
