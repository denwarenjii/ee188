ProgramStart:
    ; Write 0A0B0C0D to 0x00000000 using byte mode addressing
    ; Note, need to write out-of-order because CPU assumes big-endian
    ; order of bytes in memory.
    MOV #1, R0;
    MOV #$D, R1;
    MOV.B R1, @R0;

    MOV #0, R0;
    MOV #$C, R1;
    MOV.B R1, @R0;

    MOV #3, R0;
    MOV #$B, R1;
    MOV.B R1, @R0;

    MOV #2, R0;
    MOV #$A, R1;
    MOV.B R1, @R0;

    ; Write 00120034 to 0x00000004 using word mode
    MOV #$F, R1;
    ADD #$3, R1;
    MOV #6, R0;
    MOV.W R1, @R0;

    ADD #$F, R1;
    ADD #$F, R1;
    ADD #$4, R1;
    MOV #4, R0;
    MOV.W R1, @R0;

    ; Write 0000DEAD to 0x00000008 using longword mode
    ;MOV #8, R0;
    ;MOV #$DEAD, R1;
    ;MOV.L R1, @R0;

    ; Quit test program
    MOV #-4, R0;
    MOV.B R0, @R0;

