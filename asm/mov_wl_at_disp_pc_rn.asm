ProgramStart:

  MOV   #$AA, R1; PC is 0x00
  MOV   #$7A, R0; PC is 0x02
  MOV.B R1, @R0; PC is 0x04

  MOV   #$BB, R1; PC is 0x06
  MOV   #$7B, R0; PC is 0x08
  MOV.B R1, @R0; PC is 0x0A

  MOV   #$CC, R1; PC is 0x0C
  MOV   #$7C, R0; PC is 0x0E
  MOV.B R1, @R0; PC is 0x10

  MOV   #$DD, R1; PC is 0x12
  MOV   #$7D, R0; PC is 0x14
  MOV.B R1, @R0; PC is 0x16

  
  ; The test bench interprets a read of 0xFFFFFFFC (-4) as system exit.
  MOV #-4, R0;
  MOV.B R0, @R0
