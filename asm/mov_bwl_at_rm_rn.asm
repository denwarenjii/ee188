; mov_bwl_at_rm_rn.asm
;
; This file tests the following instructions:
;    MOV.B @Rm, Rn
;    MOV.W @Rm, Rn
;    MOV.L @Rm, Rn
;
; It assumes that moving into memory works. The memory is organized as follows:
;
;   78: 11 22 33 44
;
; Revision History:
;   13 May 2025  Chris M.  Initial revision.

Start:
  MOV    #$11,  R1   ; 0x11 at address 0x78
  MOV    #$78,  R0
  MOV.B  R1,   @R0

  MOV    #$22,  R1   ; 0x22 at address 0x79
  MOV    #$79,  R0
  MOV.B  R1,   @R0

  MOV    #$33,  R1   ; 0x33 at address 0x7A
  MOV    #$7A,  R0
  MOV.B  R1,   @R0

  MOV    #$44,  R1   ; 0x44 at address 0x7B
  MOV    #$7B,  R0
  MOV.B  R1,   @R0


  MOV   #$78, R0    
  MOV.B @R0, R1     ; Move byte from 0x78 to R1

  MOV.W @R0, R2     ; Move word from 0x78 to R2

  MOV.L @R0, R3     ; Move longword from 0x78 to R2


  MOV   #$00, R0    ; 0x00000011 expected at 0x00
  MOV.L R1, @R0

  MOV   #$04, R0    ; 0x00001122 expected at 0x04
  MOV.L R2, @R0

  MOV   #$08, R0    ; 0x11223344 expected at 0x08
  MOV.L R3, @R0

End:
    ; The test bench interprets a read of 0xFFFFFFFC (-4)
    ;as system exit.
    MOV #-4, R0;
    MOV.B R0, @R0;
    ; Extra NOPs to clear pipeline
    NOP
    NOP
    NOP
