; mov_bwl_rm_at_minus_rn.asm
;
; This file tests the following instructions:
;    MOV.B Rm, @-Rn
;    MOV.W Rm, @-Rn
;    MOV.L Rm, @-Rn
;
; It assumes that moving into memory works. 
;
; Revision History:
;   14 May 2025  Chris M.  Initial revision.

Start:
  MOV    #$11,  R1   ; 0x11 at address 0x78
  MOV    #$78,  R0
  MOV.B  R1,   @R0

  MOV    #$22,  R1   ; 0x22 at address 0x79
  MOV    #$79,  R0
  MOV.B  R1,   @R0

  MOV    #$33,  R1   ; 0x22 at address 0x7A
  MOV    #$7A,  R0
  MOV.B  R1,   @R0

  MOV    #$44,  R1   ; 0x3 at address 0x7B
  MOV    #$7B,  R0
  MOV.B  R1,   @R0

  MOV   #$78, R0  ; Move 44332211 into R1
  MOV.L @R0,  R1


  MOV   #$01,  R0    ; Write byte from R1 (0x11) into address 0x00. Note that
                     ; R0 is pre-decremented by 1.
  MOV.B  R1, @-R0

  MOV  #$01, R0      ; Write 0x00 at 0x01.
  MOV  #$00, R2
  MOV.B R2, @R0

  MOV   #$04,  R0    ; Write word from R1 (0x2211) into address 0x02. Note
                     ; that R0 is pre-decremented by 2.
  MOV.W  R0, @R0    


  MOV   #$08,  R0    ; Write word from R1 (0x44332211) into address 0x04. Note
                     ; that R0 is pre-decremented by 4.
  MOV.W  R0, @R0    

  ; 00000000  11002211
  ; 00000004  44332211

End:
  ; The test bench interprets a read of 0xFFFFFFFC (-4)
  ;as system exit.
  MOV #-4, R0;
  MOV.B R0, @R0;
