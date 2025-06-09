; mov_wl_at_disp_pc_rn.asm
;
; This file tests the following instructions:
;    MOV.W @(disp, PC), Rn
;    MOV.L @(disp, PC), Rn
;
; It assumes that moving into memory works. The MOV.W and MOV.L instructions
; must be hand assembled.
;
; Revision History:
;   12 May 2025  Chris M.  Initial revision.
;
; TODO:
;  - Use assembly directives to load 32-bit constants instead of MOV #imm, Rn
;

ProgramStart:
  ;MOV #imm, Rn is limited from -128 to 127. immediates are sign-extended.

  MOV   #127, R1   ; PC is 0x00                   (E1 7F)
  MOV   #$7A, R0   ; PC is 0x02                   (E0 7A)
  MOV.B R1, @R0    ; PC is 0x04                   (20 10)

  MOV   #126, R1   ; PC is 0x06                   (E1 7E)
  MOV   #$7B, R0   ; PC is 0x08                   (E0 7B)
  MOV.B R1, @R0    ; PC is 0x0A                   (20 10)

  MOV   #125, R1   ; PC is 0x0C                   (E1 7D)
  MOV   #$7C, R0   ; PC is 0x0E                   (E0 7C)
  MOV.B R1, @R0    ; PC is 0x10                   (20 10)

  MOV   #124, R1   ; PC is 0x12                   (E1 7C)
  MOV   #$7D, R0   ; PC is 0x14                   (E0 7D)
  MOV.B R1, @R0    ; PC is 0x16                   (20 10)
  
  MOV.W Var, R2    ; PC is 0x18
    
  MOV Var, R3;     ; PC is 0x1A

  ; Move R2 and R3 into memory to check for correctness.
   
  MOV   #0, R0     ; PC is 0x1C;  0x00007F7E expected at 0x00000000
  MOV.L R2, @R0    ; PC is 0x1E;

  MOV.L #4, R0  ; 0x7F7E7D7C expected at 0x00000004
  MOV.L R3, @R0 ;

Done:
  ; The test bench interprets a read of 0xFFFFFFFC (-4) 
  ;as system exit.
  MOV   #-4, R0;
  MOV.B R0, @R0;

    align 4
    LTORG
; Var: dc.l $12345678
Var: dc.w $1234
     dc.w $5678
