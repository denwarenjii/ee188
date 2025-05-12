; mov_wl_at_disp_pc_rn.asm
;
; This file tests the following instructions:
;    MOV.W @(disp, PC), Rn
;    MOV.L @(disp, PC), Rn
;
; It assumes that moving into memory works.
;
; Revision History:
;   12 May 2025  Chris M.  Initial revision.
;
; TODO:
;  - Use assembly directives to load 32-bit constants instead of MOV #imm, Rn
;

ProgramStart:
  ;MOV #imm, Rn is limited from $00 to $FF

  ;We will move 
  ;   0xAA at address 0x7A, 
  ;   0xBB at address 0x7B, 
  ;   0xCC at address 0x7C,
  ;   0xDD at address 0x7D

  MOV   #$80, R1  ; PC is 0x02

  MOV   #$7A, R0  ; PC is 0x04
  MOV.B R1, @R0   ; PC is 0x06

  MOV   #$7B, R0  ; PC is 0x08
  MOV.B R1, @R0   ; PC is 0x0A

  MOV   #$7C, R0  ; PC is 0x0C
  MOV.B R1, @R0   ; PC is 0x0E

  MOV   #$7D, R0  ; PC is 0x10
  MOV.B R1, @R0   ; PC is 0x18
  

  ; disp is zero extended and doubled for MOV.W
  ; 0x20 + d*2 = 0x7A -> d = 0x2D
  MOV.W ($2D, PC), R2 ; PC is 0x20

  ; disp is zero extended and quadrupled for MOV.L
  ; 0x22 + d*4 = 0x7A -> d = 0x16
  MOV.L ($16, PC), R3 ; PC is 0x22

Done:
  ; The test bench interprets a read of 0xFFFFFFFC (-4) as system exit.
  MOV #-4, R0;
  MOV.B R0, @R0
