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
  ;MOV #imm, Rn is limited from $00 to $FF. immediates are sign-extended.

  ;We will move 
  ;   0xAA at address 0x007A, 
  ;   0xBB at address 0x007B, 
  ;   0xCC at address 0x007C,
  ;   0xDD at address 0x007D

  MOV   #$AA, R1   ; PC is 0x00
  MOV   #$7A, R0   ; PC is 0x02
  MOV.B R1, @R0    ; PC is 0x04

  MOV   #$BB, R1   ; PC is 0x06
  MOV   #$7B, R0   ; PC is 0x08
  MOV.B R1, @R0    ; PC is 0x0A

  MOV   #$CC, R1   ; PC is 0x0C
  MOV   #$7C, R0   ; PC is 0x0E
  MOV.B R1, @R0    ; PC is 0x10

  MOV   #$DD, R1   ; PC is 0x12
  MOV   #$7D, R0   ; PC is 0x14
  MOV.B R1, @R0    ; PC is 0x16
  
  ; NOP              ; PC is 0x18. Needed for alignment.

  ; ; disp is zero extended and doubled for MOV.W
  ; ; 0x1A + d*2 = 0x7A -> d = 0x30
  ; MOV.W @($30, PC), R2 ; PC is 0x1A

  ; NOP              ; PC is 0x1C

  ; ; disp is zero extended and quadrupled for MOV.L
  ; ; 0x1E + d*4 = 0x7A -> d = 0x17
  ; MOV.L @($17, PC), R3 ; PC is 0x1E

  ; ; Move R2 and R3 into memory to check for correctness.
  ; MOV #$00, R0   ; 0xFFFFAABB expected at 0x00000000
  ; MOV.W R2, @R0

  ; MOV #$04, R0   ; 0xAABBCCDD expected at 0x00000004
  ; MOV.W R3, @R0

  
; Done:
;   ; The test bench interprets a read of 0xFFFFFFFC (-4) as system exit.
;   MOV #-4, R0;
;   MOV.B R0, @R0
