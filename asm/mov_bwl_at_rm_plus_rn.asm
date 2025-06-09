; mov_bwl_at_rm_plus_rn.asm
;
; This file tests the following instructions:
;    MOV.B @Rm+, Rn
;    MOV.W @Rm+, Rn
;    MOV.L @Rm+, Rn
;
; It assumes that moving into memory works. 
;
; Revision History:
;   16 May 2025  Chris M.  Initial revision.

Start:
    ; We set up the memory as follows:
    ;
    ; 78: 12 34 56 78

    MOV   #$12, R1  ;0x12 at 0x78
    MOV   #$78, R0
    MOV.B R1,   @R0

    MOV   #$34, R1  ;0x34 at 0x79
    MOV   #$79, R0
    MOV.B R1,   @R0

    MOV   #$56, R1  ;0x56 at 0x7A
    MOV   #$7A, R0
    MOV.B R1,   @R0

    MOV   #$78, R1  ;0x78 at 0x7B
    MOV   #$7B, R0
    MOV.B R1,   @R0

    ;Test instructions.

    MOV   #$78, R0  ; Start reading at 0x78
    MOV.B @R0+, R1  ; Read 0x12 into R1. 
                    ; Address in R0 should be 0x79 after this.

    MOV   #$00, R2  ; Move 0x00000012 (R1) @ 0x00000000. 
    MOV.L R1,  @R2  ; 
                    
    MOV   #$04, R2  ; Move 0x00000079 (R0) @ 0x00000004.
    MOV.L R0,  @R2


    MOV  #$78, R0   ; Start reading at 0x78
    MOV.W @R0+, R1  ; Read 0x1234 into R1.
                    ; Address in R0 should be 0x7A after this

    MOV   #$08, R2  ; Move 0x00001234 @ 0x00000008. 
    MOV.L R1,  @R2  ;
                    
    MOV   #$0C, R2  ; Move 0x0000007A (R0) @ 0x0000000C.
    MOV.L R0,  @R2

    MOV  #$78, R0   ; Start reading at 0x78
    MOV.L @R0+, R1  ; Read 0x12345678 into R1.
                    ; Address in R0 should be 0x7C after this

    MOV   #$10, R2  ; Move 0x12345678 @ 0x00000010. 
    MOV.L R1,  @R2  ;
                    ;
                    
    MOV   #$14, R2  ; Move 0x0000007C (R0) @ 0x00000014.
    MOV.L R0,  @R2


    ; Expected memory
    ; 00000000: 00000012 
    ; 00000004: 00000079 
    ; 00000008: 00001234
    ; 0000000C: 0000007A
    ; 00000010: 12345678
    ; 00000014: 0000007C

End:
    ; The test bench interprets a read of 0xFFFFFFFC (-4)
    ;as system exit.
    MOV #-4, R0;
    MOV.B R0, @R0;
    ; Extra NOPs to clear pipeline
    NOP
    NOP
    NOP
