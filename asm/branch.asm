;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                                                                              
; branch.asm                                                                   
;                                                                              
; This file tests the following instructions:                                  
;
;   BF    <label>                                                             
;   BF/S  <label>
;   BT    <label>
;   BT/S  <label>
;
; Branches are tested by writing to memory based on address we jump to. Note 
; that if jumps do not work at all, we will encounter the "exit" signal and
; the expected memory will be incomplete. The execution of the instructions in
; the delay slot is tested by seeing if the contents of a register are altered
; or not when we arrive at the target address.
;
; Revision History:                                                            
;   26 May 2025  Chris M.  Initial revision.                                   
;                                                                             
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Start:

    ; Test BF. Write at 0x00

    CLRT              ; Clear the T flag.
    BT    TRGET_T0    ; This branch should not be taken.
    BF    TRGET_F0    ; This branch is taken. The PC should be changed to TRGET_F0
    NOP
    NOP

TRGET_T0:

    MOV   TrueVar,  R1  ; Write TrueVar @ 0x00 to detect wrong branch.
    MOV   #$00,     R0
    MOV   R1,      @R0

TRGET_T1:

    MOV   TrueVar,  R1  ; Write TrueVar @ 0x08 to detect wrong branch.
    MOV   #$08,     R0
    MOV   R1,      @R0

End_T0:

    ; The test bench interprets a read of 0xFFFFFFFC (-4)
    ; as system exit.
    MOV     #-4,  R0
    MOV.B   R0,  @R0


TRGET_F0:

    ; Test BF/S. Write at 0x04 and 0x08

    MOV   FalseVar,   R1    ; Write FalseVar at 0x00
    MOV   #$00,       R0    
    MOV   R1,        @R0

    MOV   #6,   R2
    MOV   #7,   R3

    CLRT            ; Clear the T flag.
    
    BF/S  TRGET_F1  ; Branch to TRGET_F1 and exeucte the delay slot.
    ADD   R2, R3    ; 6 + 7 = 13 should be in R3 when we get to TRGET_F1.

    BT/S  TRGET_T1  ; This branch should not be taken.
    NOP

    NOP

TRGET_F1:

    MOV   #$04,  R0   ; If the delay slot was executed, then 0x0D is in R3.
    MOV   R3,   @R0  ; Write 0x0D to 0x04

    MOV   FalseVar,   R1  ; Write FalseVar at 0x08
    MOV   #$08,       R0
    MOV   R1,        @R0

    ; -- Test BT. Write at 0x0C ------------------------------------------------

    SETT              ; Set the T flag
    BF    TRGET_F2    ; This branch should not be taken.
    BT    TRGET_T2    ; This branch is taken.
    NOP
    NOP

TRGET_F2:
     
    MOV   FalseVar,   R1  ; Write FalseVar at 0x0C to detect wrong branch.
    MOV   #$0C,       R0
    MOV   R1,        @R0

TRGET_T2:

    MOV   TrueVar,  R1    ; Write TrueVar at 0x0C
    MOV   #$0C,     R0
    MOV   R1,      @R0

    CLRT                  ; Clear T flag

    ; -- Test BT/S. Write at 0x10 ----------------------------------------------

    MOV   #$09,     R3    ; Set up register for addition
    MOV   #$0C,     R4

    SETT                  ; Set T flag.

    BF/S  TRGET_F3        ; Should not be taken
    NOP

    BT/S  TRGET_T3        ; Should be taken
    ADD   R3,       R4    ; 0x15 should be in R4 if the branch slot is executed.
    NOP

TRGET_F3:

    MOV   FalseVar,   R1  ; Write FalseVar at 0x10 to detect wrong branch.
    MOV   #$10,       R0
    MOV   R1,        @R0

    ; The test bench interprets a read of 0xFFFFFFFC (-4) 
    ; as system exit.
    MOV     #-4,  R0;   ; PC = 0x1A
    MOV.B   R0,  @R0;   ; PC = 0x1C

TRGET_T3:

    MOV   #$10,       R0  ; If the delay slot is executed, then 0x15 is in R4
    MOV   R4,        @R0  ; Write 0x15 @ 0x10

    MOV   TrueVar,    R1  ; Write TrueVar at 0x14 if `BT/S` jumps to the correct address.
    MOV   #$14,       R0
    MOV   R1,        @R0



    ; -- Test BRA. Write to 0x18 -----------------------------------------------

    MOV   #$01,   R2
    MOV   #$02,   R3

    BRA TRGET_BRA;
    ADD   R2,     R3  ; Branch slot for BRA


    ; If BRA doesn't work, the test exits.
    ;
    ; The test bench interprets a read of 0xFFFFFFFC (-4)
    ; as system exit.
    MOV     #-4,  R0;   ; PC = 0x1A
    MOV.B   R0,  @R0;   ; PC = 0x1C


TRGET_BRA:

    MOV   #$18,     R0  ; Write result of branch slot (0x03) at 0x18
    MOV   R3,      @R0

    MOV   TrueVar,  R1  ; Write TrueVar at 0x1C
    MOV   #$1C,     R0
    MOV   R1,      @R0


    ; -- Test BRAF -------------------------------------------------------------


    MOV   #7,   R2
    MOV   #9,   R3

    MOV   #10,   R4  ; The target of BRAF is 4 instructions away from the BRAF.
                     ; So we add 8 + 2 because our PC points to current
                     ; instruction instead of the next as expected by the spec.


    BRAF  R4        ; Branch to TRGET_BRAF
    ADD   R2,   R3

    NOP

    ; If the branch is not taken, the system exits
    ; The test bench interprets a read of 0xFFFFFFFC (-4)
    ; as system exit.
    MOV     #-4,  R0
    MOV.B   R0,  @R0


TRGET_BRAF:

    MOV   #$20,     R0  ; Write the result of the branch slot (0x10) at 0x20
    MOV   R3,      @R0

    MOV   #$24,      R0 ; Write TrueVar at 0x24 to signal that BRAF made it to
    MOV   TrueVar,   R1 ; the target.
    MOV   R1,       @R0

    ; -- Test BSR --------------------------------------------------------------

    MOV  #10,   R3  ; Values to test execution of branch slot.
    MOV  #12,   R4


    BSR  TRGET_BSR
    ADD  R3,    R4

    NOP                 ; RTS returns here.

    MOV   #$30,   R0    ; Write the result of the RTS branch slot (0x11) at 0x30
    MOV   R1,    @R0


    ; -- Test BSRF -------------------------------------------------------------

    MOV  #5,  R3    ; Values to test execution of branch slot.
    MOV  #3,  R4

    MOV   #38,  R1     ; TRGET_BSRF is 18 instructions (36 bytes) away from BSRF.
                       ; We add an extra 2 bytes because our PC points to the 
                       ; current instruction instead of the next instruction as
                       ; expected by the spec.

    BSRF  R1
    ADD   R3,   R4     ; Branch slot for BSRF

    NOP                ; RTS returns here

    MOV   #$3C,   R0   ; Write the result of the RTS branch slot, 0x22 at 0x38
    MOV   R1,    @R0

    ; -- Test JMP  -------------------------------------------------------------

    ; MOV.W   TRGET_JMP, R0

    ; MOVA  TRGET_JMP, R0  ; Load address of target into R0

    ; MOV #$36, R0          ; 17 * 2 + 2 = 36 

    MOV.L  TRGET_JMP_DATA, R0

    MOV  #2,  R3        ; Values to test execution of branch slot.
    MOV  #5,  R4

    JMP  @R0
    ADD  R3,  R4        ; Branch slot of JMP


    MOV     #-4,  R0    ; If the JMP is not taken, the system exits.
    MOV.B   R0,  @R0


TRGET_BSR:
    MOV   #$28,   R0    ; Write result of the BSR branch slot (0x16) at 0x28
    MOV   R4,    @R0


    MOV   #$2C,     R0  ; Write TrueVar at 0x2C to signal that BSR jumped to the
    MOV   TrueVar,  R1  ; correct target.
    MOV   R1,      @R0

    RTS                 ; Return to the instruction after the branch slot of BSR
    MOV   #$11,     R1  ; Branch slot of RTS

TRGET_BSRF:


    MOV   #$34,     R0    ; Write the result of the BSRF branch slot, 0x08 at 0x34
    MOV   R4,      @R0

    MOV   #$38,     R0    ; Write TrueVar at 0x3C to signal that BSRF jumped
    MOV   TrueVar,  R1    ; to the correct target.
    MOV   R1,      @R0

    RTS                   ; Return to the instruction after the branch slot of BSRF

    MOV  #$22, R1         ; Branch slot of RTS

TRGET_JMP:

    MOV  #$40,      R0    ; Write the result of the JMP branch slot, 0x07 at 0x40
    MOV  R4,       @R0

    MOV  #$44,      R0    ; Write TrueVar at 0x40 to signal that JMP jumped to
    MOV  TrueVar,   R1    ; the correct target.
    MOV  R1,       @R0


End:

    ; The test bench interprets a read of 0xFFFFFFFC (-4)
    ; as system exit.
    MOV     #-4,  R0
    MOV.B   R0,  @R0

   
    ; Expected memory:
    ; 00000000 BBBBBBBB  ; FalseVar is written to 0x00 if `BF` jumps to the correct target.
    ; 00000004 0000000D  ; 0x0D (13) is written to 0x04 if the delay slot of a `BF/S` is executed.
    ; 00000008 BBBBBBBB  ; FalseVar is written to 0x08 if `BF/S` jumps to the correct target.
    ; 0000000C AAAAAAAA  ; TrueVar is writen to 0x0C if `BT` jumps to the correct target.
    ; 00000010 00000015  ; 0x15 is written to 0x10 if the delay slot of a `BT/S` is exeucted.
    ; 00000014 AAAAAAAA  ; TrueVar is written to 0x14 if `BT/S` jumps to the correct target.
    ; 00000018 00000003  ; 0x03 is written to 0x18 if the branch slot of the `BRA` is executed.
    ; 0000001C AAAAAAAA  ; TrueVar is written to 0x1C of `BRA` jumps to the correct target.
    ; 00000020 00000010  ; 0x10 is written to 0x20 if the branch slot of the `BRAF` is executed.
    ; 00000024 AAAAAAAA  ; TrueVar is written to 0x24 if `BRAF` jumps to the correct target.
    ; 00000028 00000016  ; 0x16 is written to 0x28 if the branch slot of `BSR` is executed.
    ; 0000002C AAAAAAAA  ; TrueVar is written to 0x2C if `BSR` jumps to the correct instruction.
    ; 00000030 00000011  ; 0x11 is written to 0x30 if the branch slot of `RTS` is executed.
    ; 00000034 00000008  ; 0x08 is written to 0x34 if the branch slot of `BSRF` is executed.
    ; 00000038 AAAAAAAA  ; TrueVar is written to 0x38 if `BSRF` jumps to the correct instruction.
    ; 0000003C 00000022  ; 0x22 is written to 0x3C if the branch slot of `RTS` is executed.
    ; 00000040 00000007  ; 0x07 is written to 0x40 if the branch slot of `JMP` is executed.
    ; 00000044 AAAAAAAA  ; TrueVar is written to 0x44 if `JMP` jumps to the correct target.
    ; 00000048 

    align 4
    LTORG

TrueVar:         dc.l $AAAAAAAA
FalseVar:        dc.l $BBBBBBBB
TRGET_JMP_DATA:  dc.l TRGET_JMP
