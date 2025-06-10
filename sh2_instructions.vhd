----------------------------------------------------------------------------------------------------
-- 
--  SH-2 Instructions
--
--  This package defines the bits defining each type of instruction in the SH-2
--  instruction set. The encodings are placed here for the purpose of
--  organization.
--
--  Revision History
--      07 Jun 25   Zack Huang      Copied over from control unit
--
----------------------------------------------------------------------------------------------------

library ieee;
library std;

use std.textio.all;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.MemoryInterfaceConstants.all;

package SH2InstructionEncodings is

  subtype Instruction is std_logic_vector(15 downto 0);

  -- Instruction encodings.

  -- Data Transfer Instructions:
  constant  MOV_IMM_RN            :  Instruction := "1110------------";  -- MOV #imm, Rn

  constant  MOV_AT_DISP_PC_RN     :  Instruction := "1-01------------";  -- MOV.X @(disp, PC), Rn (for bit decoding.)
  constant  MOV_W_AT_DISP_PC_RN   :  Instruction := "1001------------";  -- MOV.W @(disp, PC), Rn
  constant  MOV_L_AT_DISP_PC_RN   :  Instruction := "1101------------";  -- MOV.L @(disp, PC), Rn

  constant  MOV_RM_RN             :  Instruction := "0110--------0011";  -- MOV Rm, Rn

  constant  MOV_RM_AT_RN          :  Instruction := "0010--------00--";  -- MOV.X Rm, @Rn (for bit decoding).
  constant  MOV_B_RM_AT_RN        :  Instruction := "0010--------0000";  -- MOV.B Rm, @Rn
  constant  MOV_W_RM_AT_RN        :  Instruction := "0010--------0001";  -- MOV.W Rm, @Rn
  constant  MOV_L_RM_AT_RN        :  Instruction := "0010--------0010";  -- MOV.L Rm, @Rn

  constant  MOV_AT_RM_RN          :  Instruction := "0110--------00--";  -- MOV.X @Rm, Rn (for bit decoding).
  constant  MOV_B_AT_RM_RN        :  Instruction := "0110--------0000";  -- MOV.B @Rm, Rn
  constant  MOV_W_AT_RM_RN        :  Instruction := "0110--------0001";  -- MOV.W @Rm, Rn
  constant  MOV_L_AT_RM_RN        :  Instruction := "0110--------0010";  -- MOV.L @Rm, Rn

  constant  MOV_RM_AT_MINUS_RN    :  Instruction := "0010--------01--";  -- MOV.X Rm, @-Rn (for bit decoding).
  constant  MOV_B_RM_AT_MINUS_RN  :  Instruction := "0010--------0100";  -- MOV.B Rm, @-Rn
  constant  MOV_W_RM_AT_MINUS_RN  :  Instruction := "0010--------0101";  -- MOV.W Rm, @-Rn
  constant  MOV_L_RM_AT_MINUS_RN  :  Instruction := "0010--------0110";  -- MOV.L Rm, @-Rn

  constant  MOV_AT_RM_PLUS_RN     :  Instruction := "0110--------01--";  -- MOV.X @Rm+, Rn (for bit decoding)
  constant  MOV_B_AT_RM_PLUS_RN   :  Instruction := "0110--------0100";  -- MOV.B @Rm+, Rn
  constant  MOV_W_AT_RM_PLUS_RN   :  Instruction := "0110--------0101";  -- MOV.W @Rm+, Rn
  constant  MOV_L_AT_RM_PLUS_RN   :  Instruction := "0110--------0110";  -- MOV.W @Rm+, Rn

  constant  MOV_R0_AT_DISP_RN     :  Instruction := "1000000---------";  -- MOV.{B,W} RO, @(disp,Rn)
  constant  MOV_B_R0_AT_DISP_RN   :  Instruction := "10000000--------";  -- MOV.B RO, @(disp,Rn)
  constant  MOV_W_R0_AT_DISP_RN   :  Instruction := "10000001--------";  -- MOV.W RO, @(disp,Rn)

  constant  MOV_L_RM_AT_DISP_RN   :  Instruction := "0001------------";  -- MOV.L Rm, @(disp, Rn)

  constant  MOV_AT_DISP_RM_R0     :  Instruction := "1000010---------";  -- MOV.{B,W} @(disp, Rm), R0
  constant  MOV_B_AT_DISP_RM_R0   :  Instruction := "10000100--------";  -- MOV.B @(disp, Rm), R0
  constant  MOV_W_AT_DISP_RM_R0   :  Instruction := "10000101--------";  -- MOV.W @(disp, Rm), R0

  constant  MOV_L_AT_DISP_RM_RN   :  Instruction := "0101------------";  -- MOV.L @(disp, Rm), Rn

  constant  MOV_RM_AT_R0_RN       :  Instruction := "0000--------01--";  -- MOV.X Rm, @(R0, Rn)
  constant  MOV_B_RM_AT_R0_RN     :  Instruction := "0000--------0100";  -- MOV.B Rm, @(R0, Rn)
  constant  MOV_W_RM_AT_R0_RN     :  Instruction := "0000--------0101";  -- MOV.W Rm, @(R0, Rn)
  constant  MOV_L_RM_AT_R0_RN     :  Instruction := "0000--------0110";  -- MOV.L Rm, @(R0, Rn)

  constant  MOV_AT_R0_RM_RN       :  Instruction := "0000--------11--";  -- MOV.X @(R0, Rm), Rn
  constant  MOV_B_AT_R0_RM_RN     :  Instruction := "0000--------1100";  -- MOV.B @(R0, Rm), Rn
  constant  MOV_W_AT_R0_RM_RN     :  Instruction := "0000--------1101";  -- MOV.W @(R0, Rm), Rn
  constant  MOV_L_AT_R0_RM_RN     :  Instruction := "0000--------1110";  -- MOV.L @(R0, Rm), Rn

  constant  MOV_R0_AT_DISP_GBR    :  Instruction := "110000----------";  -- MOV.X R0, @(disp, GBR)
  constant  MOV_B_R0_AT_DISP_GBR  :  Instruction := "11000000--------";  -- MOV.B R0, @(disp, GBR)
  constant  MOV_W_R0_AT_DISP_GBR  :  Instruction := "11000001--------";  -- MOV.W R0, @(disp, GBR)
  constant  MOV_L_R0_AT_DISP_GBR  :  Instruction := "11000010--------";  -- MOV.L R0, @(disp, GBR)

  constant  MOV_AT_DISP_GBR_R0    :  Instruction := "110001----------";  -- MOV.X @(disp, GBR), R0
  constant  MOV_B_AT_DISP_GBR_R0  :  Instruction := "11000100--------";  -- MOV.B @(disp, GBR), R0
  constant  MOV_W_AT_DISP_GBR_R0  :  Instruction := "11000101--------";  -- MOV.W @(disp, GBR), R0
  constant  MOV_L_AT_DISP_GBR_R0  :  Instruction := "11000110--------";  -- MOV.L @(disp, GBR), R0

  constant  MOVA_AT_DISP_PC_R0    :  Instruction := "11000111--------";  -- MOVA @(disp, PC), R0

  constant  MOVT_RN               :  Instruction := "0000----00101001";  -- MOVT Rn

  constant  SWAP_RM_RN            :  Instruction := "0110--------100-";  -- SWAP.{B,W} Rm, Rn
  constant  SWAP_B_RM_RN          :  Instruction := "0110--------1000";  -- SWAP.B Rm, Rn
  constant  SWAP_W_RM_RN          :  Instruction := "0110--------1001";  -- SWAP.W Rm, Rn

  constant  XTRCT_RM_RN           :  Instruction := "0010--------1101";  -- XTRCT Rm, Rn

  -- Arithmetic Instructions:
  constant ADD_RM_RN     : Instruction := "0011--------11--";
  constant ADD_IMM_RN    : Instruction := "0111------------";
  constant SUB_RM_RN     : Instruction := "0011--------10--";
  constant NEG_RM_RN     : Instruction := "0110--------101-";
  constant DT_RN         : Instruction := "0100----00010000";
  constant EXT_RM_RN     : Instruction := "0110--------11--";

  constant CMP_EQ_IMM    : Instruction := "10001000--------";   -- CMP/EQ #imm, R0
  constant CMP_RM_RN     : Instruction := "0011--------0---";   -- CMP/{EQ,HS,GE,HI,GT} Rm, Rn
  constant CMP_RN        : Instruction := "0100----00010-01";   -- CMP/{PL/PZ}
  constant CMP_STR_RM_RN : Instruction := "0010--------1100";   -- CMP/STR

  -- Logical Operations:
  constant LOGIC_RM_RN   : Instruction := "0010--------10--";  -- AND, TST, OR, XOR
  constant LOGIC_IMM_R0  : Instruction := "110010----------";  -- AND, TST, OR, XOR
  constant NOT_RM_RN     : Instruction := "0110--------0111";  -- NOT

  -- Shift Instruction:
  constant SHIFT_RN      : Instruction := "0100----00-00-0-";  -- shift/rotate instructions
  constant BSHIFT_RN     : Instruction := "0100----00--100-";  -- barrel shifter instructions

  -- Branch Instructions:
  constant BF     :   Instruction := "10001011--------"; -- BF      <label>
  constant BF_S   :   Instruction := "10001111--------"; -- BF/S    <label>
  constant BT     :   Instruction := "10001001--------"; -- BT      <label>
  constant BT_S   :   Instruction := "10001101--------"; -- BT/S    <label>
  constant BRA    :   Instruction := "1010------------"; -- BRA     <label>
  constant BRAF   :   Instruction := "0000----00100011"; -- BRAF    Rm
  constant BSR    :   Instruction := "1011------------"; -- BSR     <label>
  constant BSRF   :   Instruction := "0000----00000011"; -- BSRF    Rm
  constant JMP    :   Instruction := "0100----00101011"; -- JMP     @Rm
  constant JSR    :   Instruction := "0100----00001011"; -- JSR     @Rm
  constant RTS    :   Instruction := "0000000000001011"; -- RTS


  -- System Control:
  constant NOP      : Instruction := "0000000000001001";
  constant CLRT     : Instruction := "0000000000001000";
  constant CLRMAC   : Instruction := "0000000000101000";
  constant SETT     : Instruction := "0000000000011000";

  constant STC_SYS_RN             : Instruction := "0000----00--0010";  -- STC {SR, GBR, VBR}, Rn
  constant STC_SR_RN              : Instruction := "0000----00000010";  -- STC SR,  Rn
  constant STC_GBR_RN             : Instruction := "0000----00010010";  -- STC GBR, Rn
  constant STC_VBR_RN             : Instruction := "0000----00100010";  -- STC VBR, Rn

  constant STC_L_SYS_RN           : Instruction := "0100----00--0011";  -- STC.L {SR, GBR, VBR}, @-Rn
  constant STC_L_SR_AT_MINUS_RN   : Instruction := "0100----00000011";  -- STC.L SR, @-Rn
  constant STC_L_GBR_AT_MINUS_RN  : Instruction := "0100----00010011";  -- STC.L GBR, @-Rn
  constant STC_L_VBR_AT_MINUS_RN  : Instruction := "0100----00100011";  -- STC.L VBR, @-Rn
  
  constant LDC_RM_SYS             : Instruction := "0100----00--1110";  -- LDC Rm, {SR, GBR, VBR}
  constant LDC_RM_SR              : Instruction := "0100----00001110";  -- LDC Rm, SR
  constant LDC_RM_GBR             : Instruction := "0100----00011110";  -- LDC Rm, GBR
  constant LDC_RM_VBR             : Instruction := "0100----00101110";  -- LDC Rm, VBR

  constant LDC_L_RM_SYS           : Instruction := "0100----00--0111";  -- LDC.L @Rm+, {SR, GBR, VBR}
  constant LDC_L_AT_RM_PLUS_SR    : Instruction := "0100----00000111";  -- LDC.L @Rm+, SR
  constant LDC_L_AT_RM_PLUS_GBR   : Instruction := "0100----00010111";  -- LDC.L @Rm+, GBR
  constant LDC_L_AT_RM_PLUS_VBR   : Instruction := "0100----00100111";  -- LDC.L @Rm+, VBR

  constant LDS_RM_SYS             : Instruction := "0100----00--1010";  -- LDS Rm, {MACH, MACL, PR}
  constant LDS_RM_MACH            : Instruction := "0100----00001010";  -- LDS Rm, MACH
  constant LDS_RM_MACL            : Instruction := "0100----00011010";  -- LDS Rm, MACL
  constant LDS_RM_PR              : Instruction := "0100----00101010";  -- LDS Rm, PR

  constant LDS_L_RM_SYS           : Instruction := "0100----00--0110";  -- LDS.L @Rm+, {MACH, MACL, PR}
  constant LDS_L_AT_RM_PLUS_MACH  : Instruction := "0100----00000110";  -- LDS.L @Rm+, MACH
  constant LDS_L_AT_RM_PLUS_MACL  : Instruction := "0100----00010110";  -- LDS.L @Rm+, MACL
  constant LDS_L_AT_RM_PLUS_PR    : Instruction := "0100----00100110";  -- LDS.L @Rm+, PR

  constant STS_SYS_RN             : Instruction := "0000----00--1010";  -- STS Rm, {MACH, MACL, PR}
  constant STS_MACH_RN            : Instruction := "0000----00001010";  -- STS Rm, MACH
  constant STS_MACL_RN            : Instruction := "0000----00011010";  -- STS Rm, MACL
  constant STS_PR_RN              : Instruction := "0000----00101010";  -- STS Rm, PR

  constant STS_L_SYS_RN           : Instruction := "0100----00--0010";  -- STS.L @Rm+, {MACH, MACL, PR}
  constant STS_L_AT_RM_PLUS_MACH  : Instruction := "0100----00000010";  -- STS.L @Rm+, MACH
  constant STS_L_AT_RM_PLUS_MACL  : Instruction := "0100----00010010";  -- STS.L @Rm+, MACL
  constant STS_L_AT_RM_PLUS_PR    : Instruction := "0100----00100010";  -- STS.L @Rm+, PR


end package SH2InstructionEncodings;

