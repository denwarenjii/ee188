------------- --------------------------------------------------------------
--
--  Control Unit
--
--
--  Revision History:
--     06 May 25  Zack Huang        Initial revision
--     07 May 25  Chris Miranda     Initial implentation of MOV and branch 
--                                  instruction decoding.
--     10 May 25  Zack Huang        Implementing ALU instruction
--     14 May 25  Chris M.          Formatting.
--     16 May 25  Zack Huang        Documentation, renaming signals
--
-- Notes:
--  - When reading/writing to registers, RegB is always Rm and RegA is always Rn
--  - When reading/writing to addresses (in registers), RegA2 is always @(Rm) and
--    RegA2 is always @(Rn).
--
-- TODO:
--  - Bit decode all movs.
--  - Remove redundant assignment of default signals.
--  - Better names for:
--      Instruction_EnableIn, EnableIn
--
--  - Generate DMAU signals with vectors.
--  - Document register output conventions.
--  - Document bit decoding.
----------------------------------------------------------------------------

library ieee;
library std;

use std.textio.all;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.MemoryInterfaceConstants.all;
use work.Logging.all;

package SH2InstructionEncodings is

  subtype Instruction is std_logic_vector(15 downto 0);

  -- Data Transfer Instruction:
  -- TODO: Bit decode when possible.
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

  constant  MOV_B_AT_RM_PLUS_RN   :  Instruction := "0110--------0100";  -- MOV.B @Rm+, Rn
  constant  MOV_W_AT_RM_PLUS_RN   :  Instruction := "0110--------0101";  -- MOV.W @Rm+, Rn
  constant  MOV_L_AT_RM_PLUS_RN   :  Instruction := "0110--------0110";  -- MOV.W @Rm+, Rn

  constant  MOV_B_R0_AT_DISP_RN   :  Instruction := "10000000--------";  -- MOV.B RO, @(disp,Rn)
  constant  MOV_W_R0_AT_DISP_RN   :  Instruction := "10000001--------";  -- MOV.W RO, @(disp,Rn)
  constant  MOV_L_RM_AT_DISP_RN   :  Instruction := "0001------------";  -- MOV.L Rm, @(disp, Rn)

  constant  MOV_B_AT_DISP_RM_R0   :  Instruction := "10000100--------";  -- MOV.B @(disp, Rm), R0
  constant  MOV_W_AT_DISP_RM_R0   :  Instruction := "10000101--------";  -- MOV.W @(disp, Rm), R0
  constant  MOV_L_AT_DISP_RM_RN   :  Instruction := "0101------------";  -- MOV.L @(disp, Rm), Rn

  constant  MOV_B_RM_AT_R0_RN     :  Instruction := "0000--------0100";  -- MOV.B Rm, @(R0, Rn)
  constant  MOV_W_RM_AT_R0_RN     :  Instruction := "0000--------0101";  -- MOV.W Rm, @(R0, Rn)
  constant  MOV_L_RM_AT_R0_RN     :  Instruction := "0000--------0110";  -- MOV.L Rm, @(R0, Rn)

  constant  MOV_B_AT_R0_RM_RN     :  Instruction := "0000--------1100";  -- MOV.B @(R0, Rm), Rn
  constant  MOV_W_AT_R0_RM_RN     :  Instruction := "0000--------1101";  -- MOV.W @(R0, Rm), Rn
  constant  MOV_L_AT_R0_RM_RN     :  Instruction := "0000--------1110";  -- MOV.L @(R0, Rm), Rn

  constant  MOV_B_R0_AT_DISP_GBR  :  Instruction := "11000000--------";  -- MOV.B R0, @(disp, GBR)
  constant  MOV_W_R0_AT_DISP_GBR  :  Instruction := "11000001--------";  -- MOV.W R0, @(disp, GBR)
  constant  MOV_L_R0_AT_DISP_GBR  :  Instruction := "11000010--------";  -- MOV.L R0, @(disp, GBR)

  constant  MOV_B_AT_DISP_GBR_R0  :  Instruction := "11000100--------";  -- MOV.B @(disp, GBR), R0
  constant  MOV_W_AT_DISP_GBR_R0  :  Instruction := "11000101--------";  -- MOV.W @(disp, GBR), R0
  constant  MOV_L_AT_DISP_GBR_R0  :  Instruction := "11000110--------";  -- MOV.L @(disp, GBR), R0

  constant  MOVA_AT_DISP_PC_R0    :  Instruction := "11000111--------";  -- MOVA @(disp, PC), R0

  constant  MOVT_RN               :  Instruction := "0000----00101001";  -- MOVT Rn

  constant  SWAP_B_RM_RN          :  Instruction := "0110--------1000";  -- SWAP.B Rm, Rn
  constant  SWAP_W_RM_RN          :  Instruction := "0110--------1001";  -- SWAP.W Rm, Rn

  constant  XTRCT_RM_RN           :  Instruction := "0010--------1101";  -- XTRCT Rm, Rn



  -- Arithmetic Instructions:
  constant ADD_RM_RN     : Instruction := "0011--------11--";
  constant ADD_IMM_RN    : Instruction := "0111------------";
  constant SUB_RM_RN     : Instruction := "0011--------10--";
  constant NEG_RM_RN     : Instruction := "0110--------101-";
  constant DT_RN         : Instruction := "0100----00010000";

  constant CMP_EQ_IMM    : Instruction := "10001000--------";   -- CMP/EQ #imm, R0
  constant CMP_RM_RN     : Instruction := "0011--------0---";   -- CMP/{EQ,HS,GE,HI,GT} Rm, Rn
  constant CMP_RN        : Instruction := "0100----00010-01";   -- CMP/{PL/PZ}
  constant CMP_STR_RM_RN : Instruction := "0010--------1100";   -- CMP/STR

  -- Logical Operations:
  constant LOGIC_RM_RN   : Instruction := "0010--------10--";  -- AND, TST, OR, XOR
  constant LOGIC_IMM_R0  : Instruction := "110010----------";  -- AND, TST, OR, XOR
  constant NOT_RM_RN     : Instruction := "0110--------0111";  -- NOT

  -- Shift Instruction:
  constant SHIFT_RN      : Instruction := "0100----00-00-0-";

  -- Branch Instructions:
  constant BF     :   Instruction := "10001011--------"; -- BF   <label>
  constant BF_S   :   Instruction := "10001111--------"; -- BF/S <label>
  constant BT     :   Instruction := "10001001--------"; -- BT   <label>
  constant BT_S   :   Instruction := "10001101--------"; -- BT/S <label>
  constant BRA    :   Instruction := "1010------------"; -- BRA  <label>
  constant BRAF   :   Instruction := "0000----00100011"; -- BRAF Rm
  constant BSR    :   Instruction := "1011------------"; -- BSR  <label>
  constant BSRF   :   Instruction := "0000----00000011"; -- BSRF Rm
  constant JMP    :   Instruction := "0100----00101011"; -- JMP @Rm
  constant JSR    :   Instruction := "0100----00001011"; -- JSR @Rm
  constant RTS    :   Instruction := "0000000000001011"; -- RTS


  -- System Control:
  constant NOP  : Instruction := "0000000000001001";
  constant CLRT : Instruction := "0000000000001000";
  constant SETT : Instruction := "0000000000011000";

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

  constant LDC_L_RM_SYS           : Instruction := "0100----00--0111"; -- LDC.L @Rm+, {SR, GBR, VBR}
  constant LDC_L_AT_RM_PLUS_SR    : Instruction := "0100----00000111"; -- LDC.L @Rm+, SR
  constant LDC_L_AT_RM_PLUS_GBR   : Instruction := "0100----00010111"; -- LDC.L @Rm+, GBR
  constant LDC_L_AT_RM_PLUS_VBR   : Instruction := "0100----00100111"; -- LDC.L @Rm+, VBR


end package SH2InstructionEncodings;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package SH2ControlConstants is

    -- Internal control signals for controlling muxes within the CPU
    constant RegDataIn_ALUResult      : std_logic_vector(3 downto 0) := "0000";
    constant RegDataIn_Immediate      : std_logic_vector(3 downto 0) := "0001";
    constant RegDataIn_RegA           : std_logic_vector(3 downto 0) := "0010";
    constant RegDataIn_RegB           : std_logic_vector(3 downto 0) := "0011";
    constant RegDataIn_SR             : std_logic_vector(3 downto 0) := "0100"; -- BIT DECODED - DO NOT CHANGE
    constant RegDataIn_GBR            : std_logic_vector(3 downto 0) := "0101"; -- BIT DECODED - DO NOT CHANGE
    constant RegDataIn_VBR            : std_logic_vector(3 downto 0) := "0110"; -- BIT DECODED - DO NOT CHANGE
    constant RegDataIn_RegA_SWAP_B    : std_logic_vector(3 downto 0) := "0111";
    constant RegDataIn_RegA_SWAP_W    : std_logic_vector(3 downto 0) := "1000";
    constant RegDataIn_SignExt_B_RegA : std_logic_vector(3 downto 0) := "1001";
    constant RegDataIn_SignExt_W_RegA : std_logic_vector(3 downto 0) := "1010";
    constant RegDataIn_ZeroExt_B_RegA : std_logic_vector(3 downto 0) := "1011";
    constant RegDataIn_ZeroExt_W_RegA : std_logic_vector(3 downto 0) := "1100";
    constant RegDataIn_DB             : std_logic_vector(3 downto 0) := "1101";
    constant RegDataIn_SR_TBit        : std_logic_vector(3 downto 0) := "1110";
    constant RegDataIn_PR             : std_logic_vector(3 downto 0) := "1111";

    constant ReadWrite_READ     : std_logic := '0';
    constant ReadWrite_WRITE    : std_logic := '1';

    constant MemOut_RegA    : std_logic_vector(2 downto 0) := "000";
    constant MemOut_RegB    : std_logic_vector(2 downto 0) := "001";
    constant MemOut_PR      : std_logic_vector(2 downto 0) := "010";
    constant MemOut_PC      : std_logic_vector(2 downto 0) := "011";
    constant MemOut_SR      : std_logic_vector(2 downto 0) := "100"; -- BIT DECODED - DO NOT CHANGE
    constant MemOut_GBR     : std_logic_vector(2 downto 0) := "101"; -- BIT DECODED - DO NOT CHANGE
    constant MemOut_VBR     : std_logic_vector(2 downto 0) := "110"; -- BIT DECODED - DO NOT CHANGE

    constant ALUOpB_RegB    : std_logic := '0';
    constant ALUOpB_Imm     : std_logic := '1';

    constant TFlagSel_T         : std_logic_vector(2 downto 0) := "000";    -- Have T retain its value
    constant TFlagSel_Zero      : std_logic_vector(2 downto 0) := "001";    -- Set T to the ALU zero flag
    constant TFlagSel_Carry     : std_logic_vector(2 downto 0) := "010";    -- Set T to the ALU carry flag
    constant TFlagSel_Overflow  : std_logic_vector(2 downto 0) := "011";    -- Set T to the ALU overflow flag
    constant TFlagSel_SET       : std_logic_vector(2 downto 0) := "100";    -- clear T (to 0)
    constant TFlagSel_CLEAR     : std_logic_vector(2 downto 0) := "101";    -- set T (to 1)
    constant TFlagSel_CMP       : std_logic_vector(2 downto 0) := "110";    -- set T to a value computed from
                                                                            -- the ALU flags

    -- BIT DECODED - DO NOT CHANGE
    constant TCMP_EQ            : std_logic_vector(2 downto 0) := "000";
    constant TCMP_HS            : std_logic_vector(2 downto 0) := "010";
    constant TCMP_GE            : std_logic_vector(2 downto 0) := "011";
    constant TCMP_HI            : std_logic_vector(2 downto 0) := "110";
    constant TCMP_GT            : std_logic_vector(2 downto 0) := "111";
    constant TCMP_STR           : std_logic_vector(2 downto 0) := "100";

    constant MemSel_ROM         : std_logic := '1';
    constant MemSel_RAM         : std_logic := '0';

    constant MemAddrSel_PMAU    : std_logic := '0';
    constant MemAddrSel_DMAU    : std_logic := '1';

    constant SysRegCtrl_NONE    : std_logic := '0';     -- do nothing with system register
    constant SysRegCtrl_LOAD    : std_logic := '1';     -- load system register with new value

    constant SysRegSrc_RegB     : std_logic := '0';     -- load system register from register bus B
    constant SysRegSrc_DB       : std_logic := '1';     -- load system register from data bus

    constant SysRegSel_SR   : std_logic_vector(1 downto 0) := "00";
    constant SysRegSel_GBR  : std_logic_vector(1 downto 0) := "01";
    constant SysRegSel_VBR  : std_logic_vector(1 downto 0) := "10";
    constant SysRegSel_PR   : std_logic_vector(1 downto 0) := "11";

    constant GBRInSel_RegB : std_logic_vector(1 downto 0) := "00";
    constant GBRInSel_DB   : std_logic_vector(1 downto 0) := "01";

    -- Whether to sign or zero extend the immediate into a 32-bit word.
    constant ImmediateMode_SIGN     : std_logic := '0';
    constant ImmediateMode_ZERO     : std_logic := '1';

end package SH2ControlConstants;


library ieee;
library std;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use std.textio.all;

use work.SH2PmauConstants.all;
use work.SH2DmauConstants.all;
use work.MemoryInterfaceConstants.all;
use work.SH2InstructionEncodings.all;
use work.SH2ControlConstants.all;
use work.SH2ALUConstants.all;
use work.Logging.all;
use work.Utils.all;

entity  SH2Control  is

    port (
        MemDataIn   : in  std_logic_vector(31 downto 0);    -- data read from memory
        clock       : in  std_logic;                        -- system clock
        reset       : in  std_logic;                        -- system reset (active low, async)

        -- control signals to control memory interface
        MemEnable   : out std_logic;                        -- if memory needs to be accessed (read or write)
        MemAddrSel  : out std_logic;
        ReadWrite   : out std_logic;                        -- if should do memory read (0) or write (1)
        MemMode     : out std_logic_vector(1 downto 0);     -- if memory access should be by byte, word, or longword
        Disp        : out std_logic_vector(11 downto 0);    -- memory displacement
        MemSel      : out std_logic;                        -- select memory address source, from DMAU output (0) or PMAU output (1)

        Immediate   : out std_logic_vector(7 downto 0);     -- 8-bit immediate
        ImmediateMode   : out std_logic;                    -- Immediate extension mode
        MemOutSel   : out std_logic_vector(2 downto 0);     -- what should be output to memory
        TFlagSel    : out std_logic_vector(2 downto 0);     -- source for next value of T flag

        -- ALU control signals
        ALUOpBSel   : out std_logic;                        -- input mux to Operand B, either RegB (0) or Immediate (1)
        LoadA       : out std_logic;                        -- determine if OperandA is loaded ('1') or zeroed ('0')
        FCmd        : out std_logic_vector(3 downto 0);     -- F-Block operation
        CinCmd      : out std_logic_vector(1 downto 0);     -- carry in operation
        SCmd        : out std_logic_vector(2 downto 0);     -- shift operation
        ALUCmd      : out std_logic_vector(1 downto 0);     -- ALU result select

        TSel        : out std_logic_vector(2 downto 0);     -- if T should be updated to a new value (T/C/V/0/1)
        TCmpSel     : out std_logic_vector(2 downto 0);     -- how to compute T from ALU status flags

        -- register array control signals
        RegDataInSel: out std_logic_vector(3 downto 0);     -- source for register input data
        EnableIn    : out std_logic;                        -- if data should be written to an input register
        RegInSel    : out integer  range 15 downto 0;       -- which register to write data to
        RegASel     : out integer  range 15 downto 0;       -- which register to read to bus A
        RegBSel     : out integer  range 15 downto 0;       -- which register to read to bus B
        RegAxIn     : out std_logic_vector(31 downto 0);    -- data to write to an address register
        RegAxInSel  : out integer  range 15 downto 0;       -- which address register to write to
        RegAxStore  : out std_logic;                        -- if data should be written to the address register
        RegA1Sel    : out integer  range 15 downto 0;       -- which register to read to address bus 1
        RegA2Sel    : out integer  range 15 downto 0;       -- which register to read to address bus 2

        -- DMAU signals
        GBRWriteEn      : out std_logic;
        DMAUOff4        : out std_logic_vector(3 downto 0);
        DMAUOff8        : out std_logic_vector(7 downto 0);
        BaseSel         : out std_logic_vector(1 downto 0);
        IndexSel        : out std_logic_vector(1 downto 0);
        OffScalarSel    : out std_logic_vector(1 downto 0);
        IncDecSel       : out std_logic_vector(1 downto 0);

        -- PMAU signals
        PCAddrMode      : out std_logic_vector(2 downto 0);
        PRWriteEn       : out std_logic;
        PMAUOff8        : out std_logic_vector(7 downto 0);
        PMAUOff12       : out std_logic_vector(11 downto 0);

        -- System control signals
        SysRegCtrl      : out std_logic;
        SysRegSel       : out std_logic_vector(1 downto 0);
        SysRegSrc       : out std_logic;

        GBRInSel        : out std_logic_vector(1 downto 0)  -- Select GBRIn source.
);
    
end  SH2Control;

architecture dataflow of sh2control is
    
    type state_t is (
        fetch,
        execute,
        writeback
    );

    signal state : state_t;


  -- The instruction register.
  signal IR : std_logic_vector(15 downto 0);

  -- Aliases for instruction arguments. 
  -- There are 13 instruction formats, shown below:
  --
  -- Key:
  --  xxxx: instruction code
  --  mmmm: Source register
  --  nnnn: Destination register
  --  iiii: immediate data
  --  dddd: displacment

  -- 0 format:   xxxx xxxx xxxx xxxx
  -- n format:   xxxx nnnn xxxx xxxx
  -- m format:   xxxx mmmm xxxx xxxx
  -- nm format:  xxxx nnnn mmmm xxxx
  -- md format:  xxxx xxxx mmmm dddd
  -- nd4 format: xxxx xxxx nnnn dddd
  -- nmd format: xxxx nnnn mmmm dddd
  -- d format:   xxxx xxxx dddd dddd
  -- d12 format: xxxx dddd dddd dddd
  -- nd8 format: xxxx nnnn dddd dddd
  -- i format:   xxxx xxxx iiii iiii
  -- ni format:  xxxx nnnn iiii iiii
  --

  -- n format
  alias n_format_n : std_logic_vector(3 downto 0) is IR(11 downto 8);

  -- m format
  alias m_format_m : std_logic_vector(3 downto 0) is IR(11 downto 8);

  -- nm format
  alias nm_format_n : std_logic_vector(3 downto 0) is IR(11 downto 8);
  alias nm_format_m : std_logic_vector(3 downto 0) is IR(7 downto 4);

  -- md format
  alias md_format_m : std_logic_vector(3 downto 0) is IR(7 downto 4);
  alias md_format_d : std_logic_vector(3 downto 0) is IR(3 downto 0);

  -- nd4 format
  alias nd4_format_n : std_logic_vector(3 downto 0) is IR(7 downto 4);
  alias nd4_format_d : std_logic_vector(3 downto 0) is IR(3 downto 0);

  -- nmd format
  alias nmd_format_n : std_logic_vector(3 downto 0) is IR(11 downto 8);
  alias nmd_format_m : std_logic_vector(3 downto 0) is IR(7 downto 4);
  alias nmd_format_d : std_logic_vector(3 downto 0) is IR(3 downto 0);

  -- d format
  alias d_format_d : std_logic_vector(7 downto 0) is IR(7 downto 0);

  -- d12 format
  alias d12_format_d : std_logic_vector(11 downto 0) is IR(11 downto 0);

  -- nd8 format
  alias nd8_format_n : std_logic_vector(3 downto 0) is IR(11 downto 8);
  alias nd8_format_d : std_logic_vector(7 downto 0) is IR(7 downto 0);

  -- i format
  alias i_format_i : std_logic_vector(7 downto 0) is IR(7 downto 0);

  -- ni format
  alias ni_format_n : std_logic_vector(3 downto 0) is IR(11 downto 8);
  alias ni_format_i : std_logic_vector(7 downto 0) is IR(7 downto 0);

  -- Internal signals computed combinatorially to memory signals can
  -- be output on the correct clock.
  signal Instruction_MemEnable   : std_logic;
  signal Instruction_ReadWrite   : std_logic;
  signal Instruction_MemSel      : std_logic;
  signal Instruction_MemAddrSel  : std_logic;

  -- The memory mode for a given instruction. The same as the constants in the
  -- MemoryInterfaceConstants package.
  signal Instruction_WordMode : std_logic_vector(1 downto 0);

  -- Register write enable for the current instruction. Output to RegisterArray 
  -- during the execute state so that it is high when the rising clock of the writeback state occurs.
  signal Instruction_EnableIn  : std_logic;   

  -- Address register write enable for the current instruction. Output to RegisterArray 
  -- during the execute state so that it is high when the rising clock of the writeback state occurs.
  signal Instruction_RegAxStore : std_logic;

  -- Program addressing mode for the current instruction. Output during the
  -- writeback state so that it is ready by the following fetch state.
  signal Instruction_PCAddrMode : std_logic_vector(2 downto 0);

  -- What to write to the TFlag for the current instruction. Output during
  -- the execute state so that it is high when the rising clock of the writeback state occurs.
  signal Instruction_TFlagSel    : std_logic_vector(2 downto 0);

  -- If the system register should be loaded or not
  signal Instruction_SysRegCtrl  : std_logic;
  
  -- If the GBR register should be updated or not. Output during execute state so it is
  -- high for the rising clock edge of writeback.
  signal Instruction_GBRWriteEn  : std_logic;

begin

    -- Outputs that change based on the CPU state
    with state select 
        PCAddrMode <= Instruction_PCAddrMode when writeback,  -- increment PC during writeback state
                      PCAddrMode_HOLD        when others;     -- otherwise, hold PC

    with state select 
        MemEnable <= Instruction_MemEnable when execute,        -- if instruction requires memory access
                     MemEnable_ON            when fetch,        -- enable to fetch instruction
                     MemEnable_OFF           when writeback;    -- no memory access during writeback

    with state select 
        ReadWrite <= Instruction_ReadWrite when execute,        -- if instruction does read/write
                     Mem_READ              when fetch,         -- read instruction during fetch
                     'X'                   when writeback;     -- no memory access during writeback

    with state select 
        MemMode <= Instruction_WordMode when execute,       -- instruction memory mode
                   WordMode             when fetch,         -- fetch instruction word
                   (others => 'X')      when others;        -- no memory access during writeback

    with state select
        MemSel <= Instruction_MemSel when execute,      -- if instruction access RAM or ROM
                  MemSel_ROM         when fetch,        -- access ROM to fetch instruction
                  'X'                when others;       -- no memory access during writeback

    with state select
        MemAddrSel <= Instruction_MemAddrSel when execute,  -- if instruction accesses PMAU or DMAU address
                      MemAddrSel_PMAU        when fetch,    -- access program memory during fetch
                      'X'                    when others;   -- no memory access during writeback

    with state select 
        GBRWriteEn <= Instruction_GBRWriteEn when execute,      -- if instruction updates GBR
                      '0'                    when others;       -- don't change GBR

    -- Only modify registers after execute clock
    EnableIn <= Instruction_EnableIn when state = execute else '0';

    -- Only modify address registers after execute clock
    RegAxStore <= Instruction_RegAxStore when state = execute else '0';

    -- Only modify T flag bit after execute clock
    TFlagSel <= Instruction_TFlagSel when state = execute else TFlagSel_T;

    -- Only update system register after execute clock (on writeback)
    SysRegCtrl <= Instruction_SysRegCtrl when state = execute else SysRegCtrl_NONE;

    decode_proc: process (IR)
      variable l : line;
    begin

        -- Default flag values are set here (these shouldn't change CPU state).
        -- This is so that not every control signal has to be set in every single
        -- instruction case. If an instruction enables writing to memory/registers,
        -- then ensure that the default value is set here as "disable" to prevent
        -- writes on the clocks following an instruction.

        -- Not accessing memory
        Instruction_MemEnable  <= '0';
        Instruction_ReadWrite  <= 'X';
        Instruction_WordMode   <= "XX";
        MemOutSel              <= "XXX";
        Instruction_MemSel     <= MemSel_RAM;       -- access data memory by default
        Instruction_MemAddrSel <= MemAddrSel_DMAU;  -- access data memory by default

        -- Register enables
        Instruction_EnableIn    <= '0';             -- Disable register write
        Instruction_RegAxStore  <= '0';             -- Disable writing to address register.
        Instruction_TFlagSel    <= TFlagSel_T;      -- Keep T flag the same
        Instruction_GBRWriteEn  <= '0';             -- Don't write to GBR.
        -- GBRWriteEn              <= '0';             -- Don't write to GBR.
        PRWriteEn               <= '0';             -- Don't write to PR.

        -- Defatult behavior
        Instruction_PCAddrMode  <= PCAddrMode_INC;  -- Increment PC
        Instruction_SysRegCtrl <= SysRegCtrl_NONE;  -- system register not selected
        ImmediateMode <= ImmediateMode_SIGN;        -- sign-extend immediates by defualt

        if std_match(IR, ADD_RM_RN) then

            LogWithTime(l, "sh2_control.vhd: Decoded Add R" & to_string(to_integer(unsigned(nm_format_m))) &
                           " , R" & to_string(to_integer(unsigned(nm_format_n))), LogFile);

            -- report "Instruction: ADD(C/V) Rm, Rn";

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));
            RegBSel <= to_integer(unsigned(nm_format_m));

            RegInSel             <= to_integer(unsigned(nm_format_n));
            RegDataInSel         <= RegDataIn_ALUResult;
            Instruction_EnableIn <= '1';

            -- Bit-decoding T flag select (None, Carry, Overflow)
            Instruction_TFlagSel <= '0' & IR(1 downto 0);

            -- ALU signals
            ALUOpBSel <= ALUOpB_RegB;
            LoadA     <= '1';
            FCmd      <= FCmd_B;

            -- Bit-decode carry in value
            if IR(1 downto 0) = "10" then
                CinCmd <= CinCmd_CIN;   -- ADDC
            else
                CinCmd <= CinCmd_ZERO;  -- ADD, ADDV
            end if;

            SCmd   <= "XXX";
            ALUCmd <= ALUCmd_ADDER;


        -- SUB Rm, Rn
        elsif std_match(IR, SUB_RM_RN) then
            -- report "Instruction: SUB(C/V) Rm, Rn";

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));
            RegBSel <= to_integer(unsigned(nm_format_m));

            RegInSel             <= to_integer(unsigned(nm_format_n));
            RegDataInSel         <= RegDataIn_ALUResult;
            Instruction_EnableIn <= '1';

            -- Bit-decoding T flag select (None, Carry, Overflow)
            Instruction_TFlagSel <= '0' & IR(1 downto 0);

            -- ALU signals
            ALUOpBSel <= ALUOpB_RegB;
            LoadA     <= '1';
            FCmd      <= FCmd_BNOT;

            -- Bit-decode carry in value
            if IR(1 downto 0) = "10" then
                CinCmd <= CinCmd_CINBAR;    -- SUBC
            else
                CinCmd <= CinCmd_ONE;       -- SUB, SUBV
            end if;

            SCmd   <= "XXX";
            ALUCmd <= ALUCmd_ADDER;

        elsif std_match(IR, DT_RN) then
            -- report "Instruction: DT Rn";

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));
            Immediate <= (others => '0');

            RegInSel             <= to_integer(unsigned(nm_format_n));
            RegDataInSel         <= RegDataIn_ALUResult;
            Instruction_EnableIn <= '1';

            -- Bit-decoding T flag select (None, Carry, Overflow)
            Instruction_TFlagSel <= TFlagSel_Zero;

            -- ALU signals to subtract 1 from Rn
            ALUOpBSel <= ALUOpB_Imm;
            LoadA     <= '1';
            FCmd      <= FCmd_BNOT;
            CinCmd <= CinCmd_ZERO;
            SCmd   <= "XXX";
            ALUCmd <= ALUCmd_ADDER;

        -- NEG Rm, Rn
        elsif std_match(IR, NEG_RM_RN) then
            -- report "Instruction: NEG(C) Rm, Rn";

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));
            RegBSel <= to_integer(unsigned(nm_format_m));

            RegInSel             <= to_integer(unsigned(nm_format_n));
            RegDataInSel         <= RegDataIn_ALUResult;
            Instruction_EnableIn <= '1';

            -- Bit-decoding T flag select
            if IR(0) = '0' then
                Instruction_TFlagSel <= TFlagSel_Carry;
            else
                Instruction_TFlagSel <= TFlagSel_T;
            end if;

            -- ALU signals
            ALUOpBSel <= ALUOpB_RegB;
            LoadA     <= '0';
            FCmd      <= FCmd_BNOT;

            -- Bit-decode carry in value
            if IR(0) = '0' then
                CinCmd <= CinCmd_CINBAR;    -- NEGC
            else
                CinCmd <= CinCmd_ONE;       -- NEG
            end if;

            SCmd   <= "XXX";
            ALUCmd <= ALUCmd_ADDER;

        -- ADD #imm, Rn
        elsif std_match(IR, ADD_IMM_RN) then
            -- report "Instruction: ADD #imm, Rn";

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));

            RegInSel             <= to_integer(unsigned(nm_format_n));
            RegDataInSel         <= RegDataIn_ALUResult;
            Instruction_EnableIn <= '1';
            Immediate            <= ni_format_i;

            -- ALU signals
            ALUOpBSel <= ALUOpB_Imm;
            LoadA     <= '1';
            FCmd      <= FCmd_B;
            CinCmd    <= CinCmd_ZERO;
            SCmd      <= "XXX";
            ALUCmd    <= ALUCmd_ADDER;

        elsif std_match(IR, LOGIC_RM_RN) then
            -- {AND, TST, OR, XOR} Rm, Rn
            -- Uses bit decoding to distinguish between the four possible operations

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));
            RegBSel <= to_integer(unsigned(nm_format_m));

            RegInSel             <= to_integer(unsigned(nm_format_n));
            RegDataInSel         <= RegDataIn_ALUResult;
            Instruction_EnableIn <= IR(1) or IR(0);   -- exclude TST

            -- Enable TFlagSel for TST
            Instruction_TFlagSel <= TFlagSel_Zero when IR(1 downto 0) = "00" else TFlagSel_T;

            -- ALU signals
            ALUOpBSel <= ALUOpB_RegB;
            LoadA     <= '1';

            FCmd <= FCmd_AND when IR(1) = '0'           else
                    FCmd_XOR when IR(1 downto 0) = "10" else
                    FCmd_OR;

            CinCmd <= CinCmd_ZERO;
            SCmd   <= "XXX";
            ALUCmd <= ALUCmd_FBLOCK;

        elsif std_match(IR, LOGIC_IMM_R0) then
            -- {AND, TST, OR, XOR} immediate, R0

            -- Register array signals
            RegASel <= 0;

            RegInSel             <= 0;
            RegDataInSel         <= RegDataIn_ALUResult;
            Instruction_EnableIn <= IR(9) or IR(8);   -- exclude TST
            Immediate            <= i_format_i;
            ImmediateMode        <= ImmediateMode_ZERO;

            -- Enable TFlagSel for TST
            Instruction_TFlagSel <= TFlagSel_Zero when IR(9 downto 8) = "00" else TFlagSel_T;

            -- ALU signals
            ALUOpBSel <= ALUOpB_Imm;
            LoadA     <= '1';

            FCmd <= FCmd_AND when IR(9) = '0' else
                    FCmd_XOR when IR(9 downto 8) = "10" else
                    FCmd_OR;

            CinCmd <= CinCmd_ZERO;
            SCmd   <= "XXX";
            ALUCmd <= ALUCmd_FBLOCK;

        elsif std_match(IR, NOT_RM_RN) then
            -- NOT Rm, Rn

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));
            RegBSel <= to_integer(unsigned(nm_format_m));

            RegInSel             <= to_integer(unsigned(nm_format_n));
            RegDataInSel         <= RegDataIn_ALUResult;
            Instruction_EnableIn <= '1';

            -- ALU signals
            ALUOpBSel <= ALUOpB_RegB;
            LoadA     <= '1';
            FCmd      <= FCmd_BNOT;
            CinCmd    <= CinCmd_ZERO;
            SCmd      <= "XXX";
            ALUCmd    <= ALUCmd_FBLOCK;

        elsif std_match(IR, CMP_EQ_IMM) then
            -- report "Instruction: CMP/EQ #Imm, R0";

            -- Register array signals
            RegASel <= 0;

            Immediate            <= i_format_i;
            ImmediateMode        <= ImmediateMode_ZERO;

            -- Compute T flag based on ALU flags
            Instruction_TFlagSel <= TFlagSel_CMP;
            TCMPSel <= TCmp_EQ;

            -- ALU Instructions that perform a subtraction (Rn - immediate) so that
            -- the ALU output flags can be used to compute the T flag
            ALUOpBSel <= ALUOpB_Imm;
            LoadA     <= '1';
            FCmd      <= FCmd_BNOT;
            CinCmd <= CinCmd_ONE;
            SCmd   <= "XXX";
            ALUCmd <= ALUCmd_ADDER;

        elsif std_match(IR, CMP_RM_RN) then
            -- report "Instruction: CMP/XX Rm, Rn";

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));
            RegBSel <= to_integer(unsigned(nm_format_m));

            -- Compute T flag based on ALU flags
            Instruction_TFlagSel <= TFlagSel_CMP;
            TCMPSel <= IR(2 downto 0);              -- bit decode T flag CMP condition

            -- ALU Instructions that perform a subtraction (Rn - Rm) so that
            -- the ALU output flags can be used to compute the T flag
            ALUOpBSel <= ALUOpB_RegB;
            LoadA     <= '1';
            FCmd      <= FCmd_BNOT;
            CinCmd <= CinCmd_ONE;
            SCmd   <= "XXX";
            ALUCmd <= ALUCmd_ADDER;

        elsif std_match(IR, CMP_STR_RM_RN) then
            -- report "Instruction: CMP/STR Rm, Rn";

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));
            RegBSel <= to_integer(unsigned(nm_format_m));

            -- Compute T flag based on ALU flags
            Instruction_TFlagSel <= TFlagSel_CMP;
            TCMPSel <= TCMP_STR;

        elsif std_match(IR, CMP_RN) then
            -- report "Instruction: CMP/{PL/PZ} Rn";

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));

            -- Compare to 0
            Immediate <= (others => '0');

            -- Compute T flag based on ALU flags
            Instruction_TFlagSel <= TFlagSel_CMP;
            TCMPSel <= IR(2) & "11";                -- bit decode CMP mode (either GT or GE)

            -- ALU Instructions that perform a subtraction (Rn - 0) so that
            -- the ALU output flags can be used to compute the T flag
            ALUOpBSel <= ALUOpB_Imm;
            LoadA     <= '1';
            FCmd      <= FCmd_BNOT;
            CinCmd <= CinCmd_ONE;
            SCmd   <= "XXX";
            ALUCmd <= ALUCmd_ADDER;

        elsif std_match(IR, SHIFT_RN) then
            -- {ROTL, ROTR, ROTCL, ROTCR, SHAL, SHAR, SHLL, SHLR} Rn
            -- Uses bit decoding to compute control signals (to reduce code size)

            -- Register array signals
            RegASel              <= to_integer(unsigned(n_format_n));
            RegInSel             <= to_integer(unsigned(n_format_n));
            RegDataInSel         <= RegDataIn_ALUResult;
            Instruction_EnableIn <= '1';

            Instruction_TFlagSel <= TFlagSel_Carry;

            -- ALU signals
            ALUOpBSel <= ALUOpB_RegB;
            LoadA     <= '1';
            FCmd      <= "XXXX";

            CinCmd    <= CinCmd_CIN when (IR(5) and IR(2)) = '1' else  -- ROTCL, ROTCR
                         CinCmd_ZERO;     

            SCmd   <= IR(0) & IR(2) & IR(5);  -- bit-decode shift operation
            ALUCmd <= ALUCmd_SHIFT;

        
        -- Data Transfer Instruction -------------------------------------------

        -- MOV #imm, Rn
        -- ni format
        elsif std_match(IR, MOV_IMM_RN) then
            LogWithTime(l, "sh2_control.vhd: Decoded MOV H'" & to_hstring(ni_format_i) &
                          ", R" & to_string(slv_to_uint(ni_format_n)), LogFile);
          
            RegInSel             <= to_integer(unsigned(ni_format_n));
            RegDataInSel         <= RegDataIn_Immediate;
            Instruction_EnableIn <= '1';
            Immediate            <= ni_format_i;

        -- MOV.W @(disp, PC), Rn
        -- nd8 format
        -- NOTE: Testing this assumes MOV into memory works.
        --
        elsif std_match(IR, MOV_W_AT_DISP_PC_RN) then
          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.W @(0x" & to_hstring(nd8_format_d) &
            ", PC), R" & to_string(slv_to_uint(nd8_format_n)), LogFile);


          RegInSel             <= to_integer(unsigned(nd8_format_n));   -- Writing to register n 
          RegDataInSel         <= RegDataIn_DB;                         -- Writing output of data bus to register. 
          Instruction_EnableIn <= '1';                                  -- Writes to register. 

          RegASel <= to_integer(unsigned(nd8_format_n));

          -- Instruction reads word from program memory (ROM).
          Instruction_MemEnable <= '1';
          Instruction_ReadWrite <= ReadWrite_READ; 
          Instruction_WordMode  <= WordMode;
          Instruction_MemSel    <= MemSel_ROM;

          -- DMAU signals for PC Relative addressing with displacement (word mode)
          BaseSel      <= BaseSel_PC;
          IndexSel     <= IndexSel_OFF8;
          OffScalarSel <= OffScalarSel_TWO;
          IncDecSel    <= IncDecSel_NONE;
          DMAUOff8     <= nd8_format_d;


        -- MOV.L @(disp, PC), Rn
        -- nd8 format
        elsif std_match(IR, MOV_L_AT_DISP_PC_RN) then
          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.L @(0x" & to_hstring(nd8_format_d) &
            ", PC), R" & to_string(slv_to_uint(nd8_format_n)), LogFile);

          RegInSel             <= to_integer(unsigned(nd8_format_n));  -- Writing to register n 
          RegDataInSel         <= RegDataIn_DB;                        -- Writing output of data bus to register. 
          Instruction_EnableIn <= '1';                                 -- Writes to register. 

          -- Instruction reads from longword memory.
          Instruction_MemEnable <= '1';
          Instruction_ReadWrite <= ReadWrite_READ; 
          Instruction_WordMode  <= LongwordMode;
          Instruction_MemSel    <= MemSel_ROM;

          -- DMAU signals for PC Relative addressing with displacement (longword mode)
          BaseSel      <= BaseSel_PC;
          IndexSel     <= IndexSel_OFF8;
          OffScalarSel <= OffScalarSel_FOUR;
          IncDecSel    <= IncDecSel_NONE;
          DMAUOff8     <= nd8_format_d;


        -- MOV Rm, Rn
        -- nm format
        elsif std_match(IR, MOV_RM_RN) then
            LogWithTime(l, 
              "sh2_control.vhd: Decoded MOV R" & to_string(slv_to_uint(nm_format_m)) &
              "R" & to_string(slv_to_uint(nm_format_n)) , LogFile);

            -- report "Instruction: MOV Rm, Rn";
            RegBSel              <= to_integer(unsigned(nm_format_m));
            RegInSel             <= to_integer(unsigned(nm_format_n));
            RegDataInSel         <= RegDataIn_RegB;
            Instruction_EnableIn <= '1';

        -- MOV.X Rm, @Rn
        -- nm format
        elsif std_match(IR, MOV_RM_AT_RN) then
            LogWithTime(l, 
              "sh2_control.vhd: Decoded MOV.X R" & to_string(slv_to_uint(nm_format_m)) &
              ", @R" & to_string(slv_to_uint(nm_format_n)) , LogFile);

            -- Writes a byte to memory to memory
            Instruction_MemEnable <= '1';             -- Uses memory.
            Instruction_ReadWrite <= ReadWrite_WRITE; -- Writes.
            Instruction_WordMode  <= IR(1 downto 0);  -- bit decode memory mode

            MemOutSel <= MemOut_RegB; -- Output RegB (Rm) to memory data bus.

            RegBSel  <= to_integer(unsigned(nm_format_m)); -- RegB is Rm.
            RegA1Sel <= to_integer(unsigned(nm_format_n)); -- RegA is @(Rn)

            -- DMAU signals (for Indirect Register Addressing)
            BaseSel      <= BaseSel_REG;
            IndexSel     <= IndexSel_NONE;
            OffScalarSel <= OffScalarSel_ONE;
            IncDecSel    <= IncDecSel_NONE;

        -- MOV.X @Rm, Rn
        -- nm format
        elsif std_match(IR, MOV_AT_RM_RN) then
          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.X @R" & to_string(slv_to_uint(nm_format_m)) &
            ", R" & to_string(slv_to_uint(nm_format_n)) , LogFile);

          -- Instruction reads byte from memory.
          Instruction_MemEnable <= '1';            -- Instr does memory access.
          Instruction_ReadWrite <= ReadWrite_READ; -- Instr reads from memory.
          Instruction_WordMode  <= IR(1 downto 0); -- bit decode memory mode

          -- DMAU signals for Indirect Register addressing.
          BaseSel      <= BaseSel_REG;
          IndexSel     <= IndexSel_NONE;
          OffScalarSel <= OffScalarSel_ONE;
          IncDecSel    <= IncDecSel_NONE;

          -- Output @(Rm) to RegA2. 
          RegA2Sel <= to_integer(unsigned(nm_format_m));

          RegInSel             <= to_integer(unsigned(nm_format_n));
          RegDataInSel         <= RegDataIn_DB;
          Instruction_EnableIn <= '1';


        -- MOV.B Rm, @-Rn
        -- nm format
        elsif std_match(IR, MOV_RM_AT_MINUS_RN) then
            LogWithTime(l, 
              "sh2_control.vhd: Decoded MOV.X R" & to_string(slv_to_uint(nm_format_m)) &
              ", @-R" & to_string(slv_to_uint(nm_format_n)) , LogFile);

            -- Writes a byte to memory
            Instruction_MemEnable <= '1';             -- Uses memory.
            Instruction_ReadWrite <= ReadWrite_WRITE; -- Writes.
            Instruction_WordMode  <= IR(1 downto 0);  -- bit decode memory mode

            MemOutSel <= MemOut_RegB; -- Output RegB (Rm) to memory data bus.

            RegBSel                <= to_integer(unsigned(nm_format_m));  -- Output Rm from RegB output.
            RegA1Sel               <= to_integer(unsigned(nm_format_n));  -- Output @(Rn) from RegA1 output.
            RegAxInSel             <= to_integer(unsigned(nm_format_n));  -- Store calculated address into Rn
            Instruction_RegAxStore <= '1';                                -- Enable writes to address registers.

            -- DMAU signals (for Pre-decrement indirect register addressing)
            BaseSel      <= BaseSel_REG;
            IndexSel     <= IndexSel_NONE;
            OffScalarSel <= IR(1 downto 0);         -- bit decode offset scalar factor
            IncDecSel    <= IncDecSel_PRE_DEC;

        -- MOV.B @Rm+, Rn
        -- nm format
        elsif std_match(IR, MOV_B_AT_RM_PLUS_RN) then
          -- report "Instruction: [MOV.B @Rm+, Rn] not implemented."
          -- severity ERROR;
          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.B @R" & to_string(slv_to_uint(nm_format_m)) &
            "+, R" & to_string(slv_to_uint(nm_format_n)) , LogFile);

          -- MOV with post-increment. This Instruction reads a byte, word,
          -- or longword from an address in Rm, into Rn. The address is
          -- incremented and stored in Rm after the value is retrieved.
          
          -- Reads a byte from memory.
          Instruction_MemEnable <= '1';             -- Uses memory.
          Instruction_ReadWrite <= ReadWrite_READ;  -- Reads.
          Instruction_WordMode  <= ByteMode;        -- A byte.

          -- Output @Rm from RegA2
          RegA2Sel <= to_integer(unsigned(nm_format_m));

          -- Write output of Data Bus to Rn
          RegInSel             <= to_integer(unsigned(nm_format_n));
          RegDataInSel         <= RegDataIn_DB;
          Instruction_EnableIn <= '1';  -- Enable writing to registers.

          -- Write the incremented address to Rm
          RegAxInSel             <= to_integer(unsigned(nm_format_m));
          Instruction_RegAxStore <= '1'; -- Enable writing to address register in the writeback state.
          
          -- DMAU signals for post-increment indirect register addressing (byte mode)
          BaseSel      <= BaseSel_REG;
          IndexSel     <= IndexSel_NONE;
          OffScalarSel <= OffScalarSel_ONE;
          IncDecSel    <= IncDecSel_POST_INC;

        -- MOV.W @Rm+, Rn
        elsif std_match(IR, MOV_W_AT_RM_PLUS_RN) then
          -- report "Instruction: [MOV.W @Rm+, Rn] not implemented."
          -- severity ERROR;
          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.W @R" & to_string(slv_to_uint(nm_format_m)) &
            "+, R" & to_string(slv_to_uint(nm_format_n)) , LogFile);

          -- Reads a word from memory.
          Instruction_MemEnable <= '1';             -- Uses memory.
          Instruction_ReadWrite <= ReadWrite_READ;  -- Reads.
          Instruction_WordMode  <= WordMode;        -- A word.

          -- Output @Rm from RegA2
          RegA2Sel <= to_integer(unsigned(nm_format_m));

          -- Write output of Data Bus to Rn
          RegInSel             <= to_integer(unsigned(nm_format_n));
          RegDataInSel         <= RegDataIn_DB;
          Instruction_EnableIn <= '1';  -- Enable writing to registers.

          -- Write the incremented address to Rm
          RegAxInSel             <= to_integer(unsigned(nm_format_m));
          Instruction_RegAxStore <= '1'; -- Enable writing to address register in the writeback state.
          
          -- DMAU signals for post-increment indirect register addressing (word mode)
          BaseSel      <= BaseSel_REG;
          IndexSel     <= IndexSel_NONE;
          OffScalarSel <= OffScalarSel_TWO;
          IncDecSel    <= IncDecSel_POST_INC;

        -- MOV.L @Rm+, Rn
        elsif std_match(IR, MOV_L_AT_RM_PLUS_RN) then

          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.L @R" & to_string(slv_to_uint(nm_format_m)) &
            "+, R" & to_string(slv_to_uint(nm_format_n)) , LogFile);

          -- Reads a longword from memory.
          Instruction_MemEnable <= '1';             -- Uses memory.
          Instruction_ReadWrite <= ReadWrite_READ;  -- Reads.
          Instruction_WordMode  <= LongwordMode;    -- A longword.

          -- Output @Rm from RegA2
          RegA2Sel <= to_integer(unsigned(nm_format_m));

          -- Write output of Data Bus to Rn
          RegInSel             <= to_integer(unsigned(nm_format_n));
          RegDataInSel         <= RegDataIn_DB;
          Instruction_EnableIn <= '1';  -- Enable writing to registers.

          -- Write the incremented address to Rm
          RegAxInSel             <= to_integer(unsigned(nm_format_m));
          Instruction_RegAxStore <= '1'; -- Enable writing to address register in the writeback state.
          
          -- DMAU signals for post-increment indirect register addressing (longword mode)
          BaseSel      <= BaseSel_REG;
          IndexSel     <= IndexSel_NONE;
          OffScalarSel <= OffScalarSel_FOUR;
          IncDecSel    <= IncDecSel_POST_INC;

        -- MOV.B RO, @(disp,Rn)
        -- nd4 format
        -- Note that the displacement depends on the mode of the address, so in
        -- byte mode, the displacement represents bytes, in word mode it represents
        -- words, etc. This is done to maximize it's range.
        elsif std_match(IR, MOV_B_R0_AT_DISP_RN) then

          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.B R0, @(0x" & to_hstring(nd4_format_d) &
            ", " & to_string(slv_to_uint(nd4_format_n)) & ")", LogFile);

          -- Instruction writes a byte to data memory.
          Instruction_MemEnable   <= '1';
          Instruction_ReadWrite   <= ReadWrite_WRITE;
          Instruction_WordMode    <= ByteMode;
          
          -- Output RegB (R0) to memory data bus
          MemOutSel <= MemOut_RegB;

          -- Output R0 to RegB
          RegBSel <= 0;
          
          -- Output Rn to RegA1. The DMAU will use this to calculate the address
          -- to write to.
          RegA1Sel <= to_integer(unsigned(nd4_format_n));

          -- DMAU signals for Indirect register addressing with displacement (byte mode)
          BaseSel       <=  BaseSel_REG;
          IndexSel      <=  IndexSel_OFF4;
          OffScalarSel  <=  OffScalarSel_ONE;
          IncDecSel     <=  IncDecSel_NONE;
          DMAUOff4      <=  nd4_format_d;


        -- MOV.W RO, @(disp,Rn)
        -- nd4 format
        elsif std_match(IR, MOV_W_R0_AT_DISP_RN) then

          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.W R0, @(0x" & to_hstring(nd4_format_d) &
            ", " & to_string(slv_to_uint(nd4_format_n)) & ")", LogFile);

          -- Instruction writes a word to memory.
          Instruction_MemEnable   <= '1';
          Instruction_ReadWrite   <= ReadWrite_WRITE;
          Instruction_WordMode    <= WordMode;

          -- Output R0 to RegB
          RegBSel <= 0;
          
          -- Output Rn to RegA1. The DMAU will use this to calculate the address
          -- to write to.
          RegA1Sel <= to_integer(unsigned(nd4_format_n));

          -- Output RegB (R0) to memory data bus
          MemOutSel <= MemOut_RegB;
          
          -- DMAU signals for Indirect register addressing with displacement (word mode)
          BaseSel      <= BaseSel_REG;
          IndexSel     <= IndexSel_OFF4;
          OffScalarSel <= OffScalarSel_TWO;
          IncDecSel    <= IncDecSel_NONE;
          DMAUOff4     <=  nd4_format_d;


        -- MOV.L Rm, @(disp, Rn)
        -- nmd format
        elsif std_match(IR, MOV_L_RM_AT_DISP_RN) then

          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.L R" & to_string(slv_to_uint(nmd_format_m)) &
            ", @(0x" & to_hstring(nmd_format_d) & ", R" & to_string(slv_to_uint(nmd_format_n)) & ")", LogFile);

          Instruction_MemEnable <= '1';
          Instruction_ReadWrite <= ReadWrite_WRITE;
          Instruction_WordMode  <= LongwordMode;

          -- Output Rm to RegB.
          RegBSel <= to_integer(unsigned(nmd_format_m));

          -- Output Rn to RegA1. The DMAU will use this to calculate the address
          -- to write to.
          RegA1Sel <= to_integer(unsigned(nmd_format_n));

          -- Output RegB (Rm) to memory data bus. This will be written to memory.
          MemOutSel <= MemOut_RegB;

          -- DMAU signals for Indirect register addressing with displacement (longword mode)
          BaseSel      <= BaseSel_REG;
          IndexSel     <= IndexSel_OFF4;
          OffScalarSel <= OffScalarSel_FOUR;
          IncDecSel    <= IncDecSel_NONE;
          DMAUOff4     <= nmd_format_d;


        -- MOV.B @(disp, Rm), R0
        -- md format
        -- Note that these instructions are very similar to MOV @(disp, PC), Rn
        --
        elsif std_match(IR, MOV_B_AT_DISP_RM_R0) then

          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.B @(0x" & to_hstring(md_format_d) &
            ", R" & to_string(slv_to_uint(md_format_m)) & "), R0", LogFile);

            -- Writing sign-extended byte from data bus to R0.
            RegInSel             <= 0;             -- Select R0 to write to.
            RegDataInSel         <= RegDataIn_DB;  -- Write DataBus to reg.
            Instruction_EnableIn <= '1';           -- Enable Reg writing for this instruction.

            -- Output @Rm from RegA2
            RegA2Sel <= to_integer(unsigned(md_format_m));

            Instruction_MemEnable  <=  '1';            -- Instr uses memory.
            Instruction_ReadWrite  <=  ReadWrite_READ; -- Reads.
            Instruction_WordMode   <=  ByteMode;       -- Reads byte.
            Instruction_MemSel     <=  MemSel_RAM;     -- Reads from RAM

            -- DMAU signals for Indirect register addressing with displacement (byte mode)
            BaseSel      <= BaseSel_REG;
            IndexSel     <= IndexSel_OFF4;
            OffScalarSel <= OffScalarSel_ONE;
            IncDecSel    <= IncDecSel_NONE;
            DMAUOff4     <= md_format_d;


        -- MOV.W @(disp, Rm), R0
        -- md format
        -- Note that these instructions are very similar to MOV @(disp, PC), Rn
        --
        elsif std_match(IR, MOV_W_AT_DISP_RM_R0) then

            LogWithTime(l, 
                "sh2_control.vhd: Decoded MOV.W @(0x" & to_hstring(md_format_d) &
                ", R" & to_string(slv_to_uint(md_format_m)) & "), R0", LogFile);

            -- Writing sign-extended word from data bus to R0.
            RegInSel             <= 0;             -- Select R0 to write to.
            RegDataInSel         <= RegDataIn_DB;  -- Write DataBus to reg.
            Instruction_EnableIn <= '1';           -- Enable Reg writing for this instruction.

            -- Output @Rm from RegA2
            RegA2Sel <= to_integer(unsigned(md_format_m));

            Instruction_MemEnable  <=  '1';            -- Instr uses memory.
            Instruction_ReadWrite  <=  ReadWrite_READ; -- Reads.
            Instruction_WordMode   <=  WordMode;       -- Reads word.
            Instruction_MemSel     <=  MemSel_RAM;     -- Reads from RAM

            -- DMAU signals for Indirect register addressing with displacement (word mode)
            BaseSel      <= BaseSel_REG;
            IndexSel     <= IndexSel_OFF4;
            OffScalarSel <= OffScalarSel_TWO;
            IncDecSel    <= IncDecSel_NONE;
            DMAUOff4     <= md_format_d;




        -- MOV.L @(disp, Rm), Rn
        -- nmd
        elsif std_match(IR, MOV_L_AT_DISP_RM_RN) then

          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.L @(0x" & to_hstring(nmd_format_d) &
            ", R" & to_string(slv_to_uint(nmd_format_m)) & "), R" & to_string(slv_to_uint(nmd_format_n))
            , LogFile);

          -- Writing longword from data bus to Rn.
          RegInSel             <= to_integer(unsigned(nmd_format_n));   -- Select Rn to write to.
          RegDataInSel         <= RegDataIn_DB;                         -- Write DataBus to reg.
          Instruction_EnableIn <= '1';                                  -- Enable Reg writing for this instruction.

           -- Output @Rm from RegA2
           RegA2Sel <= to_integer(unsigned(nmd_format_m));

          Instruction_MemEnable  <=  '1';               -- Instr uses memory.
          Instruction_ReadWrite  <=  ReadWrite_READ;    -- Reads.
          Instruction_WordMode   <=  LongwordMode;      -- Reads longword.
          Instruction_MemSel     <=  MemSel_RAM;        -- Reads from RAM


          -- DMAU signals for Indirect register addressing with displacement (longword mode)
          BaseSel      <= BaseSel_REG;
          IndexSel     <= IndexSel_OFF4;
          OffScalarSel <= OffScalarSel_FOUR;
          IncDecSel    <= IncDecSel_NONE;
          DMAUOff4     <= nmd_format_d;


        -- MOV.B Rm, @(R0, Rn)
        -- nm format
        elsif std_match(IR, MOV_B_RM_AT_R0_RN) then

          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.B R" & to_string(slv_to_uint(nm_format_m)) &
            ", @(R0, R" & to_string(slv_to_uint(nm_format_n)) & ")", LogFile);

          -- Instr writes a byte to memory.
          Instruction_MemEnable <= '1';
          Instruction_ReadWrite <= ReadWrite_WRITE;
          Instruction_WordMode  <= ByteMode;

          -- Output Rm to RegB.
          RegBSel <= to_integer(unsigned(nm_format_m));

          -- Output Rn to RegA1. The DMAU will use this to calculate the address
          -- to write to.
          RegA1Sel <= to_integer(unsigned(nm_format_n));

          -- Output R0 to RegA2.
          RegA2Sel <= 0;

          -- Output RegB (Rm) to memory data bus. This will be written to memory.
          MemOutSel <= MemOut_RegB;

          -- DMAU Signals for Indirect Register Addressing
          BaseSel       <= BaseSel_REG;
          IndexSel      <= IndexSel_R0;
          OffScalarSel  <= OffScalarSel_ONE;
          IncDecSel     <= IncDecSel_NONE;
        

        -- MOV.W Rm, @(R0, Rn)
        -- nm format
        elsif std_match(IR, MOV_W_RM_AT_R0_RN) then

          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.W R" & to_string(slv_to_uint(nm_format_m)) &
            ", @(R0, R" & to_string(slv_to_uint(nm_format_n)) & ")", LogFile);

          -- Instr writes a word to memory.
          Instruction_MemEnable <= '1';
          Instruction_ReadWrite <= ReadWrite_WRITE;
          Instruction_WordMode  <= WordMode;

          -- Output Rm to RegB.
          RegBSel <= to_integer(unsigned(nm_format_m));

          -- Output Rn to RegA1. The DMAU will use this to calculate the address
          -- to write to.
          RegA1Sel <= to_integer(unsigned(nm_format_n));

          -- Output R0 to RegA2.
          RegA2Sel <= 0;

          -- Output RegB (Rm) to memory data bus. This will be written to memory.
          MemOutSel <= MemOut_RegB;


          -- DMAU Signals for Indirect Register Addressing
          BaseSel       <= BaseSel_REG;
          IndexSel      <= IndexSel_R0;
          OffScalarSel  <= OffScalarSel_ONE;
          IncDecSel     <= IncDecSel_NONE;
        


        -- MOV.L Rm, @(R0, Rn)
        -- nm format
        elsif std_match(IR, MOV_L_RM_AT_R0_RN) then

          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.L R" & to_string(slv_to_uint(nm_format_m)) &
            ", @(R0, R" & to_string(slv_to_uint(nm_format_n)) & ")", LogFile);

          -- Instr writes a longword to memory.
          Instruction_MemEnable <= '1';
          Instruction_ReadWrite <= ReadWrite_WRITE;
          Instruction_WordMode  <= LongwordMode;

          -- Output Rm to RegB.
          RegBSel <= to_integer(unsigned(nm_format_m));

          -- Output Rn to RegA1. The DMAU will use this to calculate the address
          -- to write to.
          RegA1Sel <= to_integer(unsigned(nm_format_n));

          -- Output R0 to RegA2.
          RegA2Sel <= 0;

          -- Output RegB (Rm) to memory data bus. This will be written to memory.
          MemOutSel <= MemOut_RegB;


          -- DMAU Signals for Indirect indexed Register Addressing
          BaseSel       <= BaseSel_REG;
          IndexSel      <= IndexSel_R0;
          OffScalarSel  <= OffScalarSel_ONE;
          IncDecSel     <= IncDecSel_NONE;


        -- MOV.B @(R0, Rm), Rn
        -- nm format
        elsif std_match(IR, MOV_B_AT_R0_RM_RN) then

            LogWithTime(l, 
              "sh2_control.vhd: Decoded MOV.B @(R0, R" & to_string(slv_to_uint(nm_format_m)) &
              "), R" & to_string(slv_to_uint(nm_format_n)), LogFile);

            -- Writing sign-extended byte from data bus to Rn.
            RegInSel             <= slv_to_uint(nm_format_n);     -- Select Rn to write to.
            RegDataInSel         <= RegDataIn_DB;                 -- Write DataBus to reg.
            Instruction_EnableIn <= '1';                          -- Enable Reg writing for this instruction.

            -- Output @Rm from RegA2
            RegA2Sel <= slv_to_uint(nm_format_m);

            -- Output @R0 from RegA1
            RegA1Sel <= 0;

            Instruction_MemEnable  <=  '1';            -- Instr uses memory.
            Instruction_ReadWrite  <=  ReadWrite_READ; -- Reads.
            Instruction_WordMode   <=  ByteMode;       -- Reads byte.
            Instruction_MemSel     <=  MemSel_RAM;     -- Reads from RAM

            -- DMAU Signals for Indirect indexed Register Addressing
            BaseSel       <= BaseSel_REG;
            IndexSel      <= IndexSel_R0;
            OffScalarSel  <= OffScalarSel_ONE;
            IncDecSel     <= IncDecSel_NONE;
          
          

        -- MOV.W @(R0, Rm), Rn
        elsif std_match(IR, MOV_W_AT_R0_RM_RN) then

          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.W @(R0, R" & to_string(slv_to_uint(nm_format_m)) &
            "), R" & to_string(slv_to_uint(nm_format_n)), LogFile);

            -- Writing sign-extended word from data bus to Rn.
            RegInSel             <= slv_to_uint(nm_format_n);     -- Select Rn to write to.
            RegDataInSel         <= RegDataIn_DB;                 -- Write DataBus to reg.
            Instruction_EnableIn <= '1';                          -- Enable Reg writing for this instruction.

            -- Output @Rm from RegA2
            RegA2Sel <= slv_to_uint(nm_format_m);

            -- Output @R0 from RegA1
            RegA1Sel <= 0;

            Instruction_MemEnable  <=  '1';            -- Instr uses memory.
            Instruction_ReadWrite  <=  ReadWrite_READ; -- Reads.
            Instruction_WordMode   <=  WordMode;       -- Reads Word.
            Instruction_MemSel     <=  MemSel_RAM;     -- Reads from RAM

            -- DMAU Signals for Indirect indexed Register Addressing
            BaseSel       <= BaseSel_REG;
            IndexSel      <= IndexSel_R0;
            OffScalarSel  <= OffScalarSel_ONE;
            IncDecSel     <= IncDecSel_NONE;
           

        -- MOV.L @(R0, Rm), Rn
        elsif std_match(IR, MOV_L_AT_R0_RM_RN) then

          LogWithTime(l, 
            "sh2_control.vhd: Decoded MOV.L @(R0, R" & to_string(slv_to_uint(nm_format_m)) &
            "), R" & to_string(slv_to_uint(nm_format_n)), LogFile);

            -- Writing longword from data bus to Rn.
            RegInSel             <= slv_to_uint(nm_format_n);     -- Select Rn to write to.
            RegDataInSel         <= RegDataIn_DB;                 -- Write DataBus to reg.
            Instruction_EnableIn <= '1';                          -- Enable Reg writing for this instruction.

            -- Output @Rm from RegA2
            RegA2Sel <= slv_to_uint(nm_format_m);

            -- Output @R0 from RegA1
            RegA1Sel <= 0;

            Instruction_MemEnable  <=  '1';             -- Instr uses memory.
            Instruction_ReadWrite  <=  ReadWrite_READ;  -- Reads.
            Instruction_WordMode   <=  LongwordMode;    -- Reads longword.
            Instruction_MemSel     <=  MemSel_RAM;      -- Reads from RAM

            -- DMAU Signals for Indirect indexed Register Addressing
            BaseSel       <= BaseSel_REG;
            IndexSel      <= IndexSel_R0;
            OffScalarSel  <= OffScalarSel_ONE;
            IncDecSel     <= IncDecSel_NONE;
          

        -- MOV.B R0, @(disp, GBR)
        -- d format
        elsif std_match(IR, MOV_B_R0_AT_DISP_GBR) then

          LogWithTime(l,
            "sh2_control.vhd: Decoded MOV.B R0, @(0x" & to_hstring(d_format_d) &
            ", GBR)", LogFile);

          Instruction_MemEnable <= '1';
          Instruction_ReadWrite <= ReadWrite_WRITE;
          Instruction_WordMode  <= ByteMode;

          -- Output R0 to RegB.
          RegBSel <= 0;

          -- Output RegB (Rm) to memory data bus. This will be written to memory.
          MemOutSel <= MemOut_RegB;

          -- DMAU signals for Indirect GBR addressing with displacement (Byte Mode)
          BaseSel       <=  BaseSel_GBR;
          IndexSel      <=  IndexSel_OFF8;
          OffScalarSel  <=  OffScalarSel_ONE;
          IncDecSel     <=  IncDecSel_NONE;
          DMAUOff8      <=  d_format_d;


        -- MOV.W R0, @(disp, GBR)
        -- d format
        elsif std_match(IR, MOV_W_R0_AT_DISP_GBR) then

          LogWithTime(l,
            "sh2_control.vhd: Decoded MOV.W R0, @(0x" & to_hstring(d_format_d) &
            ", GBR)", LogFile);

          Instruction_MemEnable <= '1';
          Instruction_ReadWrite <= ReadWrite_WRITE;
          Instruction_WordMode  <= WordMode;

          -- Output R0 to RegB.
          RegBSel <= 0;

          -- Output RegB (Rm) to memory data bus. This will be written to memory.
          MemOutSel <= MemOut_RegB;

          -- DMAU signals for Indirect GBR addressing with displacement (Word Mode)
          BaseSel       <=  BaseSel_GBR;
          IndexSel      <=  IndexSel_OFF8;
          OffScalarSel  <=  OffScalarSel_TWO;
          IncDecSel     <=  IncDecSel_NONE;
          DMAUOff8      <=  d_format_d;

        -- MOV.L R0, @(disp, GBR)
        -- d format
        elsif std_match(IR, MOV_L_R0_AT_DISP_GBR) then

          LogWithTime(l,
            "sh2_control.vhd: Decoded MOV.L R0, @(0x" & to_hstring(d_format_d) &
            ", GBR)", LogFile);

          Instruction_MemEnable <= '1';
          Instruction_ReadWrite <= ReadWrite_WRITE;
          Instruction_WordMode  <= LongwordMode;

          -- Output R0 to RegB.
          RegBSel <= 0;

          -- Output RegB (Rm) to memory data bus. This will be written to memory.
          MemOutSel <= MemOut_RegB;


          -- DMAU signals for Indirect GBR addressing with displacement (LongwordMode Mode)
          BaseSel      <=  BaseSel_GBR;
          IndexSel     <=  IndexSel_OFF8;
          OffScalarSel <=  OffScalarSel_FOUR;
          IncDecSel    <=  IncDecSel_NONE;
          DMAUOff8     <=  d_format_d;


        -- MOV.B @(disp, GBR), R0
        elsif std_match(IR, MOV_B_AT_DISP_GBR_R0) then
          report "Instruction: [MOV.B @(disp, GBR), R0] not implemented."
          severity ERROR;

        -- MOV.W @(disp, GBR), R0
        elsif std_match(IR, MOV_W_AT_DISP_GBR_R0) then
          report "Instruction: [MOV.W @(disp, GBR), R0] not implemented."
          severity ERROR;

        -- MOV.L @(disp, GBR), R0
        elsif std_match(IR, MOV_L_AT_DISP_GBR_R0) then
          report "Instruction: [MOV.L @(disp, GBR), R0] not implemented."
          severity ERROR;

        -- MOVA @(disp, PC), R0
        elsif std_match(IR, MOVA_AT_DISP_PC_R0) then
          report "Instruction: [MOVA @(disp, PC), R0] not implemented."
          severity ERROR;

        -- MOVT Rn
        elsif std_match(IR, MOVT_RN) then
          report "Instruction: [MOVT Rn] not implemented."
          severity ERROR;

        -- SWAP.B Rm, Rn
        elsif std_match(IR, SWAP_B_RM_RN) then
          report "Instruction: [SWAP.B Rm, Rn] not implemented."
          severity ERROR;

        -- SWAP.W Rm, Rn
        elsif std_match(IR, SWAP_W_RM_RN) then
          report "Instruction: [SWAP.W Rm, Rn] not implemented."
          severity ERROR;

        -- XTRCT Rm, Rn
        elsif std_match(IR, XTRCT_RM_RN) then
          report "Instruction: [XTRCT Rm, Rn] not implemented."
          severity ERROR;


        -- System Control Instructions ----------------------------------------

        elsif std_match(IR, CLRT) then

            LogWithTime(l, "sh2_control.vhd: Decoded CLRT", LogFile);

            Instruction_TFlagSel <= TFlagSel_CLEAR;     -- clear the T flag

        elsif std_match(IR, SETT) then

            LogWithTime(l, "sh2_control.vhd: Decoded SETT", LogFile);

            Instruction_TFlagSel <= TFlagSel_SET;       -- set the T flag

        elsif std_match(IR, STC_SYS_RN) then

            -- STC {SR, GBR, VBR}, Rn
            -- Uses bit decoding to choose the system register to store

            LogWithTime(l, "sh2_control.vhd: Decoded STC XXX, Rn", LogFile);

            RegInSel <= to_integer(unsigned(n_format_n));

            -- selects data source to store to a register through bit decoding
            RegDataInSel <= "01" & IR(5 downto 4);
            Instruction_EnableIn <= '1';

        elsif std_match(IR, STC_L_SYS_RN) then

                -- STC.L {SR, GBR, VBR}, @-Rn
                -- Uses bit decoding to choose the system register to store
                LogWithTime(l, "sh2_control.vhd: Decoded STC.L XXX, @-Rn", LogFile);

                -- Writes a byte to memory
                Instruction_MemEnable <= '1';               -- Uses memory.
                Instruction_ReadWrite <= ReadWrite_WRITE;   -- Writes.
                Instruction_WordMode  <= LongwordMode;      -- bit decode memory mode

                MemOutSel <= '1' & IR(5 downto 4); -- bit decode system register to output

                RegA1Sel               <= to_integer(unsigned(nm_format_n));  -- Output @(Rn) from RegA1 output.
                RegAxInSel             <= to_integer(unsigned(nm_format_n));  -- Store calculated address into Rn
                Instruction_RegAxStore <= '1';                                -- Enable writes to address registers.

                -- DMAU signals (for Pre-decrement indirect register addressing)
                BaseSel      <= BaseSel_REG;
                IndexSel     <= IndexSel_NONE;
                OffScalarSel <= OffScalarSel_FOUR;
                IncDecSel    <= IncDecSel_PRE_DEC;


        elsif std_match(IR, LDC_RM_SYS) then

            -- LDC Rm, GBR must actually load into the GBR in the DMAU for later instructions
            -- to work. Must modify other system control register loads to load to their actual
            -- locations as well.
            if (std_match(IR, LDC_RM_GBR)) then

                -- LDC Rm, GBR

                LogWithTime(l, 
                    "sh2_control.vhd: Decoded LDC " & to_string(slv_to_uint(m_format_m)) &
                    ", GBR", LogFile);

                RegBSel    <= to_integer(unsigned(m_format_m));
                GBRInSel   <= GBRInSel_RegB;
                Instruction_GBRWriteEn <= '1'; 

            else

                -- LDC Rm, {SR, GBR, VBR}
                -- Uses bit decoding to choose the system register to load

                LogWithTime(l, "sh2_control.vhd: Decoded LDC Rm, X", LogFile);

                RegBSel <= to_integer(unsigned(m_format_m));
                Instruction_SysRegCtrl <= SysRegCtrl_LOAD;
                SysRegSel <= IR(5 downto 4);
                SysRegSrc <= SysRegSrc_RegB;

                        
            end if;


        elsif std_match(IR, LDC_L_RM_SYS) then

            if (std_match(IR, LDC_L_AT_RM_PLUS_GBR)) then

                -- LDC.L @Rm+, GBR
                LogWithTime(l,
                    "sh2_control.vhd: Decoded LDC.L @R" & to_string(slv_to_uint(m_format_m)) & 
                    "+, GBR", LogFile);

                -- Reads a longword from memory
                Instruction_MemEnable <= '1';             -- Uses memory.
                Instruction_ReadWrite <= ReadWrite_READ;  -- Reads.
                Instruction_WordMode  <= LongwordMode;    -- bit decode memory mode

                -- Write data bus to GBR.
                GBRInSel   <= GBRInSel_DB;
                Instruction_GBRWriteEn <= '1';

                -- Read from @Rm, and save with post-incremented value
                -- RegA2Sel               <= to_integer(unsigned(m_format_m));
                RegAxInSel             <= to_integer(unsigned(m_format_m));
                Instruction_RegAxStore <= '1';

                -- DMAU signals (for post-increment indirect register addressing)
                BaseSel      <= BaseSel_REG;
                IndexSel     <= IndexSel_NONE;
                OffScalarSel <= OffScalarSel_FOUR;
                IncDecSel    <= IncDecSel_POST_INC;


            else
                
                -- LDC.L @Rm+, {SR, GBR, VBR}
                -- Uses bit decoding to choose the system register to load

                LogWithTime(l, "sh2_control.vhd: Decoded LDC.L @Rm+, X", LogFile);

                -- Reads a longword from memory
                Instruction_MemEnable <= '1';             -- Uses memory.
                Instruction_ReadWrite <= ReadWrite_READ;  -- Reads.
                Instruction_WordMode  <= LongwordMode;    -- bit decode memory mode

                -- Load into a system register
                Instruction_SysRegCtrl <= SysRegCtrl_LOAD;
                SysRegSel <= IR(5 downto 4);    -- bit decode which system register to write to
                SysRegSrc <= SysRegSrc_DB;      -- load new register value from memory

                -- Read from @Rm, and save with post-incremented value
                RegA2Sel   <= to_integer(unsigned(m_format_m));
                RegAxInSel <= to_integer(unsigned(m_format_m));
                Instruction_RegAxStore <= '1';

                -- DMAU signals (for post-increment indirect register addressing)
                BaseSel      <= BaseSel_REG;
                IndexSel     <= IndexSel_NONE;
                OffScalarSel <= OffScalarSel_FOUR;
                IncDecSel    <= IncDecSel_POST_INC;

            end if;


        elsif std_match(IR, NOP) then
            LogWithTime(l, "sh2_control.vhd: Decoded NOP", LogFile);
            null;

        elsif not is_x(IR) then
            report "Unrecognized instruction: " & to_hstring(IR);
        end if;

    end process;

    -- Register updates done on clock edges. The state machine logic is also encoded here.
    -- Currently it is a repeating cycle of:
    --  - fetch: read the current instruction from ROM at address PC
    --  - execute: latch the instruction into IR and decode it, performing the necessary
    --             computations on this clock and also outputting read/write signals for memory access
    --  - writeback: update registers with computed values (or values read from memory).
    --               Also increment the PC to advance to the next instruction.
    state_proc: process (clock, reset)
    begin
        if reset = '0' then
            state <= fetch;
            IR <= NOP;
        elsif rising_edge(clock) then
            if state = fetch then
                state <= execute;
                IR <= MemDataIn(15 downto 0); -- latch in instruction from memory
            elsif state = execute then
                state <= writeback;
            elsif state = writeback then
                state <= fetch;
            end if;
        end if;
    end process state_proc;


end dataflow;
