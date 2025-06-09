----------------------------------------------------------------------------------------------------
-- 
--  SH-2 Control signals
--
--  This package defines the control signals and constants that are used
--  internally within the SH-2 CPU to implement instructions.
--
--  Revision History
--      07 Jun 25   Zack Huang      Copied over from control unit
--      07 Jun 25   Zack Huang      Reorganized types, renamed signals for consistency
--
----------------------------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package SH2ControlSignals is

    -- We define record types for the control signals going to each component of the CPU.

    -- Memory interface control signals
    type mem_ctrl_t is record
        Enable      : std_logic;                        -- if memory needs to be accessed (read or write)
        AddrSel     : std_logic;
        ReadWrite   : std_logic;                        -- if should do memory read (0) or write (1)
        Mode        : std_logic_vector(1 downto 0);     -- if memory access should be by byte, word, or longword
        Sel         : std_logic;                        -- select memory address source, from DMAU output (0) or PMAU output (1)
        OutSel      : std_logic_vector(2 downto 0);     -- what should be output to memory
    end record;

    -- ALU control signals
    type alu_ctrl_t is record
        OpBSel      : std_logic;                        -- input mux to Operand B, either RegB (0) or Immediate (1)
        LoadA       : std_logic;                        -- determine if OperandA is loaded ('1') or zeroed ('0')
        FCmd        : std_logic_vector(3 downto 0);     -- F-Block operation
        CinCmd      : std_logic_vector(1 downto 0);     -- carry in operation
        SCmd        : std_logic_vector(2 downto 0);     -- shift operation
        ALUCmd      : std_logic_vector(1 downto 0);     -- ALU result select
        TCmpSel     : std_logic_vector(2 downto 0);     -- how to compute T from ALU status flags
        Immediate   : std_logic_vector(7 downto 0);     -- 8-bit immediate
        ImmediateMode : std_logic;                        -- Immediate extension mode
        ExtMode     : std_logic_vector(1 downto 0);     -- mode for extending register value (zero or signed)
        TFlagSel    : std_logic_vector(2 downto 0);     -- source for next value of T flag
    end record;

    -- Register array control signals
    type reg_ctrl_t is record
        DataInSel   : std_logic_vector(3 downto 0);     -- source for register input data
        EnableIn    : std_logic;                        -- if data should be written to an input register
        InSel       : integer range 15 downto 0;        -- which register to write data to
        ASel        : integer range 15 downto 0;        -- which register to read to bus A
        BSel        : integer range 15 downto 0;        -- which register to read to bus B
    end record;

    -- DMAU control signals
    type dmau_ctrl_t is record
        GBRWriteEn  : std_logic;
        Off4        : std_logic_vector(3 downto 0);
        Off8        : std_logic_vector(7 downto 0);
        BaseSel     : std_logic_vector(1 downto 0);
        IndexSel    : std_logic_vector(1 downto 0);
        OffScalarSel: std_logic_vector(1 downto 0);
        IncDecSel   : std_logic_vector(1 downto 0);
        AxInSel     : integer range 15 downto 0;        -- which address register to write to
        AxStore     : std_logic;                        -- if data should be written to the address register
        A1Sel       : integer range 15 downto 0;        -- which register to read to address bus 1
        A2Sel       : integer range 15 downto 0;        -- which register to read to address bus 2
    end record;

    -- PMAU control signals
    type pmau_ctrl_t is record
        PCAddrMode  : std_logic_vector(2 downto 0);     -- What PC addressing mode is desired
        Off8        : std_logic_vector(7 downto 0);     -- 8-bit offset for relative addressing
        Off12       : std_logic_vector(11 downto 0);    -- 12-bit offset for relative addressing
        PCWriteCtrl : std_logic_vector(1 downto 0);     -- What to write to the PC register
        DelayedBranchTaken  : std_logic;                -- whether the delayed branch is taken
    end record;

    -- System control signals
    type sys_ctrl_t is record
        PRWriteEn   : std_logic;                        -- Enable writing to PR
        RegCtrl     : std_logic_vector(1 downto 0);
        RegSel      : std_logic_vector(2 downto 0);
        RegSrc      : std_logic_vector(1 downto 0);
    end record;

    type ctrl_t is record
        MemCtrl      : mem_ctrl_t;
        ALUCtrl      : alu_ctrl_t;
        REGCtrl      : reg_ctrl_t;
        PMAUCtrl     : pmau_ctrl_t;
        DMAUCtrl     : dmau_ctrl_t;
        SysCtrl      : sys_ctrl_t;
        BranchTaken  : std_logic;
        DBranchTaken : std_logic;
    end record;


    -- Internal control signals for controlling muxes within the CPU

    -- Constants for RegDataInSel in sh2_cpu architecture. Used to determine what to input to the
    -- register array's RegDataIn.
    constant RegDataIn_ALUResult        : std_logic_vector(3 downto 0) := "0000"; -- ALU result
    constant RegDataIn_Immediate        : std_logic_vector(3 downto 0) := "0001"; -- Sign-extended 8-bit immediate
    constant RegDataIn_RegA             : std_logic_vector(3 downto 0) := "0010"; -- RegA output
    constant RegDataIn_RegB             : std_logic_vector(3 downto 0) := "0011"; -- RegB output
    constant RegDataIn_SysReg           : std_logic_vector(3 downto 0) := "0100"; -- System register
    constant RegDataIn_RegA_SwapB       : std_logic_vector(3 downto 0) := "0111"; -- RegA output with low two bytes swapped
    constant RegDataIn_RegA_SwapW       : std_logic_vector(3 downto 0) := "1000"; -- RegA output with low/high words swapped
    constant RegDataIn_RegB_RegA_Center : std_logic_vector(3 downto 0) := "1001"; -- Low word of RegB and high word of RegA
    constant RegDataIn_SR_TBit          : std_logic_vector(3 downto 0) := "1010"; -- T-bit (in the LSB)
    constant RegDataIn_PR               : std_logic_vector(3 downto 0) := "1011"; -- Procedure Register (PR)
    constant RegDataIn_DB               : std_logic_vector(3 downto 0) := "1100"; -- Data Bus (DB) value
    constant RegDataIn_Ext              : std_logic_vector(3 downto 0) := "1101"; -- Sign/zero extended register values.

    -- Constants for ExtMode in sh2_cpu architecture. Used to determine how RegB will be extended
    -- to a long-word.
    --
    -- WARNING: Changing these will break bit decoding of instructions.
    --
    constant Ext_SignB_RegA : std_logic_vector(1 downto 0) := "10";  -- Sign extend low byte of Reg A
    constant Ext_SignW_RegA : std_logic_vector(1 downto 0) := "11";  -- Sign extend low word of Reg A
    constant Ext_ZeroB_RegA : std_logic_vector(1 downto 0) := "00";  -- Zero extend low byte of Reg A
    constant Ext_ZeroW_RegA : std_logic_vector(1 downto 0) := "01";  -- Zero extend low word of Reg A


    -- Constants for ReadWrite used in memory_tx architecture. Determines whether to read or
    -- write to memory.
    --
    constant ReadWrite_Read     : std_logic := '0';
    constant ReadWrite_Write    : std_logic := '1';

    -- Constants for selecting what to output to MemDataOut in memory_tx entity in the sh2_cpu 
    -- entity.
    --
    constant MemOut_RegA    : std_logic_vector(2 downto 0) := "000"; -- Output RegA to data bus
    constant MemOut_RegB    : std_logic_vector(2 downto 0) := "001"; -- Output RegB to data bus
    constant MemOut_SysReg  : std_logic_vector(2 downto 0) := "010"; -- Output a system register to data bus


    constant ALUOpB_RegB    : std_logic := '0'; -- Use RegB as B input to ALU. 
    constant ALUOpB_Imm     : std_logic := '1'; -- Use immediate as B input to ALU

    constant TFlagSel_T         : std_logic_vector(2 downto 0) := "000";    -- Have T retain its value
    constant TFlagSel_Zero      : std_logic_vector(2 downto 0) := "001";    -- Set T to the ALU zero flag
    constant TFlagSel_Carry     : std_logic_vector(2 downto 0) := "010";    -- Set T to the ALU carry flag
    constant TFlagSel_Overflow  : std_logic_vector(2 downto 0) := "011";    -- Set T to the ALU overflow flag
    constant TFlagSel_Clear     : std_logic_vector(2 downto 0) := "100";    -- clear T (to 0)
    constant TFlagSel_Set       : std_logic_vector(2 downto 0) := "101";    -- set T (to 1)
    constant TFlagSel_Cmp       : std_logic_vector(2 downto 0) := "110";    -- set T to a value computed from
                                                                            -- the ALU flags

    -- WARNING: Changing these will break bit decoding of instuctions.

    -- How to calculate the T-bit in the 
    constant TCmp_EQ            : std_logic_vector(2 downto 0) := "000"; -- 
    constant TCmp_HS            : std_logic_vector(2 downto 0) := "010";
    constant TCmp_GE            : std_logic_vector(2 downto 0) := "011";
    constant TCmp_HI            : std_logic_vector(2 downto 0) := "110";
    constant TCmp_GT            : std_logic_vector(2 downto 0) := "111";
    constant TCmp_STR           : std_logic_vector(2 downto 0) := "100";

    constant MemSel_ROM         : std_logic := '1';
    constant MemSel_RAM         : std_logic := '0';

    constant MemAddrSel_PMAU    : std_logic := '0';
    constant MemAddrSel_DMAU    : std_logic := '1';

    constant SysRegCtrl_None    : std_logic_vector(1 downto 0) := "00";     -- do nothing with system register
    constant SysRegCtrl_Load    : std_logic_vector(1 downto 0) := "01";     -- load system register with new value
    constant SysRegCtrl_Clear   : std_logic_vector(1 downto 0) := "10";     -- clear system register

    constant SysRegSrc_RegB     : std_logic_vector(1 downto 0) := "00";     -- load system register from register bus B
    constant SysRegSrc_DB       : std_logic_vector(1 downto 0) := "01";     -- load system register from data bus
    constant SysRegSrc_PC       : std_logic_vector(1 downto 0) := "10";     -- load system register from PC

    -- WARNING: Changing these will break bit decoding of instuctions.
    constant SysRegSel_System   : std_logic_vector(2 downto 0) := "0--";
    constant SysRegSel_SR       : std_logic_vector(2 downto 0) := "000";
    constant SysRegSel_GBR      : std_logic_vector(2 downto 0) := "001";
    constant SysRegSel_VBR      : std_logic_vector(2 downto 0) := "010";
    constant SysRegSel_Control  : std_logic_vector(2 downto 0) := "1--";
    constant SysRegSel_MACH     : std_logic_vector(2 downto 0) := "100";
    constant SysRegSel_MACL     : std_logic_vector(2 downto 0) := "101";
    constant SysRegSel_PR       : std_logic_vector(2 downto 0) := "110";

    -- Whether to sign or zero extend the immediate into a 32-bit word.
    constant ImmediateMode_Sign     : std_logic := '0';
    constant ImmediateMode_Zero     : std_logic := '1';


end package SH2ControlSignals;
