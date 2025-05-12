
--
--  Control Unit
--
--
--  Revision History:
--     06 May 25  Zack Huang        Initial revision
--     07 May 25  Chris Miranda     Initial implentation of MOV and branch 
--                                  instruction decoding.
--     10 May 25  Zack Huang        Implementing ALU instruction
----------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package SH2InstructionEncodings is

  -- Data Transfer Instruction:
  constant MOV_IMM_RN     : std_logic_vector(15 downto 0) := "1110------------";
  constant MOV_RM_RN      : std_logic_vector(15 downto 0) := "0110--------0011";
  constant MOV_L_RM_AT_RN : std_logic_vector(15 downto 0) := "0010--------00--";

  -- Arithmetic Instructions:
  constant ADD_RM_RN    : std_logic_vector(15 downto 0) := "0011--------11--";
  constant ADD_IMM_RN   : std_logic_vector(15 downto 0) := "0111------------";
  constant SUB_RM_RN    : std_logic_vector(15 downto 0) := "0011--------10--";
  constant NEG_RM_RN    : std_logic_vector(15 downto 0) := "0110--------101-";

  -- Logical Operations:
  constant LOGIC_RM_RN      : std_logic_vector(15 downto 0) := "0010--------10--";  -- AND, TST, OR, XOR
  constant LOGIC_IMM_R0     : std_logic_vector(15 downto 0) := "110010----------";  -- AND, TST, OR, XOR
  constant NOT_RM_RN        : std_logic_vector(15 downto 0) := "0110--------0111";  -- NOT

  -- Shift Instruction:
  constant SHIFT_RN         : std_logic_vector(15 downto 0) := "0100----00-00-0-";
  -- Branch Instructions:
  -- System Control:
  constant NOP          : std_logic_vector(15 downto 0) := "0000000000001001";
  constant CLRT         : std_logic_vector(15 downto 0) := "0000000000001000";
  constant SETT         : std_logic_vector(15 downto 0) := "0000000000011000";

  constant STC_SYS_RN   : std_logic_vector(15 downto 0) := "0000----00--0010";
  constant LDC_RM_SYS   : std_logic_vector(15 downto 0) := "0100----00--1110";

end package SH2InstructionEncodings;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package SH2ControlConstants is
    -- Internal control signals for controlling muxes within the CPU
    constant RegDataIn_ALUResult : std_logic_vector(2 downto 0) := "000";
    constant RegDataIn_Immediate : std_logic_vector(2 downto 0) := "001";
    constant RegDataIn_RegA      : std_logic_vector(2 downto 0) := "010";
    constant RegDataIn_RegB      : std_logic_vector(2 downto 0) := "011";
    constant RegDataIn_SR        : std_logic_vector(2 downto 0) := "100";
    constant RegDataIn_GBR       : std_logic_vector(2 downto 0) := "101";
    constant RegDataIn_VBR       : std_logic_vector(2 downto 0) := "110";

    constant ReadWrite_READ     : std_logic := '0';
    constant ReadWrite_WRITE    : std_logic := '1';

    constant MemOut_RegA    : std_logic_vector(2 downto 0) := "000";
    constant MemOut_RegB    : std_logic_vector(2 downto 0) := "001";
    constant MemOut_SR      : std_logic_vector(2 downto 0) := "010";
    constant MemOut_GBR     : std_logic_vector(2 downto 0) := "011";
    constant MemOut_VBR     : std_logic_vector(2 downto 0) := "100";
    constant MemOut_PR      : std_logic_vector(2 downto 0) := "101";
    constant MemOut_PC      : std_logic_vector(2 downto 0) := "110";

    constant ALUOpB_RegB    : std_logic := '0';
    constant ALUOpB_Imm     : std_logic := '1';

    constant TFlagSel_T         : std_logic_vector(2 downto 0) := "000";
    constant TFlagSel_Zero      : std_logic_vector(2 downto 0) := "001";
    constant TFlagSel_Carry     : std_logic_vector(2 downto 0) := "010";
    constant TFlagSel_Overflow  : std_logic_vector(2 downto 0) := "011";
    constant TFlagSel_SET       : std_logic_vector(2 downto 0) := "100";
    constant TFlagSel_CLEAR     : std_logic_vector(2 downto 0) := "101";

    constant MemSel_ROM     : std_logic := '1';
    constant MemSel_RAM     : std_logic := '0';

    constant SysRegCtrl_NONE    : std_logic := '0';
    constant SysRegCtrl_LOAD    : std_logic := '1';

    constant SysRegSel_SR   : std_logic_vector(1 downto 0) := "00";
    constant SysRegSel_GBR  : std_logic_vector(1 downto 0) := "01";
    constant SysRegSel_VBR  : std_logic_vector(1 downto 0) := "10";
    constant SysRegSel_PR   : std_logic_vector(1 downto 0) := "11";

    constant ImmediateMode_SIGN     : std_logic := '0';
    constant ImmediateMode_ZERO     : std_logic := '1';

end package SH2ControlConstants;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.SH2PmauConstants.all;
use work.SH2DmauConstants.all;
use work.MemoryInterfaceConstants.all;
use work.SH2InstructionEncodings.all;
use work.SH2ControlConstants.all;
use work.SH2ALUConstants.all;


entity  SH2Control  is

    port (
        DB          : in  std_logic_vector(31 downto 0);    -- data read from memory
        clock       : in  std_logic;                        -- system clock
        reset       : in  std_logic;                        -- system reset (active low, async)

        -- control signals to control memory interface
        MemEnable   : out std_logic;                        -- if memory needs to be accessed (read or write)
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
        TIn         : out std_logic;                        -- T bit from status register
        LoadA       : out std_logic;                        -- determine if OperandA is loaded ('1') or zeroed ('0')
        FCmd        : out std_logic_vector(3 downto 0);     -- F-Block operation
        CinCmd      : out std_logic_vector(1 downto 0);     -- carry in operation
        SCmd        : out std_logic_vector(2 downto 0);     -- shift operation
        ALUCmd      : out std_logic_vector(1 downto 0);     -- ALU result select

        TSel        : out std_logic_vector(2 downto 0);     -- if T should be updated to a new value (T/C/V/0/1)

        -- register array control signals
        RegDataInSel: out std_logic_vector(2 downto 0);     -- source for register input data
        DataIn      : out std_logic_vector(31 downto 0);    -- data to write to a register
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
        SysRegSel       : out std_logic_vector(1 downto 0)
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
  alias n_format_n : std_logic_vector(3 downto 0) is IR(11 downto 8);

  alias m_format_m : std_logic_vector(3 downto 0) is IR(11 downto 8);

  alias nm_format_n : std_logic_vector(3 downto 0) is IR(11 downto 8);
  alias nm_format_m : std_logic_vector(3 downto 0) is IR(7 downto 4);

  alias md_format_m : std_logic_vector(3 downto 0) is IR(7 downto 4);
  alias md_format_d : std_logic_vector(3 downto 0) is IR(3 downto 0);

  alias nd4_format_n : std_logic_vector(3 downto 0) is IR(7 downto 4);
  alias nd4_format_d : std_logic_vector(3 downto 0) is IR(3 downto 0);

  alias nmd_format_n : std_logic_vector(3 downto 0) is IR(11 downto 8);
  alias nmd_format_m : std_logic_vector(3 downto 0) is IR(7 downto 4);
  alias nmd_format_d : std_logic_vector(3 downto 0) is IR(3 downto 0);

  alias d_format_d : std_logic_vector(7 downto 0) is IR(7 downto 0);

  alias d12_format_d : std_logic_vector(11 downto 0) is IR(11 downto 0);

  alias nd8_format_n : std_logic_vector(3 downto 0) is IR(11 downto 8);
  alias nd8_format_d : std_logic_vector(7 downto 0) is IR(7 downto 0);

  alias i_format_i : std_logic_vector(7 downto 0) is IR(7 downto 0);

  alias ni_format_n : std_logic_vector(3 downto 0) is IR(11 downto 8);
  alias ni_format_i : std_logic_vector(7 downto 0) is IR(7 downto 0);

  -- Internal signals computed combinatorially to memory signals can
  -- be output on the correct clock.
  signal Instruction_MemEnable : std_logic;
  signal Instruction_ReadWrite : std_logic;
  signal Instruction_WordMode : std_logic_vector(1 downto 0);

  signal Instruction_EnableIn  : std_logic;
  signal Instruction_PCAddrMode : std_logic_vector(2 downto 0);

  signal Instruction_TFlagSel    : std_logic_vector(2 downto 0);

begin

    -- Outputs that change based on the CPU state
    PCAddrMode <= Instruction_PCAddrMode when state = writeback else PCAddrMode_HOLD;
    MemEnable  <= Instruction_MemEnable when state = execute else
                  '1' when state = fetch else
                  '0' when state = writeback;
    ReadWrite <= Instruction_ReadWrite when state = execute else
                 '0' when state = fetch else
                 'X' when state = writeback;

    MemMode <= Instruction_WordMode when state = execute else
               WordMode when state = fetch else
               "XX";

    MemSel <= MemSel_RAM when state = execute else
              MemSel_ROM when state = fetch else
              'X';

    -- Only modify registers during writeback
    EnableIn <= Instruction_EnableIn when state = writeback else
                '0';

    TFlagSel <= Instruction_TFlagSel when state = writeback else
                TFlagSel_T;

    decode_proc: process (IR)
    begin
        -- Default flag values (shouldn't change CPU state)

        -- Not accessing memory
        Instruction_MemEnable <= '0';
        Instruction_ReadWrite <= 'X';
        Instruction_WordMode <= "XX";
        MemOutSel <= "XXX";

        Instruction_EnableIn <= '0';                -- Disable Reg Array
        Instruction_PCAddrMode <= PCAddrMode_INC;   -- Increment PC
        Instruction_TFlagSel <= TFlagSel_T;         -- Keep T flag the same
        GBRWriteEn <= '0';                          -- keep GBR
        PRWriteEn <= '0';                           -- keep PR

        SysRegCtrl <= SysRegCtrl_NONE;

        ImmediateMode <= ImmediateMode_SIGN;

        if std_match(IR, ADD_RM_RN) then
            -- report "Instruction: ADD(C/V) Rm, Rn";

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));
            RegBSel <= to_integer(unsigned(nm_format_m));

            RegInSel <= to_integer(unsigned(nm_format_n));
            RegDataInSel <= RegDataIn_ALUResult;
            Instruction_EnableIn <= '1';

            -- Bit-decoding T flag select (None, Carry, Overflow)
            Instruction_TFlagSel <= '0' & IR(1 downto 0);

            -- ALU signals
            ALUOpBSel <= ALUOpB_RegB;
            LoadA <= '1';
            FCmd <= FCmd_B;
            -- Bit-decode carry in value
            if IR(1 downto 0) = "10" then
                CinCmd <= CinCmd_CIN;   -- ADDC
            else
                CinCmd <= CinCmd_ZERO;  -- ADD, ADDV
            end if;
            SCmd <= "XXX";
            ALUCmd <= ALUCmd_ADDER;

        elsif std_match(IR, SUB_RM_RN) then
            -- report "Instruction: SUB(C/V) Rm, Rn";

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));
            RegBSel <= to_integer(unsigned(nm_format_m));

            RegInSel <= to_integer(unsigned(nm_format_n));
            RegDataInSel <= RegDataIn_ALUResult;
            Instruction_EnableIn <= '1';

            -- Bit-decoding T flag select (None, Carry, Overflow)
            Instruction_TFlagSel <= '0' & IR(1 downto 0);

            -- ALU signals
            ALUOpBSel <= ALUOpB_RegB;
            LoadA <= '1';
            FCmd <= FCmd_BNOT;
            -- Bit-decode carry in value
            if IR(1 downto 0) = "10" then
                CinCmd <= CinCmd_CINBAR;    -- SUBC
            else
                CinCmd <= CinCmd_ONE;       -- SUB, SUBV
            end if;
            SCmd <= "XXX";
            ALUCmd <= ALUCmd_ADDER;

        elsif std_match(IR, NEG_RM_RN) then
            -- report "Instruction: NEG(C) Rm, Rn";

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));
            RegBSel <= to_integer(unsigned(nm_format_m));

            RegInSel <= to_integer(unsigned(nm_format_n));
            RegDataInSel <= RegDataIn_ALUResult;
            Instruction_EnableIn <= '1';

            -- Bit-decoding T flag select
            if IR(0) = '0' then
                Instruction_TFlagSel <= TFlagSel_Carry;
            else
                Instruction_TFlagSel <= TFlagSel_T;
            end if;

            -- ALU signals
            ALUOpBSel <= ALUOpB_RegB;
            LoadA <= '0';
            FCmd <= FCmd_BNOT;
            -- Bit-decode carry in value
            if IR(0) = '0' then
                CinCmd <= CinCmd_CINBAR;    -- NEGC
            else
                CinCmd <= CinCmd_ONE;       -- NEG
            end if;
            SCmd <= "XXX";
            ALUCmd <= ALUCmd_ADDER;

        elsif std_match(IR, ADD_IMM_RN) then
            -- report "Instruction: ADD #imm, Rn";

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));

            RegInSel <= to_integer(unsigned(nm_format_n));
            RegDataInSel <= RegDataIn_ALUResult;
            Instruction_EnableIn <= '1';
            Immediate <= ni_format_i;

            -- ALU signals
            ALUOpBSel <= ALUOpB_Imm;
            LoadA <= '1';
            FCmd <= FCmd_B;
            CinCmd <= CinCmd_ZERO;
            SCmd <= "XXX";
            ALUCmd <= ALUCmd_ADDER;

        elsif std_match(IR, LOGIC_RM_RN) then
            -- {AND, TST, OR, XOR} Rm, Rn

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));
            RegBSel <= to_integer(unsigned(nm_format_m));

            RegInSel <= to_integer(unsigned(nm_format_n));
            RegDataInSel <= RegDataIn_ALUResult;
            Instruction_EnableIn <= IR(1) or IR(0);   -- exclude TST

            -- Enable TFlagSel for TST
            Instruction_TFlagSel <= TFlagSel_Zero when IR(1 downto 0) = "00" else TFlagSel_T;

            -- ALU signals
            ALUOpBSel <= ALUOpB_RegB;
            LoadA <= '1';
            FCmd <= FCmd_AND when IR(1) = '0' else
                    FCmd_XOR when IR(1 downto 0) = "10" else
                    FCmd_OR;
            CinCmd <= CinCmd_ZERO;
            SCmd <= "XXX";
            ALUCmd <= ALUCmd_FBLOCK;

        elsif std_match(IR, LOGIC_IMM_R0) then
            -- {AND, TST, OR, XOR} immediate, R0

            -- Register array signals
            RegASel <= 0;

            RegInSel <= 0;
            RegDataInSel <= RegDataIn_ALUResult;
            Instruction_EnableIn <= IR(9) or IR(8);   -- exclude TST
            Immediate <= i_format_i;
            ImmediateMode <= ImmediateMode_ZERO;

            -- Enable TFlagSel for TST
            Instruction_TFlagSel <= TFlagSel_Zero when IR(9 downto 8) = "00" else TFlagSel_T;

            -- ALU signals
            ALUOpBSel <= ALUOpB_Imm;
            LoadA <= '1';
            FCmd <= FCmd_AND when IR(9) = '0' else
                    FCmd_XOR when IR(9 downto 8) = "10" else
                    FCmd_OR;
            CinCmd <= CinCmd_ZERO;
            SCmd <= "XXX";
            ALUCmd <= ALUCmd_FBLOCK;

        elsif std_match(IR, NOT_RM_RN) then
            -- NOT Rm, Rn

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));
            RegBSel <= to_integer(unsigned(nm_format_m));

            RegInSel <= to_integer(unsigned(nm_format_n));
            RegDataInSel <= RegDataIn_ALUResult;
            Instruction_EnableIn <= '1';

            -- ALU signals
            ALUOpBSel <= ALUOpB_RegB;
            LoadA <= '1';
            FCmd <= FCmd_BNOT;
            CinCmd <= CinCmd_ZERO;
            SCmd <= "XXX";
            ALUCmd <= ALUCmd_FBLOCK;

        elsif std_match(IR, SHIFT_RN) then
            -- {ROTL, ROTR, ROTCL, ROTCR, SHAL, SHAR, SHLL, SHLR} Rn

            -- Register array signals
            RegASel <= to_integer(unsigned(n_format_n));
            RegInSel <= to_integer(unsigned(n_format_n));
            RegDataInSel <= RegDataIn_ALUResult;
            Instruction_EnableIn <= '1';

            Instruction_TFlagSel <= TFlagSel_Carry;

            -- ALU signals
            ALUOpBSel <= ALUOpB_RegB;
            LoadA <= '1';
            FCmd <= "XXXX";
            CinCmd <= CinCmd_CIN when (IR(5) and IR(2)) = '1' else CinCmd_ZERO;     -- ROTCL, ROTCR
            SCmd <= IR(0) & IR(2) & IR(5);  -- bit-decode shift operation
            ALUCmd <= ALUCmd_SHIFT;

        elsif std_match(IR, MOV_RM_RN) then
            -- report "Instruction: MOV Rm, Rn";
            RegBSel <= to_integer(unsigned(nm_format_m));
            RegInSel <= to_integer(unsigned(nm_format_n));
            RegDataInSel <= RegDataIn_RegB;
            Instruction_EnableIn <= '1';

        elsif std_match(IR, MOV_IMM_RN) then
            -- report "Instruction: MOV #imm, Rn";
            RegInSel <= to_integer(unsigned(ni_format_n));
            RegDataInSel <= RegDataIn_Immediate;
            Instruction_EnableIn <= '1';
            Immediate <= ni_format_i;

        elsif std_match(IR, MOV_L_RM_AT_RN) then
            -- report "Instruction: MOV RM, @Rn";

            -- Writes to memory
            Instruction_MemEnable <= '1';
            Instruction_ReadWrite <= ReadWrite_WRITE;
            Instruction_WordMode <= IR(1 downto 0);     -- bit-decode word mode
            MemOutSel <= MemOut_RegB;

            RegBSel <= to_integer(unsigned(nm_format_m));
            RegA1Sel <= to_integer(unsigned(nm_format_n));

            -- DMAU signals
            BaseSel <= BaseSel_REG;
            IndexSel <= IndexSel_NONE;
            OffScalarSel <= OffScalarSel_ONE;
            IncDecSel <= IncDecSel_NONE;

        elsif std_match(IR, CLRT) then
            -- report "Instruction: NOP";
            Instruction_TFlagSel <= TFlagSel_CLEAR;
        elsif std_match(IR, SETT) then
            -- report "Instruction: NOP";
            Instruction_TFlagSel <= TFlagSel_SET;
        elsif std_match(IR, STC_SYS_RN) then
            RegInSel <= to_integer(unsigned(n_format_n));
            RegDataInSel <= '1' & IR(5 downto 4);
            Instruction_EnableIn <= '1';
        elsif std_match(IR, LDC_RM_SYS) then
            RegBSel <= to_integer(unsigned(m_format_m));
            SysRegCtrl <= SysRegCtrl_LOAD;
            SysRegSel <= IR(5 downto 4);
        elsif std_match(IR, NOP) then
            -- report "Instruction: NOP";
            null;
        elsif not is_x(IR) then
            report "Unrecognized instruction: " & to_hstring(IR);
        end if;
    end process;

    -- Register updates done on clock edges
    state_proc: process (clock, reset)
    begin
        if reset = '0' then
            state <= fetch;
            IR <= NOP;
        elsif rising_edge(clock) then
            if state = fetch then
                state <= execute;
                IR <= DB(15 downto 0); -- latch in instruction from memory
            elsif state = execute then
                -- report "Decoding instruction: " & to_hstring(IR);
                state <= writeback;
            elsif state = writeback then
                state <= fetch;
            end if;
        end if;
    end process state_proc;


end dataflow;
