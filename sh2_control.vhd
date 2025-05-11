
--
--  Control Unit
--
--
--  Revision History:
--     06 May 25  Zack Huang        Initial revision
--     07 May 25  Chris Miranda     Initial implentation of MOV and branch 
--                                  instruction decoding.
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
  constant ADD_RM_RN : std_logic_vector(15 downto 0) := "0011--------1100";

  -- Logical Operations:
  -- Shift Instruction:
  -- Branch Instructions:
  -- System Control:
  constant NOP : std_logic_vector(15 downto 0) := "0000000000001001";

end package SH2InstructionEncodings;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package SH2ControlConstants is
    -- Internal control signals for controlling muxes within the CPU
    constant RegDataIn_ALUResult : std_logic_vector(1 downto 0) := "00";
    constant RegDataIn_Immediate : std_logic_vector(1 downto 0) := "01";
    constant RegDataIn_RegA      : std_logic_vector(1 downto 0) := "10";
    constant RegDataIn_RegB      : std_logic_vector(1 downto 0) := "11";

    constant ReadWrite_READ : std_logic := '0';
    constant ReadWrite_WRITE : std_logic := '1';

    constant MemOut_RegA    : std_logic_vector(2 downto 0) := "000";
    constant MemOut_RegB    : std_logic_vector(2 downto 0) := "001";
    constant MemOut_SR      : std_logic_vector(2 downto 0) := "010";
    constant MemOut_GBR     : std_logic_vector(2 downto 0) := "011";
    constant MemOut_VBR     : std_logic_vector(2 downto 0) := "100";
    constant MemOut_PR      : std_logic_vector(2 downto 0) := "101";
    constant MemOut_PC      : std_logic_vector(2 downto 0) := "110";

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
        MemOutSel   : out std_logic_vector(2 downto 0);     -- what should be output to memory

        -- ALU control signals
        OperandA    : out std_logic_vector(31 downto 0);    -- first operand
        OperandB    : out std_logic_vector(31 downto 0);    -- second operand
        TIn         : out std_logic;                        -- T bit from status register
        LoadA       : out std_logic;                        -- determine if OperandA is loaded ('1') or zeroed ('0')
        FCmd        : out std_logic_vector(3 downto 0);     -- F-Block operation
        CinCmd      : out std_logic_vector(1 downto 0);     -- carry in operation
        SCmd        : out std_logic_vector(2 downto 0);     -- shift operation
        ALUCmd      : out std_logic_vector(1 downto 0);     -- ALU result select

        TSel        : out std_logic_vector(2 downto 0);     -- if T should be updated to a new value (T/C/V/0/1)

        -- register array control signals
        RegDataInSel: out std_logic_vector(1 downto 0);     -- source for register input data
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
        DataRegIdx      : out integer range 15 downto 0;
        GBRWriteEn      : out std_logic;
        DMAUOff4        : out std_logic_vector(3 downto 0);
        DMAUOff8        : out std_logic_vector(7 downto 0);
        BaseSel         : out std_logic_vector(1 downto 0);
        IndexSel        : out std_logic_vector(1 downto 0);
        OffScalarSel    : out std_logic_vector(1 downto 0);
        IncDecSel       : out std_logic_vector(1 downto 0);

        -- PMAU signals
        ProgRegIdx      : out integer range 15 downto 0;
        PCAddrMode      : out std_logic_vector(2 downto 0);
        PRWriteEn       : out std_logic;
        PMAUOff8        : out std_logic_vector(7 downto 0);
        PMAUOff12       : out std_logic_vector(11 downto 0)
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
  alias n_format_n : std_logic_vector(3 downto 0) is IR(7 downto 4);

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

begin

    -- Only update PC before next fetch cycle
    PCAddrMode <= Instruction_PCAddrMode when state = writeback else PCAddrMode_HOLD;

    decode_proc: process (IR)
    begin
        if std_match(IR, NOP) then
            report "Instruction: NOP";

            -- Does not access memory
            Instruction_MemEnable <= '0';
            Instruction_ReadWrite <= 'X';
            Instruction_WordMode <= "XX";
            MemOutSel <= "XXX";

            -- PMAU signals
            Instruction_PCAddrMode <= PCAddrMode_INC;
        elsif std_match(IR, ADD_RM_RN) then
            report "Instruction: ADD";

            -- Does not access memory
            Instruction_MemEnable <= '0';
            Instruction_ReadWrite <= 'X';
            Instruction_WordMode <= "XX";
            MemOutSel <= "XXX";

            -- Register array signals
            RegASel <= to_integer(unsigned(nm_format_n));
            RegBSel <= to_integer(unsigned(nm_format_m));

            RegInSel <= to_integer(unsigned(nm_format_n));
            RegDataInSel <= RegDataIn_ALUResult;
            Instruction_EnableIn <= '1';

            -- ALU signals
            LoadA <= '1';
            FCmd <= FCmd_B;
            CinCmd <= CinCmd_ZERO;
            SCmd <= "XXX";
            ALUCmd <= ALUCmd_ADDER;

            -- PMAU signals
            Instruction_PCAddrMode <= PCAddrMode_INC;

        elsif std_match(IR, MOV_RM_RN) then

            RegBSel <= to_integer(unsigned(nm_format_m));
            RegInSel <= to_integer(unsigned(nm_format_n));
            RegDataInSel <= RegDataIn_RegB;
            Instruction_EnableIn <= '1';

            -- PMAU signals
            Instruction_PCAddrMode <= PCAddrMode_INC;

        elsif std_match(IR, MOV_IMM_RN) then
            report "Instruction: MOV #imm, Rn";
            RegInSel <= to_integer(unsigned(ni_format_n));
            RegDataInSel <= RegDataIn_Immediate;
            Instruction_EnableIn <= '1';
            Immediate <= ni_format_i;

            -- PMAU signals
            Instruction_PCAddrMode <= PCAddrMode_INC;

        elsif std_match(IR, MOV_L_RM_AT_RN) then
            report "Instruction: MOV RM, @Rn";

            -- Writes to memory
            Instruction_MemEnable <= '1';
            Instruction_ReadWrite <= ReadWrite_WRITE;
            Instruction_WordMode <= IR(1 downto 0);     -- happens to work for this instruction
            MemOutSel <= MemOut_RegB;

            RegBSel <= to_integer(unsigned(nm_format_m));
            RegA1Sel <= to_integer(unsigned(nm_format_n));

            -- DMAU signals
            GBRWriteEn <= '0';
            BaseSel <= BaseSel_REG;
            IndexSel <= IndexSel_NONE;
            OffScalarSel <= OffScalarSel_ONE;
            IncDecSel <= IncDecSel_NONE;

            -- PMAU signals
            Instruction_PCAddrMode <= PCAddrMode_INC;
        else
            report "Unrecognized instruction: " & to_hstring(IR);
            -- Does not access memory
            Instruction_MemEnable <= '0';
            Instruction_ReadWrite <= 'X';
            Instruction_WordMode <= "XX";
            MemOutSel <= "XXX";

            -- PMAU signals
            Instruction_PCAddrMode <= PCAddrMode_INC;
        end if;
    end process;

    -- outputs based on the current CPU state
    output_proc: process(state, clock)
    begin
        if state = fetch then
            -- Fetch an instruction word from ROM
            -- TODO: replace magic numbers with constants
            MemEnable <= '1';
            MemSel <= '1';          -- ROM
            ReadWrite <= '0';       -- read
            MemMode <= WordMode;    -- word
            EnableIn <= '0';
        elsif state = execute then
            MemSel <= '0';          -- RAM
            MemEnable <= Instruction_MemEnable;
            ReadWrite <= Instruction_ReadWrite;
            MemMode <= Instruction_WordMode;
            EnableIn <= '0';
        elsif state = writeback then
            -- disable memory
            MemEnable <= '0';
            ReadWrite <= 'X';
            MemMode <= "XX";

            -- perform register writeback if necessary
            EnableIn <= Instruction_EnableIn;
        end if;
    end process output_proc;

    -- Register updates done on clock edges
    state_proc: process (clock, reset)
    begin
        if reset = '0' then
            state <= fetch;
            IR <= (others => '0');
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
