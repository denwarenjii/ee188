----------------------------------------------------------------------------------------------------
--                                                                                                 -
--  Control Unit                                                                                   -
--                                                                                                 -
--                                                                                                 -
--  Revision History:                                                                              -
--     06 May 25  Zack Huang        Initial revision                                               -
--     07 May 25  Chris Miranda     Initial implentation of MOV and branch instruction decoding.   - 
--     10 May 25  Zack Huang        Implementing ALU instruction                                   -
--     14 May 25  Chris M.          Formatting.                                                    -
--     16 May 25  Zack Huang        Documentation, renaming signals                                -
--     25 May 25  Zack Huang        Finishing ALU and system instructions                          -
--     26 May 25  Chris M.          Add T flag as input to control unit. Add delay slot simulation -
--                                  signals.                                                       -
--                                                                                                 -
--     29 May 25  Chris M.          Add PCWriteCtrl and DelayedBranchTaken signals to control unit -
--                                  output.                                                        -
--     07 Jun 25  Zack Huang        Re-organized control signals and constants                     -
--                                                                                                 -
-- Notes:                                                                                          -
--  - When reading/writing to registers, RegB is always Rm and RegA is always Rn                   -
--  - When reading/writing to addresses (in registers), RegA2 is always @(Rm) and                  -
--    RegA2 is always @(Rn).                                                                       -
--                                                                                                 -
-- TODO:                                                                                           -
--  - Remove redundant assignment of default signals.                                              -
--                                                                                                 -
--  - Generate DMAU signals with vectors.                                                          -
--  - Document register output conventions.                                                        -
--  - Document bit decoding.                                                                       -
--  - Add short instruction operation to std_match case.                                           -
--  - Use slv_to_uint more.                                                                        -
--  - DEAL WITH DOUBLE DELAYED BRANCHES - NOT POSSIBLE                                             -
----------------------------------------------------------------------------------------------------

library ieee;
library std;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use std.textio.all;

use work.SH2PmauConstants.all;
use work.SH2DmauConstants.all;
use work.MemoryInterfaceConstants.all;
use work.SH2InstructionEncodings.all;
use work.SH2ControlSignals.all;
use work.SH2ALUConstants.all;
use work.Logging.all;
use work.Utils.all;

entity  SH2Control  is

    port (
        -- Input signals
        MemDataIn           : in  std_logic_vector(31 downto 0);    -- data read from memory
        TFlagIn             : in  std_logic;                        -- T Flag input from top level CPU
        clock               : in  std_logic;                        -- system clock
        reset               : in  std_logic;                        -- system reset (active low, async)

        -- Control signal groups
        MemCtrl             : out mem_ctrl_t;                       -- memory interface control signals
        ALUCtrl             : out alu_ctrl_t;                       -- ALU control signals
        RegCtrl             : out reg_ctrl_t;                       -- register array control signals
        DMAUCtrl            : out dmau_ctrl_t;                      -- DMAU control signals
        PMAUCtrl            : out pmau_ctrl_t;                      -- PMAU control signals
        SysCtrl             : out sys_ctrl_t                        -- system control signals
);
    
end  SH2Control;

architecture dataflow of sh2control is
    -- 
    -- Default control signals - shouldn't touch registers, but does fetch an
    -- instruction and increment PC

    -- Default control signals for Memory Interface
    constant DEFAULT_MEM_CTRL : mem_ctrl_t := (
        Enable      => '0',                         -- do nothing
        AddrSel     => MemAddrSel_PMAU,             -- unused
        ReadWrite   => ReadWrite_Read,              -- unused
        Mode        => WordMode,                    -- unused
        Sel         => MemAddrSel_PMAU,             -- unused
        OutSel      => (others => 'X')              -- unused
    );

    -- Default control signals for ALU
    constant DEFAULT_ALU_CTRL : alu_ctrl_t := (
        TFlagSel    => TFlagSel_T,                  -- T flag retains its value
        OpBSel      => ALUOpB_RegB,                 -- unused (assuming result of ALU not stored to a register)
        LoadA       => '0',                         -- unused
        FCmd        => (others => 'X'),             -- unused
        CinCmd      => (others => 'X'),             -- unused
        SCmd        => (others => 'X'),             -- unused
        ALUCmd      => (others => 'X'),             -- unused
        TCmpSel     => TCmp_EQ,                     -- unused
        Immediate   => (others => '0'),             -- unused
        ImmediateMode => ImmediateMode_Sign,        -- unused
        ExtMode     => Ext_ZeroB_RegA               -- unused
    );

    -- Default control signals for Register Array
    constant DEFAULT_REG_CTRL : reg_ctrl_t := (
        EnableIn    => '0',                             -- Do not write to any general-purpose register
        DataInSel   => RegDataIn_ALUResult,             -- unused
        InSel       => 0, ASel        => 0,             -- unused
        BSel        => 0                                -- unused
    );

    -- Default control signals for DMAU (Data Memory Access Unit)
    constant DEFAULT_DMAU_CTRL : dmau_ctrl_t := (
        AxStore     => '0',                             -- Do not write to address register
        GBRWriteEn  => '0',                             -- Do not write to GBR
        Off4        => (others => '0'),                 -- unused
        Off8        => (others => '0'),                 -- unused
        BaseSel     => (others => '0'),                 -- unused
        IndexSel    => (others => '0'),                 -- unused
        OffScalarSel=> (others => '0'),                 -- unused
        IncDecSel   => (others => '0'),                 -- unused
        A1Sel       => 0, A2Sel       => 0,             -- unused
        AxInSel     => 0                                -- unused
    );

    -- Default control signals for PMAU (Program Memory Access Unit)
    -- These defaults ensure the PC is incremented for the next instruction fetch.
    constant DEFAULT_PMAU_CTRL : pmau_ctrl_t := (
        PCAddrMode  => PCAddrMode_INC,                  -- Increment PC
        PCWriteCtrl => PCWriteCtrl_WRITE_CALC,          -- Write to PC
        Off8        => (others => '0'),                 -- unused
        Off12       => (others => '0'),                 -- unused
        DelayedBranchTaken  => '0'                      -- No delayed branch taken by default
    );

    constant HOLD_PMAU_CTRL : pmau_ctrl_t := (
        PCAddrMode  => PCAddrMode_HOLD,                 -- Hold PC constant (for bubbles)
        PCWriteCtrl => PCWriteCtrl_WRITE_CALC,          -- Write to PC
        Off8        => (others => '0'),                 -- unused
        Off12       => (others => '0'),                 -- unused
        DelayedBranchTaken  => '0'                      -- No delayed branch taken by default
    );

    -- Default control signals for System Control
    constant DEFAULT_SYS_CTRL : sys_ctrl_t := (
        PRWriteEn   => '0',                             -- Do not write to PR (Procedure Register)
        RegCtrl     => SysRegCtrl_None,                 -- Do nothing with system registers
        RegSel      => SysRegSel_SR,                    -- unused
        RegSrc      => SysRegSrc_RegB                   -- unused
    );

    constant DEFAULT_CTRL : ctrl_t := (
        ALUCtrl      => DEFAULT_ALU_CTRL,
        REGCtrl      => DEFAULT_REG_CTRL,
        PMAUCtrl     => DEFAULT_PMAU_CTRL,
        DMAUCtrl     => DEFAULT_DMAU_CTRL,
        SysCtrl      => DEFAULT_SYS_CTRL,
        MemCtrl      => DEFAULT_MEM_CTRL,
        BranchTaken  => '0',
        DBranchTaken => '0' 
    );


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

    -- n format
    alias n_format_n    : std_logic_vector(3 downto 0) is IR(11 downto 8);

    -- m format
    alias m_format_m    : std_logic_vector(3 downto 0) is IR(11 downto 8);

    -- nm format
    alias nm_format_n   : std_logic_vector(3 downto 0) is IR(11 downto 8);
    alias nm_format_m   : std_logic_vector(3 downto 0) is IR(7 downto 4);

    -- md format
    alias md_format_m   : std_logic_vector(3 downto 0) is IR(7 downto 4);
    alias md_format_d   : std_logic_vector(3 downto 0) is IR(3 downto 0);

    -- nd4 format
    alias nd4_format_n  : std_logic_vector(3 downto 0) is IR(7 downto 4);
    alias nd4_format_d  : std_logic_vector(3 downto 0) is IR(3 downto 0);

    -- nmd format
    alias nmd_format_n  : std_logic_vector(3 downto 0) is IR(11 downto 8);
    alias nmd_format_m  : std_logic_vector(3 downto 0) is IR(7 downto 4);
    alias nmd_format_d  : std_logic_vector(3 downto 0) is IR(3 downto 0);

    -- d format
    alias d_format_d    : std_logic_vector(7 downto 0) is IR(7 downto 0);

    -- d12 format
    alias d12_format_d  : std_logic_vector(11 downto 0) is IR(11 downto 0);

    -- nd8 format
    alias nd8_format_n  : std_logic_vector(3 downto 0) is IR(11 downto 8);
    alias nd8_format_d  : std_logic_vector(7 downto 0) is IR(7 downto 0);

    -- i format
    alias i_format_i    : std_logic_vector(7 downto 0) is IR(7 downto 0);

    -- ni format
    alias ni_format_n   : std_logic_vector(3 downto 0) is IR(11 downto 8);
    alias ni_format_i   : std_logic_vector(7 downto 0) is IR(7 downto 0);


    -- control signals to control memory interface
    signal MemEnable        : std_logic;                        -- if memory needs to be accessed (read or write)
    signal MemAddrSel       : std_logic;
    signal ReadWrite        : std_logic;                        -- if should do memory read (0) or write (1)
    signal MemMode          : std_logic_vector(1 downto 0);     -- if memory access should be by byte, word, or longword
    signal MemSel           : std_logic;                        -- select memory address source, from DMAU output (0) or PMAU output (1)

    signal Immediate        : std_logic_vector(7 downto 0);     -- 8-bit immediate
    signal ImmediateMode    : std_logic;                        -- Immediate extension mode
    signal MemOutSel        : std_logic_vector(2 downto 0);     -- what should be output to memory
    signal TFlagSel         : std_logic_vector(2 downto 0);     -- source for next value of T flag
    signal ExtMode          : std_logic_vector(1 downto 0);     -- mode for extending register value (zero or signed)

        -- ALU control signals
    signal ALUOpBSel        : std_logic;                        -- input mux to Operand B, either RegB (0) or Immediate (1)
    signal LoadA            : std_logic;                        -- determine if OperandA is loaded ('1') or zeroed ('0')
    signal FCmd             : std_logic_vector(3 downto 0);     -- F-Block operation
    signal CinCmd           : std_logic_vector(1 downto 0);     -- carry in operation
    signal SCmd             : std_logic_vector(2 downto 0);     -- shift operation
    signal ALUCmd           : std_logic_vector(1 downto 0);     -- ALU result select
    signal TCmpSel          : std_logic_vector(2 downto 0);     -- how to compute T from ALU status flags

        -- register array control signals
    signal RegDataInSel     : std_logic_vector(3 downto 0);     -- source for register input data
    signal RegEnableIn      : std_logic;                        -- if data should be written to an input register
    signal RegInSel         : integer  range 15 downto 0;       -- which register to write data to
    signal RegASel          : integer  range 15 downto 0;       -- which register to read to bus A
    signal RegBSel          : integer  range 15 downto 0;       -- which register to read to bus B
    signal RegAxInSel       : integer  range 15 downto 0;       -- which address register to write to
    signal RegAxStore       : std_logic;                        -- if data should be written to the address register
    signal RegA1Sel         : integer  range 15 downto 0;       -- which register to read to address bus 1
    signal RegA2Sel         : integer  range 15 downto 0;       -- which register to read to address bus 2

        -- DMAU signals
    signal GBRWriteEn       : std_logic;                        -- GBR write enable, active high
    signal DMAUOff4         : std_logic_vector(3 downto 0);     -- 4-bit offset
    signal DMAUOff8         : std_logic_vector(7 downto 0);     -- 8-bit offset
    signal BaseSel          : std_logic_vector(1 downto 0);     -- which base register source to select
    signal IndexSel         : std_logic_vector(1 downto 0);     -- which index source to select
    signal OffScalarSel     : std_logic_vector(1 downto 0);     -- what to scale the offset by (1, 2, 4)
    signal IncDecSel        : std_logic_vector(1 downto 0);     -- post-increment or pre-decrement the base
    signal DelayedBranchTaken  : std_logic;                     -- whether the delayed branch is taken

        -- PMAU signals
    signal PCAddrMode       : std_logic_vector(2 downto 0);     -- What PC addressing mode is desired.
    signal PRWriteEn        : std_logic;                        -- Enable writing to PR.
    signal PMAUOff8         : std_logic_vector(7 downto 0);     -- 8-bit offset for relative addressing.
    signal PMAUOff12        : std_logic_vector(11 downto 0);    -- 12-bit offset for relative addressing.
    signal PCWriteCtrl      : std_logic_vector(1 downto 0);     -- What to write to the PC register inside
                                                                -- the PMAU. Can either hold current value,
                                                                -- or write calculated PC. 

        -- System control signals
    signal SysRegCtrl       : std_logic_vector(1 downto 0);     -- how to update system registers 
    signal SysRegSel        : std_logic_vector(2 downto 0);     -- system register select 
    signal SysRegSrc        : std_logic_vector(1 downto 0);     -- source for data to input into a system register

    -- Pipelining signals
    type signal_array is array (0 to 3) of ctrl_t;

    -- Define which set of signals is associated with each stage in the 5-stage pipeline
    -- We don't include instruction fetch (IF) because it doesn't involve any
    -- control signals - latching in IR data takes a clock on its own, so it's
    -- implicitly a stage in the pipeline.
    constant STAGE_ID   : integer := 0;     -- instruction decode
    constant STAGE_X    : integer := 1;     -- execute instruction
    constant STAGE_MA   : integer := 2;     -- memory access
    constant STAGE_WR   : integer := 3;     -- writeback

    signal decoded_signals  : ctrl_t;       -- signals combinatorially decoded from the IR

    signal pipeline         : signal_array;                 -- pipeline the decoded control signals (DFFs)
    signal pipeline_en      : std_logic_vector(0 to 3);     -- if each stage in the pipeline should be enabled

    -- We need to insert a bubble if instruction fetch (IF) contention with
    -- memory access (MA). In this case, MA takes precedence, and IF is delayed
    -- by a clock. This is implemented by 1) not shifting forward stages IF, ID,
    -- and X, 2) holding the PC constant, and 3) ignoring the current value on
    -- the data bus
    signal BubbleIF         : std_logic;


    -- Whether a non-delayed branch is taken or not.
    signal BranchTaken      : std_logic;


begin

    pipeline_en <= (others => '1');

    decoded_signals <= (
        ALUCtrl => (
            OpBSel => ALUOpBSel,
            LoadA => LoadA,
            FCmd => FCmd,
            CinCmd => CinCmd,
            SCmd => SCmd,
            ALUCmd => ALUCmd,
            TCmpSel => TCmpSel,
            Immediate => Immediate,
            ImmediateMode => ImmediateMode,
            ExtMode => ExtMode,
            TFlagSel => TFlagSel
        ),

        RegCtrl => (
            DataInSel => RegDataInSel,
            EnableIn => RegEnableIn,
            InSel => RegInSel,
            ASel => RegASel,
            BSel => RegBSel
        ),

        DMAUCtrl => (
            GBRWriteEn => GBRWriteEn,
            Off4 => DMAUOff4,
            Off8 => DMAUOff8,
            BaseSel => BaseSel,
            IndexSel => IndexSel,
            OffScalarSel => OffScalarSel,
            IncDecSel => IncDecSel,
            AxInSel => RegAxInSel,
            AxStore => RegAxStore,
            A1Sel => RegA1Sel,
            A2Sel => RegA2Sel
        ),

        PMAUCtrl => (
            PCAddrMode => PCAddrMode_INC,           -- TODO: change for branch instructions
            Off8 => PMAUOff8,
            Off12 => PMAUOff12,
            PCWriteCtrl => PCWriteCtrl,
            DelayedBranchTaken => DelayedBranchTaken
        ),
        SysCtrl => (
            PRWriteEn => PRWriteEn,
            RegCtrl => SysRegCtrl,
            RegSel => SysRegSel,
            RegSrc => SysRegSrc
        ),
        MemCtrl =>  (
            Enable => MemEnable,
            AddrSel => MemAddrSel,
            ReadWrite => ReadWrite,
            Mode => MemMode,
            OutSel => MemOutSel,
            Sel => MemSel
        ),
        BranchTaken => BranchTaken,
        DBranchTaken => DelayedBranchTaken
    );

    -- Currently, IF occurs every clock, so if the instruction currently in the
    -- MA stage requires memory access, a bubble needs to be inserted.
    BubbleIF <= pipeline(STAGE_MA).MemCtrl.Enable;

    mem_access: process(all)
    begin
        if BubbleIF = '1' then
            -- Perform memory access, ignoring IF this clock
            MemCtrl <= pipeline(STAGE_MA).MemCtrl;
        else
            -- fetch instruction like usual
            MemCtrl <= (
                Enable => '1',
                AddrSel => MemAddrSel_PMAU,
                ReadWrite => ReadWrite_READ,
                Mode => WordMode,
                OutSel => MemOutSel,
                Sel => MemSel_ROM
            );
        end if ;
    end process mem_access;

    ALUCtrl <= pipeline(STAGE_X).ALUCtrl;
    RegCtrl <= pipeline(STAGE_X).RegCtrl when BubbleIF = '0' else pipeline(STAGE_MA).RegCtrl;
    DMAUCtrl <= pipeline(STAGE_MA).DMAUCtrl when BubbleIF = '0' else pipeline(STAGE_MA).DMAUCtrl;
    PMAUCtrl <= pipeline(STAGE_X).PMAUCtrl when BubbleIF = '0' else HOLD_PMAU_CTRL;
    SysCtrl <= pipeline(STAGE_X).SysCtrl when BubbleIF = '0'
               else pipeline(STAGE_MA).SysCtrl;

    -- Decode the current instruction combinatorially
    decode_proc: process (IR)
        variable l : line;
    begin


        -- Default flag values are set here (these shouldn't change CPU state).
        -- This is so that not every control signal has to be set in every single
        -- instruction case. If an instruction enables writing to memory/registers,
        -- then ensure that the default value is set here as "disable" to prevent
        -- writes on the clocks following an instruction.

        -- Not accessing memory
        MemEnable  <= '0';
        ReadWrite  <= 'X';
        MemMode    <= "XX";
        MemOutSel              <= "XXX";
        MemSel     <= MemSel_RAM;       -- access data memory by default.
        MemAddrSel <= MemAddrSel_DMAU;  -- access data memory by default.

        -- Register enables
        RegEnableIn <= '0';             -- Disable register write
        RegAxStore  <= '0';             -- Disable writing to address register.
        TFlagSel    <= TFlagSel_T;      -- Keep T flag the same
        GBRWriteEn  <= '0';             -- Don't write to GBR.
        PRWriteEn   <= '0';             -- Don't write to PR.

        SysRegCtrl    <= SysRegCtrl_NONE;      -- system register not selected
        ImmediateMode <= ImmediateMode_SIGN;   -- sign-extend immediates by defualt
        ExtMode       <= Ext_SignB_RegA;

        PCWriteCtrl <= PCWriteCtrl_WRITE_CALC;  -- Write the calculated PC by default.

        DelayedBranchTaken <= '0'; -- The delayed branch taken flag is set to not taken by default.
        BranchTaken        <= '0';

        -- Note that this updates the next time this process runs. Therefore, BranchTaken is still '1'
        -- when the process goes to the next if and the decoding is aborted.
        if (BranchTaken = '1') then
            BranchTaken <= '0';
        end if;


        if (BranchTaken = '0') then

            if std_match(IR, ADD_RM_RN) then
                -- ADD{C,V} Rm, Rn

                LogWithTime(l, "sh2_control.vhd: Decoded Add R" & to_string(to_integer(unsigned(nm_format_m))) &
                               " , R" & to_string(to_integer(unsigned(nm_format_n))), LogFile);

                -- Register array signals
                RegASel <= to_integer(unsigned(nm_format_n));
                RegBSel <= to_integer(unsigned(nm_format_m));

                RegInSel             <= to_integer(unsigned(nm_format_n));
                RegDataInSel         <= RegDataIn_ALUResult;
                RegEnableIn <= '1';

                -- Bit-decoding T flag select (None, Carry, Overflow)
                TFlagSel <= '0' & IR(1 downto 0);

                -- ALU signals for addition
                ALUOpBSel <= ALUOpB_RegB;
                LoadA     <= '1';
                FCmd      <= FCmd_B;

                -- Bit-decode carry in value
                CinCmd <= CinCmd_CIN when IR(1 downto 0) = "10" else    -- ADDC
                          CinCmd_ZERO;                                  -- ADD, ADDV

                SCmd   <= "XXX";
                ALUCmd <= ALUCmd_ADDER;


            elsif std_match(IR, SUB_RM_RN) then
                -- SUB{C,V} Rm, Rn

                -- Register array signals
                RegASel <= to_integer(unsigned(nm_format_n));
                RegBSel <= to_integer(unsigned(nm_format_m));

                RegInSel             <= to_integer(unsigned(nm_format_n));
                RegDataInSel         <= RegDataIn_ALUResult;
                RegEnableIn <= '1';

                -- Bit-decoding T flag select (None, Carry, Overflow)
                TFlagSel <= '0' & IR(1 downto 0);

                -- ALU signals for subtraction
                ALUOpBSel <= ALUOpB_RegB;
                LoadA     <= '1';
                FCmd      <= FCmd_BNOT;

                -- Bit-decode carry in value
                CinCmd <= CinCmd_CINBAR when IR(1 downto 0) = "10" else     -- SUBC
                          CinCmd_ONE;                                       -- SUB, SUBV

                SCmd   <= "XXX";
                ALUCmd <= ALUCmd_ADDER;

            elsif std_match(IR, DT_RN) then
                -- DT Rn

                -- Register array signals
                RegASel <= to_integer(unsigned(nm_format_n));
                Immediate <= (others => '0');

                RegInSel             <= to_integer(unsigned(nm_format_n));
                RegDataInSel         <= RegDataIn_ALUResult;
                RegEnableIn <= '1';

                -- Bit-decoding T flag select (None, Carry, Overflow)
                TFlagSel <= TFlagSel_Zero;

                -- ALU signals to subtract 1 from Rn
                ALUOpBSel <= ALUOpB_Imm;
                LoadA     <= '1';
                FCmd      <= FCmd_BNOT;
                CinCmd    <= CinCmd_ZERO;
                SCmd      <= "XXX";
                ALUCmd    <= ALUCmd_ADDER;

            elsif std_match(IR, NEG_RM_RN) then
                -- NEG{C} Rm, Rn

                -- Register array signals
                RegASel <= to_integer(unsigned(nm_format_n));
                RegBSel <= to_integer(unsigned(nm_format_m));

                RegInSel             <= to_integer(unsigned(nm_format_n));
                RegDataInSel         <= RegDataIn_ALUResult;
                RegEnableIn <= '1';

                -- Bit-decoding T flag select
                TFlagSel <= TFlagSel_Carry when IR(0) = '0' else    -- NEGC
                                        TFlagSel_T;                             -- NEG

                -- ALU signals for negation
                ALUOpBSel <= ALUOpB_RegB;
                LoadA     <= '0';
                FCmd      <= FCmd_BNOT;

                -- Bit-decode carry in value
                CinCmd <= CinCmd_CINBAR when IR(0) = '0' else   -- NEGC
                          CinCmd_ONE;                           -- NEG

                SCmd   <= "XXX";
                ALUCmd <= ALUCmd_ADDER;

            elsif std_match(IR, EXT_RM_RN) then
                -- EXT{U,S}.{B,W Rm, Rn}

                -- Register array signals
                RegASel <= to_integer(unsigned(nm_format_n));
                RegBSel <= to_integer(unsigned(nm_format_m));

                RegInSel             <= to_integer(unsigned(nm_format_n));
                RegDataInSel         <= RegDataIn_Ext;
                ExtMode              <= IR(1 downto 0);     -- bit-decode extension mode
                RegEnableIn <= '1';

            elsif std_match(IR, ADD_IMM_RN) then
                -- ADD #imm, Rn

                -- Register array signals
                RegASel <= to_integer(unsigned(nm_format_n));

                RegInSel             <= to_integer(unsigned(nm_format_n));
                RegDataInSel         <= RegDataIn_ALUResult;
                RegEnableIn <= '1';
                Immediate            <= ni_format_i;

                -- ALU signals for addition
                ALUOpBSel <= ALUOpB_Imm;
                LoadA     <= '1';
                FCmd      <= FCmd_B;
                CinCmd    <= CinCmd_ZERO;
                SCmd      <= "XXX";
                ALUCmd    <= ALUCmd_ADDER;

            elsif std_match(IR, LOGIC_RM_RN) then
                -- {AND, TST, OR, XOR} Rm, Rn

                -- Register array signals
                RegASel <= to_integer(unsigned(nm_format_n));
                RegBSel <= to_integer(unsigned(nm_format_m));

                RegInSel             <= to_integer(unsigned(nm_format_n));
                RegDataInSel         <= RegDataIn_ALUResult;
                RegEnableIn <= IR(1) or IR(0);   -- exclude TST

                -- Enable TFlagSel for TST
                TFlagSel <= TFlagSel_Zero when IR(1 downto 0) = "00"    -- TST
                                        else TFlagSel_T;                            -- AND, OR, XOR

                -- ALU signals for logic instructions using the FBlock
                ALUOpBSel <= ALUOpB_RegB;
                LoadA     <= '1';

                -- Bit-decode f-block operation
                FCmd <= FCmd_AND when IR(1) = '0'           else    -- AND, TST
                        FCmd_XOR when IR(1 downto 0) = "10" else    -- XOR
                        FCmd_OR;                                    -- OR

                CinCmd <= CinCmd_ZERO;
                SCmd   <= "XXX";
                ALUCmd <= ALUCmd_FBLOCK;

            elsif std_match(IR, LOGIC_IMM_R0) then
                -- {AND, TST, OR, XOR} immediate, R0

                -- Register array signals
                RegASel <= 0;

                RegInSel             <= 0;
                RegDataInSel         <= RegDataIn_ALUResult;
                RegEnableIn <= IR(9) or IR(8);   -- exclude TST
                Immediate            <= i_format_i;
                ImmediateMode        <= ImmediateMode_ZERO;

                -- Enable TFlagSel for TST
                TFlagSel <= TFlagSel_Zero when IR(9 downto 8) = "00"    -- TST
                                        else TFlagSel_T;                            -- AND, OR, XOR

                -- ALU signals for logic instructions using the FBlock
                ALUOpBSel <= ALUOpB_Imm;
                LoadA     <= '1';

                -- Bit-decode f-block operation
                FCmd <= FCmd_AND when IR(9) = '0' else              -- AND, TST
                        FCmd_XOR when IR(9 downto 8) = "10" else    -- XOR
                        FCmd_OR;                                    -- OR

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
                RegEnableIn <= '1';

                -- ALU signals for logical negation
                ALUOpBSel <= ALUOpB_RegB;
                LoadA     <= '1';
                FCmd      <= FCmd_BNOT;
                CinCmd    <= CinCmd_ZERO;
                SCmd      <= "XXX";
                ALUCmd    <= ALUCmd_FBLOCK;

            elsif std_match(IR, CMP_EQ_IMM) then
                -- CMP/EQ #Imm, R0

                -- Register array signals
                RegASel <= 0;

                Immediate            <= i_format_i;
                ImmediateMode        <= ImmediateMode_ZERO;

                -- Compute T flag based on ALU flags
                TFlagSel <= TFlagSel_CMP;
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
                -- CMP/XX Rm, Rn

                -- Register array signals
                RegASel <= to_integer(unsigned(nm_format_n));
                RegBSel <= to_integer(unsigned(nm_format_m));

                -- Compute T flag based on ALU flags
                TFlagSel <= TFlagSel_CMP;
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
                -- CMP/STR Rm, Rn

                -- Register array signals
                RegASel <= to_integer(unsigned(nm_format_n));
                RegBSel <= to_integer(unsigned(nm_format_m));

                -- Compute T flag based on ALU flags
                TFlagSel <= TFlagSel_CMP;
                TCMPSel <= TCMP_STR;

            elsif std_match(IR, CMP_RN) then
                -- CMP/{PL/PZ} Rn

                -- Register array signals
                RegASel <= to_integer(unsigned(nm_format_n));

                -- Compare to 0
                Immediate <= (others => '0');

                -- Compute T flag based on ALU flags
                TFlagSel <= TFlagSel_CMP;
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
                -- Shift operations
                -- {ROTL, ROTR, ROTCL, ROTCR, SHAL, SHAR, SHLL, SHLR} Rn
                -- Uses bit decoding to compute control signals (to reduce code size)

                -- Register array signals
                RegASel              <= to_integer(unsigned(n_format_n));
                RegInSel             <= to_integer(unsigned(n_format_n));
                RegDataInSel         <= RegDataIn_ALUResult;
                RegEnableIn <= '1';

                TFlagSel <= TFlagSel_Carry;

                -- ALU signals
                ALUOpBSel <= ALUOpB_RegB;
                LoadA     <= '1';
                FCmd      <= "XXXX";

                -- Bit-decode carry command
                CinCmd    <= CinCmd_CIN when (IR(5) and IR(2)) = '1' else   -- ROTCL, ROTCR
                             CinCmd_ZERO;                                   -- all others   

                SCmd   <= IR(0) & IR(2) & IR(5);  -- bit-decode shift operation
                ALUCmd <= ALUCmd_SHIFT;

            elsif std_match(IR, BSHIFT_RN) then
                -- Barrel shift operations
                -- {SHLL,SHLR}{2,8,16} Rn
                -- Uses bit decoding to compute control signals (to reduce code size)

                -- Register array signals
                RegASel              <= to_integer(unsigned(n_format_n));
                RegInSel             <= to_integer(unsigned(n_format_n));
                RegDataInSel         <= RegDataIn_ALUResult;
                RegEnableIn <= '1';

                TFlagSel <= TFlagSel_T;

                -- ALU signals
                LoadA     <= '1';
                SCmd   <= IR(5) & IR(4) & IR(0);  -- bit-decode barrel shift operation
                ALUCmd <= ALUCmd_BSHIFT;
            
            -- Data Transfer Instruction -------------------------------------------

            -- MOV #imm, Rn
            -- ni format
            elsif std_match(IR, MOV_IMM_RN) then

                LogWithTime(l, "sh2_control.vhd: Decoded MOV H'" & to_hstring(ni_format_i) &
                              ", R" & to_string(slv_to_uint(ni_format_n)), LogFile);
              
                RegInSel             <= to_integer(unsigned(ni_format_n));
                RegDataInSel         <= RegDataIn_Immediate;
                RegEnableIn <= '1';
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
                RegEnableIn <= '1';                                  -- Writes to register. 

                RegASel <= to_integer(unsigned(nd8_format_n));

                -- Instruction reads word from program memory (ROM).
                MemEnable <= '1';
                ReadWrite <= ReadWrite_READ; 
                MemMode   <= WordMode;
                MemSel    <= MemSel_ROM;

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
                RegEnableIn <= '1';                                 -- Writes to register. 

                -- Instruction reads from longword memory.
                MemEnable <= '1';
                ReadWrite <= ReadWrite_READ; 
                MemMode   <= LongwordMode;
                MemSel    <= MemSel_ROM;

                -- DMAU signals for PC Relative addressing with displacement (longword mode)
                BaseSel      <= BaseSel_PC;
                IndexSel     <= IndexSel_OFF8;
                OffScalarSel <= OffScalarSel_FOUR;
                IncDecSel    <= IncDecSel_NONE;
                DMAUOff8     <= nd8_format_d;


            -- MOV Rm, Rn
            -- nm format
            -- Note: for bit decoding, this must be done before MOV_AT_RM_RN
            elsif std_match(IR, MOV_RM_RN) then
                LogWithTime(l, 
                  "sh2_control.vhd: Decoded MOV R" & to_string(slv_to_uint(nm_format_m)) &
                  "R" & to_string(slv_to_uint(nm_format_n)) , LogFile);

                -- report "Instruction: MOV Rm, Rn";
                RegBSel              <= to_integer(unsigned(nm_format_m));
                RegInSel             <= to_integer(unsigned(nm_format_n));
                RegDataInSel         <= RegDataIn_RegB;
                RegEnableIn <= '1';

            -- MOV.X Rm, @Rn
            -- nm format
            elsif std_match(IR, MOV_RM_AT_RN) then
                LogWithTime(l, 
                  "sh2_control.vhd: Decoded MOV.X R" & to_string(slv_to_uint(nm_format_m)) &
                  ", @R" & to_string(slv_to_uint(nm_format_n)) , LogFile);

                -- Writes a byte to memory to memory
                MemEnable <= '1';             -- Uses memory.
                ReadWrite <= ReadWrite_WRITE; -- Writes.
                MemMode   <= IR(1 downto 0);  -- bit decode memory mode

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
            -- Note: for bit decoding, this must be done after MOV_RM_RN
            elsif std_match(IR, MOV_AT_RM_RN) then
                LogWithTime(l, 
                  "sh2_control.vhd: Decoded MOV.X @R" & to_string(slv_to_uint(nm_format_m)) &
                  ", R" & to_string(slv_to_uint(nm_format_n)) , LogFile);

                -- Instruction reads byte from memory.
                MemEnable <= '1';            -- Instr does memory access.
                ReadWrite <= ReadWrite_READ; -- Instr reads from memory.
                MemMode   <= IR(1 downto 0); -- bit decode memory mode

                -- DMAU signals for Indirect Register addressing.
                BaseSel      <= BaseSel_REG;
                IndexSel     <= IndexSel_NONE;
                OffScalarSel <= OffScalarSel_ONE;
                IncDecSel    <= IncDecSel_NONE;

                -- Output @(Rm) to RegA2. 
                RegA2Sel <= to_integer(unsigned(nm_format_m));

                RegInSel             <= to_integer(unsigned(nm_format_n));
                RegDataInSel         <= RegDataIn_DB;
                RegEnableIn <= '1';


            -- MOV.B Rm, @-Rn
            -- nm format
            elsif std_match(IR, MOV_RM_AT_MINUS_RN) then
                LogWithTime(l, 
                  "sh2_control.vhd: Decoded MOV.X R" & to_string(slv_to_uint(nm_format_m)) &
                  ", @-R" & to_string(slv_to_uint(nm_format_n)) , LogFile);

                -- Writes a byte to memory
                MemEnable <= '1';             -- Uses memory.
                ReadWrite <= ReadWrite_WRITE; -- Writes.
                MemMode   <= IR(1 downto 0);  -- bit decode memory mode

                MemOutSel <= MemOut_RegB; -- Output RegB (Rm) to memory data bus.

                RegBSel                <= to_integer(unsigned(nm_format_m));  -- Output Rm from RegB output.
                RegA1Sel               <= to_integer(unsigned(nm_format_n));  -- Output @(Rn) from RegA1 output.
                RegAxInSel             <= to_integer(unsigned(nm_format_n));  -- Store calculated address into Rn
                RegAxStore <= '1';                                -- Enable writes to address registers.

                -- DMAU signals (for Pre-decrement indirect register addressing)
                BaseSel      <= BaseSel_REG;
                IndexSel     <= IndexSel_NONE;
                OffScalarSel <= IR(1 downto 0);         -- bit decode offset scalar factor
                IncDecSel    <= IncDecSel_PRE_DEC;

            -- MOV.B @Rm+, Rn
            -- nm format
            elsif std_match(IR, MOV_AT_RM_PLUS_RN) then
                LogWithTime(l, 
                  "sh2_control.vhd: Decoded MOV.{B,W,L} @R" & to_string(slv_to_uint(nm_format_m)) &
                  "+, R" & to_string(slv_to_uint(nm_format_n)) , LogFile);

                -- MOV with post-increment. This Instruction reads a byte, word,
                -- or longword from an address in Rm, into Rn. The address is
                -- incremented and stored in Rm after the value is retrieved.
                
                -- Reads a byte from memory.
                MemEnable <= '1';             -- Uses memory.
                ReadWrite <= ReadWrite_READ;  -- Reads.
                MemMode   <= IR(1 downto 0);  -- bit-decode memory word mode

                -- Output @Rm from RegA2
                RegA2Sel <= to_integer(unsigned(nm_format_m));

                -- Write output of Data Bus to Rn
                RegInSel             <= to_integer(unsigned(nm_format_n));
                RegDataInSel         <= RegDataIn_DB;
                RegEnableIn <= '1';  -- Enable writing to registers.

                -- Write the incremented address to Rm
                RegAxInSel             <= to_integer(unsigned(nm_format_m));
                RegAxStore <= '1'; -- Enable writing to address register in the writeback state.
                
                -- DMAU signals for post-increment indirect register addressing
                BaseSel      <= BaseSel_REG;
                IndexSel     <= IndexSel_NONE;
                OffScalarSel <= IR(1 downto 0);       -- bit-decode offset scalar select
                IncDecSel    <= IncDecSel_POST_INC;


            -- MOV.{B,W} RO, @(disp,Rn)
            -- nd4 format
            -- Note that the displacement depends on the mode of the address, so in
            -- byte mode, the displacement represents bytes, in word mode it represents
            -- words, etc. This is done to maximize it's range.
            elsif std_match(IR, MOV_R0_AT_DISP_RN) then

                LogWithTime(l, 
                  "sh2_control.vhd: Decoded MOV.{B,W} R0, @(0x" & to_hstring(nd4_format_d) &
                  ", " & to_string(slv_to_uint(nd4_format_n)) & ")", LogFile);

                -- Instruction writes a byte to data memory.
                MemEnable   <= '1';
                ReadWrite   <= ReadWrite_WRITE;

                MemMode     <= ByteMode when IR(8) = '0' else     -- bit-decode byte/word mode
                                           WordMode;
                
                -- Output RegB (R0) to memory data bus
                MemOutSel <= MemOut_RegB;

                -- Output R0 to RegB
                RegBSel <= 0;
                
                -- Output Rn to RegA1. The DMAU will use this to calculate the address
                -- to write to.
                RegA1Sel <= to_integer(unsigned(nd4_format_n));

                -- DMAU signals for Indirect register addressing with displacement
                BaseSel       <=  BaseSel_REG;
                IndexSel      <=  IndexSel_OFF4;
                OffScalarSel  <=  OffScalarSel_ONE when IR(8) = '0' else      -- bit-decode byte/word
                                  OffScalarSel_TWO;
                IncDecSel     <=  IncDecSel_NONE;
                DMAUOff4      <=  nd4_format_d;


            -- MOV.L Rm, @(disp, Rn)
            -- nmd format
            elsif std_match(IR, MOV_L_RM_AT_DISP_RN) then

                LogWithTime(l, 
                  "sh2_control.vhd: Decoded MOV.L R" & to_string(slv_to_uint(nmd_format_m)) &
                  ", @(0x" & to_hstring(nmd_format_d) & ", R" & to_string(slv_to_uint(nmd_format_n)) & ")", LogFile);

                MemEnable <= '1';
                ReadWrite <= ReadWrite_WRITE;
                MemMode   <= LongwordMode;

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


          -- MOV.{B,W} @(disp, Rm), R0
          -- md format
          -- Note that these instructions are very similar to MOV @(disp, PC), Rn
          elsif std_match(IR, MOV_AT_DISP_RM_R0) then

              LogWithTime(l, 
                "sh2_control.vhd: Decoded MOV.{B,W} @(0x" & to_hstring(md_format_d) &
                ", R" & to_string(slv_to_uint(md_format_m)) & "), R0", LogFile);

                -- Writing sign-extended byte from data bus to R0.
                RegInSel             <= 0;             -- Select R0 to write to.
                RegDataInSel         <= RegDataIn_DB;  -- Write DataBus to reg.
                RegEnableIn <= '1';           -- Enable Reg writing for this instruction.

                -- Output @Rm from RegA2
                RegA2Sel <= to_integer(unsigned(md_format_m));

                MemEnable  <=  '1';            -- Instr uses memory.
                ReadWrite  <=  ReadWrite_READ; -- Reads.
                MemMode    <=  ByteMode when IR(8) = '0' else       -- bit-decode word mode
                                           WordMode;
                MemSel     <=  MemSel_RAM;     -- Reads from RAM

                -- DMAU signals for Indirect register addressing with displacement (byte mode)
                BaseSel      <= BaseSel_REG;
                IndexSel     <= IndexSel_OFF4;
                OffScalarSel <= OffScalarSel_ONE when IR(8) = '0' else          -- bit-decode offset scale
                                OffScalarSel_TWO;
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
                RegEnableIn <= '1';                                  -- Enable Reg writing for this instruction.

                 -- Output @Rm from RegA2
                 RegA2Sel <= to_integer(unsigned(nmd_format_m));

                MemEnable  <=  '1';               -- Instr uses memory.
                ReadWrite  <=  ReadWrite_READ;    -- Reads.
                MemMode    <=  LongwordMode;      -- Reads longword.
                MemSel     <=  MemSel_RAM;        -- Reads from RAM


                -- DMAU signals for Indirect register addressing with displacement (longword mode)
                BaseSel      <= BaseSel_REG;
                IndexSel     <= IndexSel_OFF4;
                OffScalarSel <= OffScalarSel_FOUR;
                IncDecSel    <= IncDecSel_NONE;
                DMAUOff4     <= nmd_format_d;


            -- MOV.{B,W,L} Rm, @(R0, Rn)
            -- nm format
            elsif std_match(IR, MOV_RM_AT_R0_RN) then

                LogWithTime(l, 
                  "sh2_control.vhd: Decoded MOV.X R" & to_string(slv_to_uint(nm_format_m)) &
                  ", @(R0, R" & to_string(slv_to_uint(nm_format_n)) & ")", LogFile);

                -- Instr writes a byte to memory.
                MemEnable <= '1';
                ReadWrite <= ReadWrite_WRITE;
                MemMode   <= IR(1 downto 0);      -- bit-decode memory mode

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
            

            -- MOV.{B,W,L} @(R0, Rm), Rn
            -- nm format
            elsif std_match(IR, MOV_AT_R0_RM_RN) then

                LogWithTime(l, 
                  "sh2_control.vhd: Decoded MOV.X @(R0, R" & to_string(slv_to_uint(nm_format_m)) &
                  "), R" & to_string(slv_to_uint(nm_format_n)), LogFile);

                -- Writing sign-extended byte from data bus to Rn.
                RegInSel             <= slv_to_uint(nm_format_n);     -- Select Rn to write to.
                RegDataInSel         <= RegDataIn_DB;                 -- Write DataBus to reg.
                RegEnableIn <= '1';                          -- Enable Reg writing for this instruction.

                -- Output @Rm from RegA2
                RegA2Sel <= slv_to_uint(nm_format_m);

                -- Output @R0 from RegA1
                RegA1Sel <= 0;

                MemEnable  <=  '1';             -- Instr uses memory.
                ReadWrite  <=  ReadWrite_READ;  -- Reads.
                MemMode    <=  IR(1 downto 0);  -- bit decode memory mode
                MemSel     <=  MemSel_RAM;      -- Reads from RAM

                -- DMAU Signals for Indirect indexed Register Addressing
                BaseSel       <= BaseSel_REG;
                IndexSel      <= IndexSel_R0;
                OffScalarSel  <= OffScalarSel_ONE;
                IncDecSel     <= IncDecSel_NONE;
              

            -- MOV.{B,W,L} R0, @(disp, GBR)
            -- d format
            elsif std_match(IR, MOV_R0_AT_DISP_GBR) then

                LogWithTime(l,
                  "sh2_control.vhd: Decoded MOV.X R0, @(0x" & to_hstring(d_format_d) &
                  ", GBR)", LogFile);

                -- Writing to memory
                MemEnable <= '1';
                ReadWrite <= ReadWrite_WRITE;
                MemMode   <= IR(9 downto 8);      -- bit-decode memory mode

                -- Output R0 to RegB.
                RegBSel <= 0;

                -- Output RegB (Rm) to memory data bus. This will be written to memory.
                MemOutSel <= MemOut_RegB;

                -- DMAU signals for Indirect GBR addressing with displacement
                BaseSel       <=  BaseSel_GBR;
                IndexSel      <=  IndexSel_OFF8;
                OffScalarSel  <=  IR(9 downto 8);     -- bit decode offset scalar select
                IncDecSel     <=  IncDecSel_NONE;
                DMAUOff8      <=  d_format_d;

            -- MOVA @(disp, PC), R0
            -- d format
            -- disp*4 + PC -> R0
            -- Note: due to bit decoding, this must come before MOV_AT_DISP_GBR_R0
            elsif std_match(IR, MOVA_AT_DISP_PC_R0) then

                LogWithTime(l,
                    "sh2_control.vhd: Decoded MOVA @(" & to_hstring(d_format_d) & 
                    ", PC), R0", LogFile);

                -- Note that this instruction moves the address, disp*4 + PC
                -- (calculated by the DMAU) into R0. It does NOT move the data at
                -- this address.

                RegAxInSel             <=  0;   -- Write address to R0
                RegAxStore <= '1';  -- Enable writing to address register in writeback state.

                -- DMAU signals for PC Relative addressing with displacement (longword mode)
                BaseSel      <= BaseSel_PC;
                IndexSel     <= IndexSel_OFF8;
                OffScalarSel <= OffScalarSel_FOUR;
                IncDecSel    <= IncDecSel_NONE;
                DMAUOff8     <= nd8_format_d;

            -- MOV.{B,W,L} @(disp, GBR), R0
            -- d format
            -- Note: due to bit decoding, this must come after MOVA_AT_DISP_PC_R0
            elsif std_match(IR, MOV_AT_DISP_GBR_R0) then

                LogWithTime(l,
                    "sh2_control.vhd: Decoded MOV.X @(0x" & to_hstring(d_format_d) &
                    ", GBR), R0", LogFile);

               RegInSel             <= 0;               -- Write to R0
               RegDataInSel         <= RegDataIn_DB;    -- Write Data bus to R0
               RegEnableIn <= '1';             -- Enable register writing for this instruction.
               
               MemEnable <= '1';    
               ReadWrite <= ReadWrite_READ;
               MemMode   <= IR(9 downto 8); -- bit decode memory mode

                -- DMAU signals for Indirect GBR addressing with displacement (byte mode)
                BaseSel      <=  BaseSel_GBR;
                IndexSel     <=  IndexSel_OFF8;
                OffScalarSel <=  IR(9 downto 8);          -- bit decode offset scalar select
                IncDecSel    <=  IncDecSel_NONE;
                DMAUOff8     <=  d_format_d;

            -- MOVT Rn
            -- n format.
            elsif std_match(IR, MOVT_RN) then

                LogWithTime(l,
                    "sh2_control.vhd: Decoded MOVT R" & to_string(slv_to_uint(n_format_n)), 
                    LogFile);

                RegInSel             <= to_integer(unsigned(n_format_n));
                RegDataInSel         <= RegDataIn_SR_TBit;
                RegEnableIn <= '1';


            -- SWAP.B Rm, Rn
            -- nm format
            -- Rm -> Swap upper and lower 2 bytes -> Rn
            elsif std_match(IR, SWAP_RM_RN) then

                LogWithTime(l,
                    "sh2_control.vhd: Decoded SWAP.X R" & to_string(slv_to_uint(nm_format_m))
                    & ", R" & to_string(nm_format_n), LogFile);

                RegASel      <= slv_to_uint(nm_format_m);
                RegInSel     <= slv_to_uint(nm_format_n);

                -- Bit decode if whether byte or word mode
                RegDataInSel <= RegDataIn_RegA_SwapB when IR(0) = '0' else
                                RegDataIn_RegA_SwapW;

                RegEnableIn <= '1';


            -- XTRCT Rm, Rn
            -- nm format
            -- Center 32 bits of Rm and Rn -> Rn
            elsif std_match(IR, XTRCT_RM_RN) then
            
                LogWithTime(l,
                    "sh2_control.vhd: Decoded XTRCT R" & to_string(slv_to_uint(nm_format_m))
                    & ", R" & to_string(nm_format_n), LogFile);

                RegASel <= slv_to_uint(nm_format_n);
                RegBSel <= slv_to_uint(nm_format_m);

                RegInSel <= slv_to_uint(nm_format_n); -- Write to Rn

                RegDataInSel <= RegDataIn_REGB_REGA_CENTER;

                RegEnableIn <= '1';
                


            -- Branch Instructions -------------------------------------------------

            -- BF <label> (where label is disp*2 + PC)
            -- d format
             elsif std_match(IR, BF) then
     
                 LogWithTime(l,
                     "sh2_control.vhd: Decoded BF (label=" & to_hstring(d_format_d) &
                     "*2 + PC)", LogFile);

                 -- If T=0, disp*2 + PC -> PC; if T=1, nop (where label is disp*2 + PC)

                 if (TFlagIn = '0') then

                     PCAddrMode  <= PCAddrMode_RELATIVE_8;
                     PMAUOff8    <= d_format_d;
                     BranchTaken <= '1';

                 else

                     -- Go to the next instruction.
                     PCAddrMode  <= PCAddrMode_INC;  -- Increment PC

                 end if;
     
     
     
             -- BF/S <label> (where label is disp*2 + PC)
             -- d format
             elsif std_match(IR, BF_S) then
     
                 LogWithTime(l,
                     "sh2_control.vhd: Decoded BF/S (label=" & to_hstring(d_format_d) &
                     "*2 + PC)", LogFile);
     
                 if (TFlagIn = '0') then
                     -- Take the branch

                     --  The delay will be taken.
                     DelayedBranchTaken  <= '1';
                     PCWriteCtrl                     <= PCWriteCtrl_WRITE_CALC;

                     PCAddrMode <= PCAddrMode_RELATIVE_8;
                     PMAUOff8               <= d_format_d;

                 else
                     -- Go to the next instruction.
                     PCAddrMode  <= PCAddrMode_INC;  -- Increment PC
                 end if;
     
     
             -- BT <label> (where label is disp*2 + PC)
             -- d format
             elsif std_match(IR, BT) then
     
                 -- Branch true without delay slot.

                 LogWithTime(l,
                     "sh2_control.vhd: Decoded BT (label=" & to_hstring(d_format_d) &
                     "*2 + PC)", LogFile);
                
                 -- If T=1, disp*2 + PC -> PC; if T=0, nop (where label is disp*2 + PC)

                 if (TFlagIn = '1') then

                    PCAddrMode  <= PCAddrMode_RELATIVE_8;
                    PMAUOff8    <= d_format_d;
                    BranchTaken <= '1';

                 else
                     -- Go to the next instruction.
                     PCAddrMode  <= PCAddrMode_INC;  -- Increment PC
                 end if;
     
     
             -- BT/S <label> (where label is disp*2 + PC)
             -- d format
             elsif std_match(IR, BT_S) then
     
                 LogWithTime(l,
                     "sh2_control.vhd: Decoded BT/S (label=" & to_hstring(d_format_d) &
                     "*2 + PC)", LogFile);
     

                 -- If T=1, disp*2 + PC -> PC; if T=0, nop (where label is disp*2 + PC)
                 if (TFlagIn = '1') then

                     --  The delay will be taken.
                     DelayedBranchTaken  <= '1';
                     PCWriteCtrl                     <= PCWriteCtrl_WRITE_CALC;

                     PCAddrMode <= PCAddrMode_RELATIVE_8;
                     PMAUOff8               <= d_format_d;

                 else
                     -- Go to the next instruction.
                     PCAddrMode  <= PCAddrMode_INC;  -- Increment PC
                 end if;
     
     
             -- BRA <label> (where label is disp*2 + PC)
             -- d12 format
             elsif std_match(IR, BRA) then

                 LogWithTime(l,
                     "sh2_control.vhd: Decoded BRA (label=" & to_hstring(d12_format_d) &
                     "*2 + PC)", LogFile);

                DelayedBranchTaken <= '1';
                PCWriteCtrl                    <= PCWriteCtrl_WRITE_CALC;

                PCAddrMode <= PCAddrMode_RELATIVE_12;
                PMAUOff12              <= d12_format_d;
     

             -- BRAF Rm
             -- m format
             elsif std_match(IR, BRAF) then

                 -- Delayed branch, Rm + PC -> PC
                 -- Note that the PMAU's register input is always RegB.

                  RegBSel <= slv_to_uint(m_format_m);

                  LogWithTime(l,
                      "sh2_control.vhd: Decoded BRAF R" & to_string(slv_to_uint(m_format_m)), LogFile); 
     
                  DelayedBranchTaken <= '1';
                  PCWriteCtrl                    <= PCWriteCtrl_WRITE_CALC;

                  PCAddrMode <= PCAddrMode_REG_DIRECT_RELATIVE;


             -- BSR <label> (where label is disp*2)
             -- d12 format
             elsif std_match(IR, BSR) then
     
                 LogWithTime(l,
                     "sh2_control.vhd: Decoded BSR (label=" & to_hstring(d12_format_d) &
                     "*2 + PC)", LogFile);
     
                 DelayedBranchTaken <= '1';
                 PCWriteCtrl                    <= PCWriteCtrl_WRITE_CALC;

                 PCAddrMode <= PCAddrMode_RELATIVE_12;
                 PMAUOff12  <= d12_format_d;

                 PRWriteEn <= '1';

                 -- Control signals to write PC to PR.
                 SysRegSrc              <= SysRegSrc_PC;
                 SysRegCtrl <= SysRegCtrl_LOAD;
     

             -- BSRF Rm
             -- m format
             --
             -- Branch to sub-routine far.
             -- PC -> PR, Rm + PC -> PC
             elsif std_match(IR, BSRF) then

                 LogWithTime(l,
                     "sh2_control.vhd: Decoded BSRF R" & to_string(slv_to_uint(m_format_m)), LogFile);

                -- Basically BSR, but with a different target.
                 DelayedBranchTaken <= '1';
                 PCWriteCtrl                    <= PCWriteCtrl_WRITE_CALC;

                 PCAddrMode <= PCAddrMode_REG_DIRECT_RELATIVE;

                 PRWriteEn <= '1';

                 -- Control signals to write PC to PR.
                 SysRegSrc              <= SysRegSrc_PC;
                 SysRegCtrl <= SysRegCtrl_LOAD;
     
     
             -- JMP @Rm
             -- m format
             -- Delayed branch, Rm -> PC
             elsif std_match(IR, JMP) then
                 
                 LogWithTime(l,
                     "sh2_control.vhd: Decoded JMP @R" & to_string(slv_to_uint(m_format_m)), LogFile);

                 -- PMAU Register input is RegB.
                 RegBSel <= slv_to_uint(m_format_m);

                 DelayedBranchTaken <= '1';
                 PCWriteCtrl                    <= PCWriteCtrl_WRITE_CALC;
                 PCAddrMode         <= PCAddrMode_REG_DIRECT;
     
     
             -- JSR @Rm
             -- m format
             -- Delayed branch, PC -> PR, Rm -> PC
             elsif std_match(IR, JSR) then

                 LogWithTime(l,
                     "sh2_control.vhd: Decoded JSR @R" & to_string(slv_to_uint(m_format_m)), LogFile);

                 RegBSel <= slv_to_uint(m_format_m);

                 DelayedBranchTaken <= '1';
                 PCWriteCtrl                    <= PCWriteCtrl_WRITE_CALC;
                 PCAddrMode         <= PCAddrMode_REG_DIRECT;

                 PRWriteEn <= '1';

                 SysRegSrc <= SysRegSrc_PC;
                 SysRegCtrl <= SysRegCtrl_LOAD;
     
             elsif std_match(IR, RTS) then

                 LogWithTime(l,
                     "sh2_control.vhd: Decoded RTS", LogFile);
     
                 PCAddrMode         <= PCAddrMode_PR_DIRECT;
                 DelayedBranchTaken <= '1';
                 PCWriteCtrl                    <= PCWriteCtrl_WRITE_CALC;


            -- System Control Instructions ----------------------------------------

            elsif std_match(IR, CLRT) then

                LogWithTime(l, "sh2_control.vhd: Decoded CLRT", LogFile);

                TFlagSel <= TFlagSel_CLEAR;     -- clear the T flag

            elsif std_match(IR, CLRMAC) then

                LogWithTime(l, "sh2_control.vhd: Decoded CLRMAC", LogFile);

                SysRegCtrl <= SysRegCtrl_CLEAR;
                SysRegSel <= SysRegSel_MACL;

            elsif std_match(IR, SETT) then

                LogWithTime(l, "sh2_control.vhd: Decoded SETT", LogFile);

                TFlagSel <= TFlagSel_SET;       -- set the T flag

            elsif std_match(IR, STC_SYS_RN) then

                -- STC {SR, GBR, VBR}, Rn
                -- Uses bit decoding to choose the system register to store

                LogWithTime(l, "sh2_control.vhd: Decoded STC XXX, Rn", LogFile);

                RegInSel <= to_integer(unsigned(n_format_n));

                -- selects data source to store to a register through bit decoding
                SysRegSel <= "0" & IR(5 downto 4);
                RegDataInSel <= RegDataIn_SysReg;
                RegEnableIn <= '1';

            elsif std_match(IR, STS_SYS_RN) then

                -- STS {MACH, MACL, PR}, Rn
                -- Uses bit decoding to choose the system register to store

                LogWithTime(l, "sh2_control.vhd: Decoded STS XXX, Rn", LogFile);

                RegInSel <= to_integer(unsigned(n_format_n));

                -- selects data source to store to a register through bit decoding
                SysRegSel <= "1" & IR(5 downto 4);
                RegDataInSel <= RegDataIn_SysReg;
                RegEnableIn <= '1';

            elsif std_match(IR, STC_L_SYS_RN) then

                -- STC.L {SR, GBR, VBR}, @-Rn
                -- Uses bit decoding to choose the system register to store
                LogWithTime(l, "sh2_control.vhd: Decoded STC.L XXX, @-Rn", LogFile);

                -- Writes a byte to memory
                MemEnable <= '1';               -- Uses memory.
                ReadWrite <= ReadWrite_WRITE;   -- Writes.
                MemMode   <= LongwordMode;      -- bit decode memory mode

                -- selects data source to store to a register through bit decoding
                SysRegSel <= "0" & IR(5 downto 4);
                MemOutSel <= MemOut_SysReg;

                RegA1Sel               <= to_integer(unsigned(nm_format_n));  -- Output @(Rn) from RegA1 output.
                RegAxInSel             <= to_integer(unsigned(nm_format_n));  -- Store calculated address into Rn
                RegAxStore <= '1';                                -- Enable writes to address registers.

                -- DMAU signals (for Pre-decrement indirect register addressing)
                BaseSel      <= BaseSel_REG;
                IndexSel     <= IndexSel_NONE;
                OffScalarSel <= OffScalarSel_FOUR;
                IncDecSel    <= IncDecSel_PRE_DEC;

            elsif std_match(IR, STS_L_SYS_RN) then

                -- STC.L {MACH, MACL, PR}, @-Rn
                -- Uses bit decoding to choose the system register to store
                LogWithTime(l, "sh2_control.vhd: Decoded STC.L XXX, @-Rn", LogFile);

                -- Writes a byte to memory
                MemEnable <= '1';               -- Uses memory.
                ReadWrite <= ReadWrite_WRITE;   -- Writes.
                MemMode   <= LongwordMode;      -- bit decode memory mode

                -- selects data source to store to a register through bit decoding
                SysRegSel <= "1" & IR(5 downto 4);
                MemOutSel <= MemOut_SysReg;

                RegA1Sel               <= to_integer(unsigned(nm_format_n));  -- Output @(Rn) from RegA1 output.
                RegAxInSel             <= to_integer(unsigned(nm_format_n));  -- Store calculated address into Rn
                RegAxStore <= '1';                                -- Enable writes to address registers.

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
                    GBRWriteEn <= '1'; 
                end if;

                -- LDC Rm, {SR, GBR, VBR}
                -- Uses bit decoding to choose the system register to load

                LogWithTime(l, "sh2_control.vhd: Decoded LDC Rm, X", LogFile);

                RegBSel <= to_integer(unsigned(m_format_m));
                SysRegCtrl <= SysRegCtrl_LOAD;
                SysRegSel <= "0" & IR(5 downto 4);      -- bit decode register to select
                SysRegSrc <= SysRegSrc_RegB;

            elsif std_match(IR, LDC_L_RM_SYS) then
                -- LDC.L @Rm+, {SR, GBR, VBR}
                -- Uses bit decoding to choose the system register to load

                if (std_match(IR, LDC_L_AT_RM_PLUS_GBR)) then
                    GBRWriteEn <= '1';
                end if;

                LogWithTime(l, "sh2_control.vhd: Decoded LDC.L @Rm+, X", LogFile);

                -- Reads a longword from memory
                MemEnable <= '1';             -- Uses memory.
                ReadWrite <= ReadWrite_READ;  -- Reads.
                MemMode   <= LongwordMode;    -- bit decode memory mode

                -- Load into a system register
                SysRegCtrl <= SysRegCtrl_LOAD;
                SysRegSel <= "0" & IR(5 downto 4);    -- bit decode which system register to write to
                SysRegSrc <= SysRegSrc_DB;      -- load new register value from memory

                -- Read from @Rm, and save with post-incremented value
                RegA2Sel   <= to_integer(unsigned(m_format_m));
                RegAxInSel <= to_integer(unsigned(m_format_m));
                RegAxStore <= '1';

                -- DMAU signals (for post-increment indirect register addressing)
                BaseSel      <= BaseSel_REG;
                IndexSel     <= IndexSel_NONE;
                OffScalarSel <= OffScalarSel_FOUR;
                IncDecSel    <= IncDecSel_POST_INC;

            elsif std_match(IR, LDS_RM_SYS) then
                -- LDS Rm, {MACH, MACL, PR}
                -- Uses bit decoding to choose the system register to load

                -- Ensure that PR does actually get written to
                if (std_match(IR, LDS_RM_PR)) then
                    PRWriteEn <= '1'; 
                end if;

                LogWithTime(l, "sh2_control.vhd: Decoded LDS Rm, X", LogFile);

                RegBSel <= to_integer(unsigned(m_format_m));
                SysRegCtrl <= SysRegCtrl_LOAD;
                SysRegSel <= "1" & IR(5 downto 4);  -- bit decode register to select
                SysRegSrc <= SysRegSrc_RegB;

            elsif std_match(IR, LDS_L_RM_SYS) then
                -- LDS.L @Rm+, {MACH, MACL, PR}
                -- Uses bit decoding to choose the system register to load

                if (std_match(IR, LDS_L_AT_RM_PLUS_PR)) then
                    PRWriteEn <= '1';
                end if;

                LogWithTime(l, "sh2_control.vhd: Decoded LDS.L @Rm+, X", LogFile);

                -- Reads a longword from memory
                MemEnable <= '1';             -- Uses memory.
                ReadWrite <= ReadWrite_READ;  -- Reads.
                MemMode   <= LongwordMode;    -- bit decode memory mode

                -- Load into a system register
                SysRegCtrl <= SysRegCtrl_LOAD;
                SysRegSel <= "1" & IR(5 downto 4);    -- bit decode which system register to write to
                SysRegSrc <= SysRegSrc_DB;      -- load new register value from memory

                -- Read from @Rm, and save with post-incremented value
                RegA2Sel   <= to_integer(unsigned(m_format_m));
                RegAxInSel <= to_integer(unsigned(m_format_m));
                RegAxStore <= '1';

                -- DMAU signals (for post-increment indirect register addressing)
                BaseSel      <= BaseSel_REG;
                IndexSel     <= IndexSel_NONE;
                OffScalarSel <= OffScalarSel_FOUR;
                IncDecSel    <= IncDecSel_POST_INC;

            elsif std_match(IR, NOP) then

                LogWithTime(l, "sh2_control.vhd: Decoded NOP", LogFile);

            elsif not is_x(IR) then
                report "Unrecognized instruction: " & to_hstring(IR);
            end if;

        end if;


    end process decode_proc;

    pipeline_stages: for i in 0 to 3 generate
        process (clock, reset)
        begin
            if reset = '0' then
                pipeline(i) <= DEFAULT_CTRL;
            elsif rising_edge(clock) then
                -- TODO: should comparing stages be < or <= ??
                -- Advance stages of the pipeline that are enabled and not bubbled.
                if BubbleIF = '1' then
                    if i < STAGE_MA then
                        -- hold previous stages constant
                        pipeline(i) <= pipeline(i);
                    elsif i = STAGE_MA then
                        -- Insert a bubble at MA
                        pipeline(i) <= DEFAULT_CTRL;
                    else
                        -- The rest of the stages can move on
                        pipeline(i) <= pipeline(i - 1);
                    end if;
                else
                    if i = 0 then
                        -- First item in pipeline comes from decoded signals
                        pipeline(i) <= decoded_signals;
                    else
                        -- Next, simply pipe each set of signals into a new DFF
                        pipeline(i) <= pipeline(i - 1);
                    end if;
                end if;
            end if;
        end process;
    end generate;

    -- Register updates done on clock edges
    state_proc: process (clock, reset)
    begin
        if reset = '0' then
            IR <= NOP;
        elsif rising_edge(clock) then
            if BubbleIF = '0' then
                -- Latch in instruction from memory if no bubble. Should not
                -- occur when the IF and MA stages have contention on the
                -- memory bus (since MA takes precendence).
                IR <= MemDataIn(15 downto 0);
            end if;
        end if;
    end process state_proc;

end dataflow;
