----------------------------------------------------------------------------
--
--  Hitachi SH-2 CPU Entity Declaration
--
--  This is the entity declaration for the complete SH-2 CPU.  The design
--  should implement this entity to make testing possible.
--
--  Revision History:
--     28 Apr 25  Glen George       Initial revision.
--     01 May 25  Zack Huang        Declare all sub-unit entities
--     03 May 25  Zack Huang        Add state machine, test basic I/O
--     04 May 25  Zack Huang        Integrate memory interface
--     07 May 25  Chris Miranda     Change code formatting.
--
----------------------------------------------------------------------------


--
--  SH2_CPU
--
--  This is the complete entity declaration for the SH-2 CPU.  It is used to
--  test the complete design.
--
--  Inputs:
--    Reset  - active low reset signal
--    NMI    - active falling edge non-maskable interrupt
--    INT    - active low maskable interrupt
--    clock  - the system clock
--
--  Outputs:
--    AB     - memory address bus (32 bits)
--    RE0    - first byte read signal, active low
--    RE1    - second byte read signal, active low
--    RE2    - third byte read signal, active low
--    RE3    - fourth byte read signal, active low
--    WE0    - first byte write signal, active low
--    WE1    - second byte write signal, active low
--    WE2    - third byte write signal, active low
--    WE3    - fourth byte write signal, active low
--
--  Inputs/Outputs:
--    DB     - memory data bus (32 bits)
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.SH2PmauConstants.all;
use work.MemoryInterfaceConstants.all;
use work.SH2ControlConstants.all;

--library opcodes;
--use opcodes.opcodes.all;


entity  SH2CPU  is

    port (
        Reset   :  in     std_logic;                       -- reset signal (active low)
        NMI     :  in     std_logic;                       -- non-maskable interrupt signal (falling edge)
        INT     :  in     std_logic;                       -- maskable interrupt signal (active low)
        clock   :  in     std_logic;                       -- system clock
        AB      :  out    std_logic_vector(31 downto 0);   -- memory address bus
        memsel  :  out    std_logic;                       -- whether to access data memory (0) or program memory (1)
        RE0     :  out    std_logic;                       -- first byte active low read enable
        RE1     :  out    std_logic;                       -- second byte active low read enable
        RE2     :  out    std_logic;                       -- third byte active low read enable
        RE3     :  out    std_logic;                       -- fourth byte active low read enable
        WE0     :  out    std_logic;                       -- first byte active low write enable
        WE1     :  out    std_logic;                       -- second byte active low write enable
        WE2     :  out    std_logic;                       -- third byte active low write enable
        WE3     :  out    std_logic;                       -- fourth byte active low write enable
        DB      :  inout  std_logic_vector(31 downto 0)    -- memory data bus
    );

end  SH2CPU;

architecture structural of sh2cpu is

    -- Register array inputs
    signal RegDataIn     : std_logic_vector(31 downto 0);    -- data to write to a register
    signal EnableIn   : std_logic;                        -- if data should be written to an input register
    signal RegInSel   : integer  range 15 downto 0;       -- which register to write data to
    signal RegASel    : integer  range 15 downto 0;       -- which register to read to bus A
    signal RegBSel    : integer  range 15 downto 0;       -- which register to read to bus B
    signal RegAxIn    : std_logic_vector(31 downto 0);    -- data to write to an address register
    signal RegAxInSel : integer  range 15 downto 0;       -- which address register to write to
    signal RegAxStore : std_logic;                        -- if data should be written to the address register
    signal RegA1Sel   : integer  range 15 downto 0;       -- which register to read to address bus 1
    signal RegA2Sel   : integer  range 15 downto 0;       -- which register to read to address bus 2

    -- register array outputs
    signal RegA       : std_logic_vector(31 downto 0);    -- register bus A
    signal RegB       : std_logic_vector(31 downto 0);    -- register bus B
    signal RegA1      : std_logic_vector(31 downto 0);    -- address register bus 1
    signal RegA2      : std_logic_vector(31 downto 0);    -- address register bus 2
    
    -- ALU inputs
    signal OperandA : std_logic_vector(31 downto 0);    -- first operand
    signal OperandB : std_logic_vector(31 downto 0);    -- second operand
    signal TIn      : std_logic;                        -- T bit from status register
    signal LoadA    : std_logic;                        -- determine if OperandA is loaded ('1') or zeroed ('0')
    signal FCmd     : std_logic_vector(3 downto 0);     -- F-Block operation
    signal CinCmd   : std_logic_vector(1 downto 0);     -- carry in operation
    signal SCmd     : std_logic_vector(2 downto 0);     -- shift operation
    signal ALUCmd   : std_logic_vector(1 downto 0);     -- ALU result select

    -- ALU outputs
    signal Result   : std_logic_vector(31 downto 0);   -- ALU result
    signal Cout     : std_logic;                       -- carry out
    signal Overflow : std_logic;                       -- signed overflow
    signal Zero     : std_logic;                       -- result is zero
    signal Sign     : std_logic;                       -- sign of result

    -- DMAU inputs
    signal RegSrc       : std_logic_vector(31 downto 0);
    signal R0Src        : std_logic_vector(31 downto 0);
    signal PCSrc        : std_logic_vector(31 downto 0);
    signal GBRIn        : std_logic_vector(31 downto 0);
    signal GBRWriteEn   : std_logic;
    signal DMAUOff4     : std_logic_vector(3 downto 0);
    signal DMAUOff8     : std_logic_vector(7 downto 0);
    signal BaseSel      : std_logic_vector(1 downto 0);
    signal IndexSel     : std_logic_vector(1 downto 0);
    signal OffScalarSel : std_logic_vector(1 downto 0);
    signal IncDecSel    : std_logic_vector(1 downto 0);

    -- DMAU outputs
    signal DataAddress  : std_logic_vector(31 downto 0);
    signal AddrSrcOut   : std_logic_vector(31 downto 0);
    signal GBROut       : std_logic_vector(31 downto 0);

    -- PMAU inputs
    signal RegIn       : std_logic_vector(31 downto 0);
    signal PRIn        : std_logic_vector(31 downto 0);
    signal PCAddrMode  : std_logic_vector(2 downto 0);
    signal PRWriteEn   : std_logic;
    signal PMAUOff8    : std_logic_vector(7 downto 0);
    signal PMAUOff12   : std_logic_vector(11 downto 0);

    -- PMAU outputs
    signal PCOut       : std_logic_vector(31 downto 0);
    signal PROut       : std_logic_vector(31 downto 0);

    -- Memory interface inputs/outputs
    signal MemEnable   : std_logic;
    signal ReadWrite   : std_logic;
    signal MemMode     : std_logic_vector(1 downto 0);
    signal MemAddress  : std_logic_vector(31 downto 0);
    signal MemDataOut  : std_logic_vector(31 downto 0);
    signal ReadMask    : std_logic_vector(3 downto 0);
    signal WriteMask   : std_logic_vector(3 downto 0);
    signal MemDataIn   : std_logic_vector(31 downto 0);

    -- CPU system/control registers
    signal MemOutSel        : std_logic_vector(2 downto 0);
    signal Disp             : std_logic_vector(11 downto 0);
    signal TSel             : std_logic_vector(2 downto 0);
    signal DataRegIdx       : integer  range 15 downto 0; 
    signal ProgRegIdx       : integer  range 15 downto 0; 
    signal RegDataInSel     : std_logic_vector(1 downto 0);     -- source for register input data

    signal Immediate        : std_logic_vector(7 downto 0);     -- immediate value from instruction
    signal ImmediateExt     : std_logic_vector(31 downto 0);    -- sign-extended immediate
    signal ALUOpBSel        : std_logic;

    signal SR               : std_logic_vector(31 downto 0);
    signal GBR              : std_logic_vector(31 downto 0);
    signal VBR              : std_logic_vector(31 downto 0);

    -- Not implemented
    -- signal MACL             : std_logic_vector(31 downto 0);
    -- signal MACH             : std_logic_vector(31 downto 0);


begin

    RE0 <= ReadMask(0) when MemEnable and (not clock) else '1';
    RE1 <= ReadMask(1) when MemEnable and (not clock) else '1';
    RE2 <= ReadMask(2) when MemEnable and (not clock) else '1';
    RE3 <= ReadMask(3) when MemEnable and (not clock) else '1';

    WE0 <= WriteMask(0) when MemEnable and (not clock) else '1';
    WE1 <= WriteMask(1) when MemEnable and (not clock) else '1';
    WE2 <= WriteMask(2) when MemEnable and (not clock) else '1';
    WE3 <= WriteMask(3) when MemEnable and (not clock) else '1';

    MemAddress <= PCOut when MemSel = '1' else DataAddress;

    AB <= MemAddress;

    MemDataOut <= RegA  when MemOutSel = MemOut_RegA else
                  RegB  when MemOutSel = MemOut_RegB else
                  SR    when MemOutSel = MemOut_SR   else
                  GBR   when MemOutSel = MemOut_GBR  else
                  VBR   when MemOutSel = MemOut_VBR  else
                  PROut when MemOutSel = MemOut_PR   else
                  PCOut when MemOutSel = MemOut_PC   else
                  (others => 'X');

    ImmediateExt(7 downto 0) <= Immediate;
    ImmediateExt(31 downto 8) <= (others => Immediate(7));

    RegDataIn <= Result         when RegDataInSel = RegDataIn_ALUResult else
                 ImmediateExt   when RegDataInSel = RegDataIn_Immediate else
                 RegA           when RegDataInSel = RegDataIn_RegA else
                 RegB           when RegDataInSel = RegDataIn_RegB else
                 (others => 'X');

    -- Route control signals and data into register array
    registers : entity work.SH2Regs
    port map (
        -- Inputs:
        clock       => clock,
        reset       => reset,
        RegDataIn   => RegDataIn,
        EnableIn    => EnableIn,
        RegInSel    => RegInSel,
        RegASel     => RegASel,
        RegBSel     => RegBSel,
        RegAxIn     => RegAxIn,
        RegAxInSel  => RegAxInSel,
        RegAxStore  => RegAxStore,
        RegA1Sel    => RegA1Sel,
        RegA2Sel    => RegA2Sel,
        -- Outputs:
        RegA    => RegA,
        RegB    => RegB,
        RegA1   => RegA1,
        RegA2   => RegA2
    );


    -- ALU Input mux
    OperandA <= RegA;

    OperandB <= RegB            when ALUOpBSel = ALUOpB_RegB else
                ImmediateExt    when ALUOpBSel = ALUOpB_Imm  else
                (others => 'X');

    TIn <= SR(0);

    alu : entity work.sh2alu
    port map (
        -- Inputs:
        OperandA => OperandA,
        OperandB => OperandB,
        TIn      => TIn,
        LoadA    => LoadA,
        FCmd     => FCmd,
        CinCmd   => CinCmd,
        SCmd     => SCmd,
        ALUCmd   => ALUCmd,
        -- Outputs:
        Result   => Result,
        Cout     => Cout,
        Overflow => Overflow,
        Zero     => Zero,
        Sign     => Sign
    );


    RegSrc <= RegA1;

    dmau : entity work.sh2dmau
    port map (
        -- Inputs:
        RegSrc       => RegSrc,
        R0Src        => R0Src,
        PCSrc        => PCSrc,
        GBRIn        => GBRIn,
        GBRWriteEn   => GBRWriteEn,
        Off4         => DMAUOff4,
        Off8         => DMAUOff8,
        BaseSel      => BaseSel,
        IndexSel     => IndexSel,
        OffScalarSel => OffScalarSel,
        IncDecSel    => IncDecSel,
        Clk          => clock,

        -- Outputs:
        Address    => DataAddress,
        AddrSrcOut => AddrSrcOut,
        GBROut     => GBROut
    );

    pmau : entity work.sh2pmau
    port map (
        -- Inputs:
        RegIn      => RegIn,
        PRIn       => PRIn,
        PRWriteEn  => PRWriteEn,
        Off8       => PMAUOff8,
        Off12      => PMAUOff12,
        PCAddrMode => PCAddrMode,
        Clk        => clock,
        Reset      => Reset,
        -- Outputs:
        PCOut => PCOut,
        PROut => PROut
    );

    memory_tx : entity work.MemoryInterfaceTx
    port map (
        -- Inputs:
        ReadWrite => ReadWrite,
        MemMode   => MemMode,
        Address   => unsigned(MemAddress),
        data_in   => MemDataOut,
        -- Outputs:
        RE => ReadMask,
        WE => WriteMask,
        DB => DB
    );

    memory_rx : entity work.MemoryInterfaceRx
    port map (
        -- Inputs:
        MemMode => MemMode,
        Address => unsigned(MemAddress),
        DB      => DB,
        -- Outputs:
        data => MemDataIn
    );

    control_unit : entity work.SH2Control
    port map (
        -- Inputs:
        DB    => MemDataIn,
        clock => clock,
        reset => reset,

        -- Outputs:
        Immediate    => Immediate,

        -- Memory interface control signals:
        MemEnable    => MemEnable,
        ReadWrite    => ReadWrite,
        MemMode      => MemMode,
        Disp         => Disp,
        MemSel       => MemSel,
        MemOutSel    => MemOutSel,

        -- ALU control signals:
        ALUOpBSel    => ALUOpBSel,
        LoadA        => LoadA,
        FCmd         => FCmd,
        CinCmd       => CinCmd,
        SCmd         => SCmd,
        ALUCmd       => ALUCmd,
        TSel         => TSel,

        -- Register Array control signals:
        RegDataInSel => RegDataInSel,
        EnableIn     => EnableIn,
        RegInSel     => RegInSel,
        RegASel      => RegASel,
        RegBSel      => RegBSel,
        RegAxIn      => RegAxIn,
        RegAxInSel   => RegAxInSel,
        RegAxStore   => RegAxStore,
        RegA1Sel     => RegA1Sel,
        RegA2Sel     => RegA2Sel,

        -- DMAU control signals:
        DataRegIdx   => DataRegIdx,
        GBRWriteEn   => GBRWriteEn,
        DMAUOff4     => DMAUOff4,
        DMAUOff8     => DMAUOff8,
        BaseSel      => BaseSel,
        IndexSel     => IndexSel,
        OffScalarSel => OffScalarSel,
        IncDecSel    => IncDecSel,

        -- PMAU control signals:
        ProgRegIdx   => ProgRegIdx,
        PCAddrMode   => PCAddrMode,
        PRWriteEn    => PRWriteEn,
        PMAUOff8     => PMAUOff8,
        PMAUOff12    => PMAUOff12
    );

    register_proc: process(clock, reset)
    begin
        if reset = '0' then
            SR <=  (others => '0');
            GBR <=  (others => '0');
            VBR <=  (others => '0');
        elsif rising_edge(clock) then
            null;
        end if;
    end process register_proc;

end architecture structural;
