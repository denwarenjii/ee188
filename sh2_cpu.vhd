----------------------------------------------------------------------------
--
--  Hitachi SH-2 CPU Entity Declaration
--
--  This is the entity declaration for the complete SH-2 CPU.  The design
--  should implement this entity to make testing possible.
--
--  Revision History:
--     28 Apr 25  Glen George       Initial revision.
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
use ieee.std_logic_arith.all;
use ieee.numeric_std.all;

--library opcodes;
--use opcodes.opcodes.all;


entity  SH2CPU  is

    port (
        Reset   :  in     std_logic;                       -- reset signal (active low)
        NMI     :  in     std_logic;                       -- non-maskable interrupt signal (falling edge)
        INT     :  in     std_logic;                       -- maskable interrupt signal (active low)
        clock   :  in     std_logic;                       -- system clock
        AB      :  out    std_logic_vector(31 downto 0);   -- memory address bus
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
    signal DataIn     : std_logic_vector(31 downto 0);    -- data to write to a register
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
    signal OperandA : std_logic_vector(31 downto 0); -- first operand
    signal OperandB : std_logic_vector(31 downto 0); -- second operand
    signal TIn      : std_logic;                     -- T bit from status register
    signal LoadA    : std_logic;                     -- determine if OperandA is loaded ('1') or zeroed ('0')
    signal FCmd     : std_logic_vector(3 downto 0);  -- F-Block operation
    signal CinCmd   : std_logic_vector(1 downto 0);  -- carry in operation
    signal SCmd     : std_logic_vector(2 downto 0);  -- shift operation
    signal ALUCmd   : std_logic_vector(1 downto 0);  -- ALU result select

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
    signal Off4         : std_logic_vector(3 downto 0);
    signal Off8         : std_logic_vector(7 downto 0);
    signal BaseSel      : std_logic_vector(1 downto 0);
    signal IndexSel     : std_logic_vector(1 downto 0);
    signal OffScalarSel : std_logic_vector(1 downto 0);
    signal IncDecSel    : std_logic_vector(1 downto 0);

    -- DMAU outputs
    signal Address      : std_logic_vector(31 downto 0);
    signal AddrSrcOut   : std_logic_vector(31 downto 0);
    signal GBROut       : std_logic_vector(31 downto 0);

    -- PMAU inputs
    signal RegIn       : std_logic_vector(31 downto 0);
    signal PRIn        : std_logic_vector(31 downto 0);
    signal AddrOffset  : std_logic_vector(31 downto 0);
    signal Disp        : std_logic_vector(11 downto 0);
    signal PCAddrMode  : std_logic_vector(2 downto 0);

    -- PMAU outputs
    signal PCOut       : std_logic_vector(31 downto 0);
    signal PROut       : std_logic_vector(31 downto 0);

    -- CPU system/control registers
begin

    -- Route control signals and data into register array
    registers : entity work.SH2Regs
    port map (
        clock => clock,
        reset => reset,
        DataIn => DataIn,
        EnableIn => EnableIn,
        RegInSel => RegInSel,
        RegASel => RegASel,
        RegBSel => RegBSel,
        RegAxIn => RegAxIn,
        RegAxInSel => RegAxInSel,
        RegAxStore => RegAxStore,
        RegA1Sel => RegA1Sel,
        RegA2Sel => RegA2Sel,
        RegA => RegA,
        RegB => RegB,
        RegA1 => RegA1,
        RegA2 => RegA2
    );

    alu : entity work.sh2alu
    port map (
        OperandA => OperandA,
        OperandB => OperandB,
        TIn => TIn,
        LoadA => LoadA,
        FCmd => FCmd,
        CinCmd => CinCmd,
        SCmd => SCmd,
        ALUCmd => ALUCmd,
        Result => Result,
        Cout => Cout,
        Overflow => Overflow,
        Zero => Zero,
        Sign => Sign
    );

    dmau : entity work.sh2dmau
    port map (
        RegSrc => RegSrc,
        R0Src => R0Src,
        PCSrc => PCSrc,
        GBRIn => GBRIn,
        GBRWriteEn => GBRWriteEn,
        Off4 => Off4,
        Off8 => Off8,
        BaseSel => BaseSel,
        IndexSel => IndexSel,
        OffScalarSel => OffScalarSel,
        IncDecSel => IncDecSel,
        Clk => clock,
        Address => Address,
        AddrSrcOut => AddrSrcOut,
        GBROut => GBROut
    );

    pmau : entity work.sh2pmau
    port map (
        RegIn => RegIn,
        PRIn => PRIn,
        AddrOffset => AddrOffset,
        Disp => Disp,
        PCAddrMode => PCAddrMode,
        Clk => clock,
        reset => reset,
        PCOut => PCOut,
        PROut => PROut
    );

    -- proc_name: process(clock, reset)
    -- begin
    --     -- asyncronous low reset
    --     if reset = '1' then
    --
    --     elsif rising_edge(clk) then
    --
    --     end if;
    -- end process proc_name;
    
end architecture structural;
