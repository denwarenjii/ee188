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
--     04 May 25  Zack Huang        Implement memory interface for byte,
--                                  word, and longword read/writes.
--
----------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package MemoryInterfaceConstants is

  constant ByteMode         : std_logic_vector(1 downto 0) := "00";
  constant WordMode         : std_logic_vector(1 downto 0) := "10";
  constant LongwordMode     : std_logic_vector(1 downto 0) := "11";

end package MemoryInterfaceConstants;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.MemoryInterfaceConstants.all;

-- Outputs the necessary flags and data bits to read/write a byte, word, or longword to memory.
entity MemoryInterfaceTx is
    port (
        ReadWrite :  in     std_logic;                          -- memory read (0) or write (1)
        MemMode   :  in     STD_LOGIC_VECTOR(1 downto 0);       -- memory access mode (byte, word, or longword)
        Address   :  in     unsigned(31 downto 0);              -- memory address bus
        data_in   :  in     std_logic_vector(31 downto 0);      -- the input to write to memory
        RE        :  out    std_logic_vector(3 downto 0);       -- read enable mask (active low)
        WE        :  out    std_logic_vector(3 downto 0);       -- write enable mask (active low)
        DB        :  out    std_logic_vector(31 downto 0)       -- memory data bus
    );
end entity;


architecture structural of MemoryInterfaceTx is
begin
    output_proc: process(ReadWrite, MemMode, Address, data_in)
    begin
        if ReadWrite = '0' then
            -- Disable writing
            WE(3 downto 0) <= (others => '1');
            DB <= (others => 'Z');

            -- Enable specific bytes based on type of read
            case MemMode is
                when ByteMode =>
                    RE(0) <= '0' when address mod 4 = 0 else '1';
                    RE(1) <= '0' when address mod 4 = 1 else '1';
                    RE(2) <= '0' when address mod 4 = 2 else '1';
                    RE(3) <= '0' when address mod 4 = 3 else '1';

                when WordMode =>
                    RE(0) <= '0' when address mod 4 = 0 else '1';
                    RE(1) <= '0' when address mod 4 = 0 else '1';
                    RE(2) <= '0' when address mod 4 = 2 else '1';
                    RE(3) <= '0' when address mod 4 = 2 else '1';

                when LongwordMode =>
                    RE(3 downto 0) <= (others => '0');

                when others =>
                    assert (false)
                    report "Memory interface Tx: Invalid memory mode for read"
                    severity error;
            end case;

        elsif ReadWrite = '1' then
            -- Disable reading
            RE(3 downto 0) <= (others => '1');

            -- Enable specific bytes based on type of read
            case MemMode is
                when ByteMode =>
                    WE(0) <= '0' when address mod 4 = 0 else '1';
                    WE(1) <= '0' when address mod 4 = 1 else '1';
                    WE(2) <= '0' when address mod 4 = 2 else '1';
                    WE(3) <= '0' when address mod 4 = 3 else '1';
                    -- TODO: may not synthesize efficiently, use conditionals instead?
                    DB <= std_logic_vector(unsigned(data_in) sll to_integer(8 * (address mod 4)));

                when WordMode =>
                    WE(0) <= '0' when address mod 4 = 0 else '1';
                    WE(1) <= '0' when address mod 4 = 0 else '1';
                    WE(2) <= '0' when address mod 4 = 2 else '1';
                    WE(3) <= '0' when address mod 4 = 2 else '1';
                    -- TODO: may not synthesize efficiently, use conditionals instead?
                    DB <= std_logic_vector(unsigned(data_in) sll to_integer(8 * (address mod 4)));

                when LongwordMode =>
                    WE(3 downto 0) <= (others => '0');
                    DB <= data_in;

                when others =>
                    assert (false)
                    report "Memory interface Tx: Invalid memory mode for write"
                    severity error;
            end case;

        end if;

    end process output_proc;
end architecture;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.MemoryInterfaceConstants.all;

-- Performs shifting to read a byte, word, or longword from a data bus, sign-extended if necessary.
entity MemoryInterfaceRx is
    port (
        MemMode :  in     std_logic_vector(1 downto 0);     -- memory access mode (byte, word, or longword)
        Address :  in     unsigned(31 downto 0);            -- memory address bus
        DB      :  in     std_logic_vector(31 downto 0);    -- memory data bus
        data    :  out    std_logic_vector(31 downto 0)     -- data read from memory
    );
end entity;

architecture structural of MemoryInterfaceRx is
begin
    output_proc: process(MemMode, Address, DB)
    begin
        -- Shift and sign-extend based on the mode
        case MemMode is
            when ByteMode =>
                if (Address mod 4 = 0) then
                    data(7 downto 0) <= DB(7 downto 0);
                    data(31 downto 8) <= (others => DB(7));
                elsif (Address mod 4 = 1) then
                    data(7 downto 0) <= DB(15 downto 8);
                    data(31 downto 8) <= (others => DB(15));
                elsif (Address mod 4 = 2) then
                    data(7 downto 0) <= DB(23 downto 16);
                    data(31 downto 8) <= (others => DB(23));
                elsif (Address mod 4 = 3) then
                    data(7 downto 0) <= DB(31 downto 24);
                    data(31 downto 8) <= (others => DB(31));
                end if;
                    
            when WordMode =>
                if (Address mod 4 = 0) then
                    data(15 downto 0) <= DB(15 downto 0);
                    data(31 downto 16) <= (others => DB(15));
                elsif (Address mod 4 = 2) then
                    data(15 downto 0) <= DB(31 downto 16);
                    data(31 downto 16) <= (others => DB(31));
                end if;

            when LongwordMode =>
                data <= DB;

            when others =>
                assert (false)
                report "Memory interface Rx: Invalid memory mode for read"
                severity error;
        end case;

    end process output_proc;
end architecture;

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
    signal DMAUOff4     : std_logic_vector(3 downto 0);
    signal DMAUOff8     : std_logic_vector(7 downto 0);
    signal BaseSel      : std_logic_vector(1 downto 0);
    signal IndexSel     : std_logic_vector(1 downto 0);
    signal OffScalarSel : std_logic_vector(1 downto 0);
    signal IncDecSel    : std_logic_vector(1 downto 0);

    -- DMAU outputs
    signal DataAddress      : std_logic_vector(31 downto 0);
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

    -- Testing basic CPU functionality
    type state_t is (
        fetch,
        decode,
        execute
    );

    signal state : state_t;

    signal IR : std_logic_vector(15 downto 0);

begin

    PCAddrMode <= PCAddrMode_INC when state = execute else PCAddrMode_HOLD;

    RE0 <= ReadMask(0) when MemEnable and (not clock) else '1';
    RE1 <= ReadMask(1) when MemEnable and (not clock) else '1';
    RE2 <= ReadMask(2) when MemEnable and (not clock) else '1';
    RE3 <= ReadMask(3) when MemEnable and (not clock) else '1';

    WE0 <= WriteMask(0) when MemEnable and (not clock) else '1';
    WE1 <= WriteMask(1) when MemEnable and (not clock) else '1';
    WE2 <= WriteMask(2) when MemEnable and (not clock) else '1';
    WE3 <= WriteMask(3) when MemEnable and (not clock) else '1';

    AB <= MemAddress;

    -- outputs based on the current CPU state
    output_proc: process(clock, state)
    begin
        if state = fetch then
            -- Fetch an instruction word from ROM
            MemEnable <= '1';
            memsel <= '1';
            MemAddress <= PCOut;
            ReadWrite <= '0';
            MemMode <= WordMode;
        elsif state = decode then
            -- Testing if writing to memory works
            MemEnable <= '1';
            memsel <= '0';
            MemAddress <= PCOut;
            ReadWrite <= '1';
            MemMode <= WordMode;
            MemDataOut <= PCOut;
        elsif state = execute then
            MemEnable <= '0';
        end if;
    end process output_proc;


    -- Register updates done on clock edges
    state_proc: process (clock, reset)
    begin
        if reset = '0' then
            state <= fetch;
        elsif rising_edge(clock) then
            if state = fetch then
                state <= decode;
                IR <= MemDataIn(15 downto 0); -- latch in instruction from memory
            elsif state = decode then
                report "Decoding instruction: " & to_hstring(IR);
                state <= execute;
            elsif state = execute then
                state <= fetch;
            end if;
        end if;
    end process state_proc;

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
        Off4 => DMAUOff4,
        Off8 => DMAUOff8,
        BaseSel => BaseSel,
        IndexSel => IndexSel,
        OffScalarSel => OffScalarSel,
        IncDecSel => IncDecSel,
        Clk => clock,
        Address => DataAddress,
        AddrSrcOut => AddrSrcOut,
        GBROut => GBROut
    );

    pmau : entity work.sh2pmau
    port map (
        RegIn => RegIn,
        PRIn => PRIn,
        PRWriteEn => PRWriteEn,
        Off8 => PMAUOff8,
        Off12 => PMAUOff12,
        PCAddrMode => PCAddrMode,
        Clk => clock,
        reset => reset,
        PCOut => PCOut,
        PROut => PROut
    );

    memory_tx : entity work.MemoryInterfaceTx
    port map (
        ReadWrite => ReadWrite,
        MemMode => MemMode,
        Address => unsigned(MemAddress),
        data_in => MemDataOut,
        RE => ReadMask,
        WE => WriteMask,
        DB => DB
    );

    memory_rx : entity work.MemoryInterfaceRx
    port map (
        MemMode => MemMode,
        Address => unsigned(MemAddress),
        DB => DB,
        data => MemDataIn
    );
    
end architecture structural;
