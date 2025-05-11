------------------------------------------------------------------------------
-- SH2Regs
--
-- This entity implements the general-purpose registers for the SH-2 CPU. The
-- SH-2 contains 16 registers, numbered R0 through R15, each 32 bits wide.
-- These registers can be used for both ALU instructions and memory addressing.
-- To allow the CPU to perform ALU operations and memory accesses in parallel,
-- two dual-port memory interfaces are provided. Each interface supports
-- writing to one register and reading from two registers in a single clock.
-- Writing to the same register through both of these interfaces should be
-- avoided, but if it occurs, then the "regular" interface will take
-- precendence over the memory addressing interface.
--
------------------------------------------------------------------------------

-- import libraries
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity SH2Regs is
    port (
        RegDataIn  : in   std_logic_vector(31 downto 0);    -- data to write to a register
        EnableIn   : in   std_logic;                        -- if data should be written to an input register
        RegInSel   : in   integer  range 15 downto 0;       -- which register to write data to
        RegASel    : in   integer  range 15 downto 0;       -- which register to read to bus A
        RegBSel    : in   integer  range 15 downto 0;       -- which register to read to bus B
        RegAxIn    : in   std_logic_vector(31 downto 0);    -- data to write to an address register
        RegAxInSel : in   integer  range 15 downto 0;       -- which address register to write to
        RegAxStore : in   std_logic;                        -- if data should be written to the address register
        RegA1Sel   : in   integer  range 15 downto 0;       -- which register to read to address bus 1
        RegA2Sel   : in   integer  range 15 downto 0;       -- which register to read to address bus 2
        clock      : in   std_logic;                        -- system clock
        reset      : in   std_logic;                        -- system reset (async, active low)
        RegA       : out  std_logic_vector(31 downto 0);    -- register bus A
        RegB       : out  std_logic_vector(31 downto 0);    -- register bus B
        RegA1      : out  std_logic_vector(31 downto 0);    -- address register bus 1
        RegA2      : out  std_logic_vector(31 downto 0)     -- address register bus 2
    );
end SH2Regs;

architecture structural of SH2Regs is

    component  RegArray  is

        generic (
            regcnt   : integer := 32;    -- default number of registers is 32
            wordsize : integer := 8      -- default width is 8-bits
        );

        port(
            RegIn      : in   std_logic_vector(wordsize - 1 downto 0);      -- input bus to the registers
            RegInSel   : in   integer  range regcnt - 1 downto 0;           -- which register to write (log regcnt bits)
            RegStore   : in   std_logic;                                    -- actually write to a register
            RegASel    : in   integer  range regcnt - 1 downto 0;           -- register to read onto bus A (log regcnt bits)
            RegBSel    : in   integer  range regcnt - 1 downto 0;           -- register to read onto bus B (log regcnt bits)
            RegAxIn    : in   std_logic_vector(wordsize - 1 downto 0);      -- input bus for address register updates
            RegAxInSel : in   integer  range regcnt - 1 downto 0;           -- which address register to write (log regcnt bits - 1)
            RegAxStore : in   std_logic;                                    -- actually write to an address register
            RegA1Sel   : in   integer  range regcnt - 1 downto 0;           -- register to read onto address bus 1 (log regcnt bits)
            RegA2Sel   : in   integer  range regcnt - 1 downto 0;           -- register to read onto address bus 2 (log regcnt bits)
            RegDIn     : in   std_logic_vector(2 * wordsize - 1 downto 0);  -- input bus to the double-width registers
            RegDInSel  : in   integer  range regcnt/2 - 1 downto 0;         -- which double register to write (log regcnt bits - 1)
            RegDStore  : in   std_logic;                                    -- actually write to a double register
            RegDSel    : in   integer  range regcnt/2 - 1 downto 0;         -- register to read onto double width bus D (log regcnt bits)
            clock      : in   std_logic;                                    -- the system clock
            reset      : in   std_logic;                                    -- system reset (async, active low)
            RegA       : out  std_logic_vector(wordsize - 1 downto 0);      -- register value for bus A
            RegB       : out  std_logic_vector(wordsize - 1 downto 0);      -- register value for bus B
            RegA1      : out  std_logic_vector(wordsize - 1 downto 0);      -- register value for address bus 1
            RegA2      : out  std_logic_vector(wordsize - 1 downto 0);      -- register value for address bus 2
            RegD       : out  std_logic_vector(2 * wordsize - 1 downto 0)   -- register value for bus D (double width bus)
        );

    end  component;
begin

    -- Specialize the provided generic register array entity for the SH-2 CPU.
    -- We will use the memory interfaces exactly as implemented, passing the
    -- bits through directly. We will use RegIn, RegInSel, RegA, and RegB as
    -- the "normal" memory interface used for reading ALU operands and writing
    -- ALU results. We will use RegAXIn, RegAxSel, RegA1, and RegA2 for
    -- register accesses dedicated to memory addressing. We must be able to
    -- read up to two registers to get indirect indexed register addressing (R0
    -- + Rn), and we should also be able to update address register values to
    -- implement pre/post increment/decrement.
    Registers: RegArray
    generic map (
        wordsize => 32,
        regcnt => 16
    )
    port map(
        clock => clock,
        reset => reset,
        -- dual register access for ALU operations
        RegIn => RegDataIn,
        RegInSel => RegInSel,
        RegStore => EnableIn,
        RegASel => RegASel,
        RegBSel => RegBSel,
        RegA => RegA,
        RegB => RegB,
        -- dual register access for indirect/relative memory access
        RegAxIn => RegAxIn,
        RegAxInSel => RegAxInSel,
        RegAxStore => RegAxStore,
        RegA1Sel => RegA1Sel,
        RegA2Sel => RegA2Sel,
        RegA1 => RegA1,
        RegA2 => RegA2,
        -- unused
        RegDIn => (others => '0'),
        RegDInSel => 0,
        RegDStore => '0',
        RegDSel => 0
    );
end structural;
