----------------------------------------------------------------------------
--
--  TODO
-- 
--  Revision History:
--     28 Apr 25    Zack Huang      initial revision
--
----------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.textio.all;

use work.utils.all;

entity sh2_reg_tb is
end sh2_reg_tb;

architecture behavioral of sh2_reg_tb is
    -- Stimulus signals for unit under test
    signal RegDataIn  : std_logic_vector(31 downto 0);    -- data to write to a register
    signal EnableIn   : std_logic;                        -- if data should be written to an input register
    signal RegInSel   : integer  range 15 downto 0;       -- which register to write data to
    signal RegASel    : integer  range 15 downto 0;       -- which register to read to bus A
    signal RegBSel    : integer  range 15 downto 0;       -- which register to read to bus B
    signal RegAxIn    : std_logic_vector(31 downto 0);    -- data to write to an address register
    signal RegAxInSel : integer  range 15 downto 0;       -- which address register to write to
    signal RegAxStore : std_logic;                        -- if data should be written to the address register
    signal RegA1Sel   : integer  range 15 downto 0;       -- which register to read to address bus 1
    signal RegA2Sel   : integer  range 15 downto 0;       -- which register to read to address bus 2
    signal clock      : std_logic;                        -- system clock
    signal reset      : std_logic;                        -- system reset (async, active low)

    -- Outputs from unit under test
    signal RegA       : std_logic_vector(31 downto 0);    -- register bus A
    signal RegB       : std_logic_vector(31 downto 0);    -- register bus B
    signal RegA1      : std_logic_vector(31 downto 0);    -- address register bus 1
    signal RegA2      : std_logic_vector(31 downto 0);    -- address register bus 2

    -- Test signals
    signal END_SIM : boolean    := false;   -- if the simulation should end

begin
    -- Instantiate UUT
    UUT: entity work.sh2regs
    port map(
        RegDataIn => RegDataIn,
        EnableIn => EnableIn,
        RegInSel => RegInSel,
        RegASel => RegASel,
        RegBSel => RegBSel,
        RegAxIn => RegAxIn,
        RegAxInSel => RegAxInSel,
        RegAxStore => RegAxStore,
        RegA1Sel => RegA1Sel,
        RegA2Sel => RegA2Sel,
        clock => clock,
        reset => reset,
        RegA => RegA,
        RegB => RegB,
        RegA1 => RegA1,
        RegA2 => RegA2
    );

    process

        procedure Tick is
        begin
            clock <= '0';
            wait for 10 ns;
            clock <= '1';
            wait for 10 ns;
        end procedure;

        procedure Read(
            rn : integer  range 15 downto 0;
            rm : integer  range 15 downto 0
        ) is
        begin
            -- Read both Rn and Rm
            RegASel <= rn;
            RegBSel <= rm;
            wait for 5 ns;  -- wait for signal to propagate, no clock tick required
        end procedure;

        procedure Write(
            r : integer  range 15 downto 0;
            Data : std_logic_vector
        ) is
        begin
            -- Write to a register
            RegInSel <= r;
            EnableIn <= '1';
            RegDataIn <= Data;
            Tick;               -- clock write op into register array
            EnableIn <= '0';    -- Don't accidentally write more than expected
        end procedure;

        type reg_values is array (natural range <>) of std_logic_vector(31 downto 0);

        variable regs : reg_values(0 to 15);
        variable val  : std_logic_vector(31 downto 0);

        variable random : rng;
    begin
        reset <= '0';   -- start system reset
        Tick;           -- propagate signal
        reset <= '1';   -- finished system reset

        -- Write random values to each register
        for i in 0 to 15 loop
            Write(i, random.rand_slv(32));
        end loop;

        -- Write a random value to each register
        for i in 0 to 15 loop
            regs(i) := random.rand_slv(32);
            Write(i, regs(i));
        end loop;

        -- Read each register to check for the correct values (through RegA)
        for i in 0 to 15 loop
            Read(i, 0);
            assert RegA = regs(i)
            report "Expected R" & to_string(i) & " = " & to_hstring(regs(i)) & ", found " & to_hstring(RegA)
            severity error;
        end loop;

        -- Increment and cycle the registers
        for i in 0 to 15 loop
            Read(i, (i + 1) mod 16);
            regs(i) := std_logic_vector(i + unsigned(regs((i + 1) mod 16)));
            Write(i, std_logic_vector(i + unsigned(RegB)));
        end loop;

        -- Check that all of the registers hold the correct values (through RegB)
        for i in 0 to 15 loop
            Read(0, i);
            assert RegB = regs(i)
            report "Expected R" & to_string(i) & " = " & to_hstring(regs(i)) & ", found " & to_hstring(RegA)
            severity error;
        end loop;

        END_SIM <= TRUE;
        wait;
    end process;
end behavioral;
