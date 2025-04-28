----------------------------------------------------------------------------
--
--  TODO
-- 
--  Revision History:
--     28 Apr 25    Zack Huang      initial revision
--
----------------------------------------------------------------------------

-- import libraries
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.textio.all;

entity sh2_reg_tb is
end sh2_reg_tb;

architecture behavioral of sh2_reg_tb is
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
    signal clock      : std_logic;                        -- system clock
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
        clock => clock,
        RegA => RegA,
        RegB => RegB,
        RegA1 => RegA1,
        RegA2 => RegA2
    );

    process
    begin
        report "Hello, world!";
        END_SIM <= TRUE;
        wait;
    end process;
end behavioral;
