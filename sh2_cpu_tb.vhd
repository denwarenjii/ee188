----------------------------------------------------------------------------
--
--  TODO
-- 
--  Revision History:
--     01 May 25    Zack Huang      initial revision
--
----------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.textio.all;

use work.sh2utils.all;

entity sh2_cpu_tb is
end sh2_cpu_tb;

architecture behavioral of sh2_cpu_tb is
    -- Stimulus signals for unit under test
    signal Reset   :  std_logic;                       -- reset signal (active low)
    signal NMI     :  std_logic;                       -- non-maskable interrupt signal (falling edge)
    signal INT     :  std_logic;                       -- maskable interrupt signal (active low)
    signal clock   :  std_logic;                       -- system clock

    -- Outputs from unit under test
    signal AB      :  std_logic_vector(31 downto 0);   -- memory address bus
    signal RE0     :  std_logic;                       -- first byte active low read enable
    signal RE1     :  std_logic;                       -- second byte active low read enable
    signal RE2     :  std_logic;                       -- third byte active low read enable
    signal RE3     :  std_logic;                       -- fourth byte active low read enable
    signal WE0     :  std_logic;                       -- first byte active low write enable
    signal WE1     :  std_logic;                       -- second byte active low write enable
    signal WE2     :  std_logic;                       -- third byte active low write enable
    signal WE3     :  std_logic;                       -- fourth byte active low write enable
    signal DB      :  std_logic_vector(31 downto 0);   -- memory data bus

    -- Test signals
    signal END_SIM : boolean    := false;   -- if the simulation should end

begin
    -- Instantiate UUT
    UUT: entity work.sh2cpu
    port map(
        Reset => Reset,
        NMI => NMI,
        INT => INT,
        clock => clock,
        AB => AB,
        RE0 => RE0,
        RE1 => RE1,
        RE2 => RE2,
        RE3 => RE3,
        WE0 => WE0,
        WE1 => WE1,
        WE2 => WE2,
        WE3 => WE3,
        DB => DB
    );

    mem : entity work.MEMORY32x32
    generic map (
        MEMSIZE => 256,
        -- four contiguous blocks of memory (256 bytes each)
        START_ADDR0 => 16#000#,
        START_ADDR1 => 16#100#,
        START_ADDR2 => 16#200#,
        START_ADDR3 => 16#300#
    )
    port map (
        RE0 => RE0,
        RE1 => RE1,
        RE2 => RE2,
        RE3 => RE3,
        WE0 => WE0,
        WE1 => WE1,
        WE2 => WE2,
        WE3 => WE3,
        MemAB => AB,
        MemDB => DB
    );

    process

        procedure Tick is
        begin
            clock <= '0';
            wait for 10 ns;
            clock <= '1';
            wait for 10 ns;
        end procedure;

    begin
        report "Hello, world!";

        END_SIM <= TRUE;
        wait;
    end process;
end behavioral;

