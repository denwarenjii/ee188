----------------------------------------------------------------------------
--
--  TODO
-- 
--  Revision History:
--     26 Apr 25    Zack Huang      initial revision
--
----------------------------------------------------------------------------

-- import libraries
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.textio.all;

entity sh2alu_tb is
end sh2alu_tb;

architecture behavioral of sh2alu_tb is
    -- Stimulus signals for unit under test
    signal OperandA : std_logic_vector(31 downto 0);   -- first operand
    signal OperandB : std_logic_vector(31 downto 0);   -- second operand
    signal TIn      : std_logic;                       -- T bit from status register
    signal LoadA    : std_logic;                       -- determine if OperandA is loaded ('1') or zeroed ('0')
    signal FCmd     : std_logic_vector(3 downto 0);    -- F-Block operation
    signal CinCmd   : std_logic_vector(1 downto 0);    -- carry in operation
    signal SCmd     : std_logic_vector(2 downto 0);    -- shift operation
    signal ALUCmd   : std_logic_vector(1 downto 0);    -- ALU result select

    -- Outputs from unit under test
    signal Result   : std_logic_vector(31 downto 0);   -- ALU result
    signal Cout     : std_logic;                       -- carry out
    signal Overflow : std_logic;                       -- signed overflow
    signal Zero     : std_logic;                       -- result is zero
    signal Sign     : std_logic;                       -- sign of result

    -- Test signals
    signal END_SIM : boolean    := false;   -- if the simulation should end

begin
    -- Instantiate UUT
    UUT: entity work.sh2alu
    port map(
        OperandA => OperandA,
        OperandB => OperandB,
        TIn      => TIn,
        LoadA    => LoadA,
        FCmd     => FCmd,
        CinCmd   => CinCmd,
        SCmd     => SCmd,
        ALUCmd   => ALUCmd,
        Result   => Result,
        Cout     => Cout,
        Overflow => Overflow,
        Zero     => Zero,
        Sign     => Sign
    );

    process
    begin
        report "Hello, world!";
        -- End simulation
        END_SIM <= TRUE;
        wait;
    end process;
end behavioral;
