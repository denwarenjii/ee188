----------------------------------------------------------------------------
--
--  Generic Register Array
--
--  This is an implementation of a Register Array for the register-based
--  microprocessors.  It allows the registers to be accessed as single words
--  or double words.  Multiple interfaces to the registers are allowed so they
--  may be simultaneously used as ALU registers and address registers.  Double
--  word access may be used for addressing (typically used in 8-bit
--  processors) for example.
--
--  Entities included are:
--     RegArray  - the register array
--
--  Revision History:
--     25 Jan 21  Glen George       Initial revision.
--     11 Apr 25  Glen George       Added separate address register interface.
--
----------------------------------------------------------------------------


--
--  RegArray
--
--  This is a generic register array.  It contains regcnt wordsize bit
--  registers along with the appropriate reading and writing controls.  The
--  registers can also be read and written as double width registers.  There
--  is also two separate access ports and a write port to allow the registers
--  to be used as address registers simultaneous to their use in other blocks
--  such as the ALU.
--
--  Generics:
--    regcnt   - number of registers in the array (must be a multiple of 2)
--    wordsize - width of each register
--
--  Inputs:
--    RegIn      - input bus to the registers
--    RegInSel   - which register to write (log regcnt bits)
--    RegStore   - actually write to a register
--    RegASel    - register to read onto bus A (log regcnt bits)
--    RegBSel    - register to read onto bus B (log regcnt bits)
--    RegAxIn    - input bus for address register updates
--    RegAxInSel - which address register to write (log regcnt bits - 1)
--    RegAxStore - actually write to an address register
--    RegA1Sel   - register to read onto address bus 1 (log regcnt bits)
--    RegA2Sel   - register to read onto address bus 2 (log regcnt bits)
--    RegDIn     - input bus to the double-width registers
--    RegDInSel  - which double register to write (log regcnt bits - 1)
--    RegDStore  - actually write to a double register
--    RegDSel    - register to read onto double width bus D (log regcnt bits)
--    clock      - the system clock
--
--  Outputs:
--    RegA       - register value for bus A
--    RegB       - register value for bus B
--    RegA1      - register value for address bus 1
--    RegA2      - register value for address bus 2
--    RegD       - register value for bus D (double width bus)
--

library ieee;
use ieee.std_logic_1164.all;

entity  RegArray  is

    generic (
        regcnt   : integer := 32;    -- default number of registers is 32
        wordsize : integer := 8      -- default width is 8-bits
    );

    port(
        RegIn      : in   std_logic_vector(wordsize - 1 downto 0);
        RegInSel   : in   integer  range regcnt - 1 downto 0;
        RegStore   : in   std_logic;
        RegASel    : in   integer  range regcnt - 1 downto 0;
        RegBSel    : in   integer  range regcnt - 1 downto 0;
        RegAxIn    : in   std_logic_vector(wordsize - 1 downto 0);
        RegAxInSel : in   integer  range regcnt - 1 downto 0;
        RegAxStore : in   std_logic;
        RegA1Sel   : in   integer  range regcnt - 1 downto 0;
        RegA2Sel   : in   integer  range regcnt - 1 downto 0;
        RegDIn     : in   std_logic_vector(2 * wordsize - 1 downto 0);
        RegDInSel  : in   integer  range regcnt/2 - 1 downto 0;
        RegDStore  : in   std_logic;
        RegDSel    : in   integer  range regcnt/2 - 1 downto 0;
        clock      : in   std_logic;
        RegA       : out  std_logic_vector(wordsize - 1 downto 0);
        RegB       : out  std_logic_vector(wordsize - 1 downto 0);
        RegA1      : out  std_logic_vector(wordsize - 1 downto 0);
        RegA2      : out  std_logic_vector(wordsize - 1 downto 0);
        RegD       : out  std_logic_vector(2 * wordsize - 1 downto 0)
    );

end  RegArray;


architecture  behavioral  of  RegArray  is

    type  RegType  is array (regcnt - 1 downto 0) of
                      std_logic_vector(wordsize - 1 downto 0);

    signal  Registers : RegType;                -- the register array

    -- aliases for the upper and lower input word
    alias  RegDInHigh : std_logic_vector(wordsize - 1 downto 0) IS
                        RegDIn(2 * wordsize - 1 downto wordsize);
    alias  RegDInLow  : std_logic_vector(wordsize - 1 downto 0) IS
                        RegDIn(wordsize - 1 downto 0);

begin

    -- setup the outputs - choose based on select signals
    RegA   <=  Registers(RegASel);
    RegB   <=  Registers(RegBSel);
    RegA1  <=  Registers(RegA1Sel);
    RegA2  <=  Registers(RegA2Sel);
    RegD   <=  Registers(2 * RegDSel + 1) & Registers(2 * RegDSel);


    -- only write registers on the clock
    process(clock)
    begin

        -- by default leave all registers with their current value
        Registers  <=  Registers;

        -- if storing, update that register on the clock
        if  rising_edge(clock)  then

            -- handle double word stores
            if (RegDStore = '1')  then
                Registers(2 * RegDInSel + 1)  <=  RegDInHigh;
                Registers(2 * RegDInSel)      <=  RegDInLow;
            end if;

            -- handle address register stores second so they take precedence
            --    over double word stores
            if (RegAxStore = '1')  then
                Registers(RegAxInSel)  <=  RegAxIn;
            end if;

            -- handle normal stores last so they have highest precedence
            if (RegStore = '1')  then
                Registers(RegInSel)  <=  RegIn;
            end if;
        end if;

    end process;

end  behavioral;
