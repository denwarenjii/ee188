----------------------------------------------------------------------------
--
--  Memory Interface
--
--  TODO
--
--  Revision History:
--     03 May 25  Zack Huang        Implement memory interface for byte,
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
