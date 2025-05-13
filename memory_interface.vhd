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
  constant WordMode         : std_logic_vector(1 downto 0) := "01";
  constant LongwordMode     : std_logic_vector(1 downto 0) := "10";

end package MemoryInterfaceConstants;


library ieee;
library std;

use std.textio.all;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.MemoryInterfaceConstants.all;
use work.sh2utils.all;
use work.Logging.all;

-- Outputs the necessary flags and data bits to read/write a byte, word, or longword to memory.
entity MemoryInterfaceTx is
    port (
        clock     :  in     std_logic;                          -- system clock
        MemEnable :  in     std_logic;                          -- if memory interface should be active or not
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

    signal data_in_BE : std_logic_vector(31 downto 0);

begin

    -- Re-order bytes since CPU is big-endian but registers are little-endian internally
    -- data_in_BE(7 downto 0) <= data_in(15 downto 8);
    -- data_in_BE(15 downto 8) <= data_in(7 downto 0);
    -- data_in_BE(23 downto 16) <= data_in(31 downto 24);
    -- data_in_BE(31 downto 24) <= data_in(23 downto 16);

    data_in_BE <= swap_bytes(data_in);

    output_proc: process(MemEnable, ReadWrite, MemMode, Address, data_in_BE, clock)
      variable l : line;
    begin
        if MemEnable = '1' and clock = '0' and not is_x(address) then
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

                        LogWithTime(l, "memory_interface.vhd: Reading byte at address " & to_hstring(address), LogFile);

                    when WordMode =>
                        assert (address mod 2 = 0)
                        report "Memory interface Tx: Cannot read word from non-aligned address: " & to_hstring(address)
                        severity error;

                        RE(0) <= '0' when address mod 4 = 0 else '1';
                        RE(1) <= '0' when address mod 4 = 0 else '1';
                        RE(2) <= '0' when address mod 4 = 2 else '1';
                        RE(3) <= '0' when address mod 4 = 2 else '1';

                        LogWithTime(l, "memory_interface.vhd: Reading word at address " & to_hstring(address), LogFile);


                    when LongwordMode =>
                        assert (address mod 4 = 0)
                        report "Memory interface Tx: Cannot read longword from non-aligned address: " & to_hstring(address)
                        severity error;

                        RE(3 downto 0) <= (others => '0');

                        LogWithTime(l, "memory_interface.vhd: Reading longword at address " & to_hstring(address), LogFile);

                    when others =>
                        assert (false)
                        report "Memory interface Tx: unrecognized read mode" & to_hstring(address)
                        severity error;

                        -- When unrecognized mode, don't read/write anything
                        RE <= (others => '1');
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
                        -- Note: not using data_in_BE since reading/writing individual byte
                        
                        LogWithTime(l, "memory_interface.vhd: Writing byte at address " & to_hstring(address), LogFile);

                        DB <= std_logic_vector(unsigned(data_in) sll to_integer(8 * (address mod 4)));

                    when WordMode =>
                        assert (address mod 2 = 0)
                        report "Memory interface Tx: Cannot write word to non-aligned address: " & to_hstring(address)
                        severity error;

                        WE(0) <= '0' when address mod 4 = 0 else '1';
                        WE(1) <= '0' when address mod 4 = 0 else '1';
                        WE(2) <= '0' when address mod 4 = 2 else '1';
                        WE(3) <= '0' when address mod 4 = 2 else '1';

                        LogWithTime(l, "memory_interface.vhd: Writing word at address " & to_hstring(address), LogFile);

                        -- TODO: may not synthesize efficiently, use conditionals instead?
                        DB <= std_logic_vector(unsigned(data_in_BE) sll to_integer(8 * (address mod 4)));

                    when LongwordMode =>
                        assert (address mod 4 = 0)
                        report "Memory interface Tx: Cannot write longword to non-aligned address: " & to_hstring(address)
                        severity error;

                        LogWithTime(l, "memory_interface.vhd: Writing longword at address " & to_hstring(address), LogFile);

                        WE(3 downto 0) <= (others => '0');
                        DB <= data_in_BE;

                    when others =>
                        WE(3 downto 0) <= (others => '1');
                        assert (false)
                        report "Memory interface Tx: Invalid memory mode for write"
                        severity error;
                end case;
            end if;
        else
            -- should not enable memory interface
            RE <= (others => '1');
            WE <= (others => '1');
        end if;

    end process output_proc;
end architecture;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.MemoryInterfaceConstants.all;
use work.sh2utils.all;

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
    signal data_BE : std_logic_vector(31 downto 0);
begin
    output_proc: process(MemMode, Address, DB)
    begin
        -- Shift and sign-extend based on the mode
        case MemMode is
            when ByteMode =>
                if (Address mod 4 = 0) then
                    data_BE(7 downto 0) <= DB(7 downto 0);
                    data_BE(31 downto 8) <= (others => DB(7));
                elsif (Address mod 4 = 1) then
                    data_BE(7 downto 0) <= DB(15 downto 8);
                    data_BE(31 downto 8) <= (others => DB(15));
                elsif (Address mod 4 = 2) then
                    data_BE(7 downto 0) <= DB(23 downto 16);
                    data_BE(31 downto 8) <= (others => DB(23));
                elsif (Address mod 4 = 3) then
                    data_BE(7 downto 0) <= DB(31 downto 24);
                    data_BE(31 downto 8) <= (others => DB(31));
                end if;
                    
            when WordMode =>
                if (Address mod 4 = 0) then
                    data_BE(15 downto 0) <= DB(15 downto 0);
                    data_BE(31 downto 16) <= (others => DB(15));
                elsif (Address mod 4 = 2) then
                    data_BE(15 downto 0) <= DB(31 downto 16);
                    data_BE(31 downto 16) <= (others => DB(31));
                end if;

            when LongwordMode =>
                data_BE <= DB;

            when others =>
                -- When invalid memory mode, don't read anything
                data_BE <= (others => 'X');
        end case;

    end process output_proc;

    -- Re-order bytes since CPU is big-endian but registers are little-endian internally
    data <= swap_bytes(data_BE);

end architecture;
