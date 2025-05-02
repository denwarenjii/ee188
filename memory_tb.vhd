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

entity memory_tb is
end memory_tb;

architecture behavioral of memory_tb is
    -- Stimulus signals for unit under test
    signal AB      :  std_logic_vector(31 downto 0);   -- memory address bus
    signal RE0     :  std_logic;                       -- first byte active low read enable
    signal RE1     :  std_logic;                       -- second byte active low read enable
    signal RE2     :  std_logic;                       -- third byte active low read enable
    signal RE3     :  std_logic;                       -- fourth byte active low read enable
    signal WE0     :  std_logic;                       -- first byte active low write enable
    signal WE1     :  std_logic;                       -- second byte active low write enable
    signal WE2     :  std_logic;                       -- third byte active low write enable
    signal WE3     :  std_logic;                       -- fourth byte active low write enable

    -- Outputs from unit under test
    signal DB      :  std_logic_vector(31 downto 0);   -- memory data bus

    -- Test signals
    signal END_SIM : boolean    := false;   -- if the simulation should end

begin

    -- Instantiate UUT
    UUT : entity work.MEMORY32x32
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

        procedure Reset is
        begin
            WE0 <= '1';
            WE1 <= '1';
            WE2 <= '1';
            WE3 <= '1';
            RE0 <= '1';
            RE1 <= '1';
            RE2 <= '1';
            RE3 <= '1';
            wait for 5 ns;
        end procedure;

        procedure WriteByte(address : integer; data : integer) is
        begin
            AB <= std_logic_vector(to_unsigned(address, 32));
            -- Shift byte of data over to correct location
            DB <= std_logic_vector(to_unsigned(data, 32) sll (8 * (address mod 4)));
            -- Write only the byte being addressed
            WE0 <= '0' when address mod 4 = 0 else '1';
            WE1 <= '0' when address mod 4 = 1 else '1';
            WE2 <= '0' when address mod 4 = 2 else '1';
            WE3 <= '0' when address mod 4 = 3 else '1';

            wait for 5 ns;  -- wait for signal to propagate

            -- Disable writing
            WE0 <= '1';
            WE1 <= '1';
            WE2 <= '1';
            WE3 <= '1';

            wait for 5 ns;  -- wait for signal to propagate
        end procedure;

        procedure ReadByte(address : integer ; data : out integer) is
            variable data_bits : std_logic_vector(31 downto 0);
        begin
            AB <= std_logic_vector(to_unsigned(address, 32));
            DB <= (others => 'Z');  -- Data bus unused, don't set
            -- Read only the byte being addressed
            RE0 <= '0' when address mod 4 = 0 else '1';
            RE1 <= '0' when address mod 4 = 1 else '1';
            RE2 <= '0' when address mod 4 = 2 else '1';
            RE3 <= '0' when address mod 4 = 3 else '1';

            wait for 5 ns;  -- wait for signal to propagate

            -- Shift the desired byte to the bottom 8 bits
            data_bits := DB srl (8 * (address mod 4));
            data := to_integer(unsigned(data_bits(7 downto 0)));

            -- Disable writing
            RE0 <= '1';
            RE1 <= '1';
            RE2 <= '1';
            RE3 <= '1';
            wait for 5 ns;  -- wait for signal to propagate
        end procedure;


        procedure WriteWord(address : integer; data : integer) is
        begin
            AB <= std_logic_vector(to_unsigned(address * 2, 32));
            -- Shift byte of data over to correct location
            DB <= std_logic_vector(to_unsigned(data, 32) sll (16 * (address mod 2)));
            -- Write only the word being addressed
            WE0 <= '0' when address mod 2 = 0 else '1';
            WE1 <= '0' when address mod 2 = 0 else '1';
            WE2 <= '0' when address mod 2 = 1 else '1';
            WE3 <= '0' when address mod 2 = 1 else '1';

            wait for 5 ns;  -- wait for signal to propagate

            -- Disable writing
            WE0 <= '1';
            WE1 <= '1';
            WE2 <= '1';
            WE3 <= '1';

            wait for 5 ns;  -- wait for signal to propagate
        end procedure;

        procedure ReadWord(address : integer ; data : out integer) is
            variable data_bits : std_logic_vector(31 downto 0);
        begin
            AB <= std_logic_vector(to_unsigned(address * 2, 32));
            DB <= (others => 'Z');  -- Data bus unused, don't set
            -- Read only the byte being addressed
            RE0 <= '0' when address mod 2 = 0 else '1';
            RE1 <= '0' when address mod 2 = 0 else '1';
            RE2 <= '0' when address mod 2 = 1 else '1';
            RE3 <= '0' when address mod 2 = 1 else '1';

            wait for 5 ns;  -- wait for signal to propagate

            -- Shift the desired byte to the bottom 8 bits
            data_bits := DB srl (16 * (address mod 2));
            data := to_integer(unsigned(data_bits(15 downto 0)));

            -- Disable writing
            RE0 <= '1';
            RE1 <= '1';
            RE2 <= '1';
            RE3 <= '1';
            wait for 5 ns;  -- wait for signal to propagate
        end procedure;

        procedure WriteLongword(address : integer; data : integer) is
        begin
            AB <= std_logic_vector(to_unsigned(address * 4, 32));
            -- Shift byte of data over to correct location
            DB <= std_logic_vector(to_unsigned(data, 32));
            -- Write only the word being addressed
            WE0 <= '0';
            WE1 <= '0';
            WE2 <= '0';
            WE3 <= '0';

            wait for 5 ns;  -- wait for signal to propagate

            -- Disable writing
            WE0 <= '1';
            WE1 <= '1';
            WE2 <= '1';
            WE3 <= '1';

            wait for 5 ns;  -- wait for signal to propagate
        end procedure;

        procedure ReadLongword(address : integer ; data : out integer) is
        begin
            AB <= std_logic_vector(to_unsigned(address * 4, 32));
            DB <= (others => 'Z');  -- Data bus unused, don't set
            -- Read only the byte being addressed
            RE0 <= '0';
            RE1 <= '0';
            RE2 <= '0';
            RE3 <= '0';

            wait for 5 ns;  -- wait for signal to propagate

            data := to_integer(unsigned(DB));

            -- Disable writing
            RE0 <= '1';
            RE1 <= '1';
            RE2 <= '1';
            RE3 <= '1';
            wait for 5 ns;  -- wait for signal to propagate
        end procedure;

        variable data_out : integer;
    begin
        Reset;

        -- Write to each byte of memory
        for j in 0 to 3 loop
            for i in 0 to 255 loop
                WriteByte(i + 256 * j, i);
            end loop;
        end loop;

        -- Check that we read the correct values back
        for i in 0 to 1023 loop
            ReadByte(i, data_out);
            assert (i mod 256) = data_out
            report "Byte: Expected integer at address " & to_string(i) & " to be " &
                   to_string(i mod 256) & ", got " & to_string(data_out)
            severity error;
        end loop;

        -- Write to every word in memory
        for i in 0 to 511 loop
            WriteWord(i, i);
        end loop;

        -- Check that we read the correct values back
        for i in 0 to 511 loop
            ReadWord(i, data_out);
            assert i = data_out
            report "Word: Expected integer at address " & to_string(i) & " to be " &
                   to_string(i) & ", got " & to_string(data_out)
            severity error;
        end loop;

        -- Write to every longword in memory
        for i in 0 to 255 loop
            WriteLongword(i, i);
        end loop;

        -- Check that we read the correct values back
        for i in 0 to 255 loop
            ReadLongword(i, data_out);
            assert i = data_out
            report "Longword: Expected integer at address " & to_string(i) & " to be " &
                   to_string(i) & ", got " & to_string(data_out)
            severity error;
        end loop;

        END_SIM <= TRUE;
        wait;
    end process;
end behavioral;

