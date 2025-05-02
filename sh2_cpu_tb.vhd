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
        MEMSIZE => 1024,
        -- four contiguous blocks of memory (1024 bytes each)
        START_ADDR0 => 16#0000#,
        START_ADDR1 => 16#1000#,
        START_ADDR2 => 16#2000#,
        START_ADDR3 => 16#3000#
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

        -- Assumes that address is word-aligned
        procedure WriteWord(address : unsigned; data : std_logic_vector) is
        begin
            AB <= std_logic_vector(address);

            -- Shift word of data over to correct location
            DB(15 downto 0)  <= data when address mod 4 = 0 else (others => 'X');
            DB(31 downto 16) <= data when address mod 4 = 2 else (others => 'X');

            -- Write only the word being addressed
            WE0 <= '0' when address mod 4 = 0 else '1';
            WE1 <= '0' when address mod 4 = 0 else '1';
            WE2 <= '0' when address mod 4 = 2 else '1';
            WE3 <= '0' when address mod 4 = 2 else '1';

            wait for 5 ns;  -- wait for signal to propagate

            -- Disable writing
            WE0 <= '1';
            WE1 <= '1';
            WE2 <= '1';
            WE3 <= '1';

            wait for 5 ns;  -- wait for signal to propagate
        end procedure;

        -- Reading in a binary file byte-by-byte
        -- Reference: https://stackoverflow.com/a/42581872
        procedure LoadProgram(path : string) is
            type char_file_t is file of character;
            file char_file : char_file_t;
            variable char_v : character;
            subtype byte_t is natural range 0 to 255;
            variable byte_v : byte_t;

            variable curr_opcode : std_logic_vector(15 downto 0);
            variable curr_pc     : unsigned(31 downto 0);
        begin
            curr_pc := to_unsigned(0, 32);

            -- read file as "characters" to get individual bytes
            file_open(char_file, path);
            while not endfile(char_file) loop
                -- read low byte of instruction
                read(char_file, char_v);
                byte_v := character'pos(char_v);
                curr_opcode(7 downto 0) := std_logic_vector(to_unsigned(byte_v, 8));

                -- read high byte of instruction
                read(char_file, char_v);
                byte_v := character'pos(char_v);
                curr_opcode(15 downto 8) := std_logic_vector(to_unsigned(byte_v, 8));

                -- Write instruction word into memory
                WriteWord(curr_pc, curr_opcode);

                curr_pc := curr_pc + 2;
            end loop;
            file_close(char_file);
        end procedure;

        procedure Tick is
        begin
            clock <= '0';
            wait for 10 ns;
            clock <= '1';
            wait for 10 ns;
        end procedure;

    begin
        report "Hello, world!";

        LoadProgram("asm/hello.bin");

        END_SIM <= TRUE;
        wait;
    end process;
end behavioral;

