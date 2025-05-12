----------------------------------------------------------------------------
--
--  TODO
-- 
--  Revision History:
--     01 May 25    Zack Huang      initial revision
--     03 May 25    Zack Huang      working with data/program memory units
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
    signal CPU_AB  :  std_logic_vector(31 downto 0);   -- program memory address bus
    signal CPU_RE0     :  std_logic;                       -- first byte active low read enable
    signal CPU_RE1     :  std_logic;                       -- second byte active low read enable
    signal CPU_RE2     :  std_logic;                       -- third byte active low read enable
    signal CPU_RE3     :  std_logic;                       -- fourth byte active low read enable
    signal CPU_WE0     :  std_logic;                       -- first byte active low write enable
    signal CPU_WE1     :  std_logic;                       -- second byte active low write enable
    signal CPU_WE2     :  std_logic;                       -- third byte active low write enable
    signal CPU_WE3     :  std_logic;                       -- fourth byte active low write enable
    signal CPU_DB      :  std_logic_vector(31 downto 0);   -- memory data bus
    signal CPU_MEMSEL  :  std_logic;                       -- if should access data memory (0) or program memory (1)

    -- test signals used to read/write the RAM independently of the CPU
    signal TEST_AB      :  std_logic_vector(31 downto 0);   -- memory address bus
    signal TEST_RE0     :  std_logic;                       -- first byte active low read enable
    signal TEST_RE1     :  std_logic;                       -- second byte active low read enable
    signal TEST_RE2     :  std_logic;                       -- third byte active low read enable
    signal TEST_RE3     :  std_logic;                       -- fourth byte active low read enable
    signal TEST_WE0     :  std_logic;                       -- first byte active low write enable
    signal TEST_WE1     :  std_logic;                       -- second byte active low write enable
    signal TEST_WE2     :  std_logic;                       -- third byte active low write enable
    signal TEST_WE3     :  std_logic;                       -- fourth byte active low write enable
    signal TEST_DB      :  std_logic_vector(31 downto 0);   -- memory data bus
    signal TEST_MEMSEL  :  std_logic;                       -- if should access data memory (0) or program memory (1)

    -- Memory control signals
    signal RAM_RE0     :  std_logic;                       -- first byte active low read enable
    signal RAM_RE1     :  std_logic;                       -- second byte active low read enable
    signal RAM_RE2     :  std_logic;                       -- third byte active low read enable
    signal RAM_RE3     :  std_logic;                       -- fourth byte active low read enable
    signal RAM_WE0     :  std_logic;                       -- first byte active low write enable
    signal RAM_WE1     :  std_logic;                       -- second byte active low write enable
    signal RAM_WE2     :  std_logic;                       -- third byte active low write enable
    signal RAM_WE3     :  std_logic;                       -- fourth byte active low write enable
    signal RAM_DB      :  std_logic_vector(31 downto 0);   -- data memory data bus
    signal RAM_AB      :  std_logic_vector(31 downto 0);   -- data memory address bus

    signal ROM_RE0     :  std_logic;                       -- first byte active low read enable
    signal ROM_RE1     :  std_logic;                       -- second byte active low read enable
    signal ROM_RE2     :  std_logic;                       -- third byte active low read enable
    signal ROM_RE3     :  std_logic;                       -- fourth byte active low read enable
    signal ROM_WE0     :  std_logic;                       -- first byte active low write enable
    signal ROM_WE1     :  std_logic;                       -- second byte active low write enable
    signal ROM_WE2     :  std_logic;                       -- third byte active low write enable
    signal ROM_WE3     :  std_logic;                       -- fourth byte active low write enable
    signal ROM_DB      :  std_logic_vector(31 downto 0);   -- program memory data bus
    signal ROM_AB      :  std_logic_vector(31 downto 0);   -- program memory address bus

    signal CPU_ACTIVE  :  boolean    := false;   -- if the cpu outputs or test signals should be routed into the memory units

    signal CPU_RD      :  std_logic;
    signal CPU_WR      :  std_logic;
    signal TEST_RD     :  std_logic;
    signal TEST_WR     :  std_logic;

    -- Test signals
    signal END_SIM    : boolean    := false;    -- if the simulation should end

begin

    CPU_RD  <= CPU_RE0 and CPU_RE1 and CPU_RE2 and CPU_RE3;
    TEST_RD <= TEST_RE0 and TEST_RE1 and TEST_RE2 and TEST_RE3;

    CPU_WR  <= CPU_WE0 and CPU_WE1 and CPU_WE2 and CPU_WE3;
    TEST_WR <= TEST_WE0 and TEST_WE1 and TEST_WE2 and TEST_WE3;

    ROM_AB <= CPU_AB when CPU_ACTIVE else TEST_AB;
    RAM_AB <= CPU_AB when CPU_ACTIVE else TEST_AB;

    ROM_DB <= CPU_DB  when     CPU_ACTIVE and CPU_WR  = '0' and CPU_MEMSEL  = '1'  else
              TEST_DB when not CPU_ACTIVE and TEST_WR = '0' and TEST_MEMSEL = '1' else
              (others => 'Z');

    RAM_DB <= CPU_DB  when     CPU_ACTIVE and CPU_WR  = '0' and CPU_MEMSEL  = '0'  else
              TEST_DB when not CPU_ACTIVE and TEST_WR = '0' and TEST_MEMSEL = '0' else
              (others => 'Z');

    CPU_DB <= RAM_DB when CPU_MEMSEL = '0' and CPU_RD = '0' else
              ROM_DB when CPU_MEMSEL = '1' and CPU_RD = '0' else
              (others => 'Z');

    TEST_DB <= RAM_DB when TEST_MEMSEL = '0' and TEST_RD = '0' else
               ROM_DB when TEST_MEMSEL = '1' and TEST_RD = '0' else
              (others => 'Z');

    ROM_WE0 <= '1' when CPU_ACTIVE else
               TEST_WE0 when TEST_MEMSEL = '1' else
               '1';

    ROM_WE1 <= '1' when CPU_ACTIVE else
               TEST_WE1 when TEST_MEMSEL = '1' else
               '1';

    ROM_WE2 <= '1' when CPU_ACTIVE else
               TEST_WE2 when TEST_MEMSEL = '1' else
               '1';

    ROM_WE3 <= '1' when CPU_ACTIVE else
               TEST_WE3 when TEST_MEMSEL = '1' else
               '1';

    ROM_RE0 <= CPU_RE0 when CPU_ACTIVE and CPU_MEMSEL = '1' else
               TEST_RE0 when not CPU_ACTIVE and TEST_MEMSEL = '1' else
               '1';

    ROM_RE1 <= CPU_RE1 when CPU_ACTIVE and CPU_MEMSEL = '1' else
               TEST_RE1 when not CPU_ACTIVE and TEST_MEMSEL = '1' else
               '1';

    ROM_RE2 <= CPU_RE2 when CPU_ACTIVE and CPU_MEMSEL = '1' else
               TEST_RE2 when not CPU_ACTIVE and TEST_MEMSEL = '1' else
               '1';

    ROM_RE3 <= CPU_RE3 when CPU_ACTIVE and CPU_MEMSEL = '1' else
               TEST_RE3 when not CPU_ACTIVE and TEST_MEMSEL = '1' else
               '1';


    RAM_WE0 <= CPU_WE0 when CPU_ACTIVE and CPU_MEMSEL = '0' else
               TEST_WE0 when not CPU_ACTIVE and TEST_MEMSEL = '0' else
               '1';

    RAM_WE1 <= CPU_WE1 when CPU_ACTIVE and CPU_MEMSEL = '0' else
               TEST_WE1 when not CPU_ACTIVE and TEST_MEMSEL = '0' else
               '1';

    RAM_WE2 <= CPU_WE2 when CPU_ACTIVE and CPU_MEMSEL = '0' else
               TEST_WE2 when not CPU_ACTIVE and TEST_MEMSEL = '0' else
               '1';

    RAM_WE3 <= CPU_WE3 when CPU_ACTIVE and CPU_MEMSEL = '0' else
               TEST_WE3 when not CPU_ACTIVE and TEST_MEMSEL = '0' else
               '1';

    RAM_RE0 <= CPU_RE0 when CPU_ACTIVE and CPU_MEMSEL = '0' else
               TEST_RE0 when not CPU_ACTIVE and TEST_MEMSEL = '0' else
               '1';

    RAM_RE1 <= CPU_RE1 when CPU_ACTIVE and CPU_MEMSEL = '0' else
               TEST_RE1 when not CPU_ACTIVE and TEST_MEMSEL = '0' else
               '1';

    RAM_RE2 <= CPU_RE2 when CPU_ACTIVE and CPU_MEMSEL = '0' else
               TEST_RE2 when not CPU_ACTIVE and TEST_MEMSEL = '0' else
               '1';

    RAM_RE3 <= CPU_RE3 when CPU_ACTIVE and CPU_MEMSEL = '0' else
               TEST_RE3 when not CPU_ACTIVE and TEST_MEMSEL = '0' else
               '1';

    -- Instantiate UUT
    UUT: entity work.sh2cpu
    port map (
        Reset => Reset,
        NMI => NMI,
        INT => INT,
        clock => clock,
        AB => CPU_AB,
        RE0 => CPU_RE0,
        RE1 => CPU_RE1,
        RE2 => CPU_RE2,
        RE3 => CPU_RE3,
        WE0 => CPU_WE0,
        WE1 => CPU_WE1,
        WE2 => CPU_WE2,
        WE3 => CPU_WE3,
        DB => CPU_DB,
        memsel => CPU_MEMSEL
    );

    ram : entity work.MEMORY32x32
    generic map (
        MEMSIZE => 1024,
        -- four contiguous blocks of memory (1024 bytes each)
        START_ADDR0 => 16#0000#,
        START_ADDR1 => 16#1000#,
        START_ADDR2 => 16#2000#,
        START_ADDR3 => 16#3000#
    )
    port map (
        RE0 => RAM_RE0,
        RE1 => RAM_RE1,
        RE2 => RAM_RE2,
        RE3 => RAM_RE3,
        WE0 => RAM_WE0,
        WE1 => RAM_WE1,
        WE2 => RAM_WE2,
        WE3 => RAM_WE3,
        MemAB => RAM_AB,
        MemDB => RAM_DB
    );

    rom : entity work.MEMORY32x32
    generic map (
        MEMSIZE => 1024,
        -- four contiguous blocks of memory (1024 bytes each)
        START_ADDR0 => 16#0000#,
        START_ADDR1 => 16#1000#,
        START_ADDR2 => 16#2000#,
        START_ADDR3 => 16#3000#
    )
    port map (
        RE0 => ROM_RE0,
        RE1 => ROM_RE1,
        RE2 => ROM_RE2,
        RE3 => ROM_RE3,
        WE0 => ROM_WE0,
        WE1 => ROM_WE1,
        WE2 => ROM_WE2,
        WE3 => ROM_WE3,
        MemAB => ROM_AB,
        MemDB => ROM_DB
    );

    process

        -- Assumes that address is word-aligned
        procedure WriteWord(address : unsigned; data : std_logic_vector) is
        begin
            TEST_AB <= std_logic_vector(address);

            -- Shift word of data over to correct location
            TEST_DB(15 downto 0)  <= data when address mod 4 = 0 else (others => 'X');
            TEST_DB(31 downto 16) <= data when address mod 4 = 2 else (others => 'X');

            -- Write only the word being addressed
            TEST_WE0 <= '0' when address mod 4 = 0 else '1';
            TEST_WE1 <= '0' when address mod 4 = 0 else '1';
            TEST_WE2 <= '0' when address mod 4 = 2 else '1';
            TEST_WE3 <= '0' when address mod 4 = 2 else '1';

            wait for 5 ns;  -- wait for signal to propagate

            -- Disable writing
            TEST_WE0 <= '1';
            TEST_WE1 <= '1';
            TEST_WE2 <= '1';
            TEST_WE3 <= '1';

            wait for 5 ns;  -- wait for signal to propagate
        end procedure;

        procedure ReadWord(address : unsigned ; data : out std_logic_vector) is
        begin
            TEST_AB <= std_logic_vector(address);
            TEST_DB <= (others => 'Z');  -- Data bus unused, don't set
            -- Read only the bytes being addressed
            TEST_RE0 <= '0' when address mod 4 = 0 else '1';
            TEST_RE1 <= '0' when address mod 4 = 0 else '1';
            TEST_RE2 <= '0' when address mod 4 = 2 else '1';
            TEST_RE3 <= '0' when address mod 4 = 2 else '1';

            wait for 5 ns;  -- wait for signal to propagate

            -- Shift the desired byte to the bottom 8 bits
            data := TEST_DB(15 downto 0) when address mod 4 = 0 else TEST_DB(31 downto 16);

            -- Disable writing
            TEST_RE0 <= '1';
            TEST_RE1 <= '1';
            TEST_RE2 <= '1';
            TEST_RE3 <= '1';
            wait for 5 ns;  -- wait for signal to propagate
        end procedure;

        -- assumes address is longword-aligned
        procedure ReadLongword(address : unsigned ; data : out std_logic_vector) is
        begin
            TEST_AB <= std_logic_vector(address);
            TEST_DB <= (others => 'Z');  -- Data bus unused, don't set
            -- Read all 4 bytes of longword
            TEST_RE0 <= '0';
            TEST_RE1 <= '0';
            TEST_RE2 <= '0';
            TEST_RE3 <= '0';

            wait for 5 ns;  -- wait for signal to propagate

            -- Shift the desired byte to the bottom 8 bits
            data := TEST_DB;

            -- Disable writing
            TEST_RE0 <= '1';
            TEST_RE1 <= '1';
            TEST_RE2 <= '1';
            TEST_RE3 <= '1';
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
            -- Write to ROM
            CPU_ACTIVE <= false;
            TEST_MEMSEL <= '1';

            curr_pc := to_unsigned(0, 32);

            -- read file as "characters" to get individual bytes
            file_open(char_file, path & ".bin");
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

        procedure ReadMemory(start : integer; length : integer) is
            variable curr_pc     : unsigned(31 downto 0);
            variable data_out    : std_logic_vector(15 downto 0);
        begin
            curr_pc := to_unsigned(start, 32);
            for i in 1 to length loop
                ReadWord(curr_pc, data_out);
                report "Data: " & to_hstring(data_out);
                curr_pc := curr_pc + 2;
            end loop;
        end procedure;

        procedure CheckOutput(path : string) is
            file test_file : text;
            variable row   : line;

            variable address : unsigned(31 downto 0);
            variable expected_value : std_logic_vector(31 downto 0);
            variable actual_value : std_logic_vector(31 downto 0);
        begin
            -- Access RAM
            CPU_ACTIVE <= false;
            TEST_MEMSEL <= '0';

            file_open(test_file, path & ".expect", read_mode);
            while not endfile(test_file) loop
                -- Read expected address/value pairs from test file
                readline(test_file, row);
                hread(row, address);
                hread(row, expected_value);

                -- Read value at address from RAM
                ReadLongword(address, actual_value);

                -- Convert from BE (RAM) to LE (test file)
                actual_value := swap_bytes(actual_value);

                -- Check that the values match up
                assert expected_value = actual_value
                    report path & ": expected " & to_hstring(expected_value) & " at address " &
                           to_hstring(address) & ", got " & to_hstring(actual_value) & " instead."
                    severity error;
            end loop;
        end procedure;

        procedure Tick is
        begin
            clock <= '0';
            wait for 10 ns;
            clock <= '1';
            wait for 10 ns;
        end procedure;

        -- We define the CPU exit signal to be when it tries to access
        -- address 0xFFFFFFFC (signed integer representation of -4);
        impure function CheckDone return boolean is
        begin
            return CPU_AB = X"FFFFFFFC";
        end function;

        procedure RunCPU is
        begin
            -- Give memory control to CPU
            CPU_ACTIVE <= true;

            -- Reset CPU
            reset <= '0';
            Tick;

            -- Run program until finished
            reset <= '1';
            while not CheckDone loop
                Tick;
            end loop;
        end procedure;

        procedure RunTest(path : string) is
        begin
            report "Running test: " & path;
            LoadProgram(path);      -- write program in to ROM
            RunCPU;                 -- execute program
            CheckOutput(path);      -- check RAM has expected values
        end procedure;

    begin
        RunTest("asm/hello");
        RunTest("asm/mov_reg");
        RunTest("asm/reg_indirect");
        RunTest("asm/arith");
        RunTest("asm/sr");

        wait;
    end process;
end behavioral;

