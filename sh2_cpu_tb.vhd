  ----------------------------------------------------------------------------
--
-- 
--  Revision History:
--     01 May 25    Zack Huang      initial revision
--     03 May 25    Zack Huang      working with data/program memory units
--
-- TODO:
--  - Support loading into program memory with assembly directives.
--  - Clear memory between tests (or set to X).
----------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.textio.all;

use work.sh2utils.all;
use work.Logging.all;
use work.ANSIEscape.all;

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


    -- vectors to keep track of written memory locations
    signal RAMSegment0Writes : std_logic_vector(0 to 1023) := (others => '0');
    signal RAMSegment1Writes : std_logic_vector(0 to 1023) := (others => '0');
    signal RAMSegment2Writes : std_logic_vector(0 to 1023) := (others => '0');
    signal RAMSegment3Writes : std_logic_vector(0 to 1023) := (others => '0');

    function IntToHString(i : integer) return string is
      variable slv     : std_logic_vector(31 downto 0);
      variable hex_str : string(1 to 8);
    begin
      slv := std_logic_vector(to_unsigned(i, 32));
      hex_str := to_hstring(slv);
      return hex_str;
    end function;


    file MemLogFile : text open write_mode is "mem_log.txt";

    -- procedure LogWrites(seg_no : integer range 0 to 3) is

    --   variable l : line;
    --   variable slv : std_logic_vector(31 downto 0);
    --   variable hex_str : string(1 to 8) := (others => ' ');

    -- begin

    --   case seg_no is

    --     when 0 =>
    --       for i in 0 to 1023 loop

    --         if ((i mod 8) = 0) then
    --           writeline(MemLogFile, l);
    --           write(l, string'(IntToHString(i) & ":"  & HT));
    --         -- elsif ((i mod 2) = 0) then
    --         end if;

    --         if (RAMSegment0Writes(i) = '0') then
    --           write(l, string'("UU"));
    --           write(l, string'(" "));
    --         else
    --           write(l, string'(GREEN & "!" & ANSI_RESET));
    --         end if;

    --       end loop;

    --     when 1 =>
    --       -- TODO:
    --       null;

    --     when 2 =>
    --       -- TODO:
    --       null;

    --     when 3 =>
    --       -- TODO:
    --       null;

    --   end case;

    -- end procedure;


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

    LogWithTime("sh2_cpu_tb.vhd: [RAM] Initializing memory from byte " & 
                to_string(16#0000#) & " to " & to_string(16#0000# + 1024), LogFile);
    LogWithTime("sh2_cpu_tb.vhd: [RAM] Initializing memory from byte " & 
                to_string(16#1000#) & " to " & to_string(16#1000# + 1024), LogFile);
    LogWithTime("sh2_cpu_tb.vhd: [RAM] Initializing memory from byte " & 
                to_string(16#2000#) & " to " & to_string(16#2000# + 1024), LogFile);
    LogWithTime("sh2_cpu_tb.vhd: [RAM] Initializing memory from byte " & 
                to_string(16#3000#) & " to " & to_string(16#3000# + 1024), LogFile);
    LogWithTime("sh2_cpu_tb.vhd: [RAM] Valid Data Memory Range is 0x0000 to 0x40000", LogFile);

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

    LogWithTime("sh2_cpu_tb.vhd: [ROM] Initializing memory from byte " & 
                to_string(16#0000#) & " to " & to_string(16#0000# + 1024), LogFile);
    LogWithTime("sh2_cpu_tb.vhd: [ROM] Initializing memory from byte " & 
                to_string(16#1000#) & " to " & to_string(16#1000# + 1024), LogFile);
    LogWithTime("sh2_cpu_tb.vhd: [ROM] Initializing memory from byte " & 
                to_string(16#2000#) & " to " & to_string(16#2000# + 1024), LogFile);
    LogWithTime("sh2_cpu_tb.vhd: [ROM] Initializing memory from byte " & 
                to_string(16#3000#) & " to " & to_string(16#3000# + 1024), LogFile);
    LogWithTime("sh2_cpu_tb.vhd: [ROM] Valid Program Memory Range is 0x0000 to 0x40000", LogFile);

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

            -- Reverse bytes to convert from big-endian (in memory) to little-endian (in CPU)
            data := TEST_DB(7 downto 0) & TEST_DB(15 downto 8) & TEST_DB(23 downto 16) & TEST_DB(31 downto 24);

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

                LogWithTime(
                  "Read " & to_hstring(curr_opcode(15 downto 8)) &  " " &
                            to_hstring(curr_opcode(7 downto 0)) &
                  " @ PC 0x" & to_hstring(curr_pc), LogFile);

                -- Write instruction word into memory
                WriteWord(curr_pc, curr_opcode);

                curr_pc := curr_pc + 2;
            end loop;
            file_close(char_file);
        end procedure;

        procedure DumpMemory(path : string; start : integer; length : integer) is
            file out_file : text;
            variable curr_line : line;
            variable curr_addr   : unsigned(31 downto 0);
            variable data_out    : std_logic_vector(31 downto 0);
            variable curr_byte   : std_logic_vector(7 downto 0);
        begin
            -- Access RAM
            CPU_ACTIVE <= false;
            TEST_MEMSEL <= '0';

            file_open(out_file, path & ".dump", write_mode);

            write(curr_line, YELLOW & "Memory dump for " & path & ANSI_RESET);
            writeline(out_file, curr_line);

            curr_addr := to_unsigned(start, 32);
            for i in 1 to length loop
                ReadLongword(curr_addr, data_out);


                write(curr_line, to_hstring(curr_addr) & " ");

                for j in 3 downto 0 loop
                    curr_byte := data_out(7 + 8 * j downto 8 * j);

                    if (curr_byte = "XXXXXXXX" or   curr_byte = "UUUUUUUU") then
                        -- Color unitialized memory grey.
                        write(curr_line, GREY);
                    end if;

                    write(curr_line, to_hstring(curr_byte) & " ");
                    write(curr_line, ANSI_RESET);

                end loop;

                writeline(out_file, curr_line);

                curr_addr := curr_addr + 4;
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

            procedure ReadByte(address : unsigned ; data : out std_logic_vector) is
            begin
                TEST_AB <= std_logic_vector(address);
                TEST_DB <= (others => 'Z');

                -- Enable only the specific byte being read
                TEST_RE0 <= '0' when address mod 4 = 0 else '1';
                TEST_RE1 <= '0' when address mod 4 = 1 else '1';
                TEST_RE2 <= '0' when address mod 4 = 2 else '1';
                TEST_RE3 <= '0' when address mod 4 = 3 else '1';

                wait for 5 ns;

                data := TEST_DB(7 downto 0) when address mod 2 = 0 else TEST_DB(31 downto 24);
                
                -- Disable writing
                TEST_RE0 <= '1';
                TEST_RE1 <= '1';
                TEST_RE2 <= '1';
                TEST_RE3 <= '1';
                wait for 5 ns;  -- wait for signal to propagate

            end procedure;


            procedure DumpMemToLog is
                variable l : line;
                variable slv: std_logic_vector(31 downto 0);
                variable hex_str : string(1 to 8) := (others => ' ');
                variable test_data : std_logic_vector(7 downto 0);
                
            begin
                write(l, string'("Segment 0:"));
                writeline(MemLogFile, l);

                for i in 0 to 1023 loop
                    
                    if ((i mod 8) = 0) then
                      writeline(MemLogFile, l);
                      write(l, string'(IntToHString(i) & ":"  & HT));
                    end if;

                    ReadByte(to_unsigned(i, 32), test_data);
                    wait for 5 ns;

                    if ((test_data = "UU") or (test_data = "XX")) then
                        write(l, string'(to_hstring(test_data)));
                    else
                        write(l, string'(GREEN & to_hstring(test_data) & ANSI_RESET));
                    end if;

                end loop;

            end procedure;

        begin
            -- report "Running test: " & path;
            LogBothWithTime("Running test: " & path, LogFile);
            LoadProgram(path);      -- write program in to ROM
            RunCPU;                 -- execute program

            -- Clock is stopped at this point.
            DumpMemory(path, 0, 64);

            CheckOutput(path);      -- check RAM has expected values
        end procedure;

    begin

        -- RunTest("asm/hello");
        -- RunTest("asm/mov_reg");
        -- RunTest("asm/reg_indirect");
        -- RunTest("asm/arith");
        -- RunTest("asm/logic");
        -- RunTest("asm/shift");
        -- RunTest("asm/sr");
        -- RunTest("asm/system");

        -- RunTest("asm/mov_wl_at_disp_pc_rn");          -- Tests MOV (disp, PC), Rn

        -- RunTest("asm/mov_bwl_at_rm_rn");              -- Tests MOV @Rm, Rn

        -- RunTest("asm/mov_bwl_rm_at_minus_rn");        -- Tests Mov Rm, @-Rn

        -- RunTest("asm/mov_bwl_at_rm_plus_rn");         -- Test Mov @Rm+, Rn

        -- RunTest("asm/mov_bwl_r0_or_rm_at_disp_rn");   -- Test Mov R0, @(disp, Rn) and Mov Rm, @(disp,Rn)

        -- RunTest("asm/mov_bwl_at_disp_rm_r0_or_rn");   -- Test Mov @(disp, Rm), R0 and Mov @(disp, Rm), Rn

        -- RunTest("asm/mov_rm_at_r0_rn");               -- Test Mov Rm, @(R0, Rn)

        -- RunTest("asm/mov_b_at_r0_rm_rn");              -- Test Mov @(R0, Rm), Rn 

        RunTest("asm/mov_b_r0_at_disp_gbr")              -- Test Mov R0, @(disp, GBR)

        wait;
    end process;

    LogRAMWrites : process(RAM_WE0, RAM_WE1, RAM_WE2, RAM_WE3)
    begin
      -- If the data memory is being accessed and there has been a changed to
      -- the RAM write enable lines, log the address being written to and the
      -- mode.
      if (CPU_MEMSEL = '0') then
      end if;

    end process LogRAMWrites;

end behavioral;

