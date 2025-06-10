----------------------------------------------------------------------------
--
--  Memory Subsystem
--
--  This component describes the memory for a 32-bit byte-addressable CPU
--  with a 32-bit address bus.  Only a portion of the full address space is
--  filled in.  Addresses outside the filled in range return 'X' when read
--  and generate error messages when written.
--
--  Revision History:
--     28 Apr 25  Glen George       Initial revision.
--     29 Apr 25  Glen George       Fixed some syntax errors.
--     29 Apr 25  Glen George       Fixed inconsistencies in byte vs word
--                                  addressing.
--     01 May 25  Zack Huang        Fixed compile errors
--     14 May 25  Chris M.          Track locations that have been written.
--     16 May 25  Zack Huang        Added more documentation
--
----------------------------------------------------------------------------


--
--  MEMORY32x32
--
--  This is a memory component that supports a byte-addressable 32-bit wide
--  memory with 32-bits of address.  No timing restrictions are implemented,
--  but if the address bus changes while a WE signal is active an error is
--  generated.  Only a portion of the memory is actually usable.  Addresses
--  outside of the four usable ranges return 'X' on read and generate error
--  messages on write.  The size and address of each memory chunk are generic
--  parameters.
--
--  Generics:
--    MEMSIZE     - size of the four memory blocks in 32-bit words
--    START_ADDR0 - starting address of first memory block/chunk
--    START_ADDR1 - starting address of second memory block/chunk
--    START_ADDR2 - starting address of third memory block/chunk
--    START_ADDR3 - starting address of fourth memory block/chunk
--
--  Inputs:
--    RE0    - low byte read enable (active low)
--    RE1    - byte 1 read enable (active low)
--    RE2    - byte 2 read enable (active low)
--    RE3    - high byte read enable (active low)
--    WE0    - low byte write enable (active low)
--    WE1    - byte 1 write enable (active low)
--    WE2    - byte 2 write enable (active low)
--    WE3    - high byte write enable (active low)
--    MemAB  - memory address bus (32 bits)
--
--  Inputs/Outputs:
--    MemDB  - memory data bus (32 bits)
--

library ieee;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.Logging.all;
use work.Utils.all;

entity  MEMORY32x32  is

    generic (
        MEMSIZE     : integer := 256;   -- default size is 256 words
        START_ADDR0 : integer;          -- starting address of first block
        START_ADDR1 : integer;          -- starting address of second block
        START_ADDR2 : integer;          -- starting address of third block
        START_ADDR3 : integer           -- starting address of fourth block
    );

    port (
        RE0    : in     std_logic;      -- low byte read enable (active low)
        RE1    : in     std_logic;      -- byte 1 read enable (active low)
        RE2    : in     std_logic;      -- byte 2 read enable (active low)
        RE3    : in     std_logic;      -- high byte read enable (active low)
        WE0    : in     std_logic;      -- low byte write enable (active low)
        WE1    : in     std_logic;      -- byte 1 write enable (active low)
        WE2    : in     std_logic;      -- byte 2 write enable (active low)
        WE3    : in     std_logic;      -- high byte write enable (active low)
        MemAB  : in     std_logic_vector(31 downto 0);  -- memory address bus
        MemDB  : inout  std_logic_vector(31 downto 0)   -- memory data bus
    );

end  MEMORY32x32;


architecture  behavioral  of  MEMORY32x32  is

    -- define the type for the RAM chunks
    type  RAMtype  is array (0 to MEMSIZE - 1) of std_logic_vector(31 downto 0);

    -- now define the RAMs (initialized to X)
    signal  RAMbits0  :  RAMtype  := (others => (others => 'X'));
    signal  RAMbits1  :  RAMtype  := (others => (others => 'X'));
    signal  RAMbits2  :  RAMtype  := (others => (others => 'X'));
    signal  RAMbits3  :  RAMtype  := (others => (others => 'X'));

    -- general read and write signals
    signal  RE  :  std_logic;
    signal  WE  :  std_logic;

    -- data read from memory
    signal  Curr_RAM  :  RAMtype;
    signal  RamAddr   :  integer;
    signal  MemData   :  std_logic_vector(31 downto 0);

begin

    -- compute the general read and write signals (active low signals)
    RE  <=  RE0  and  RE1  and  RE2  and  RE3;
    WE  <=  WE0  and  WE1  and  WE2  and  WE3;


    -- On input change, combinatorially compute the address and segment of RAM
    -- that needs to be accessed.
    ram_access: process (all) is
    begin
        -- Check that MemAB is a valid value and is within the range of the integer type
        if not is_x(MemAB) and unsigned(MemAB) <= to_unsigned(integer'high, 32) then
            if ((to_integer(unsigned(MemAB)) >= START_ADDR0) and
                (to_integer(unsigned(MemAB) - START_ADDR0) < (4 * MEMSIZE))) then
                    Curr_RAM <= RAMBits0;
                    RamAddr <= to_integer(unsigned(MemAB(31 downto 2))) - START_ADDR0 / 4;
            elsif ((to_integer(unsigned(MemAB)) >= START_ADDR1) and
                   (to_integer(unsigned(MemAB) - START_ADDR1) < (4 * MEMSIZE))) then
                    Curr_RAM <= RAMBits1;
                    RamAddr <= to_integer(unsigned(MemAB(31 downto 2))) - START_ADDR1 / 4;
            elsif ((to_integer(unsigned(MemAB)) >= START_ADDR2) and
                   (to_integer(unsigned(MemAB) - START_ADDR2) < (4 * MEMSIZE))) then
                    Curr_RAM <= RAMBits2;
                    RamAddr <= to_integer(unsigned(MemAB(31 downto 2))) - START_ADDR2 / 4;
            elsif ((to_integer(unsigned(MemAB)) >= START_ADDR3) and
                   (to_integer(unsigned(MemAB) - START_ADDR3) < (4 * MEMSIZE))) then
                    Curr_RAM <= RAMBits3;
                    RamAddr <= to_integer(unsigned(MemAB(31 downto 2))) - START_ADDR3 / 4;
            else
                Curr_RAM <= (others => (others => 'X'));
                RamAddr <= -1;
            end if;
        end if;
    end process;


    -- Get the 32 bits from the address being read from/written to (as
    -- a extension of the ram_access process)
    MemData <= Curr_RAM(RamAddr) when RamAddr >= 0 and RamAddr < MEMSIZE else (others => 'X');


    -- On read, simply output the (masked) bytes that were accessed
    -- combinatorially (e.g. MemData)
    read_proc: process (all) is
    begin

        -- first check if reading
        if  (RE = '0' and not IS_X(MemAB))  then
            -- report "Reading " & to_hstring(MemAB) & ", got " & to_hstring(MemData);
            MemDB <= MemData;

            -- only set the bytes that are being read
            if  RE0 /= '0'  then
                MemDB(7 downto 0) <= (others => 'Z');
            end if;
            if  RE1 /= '0'  then
                MemDB(15 downto 8) <= (others => 'Z');
            end if;
            if  RE2 /= '0'  then
                MemDB(23 downto 16) <= (others => 'Z');
            end if;
            if  RE3 /= '0'  then
                MemDB(31 downto 24) <= (others => 'Z');
            end if;

        else
            -- not reading, send data bus to hi-Z
            MemDB <= (others => 'Z');
        end if;

    end process;


    -- On write, set the desired bytes within the RAM segment being currently
    -- accessed, at the correct address within the RAM (previously computed
    -- combinatorially).
    write_proc: process (all) is
    begin

        -- check if writing
        if  (WE'event and (WE = '0') and not (is_x(MemAB)) and
             unsigned(MemAB) <= to_unsigned(integer'high, 32))  then
            -- rising edge of write - write the data (check which address range)
            -- report "Writing to " & to_hstring(MemAB) & " with: " & to_hstring(MemDB);

            -- write the updated value to memory (computed combinatorially
            -- above), with byte masking
            if  ((to_integer(unsigned(MemAB)) >= START_ADDR0) and
                 (to_integer(unsigned(MemAB) - START_ADDR0) < (4 * MEMSIZE)))  then
                if (WE0 = '0') then RAMbits0(RamAddr)(7 downto 0) <= MemDB(7 downto 0); end if;
                if (WE1 = '0') then RAMbits0(RamAddr)(15 downto 8) <= MemDB(15 downto 8); end if;
                if (WE2 = '0') then RAMbits0(RamAddr)(23 downto 16) <= MemDB(23 downto 16); end if;
                if (WE3 = '0') then RAMbits0(RamAddr)(31 downto 24) <= MemDB(31 downto 24); end if;
            elsif  ((to_integer(unsigned(MemAB)) >= START_ADDR1) and
                    (to_integer(unsigned(MemAB) - START_ADDR1) < (4 * MEMSIZE)))  then
                if (WE0 = '1') then RAMbits0(RamAddr)(7 downto 0) <= MemDB(7 downto 0); end if;
                if (WE1 = '1') then RAMbits0(RamAddr)(15 downto 8) <= MemDB(15 downto 8); end if;
                if (WE2 = '1') then RAMbits0(RamAddr)(23 downto 16) <= MemDB(23 downto 16); end if;
                if (WE3 = '1') then RAMbits0(RamAddr)(31 downto 24) <= MemDB(31 downto 24); end if;
            elsif  ((to_integer(unsigned(MemAB)) >= START_ADDR2) and
                    (to_integer(unsigned(MemAB) - START_ADDR2) < (4 * MEMSIZE)))  then
                if (WE0 = '0') then RAMbits2(RamAddr)(7 downto 0) <= MemDB(7 downto 0); end if;
                if (WE1 = '0') then RAMbits2(RamAddr)(15 downto 8) <= MemDB(15 downto 8); end if;
                if (WE2 = '0') then RAMbits2(RamAddr)(23 downto 16) <= MemDB(23 downto 16); end if;
                if (WE3 = '0') then RAMbits2(RamAddr)(31 downto 24) <= MemDB(31 downto 24); end if;
            elsif  ((to_integer(unsigned(MemAB)) >= START_ADDR3) and
                    (to_integer(unsigned(MemAB) - START_ADDR3) < (4 * MEMSIZE)))  then
                if (WE0 = '0') then RAMbits3(RamAddr)(7 downto 0) <= MemDB(7 downto 0); end if;
                if (WE1 = '0') then RAMbits3(RamAddr)(15 downto 8) <= MemDB(15 downto 8); end if;
                if (WE2 = '0') then RAMbits3(RamAddr)(23 downto 16) <= MemDB(23 downto 16); end if;
                if (WE3 = '0') then RAMbits3(RamAddr)(31 downto 24) <= MemDB(31 downto 24); end if;
            else
                -- outside of any allowable address range - generate an error
                assert (false)
                    report  "Attempt to write to a non-existant address"
                    severity  ERROR;
            end if;

        end if;

        -- finally check if WE low with the address changing
        if  (MemAB'event and (WE = '0'))  then
            -- output error message
            REPORT "Glitch on Memory Address bus"
            SEVERITY  ERROR;
        end if;

    end process;


end  behavioral;

