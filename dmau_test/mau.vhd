----------------------------------------------------------------------------
--
--  Generic Memory Access Unit
--
--  This is an implementation of a generic memory access unit for
--  microprocessors.  This unit generates the memory address for either load
--  and store operations or instruction access.  It is parameterized by the
--  number of sources and the data width.
--
--  Packages included are:
--     MemUnitConstants - constants for the memory access unit
--     array_type_pkg   - type definition for 2-D arrays of std_logic
--
--  Entities included are:
--     AdderBit - one bit of an adder (a full adder)
--     MemUnit  - generic memory access unit
--
--  Revision History:
--     27 Jan 21  Glen George       Initial revision.
--      4 Feb 21  Glen George       Added missing library declaration.
--      4 Feb 21  Glen George       Added initialization of low bit of carry
--                                  for the adder.
--
----------------------------------------------------------------------------


--
--  Package containing the type definition for 2-D arrays of std_logic.  The
--  type is an unconstrained 2-D array which is supported in VHDL-2008.
--

library ieee;
use ieee.std_logic_1164.all;

package array_type_pkg is

--  a 2D array of std_logic (VHDL-2008)
   type  std_logic_array  is  array (natural range<>) of std_logic_vector;


end package;



--
--  Package containing the constants for the Memory Unit
--

library ieee;
use ieee.std_logic_1164.all;

package MemUnitConstants is

--  memory access unit constants for pre- and post- increment and decrement
--     these constants may be freely changed

   constant MemUnit_PRE  : std_logic := '0';            -- pre- inc/dec
   constant MemUnit_POST : std_logic := '1';            -- post- inc/dec
   constant MemUnit_INC  : std_logic := '0';            -- pre/post increment
   constant MemUnit_DEC  : std_logic := '1';            -- pre/post decrement


end package;



--
--  AdderBit
--
--  This is a bit of the adder for doing addition in the ALU.
--
--  Inputs:
--    A  - first operand bit (bus A)
--    B  - second operand bit (bus B)
--    Ci - carry in (from previous bit)
--
--  Outputs:
--    S  - sum for this bit
--    Co - carry out for this bit
--

library ieee;
use ieee.std_logic_1164.all;

entity  AdderBit  is

    port(
        A  : in   std_logic;        -- first operand
        B  : in   std_logic;        -- second operand
        Ci : in   std_logic;        -- carry in from previous bit
        S  : out  std_logic;        -- sum (result)
        Co : out  std_logic         -- carry out to next bit
    );

end  AdderBit;


architecture  dataflow  of  AdderBit  is
begin

    S  <=  A  xor  B  xor  Ci;
    Co <=  (A  and  B)  or  (A  and Ci)  or  (B  and  Ci);

end  dataflow;



--
--  MemUnit
--
--  This is a generic memory access unit.  It allows for pre- or post-
--  increment/decrement of an address as well as multiple sources for the
--  address and offset.
--
--  Generics:
--    srcCnt       - number of possible sources
--    offsetCnt    - number of possible address offsets
--    maxIncDecBit - maximum value for IncDecBit input (for optimization)
--    wordsize     - address width
--
--  Inputs:
--    AddrSrc    - array (srccnt x wordsize) of address sources
--    SrcSel     - source to use (log srccnt bits)
--    AddrOff    - array (offsetcnt x wordsize) of address offsets
--    OffsetSel  - offset to use (log offsetcnt bits)
--    IncDecSel  - whether to increment (0) or decrement (1) address source
--    IncDecBit  - bit of address source to increment/decrement
--    PrePostSel - whether to pre- (0) or post- (1) inc/dec address source
--
--  Outputs:
--    Address    - address bus (wordsize bits)
--    AddrSrcOut - incremented/decremented source (wordsize bits)
--

library ieee;
use ieee.std_logic_1164.all;
use work.array_type_pkg.all;
use work.MemUnitConstants.all;

entity  MemUnit  is

    generic (
        srcCnt       : integer;
        offsetCnt    : integer;
        maxIncDecBit : integer := 0; -- default is only inc/dec bit 0
        wordsize     : integer := 16 -- default address width is 16 bits
    );

    port(
        AddrSrc    : in      std_logic_array(srccnt - 1 downto 0)(wordsize - 1 downto 0);
        SrcSel     : in      integer  range srccnt - 1 downto 0;
        AddrOff    : in      std_logic_array(offsetcnt - 1 downto 0)(wordsize - 1 downto 0);
        OffsetSel  : in      integer  range offsetcnt - 1 downto 0;
        IncDecSel  : in      std_logic;
        IncDecBit  : in      integer  range maxIncDecBit downto 0;
        PrePostSel : in      std_logic;
        Address    : out     std_logic_vector(wordsize - 1 downto 0);
        AddrSrcOut : buffer  std_logic_vector(wordsize - 1 downto 0)
    );

end  MemUnit;


architecture  dataflow  of  MemUnit  is

    -- need adders for computing the address
    component  AdderBit  port(
        A  : in   std_logic;        -- first operand
        B  : in   std_logic;        -- second operand
        Ci : in   std_logic;        -- carry in from previous bit
        S  : out  std_logic;        -- sum (result)
        Co : out  std_logic         -- carry out to next bit
    );
    end component;

    -- intermediate carry results
    --   for adder
    signal  acarry : std_logic_vector(wordsize downto 0);
    --   for incrementer/decrementer
    signal  idcarry : std_logic_vector(wordsize downto 0);

    -- source address, depends on whether doing pre- or post- inc/dec
    signal  SrcAddr : std_logic_vector(wordsize - 1 downto 0);

    -- input to incrementer/decrementer adder, depends on whether doing an
    --    increment or decrement and which bit it is being applied to
    signal  IncDecIn : std_logic_vector(wordsize - 1 downto 0);

    -- incremented/decremented source address
    signal  OutSrcAddr : std_logic_vector(wordsize - 1 downto 0);


		signal SrcSelMux : std_logic_vector(wordsize - 1 downto 0);

		signal AddrOffMux : std_logic_vector(wordsize - 1 downto 0);

begin

    -- compute the input for the incrementer/decrementer
    --    when incrementing the input is all 0's except for the IncDecBit (bit
    --       to start incrementing)
    --    when decrementing the input is 0's until the IncDecBit (the bit to
    --       start decrementing) and after that it is all 1's
    --    thus the IncDecBit is always a 1
    IDin:  for  i  in  IncDecIn'Range  generate     -- generate the bits independently
    begin

        -- if past the maximum allowable bit to increment/decrement use 0 when
        --    incrementing and 1 when decrementing (optimization)
        IDin1:  if  (i > maxIncDecBit)  generate
            IncDecIn(i)  <=  '0'  when  IncDecSel = MemUnit_INC  else
                             '1'  when  IncDecSel = MemUnit_DEC  else
                             'X';
        end generate;

        -- if not past the maximum allowable bit to increment/decrement use a
        --    0 if below the IncDecBit and a 1 if at the IncDecBit and a 0 if
        --    above the IncDecBit and incrementing and a 1 if above the
        --    IncDecBit and decrementing
        IDin2:  if  (i <= maxIncDecBit)  generate
            IncDecIn(i) <= '1'  when  (i = IncDecBit) or
                                      ((IncDecSel = MemUnit_DEC) and
                                       (i >= IncDecBit))  else
                           '0';
        end generate;

    end generate;


    -- adder for doing increment/decrement
    --    it adds the increment/decrement input to the selected source address
    --    to generate the source address output (AddrSrcOut)
    idcarry(0) <= '0';                              -- there is no carry in


		SrcSelMux <= AddrSrc(SrcSel);

    IDA1:  for  i  in  AddrSrcOut'Range  generate   -- make enough AdderBits
    begin
        IDABx: AdderBit  port map  (
					SrcSelMux(i), 
					IncDecIn(i), 
					idcarry(i),
          OutSrcAddr(i), 
					idcarry(i + 1));
    end generate;


    -- input to the offset adder is either the original source or the
    --    incremented/decremented source, depending on whether doing pre- or
    --    post- increment/decrement
    SrcAddr  <=  AddrSrc(SrcSel)  when  PrePostSel = MemUnit_PRE   else
                 OutSrcAddr       when  PrePostSel = MemUnit_POST  else
                 (others => 'X');


    -- adder for adding offset to the source address
    acarry(0) <= '0';                               -- there is no carry in

		AddrOffMux <= AddrOff(OffsetSel);

    AA1:  for  i  in  Address'Range  generate       -- make enough AdderBits
    begin
        AABx: AdderBit  port map  (
					SrcAddr(i), 
					AddrOffMux(i), 
					acarry(i),
          Address(i), 
					acarry(i + 1));
    end generate;


    -- output the incremented/decremented source address
    AddrSrcOut  <=  OutSrcAddr;

end  dataflow;



