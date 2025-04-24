----------------------------------------------------------------------------
--	sh2_dmau.vhd
--
-- SH-2 DMAU (Data Memory Access Unit). 
--
--  This is an implementation of the SH-2's DMAU using Glen A. George's 
--  generic memory access unit. The SH-2 is a Princeton architecture CPU
--  where program memory can only be read. The following 11 addressing modes
--  are supported by the SH-2 instruction set.
--
--    1.  Direct Register Addressing
--    2.  Indirect Register Addressing
--    3.  Post-increment indirect register addressing
--    4.  Pre-decrement indirect register addressing
--    5.  Indirect register addressing with displacement
--    6.  Indirect indexed register addressing
--    7.  Indirect GBR addressing with displacement 
--    8.  Indirect indexed GBR addressing
--    9.  PC relative addressing with displacement
--    10. PC relative addressing
--    11. Immediate addressing
--
-- The modes are described in further detail in Table 4.7 of the SH-2 
-- programming manual.
--
-- Note that the DMAU only takes care of addressing modes related to memory 
-- locations. As such, the DMAU will not do direct register addressing, since
-- that is done by the register array, nor will it do immediate addressing, 
-- which is done by the control unit during instruction decoding. Finally,
-- there are not safeguards agaisnt addressing modes which do not actually
-- exist in the SH-2.
--
--  Revision History:
--		16 April 25		Chris M. Initial reivision.
--    23 April 25   Chris M. Add seperate calculated offsets to AddrOff matrix
--                           instead of muxing between a single offset.
--
-- [TODO]:
--    - Don't allow inputs that don't correspond to addressing modes.
--    - Make mapping from IndexSel to DMAUOffsetSel one-to-one ?
--    - Only have one offset input ?
--
----------------------------------------------------------------------------

library ieee;
library std;
library work;

use ieee.std_logic_1164.all;

use work.MemUnitConstants.all; -- memory access unit constants for pre/post inc/dec.
use work.SH2Constants.all;     -- global SH-2 constants.
use work.array_type_pkg.all;   -- 2D Array of std_logic (VHDL-2008 only).

package SH2DmauConstants is

  -- BaseSel constants.
  --
  constant BaseSel_REG : std_logic_vector(1 downto 0)  := "00"; -- 0
  constant BaseSel_GBR : std_logic_vector(1 downto 0)  := "01"; -- 1
  constant BaseSel_PC  : std_logic_vector(1 downto 0)  := "10"; -- 2

  -- IndexSel constants.
  constant IndexSel_NONE   : std_logic_vector(2 downto 0) := "000";   -- 0
  constant IndexSel_OFF4   : std_logic_vector(2 downto 0) := "001";   -- 1
  constant IndexSel_OFF8   : std_logic_vector(2 downto 0) := "010";   -- 2
  constant IndexSel_OFF12  : std_logic_vector(2 downto 0) := "011";   -- 3
  constant IndexSel_R0     : std_logic_vector(2 downto 0) := "100";   -- 4
  constant IndexSel_REG    : std_logic_vector(2 downto 0) := "101";   -- 5

  -- OffExtendSel constants. Zero-extend or sign-extend the offset into a 32-bit word.
  constant OffExtendSel_ZERO : std_logic := '0';
  constant OffExtendSel_SIGN : std_logic := '1';

  -- OffsetScalarSel constant. What to scale the offset (or increment value) by.
  --
  constant OffScalarSel_ONE : 	std_logic_vector(1 downto 0) := "00"; -- 0
  constant OffScalarSel_TWO :  	std_logic_vector(1 downto 0) := "01"; -- 1
  constant OffScalarSel_FOUR : 	std_logic_vector(1 downto 0) := "10"; -- 2

  -- IncDecSel constants.
  --
  constant IncDecSel_NONE     : std_logic_vector(1 downto 0) := "00";
  constant IncDecSel_PRE_DEC  : std_logic_vector(1 downto 0) := "01";
  constant IncDecSel_POST_INC : std_logic_vector(1 downto 0) := "10";

end package SH2DmauConstants;


library ieee;
library std;
library work;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.all;
use work.MemUnitConstants.all; -- memory access unit constants for pre/post inc/dec.
use work.SH2Constants.all;     -- global SH-2 constants.
use work.array_type_pkg.all;   -- 2D Array of std_logic (VHDL-2008 only).
use work.SH2DmauConstants.all;


-- SH2Dmau
--
-- This is the SH-2s DMAU. It implements the following addressing modes.
--
--
-- +------------------------------------------------------+-----------------------------------------------------+
-- |                   Addressing Mode                    |               Formula(s)                            |
-- +------------------------------------------------------+-----------------------------------------------------+
-- | 1.  Indirect Register Addressing                     | Addr = @(Rn)                                        |
-- +------------------------------------------------------+-----------------------------------------------------+
-- | 2.  Post-increment indirect register addressing      | (After the instruction is executed)                 |
-- |                                                      | Addr = @(Rn + 1)                                    |
-- |                                                      | Addr = @(Rn + 2)                                    |
-- |                                                      | Addr = @(Rn + 4)                                    |
-- +------------------------------------------------------+-----------------------------------------------------+
-- | 3.  Pre-decrement indirect register addressing       | (Before the instruction is executed)                |
-- |                                                      | Addr = @(Rn - 1)                                    |
-- |                                                      | Addr = @(Rn - 2)                                    |
-- |                                                      | Addr = @(Rn - 4)                                    |
-- +------------------------------------------------------+-----------------------------------------------------+
-- | 4.  Indirect register addressing with displacement   | Addr = @(Rn + zero_extend(Off4))                    |
-- | -                                                    | Addr = @(Rn + zero_extend(Off4) * 2)                |
-- | -                                                    | Addr = @(Rn + zero_extend(Off4) * 4)                |
-- +------------------------------------------------------+-----------------------------------------------------+
-- | 5. Indirect indexed register addressing              | Addr = @(Rn + R0), Rn =/= R0                        |
-- +------------------------------------------------------+-----------------------------------------------------+
-- | 6.  Indirect GBR addressing with displacement        | Addr = @(GBR + zero_extend(Off8))                   |
-- | -                                                    | Addr = @(GBR + zero_extend(Off8) * 2)               |
-- | -                                                    | Addr = @(GBR + zero_extend(Off8) * 4)               |
-- +------------------------------------------------------+-----------------------------------------------------+
-- | 7.  Indirect indexed GBR addressing                  | Addr = @(GBR + R0)                                  |
-- +------------------------------------------------------+-----------------------------------------------------+
-- | 8.  PC relative addressing with displacement         | Addr = @(PC + zero_extend(Off8) * 2)                |
-- | -                                                    | Addr = @((PC & 0xFFFFFFFC) + zero_extend(Off8) * 4) |
-- +------------------------------------------------------+-----------------------------------------------------+
-- | 9. PC relative addressing                            | Addr = @(PC + sign_extend(Off8) * 2)                |
-- | -                                                    | Addr = @(PC + sign_extend(Off12) * 2)               |
-- | -                                                    | Addr = @(PC + Rn)                                   |
-- +------------------------------------------------------+-----------------------------------------------------+
--
-- "Base" sources are the sources to the left hand side of an address 
-- calculation, and consist of Rn (a register), GBR (Global Base Register), or 
-- PC (Program Counter).
-- 
-- "Index" sources are the sources to the right hand side of an address 
-- calculation, and consist of an immediate offset which is either sign or 
-- zero extended and then scaled by 1, 2, or 4, R0, or Rn.
--
-- Inputs:
--  RegSrc       -   Input register. One of three possible base sources.
--  R0Src        -   R0 Register input.
--  PCSrc        -   Program Counter Source.
--  Off4         -   4-Bit Offset.
--  Off8         -   8-Bit Offset.
--  Off12        -   12-Bit Offset.
--  BaseSel      -   Which base register source to select (Reg, GBR, or PC).
--  IndexSel     -   Which index source to select (None, Off4, Off8, Off12, Rn,
--                   or R0). Note that Off4, Off8, Off12 are zero or sign 
--                   extended before being added to the base.
--
--  OffExtendSel -   Zero or sign extend the offset into a 32-bit word.
--  OffScalar    -   What to scale the offset by (1, 2, 4)
--  IncDecSel    -   Post Increment or PreDecrement the base. Note that the
--                   amount added or subtracted is scaled ny OffScalar. Note 
--                   that even when this is set to NONE, an 
--                   incremented/decremented value is output to AddrSrcOut as a
--                   result of the generic memory access unit design.
--  GBRIn        -   GBR input.
--  Clk          -   Clk input.
--  
-- Outputs:
--
-- Addr       - output address
-- AddrSrcOut - incremented/decremented address (for storing back into register).
-- GBROut     - GBR output.
--
entity SH2Dmau is
  port (
    RegSrc       : in     std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    R0Src        : in     std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    PCSrc        : in     std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    Off4         : in     std_logic_vector(3 downto 0);
    Off8         : in     std_logic_vector(7 downto 0);
    Off12        : in     std_logic_vector(11 downto 0);
    BaseSel      : in     std_logic_vector(1 downto 0);
    IndexSel     : in     std_logic_vector(2 downto 0);
    OffExtendSel : in     std_logic;
    OffScalarSel : in     std_logic_vector(1 downto 0);
    IncDecSel    : in     std_logic_vector(1 downto 0);
    Clk          : in     std_logic;

    Address      : out    std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    AddrSrcOut   : buffer std_logic_vector(SH2_WORDSIZE - 1 downto 0)
  );
end SH2Dmau;

architecture structural of SH2Dmau is

    -- ZeroExtend a std_logic_vector into an SH2_WORDSIZE std_logic_vector.
    --
    pure function ZeroExtend(slv : std_logic_vector) return std_logic_vector is
      variable result : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    begin
      result := (others => '0');
      result(slv'range) := slv;
    end function;

    -- SignExtend a std_logic_vector into an SH2_WORDSIZE std_logic_vector.
    --
    pure function SignExtend(slv : std_logic_vector) return std_logic_vector is
      variable result : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    begin
      result := (others => slv(slv'left));
      result(slv'range)  := slv;
    end function;

    -- shift_left is defined for unsigned/signed types only; wrap for slv.
    --
    pure function shift_left_slv(slv : std_logic_vector; 
                                 k   : natural) return std_logic_vector is
    begin
      return std_logic_vector(shift_left(unsigned(slv), k));
    end function;

    -- Global Base Register.
    signal GBR : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    

    -- MemUnit generic constants.
    --
    constant SRCCNT       : integer := 3;  -- Number of vectors in AddrSrc matrix. 
    constant OFFSETCNT    : integer := 9;  -- Number of vectors in AddrOff matrix.
    constant MAXINCDECBIT : integer := 2;  -- The maximum index of the bit of 
                                           -- the calculted address that we want 
                                           -- to increment or decrement.  

    -- DMAUAddrSrc.
    -- Consists of RegSrc, GBR, and PC. Note that this is a one-to-one mapping
    -- between the BaseSel constants.
    --
    signal DMAUAddrSrc : std_logic_array(SRCCNT - 1 downto 0)(SH2_WORDSIZE - 1 downto 0);


    -- DMAUSrcSel constants.
    constant DMAUAddrSrc_REG : integer := 0;
    constant DMAUAddrSrc_GBR : integer := 1;
    constant DMAUAddrSrc_PC  : integer := 2;
    signal 	 DMAUSrcSel      : integer  range SRCCNT - 1 downto 0;
    

    -- DMAUOffsetSel constants.
    constant ZERO                 : integer := 0;
    constant OFF4_ZERO_EXTENDED   : integer := 1;
    constant OFF8_ZERO_EXTENDED   : integer := 2;
    constant OFF12_ZERO_EXTENDED  : integer := 3;
    constant R0                   : integer := 4;
    constant OFF4_SIGN_EXTENDED   : integer := 5;
    constant OFF8_SIGN_EXTENDED   : integer := 6;
    constant OFF12_SIGN_EXTENDED  : integer := 7;
    constant REG                  : integer := 8;
    signal 	 DMAUOffsetSel        : integer range OFFSETCNT - 1 downto 0;

    signal DMAUAddrOff : std_logic_array(OFFSETCNT - 1 downto 0)(SH2_WORDSIZE - 1 downto 0);


    constant 	DMAU_INC 			 : std_logic := '0';
    constant 	DMAU_DEC 			 : std_logic := '1';
    signal 		DMAUIncDecSel  : std_logic;

    -- The bit of the calculted address we want to increment or decrement.
    --    + 1 = increment bit 0
    --    * 2 = increment bit 1
    --    * 4 = increment bit 2
    signal DMAUIncDecBit  : integer range MAXINCDECBIT downto 0;

    constant DMAU_PRE  		: std_logic := '0';
    constant DMAU_POST 		: std_logic := '1';
    signal DMAUPrePostSel : std_logic;

    -- The low two bits of the PC are masked if PC is selected as the base address, 
    -- Off8 is selected as the index, and the scaled factor is 4.
    constant 	PC_MASK : std_logic_vector(SH2_WORDSIZE - 1 downto 0) := x"FFFFFFFC";
    signal 		PCMux 	: std_logic_vector(SH2_WORDSIZE - 1 downto 0);


    -- Zero/sign extedned offsets.
    signal Off4ZeroExtended   : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal Off8ZeroExtended   : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal Off12ZeroExtended  : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal Off4SignExtended   : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal Off8SignExtended   : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal Off12SignExtended  : std_logic_vector(SH2_WORDSIZE - 1 downto 0);

begin

  WriteGBR : process(Clk)
  begin
    if rising_edge(Clk) then
      GBR <= GBRIn;
    end if;
  end process WriteGBR;


  GBROut <= GBR;
  -- DMAUAddrSrc  ------------------------------------------------------------

  -- The low two bits of the PC are masked if PC is selected as the base address, 
  -- Off8 is selected as the index, and the scaled factor is 4. 
  PCMux <= (PCSrc and PC_MASK) when (BaseSel = BaseSel_PC)      and 
                                    (IndexSel = IndexSel_OFF8)  and
                                    (OffScalarSel = OffScalarSel_FOUR) else
            PCSrc;

  DMAUAddrSrc(DMAUAddrSrc_REG) <= RegSrc;
  DMAUAddrSrc(DMAUAddrSrc_GBR) <= GBR;
  DMAUAddrSrc(DMAUAddrSrc_PC)  <= PCMux;


  -- DMAUSrcSel --------------------------------------------------------------
  DMAUSrcSel <= to_integer(unsigned(BaseSel));

  -- DMAUAddrOff  ------------------------------------------------------------

  Off4ZeroExtended  <= ZeroExtend(Off4);
  Off8ZeroExtended  <= ZeroExtend(Off8);
  Off12ZeroExtended <= ZeroExtend(Off12);

  Off4SignExtended  <= SignExtend(Off4);
  Off8SignExtended  <= SignExtend(Off8);
  Off12SignExtended <= SignExtend(Off12);
  
  -- Populate the DMAUAddrOff matrix. 
  DMAUAddrOff(ZERO) <= (others => '0');

  DMAUAddrOff(OFF4_ZERO_EXTENDED)   <= 
      shift_left_slv(Off4ZeroExtended, to_integer(unsigned(OffScalarSel)));

  DMAUAddrOff(OFF8_ZERO_EXTENDED)   <= 
      shift_left_slv(Off8ZeroExtended, to_integer(unsigned(OffScalarSel)));

  DMAUAddrOff(OFF12_ZERO_EXTENDED)  <= 
      shift_left_slv(Off12ZeroExtended, to_integer(unsigned(OffScalarSel)));
      
  DMAUAddrOff(R0) <= R0Src;

  DMAUAddrOff(OFF4_SIGN_EXTENDED)   <= 
      shift_left_slv(Off4ZeroExtended, to_integer(unsigned(OffScalarSel)));

  DMAUAddrOff(OFF8_SIGN_EXTENDED)   <= 
      shift_left_slv(Off8ZeroExtended, to_integer(unsigned(OffScalarSel)));

  DMAUAddrOff(OFF12_SIGN_EXTENDED)  <= 
      shift_left_slv(Off12ZeroExtended, to_integer(unsigned(OffScalarSel)));

  DMAUAddrOff(REG) <= RegSrc;

  -- DMAUOffsetSel  ----------------------------------------------------------
  DMAUOffsetSel <= 
      ZERO when (IndexSel = IndexSel_NONE) else

      OFF4_ZERO_EXTENDED when (IndexSel = IndexSel_OFF4) and 
                              (OffExtendSel = OffExtendSel_ZERO) else

      OFF8_ZERO_EXTENDED when (IndexSel = IndexSel_OFF8) and
                              (OffExtendSel = OffExtendSel_ZERO) else

      OFF12_ZERO_EXTENDED when (IndexSel = IndexSel_OFF12) and
                               (OffExtendSel = OffExtendSel_ZERO) else    
  
      R0 when (IndexSel = IndexSel_R0) else

      OFF4_SIGN_EXTENDED when (IndexSel = IndexSel_OFF4) and
                              (OffExtendSel = OffExtendSel_SIGN) else

      OFF8_SIGN_EXTENDED when (IndexSel = IndexSel_OFF8) and
                              (OffExtendSel = OffExtendSel_SIGN) else

      OFF12_SIGN_EXTENDED when (IndexSel = IndexSel_OFF12) and
                               (OffExtendSel = OffExtendSel_SIGN) else

      REG                 when (IndexSel = IndexSel_REG) else
  
      DMAUOffsetSel;
      

  -- DMAUIncDecSel ------------------------------------------------------------

  with IncDecSel select DMAUIncDecSel <=
    DMAU_INC      when  IncDecSel_POST_INC,
    DMAU_DEC      when  IncDecSel_PRE_DEC,
    DMAUIncDecSel when  others;

  -- DMAUIncDecBit ------------------------------------------------------------

  DMAUIncDecBit <= 0 when (OffScalarSel = OffScalarSel_ONE) and  
                          ((IncDecSel = IncDecSel_PRE_DEC) or
                          (IncDecSel = IncDecSel_POST_INC)) else

                   1 when (OffScalarSel = OffScalarSel_TWO) and  
                          ((IncDecSel = IncDecSel_PRE_DEC) or
                          (IncDecSel = IncDecSel_POST_INC)) else

                   2 when (OffScalarSel = OffScalarSel_FOUR) and  
                          ((IncDecSel = IncDecSel_PRE_DEC) or
                          (IncDecSel = IncDecSel_POST_INC)) else
                   0;
  
  -- DMAUPrePostSel -----------------------------------------------------------
  with IncDecSel select DMAUPrePostSel <=
    DMAU_PRE       when IncDecSel_PRE_DEC,
    DMAU_POST      when IncDecSel_POST_INC,
    DMAUPrePostSel when others;


  SH2Dmau_Instance : entity MemUnit
    generic map (
      srcCnt        => SRCCNT,
      offsetCnt     => OFFSETCNT,
      maxIncDecBit  => MAXINCDECBIT,
      wordsize      => SH2_WORDSIZE
    )
    port map (
      -- Inputs:
      AddrSrc    => DMAUAddrSrc,
      SrcSel     => DMAUSrcSel,
      AddrOff    => DMAUAddrOff,
      OffsetSel  => DMAUOffsetSel,
      IncDecSel  => DMAUIncDecSel,
      IncDecBit  => DMAUIncDecBit,
      PrePostSel => DMAUPrePostSel,
      -- Ouputs:
      Address    => Address,
      AddrSrcOut => AddrSrcOut
    );

end structural;

