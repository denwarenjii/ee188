----------------------------------------------------------------------------
--  sh2_constants.vhd;
--
-- Shared SH-2 constants.
--
-- This file describes constants that are shared across multiple blocks of
-- the SH-2.
--
-- Packages Provided:
--    SH2Constants
--
--  Revision History:
--    16 April 25   Chris M. Initial reivision.
--
----------------------------------------------------------------------------

library ieee;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package SH2Constants is
  constant SH2_WORDSIZE     : integer := 32;
  constant SH2_REGCNT       : integer := 16;
  
  -- Note that these must be hard-coded to avoid overflow in static computation.
  -- This is just (-(2**(SH2_WORDSIZE - 1)))
  constant SH2_SIGNED_MIN   : integer := -2147483648;
  constant SH2_SIGNED_MAX   : integer := 2147483647;
  constant SH2_UNSIGNED_MIN : integer := 0;
  constant SH2_UNSIGNED_MAX : unsigned(SH2_WORDSIZE - 1 downto 0) := (others => '1');

end package SH2Constants;


