----------------------------------------------------------------------------
-- utils.vhd
--
-- Miscellaneous functions and procedures for SH-2 block testing. Currently
-- includes randomization for an SH-2 word, as well as utility functions
-- for converting between ints, and std_logic_vector.
--  
-- Packages provided:
--    SH2Utils - SH2 specific utility functions.
--    Utils    - generic utility functions.
--
--  Revision History:
--    28 April 25   Zach H.    Initial revision.
--    30 April 25   Chris M.   Add conversion functions.
--
----------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.textio.all;

package  SH2Utils  is
    type rng is protected
        impure function rand_slv(len : integer) return std_logic_vector;
    end protected rng;
end package Utils;

package Utils is

  function int_to_slv  (i : integer; width : natural)  return std_logic_vector;
  function uint_to_slv (i : natural; width : natural)  return std_logic_vector;
  function slv_to_int  (slv : std_logic_vector) return integer;
  function slv_to_uint (slv : std_logic_vector) return natural;

end package Utils;

package body SH2Utils is
    type rng is protected body
        variable seed1, seed2 : integer := 1000;

        impure function rand_slv(len : integer) return std_logic_vector is
            variable r : real;
            variable slv : std_logic_vector(len - 1 downto 0);
        begin
            for i in slv'range loop
                uniform(seed1, seed2, r);
                slv(i) := '1' when r > 0.5 else '0';
            end loop;
            return slv;
        end function;
    end protected body;
end package body;

package body Utils is

  function int_to_slv (i : integer; width : natural) return std_logic_vector is
  begin

    assert ((width <= 32) and (i <= 2**(width - 1) - 1) and (i >= -2**(width - 1)))
      report "signed integer " & to_string(i) & " cannot be converted to a " &
             "std_logic_vector of width " & to_string(width)
      severity ERROR;

    return std_logic_vector(to_signed(i, width));
  end function;

  function uint_to_slv (i : natural; width : natural)  return std_logic_vector is
  beg
  in
    assert ((width <= 32) and (i <= 2**(width) - 1) and (i >= -2**(width - 1)))
      report "signed integer " & to_string(i) & " cannot be converted to a " &
             "std_logic_vector of width " & to_string(width)
      severity ERROR;

  end function;


end package body Utils;
