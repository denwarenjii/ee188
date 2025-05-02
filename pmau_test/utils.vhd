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
end package SH2Utils;

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



library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

package Utils is

  function int_to_slv  (i : integer; width : natural)  return std_logic_vector;
  function uint_to_slv (i : natural; width : natural)  return std_logic_vector;
  function slv_to_int  (slv : std_logic_vector) return integer;
  function slv_to_uint (slv : std_logic_vector) return natural;

end package Utils;

package body Utils is

  function int_to_slv (i : integer; width : natural) return std_logic_vector is
    variable max_int : signed(width - 1 downto 0);
    variable min_int : signed(width - 1 downto 0); 
  begin

    max_int := (others => '1');
    max_int(max_int'high) := '0';

    min_int := (others => '0');
    min_int(max_int'high) := '1';

    assert ((width <= 32) and (to_signed(i, width) <= MAX_INT) and 
            (to_signed(i, width) >= MIN_INT))
      report "signed integer " & to_string(i) & " cannot be converted to a " &
             "std_logic_vector of width " & to_string(width)
      severity ERROR;

    return std_logic_vector(to_signed(i, width));

  end function;

  function uint_to_slv (i : natural; width : natural)  return std_logic_vector is
  begin

    assert ((width <= 32) and (i <= 2**(width - 1) - 1))
      report "signed integer " & to_string(i) & " cannot be converted to a " &
             "std_logic_vector of width " & to_string(width)
      severity ERROR;

    return std_logic_vector(to_unsigned(i, width));

  end function;

  function slv_to_int  (slv : std_logic_vector) return integer is
    constant MIN_32_SIGNED : std_logic_vector(31 downto 0) := x"80000000";
  begin
    -- The VHDL integer range is guaranteed to be at least,
    -- -2,147,483,647 to +2,147,483,647, but 2**31 in two's complement is
    -- -2,147,483,648. Trying to convert this to an integer causes a runtime 
    -- error.
    if (slv'length = 32) then
      assert (slv /= MIN_32_SIGNED)
        report "std_logic_vector " & to_string(slv) & " cannot be represented as " &
               "an integer."
        severity ERROR;
    end if;

    assert ((slv'length <= 32))
      report "std_logic_vector is too wide to be represented as an integer (length = " &
             to_string(slv'length) & " )"
      severity ERROR;

    return to_integer(signed(slv));

  end function;

  function slv_to_uint (slv : std_logic_vector) return natural is
    constant MAX_32_SIGNED : std_logic_vector(31 downto 0) := x"7fffffff";
  begin

    if (slv'length = 32) then
      assert (slv /= MAX_32_SIGNED)
        report "std_logic_vector " & to_string(slv) & " cannot be represented as " &
               "an integer."
        severity ERROR;
    end if;

    assert ((slv'length <= 32))
      report "std_logic_vector is too wide to be represented as an integer (length = " &
             to_string(slv'length) & " )"
      severity ERROR;

    return to_integer(unsigned(slv));

  end function;

end package body Utils;
