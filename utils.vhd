library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.textio.all;

package  SH2Utils  is
    type rng is protected
        impure function rand_slv(len : integer) return std_logic_vector;
    end protected rng;
end package;

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
