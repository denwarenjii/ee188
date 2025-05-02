----------------------------------------------------------------------------
--	sh2_pmau.vhd
--
-- SH-2 PMAU (Program Memory Access Unit). 
--
--  This is an implementation of the SH-2's PMAU using Glen A. George's generic
--  MAU (memory access unit). The purpose of the PMAU is to calculate program
--  memory addresses for branch instructions. The program counter is modified
--  in the following ways depending on the branch instruction used:
--
--    - PC <- PC + 2*disp:8  (relative)
--    - PC <- PC + 2*disp:12 (relative)
--    - PC <- PC + @(Rm)     (register indirect relative)
--    - PC <- PR             (PR direct)
--    - PC <- PC + 1         (increment)
--    - PC <- Rm             (register direct)
--
--
--  Revision History:
--		16 April 25		Chris M. Initial reivision.
--
-- - Add signal for PR.
----------------------------------------------------------------------------

library ieee;
library std;

use work.SH2Constants.all;
use ieee.std_logic_1164.all;

-- SH2Pmau
--
-- This is the SH-2s PMAU. It handles altering the PC according to the input 
-- control signals. The ways in which the PC can change are
--
--    PC <- PC + 2*disp:8  (relative)
--    PC <- PC + 2*disp:12 (relative)
--    PC <- PC + @(Rm)     (register indirect relative)
--    PC <- PR             (PR direct)
--    PC <- PC + 1         (increment)
--    PC <- Rm             (register direct)
--
-- Note that the possible adresses sources to the general memory access unit
-- are only the zero vector or the PC. Thus, direct replacements of the PC
-- will be accomplished by adding the offset to the zero vector. Finally
-- note neither the PC nor the PR are stored within the program memory access
-- unit.
--
-- Inputs:
--    PCSrc      - Program Counter source.
--    PRSrc      - Procedure Register source.
--    AddrOffset - Address offset source.
--    Disp       - Signed 12-bit immediate displacement.
--    PCAddrMode - Which addressing mode to select.
--    
-- Outputs:
--    PCOut - The updated PC based on the addressing mode.
--
entity SH2Pmau is
  port (
    RegIn       : in std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    PRIn        : in std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    AddrOffset  : in std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    Disp        : in std_logic_vector(11 downto 0);
    PCAddrMode  : in std_logic_vector(2 downto 0);
    Clk         : in std_logic;
    PCOut       : out std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    PROut       : out std_logic_vector(SH2_WORDSIZE - 1 downto 0)
  );
end entity SH2Pmau;


library ieee;
use ieee.std_logic_1164.all;

package SH2PmauConstants is

  constant PCAddrMode_INC                   : std_logic_vector(2 downto 0) := "000"; -- PC <- PC + 2
  constant PCAddrMode_RELATIVE              : std_logic_vector(2 downto 0) := "001"; -- PC <- PC + disp
  constant PCAddrMode_REG_INDIRECT_RELATIVE : std_logic_vector(2 downto 0) := "010"; -- PC <- PC +@[Rm]
  constant PCAddrMode_REG_DIRECT            : std_logic_vector(2 downto 0) := "011"; -- PC <- Rm
  constant PCAddrMode_PR_DIRECT             : std_logic_vector(2 downto 0) := "100"; -- PC <- PR

end package SH2PmauConstants;

library ieee;
library std;

use ieee.std_logic_1164.all;
use work.SH2PmauConstants.all;
use work.SH2Constants.all;
use work.MemUnitConstants.all;
use work.array_type_pkg.all;

architecture structural of SH2Pmau is

  -- SignExtend a std_logic_vector into an SH2_WORDSIZE std_logic_vector.
  --
  pure function SignExtend(slv : std_logic_vector) return std_logic_vector is
    variable result : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
  begin
    result := (others => slv(slv'left));
    result(slv'range)  := slv;
    return result;
  end function;

  -- Possible sources are PC, PR, and Rm.
  constant SRCCNT       : integer := 3;

  -- Possible offfsets are None, Disp, or the contents at some address.
  constant OFFSETCNT    : integer := 3;

  -- Adding two is the same as incrementing bit 1 of the PC.
  constant MAXINCDECBIT : integer := 1;
  
  constant PMAUAddrSrc_PC : integer := 0;
  constant PMAUAddrSrc_PR : integer := 1;
  constant PMAUAddrSrc_Rm : integer := 2;
  signal PMAUAddrSrc : std_logic_array(SRCCNT - 1 downto 0)(SH2_WORDSIZE - 1 downto 0);
  signal PMAUSrcSel  : integer range SRCCNT - 1 downto 0;

  constant PMAUAddrOff_NONE : integer := 0;
  constant PMAUAddrOff_DISP : integer := 1;
  constant PMAUAddrOff_REG  : integer := 2;
  signal PMAUAddrOff : std_logic_array(OFFSETCNT - 1 downto 0)(SH2_WORDSIZE - 1 downto 0);
  signal PMAUOffsetSel : integer range OFFSETCNT - 1 downto 0;

  
  -- constant MemUnit_PRE  : std_logic := '0';            -- pre- inc/dec
  -- constant MemUnit_POST : std_logic := '1';            -- post- inc/dec
  -- constant MemUnit_INC  : std_logic := '0';            -- pre/post increment
  -- constant MemUnit_DEC  : std_logic := '1';            -- pre/post decrement

  signal PMAUIncDecSel  : std_logic;
  signal PMAUIncDecBit  : integer range 0 to 1;
  signal PMAUPrePostSel : std_logic;

  signal CalculatedPC  : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
  signal IncrementedPC : std_logic_vector(SH2_WORDSIZE - 1 downto 0);

  signal PC : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
  signal PR : std_logic_vector(SH2_WORDSIZE - 1 downto 0);

begin

  UpdateRegisters : process(Clk)
  begin

    if rising_edge(Clk) then
      case PCAddrMode is
        when PCAddrMode_INC =>
          PC <= IncrementedPC;
        when PCAddrMode_RELATIVE =>
          PC <= CalculatedPC;
        when PCAddrMode_REG_INDIRECT_RELATIVE =>
          PC <= CalculatedPC;
        when PCAddrMode_REG_DIRECT =>
          PC <= CalculatedPC;
        when PCAddrMode_PR_DIRECT =>
          PC <= PR;
        when others =>
          PC <= PC;
      end case;

      PR <= PRIn;

    end if;
  end process;

  -- PMAUAddrSrc --------------------------------------------------------------

  PMAUAddrSrc(PMAUAddrSrc_PC) <= PC;
  PMAUAddrSrc(PMAUAddrSrc_PR) <= PR;
  PMAUAddrSrc(PMAUAddrSrc_Rm) <= RegIn; 

  -- PMAUSrcSel ---------------------------------------------------------------

  with PCAddrMode select PMAUSrcSel <=
    PMAUAddrSrc_PR when PCAddrMode_INC | PCAddrMode_RELATIVE | PCAddrMode_REG_INDIRECT_RELATIVE,
    PMAUAddrSrc_PR when PCAddrMode_PR_DIRECT,
    PMAUAddrSrc_Rm when PCAddrMode_REG_DIRECT,
    PMAUSrcSel when others;

  -- PMAUAddrOff --------------------------------------------------------------

  PMAUAddrOff(PMAUAddrOff_NONE) <= (others => '0');
  PMAUAddrOff(PMAUAddrOff_DISP) <= SignExtend(Disp);
  PMAUAddrOff(PMAUAddrOff_REG) <= RegIn;

  -- PMAUOffsetSel -----------------------------------------------------------

  with PCAddrMode select PMAUOffsetSel <=
    PMAUAddrOff_NONE when PCAddrMode_REG_DIRECT | PCAddrMode_PR_DIRECT | PCAddrMode_INC,
    PMAUAddrOff_DISP when PCAddrMode_RELATIVE,
    PMAUAddrOff_REG  when PCAddrMode_REG_INDIRECT_RELATIVE,
    PMAUOffsetSel when others;

  -- PMAUIncDecSel ------------------------------------------------------------
  PMAUIncDecSel <= MemUnit_INC;

  -- PMAUIncDecBit ------------------------------------------------------------
  PMAUIncDecBit <= 1;

  -- PMAUPrePostSel ------------------------------------------------------------
  PMAUPrePostSel <= MemUnit_POST;

  SH2Pmau_Instance : entity work.MemUnit
    generic map (
      srcCnt        => SRCCNT,
      offsetCnt     => OFFSETCNT,
      maxIncDecBit  => MAXINCDECBIT,
      wordsize      => SH2_WORDSIZE
    )
    port map (
      -- Inputs:
      AddrSrc    => PMAUAddrSrc,
      SrcSel     => PMAUSrcSel,
      AddrOff    => PMAUAddrOff,
      OffsetSel  => PMAUOffsetSel,
      IncDecSel  => PMAUIncDecSel,
      IncDecBit  => PMAUIncDecBit,
      PrePostSel => PMAUPrePostSel,
      -- Ouputs:
      Address    => CalculatedPC,
      AddrSrcOut => IncrementedPC 
    );
end structural;



