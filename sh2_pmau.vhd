----------------------------------------------------------------------------
--  sh2_pmau.vhd
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
--    - PC <- PC + Rm        (register relative)
--    - PC <- PR             (PR direct)
--    - PC <- PC + 2         (increment)
--    - PC <- Rm             (register direct)
--
--
--  Revision History:
--    16 April 25   Chris M. Initial reivision.
--    01 May   25   Chris M. Added PRWriteEn and seperate offset signals. Made
--                           PrePostSel in MAU be POST when we don't care.
--    02 May   25   Chris M. Changed SignExtend function to wrap numeric_std
--                           conversion.
--    07 May   25   Chris M. Add reset signal and logic.
--    26 May   25   Chris M. Add 2 to 8-bit offset.
--    29 May   25   Chris M. Add PCWriteEn and PCIn. Add PCRegOut and PCCalcOut.
--
----------------------------------------------------------------------------

library ieee;
library std;
library work;

use work.SH2Constants.all;
use ieee.std_logic_1164.all;

-- SH2Pmau
--
-- This is the SH-2s PMAU. It handles altering the PC according to the input 
-- control signals. The ways in which the PC can change are
--
--    PC <- PC + 2*disp:8  (relative)
--    PC <- PC + 2*disp:12 (relative)
--    PC <- PC + Rm        (register relative)
--    PC <- PR             (PR direct)
--    PC <- PC + 2         (increment)
--    PC <- Rm             (register direct)
--
-- Note that the possible adresses sources to the general memory access unit
-- are only the zero vector or the PC. Thus, direct replacements of the PC
-- will be accomplished by adding the offset to the zero vector. Finally
-- note neither the PC nor the PR are stored within the program memory access
-- unit.
--
-- Inputs:
--   RegIn         : Register source input.
--   PRIn          : PR Register input (for writing to PR).
--   PRWriteEn     : Enable writing to PR (active high).
--   PCIn          : Input for parallel loading of PC.
--   PCWriteCtrl   : Write control signal for PC. Can either hold the current 
--                   value, write PCIn to the PC register on the rising edge,
--                   or write the calculated PC value on the rising edge.
--   Off8          : 8-bit signed offset input.
--   Off12         : 12-bit signed offset input.
--   PCAddrMode    : Program address mode select signal.
--   Clk           : Clock.
--   Reset         : system reset (active low).
--    
-- Outputs:
--   PCCalcOut     : Calculated PC address output. Note that this is not the
--                   value of the PR register.
--   PCRegOut      : Current value of the PC register.
--   
--   PROut         : PR (Procedure Register) output.
--
entity SH2Pmau is
  port (
    RegIn         : in std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    PRIn          : in std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    PRWriteEn     : in std_logic;
    PCIn          : in std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    PCWriteCtrl   : in std_logic_vector(1 downto 0);
    Off8          : in std_logic_vector(7 downto 0);
    Off12         : in std_logic_vector(11 downto 0);
    PCAddrMode    : in std_logic_vector(2 downto 0);
    Clk           : in std_logic;
    Reset         : in std_logic;
    PCCalcOut     : out std_logic_vector(31 downto 0);   -- TODO
    PCRegOut      : out std_logic_vector(31 downto 0);   -- TODO
    PROut         : out std_logic_vector(SH2_WORDSIZE - 1 downto 0)
  );
end entity SH2Pmau;


library ieee;
use ieee.std_logic_1164.all;

package SH2PmauConstants is

  constant PCAddrMode_INC                     : std_logic_vector(2 downto 0) := "000";  -- PC <- PC + 2
  constant PCAddrMode_RELATIVE_8              : std_logic_vector(2 downto 0) := "001";  -- PC <- PC + disp:8
  constant PCAddrMode_RELATIVE_12             : std_logic_vector(2 downto 0) := "010";  -- PC <- PC + disp:12
  constant PCAddrMode_REG_DIRECT_RELATIVE     : std_logic_vector(2 downto 0) := "011";  -- PC <- PC + Rm
  constant PCAddrMode_REG_DIRECT              : std_logic_vector(2 downto 0) := "100";  -- PC <- Rm
  constant PCAddrMode_PR_DIRECT               : std_logic_vector(2 downto 0) := "101";  -- PC <- PR
  constant PCAddrMode_HOLD                    : std_logic_vector(2 downto 0) := "110";  -- PC <- PC

  
  constant PCWriteCtrl_HOLD       : std_logic_vector(1 downto 0) := "00";   -- Hold the current PC. 
  constant PCWriteCtrl_WRITE_IN   : std_logic_vector(1 downto 0) := "01";   -- Write PCIn to the PC reg. 
  constant PCWriteCtrl_WRITE_CALC : std_logic_vector(1 downto 0) := "10";   -- Write the calculated PC.
  

end package SH2PmauConstants;

library ieee;
library std;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use std.textio.all;

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
    -- slv -> signed, resize to sign-extend, then convert to slv. 
    result := std_logic_vector(resize(signed(slv), SH2_WORDSIZE));
    return result;
  end function;

  -- shift_left is defined for unsigned/signed types only; wrap for slv.
  --
  pure function shift_left_slv(slv : std_logic_vector; 
                               k   : natural) return std_logic_vector is
  begin
    return std_logic_vector(shift_left(unsigned(slv), k));
  end function;

  -- Possible sources are PC, PR, and Rm.
  constant SRCCNT : integer := 3;

  -- Possible offfsets are None, Off8, Off12, or Rm.
  constant OFFSETCNT : integer := 4;

  -- Adding two is the same as incrementing bit 1 of the PC.
  constant MAXINCDECBIT : integer := 1;
  
  constant PMAUAddrSrc_PC : integer := 0;
  constant PMAUAddrSrc_PR : integer := 1;
  constant PMAUAddrSrc_Rm : integer := 2;
  signal PMAUAddrSrc : std_logic_array(SRCCNT - 1 downto 0)(SH2_WORDSIZE - 1 downto 0);
  signal PMAUSrcSel  : integer range SRCCNT - 1 downto 0;

  constant PMAUAddrOff_NONE   : integer := 0;
  constant PMAUAddrOff_OFF8   : integer := 1;
  constant PMAUAddrOff_OFF12  : integer := 2;
  constant PMAUAddrOff_REG    : integer := 3;

  signal PMAUAddrOff : std_logic_array(OFFSETCNT - 1 downto 0)(SH2_WORDSIZE - 1 downto 0);
  signal PMAUOffsetSel : integer range OFFSETCNT - 1 downto 0;

  
  -- constant MemUnit_INC  : std_logic := '0';            -- pre/post increment
  signal PMAUIncDecSel  : std_logic;

  -- MAXINCDECBIT := 1
  signal PMAUIncDecBit  : integer range 0 to 1;

  -- constant MemUnit_POST : std_logic := '1';            -- post- inc/dec
  signal PMAUPrePostSel : std_logic;

  signal CalculatedPC  : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
  signal IncrementedPC : std_logic_vector(SH2_WORDSIZE - 1 downto 0);

  -- signal PC : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
  -- signal PR : std_logic_vector(SH2_WORDSIZE - 1 downto 0);

  -- signal PCMux : std_logic_vector(SH2_WORDSIZE - 1 downto 0);

  signal PRReg : std_logic_vector(SH2_WORDSIZE - 1 downto 0);

  signal PCReg : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
  signal PCMux : std_logic_vector(SH2_WORDSIZE - 1 downto 0);

  constant ZERO_32 : std_logic_vector(SH2_WORDSIZE - 1 downto 0) := (others => '0');

begin

  PCCalcOut <= (ZERO_32) when (Reset = '0') else
           PCMux;


  PCRegOut <= PCReg;

  -- PCOut <= (ZERO_32) when (Reset = '0') else
  --          PCMux;

  PROut <= (ZERO_32) when (Reset = '0') else
           PRReg;

  with PCAddrMode select 
      PCMux <= IncrementedPC    when  PCAddrMode_INC,
               CalculatedPC     when  PCAddrMode_RELATIVE_8 | PCAddrMode_RELATIVE_12,
               CalculatedPC     when  PCAddrMode_REG_DIRECT_RELATIVE,
               CalculatedPC     when  PCAddrMode_REG_DIRECT,
               PRReg            when  PCAddrMode_PR_DIRECT,
               PCReg            when  PCAddrMode_HOLD,
               (others => '0')  when  others;

  UpdateRegisters : process(Clk, reset)
  begin

    if reset = '0' then
      -- Reset PC and PR to all zeros.
      PCReg <= (others => '0');
      PRReg <= (others => '0');

    elsif (rising_edge(Clk)) then

      -- PCReg <= PCMux;

      -- Only write to register if their enable signal is set.
      if (PRWriteEn = '1') then
        PRReg <= PRIn;
      else
        PRReg <= PRReg;
      end if;

      -- Choose what to do to the PC based on the PC write control signal.
      PCReg <= PCMux;

      case PCWriteCtrl is

          when PCWriteCtrl_HOLD =>
              PCReg <= PCReg;

          when PCWriteCtrl_WRITE_IN =>
              PCReg <= PCIn;

         when PCWriteCtrl_WRITE_CALC =>
             PCReg <= PCMux;

         -- This has to be set intially otherwise instructions are not loaded.
         -- TODO: Hold when an uncrecognized signal is sent.
         when others =>
             PCReg <= PCReg;

      end case;

    end if;

  end process;


  -- PMAUAddrSrc --------------------------------------------------------------

  PMAUAddrSrc(PMAUAddrSrc_PC) <= (ZERO_32) when (Reset = '0') else
                                 PCReg;

  PMAUAddrSrc(PMAUAddrSrc_PR) <= (ZERO_32) when (Reset = '0') else 
                                 PRReg;

  PMAUAddrSrc(PMAUAddrSrc_Rm) <= (ZERO_32) when (Reset = '0') else
                                 RegIn; 

  -- PMAUSrcSel ---------------------------------------------------------------

  with PCAddrMode select PMAUSrcSel <=

    PMAUAddrSrc_PC  when   PCAddrMode_INC | PCAddrMode_RELATIVE_8 | PCAddrMode_RELATIVE_12 | 
                           PCAddrMode_REG_DIRECT_RELATIVE,
    PMAUAddrSrc_PR  when   PCAddrMode_PR_DIRECT,
    PMAUAddrSrc_Rm  when   PCAddrMode_REG_DIRECT,
    0               when   others;


  -- PMAUAddrOff --------------------------------------------------------------

  PMAUAddrOff(PMAUAddrOff_NONE)   <=   (others => '0');

  -- 2 * SignExtend(Off8) (*2 is shift left by 1)

  PMAUAddrOff(PMAUAddrOff_OFF8)   <=   (ZERO_32) when (Reset = '0') else
                                       std_logic_vector(unsigned(shift_left_slv(SignExtend(Off8), 1)));

  -- 2 * SignExtend(Off12)

  -- TODO: I guess this needs an offset too ???
  PMAUAddrOff(PMAUAddrOff_OFF12)  <=   (ZERO_32) when (Reset = '0') else

                                       std_logic_vector(unsigned(shift_left_slv(SignExtend(Off12), 1)) + to_unsigned(4, 32));

  PMAUAddrOff(PMAUAddrOff_REG)    <=   (ZERO_32) when (Reset = '0') else
                                       RegIn;
                                        
  -- PMAUOffsetSel -----------------------------------------------------------

  with PCAddrMode select PMAUOffsetSel <=

    PMAUAddrOff_NONE   when   PCAddrMode_REG_DIRECT | PCAddrMode_PR_DIRECT | PCAddrMode_INC,
    PMAUAddrOff_OFF8   when   PCAddrMode_RELATIVE_8,
    PMAUAddrOff_OFF12  when   PCAddrMode_RELATIVE_12,
    PMAUAddrOff_REG    when   PCAddrMode_REG_DIRECT_RELATIVE,
    0                  when    others;

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



