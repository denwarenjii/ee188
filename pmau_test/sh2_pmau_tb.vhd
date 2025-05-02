--------------------------------------------------------------------------
--	sh2_pmau_tb.vhd
--
-- SH-2 PMAU (Program Memory Access Unit) Test Bench. 
--
-- This is a test-bench for the Program Memory Access Unit (PMAU) of the
-- Hitachi SH-2 CPU. It tests all the addressing modes used for calculating
-- program addresses for control flow instructions. It also tests parallel
-- loading of the PR register, which is part of the PMAU.
--
--  
--  Revision History:
--    1 May 2025  Chris M.   Initial revision.

----------------------------------------------------------------------------


library ieee;
library std;
library work;

use std.env.all;
use std.textio.all;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.MemUnitConstants.all;
use work.SH2Constants.all;
use work.array_type_pkg.all;
use work.SH2PmauConstants.all;
use work.utils.all;
use work.SH2Utils.all;
use work.all;

entity sh2_pmau_tb is
end sh2_pmau_tb;

architecture TestBench of sh2_pmau_tb is

    -- UUT Inputs:
    --
    signal RegIn_TB         :  std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal PRIn_TB          :  std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal PRWriteEn_TB     :  std_logic;
    signal Off8_TB          :  std_logic_vector(7 downto 0);
    signal Off12_TB         :  std_logic_vector(11 downto 0);
    signal PCAddrMode_TB    :  std_logic_vector(2 downto 0);
    signal Clk_TB           :  std_logic;

    -- UUT Outputs:
    --
    signal PCOut_TB  : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal PROut_TB  : std_logic_vector(SH2_WORDSIZE - 1 downto 0);

    -- constant 
    constant CLK_PERIOD : time := (1 sec) / (1e6);
    constant DELTA : time := 1 fs;


  pure function SignExtend(slv : std_logic_vector) return std_logic_vector is
    variable result : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
  begin
    -- slv -> signed, resize to sign-extend, then conver to slv. 
    result := std_logic_vector(resize(signed(slv), SH2_WORDSIZE));
    return result;
  end function;


begin

  UUT : entity SH2Pmau
    port map (
      -- Inputs:
      --
      RegIn         => RegIn_TB,
      PRIn          => PRIn_TB,
      PRWriteEn     => PRWriteEn_TB,
      Off8          => Off8_TB,
      Off12         => Off12_TB,
      PCAddrMode    => PCAddrMode_TB,
      Clk           => Clk_TB,
      -- Outputs:
      --
      PCOut       => PCOut_TB,
      PROut       => PROut_TB
    ); 

    GenClk : process
    begin
      Clk_TB <= '1';
      wait for CLK_PERIOD/2;
      Clk_TB <= '0';
      wait for CLK_PERIOD/2;
    end process GenClk;

  -- DMAU Adressing Modes to control signals map:
  --
  -- PC Relative (8-bit displacement):
  --    PCAddrMode = PCAddrMode_RELATIVE_8
  --
  -- PC Relative (12-bit displacement):
  --    PCAddrMode = PCAddrMode_RELATIVE_12
  --
  -- Register Relative
  --    PCAddrMode = PCAddrMode_REG_DIRECT_RELATIVE
  --
  -- PR Direct
  --    PCAddrMode = PCAddrMode_PR_DIRECT
  --
  -- Post-increment
  --    PCAddrMode = PCAddrMode_INC
  --
  -- Register direct
  --    PCAddrMode = PCAddrMode_REG_DIRECT
  --

    RunTests : process

      -- Initialize the random variable.
      variable RV : rng;

      variable ExpectedPC : std_logic_vector(SH2_WORDSIZE - 1 downto 0) :=
        (others => '0');

      variable PrevPCOut_TB : std_logic_vector(SH2_WORDSIZE - 1 downto 0);

      variable l : line;

      constant NUM_TESTS : integer := 128;


      -- Randomize the test bench inputs, but not the control signals.
      procedure RandomizeInputs is
      begin
        RegIn_TB <= RV.rand_slv(RegIn_TB'length);
        PRIn_TB  <= RV.rand_slv(PRIn_TB'length);
        Off8_TB  <= RV.rand_slv(Off8_TB'length);
        Off12_TB <= RV.rand_slv(Off12_TB'length);
      end procedure RandomizeInputs;


      -- Log the test bench inputs.
      procedure LogInputs is
        variable l_1 : line;
      begin
        write(l_1, string'(HT & "RegIn_TB: "  & HT & "0x" & to_hstring(RegIn_TB)));
        write(l_1, string'(LF & HT & "PRIn_TB: "   & HT & "0x" & to_hstring(PRIn_TB)));
        write(l_1, string'(LF & HT & "Off8_TB: "   & HT & "0x" & to_hstring(Off8_TB)));
        write(l_1, string'(LF & HT & "Off12_TB: "  & HT & "0x" & to_hstring(Off12_TB)));
        writeline(output, l_1);
      end procedure LogInputs;


      -- Compare the actual and expected PC.
      procedure CheckResult (
          actual_pc : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
          verbose : boolean := false
      ) is
        variable l_2 : line;
      begin
        assert (ExpectedPC = actual_pc)
        report "ExpectedPC (0x" & to_hstring(ExpectedPC) & ") is not equal " &
               "to actual_pc (0x" & to_hstring(actual_pc) & ")"
        severity ERROR;

        if (verbose) then
          LogInputs;
          write(l_2, string'(HT & "ExpectedPC:" & HT & "0x" & to_hstring(ExpectedPC) & LF));
          write(l_2, string'(HT & "actual_pc:"  & HT & "0x" & to_hstring(actual_pc)));
          writeline(output, l_2);
        end if;
      end procedure CheckResult;


      -- Advance the clock to the next rising edge.
      procedure Tick is
      begin
        wait until rising_edge(Clk_TB);
      end procedure Tick;


      procedure LogNow is
      begin
        write(l, string'("@"));
        write(l, now);
        write(l, string'(": "));
      end procedure LogNow;

    begin
      
      -- Test order:
      --    - Register direct (PC <- Rm)
      --    - PR direct (PC <- PR)
      --    - PC relative 8 (PC + 2*disp:8)
      --    - PC relative 12 (PC + 2*disp:12)
      --    - Register relative (PC + Rm)
      --    - Post increment (PC <- PC + 2)
      PRWriteEn_TB <= '0';


      --  Register direct (PC <- Rm)
      --
      LogNow;
      write(l, string'("Testing Register Direct Addressing"));
      writeline(output, l);
      
      PCAddrMode_TB <= PCAddrMode_REG_DIRECT;

      for i in 0 to NUM_TESTS loop

        RandomizeInputs;

        Tick;

        -- LogNow;
        -- write(l, string'("PCOut_TB is 0x" & to_hstring(PCOut_TB)));
        -- writeline(output, l);

        ExpectedPC := RegIn_TB;
        CheckResult(PCOut_TB);

      end loop;


      -- PR direct (PC <- PR)
      --
      LogNow;
      write(l, string'("Testing PR Direct Addressing"));
      writeline(output, l);

      PCAddrMode_TB <= PCAddrMode_PR_DIRECT;
      PRWriteEn_TB <= '1';


      for i in 0 to NUM_TESTS loop

        RandomizeInputs;

        Tick;

        ExpectedPC := PRIn_TB;

        Tick;

        -- LogNow;
        -- write(l, string'("PCOut_TB is 0x" & to_hstring(PCOut_TB)));
        -- writeline(output, l);

        CheckResult(PCOut_TB);

      end loop;

      PRWriteEn_TB <= '0';


      -- PC relative 8 (PC + 2*disp:8)
      --
      LogNow;
      write(l, string'("Testing PC Relative (8-Bit) addressing."));
      writeline(output, l);

      PCAddrMode_TB <= PCAddrMode_RELATIVE_8;

      for i in 0 to NUM_TESTS loop

        PrevPCOut_TB := PCOut_TB;

        RandomizeInputs;

        Tick;

        ExpectedPC := std_logic_vector(
          unsigned(PrevPCOut_TB) +
          shift_left(unsigned(SignExtend(Off8_TB)), 1)
        );

        -- LogNow;
        -- write(l, string'("PCOut_TB is 0x" & to_hstring(PCOut_TB)));
        -- writeline(output, l);

        CheckResult(PCOut_TB);

      end loop;


      -- PC relative 12 (PC + 2*disp:12)
      --
      LogNow;
      write(l, string'("Testing PC Relative (12-Bit) addressing."));
      writeline(output, l);

      PCAddrMode_TB <= PCAddrMode_RELATIVE_12;

      for i in 0 to NUM_TESTS loop

        PrevPCOut_TB := PCOut_TB;

        RandomizeInputs;

        Tick;

        ExpectedPC := std_logic_vector(
          unsigned(PrevPCOut_TB) +
          shift_left(unsigned(SignExtend(Off12_TB)), 1)
        );

        -- LogNow;
        -- write(l, string'("PCOut_TB is 0x" & to_hstring(PCOut_TB)));
        -- writeline(output, l);

        CheckResult(PCOut_TB);

      end loop;

      -- Register relative (PC + Rm)
      --
      LogNow;
      write(l, string'("Testing Register Relative addressing."));
      writeline(output, l);

      PCAddrMode_TB <= PCAddrMode_REG_DIRECT_RELATIVE;

      for i in 0 to NUM_TESTS loop

        PrevPCOut_TB := PCOut_TB;

        RandomizeInputs;

        Tick;

        ExpectedPC := std_logic_vector (
          unsigned(PrevPCOut_TB) +
          unsigned(RegIn_TB)
        );

        -- LogNow;
        -- write(l, string'("PCOut_TB is 0x" & to_hstring(PCOut_TB)));
        -- writeline(output, l);

        CheckResult(PCOut_TB);

      end loop;
    

      -- Post increment (PC <- PC + 2)
      --
      LogNow;
      write(l, string'("Testing Post Incremented addressing."));
      writeline(output, l);

      PCAddrMode_TB <= PCAddrMode_INC;

      for i in 0 to NUM_TESTS loop

        PrevPCOut_TB := PCOut_TB;

        Tick;

        ExpectedPC := std_logic_vector (
          unsigned(PrevPCOut_TB) +
          to_unsigned(2, SH2_WORDSIZE)
        );

        -- LogNow;
        -- write(l, string'("PCOut_TB is 0x" & to_hstring(PCOut_TB)));
        -- writeline(output, l);

        CheckResult(PCOut_TB);

      end loop;


      stop;

    end process RunTests;
    
end TestBench;

