--------------------------------------------------------------------------
--	sh2_dmau_tb.vhd
--
-- SH-2 DMAU (Data Memory Access Unit) Test Bench. 
--
-- This is a test-bench for the Data Memory Access Unit (DMAU) of the
-- Hitachi SH-2 CPU. It tests all the addressing modes which calculate
-- an address (decsribed in the table below). Note that it is possible
-- for the control signals to generate an address that does not belong to an
-- SH-2 addressing mode, and the DMAU does not prevent this.
--
-- The test plan is as follows:
--  
--  
--  Revision History:
--    23 April 25   Chris M. Inital revision.
--                           
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
use work.SH2DmauConstants.all;
use work.utils.all;
use work.SH2Utils.all;
use work.all;

entity sh2_dmau_tb is
end sh2_dmau_tb;

architecture TestBench of sh2_dmau_tb is

    -- UUT Inputs:
    --
    signal RegSrc_TB       : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal R0Src_TB        : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal PCSrc_TB        : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal GBRIn_TB        : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal GBRWriteEn_TB   : std_logic;
    signal Off4_TB         : std_logic_vector(3 downto 0);
    signal Off8_TB         : std_logic_vector(7 downto 0);
    signal BaseSel_TB      : std_logic_vector(1 downto 0);
    signal IndexSel_TB     : std_logic_vector(1 downto 0);
    signal OffScalarSel_TB : std_logic_vector(1 downto 0);
    signal IncDecSel_TB    : std_logic_vector(1 downto 0);
    signal Clk_TB          : std_logic;

    -- UUT Outputs:
    --
    signal Address_TB      : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal AddrSrcOut_TB   : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal GBROut_TB       : std_logic_vector(SH2_WORDSIZE - 1 downto 0);


    -- constant 
    constant CLK_PERIOD : time := (1 sec) / (1e6);

    constant DELTA : time := 1 fs;


begin

  UUT : entity SH2Dmau
    port map (
      -- Inputs:
      --
      RegSrc       => RegSrc_TB,
      R0Src        => R0Src_TB,
      PCSrc        => PCSrc_TB,       
      GBRIn        => GBRIn_TB,
      GBRWriteEn   => GBRWriteEn_TB,
      Off4         => Off4_TB,
      Off8         => Off8_TB,
      BaseSel      => BaseSel_TB,
      IndexSel     => IndexSel_TB,
      OffScalarSel => OffScalarSel_TB,
      IncDecSel    => IncDecSel_TB,
      Clk          => Clk_TB,
                                     
      -- Outputs:
      --
      Address      => Address_TB,
      AddrSrcOut   => AddrSrcOut_TB,
      GBROut       => GBROut_TB      
    );


    GenClk : process
    begin
      Clk_TB <= '1';
      wait for CLK_PERIOD;
      Clk_TB <= '0';
      wait for CLK_PERIOD;
    end process GenClk;


    -- DMAU Adressing Modes to control signals map:

    -- Indirect Register Addressing -------------------------------------------
    --
    --   GBRWriteEn = '0'   
    --   BaseSel    = BaseSel_REG  
    --   IndexSel   = IndexSel_NONE 
    --   OffScalarSel = OffScalarSel_ONE
    --   IncDecSel    = IncDecSel_NONE
    --
    -- Use `Addr` as output
    ---------------------------------------------------------------------------

    -- Post-increment indirect register addressing ----------------------------
    --
    --   Byte mode:
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_REG  
    --     IndexSel     = IndexSel_NONE 
    --     OffScalarSel = OffScalarSel_ONE
    --     IncDecSel    = IncDecSel_POST_INC
    -- 
    --   Word mode:
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_REG  
    --     IndexSel     = IndexSel_NONE 
    --     OffScalarSel = OffScalarSel_TWO
    --     IncDecSel    = IncDecSel_POST_INC
    -- 
    --   Long-word mode:
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_REG  
    --     IndexSel     = IndexSel_NONE 
    --     OffScalarSel = OffScalarSel_FOUR
    --     IncDecSel    = IncDecSel_POST_INC
    -- 
    -- Use `AddrSrcOut` as output
    ---------------------------------------------------------------------------

    -- Pre-decrement indirect register addressing -----------------------------
    --
    --   Byte mode:
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_REG  
    --     IndexSel     = IndexSel_NONE 
    --     OffScalarSel = OffScalarSel_ONE
    --     IncDecSel    = IncDecSel_PRE_DEC
    --
    --   Word mode:
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_REG  
    --     IndexSel     = IndexSel_NONE 
    --     OffScalarSel = OffScalarSel_TWO
    --     IncDecSel    = IncDecSel_PRE_DEC
    --
    --   Long-word mode:
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_REG  
    --     IndexSel     = IndexSel_NONE 
    --     OffScalarSel = OffScalarSel_FOUR
    --     IncDecSel    = IncDecSel_PRE_DEC
    --
    -- Use `AddrSrcOut` as output
    ---------------------------------------------------------------------------

    -- Indirect register addressing with displacement -------------------------
    --
    --  Byte mode:
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_REG  
    --     IndexSel     = IndexSel_OFF4
    --     OffScalarSel = OffScalarSel_ONE
    --     IncDecSel    = IncDecSel_NONE
    --
    --  Word mode:
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_REG  
    --     IndexSel     = IndexSel_OFF4
    --     OffScalarSel = OffScalarSel_TWO
    --     IncDecSel    = IncDecSel_NONE
    --
    --  Long-word mode:
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_REG  
    --     IndexSel     = IndexSel_OFF4
    --     OffScalarSel = OffScalarSel_FOUR
    --     IncDecSel    = IncDecSel_NONE
    --
    -- Use `Addr` as output
    ---------------------------------------------------------------------------

    -- Indirect indexed register addressing -----------------------------------
    --
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_REG  
    --     IndexSel     = IndexSel_R0
    --     OffScalarSel = OffScalarSel_ONE
    --     IncDecSel    = IncDecSel_NONE
    --
    -- Use `Addr` as output
    ---------------------------------------------------------------------------

    -- Indirect GBR addressing with displacement ------------------------------
    --
    --  Byte Mode:
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_GBR
    --     IndexSel     = IndexSel_OFF8
    --     OffScalarSel = OffScalarSel_ONE
    --     IncDecSel    = IncDecSel_NONE
    --
    --  Word Mode:
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_GBR
    --     IndexSel     = IndexSel_OFF8
    --     OffScalarSel = OffScalarSel_TWO
    --     IncDecSel    = IncDecSel_NONE
    --
    --  Long-word Mode:
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_GBR
    --     IndexSel     = IndexSel_OFF8
    --     OffScalarSel = OffScalarSel_FOUR
    --     IncDecSel    = IncDecSel_NONE
    --
    -- Use `Addr` as output.
    ---------------------------------------------------------------------------

    -- Indirect indexed GBR addressing ----------------------------------------
    --
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_GBR
    --     IndexSel     = IndexSel_R0
    --     OffScalarSel = OffScalarSel_FOUR
    --     IncDecSel    = IncDecSel_NONE
    --
    -- Use `Addr` as output.
    ---------------------------------------------------------------------------

    -- PC relative addressing with displacement -------------------------------
    --
    -- Word Mode:
    --
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_PC
    --     IndexSel     = IndexSel_OFF8
    --     OffScalarSel = OffScalarSel_TWO
    --     IncDecSel    = IncDecSel_NONE
    --
    -- Long-word mode:
    --
    --     GBRWriteEn   = '0'
    --     BaseSel      = BaseSel_PC
    --     IndexSel     = IndexSel_OFF8
    --     OffScalarSel = OffScalarSel_FOUR
    --     IncDecSel    = IncDecSel_NONE
    --
    -- Use `Addr` as output.
    ---------------------------------------------------------------------------


    RunTests : process

      variable l : line;

      -- Initialize the random variable.
      variable RV : rng;

      variable ExpectedAddr : std_logic_vector(SH2_WORDSIZE - 1 downto 0) :=
        (others => '0');

      procedure RandomizeInputs is
      begin
        RegSrc_TB     <= RV.rand_slv(RegSrc_TB'length);
        R0Src_TB      <= RV.rand_slv(R0Src_TB'length);
        PCSrc_TB      <= RV.rand_slv(PCSrc_TB'length);
        GBRIn_TB      <= RV.rand_slv(GBRIn_TB'length);
        Off4_TB       <= RV.rand_slv(Off4_TB'length);
        Off8_TB       <= RV.rand_slv(Off8_TB'length);
      end procedure RandomizeInputs;

      procedure LogInputs is
        variable l : line;
      begin
        write(l, string'(     HT & "RegSrc_TB: "  & HT & "0x" & to_hstring(RegSrc_TB)));
        write(l, string'(LF & HT & "R0Src_TB: "   & HT & "0x" & to_hstring(R0Src_TB)));
        write(l, string'(LF & HT & "PCSrc_TB: "   & HT & "0x" & to_hstring(PCSrc_TB)));
        write(l, string'(LF & HT & "GBROut_TB: "  & HT & "0x" & to_hstring(GBROut_TB)));
        write(l, string'(LF & HT & "Off4_TB: "    & HT & "0x" & to_hstring(Off4_TB)));
        write(l, string'(LF & HT & "Off8_TB: "    & HT & "0x" & to_hstring(Off8_TB)));
        writeline(output, l);
      end procedure LogInputs;

      procedure SetControlLines(
          GBRWriteEn_v    : std_logic;
          BaseSel_v       : std_logic_vector(BaseSel_TB'range);
          IndexSel_v      : std_logic_vector(IndexSel_TB'range);
          OffScalarSel_v  : std_logic_vector(OffScalarSel_TB'range);
          IncDecSel_v     : std_logic_vector(IncDecSel_TB'range)
      ) is
      begin
         GBRWriteEn_TB   <= GBRWriteEn_v;
         BaseSel_TB      <= BaseSel_v;
         IndexSel_TB     <= IndexSel_v;
         OffScalarSel_TB <= OffScalarSel_v;
         IncDecSel_TB    <= IncDecSel_v; 
      end procedure SetControlLines;

      procedure CheckResult (
          actual_addr : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
          verbose : boolean := false
      ) is
        variable l : line;
      begin
        assert (ExpectedAddr = actual_addr)
        report "ExpectedAddr (0x" & to_hstring(ExpectedAddr) & ") is not equal " &
               "to actual_addr (0x" & to_hstring(actual_addr) & ")"
        severity ERROR;

        if (verbose) then
          LogInputs;
          write(l, string'(HT & "ExpectedAddr:" & HT & "0x" & to_hstring(ExpectedAddr) & LF));
          write(l, string'(HT & "actual_addr:" & HT & "0x" & to_hstring(actual_addr)));
          writeline(output, l);
        end if;

      end procedure CheckResult;

      procedure Tick is
      begin
        wait for CLK_PERIOD;
      end procedure Tick;

    begin

      -- Test plan:
      --
      --  For each adresing mode
      --     Pick random inputs.
      --     Set the appropriate control signals.
      --     Compare expected and actual outputs.

      GBRIn_TB <= (others => '0');

      write(l, string'(LF & "------------------------------------------------------"));
      write(l, string'("------------------------------------------------------" & LF));
      writeline(output, l);

      -- Indirect Register Addressing -----------------------------------------
      -- ExpectedAddr = RegSrc (in `Address_TB`)
      --
      write(l, string'("Testing Indirect Regsiter Addressing"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('0', BaseSel_REG, IndexSel_NONE, OffScalarSel_ONE, IncDecSel_NONE);

      Tick;

      -- Set ExpectedAddr after the signal propagates.
      ExpectedAddr := RegSrc_TB;
      CheckResult(Address_TB);


      -- Post-increment indirect register addressing ----------------------------
      -- ExpectedAddr = RegSrc + 1/2/4 (in `AddrSrcOut_TB`)
      --
      write(l, string'("Testing Indirect Regsiter Addressing (Byte Mode)"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('0', BaseSel_REG, IndexSel_NONE, OffScalarSel_ONE, IncDecSel_POST_INC);
      Tick;
      ExpectedAddr := std_logic_vector(unsigned(RegSrc_TB) + to_unsigned(1, SH2_WORDSIZE));
      CheckResult(AddrSrcOut_TB);


      write(l, string'("Testing Indirect Regsiter Addressing (Word Mode)"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('0', BaseSel_REG, IndexSel_NONE, OffScalarSel_TWO, IncDecSel_POST_INC);
      Tick;
      ExpectedAddr := std_logic_vector(unsigned(RegSrc_TB) + to_unsigned(2, SH2_WORDSIZE));
      CheckResult(AddrSrcOut_TB);

      write(l, string'("Testing Indirect Regsiter Addressing (Long-word Mode)"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('0', BaseSel_REG, IndexSel_NONE, OffScalarSel_FOUR, IncDecSel_POST_INC);
      Tick;
      ExpectedAddr := std_logic_vector(unsigned(RegSrc_TB) + to_unsigned(4, SH2_WORDSIZE));
      CheckResult(AddrSrcOut_TB);


      -- Pre-decrement indirect register addressing -----------------------------
      -- ExpectedAddr = RegSrc_TB - 1/2/4 (in `AddrSrcOut_TB`)
      --
      write(l, string'("Testing Indirect Regsiter Addressing (Byte Mode)"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('0', BaseSel_REG, IndexSel_NONE, OffScalarSel_ONE, IncDecSel_PRE_DEC);
      Tick;
      ExpectedAddr := std_logic_vector(unsigned(RegSrc_TB) - to_unsigned(1, SH2_WORDSIZE));
      CheckResult(AddrSrcOut_TB);

      write(l, string'("Testing Indirect Regsiter Addressing (Word Mode)"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('0', BaseSel_REG, IndexSel_NONE, OffScalarSel_TWO, IncDecSel_PRE_DEC);
      Tick;
      ExpectedAddr := std_logic_vector(unsigned(RegSrc_TB) - to_unsigned(2, SH2_WORDSIZE));
      CheckResult(AddrSrcOut_TB);

      write(l, string'("Testing Indirect Regsiter Addressing (Long word Mode)"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('0', BaseSel_REG, IndexSel_NONE, OffScalarSel_FOUR, IncDecSel_PRE_DEC);
      Tick;
      ExpectedAddr := std_logic_vector(unsigned(RegSrc_TB) - to_unsigned(4, SH2_WORDSIZE));
      CheckResult(AddrSrcOut_TB);

      -- Indirect register addressing with displacement -------------------------
      -- ExpectedAddr = RegSrc_TB + (1/2/4)*Off4 (in `Address_TB`)
      --
      write(l, string'("Indirect register addressing with displacement (Byte Mode)"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('0', BaseSel_REG, IndexSel_OFF4, OffScalarSel_ONE, IncDecSel_NONE);
      Tick;

      ExpectedAddr := std_logic_vector(
        unsigned(RegSrc_TB) + 
        resize(unsigned(Off4_TB), SH2_WORDSIZE) 
      );

      CheckResult(Address_TB);

      write(l, string'("Indirect register addressing with displacement (Word Mode)"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('0', BaseSel_REG, IndexSel_OFF4, OffScalarSel_TWO, IncDecSel_NONE);
      Tick;

      ExpectedAddr := std_logic_vector(
        unsigned(RegSrc_TB) + 
        shift_left(resize(unsigned(Off4_TB), SH2_WORDSIZE), 1)
      );

      CheckResult(Address_TB);

      write(l, string'("Indirect register addressing with displacement (Long-word Mode)"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('0', BaseSel_REG, IndexSel_OFF4, OffScalarSel_FOUR, IncDecSel_NONE);
      Tick;

      ExpectedAddr := std_logic_vector(
        unsigned(RegSrc_TB) + 
        shift_left(resize(unsigned(Off4_TB), SH2_WORDSIZE), 2)
      );

      CheckResult(Address_TB);

      -- Indirect indexed register addressing -----------------------------------
      -- ExpectedAddr = RegSrc_TB + R0Src_TB (in `Address_TB`)
      --
      write(l, string'("Indirect indexed register addressing"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('0', BaseSel_REG, IndexSel_R0, OffScalarSel_ONE, IncDecSel_NONE);
      Tick;

      ExpectedAddr := std_logic_vector(
        unsigned(RegSrc_TB) + 
        unsigned(R0Src_TB)
      );

      CheckResult(Address_TB);


      -- Indirect GBR addressing with displacement ------------------------------
      -- ExpectedAddr = GBR + (1/2/4) * Off8_TB (in `Address_TB`)
      --

      -- Load GBR (NOTE: The desired GBRIn_TB must propogate to the input before 
      -- clocking it again).
      GBRIn_TB <= RV.rand_slv(SH2_WORDSIZE);
      Tick;

      write(l, string'("Indirect GBR addresing with displacement (Byte Mode)"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('1', BaseSel_GBR, IndexSel_OFF8, OffScalarSel_ONE, IncDecSel_NONE);
      Tick;

      ExpectedAddr := std_logic_vector(
        unsigned(GBROut_TB) + 
        resize(unsigned(Off8_TB), SH2_WORDSIZE)
      );

      CheckResult(Address_TB);

      write(l, string'("Indirect GBR addresing with displacement (Word Mode)"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('0', BaseSel_GBR, IndexSel_OFF8, OffScalarSel_TWO, IncDecSel_NONE);
      Tick;

      ExpectedAddr := std_logic_vector(
        unsigned(GBROut_TB) + 
        shift_left(resize(unsigned(Off8_TB), SH2_WORDSIZE), 1)
      );

      CheckResult(Address_TB);

      write(l, string'("Indirect GBR addresing with displacement (Long-word Mode)"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('0', BaseSel_GBR, IndexSel_OFF8, OffScalarSel_FOUR, IncDecSel_NONE);
      Tick;

      ExpectedAddr := std_logic_vector(
        unsigned(GBROut_TB) + 
        shift_left(resize(unsigned(Off8_TB), SH2_WORDSIZE), 2)
      );

      CheckResult(Address_TB);

      -- PC relative addressing with displacement -------------------------------
      -- ExpectedAddr = PCSrc_TB + 2*Off8_TB or (PCSrc_TB & 0xFFFFFFFC) + 4*Off8_TB
      -- (in `Address_TB`)
      --
      write(l, string'("PC relative addressing with displacement (word mode)"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('0', BaseSel_PC, IndexSel_OFF8, OffScalarSel_TWO, IncDecSel_NONE);
      Tick;

      ExpectedAddr := std_logic_vector(
        unsigned(PCSrc_TB) + 
        shift_left(resize(unsigned(Off8_TB), SH2_WORDSIZE), 1)
      );

      CheckResult(Address_TB);

      -- NOTE: Long-word mode clears the low two bits of PC before doing the
      -- calculation.
      --
      write(l, string'("PC relative addressing with displacement (Long-word mode)"));
      writeline(output, l);
      RandomizeInputs;
      SetControlLines('0', BaseSel_PC, IndexSel_OFF8, OffScalarSel_FOUR, IncDecSel_NONE);
      Tick;

      ExpectedAddr := std_logic_vector(
        unsigned(PCSrc_TB and x"FFFFFFFC") + 
        shift_left(resize(unsigned(Off8_TB), SH2_WORDSIZE), 2)
      );

      CheckResult(Address_TB);

      stop;

    end process RunTests;
    
end TestBench;
