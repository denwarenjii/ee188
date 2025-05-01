----------------------------------------------------------------------------
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
library osvvm;

use std.env.all;
use std.textio.all;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.MemUnitConstants.all;
use work.SH2Constants.all;
use work.array_type_pkg.all;
use work.SH2DmauConstants.all;
use work.all;

use osvvm.RandomPkg.all;
use osvvm.CoveragePkg.all;
use osvvm.TranscriptPkg.all;
context osvvm.OsvvmContext;

entity SH2Dmau_TB is
end SH2Dmau_TB;

architecture TestBench of SH2Dmau_TB is

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
    signal IndexSel_TB     : std_logic_vector(2 downto 0);
    signal OffScalarSel_TB : std_logic_vector(1 downto 0);
    signal IncDecSel_TB    : std_logic_vector(1 downto 0);
    signal Clk_TB          : std_logic;

    -- UUT Outputs:
    --
    signal Address_TB      : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal AddrSrcOut_TB   : std_logic_vector(SH2_WORDSIZE - 1 downto 0);
    signal GBROut_TB       : std_logic_vector(SH2_WORDSIZE - 1 downto 0);


    -- OSVVM signals.
    signal SH2Dmau_TBCovID : CoverageIDType := NewID("SH2 DMAU Coverage");
    signal SH2Dmau_TBLogID : AlertLogIDType := GetAlertLogID("SH2Dmau_TB");

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
      Clk_TB <= '1';
      wait for CLK_PERIOD;
      Clk_TB <= '0';
      wait for CLK_PERIOD;
    end process GenClk;


    RunTests : process(Clk_TB)
    begin
      RegSrc_TB <= ()
    end process RunTests;
    
end TestBench;
