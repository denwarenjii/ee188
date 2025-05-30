----------------------------------------------------------------------------
--
--  Hitachi SH-2 CPU Entity Declaration
--
--  This is the entity declaration for the complete SH-2 CPU.  The design
--  should implement this entity to make testing possible.
--
--  Revision History:
--     28 Apr 25  Glen George       Initial revision.
--     01 May 25  Zack Huang        Declare all sub-unit entities
--     03 May 25  Zack Huang        Add state machine, test basic I/O
--     04 May 25  Zack Huang        Integrate memory interface
--     07 May 25  Chris Miranda     Change code formatting.
--     11 May 25  Zack Huang        Start system control instructions
--     12 May 25  Chris M.          Add extra RegDataIn sources and connect 
--                                  PCSrc of DMAU.
--     14 May 25  Chris M.          Tri-state address in writeback state.
--     16 May 25  Zack Huang        Documentation, renaming signals
--     19 May 25  Chris M.          Connect R0Src to DMAU.
--     24 May 25  Chris M.          Add GBRIn mux.
--     25 May 25  Zack Huang        Finishing ALU and system instructions
--     26 May 25  Chris M.          Add internal signal for XTRCT instruction
--                                  register manipulation. Also added this
--                                  as possible input to RegDataIn.
----------------------------------------------------------------------------


--
--  SH2_CPU
--
--  This is the complete entity declaration for the SH-2 CPU.  It is used to
--  test the complete design.
--
--  Inputs:
--    Reset  - active low reset signal
--    NMI    - active falling edge non-maskable interrupt
--    INT    - active low maskable interrupt
--    clock  - the system clock
--
--  Outputs:
--    AB     - memory address bus (32 bits)
--    RE0    - first byte read signal, active low
--    RE1    - second byte read signal, active low
--    RE2    - third byte read signal, active low
--    RE3    - fourth byte read signal, active low
--    WE0    - first byte write signal, active low
--    WE1    - second byte write signal, active low
--    WE2    - third byte write signal, active low
--    WE3    - fourth byte write signal, active low
--
--  Inputs/Outputs:
--    DB     - memory data bus (32 bits)
--

library ieee;
library std;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use std.textio.all;

use work.SH2PmauConstants.all;
use work.MemoryInterfaceConstants.all;
use work.SH2ControlConstants.all;
use work.Logging.all;
use work.SH2Constants.all;
use work.SH2DmauConstants.all;


entity  SH2CPU  is

    port (
        Reset   :  in     std_logic;                       -- reset signal (active low)
        NMI     :  in     std_logic;                       -- non-maskable interrupt signal (falling edge)
        INT     :  in     std_logic;                       -- maskable interrupt signal (active low)
        clock   :  in     std_logic;                       -- system clock
        AB      :  out    std_logic_vector(31 downto 0);   -- memory address bus
        MemSel  :  out    std_logic;                       -- whether to access data memory (0) or program memory (1)
        RE0     :  out    std_logic;                       -- first byte active low read enable
        RE1     :  out    std_logic;                       -- second byte active low read enable
        RE2     :  out    std_logic;                       -- third byte active low read enable
        RE3     :  out    std_logic;                       -- fourth byte active low read enable
        WE0     :  out    std_logic;                       -- first byte active low write enable
        WE1     :  out    std_logic;                       -- second byte active low write enable
        WE2     :  out    std_logic;                       -- third byte active low write enable
        WE3     :  out    std_logic;                       -- fourth byte active low write enable
        DB      :  inout  std_logic_vector(31 downto 0)    -- memory data bus
    );

end  SH2CPU;

architecture structural of sh2cpu is

    pure function SignExtend(slv : std_logic_vector) return std_logic_vector is
    begin
      -- slv -> signed, resize to sign-extend, then convert to slv. 
      return std_logic_vector(resize(signed(slv), SH2_WORDSIZE));
    end function;

    pure function ZeroExtend(slv : std_logic_vector) return std_logic_vector is
    begin
      return std_logic_vector(resize(unsigned(slv), SH2_WORDSIZE));
    end function;

    pure function LowByte(slv : std_logic_vector) return std_logic_vector is
    begin
      assert slv'length >= 8
      report "slv must be >= 8 bits."
      severity ERROR;

      return slv(7 downto 0);
    end function;

    pure function LowWord(slv : std_logic_vector) return std_logic_vector is
    begin
      assert slv'length >= 16
      report "slv must be >= 16 bits"
      severity ERROR;
      
      return slv(15 downto 0);
    end function;


    -- Register array inputs
    signal RegDataIn  : std_logic_vector(31 downto 0);    -- data to write to a register
    signal EnableIn   : std_logic;                        -- if data should be written to an input register
    signal RegInSel   : integer  range 15 downto 0;       -- which register to write data to
    signal RegASel    : integer  range 15 downto 0;       -- which register to read to bus A
    signal RegBSel    : integer  range 15 downto 0;       -- which register to read to bus B
    signal RegAxIn    : std_logic_vector(31 downto 0);    -- data to write to an address register
    signal RegAxInSel : integer  range 15 downto 0;       -- which address register to write to
    signal RegAxStore : std_logic;                        -- if data should be written to the address register
    signal RegA1Sel   : integer  range 15 downto 0;       -- which register to read to address bus 1
    signal RegA2Sel   : integer  range 15 downto 0;       -- which register to read to address bus 2

    -- register array outputs
    signal RegA       : std_logic_vector(31 downto 0);    -- register bus A
    signal RegB       : std_logic_vector(31 downto 0);    -- register bus B
    signal RegA1      : std_logic_vector(31 downto 0);    -- address register bus 1
    signal RegA2      : std_logic_vector(31 downto 0);    -- address register bus 2
    
    -- ALU inputs
    signal OperandA : std_logic_vector(31 downto 0);    -- first operand
    signal OperandB : std_logic_vector(31 downto 0);    -- second operand
    signal TIn      : std_logic;                        -- T bit from status register
    signal LoadA    : std_logic;                        -- determine if OperandA is loaded ('1') or zeroed ('0')
    signal FCmd     : std_logic_vector(3 downto 0);     -- F-Block operation
    signal CinCmd   : std_logic_vector(1 downto 0);     -- carry in operation
    signal SCmd     : std_logic_vector(2 downto 0);     -- shift operation
    signal ALUCmd   : std_logic_vector(1 downto 0);     -- ALU result select

    -- ALU outputs
    signal Result   : std_logic_vector(31 downto 0);   -- ALU result
    signal Cout     : std_logic;                       -- carry out
    signal Overflow : std_logic;                       -- signed overflow
    signal Zero     : std_logic;                       -- result is zero
    signal Sign     : std_logic;                       -- sign of result

    -- DMAU inputs
    signal RegSrc       : std_logic_vector(31 downto 0);
    signal R0Src        : std_logic_vector(31 downto 0);
    signal PCSrc        : std_logic_vector(31 downto 0);
    signal GBRIn        : std_logic_vector(31 downto 0);
    signal GBRWriteEn   : std_logic;
    signal DMAUOff4     : std_logic_vector(3 downto 0);
    signal DMAUOff8     : std_logic_vector(7 downto 0);
    signal BaseSel      : std_logic_vector(1 downto 0);
    signal IndexSel     : std_logic_vector(1 downto 0);
    signal OffScalarSel : std_logic_vector(1 downto 0);
    signal IncDecSel    : std_logic_vector(1 downto 0);

    -- DMAU outputs
    signal DataAddress  : std_logic_vector(31 downto 0);
    signal AddrSrcOut   : std_logic_vector(31 downto 0);
    signal GBROut       : std_logic_vector(31 downto 0);

    -- PMAU inputs
    signal RegIn       : std_logic_vector(31 downto 0);
    signal PRIn        : std_logic_vector(31 downto 0);
    signal PCAddrMode  : std_logic_vector(2 downto 0);
    signal PRWriteEn   : std_logic;
    signal PMAUOff8    : std_logic_vector(7 downto 0);
    signal PMAUOff12   : std_logic_vector(11 downto 0);
    signal PCIn        : std_logic_vector(31 downto 0);
    signal PCWriteCtrl : std_logic_vector(1 downto 0);

    -- PMAU outputs
    signal PCOut       : std_logic_vector(31 downto 0);
    signal PROut       : std_logic_vector(31 downto 0);

    -- Memory interface inputs/outputs
    signal MemEnable   : std_logic;
    signal ReadWrite   : std_logic;
    signal MemMode     : std_logic_vector(1 downto 0);
    signal MemAddress  : std_logic_vector(31 downto 0);
    signal MemDataOut  : std_logic_vector(31 downto 0);
    signal ReadMask    : std_logic_vector(3 downto 0);
    signal WriteMask   : std_logic_vector(3 downto 0);
    signal MemDataIn   : std_logic_vector(31 downto 0);

    -- CPU system/control registers
    signal MemOutSel        : std_logic_vector(2 downto 0);
    signal Disp             : std_logic_vector(11 downto 0);
    signal TSel             : std_logic_vector(2 downto 0);
    signal RegDataInSel     : std_logic_vector(3 downto 0);     -- source for register input data
    signal TFlagSel         : std_logic_vector(2 downto 0);     -- source for next value of T flag

    signal Immediate        : std_logic_vector(7 downto 0);     -- immediate value from instruction
    signal ImmediateMode    : std_logic;                        -- immediate extension mode (zero or signed)
    signal ImmediateExt     : std_logic_vector(31 downto 0);    -- sign-extended immediate
    signal ALUOpBSel        : std_logic;


    
    signal SR               : std_logic_vector(31 downto 0);  -- Status register.
    signal VBR              : std_logic_vector(31 downto 0);  -- Vector Base Register.
    signal MACL             : std_logic_vector(31 downto 0);  -- Multiply and Accumulate Low.
    signal MACH             : std_logic_vector(31 downto 0);  -- Multiply and Accumulate High.


    -- Aliases for status register bits.
    alias MBit  : std_logic is SR(9);   -- The M and Q bits are used by DIVOU/S and DIV1 instructions.
    alias QBit  : std_logic is SR(8);   -- ...
    alias I3Bit : std_logic is SR(7);   -- Interrupt mask bits.
    alias I2Bit : std_logic is SR(6);   -- ...
    alias I1Bit : std_logic is SR(5);   -- ...
    alias I0BIt : std_logic is SR(4);   -- ...
    alias SBit  : std_logic is SR(1);   -- Used by MAC instructions.
    alias TBit  : std_logic is SR(0);   -- True flag.

    signal SysRegCtrl       : std_logic;
    signal SysRegSel        : std_logic_vector(2 downto 0);
    signal SysRegSrc        : std_logic;

    signal NextSysReg       : std_logic_vector(31 downto 0);

    signal TNext        : std_logic;    -- Next value for T bit

    signal ExtMode     : std_logic_vector(1 downto 0);      -- mode for extending register value (zero or signed)

    signal ExtendedReg : std_logic_vector(31 downto 0);     -- extended register value (for EXT* instructions)

    -- RegA with the upper and lower halves of the low two bytes swapped (for the SWAP.B instruction).
    signal RegASwapB : std_logic_vector(31 downto 0);

    -- RegA with the high and low words swapped (for the SWAP.W instruction).
    signal RegASwapW : std_logic_vector(31 downto 0);

    -- The center 32-bits of RegB and RegA (ie, low word of RegB and high word of RegA).
    signal RegB_RegA_Center : std_logic_vector(31 downto 0);

    signal MemAddrSel  : std_logic;

    signal TCmp         : std_logic;
    signal TCmpSel      : std_logic_vector(2 downto 0);     -- how to compute T from ALU status flags

    signal StrCmp       : std_logic;    -- used to compare the bytes of RegA and RegB

    -- Value of the current system/control reginster of interest
    signal SysReg   : std_logic_vector(31 downto 0);

begin

    RE0 <= ReadMask(0) when (not clock) else '1';
    RE1 <= ReadMask(1) when (not clock) else '1';
    RE2 <= ReadMask(2) when (not clock) else '1';
    RE3 <= ReadMask(3) when (not clock) else '1';

    WE0 <= WriteMask(0) when (not clock) else '1';
    WE1 <= WriteMask(1) when (not clock) else '1';
    WE2 <= WriteMask(2) when (not clock) else '1';
    WE3 <= WriteMask(3) when (not clock) else '1';

    with MemAddrSel select 
        MemAddress <=  PCOut           when MemAddrSel_PMAU,
                       DataAddress     when MemAddrSel_DMAU,
                       (others => 'Z') when others;

    AB <= MemAddress;

    -- What to output to the data bus (to be written to an address).
    with MemOutSel select
        MemDataOut <=   RegA            when MemOut_RegA,
                        RegB            when MemOut_RegB,
                        SysReg          when MemOut_SysReg,
                        (others => 'Z') when others;

    ImmediateExt(7 downto 0) <= Immediate;

    with ImmediateMode select
        ImmediateExt(31 downto 8) <= (others => Immediate(7)) when ImmediateMode_SIGN,
                                     (others => '0')          when ImmediateMode_ZERO,
                                     (others => 'X')          when others;
                                 
                                 

    -- RegA with the high and low bytes swapped.
    RegASwapB <= RegA(31 downto 16) & RegA(7 downto 0) & RegA(15 downto 8);

    -- RegA with the high and low words swapped.
    RegASwapW <= RegA(15 downto 0) & RegA(31 downto 16);

    -- The center 32-bits of RegB and RegA (ie, low word of RegB and high word of RegA).
    RegB_RegA_Center <= RegB(15 downto 0) & RegA(31 downto 16);
    
    with ExtMode select
        ExtendedReg <= SignExtend(LowByte(RegB))  when  Ext_Sign_B_RegA,
                       SignExtend(LowWord(RegB))  when  Ext_Sign_W_RegA,
                       ZeroExtend(LowByte(RegB))  when  Ext_Zero_B_RegA,
                       ZeroExtend(LowWord(RegB))  when  Ext_Zero_W_RegA,
                       (others => 'X') when others;

    with SysRegSel select
        SysReg <= SR      when SysRegSel_SR,
                  GBROut  when SysRegSel_GBR,
                  VBR     when SysRegSel_VBR,
                  MACH    when SysRegSel_MACH,
                  MACL    when SysRegSel_MACL,
                  PROut   when SysRegSel_PR,
                  (others => 'X') when others;

    -- Select the data to write the the register based on the decoded instruction.
    with RegDataInSel select 
        RegDataIn <= Result                     when  RegDataIn_ALUResult,
                     ImmediateExt               when  RegDataIn_Immediate,
                     RegA                       when  RegDataIn_RegA,
                     RegB                       when  RegDataIn_RegB,
                     SysReg                     when  RegDataIn_SysReg,
                     RegASwapB                  when  RegDataIn_RegA_SWAP_B,
                     RegASwapW                  when  RegDataIn_RegA_SWAP_W,
                     RegB_RegA_Center           when  RegDataIn_REGB_REGA_CENTER,
                     ExtendedReg                when  RegDataIn_Ext,
                     MemDataIn                  when  RegDataIn_DB,

                     -- Extract the T bit from the status register.
                     SR and x"00000001"         when  RegDataIn_SR_TBit,
                     PROut                      when  RegDataIn_PR,
                     (others => 'X')            when  others;


    -- The address being stored to a register is the pre-decremented or 
    -- post-incremented address when we are in that mode. If we are not in 
    -- that mode, it is just the normal address.
    with IncDecSel select 
        RegAxIn <= AddrSrcOut  when IncDecSel_PRE_DEC | IncDecSel_POST_INC,
                   DataAddress when others;

    -- Route control signals and data into register array
    registers : entity work.SH2Regs
    port map (
        -- Inputs:
        clock       => clock,
        reset       => reset,
        RegDataIn   => RegDataIn,
        EnableIn    => EnableIn,
        RegInSel    => RegInSel,
        RegASel     => RegASel,
        RegBSel     => RegBSel,
        RegAxIn     => RegAxIn,
        RegAxInSel  => RegAxInSel,
        RegAxStore  => RegAxStore,
        RegA1Sel    => RegA1Sel,
        RegA2Sel    => RegA2Sel,
        -- Outputs:
        RegA    => RegA,
        RegB    => RegB,
        RegA1   => RegA1,
        RegA2   => RegA2
    );


    -- ALU Input mux
    OperandA <= RegA;

    with ALUOpBSel select
        OperandB <=  RegB            when ALUOpB_RegB,
                     ImmediateExt    when ALUOpB_Imm,
                     (others => 'X') when others;

    StrCmp <= '1' when (RegA(31 downto 24) = RegB(31 downto 24)) or
                       (RegA(23 downto 16) = RegB(23 downto 16)) or
                       (RegA(15 downto 8)  = RegB(15 downto 8))  or
                       (RegA(7 downto 0)   = RegB(7 downto 0))
                   else '0';

    -- Compute T flag value based on ALU output flags. Used for operations
    -- of the form CMP/XX.
    with TCmpSel select
        TCmp <= Zero                                when TCMP_EQ,   -- 1 if Rn = Rm
                Cout                                when TCMP_HS,   -- 1 if Rn >= Rm, unsigned
                not (Sign xor Overflow)             when TCMP_GE,   -- 1 if Rn >= Rm, signed
                Cout and (not Zero)                 when TCMP_HI,   -- 1 if Rn >  Rm, unsigned
                not ((Sign xor Overflow) or Zero)   when TCMP_GT,   -- 1 if Rn >  Rm, signed
                StrCmp                              when TCMP_STR,  -- 1 if Rn byte matches Rm byte
                'X' when others;


    with TFlagSel select
        TNext <=  TBit       when TFlagSel_T,           -- retain T flag
                  Cout       when TFlagSel_Carry,       -- Set T flag to ALU carry flag
                  Overflow   when TFlagSel_Overflow,    -- Set T flag to ALU overflow flag
                  Zero       when TFlagSel_Zero,        -- set T flag to ALU Zero flag
                  '0'        when TFlagSel_CLEAR,       -- clear T flag
                  '1'        when TFlagSel_SET,         -- set T flag
                  TCmp       when TFlagSel_CMP,         -- compute T flag based on compare result
                  'X'        when others;

    alu : entity work.sh2alu
    port map (
        -- Inputs:
        OperandA => OperandA,
        OperandB => OperandB,
        TIn      => TBit,
        LoadA    => LoadA,
        FCmd     => FCmd,
        CinCmd   => CinCmd,
        SCmd     => SCmd,
        ALUCmd   => ALUCmd,
        -- Outputs:
        Result   => Result,
        Cout     => Cout,
        Overflow => Overflow,
        Zero     => Zero,
        Sign     => Sign
    );


    -- Use RegA1 (@Rn) if we are writing and RegA2 (@Rm) if we are reading.
    with ReadWrite select
        RegSrc <= RegA2           when Mem_READ,
                  RegA1           when Mem_WRITE,
                  (others => 'X') when others;

    -- Connect PCSrc to PCOut
    PCSrc <= PCOut;

    -- R0 comes from RegA2 when we are reading and RegA1 when we are writing.
    with ReadWrite select
        R0Src <= RegA1            when Mem_READ,
                 RegA2            when Mem_WRITE,
                 (others => 'X')  when others;

    dmau : entity work.sh2dmau
    port map (
        -- Inputs:
        RegSrc       => RegSrc,
        R0Src        => R0Src,
        PCSrc        => PCSrc,
        GBRIn        => GBRIn,
        GBRWriteEn   => GBRWriteEn,
        Off4         => DMAUOff4,
        Off8         => DMAUOff8,
        BaseSel      => BaseSel,
        IndexSel     => IndexSel,
        OffScalarSel => OffScalarSel,
        IncDecSel    => IncDecSel,
        Clk          => clock,

        -- Outputs:
        Address    => DataAddress,
        AddrSrcOut => AddrSrcOut,
        GBROut     => GBROut
    );


    -- Default PC in is all zeroes.

    PCIn <= (others => '0');


    pmau : entity work.sh2pmau
    port map (
        -- Inputs:
        RegIn       => RegIn,
        PRIn        => PRIn,
        PRWriteEn   => PRWriteEn,
        PCIn        => PCIn,
        PCWriteCtrl => PCWriteCtrl,
        Off8        => PMAUOff8,
        Off12       => PMAUOff12,
        PCAddrMode  => PCAddrMode,
        Clk         => clock,
        Reset       => Reset,
        -- Outputs:
        PCOut       => PCOut,
        PROut       => PROut
    );

    memory_tx : entity work.MemoryInterfaceTx
    port map (
        -- Inputs:
        clock     => clock,
        MemEnable => MemEnable,
        ReadWrite => ReadWrite,
        MemMode   => MemMode,
        Address   => unsigned(MemAddress),
        MemDataOut=> MemDataOut,
        -- Outputs:
        RE => ReadMask,
        WE => WriteMask,
        DB => DB
    );

    memory_rx : entity work.MemoryInterfaceRx
    port map (
        -- Inputs:
        MemEnable => MemEnable,
        MemMode => MemMode,
        Address => unsigned(MemAddress),
        DB      => DB,
        -- Outputs:
        MemDataIn => MemDataIn
    );

    control_unit : entity work.SH2Control
    port map (
        -- Inputs:
        MemDataIn   => MemDataIn,
        TFlagIn     => TBit,
        clock       => clock,
        reset       => reset,

        -- Outputs:
        Immediate       => Immediate,
        ImmediateMode   => ImmediateMode,
        TFlagSel        => TFlagSel,
        ExtMode         => ExtMode,

        -- Memory interface control signals:
        MemEnable    => MemEnable,
        ReadWrite    => ReadWrite,
        MemMode      => MemMode,
        Disp         => Disp,
        MemSel       => MemSel,
        MemOutSel    => MemOutSel,
        MemAddrSel   => MemAddrSel,

        -- ALU control signals:
        ALUOpBSel    => ALUOpBSel,
        LoadA        => LoadA,
        FCmd         => FCmd,
        CinCmd       => CinCmd,
        SCmd         => SCmd,
        ALUCmd       => ALUCmd,
        TSel         => TSel,
        TCmpSel      => TCmpSel,

        -- Register Array control signals:
        RegDataInSel    => RegDataInSel,
        EnableIn        => EnableIn,
        RegInSel        => RegInSel,
        RegASel         => RegASel,
        RegBSel         => RegBSel,
        RegAxIn         => RegAxIn,
        RegAxInSel      => RegAxInSel,
        RegAxStore      => RegAxStore,
        RegA1Sel        => RegA1Sel,
        RegA2Sel        => RegA2Sel,

        -- DMAU control signals:
        GBRWriteEn      => GBRWriteEn,
        DMAUOff4        => DMAUOff4,
        DMAUOff8        => DMAUOff8,
        BaseSel         => BaseSel,
        IndexSel        => IndexSel,
        OffScalarSel    => OffScalarSel,
        IncDecSel       => IncDecSel,

        -- PMAU control signals:
        PCAddrMode   => PCAddrMode,
        PRWriteEn    => PRWriteEn,
        PMAUOff8     => PMAUOff8,
        PMAUOff12    => PMAUOff12,

        -- system control signals
        SysRegCtrl  => SysRegCtrl,
        SysRegSel   => SysRegSel,
        SysRegSrc   => SysRegSrc
    );

    -- Mux system register input values based on SysRegSrc. Note that individual
    -- write-enables (like GBRWriteEn and PRWriteEn) must be enabled seperately.
    NextSysReg <= RegB      when SysRegSrc = SysRegSrc_RegB else
                  MemDataIn when SysRegSrc = SysRegSrc_DB else
                  (others => 'X');

    GBRIn <= NextSysReg;
    PRIn  <= NextSysReg;

    register_proc: process(clock, reset)
      variable l : line;
    begin
        if reset = '0' then

            SR  <=  (others => '0');
            VBR <=  (others => '0');

        elsif rising_edge(clock) then
            SR(0) <= TNext;

            -- LogWithTime(l, "sh2_cpu.vhd: PC is 0x" & to_hstring(PCOut), LogFile);

            if SysRegCtrl = SysRegCtrl_LOAD then
                if SysRegSel = SysRegSel_SR then
                    SR <= NextSysReg;
                elsif SysRegSel = SysRegSel_VBR then
                    VBR <= NextSysReg;
                elsif SysRegSel = SysRegSel_MACH then
                    MACH <= NextSysReg;
                elsif SysRegSel = SysRegSel_MACL then
                    MACL <= NextSysReg;
                end if;
            end if;

            LogWithTime(l, "sh2_cpu.vhd: MemDataIn: " & to_hstring(MemDataIn), LogFile);
        end if;
    end process register_proc;

end architecture structural;
