----------------------------------------------------------------------------
--
--  Hitachi SH-2 CPU
--
--  This file implements the complete SH-2 CPU, implemented for EE 188, Spring
--  term 2024-2025. The CPU supports the entirety of the SH-2 instruction set
--  except for MUL/DIV, MAC instructions, and other multi-clock instructions.
--  The CPU consists of a register array, arithmetic logic unit (ALU), program
--  memory access unit (PMAU), data memory access unit (DMAU), and memory
--  interfaces for input/output. This CPU contains sixteen 32-bit general
--  purpose registers, along with special registers such as PC, PR, GBR, VBR,
--  and SR. Each instruction is encoded in 16 bits, and memory can be accessed
--  either byte-wise, word-wise, or longword-wise. Currently, the CPU is not
--  pipelined and executes every instruction in three clocks.
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
--     01 Jun 25  Zack Huang        Finishing documentation
--     07 Jun 25  Zack Huang        Move all control signals into record types for organization    -
--
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
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

use work.SH2Constants.all;
use work.SH2ControlSignals.all;
use work.SH2PmauConstants.all;
use work.SH2DmauConstants.all;
use work.MemoryInterfaceConstants.all;


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

    -- Sign-extends a standard logic vector to the SH2 wordsize
    pure function SignExtend(slv : std_logic_vector) return std_logic_vector is
    begin
      return std_logic_vector(resize(signed(slv), SH2_WORDSIZE));
    end function;

    -- Zero-extends a standard logic vector to the SH2 wordsize
    pure function ZeroExtend(slv : std_logic_vector) return std_logic_vector is
    begin
      return std_logic_vector(resize(unsigned(slv), SH2_WORDSIZE));
    end function;

    -- Returns the low byte of a standard logic vector
    pure function LowByte(slv : std_logic_vector) return std_logic_vector is
    begin
      assert slv'length >= 8
      report "slv must be >= 8 bits."
      severity ERROR;

      return slv(7 downto 0);
    end function;

    -- Returns the low word of a standard logic vector
    pure function LowWord(slv : std_logic_vector) return std_logic_vector is
    begin
      assert slv'length >= 16
      report "slv must be >= 16 bits"
      severity ERROR;
      
      return slv(15 downto 0);
    end function;


    -- Register array inputs
    signal RegDataIn  : std_logic_vector(31 downto 0);      -- data to write to a register
    signal RegAxIn    : std_logic_vector(31 downto 0);      -- data to write to an address register

    -- register array outputs
    signal RegA       : std_logic_vector(31 downto 0);      -- register bus A
    signal RegB       : std_logic_vector(31 downto 0);      -- register bus B
    signal RegA1      : std_logic_vector(31 downto 0);      -- address register bus 1
    signal RegA2      : std_logic_vector(31 downto 0);      -- address register bus 2
    
    -- ALU inputs
    signal OperandA : std_logic_vector(31 downto 0);        -- first operand
    signal OperandB : std_logic_vector(31 downto 0);        -- second operand

    -- ALU outputs
    signal Result   : std_logic_vector(31 downto 0);        -- ALU result
    signal Cout     : std_logic;                            -- carry out
    signal Overflow : std_logic;                            -- signed overflow
    signal Zero     : std_logic;                            -- result is zero
    signal Sign     : std_logic;                            -- sign of result

    -- DMAU inputs
    signal RegSrc       : std_logic_vector(31 downto 0);    -- input register
    signal R0Src        : std_logic_vector(31 downto 0);    -- R0 register input
    signal PCSrc        : std_logic_vector(31 downto 0);    -- program counter source
    signal GBRIn        : std_logic_vector(31 downto 0);    -- GBR input

    -- DMAU outputs
    signal DataAddress  : std_logic_vector(31 downto 0);    -- output address
    signal AddrSrcOut   : std_logic_vector(31 downto 0);    -- incremented/decremented address (store back to register)
    signal GBROut       : std_logic_vector(31 downto 0);    -- GBR output
                                                            --
    -- PMAU inputs
    signal RegIn       : std_logic_vector(31 downto 0);     -- register source input
    signal PRIn        : std_logic_vector(31 downto 0);     -- PR register input (for writing to PR)

    -- PMAU outputs
    signal PCCalcOut : std_logic_vector(31 downto 0);       -- calculated PC address output
    signal PCRegOut  : std_logic_vector(31 downto 0);       -- currenet value of the PC register
    signal PROut     : std_logic_vector(31 downto 0);       -- PR (procedure register) output

    -- Memory interface inputs
    signal MemAddress  : std_logic_vector(31 downto 0);     -- memory address bus (MUST BE ALIGNED)
    signal MemDataOut  : std_logic_vector(31 downto 0);     -- the data to output to memory

    -- Memory interface outputs
    signal ReadMask    : std_logic_vector(3 downto 0);      -- read enable mask (active low)
    signal WriteMask   : std_logic_vector(3 downto 0);      -- write enable mask (active low)
    signal MemDataIn   : std_logic_vector(31 downto 0);     -- the data read in from memory

    -- CPU internal control signals
    signal DelayedBranchTaken   : std_logic;                    -- Whether the delayed branch is taken or not.
    
    -- CPU system/control registers
    signal SR               : std_logic_vector(31 downto 0);    -- Status register.
    signal VBR              : std_logic_vector(31 downto 0);    -- Vector Base Register.
    signal MACL             : std_logic_vector(31 downto 0);    -- Multiply and Accumulate Low.
    signal MACH             : std_logic_vector(31 downto 0);    -- Multiply and Accumulate High.

    -- Aliases for status register bits.
    alias MBit  : std_logic is SR(9);   -- The M and Q bits are used by DIVOU/S and DIV1 instructions.
    alias QBit  : std_logic is SR(8);   -- ...
    alias I3Bit : std_logic is SR(7);   -- Interrupt mask bits.
    alias I2Bit : std_logic is SR(6);   -- ...
    alias I1Bit : std_logic is SR(5);   -- ...
    alias I0BIt : std_logic is SR(4);   -- ...
    alias SBit  : std_logic is SR(1);   -- Used by MAC instructions.
    alias TBit  : std_logic is SR(0);   -- True flag.

    -- Intermediate terms
    signal ExtendedReg      : std_logic_vector(31 downto 0);    -- extended register value (for EXT* instructions)
    signal NextSysReg       : std_logic_vector(31 downto 0);    -- data to input into a system register
    signal ImmediateExt     : std_logic_vector(31 downto 0);    -- sign-extended immediate
    signal TCmp             : std_logic;                        -- The value of T generated from a compare
    signal StrCmp           : std_logic;                        -- used to compare the bytes of RegA and RegB
    signal SysReg           : std_logic_vector(31 downto 0);    -- Value of selected system/control register
    signal PCNext           : std_logic_vector(31 downto 0);    -- The current PC incremented by two.
    signal PrevPCReg        : std_logic_vector(31 downto 0);    -- the prevous value fo PC
    signal PCUsed           : std_logic_vector(31 downto 0);    -- The PC that will be fetched from program memory.
    signal TNext            : std_logic;                        -- next value for T bit in SR

    -- RegA with the upper and lower halves of the low two bytes swapped (for the SWAP.B instruction).
    signal RegASwapB : std_logic_vector(31 downto 0);

    -- RegA with the high and low words swapped (for the SWAP.W instruction).
    signal RegASwapW : std_logic_vector(31 downto 0);

    -- The center 32-bits of RegB and RegA (ie, low word of RegB and high word of RegA).
    signal RegB_RegA_Center : std_logic_vector(31 downto 0);

    signal MemCtrl             : mem_ctrl_t;                       -- memory interface control signals
    signal ALUCtrl             : alu_ctrl_t;                       -- ALU control signals
    signal RegCtrl             : reg_ctrl_t;                       -- register array control signals
    signal DMAUCtrl            : dmau_ctrl_t;                      -- DMAU control signals
    signal PMAUCtrl            : pmau_ctrl_t;                      -- PMAU control signals
    signal SysCtrl             : sys_ctrl_t;                       -- system control signals

begin

    -- Output read enable signals when clock is low
    RE0 <= ReadMask(0) when (not clock) else '1';
    RE1 <= ReadMask(1) when (not clock) else '1';
    RE2 <= ReadMask(2) when (not clock) else '1';
    RE3 <= ReadMask(3) when (not clock) else '1';

    -- Output write enable signals when clock is low
    WE0 <= WriteMask(0) when (not clock) else '1';
    WE1 <= WriteMask(1) when (not clock) else '1';
    WE2 <= WriteMask(2) when (not clock) else '1';
    WE3 <= WriteMask(3) when (not clock) else '1';

    StorePCReg : process(clock)
    begin
        if rising_edge(clock) then
            -- Store previous PC on rising clock
            PrevPCReg <= PCRegOut;
        end if;
    end process;

    -- The "next" PC is the current value of the PC register (NOT PCCalcOut) + 2.
    PCNext <= std_logic_vector(unsigned(PrevPCReg) + to_unsigned(2, 32));

    -- Decide which value of PC should be used
    -- PCUsed <= PCNext when (DelayedBranchTaken = '1') else PCCalcOut;
    PCUsed <= PCRegOut;

    -- Decide which memory address to output
    with MemCtrl.AddrSel select 
        MemAddress <=  PCUsed           when MemAddrSel_PMAU,
                       DataAddress      when MemAddrSel_DMAU,
                       (others => 'Z')  when others;

    AB <= MemAddress;   -- Output memory address to the address bus

    MemSel <= MemCtrl.Sel;

    -- What to output to the data bus (to be written to an address).
    with MemCtrl.OutSel select
        MemDataOut <=   RegA            when MemOut_RegA,
                        RegB            when MemOut_RegB,
                        SysReg          when MemOut_SysReg,
                        (others => 'Z') when others;

    -- Compute the zero/sign-extended immediate from an instruction
    ImmediateExt(7 downto 0) <= AluCtrl.Immediate;
    with AluCtrl.ImmediateMode select
        ImmediateExt(31 downto 8) <= (others => AluCtrl.Immediate(7)) when ImmediateMode_SIGN,
                                     (others => '0')          when ImmediateMode_ZERO,
                                     (others => 'X')          when others;

    -- RegA with the high and low bytes swapped.
    RegASwapB <= RegA(31 downto 16) & RegA(7 downto 0) & RegA(15 downto 8);

    -- RegA with the high and low words swapped.
    RegASwapW <= RegA(15 downto 0) & RegA(31 downto 16);

    -- The center 32-bits of RegB and RegA (ie, low word of RegB and high word of RegA).
    RegB_RegA_Center <= RegB(15 downto 0) & RegA(31 downto 16);
    
    -- the zero/sign-extended version of register B
    with AluCtrl.ExtMode select
        ExtendedReg <= SignExtend(LowByte(RegB))  when  Ext_SignB_RegA,
                       SignExtend(LowWord(RegB))  when  Ext_SignW_RegA,
                       ZeroExtend(LowByte(RegB))  when  Ext_ZeroB_RegA,
                       ZeroExtend(LowWord(RegB))  when  Ext_ZeroW_RegA,
                       (others => 'X') when others;

    -- Choose which system register to select
    with SysCtrl.RegSel select
        SysReg <= SR      when SysRegSel_SR,
                  GBROut  when SysRegSel_GBR,
                  VBR     when SysRegSel_VBR,
                  MACH    when SysRegSel_MACH,
                  MACL    when SysRegSel_MACL,
                  PROut   when SysRegSel_PR,
                  (others => 'X') when others;

    -- Select the data to write the the register based on the decoded instruction.
    with RegCtrl.DataInSel select 
        RegDataIn <= Result                     when  RegDataIn_ALUResult,
                     ImmediateExt               when  RegDataIn_Immediate,
                     RegA                       when  RegDataIn_RegA,
                     RegB                       when  RegDataIn_RegB,
                     SysReg                     when  RegDataIn_SysReg,
                     RegASwapB                  when  RegDataIn_RegA_SwapB,
                     RegASwapW                  when  RegDataIn_RegA_SwapW,
                     RegB_RegA_Center           when  RegDataIn_REGB_REGA_CENTER,
                     ExtendedReg                when  RegDataIn_Ext,
                     MemDataIn                  when  RegDataIn_DB,

                     -- Extract the T bit from the status register.
                     SR and X"00000001"         when  RegDataIn_SR_TBit,
                     PROut                      when  RegDataIn_PR,
                     (others => 'X')            when  others;


    -- The address being stored to a register is the pre-decremented or 
    -- post-incremented address when we are in that mode. If we are not in 
    -- that mode, it is just the normal address.
    with DMAUCtrl.IncDecSel select 
        RegAxIn <= AddrSrcOut  when IncDecSel_PRE_DEC | IncDecSel_POST_INC,
                   DataAddress when others;

    -- Route control signals and data into register array
    registers : entity work.SH2Regs
    port map (
        -- Inputs:
        clock       => clock,
        reset       => reset,
        RegDataIn   => RegDataIn,
        EnableIn    => RegCtrl.EnableIn,
        RegInSel    => RegCtrl.InSel,
        RegASel     => RegCtrl.ASel,
        RegBSel     => RegCtrl.BSel,
        RegAxIn     => RegAxIn,
        RegAxInSel  => DMAUCtrl.AxInSel,
        RegAxStore  => DMAUCtrl.AxStore,
        RegA1Sel    => DMAUCtrl.A1Sel,
        RegA2Sel    => DMAUCtrl.A2Sel,
        -- Outputs:
        RegA    => RegA,
        RegB    => RegB,
        RegA1   => RegA1,
        RegA2   => RegA2
    );


    -- Always use RegA as Operand A for ALU
    OperandA <= RegA;

    -- Input mux for ALU Operand B
    with ALUCtrl.OpBSel select
        OperandB <=  RegB            when ALUOpB_RegB,
                     ImmediateExt    when ALUOpB_Imm,
                     (others => 'X') when others;

    -- If two registers share a byte value
    StrCmp <= '1' when (RegA(31 downto 24) = RegB(31 downto 24)) or
                       (RegA(23 downto 16) = RegB(23 downto 16)) or
                       (RegA(15 downto 8)  = RegB(15 downto 8))  or
                       (RegA(7 downto 0)   = RegB(7 downto 0))
                   else '0';

    -- Compute T flag value based on ALU output flags. Used for operations
    -- of the form CMP/XX.
    with AluCtrl.TCmpSel select
        TCmp <= Zero                                when TCMP_EQ,   -- 1 if Rn = Rm
                Cout                                when TCMP_HS,   -- 1 if Rn >= Rm, unsigned
                not (Sign xor Overflow)             when TCMP_GE,   -- 1 if Rn >= Rm, signed
                Cout and (not Zero)                 when TCMP_HI,   -- 1 if Rn >  Rm, unsigned
                not ((Sign xor Overflow) or Zero)   when TCMP_GT,   -- 1 if Rn >  Rm, signed
                StrCmp                              when TCMP_STR,  -- 1 if Rn byte matches Rm byte
                'X' when others;

    -- Select what value T should be set to
    with AluCtrl.TFlagSel select
        TNext <=  TBit       when TFlagSel_T,           -- retain T flag
                  Cout       when TFlagSel_Carry,       -- Set T flag to ALU carry flag
                  Overflow   when TFlagSel_Overflow,    -- Set T flag to ALU overflow flag
                  Zero       when TFlagSel_Zero,        -- set T flag to ALU Zero flag
                  '0'        when TFlagSel_CLEAR,       -- clear T flag
                  '1'        when TFlagSel_SET,         -- set T flag
                  TCmp       when TFlagSel_CMP,         -- compute T flag based on compare result
                  'X'        when others;

    -- Route ALU control signals
    alu : entity work.sh2alu
    port map (
        -- Inputs:
        OperandA => OperandA,
        OperandB => OperandB,
        TIn      => TBit,
        LoadA    => ALUCtrl.LoadA,
        FCmd     => ALUCtrl.FCmd,
        CinCmd   => ALUCtrl.CinCmd,
        SCmd     => ALUCtrl.SCmd,
        ALUCmd   => ALUCtrl.ALUCmd,
        -- Outputs:
        Result   => Result,
        Cout     => Cout,
        Overflow => Overflow,
        Zero     => Zero,
        Sign     => Sign
    );

    -- Use RegA1 (@Rn) if we are writing and RegA2 (@Rm) if we are reading.
    with MemCtrl.ReadWrite select
        RegSrc <= RegA2           when Mem_READ,
                  RegA1           when Mem_WRITE,
                  (others => 'X') when others;

    -- Connect PCSrc to PCUsed
    PCSrc <= PCUsed;

    -- R0 comes from RegA2 when we are reading and RegA1 when we are writing.
    with MemCtrl.ReadWrite select
        R0Src <= RegA1            when Mem_READ,
                 RegA2            when Mem_WRITE,
                 (others => 'X')  when others;

    -- Route DMAU control signals
    dmau : entity work.sh2dmau
    port map (
        -- Inputs:
        RegSrc       => RegSrc,
        R0Src        => R0Src,
        PCSrc        => PCSrc,
        GBRIn        => GBRIn,
        GBRWriteEn   => DMAUCtrl.GBRWriteEn,
        Off4         => DMAUCtrl.Off4,
        Off8         => DMAUCtrl.Off8,
        BaseSel      => DMAUCtrl.BaseSel,
        IndexSel     => DMAUCtrl.IndexSel,
        OffScalarSel => DMAUCtrl.OffScalarSel,
        IncDecSel    => DMAUCtrl.IncDecSel,
        Clk          => clock,
        -- Outputs:
        Address    => DataAddress,
        AddrSrcOut => AddrSrcOut,
        GBROut     => GBROut
    );


    -- PMAU Register input is always RegB.
    RegIn <= RegB;

    -- Route PMAU control signals
    pmau : entity work.sh2pmau
    port map (
        -- Inputs:
        RegIn       => RegIn,
        PRIn        => PRIn,
        PCIn        => (others => '0'),         -- default PC is all 0s
        PRWriteEn   => SysCtrl.PRWriteEn,
        PCWriteCtrl => PMAUCtrl.PCWriteCtrl,
        Off8        => PMAUCtrl.Off8,
        Off12       => PMAUCtrl.Off12,
        PCAddrMode  => PMAUCtrl.PCAddrMode,
        Clk         => clock,
        Reset       => Reset,
        -- Outputs:
        PCRegOut    => PCRegOut,
        PCCalcOut   => PCCalcOut,
        PROut       => PROut
    );

    -- Route memory interface control signals
    memory_tx : entity work.MemoryInterfaceTx
    port map (
        -- Inputs:
        clock      => clock,
        MemEnable  => MemCtrl.Enable,
        ReadWrite  => MemCtrl.ReadWrite,
        MemMode    => MemCtrl.Mode,
        Address    => unsigned(MemAddress),
        MemDataOut => MemDataOut,
        -- Outputs:
        RE => ReadMask,
        WE => WriteMask,
        DB => DB
    );

    -- Route memory interface control signals
    memory_rx : entity work.MemoryInterfaceRx
    port map (
        -- Inputs:
        MemEnable => MemCtrl.Enable,
        MemMode => MemCtrl.Mode,
        Address => unsigned(MemAddress),
        DB      => DB,
        -- Outputs:
        MemDataIn => MemDataIn
    );

    -- Route control unit control signals
    control_unit : entity work.SH2Control
    port map (
        -- Inputs:
        MemDataIn   => MemDataIn,
        TFlagIn     => TBit,
        clock       => clock,
        reset       => reset,

        -- Outputs:
        MemCtrl => MemCtrl,
        ALUCtrl => ALUCtrl,
        RegCtrl => RegCtrl,
        DMAUCtrl => DMAUCtrl,
        PMAUCtrl => PMAUCtrl,
        SysCtrl => SysCtrl
    );

    -- Mux system register input values based on SysRegSrc. Note that individual
    -- write-enables (like GBRWriteEn and PRWriteEn) must be enabled seperately.
    NextSysReg <= RegB      when SysCtrl.RegSrc = SysRegSrc_RegB  else
                  MemDataIn when SysCtrl.RegSrc = SysRegSrc_DB    else

                  -- The return address of a BSR is the PC at the point of decoding the BSR plus 4.
                  std_logic_vector(unsigned(PCRegOut) + to_unsigned(4, 32))  when SysCtrl.RegSrc = SysRegSrc_PC    else

                  (others => 'X');

    GBRIn <= NextSysReg;    -- set GBR to selected sysreg value (when GBRWriteEn active)
    PRIn  <= NextSysReg;    -- set PR to selected sysreg value (when PRWriteEn active)

    register_proc: process(clock, reset)
    begin
        if reset = '0' then
            -- Reset system registers (async)
            SR   <=  (others => '0');
            VBR  <=  (others => '0');
            MACH <=  (others => '0');
            MACL <=  (others => '0');

        elsif rising_edge(clock) then
            SR(0) <= TNext;     -- set new value of T

            if SysCtrl.RegCtrl = SysRegCtrl_LOAD then
                -- Load new value into a system register
                -- (note that PR and GBR are handled separately)
                if SysCtrl.RegSel = SysRegSel_SR then
                    SR <= NextSysReg;
                elsif SysCtrl.RegSel = SysRegSel_VBR then
                    VBR <= NextSysReg;
                elsif SysCtrl.RegSel = SysRegSel_MACH then
                    MACH <= NextSysReg;
                elsif SysCtrl.RegSel = SysRegSel_MACL then
                    MACL <= NextSysReg;
                end if;
            elsif SysCtrl.RegCtrl = SysRegCtrl_CLEAR then
                -- Clear a system register value
                -- (note that PR and GBR are handled separately)
                if SysCtrl.RegSel = SysRegSel_SR then
                    SR <= (others => '0');
                elsif SysCtrl.RegSel = SysRegSel_VBR then
                    VBR <= (others => '0');
                elsif (SysCtrl.RegSel = SysRegSel_MACH) or (SysCtrl.RegSel = SysRegSel_MACL) then
                    -- Reset both MACH and MACL for CLRMAC instruction
                    MACH <= (others => '0');
                    MACL <= (others => '0');
                end if;
            end if;
        end if;
    end process register_proc;

end architecture structural;
