------------------------------------------------------------------------------
-- SH2ALU
--
-- This entity implements all the operations required for the SH-2 ALU. It
-- supports arithmetic, logical, and shift operations as required for the SH-2
-- instruction set. It does not implement DSP operations such as MAC or barrel
-- shifting. This entity does not do instruction decoding - the control unit
-- must provide the correct operand values (which could come from registers or
-- immediate values) and control signals to produce the correct result (which
-- may then be written back to a register by the control unit). Additionally, 
-- this entity outputs carry, sign, overflow, and zero flags, which can be
-- used by the control unit to set the T bit in the status register.
--
------------------------------------------------------------------------------

-- import libraries
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Set the SH2 ALU control signals as follows for each instruction:
-- It is assumed that single-operand instructions operate on OperandB,
-- while dual-operand instructions operate on OperandA and OperandB, in
-- that order. This is because SH-2 instructions are almost in the format
-- "OPCODE Rm, Rn", which usually does something like Rn <= Rn OPERATION Rm.
-- If the instruction is unary, then it usually only acts on Rm. As such, the
-- CPU can assign OperandA <= Rn and OperandB <= Rm, and then put the result
-- back in Rn. This is convenient because of the implementation of the generic
-- ALU being used inside this SH-2-specific ALU. We exclude DSP instructions
-- (MAC, SHLLn, SHLRN, etc) and multi-clock instructions (DIV, MUL, etc) for
-- now.
--
-- The possible ALU operations with the control signals that produce them
-- are listed below. Note that this entity also outputs carry, overflow,
-- zero, and sign flags so that they can be used by the CPU for setting
-- the T bit, doing sign-extension, checking compare results, etc.
--
-- ADD(C,V) - Result <= OperandA + OperandB
--  - FCmd <= FCmd_B
--  - CinCmd <= CinCmd_ZERO
--  - OpASel <= OpA_NONE
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_ADDER
-- SUB(C,V), CMP/XX - Result <= OperandA - OperandB
--  - FCmd <= FCmd_BNOT
--  - CinCmd <= CinCmd_ONE
--  - OpASel <= OpA_NONE
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_ADDER
-- NEG(C) - Result <= 0 - OperandB
--  - FCmd <= FCmd_BNOT
--  - CinCmd <= CinCmd_ONE
--  - OpASel <= OpA_ZERO
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_ADDER
-- DT - Result <= OperandB - 1
--  - FCmd <= FCmd_ONES
--  - CinCmd <= CinCmd_ZERO
--  - OpASel <= OpA_B
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_ADDER
-- MOV - Result <= OperandB
--  - FCmd <= FCmd_B
--  - CinCmd <= CinCmd_ZERO
--  - OpASel <= OpA_B
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_FBLOCK
-- AND/TST - Result <= OperandA & OperandB
--  - FCmd <= FCmd_AND
--  - CinCmd <= CinCmd_ZERO
--  - OpASel <= OpA_NONE
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_FBLOCK
-- OR - Result <= OperandA | OperandB
--  - FCmd <= FCmd_OR
--  - CinCmd <= CinCmd_ZERO
--  - OpASel <= OpA_NONE
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_FBLOCK
-- XOR - Result <= OperandA ^ OperandB
--  - FCmd <= FCmd_XOR
--  - CinCmd <= CinCmd_ZERO
--  - OpASel <= OpA_NONE
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_FBLOCK
-- NOT - Result <= ~OperandB
--  - FCmd   <= FCmd_BNOT
--  - CinCmd <= CinCmd_ZERO
--  - OpASel <= "XX"
--  - SCmd   <= "XX"
--  - ALUCmd <= ALUCmd_FBLOCK
-- SHAL/SHLL - Result <= OperandB << 1
--  - FCmd   <= "XX"
--  - CinCmd <= CinCmd_ZERO
--  - OpASel <= OpA_B
--  - SCmd   <= SCmd_LSR
--  - ALUCmd <= ALUCmd_SHIFT
-- SHAR - Result <= OperandB >> 1 (sign-extended)
--  - FCmd   <= "XX"
--  - CinCmd <= CinCmd_ZERO
--  - OpASel <= OpA_B
--  - SCmd   <= SCmd_ASR
--  - ALUCmd <= ALUCmd_SHIFT
-- SHLR - Result <= OperandB >> 1
--  - FCmd   <= "XX"
--  - CinCmd <= CinCmd_ZERO
--  - OpASel <= OpA_B
--  - SCmd   <= SCmd_LSR
--  - ALUCmd <= ALUCmd_SHIFT
-- ROTL - Result <= rotate_left(OperandB)
--  - FCmd   <= "XX"
--  - CinCmd <= CinCmd_ZERO
--  - OpASel <= OpA_B
--  - SCmd   <= SCmd_ROL
--  - ALUCmd <= ALUCmd_SHIFT
-- ROTR - Result <= rotate_right(OperandB)
--  - FCmd   <= "XX"
--  - CinCmd <= CinCmd_ZERO
--  - OpASel <= OpA_B
--  - SCmd   <= SCmd_ROR
--  - ALUCmd <= ALUCmd_SHIFT
-- ROTCL - Result <= rotate_left(OperandB, T)
--  - FCmd   <= "XX"
--  - CinCmd <= CinCmd_CIN
--  - OpASel <= OpA_B
--  - SCmd   <= SCmd_RLC
--  - ALUCmd <= ALUCmd_SHIFT
-- ROTCR - Result <= rotate_right(OperandB, T)
--  - FCmd   <= "XX"
--  - CinCmd <= CinCmd_CIN
--  - OpASel <= OpA_B
--  - SCmd   <= SCmd_RRC
--  - ALUCmd <= ALUCmd_SHIFT


entity  SH2ALU  is
    port(
        OperandA : in      std_logic_vector(31 downto 0);   -- first operand
        OperandB : in      std_logic_vector(31 downto 0);   -- second operand
        TIn      : in      std_logic;                       -- T bit from status register
        OpASel   : in      std_logic_vector(1 downto 0);    -- determine how to interpret operand A
        FCmd     : in      std_logic_vector(3 downto 0);    -- F-Block operation
        CinCmd   : in      std_logic_vector(1 downto 0);    -- carry in operation
        SCmd     : in      std_logic_vector(2 downto 0);    -- shift operation
        ALUCmd   : in      std_logic_vector(1 downto 0);    -- ALU result select

        Result   : buffer  std_logic_vector(32 downto 0);   -- ALU result
        Cout     : out     std_logic;                       -- carry out
        Overflow : out     std_logic;                       -- signed overflow
        Zero     : out     std_logic;                       -- result is zero
        Sign     : out     std_logic                        -- sign of result
    );

    constant OpA_ZERO     : std_logic_vector(1 downto 0) := "00";   -- clear OperandA before using it in a computation
    constant OpA_ONE      : std_logic_vector(1 downto 0) := "01";   -- Set OperandA value to 1 
    constant OpA_B        : std_logic_vector(1 downto 0) := "10";   -- Set OperandA to have the value of OperandB
    constant OpA_NONE     : std_logic_vector(1 downto 0) := "11";   -- Pass OperandA through

    
    -- FBlock commands (for convenience)
    constant FCmd_A         : std_logic_vector(3 downto 0) := "1100";
    constant FCmd_B         : std_logic_vector(3 downto 0) := "1010";
    constant FCmd_BNOT      : std_logic_vector(3 downto 0) := "0101";
    constant FCmd_ONES      : std_logic_vector(3 downto 0) := "1111";
    constant FCmd_AND       : std_logic_vector(3 downto 0) := "1000";
    constant FCmd_OR        : std_logic_vector(3 downto 0) := "1110";
    constant FCmd_XOR       : std_logic_vector(3 downto 0) := "0110";

end  SH2ALU;


architecture  structural  of  SH2ALU  is

    component  ALU  is

        generic (
            wordsize : integer := 8      -- default width is 8-bits
        );

        port(
            ALUOpA   : in      std_logic_vector(wordsize - 1 downto 0);   -- first operand
            ALUOpB   : in      std_logic_vector(wordsize - 1 downto 0);   -- second operand
            Cin      : in      std_logic;                                 -- carry in
            FCmd     : in      std_logic_vector(3 downto 0);              -- F-Block operation
            CinCmd   : in      std_logic_vector(1 downto 0);              -- carry in operation
            SCmd     : in      std_logic_vector(2 downto 0);              -- shift operation
            ALUCmd   : in      std_logic_vector(1 downto 0);              -- ALU result select

            Result   : buffer  std_logic_vector(wordsize - 1 downto 0);   -- ALU result
            Cout     : out     std_logic;                                 -- carry out
            HalfCout : out     std_logic;                                 -- half carry out
            Overflow : out     std_logic;                                 -- signed overflow
            Zero     : out     std_logic;                                 -- result is zero
            Sign     : out     std_logic                                  -- sign of result
        );

    end  component;

    signal OperandAMux : std_logic_vector(31 downto 0);     -- the input to the internal ALU

    -- Constant values for convenience
    constant ZEROS : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(0, 32));
    constant ONE   : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(1, 32));

begin

    -- By adding this mux, we should be able to support every SH-2 ALU
    -- operation. This is needed due to the internal structure of the ALU
    -- entity. In some cases, we need OperandB to be the "first" operand
    -- in an operation, such as to do DT (which must calculate OperandB - 1).
    OperandAMux <= OperandA when OpASel = OpA_NONE else
                   ZEROS    when OpASel = OpA_ZERO else
                   ONE      when OpASel = OpA_ONE  else
                   OperandB when OpASel = OpA_B;

    -- We use a generic ALU to implement all of the SH-2 ALU operations. We
    -- pass in the T bit in place of a dedicated carry input, and the CPU can
    -- route the correct output flag (carry, sign, zero, overflow) back into
    -- the status register.
    ALUInternal: ALU
    generic map (wordsize => 32)
    port map  (
        ALUOpA => OperandAMux,
        ALUOpB => OperandB,
        Cin => Tin,
        FCmd => FCmd,
        SCmd => SCmd,
        ALUCmd => ALUCmd,
        CinCmd => CinCmd,
        Result => Result,
        Cout => Cout,
        Overflow => Overflow,
        Zero => Zero,
        Sign => Sign
    );

end  structural;
