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
--  Revision History:
--     26 Apr 25    Zack Huang      copied over from HW 1, prepare for testing
--     25 May 25    Zack Huang      added barrel shifter (extra credit instructions)
--
------------------------------------------------------------------------------

-- Import libraries
library ieee;
use ieee.std_logic_1164.all;

package  SH2ALUConstants  is

--  Adder carry in select constants
   constant CinCmd_ZERO   : std_logic_vector(1 downto 0) := "00";
   constant CinCmd_ONE    : std_logic_vector(1 downto 0) := "01";
   constant CinCmd_CIN    : std_logic_vector(1 downto 0) := "10";
   constant CinCmd_CINBAR : std_logic_vector(1 downto 0) := "11";


--  Shifter command constants
   constant SCmd_LEFT  : std_logic_vector(2 downto 0) := "0--"; -- BIT DECODED - DO NOT CHANGE
   constant SCmd_LSL   : std_logic_vector(2 downto 0) := "000"; -- BIT DECODED - DO NOT CHANGE
   constant SCmd_ASL   : std_logic_vector(2 downto 0) := "001"; -- BIT DECODED - DO NOT CHANGE
   constant SCmd_ROL   : std_logic_vector(2 downto 0) := "010"; -- BIT DECODED - DO NOT CHANGE
   constant SCmd_RLC   : std_logic_vector(2 downto 0) := "011"; -- BIT DECODED - DO NOT CHANGE
   constant SCmd_RIGHT : std_logic_vector(2 downto 0) := "1--"; -- BIT DECODED - DO NOT CHANGE
   constant SCmd_LSR   : std_logic_vector(2 downto 0) := "100"; -- BIT DECODED - DO NOT CHANGE
   constant SCmd_ASR   : std_logic_vector(2 downto 0) := "101"; -- BIT DECODED - DO NOT CHANGE
   constant SCmd_ROR   : std_logic_vector(2 downto 0) := "110"; -- BIT DECODED - DO NOT CHANGE
   constant SCmd_RRC   : std_logic_vector(2 downto 0) := "111"; -- BIT DECODED - DO NOT CHANGE

   -- Barrel shifter command constants
   constant BSCmd_L2  : std_logic_vector(2 downto 0) := "000"; -- BIT DECODED - DO NOT CHANGE
   constant BSCmd_R2  : std_logic_vector(2 downto 0) := "001"; -- BIT DECODED - DO NOT CHANGE
   constant BSCmd_L8  : std_logic_vector(2 downto 0) := "010"; -- BIT DECODED - DO NOT CHANGE
   constant BSCmd_R8  : std_logic_vector(2 downto 0) := "011"; -- BIT DECODED - DO NOT CHANGE
   constant BSCmd_L16 : std_logic_vector(2 downto 0) := "100"; -- BIT DECODED - DO NOT CHANGE
   constant BSCmd_R16 : std_logic_vector(2 downto 0) := "101"; -- BIT DECODED - DO NOT CHANGE

--  ALU command constants
   constant ALUCmd_FBLOCK  : std_logic_vector(1 downto 0) := "00";
   constant ALUCmd_ADDER   : std_logic_vector(1 downto 0) := "01";
   constant ALUCmd_SHIFT   : std_logic_vector(1 downto 0) := "10";
   constant ALUCmd_BSHIFT  : std_logic_vector(1 downto 0) := "11";


    constant OpA_Zero     : std_logic_vector(1 downto 0) := "00"; -- clear OperandA before using it in a computation
    constant OpA_One      : std_logic_vector(1 downto 0) := "01"; -- Set OperandA value to 1
    constant OpA_B        : std_logic_vector(1 downto 0) := "10"; -- Set OperandA to have the value of OperandB
    constant OpA_None     : std_logic_vector(1 downto 0) := "11"; -- Pass OperandA through

    -- FBlock commands (for convenience)
    constant FCmd_A         : std_logic_vector(3 downto 0) := "1100";
    constant FCmd_B         : std_logic_vector(3 downto 0) := "1010";
    constant FCmd_BNOT      : std_logic_vector(3 downto 0) := "0101";
    constant FCmd_ONES      : std_logic_vector(3 downto 0) := "1111";
    constant FCmd_AND       : std_logic_vector(3 downto 0) := "1000";
    constant FCmd_OR        : std_logic_vector(3 downto 0) := "1110";
    constant FCmd_XOR       : std_logic_vector(3 downto 0) := "0110";

end package;

-- import libraries

library ieee;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;
use work.SH2ALUConstants.all;

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
--  - LoadA <= '1'
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_ADDER
-- SUB(C,V), CMP/XX - Result <= OperandA - OperandB
--  - FCmd <= FCmd_BNOT
--  - CinCmd <= CinCmd_ONE
--  - LoadA <= '1'
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_ADDER
-- NEG(C) - Result <= 0 - OperandB
--  - FCmd <= FCmd_BNOT
--  - CinCmd <= CinCmd_ONE
--  - LoadA <= '0'
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_ADDER
-- DT - Result <= OperandA - 1
--  - FCmd <= FCmd_ONES
--  - CinCmd <= CinCmd_ZERO
--  - LoadA <= '1'
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_ADDER
-- MOV - Result <= OperandB
--  - FCmd <= FCmd_B
--  - CinCmd <= CinCmd_ZERO
--  - LoadA <= '1'
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_FBLOCK
-- AND/TST - Result <= OperandA & OperandB
--  - FCmd <= FCmd_AND
--  - CinCmd <= CinCmd_ZERO
--  - LoadA <= '1'
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_FBLOCK
-- OR - Result <= OperandA | OperandB
--  - FCmd <= FCmd_OR
--  - CinCmd <= CinCmd_ZERO
--  - LoadA <= '1'
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_FBLOCK
-- XOR - Result <= OperandA ^ OperandB
--  - FCmd <= FCmd_XOR
--  - CinCmd <= CinCmd_ZERO
--  - LoadA <= '1'
--  - SCmd <= "XX"
--  - ALUCmd <= ALUCmd_FBLOCK
-- NOT - Result <= ~OperandB
--  - FCmd   <= FCmd_BNOT
--  - CinCmd <= CinCmd_ZERO
--  - LoadA <= '1'
--  - SCmd   <= "XX"
--  - ALUCmd <= ALUCmd_FBLOCK
-- SHAL/SHLL - Result <= OperandA << 1
--  - FCmd   <= "XX"
--  - CinCmd <= CinCmd_ZERO
--  - LoadA <= '1'
--  - SCmd   <= SCmd_LSR
--  - ALUCmd <= ALUCmd_SHIFT
-- SHAR - Result <= OperandA >> 1 (sign-extended)
--  - FCmd   <= "XX"
--  - CinCmd <= CinCmd_ZERO
--  - LoadA <= '1'
--  - SCmd   <= SCmd_ASR
--  - ALUCmd <= ALUCmd_SHIFT
-- SHLR - Result <= OperandA >> 1
--  - FCmd   <= "XX"
--  - CinCmd <= CinCmd_ZERO
--  - LoadA <= '1'
--  - SCmd   <= SCmd_LSR
--  - ALUCmd <= ALUCmd_SHIFT
-- ROTL - Result <= rotate_left(OperandA)
--  - FCmd   <= "XX"
--  - CinCmd <= CinCmd_ZERO
--  - LoadA <= '1'
--  - SCmd   <= SCmd_ROL
--  - ALUCmd <= ALUCmd_SHIFT
-- ROTR - Result <= rotate_right(OperandA)
--  - FCmd   <= "XX"
--  - CinCmd <= CinCmd_ZERO
--  - LoadA <= '1'
--  - SCmd   <= SCmd_ROR
--  - ALUCmd <= ALUCmd_SHIFT
-- ROTCL - Result <= rotate_left(OperandA, T)
--  - FCmd   <= "XX"
--  - CinCmd <= CinCmd_CIN
--  - LoadA <= '1'
--  - SCmd   <= SCmd_RLC
--  - ALUCmd <= ALUCmd_SHIFT
-- ROTCR - Result <= rotate_right(OperandA, T)
--  - FCmd   <= "XX"
--  - CinCmd <= CinCmd_CIN
--  - LoadA <= '1'
--  - SCmd   <= SCmd_RRC
--  - ALUCmd <= ALUCmd_SHIFT

entity sh2alu is
    port (
        OperandA : in    std_logic_vector(31 downto 0); -- first operand
        OperandB : in    std_logic_vector(31 downto 0); -- second operand
        TIn      : in    std_logic;                     -- T bit from status register
        LoadA    : in    std_logic;                     -- determine if OperandA is loaded ('1') or zeroed ('0')
        FCmd     : in    std_logic_vector(3 downto 0);  -- F-Block operation
        CinCmd   : in    std_logic_vector(1 downto 0);  -- carry in operation
        SCmd     : in    std_logic_vector(2 downto 0);  -- shift operation
        ALUCmd   : in    std_logic_vector(1 downto 0);  -- ALU result select

        Result   : buffer  std_logic_vector(31 downto 0); -- ALU result
        Cout     : out   std_logic;                       -- carry out
        Overflow : out   std_logic;                       -- signed overflow
        Zero     : out   std_logic;                       -- result is zero
        Sign     : out   std_logic                        -- sign of result
    );
end entity sh2alu;

architecture structural of sh2alu is

    component ALU is

        generic (
            wordsize : integer := 8
        );
        port (
            AluOpA : in    std_logic_vector(wordsize - 1 downto 0);
            AluOpB : in    std_logic_vector(wordsize - 1 downto 0);
            Cin    : in    std_logic;
            FCmd   : in    std_logic_vector(3 downto 0);
            CinCmd : in    std_logic_vector(1 downto 0);
            SCmd   : in    std_logic_vector(2 downto 0);
            AluCmd : in    std_logic_vector(1 downto 0);

            Result   : buffer  std_logic_vector(wordsize - 1 downto 0);
            Cout     : out   std_logic;
            HalfCout : out   std_logic;
            Overflow : out   std_logic;
            Zero     : out   std_logic;
            Sign     : out   std_logic
        );
    end component ALU;

    signal BarrelShifter : std_logic_vector(31 downto 0);

    signal ALUResult : std_logic_vector(31 downto 0);

begin
    -- We use a generic ALU to implement all of the SH-2 ALU operations. We
    -- pass in the T bit in place of a dedicated carry input, and the CPU can
    -- route the correct output flag (carry, sign, zero, overflow) back into
    -- the status register.
    ALUinternal : component ALU
        generic map (
            wordsize => 32
        )
        port map (
            AluOpA   => OperandA and LoadA,
            AluOpB   => OperandB,
            Cin      => TIn,
            FCmd     => FCmd,
            SCmd     => SCmd,
            AluCmd   => AluCmd,
            CinCmd   => CinCmd,
            Result   => ALUResult,
            Cout     => Cout,
            Overflow => Overflow,
            Zero     => Zero,
            Sign     => Sign
        );

    -- We also add in a barrel shifter to implement extra-credit instructions
    -- For convenience, we will re-use the SCmd bits to control this barrel shifter.
    with SCmd select
        BarrelShifter <= OperandA(29 downto 0) & "00"                   when BSCmd_L2,
                         "00" & OperandA(31 downto 2)                   when BSCmd_R2,
                         OperandA(23 downto 0) & "00000000"             when BSCmd_L8,
                         "00000000" & OperandA(31 downto 8)             when BSCmd_R8,
                         OperandA(15 downto 0) & "0000000000000000"     when BSCmd_L16,
                         "0000000000000000" & OperandA(31 downto 16)    when BSCmd_R16,
                         (others => 'X') when others;

    -- Mux between the generic ALU and the barrel shifter to get the result
    Result <= BarrelShifter when ALUCmd = ALUCmd_BSHIFT else ALUResult;

end architecture structural;
