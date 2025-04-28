----------------------------------------------------------------------------
--
--  TODO
-- 
--  Revision History:
--     26 Apr 25    Zack Huang      initial revision
--
----------------------------------------------------------------------------

-- import libraries
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.textio.all;

use work.SH2ALUConstants.all;

entity sh2alu_tb is
end sh2alu_tb;

architecture behavioral of sh2alu_tb is
    -- Stimulus signals for unit under test
    signal OperandA : std_logic_vector(31 downto 0);   -- first operand
    signal OperandB : std_logic_vector(31 downto 0);   -- second operand
    signal TIn      : std_logic;                       -- T bit from status register
    signal LoadA    : std_logic;                       -- determine if OperandA is loaded ('1') or zeroed ('0')
    signal FCmd     : std_logic_vector(3 downto 0);    -- F-Block operation
    signal CinCmd   : std_logic_vector(1 downto 0);    -- carry in operation
    signal SCmd     : std_logic_vector(2 downto 0);    -- shift operation
    signal ALUCmd   : std_logic_vector(1 downto 0);    -- ALU result select

    -- Outputs from unit under test
    signal Result   : std_logic_vector(31 downto 0);   -- ALU result
    signal Cout     : std_logic;                       -- carry out
    signal Overflow : std_logic;                       -- signed overflow
    signal Zero     : std_logic;                       -- result is zero
    signal Sign     : std_logic;                       -- sign of result

    -- Test signals
    signal END_SIM : boolean    := false;   -- if the simulation should end

begin
    -- Instantiate UUT
    UUT: entity work.sh2alu
    port map(
        OperandA => OperandA,
        OperandB => OperandB,
        TIn      => TIn,
        LoadA    => LoadA,
        FCmd     => FCmd,
        CinCmd   => CinCmd,
        SCmd     => SCmd,
        ALUCmd   => ALUCmd,
        Result   => Result,
        Cout     => Cout,
        Overflow => Overflow,
        Zero     => Zero,
        Sign     => Sign
    );

    process
        procedure CheckResult(op : string; ExpectedResult : std_logic_vector) is
        begin
            assert Result = ExpectedResult
            report op & ": Expected " & to_hstring(ExpectedResult) & ", found " & to_hstring(Result)
            severity error;
        end procedure;

        procedure CheckFlags(
            op : string;
            ExpectedCarry    : std_logic := 'X';
            ExpectedOverflow : std_logic := 'X';
            ExpectedZero     : std_logic := 'X'
        ) is
        begin
            assert (ExpectedCarry = 'X' or Cout = ExpectedCarry)
            report op & ": Expected C = " & to_string(ExpectedCarry) & ", found " & to_string(Cout)
            severity error;
            assert (ExpectedOverflow = 'X' or Overflow = ExpectedOverflow)
            report op & ": Expected V = " & to_string(ExpectedOverflow) & ", found " & to_string(Overflow)
            severity error;
            assert (ExpectedZero = 'X' or Zero = ExpectedZero)
            report op & ": Expected Z = " & to_string(ExpectedZero) & ", found " & to_string(Zero)
            severity error;
        end procedure;

        procedure PerformOp(
            OpA : std_logic_vector(31 downto 0);
            OpB : std_logic_vector(31 downto 0);
            T   : std_logic;
            F   : std_logic_vector(3 downto 0);
            Cin : std_logic_vector(1 downto 0);
            LA  : std_logic;
            S   : std_logic_vector(2 downto 0);
            Cmd : std_logic_vector(1 downto 0)
        ) is
        begin
            OperandA <= OpA;
            OperandB <= OpB;
            TIn    <= T;
            FCmd   <= F;
            CinCmd <= Cin;
            LoadA  <= LA;
            SCmd   <= S;
            ALUCmd <= Cmd;
            wait for 10 ns;
        end procedure;

        -- Used to jointly test ADD, ADDC, and ADDV
        procedure TestADD(A : integer; B : integer; T : integer) is
            variable ExpectedCarry : std_logic;
            variable ExpectedOverflow : std_logic;
            variable TBit : std_logic;

            variable Left : std_logic_vector(31 downto 0);
            variable Right : std_logic_vector(31 downto 0);
            variable Sum : unsigned(32 downto 0);
            variable LowerSum : unsigned(31 downto 0);
        begin
            Left     := std_logic_vector(to_signed(A, 32));
            Right    := std_logic_vector(to_signed(B, 32));
            TBit     := '0' when T = 0 else '1';

            PerformOp(Left, Right, TBit, FCmd_B, CinCmd_CIN, '1', "XXX", ALUCmd_ADDER);

            Sum      := ('0' & unsigned(Left(31 downto 0))) + ('0' & unsigned(Right(31 downto 0))) + TBit;
            LowerSum := ('0' & unsigned(Left(30 downto 0))) + ('0' & unsigned(Right(30 downto 0))) + TBit;

            ExpectedCarry := Sum(32);
            ExpectedOverflow := Sum(32) xor LowerSum(31);

            CheckResult("ADD", std_logic_vector(sum(31 downto 0)));
            CheckFlags ("ADD", ExpectedCarry, ExpectedOverflow);
        end procedure;

        -- Used to jointly test SUB, SUBC, and SUBV
        procedure TestSUB(A : integer; B : integer; T : integer) is
            variable ExpectedCarry : std_logic;
            variable ExpectedOverflow : std_logic;
            variable TBit : std_logic;

            variable Left : std_logic_vector(31 downto 0);
            variable Right : std_logic_vector(31 downto 0);
            variable Sum : unsigned(32 downto 0);
            variable LowerSum : unsigned(31 downto 0);
        begin
            Left     := std_logic_vector(to_signed(A, 32));
            Right    := std_logic_vector(to_signed(B, 32));
            TBit     := '0' when T = 0 else '1';

            PerformOp(Left, Right, TBit, FCmd_BNOT, CinCmd_CINBAR, '1', "XXX", ALUCmd_ADDER);

            Sum      := ('0' & unsigned(Left(31 downto 0))) + ('0' & unsigned(not Right(31 downto 0))) + (not TBit);
            LowerSum := ('0' & unsigned(Left(30 downto 0))) + ('0' & unsigned(not Right(30 downto 0))) + (not TBit);

            ExpectedCarry := Sum(32);
            ExpectedOverflow := Sum(32) xor LowerSum(31);

            CheckResult("SUB", std_logic_vector(sum(31 downto 0)));
            CheckFlags ("SUB", ExpectedCarry, ExpectedOverflow);
        end procedure;

        -- Used to jointly test NEG and NEGC (A ignored)
        procedure TestNEG(A : integer; B : integer; T : integer) is
            variable ExpectedCarry : std_logic;
            variable TBit : std_logic;

            variable Left : std_logic_vector(31 downto 0);
            variable Right : std_logic_vector(31 downto 0);
            variable Sum : unsigned(32 downto 0);
            variable LowerSum : unsigned(31 downto 0);
        begin
            Left     := std_logic_vector(to_signed(A, 32));
            Right    := std_logic_vector(to_signed(B, 32));
            TBit     := '0' when T = 0 else '1';

            PerformOp(Left, Right, TBit, FCmd_BNOT, CinCmd_CINBAR, '0', "XXX", ALUCmd_ADDER);

            Sum      := ('0' & unsigned(not Right(31 downto 0))) + (not TBit);
            LowerSum := ('0' & unsigned(not Right(30 downto 0))) + (not TBit);

            ExpectedCarry := Sum(32);

            CheckResult("NEG", std_logic_vector(sum(31 downto 0)));
            CheckFlags ("NEG", ExpectedCarry);
        end procedure;

        -- Used to test DT (B ignored)
        procedure TestDT(A : integer; B : integer; T : integer) is
            variable ExpectedZero : std_logic;
            variable TBit : std_logic;

            variable Left : std_logic_vector(31 downto 0);
            variable Right : std_logic_vector(31 downto 0);
            variable Sum : unsigned(31 downto 0);
        begin
            Left     := std_logic_vector(to_signed(A, 32));
            Right    := std_logic_vector(to_signed(B, 32));
            TBit     := '0' when T = 0 else '1';

            PerformOp(Left, Right, TBit, FCmd_ONES, CinCmd_ZERO, '1', "XXX", ALUCmd_ADDER);

            Sum      := unsigned(Left(31 downto 0)) - to_unsigned(1, 32);

            ExpectedZero := '1' when Sum = (Sum'range => '0') else '0';

            CheckResult("DT", std_logic_vector(Sum));
            CheckFlags ("DT", 'X', 'X', ExpectedZero);
        end procedure;

        -- jointly test all f-block based logic instructions (AND, OR, NOT, XOR)
        procedure TestLogic(A : integer; B : integer; F : std_logic_vector) is
            variable ExpectedZero : std_logic;

            variable Left : std_logic_vector(31 downto 0);
            variable Right : std_logic_vector(31 downto 0);
            variable Sum : std_logic_vector(31 downto 0);
        begin
            Left     := std_logic_vector(to_signed(A, 32));
            Right    := std_logic_vector(to_signed(B, 32));

            PerformOp(Left, Right, '0', F, CinCmd_ZERO, '1', "XXX", ALUCmd_FBLOCK);

            for i in 0 to 31 loop
                Sum(i) := (Left(i) and Right(i) and F(3)) or
                          (Left(i) and not Right(i) and F(2)) or
                          (not Left(i) and Right(i) and F(1)) or
                          (not Left(i) and not Right(i) and F(0));
            end loop;

            ExpectedZero := '1' when Sum = (Sum'range => '0') else '0';

            CheckResult("LOGIC:" & to_string(F), Sum);
            CheckFlags ("LOGIC:" & to_string(F), 'X', 'X', ExpectedZero);
        end procedure;

        -- jointly test all shifter based instructions (SHLL, SHAR, SHLR, ROTL, ROTR, ROTCL, ROTCR)
        -- note that B is ignored
        procedure TestShift(A : integer; B : integer; T : integer; S : std_logic_vector) is
            variable ExpectedCarry : std_logic;

            variable Left : std_logic_vector(31 downto 0);
            variable Right : std_logic_vector(31 downto 0);
            variable Sum : std_logic_vector(31 downto 0);

            variable TBit : std_logic;
            variable C    : std_logic_vector(1 downto 0);
        begin
            Left     := std_logic_vector(to_signed(A, 32));
            Right    := std_logic_vector(to_signed(B, 32));
            TBit     := '0' when T = 0 else '1';

            C        := CinCmd_CIN when S = SCmd_RLC or S = SCmd_RRC else
                        CinCmd_ZERO;

            PerformOp(Left, Right, TBit, "XXXX", C, '1', S, ALUCmd_SHIFT);

            case S is
                when SCmd_LSL =>
                    Sum := Left(30 downto 0) & '0';
                    ExpectedCarry := Left(31);
                when SCmd_ROL =>
                    Sum := Left(30 downto 0) & Left(31);
                    ExpectedCarry := Left(31);
                when SCmd_RLC =>
                    Sum := Left(30 downto 0) & TBit;
                    ExpectedCarry := Left(31);
                when SCmd_LSR =>
                    Sum := '0' & Left(31 downto 1);
                    ExpectedCarry := Left(0);
                when SCmd_ASR =>
                    Sum := Left(31) & Left(31 downto 1);
                    ExpectedCarry := Left(0);
                when SCmd_ROR =>
                    Sum := Left(0) & Left(31 downto 1);
                    ExpectedCarry := Left(0);
                when SCmd_RRC =>
                    Sum := TBit & Left(31 downto 1);
                    ExpectedCarry := Left(0);
                when SCmd_SWAP =>
                    Sum := Left(15 downto 0) & Left(31 downto 16);
                    ExpectedCarry := 'X';
                when others =>
                    report "Invalid Shift command";
            end case;

            CheckResult("SHIFT:" & to_string(S), Sum);
            CheckFlags ("SHIFT:" & to_string(S), ExpectedCarry);
        end procedure;

    begin
        -- TODO: test with bin coverage
        for j in -20 to 20 loop
            for k in -20 to 20 loop
                for t in 0 to 1 loop
                    TestADD(j, k, t);
                    TestSUB(j, k, t);
                    TestNEG(j, k, t);
                    TestDT(j, k, t);
                    TestLogic(j, k, FCmd_AND);
                    TestLogic(j, k, FCmd_OR);
                    TestLogic(j, k, FCmd_XOR);
                    TestLogic(j, k, FCmd_BNOT);
                    TestShift(j, k, t, SCmd_LSL);
                    TestShift(j, k, t, SCmd_LSR);
                    TestShift(j, k, t, SCmd_ASR);
                    TestShift(j, k, t, SCmd_ROL);
                    TestShift(j, k, t, SCmd_ROR);
                    TestShift(j, k, t, SCmd_RLC);
                    TestShift(j, k, t, SCmd_RRC);
                end loop;
            end loop;
        end loop;
        -- End simulation
        END_SIM <= TRUE;
        wait;
    end process;
end behavioral;
