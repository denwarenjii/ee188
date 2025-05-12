----------------------------------------------------------------------------
--
--  Generic ALU and Status Register
--
--  This is a generic implementation of the ALU for simple microprocessors.
--  It does not include a multiplier, MAC, divider, or barrel shifter.
--
--  Packages included are:
--     ALUConstants - constants for all entities making up the ALU
--
--  Entities included are:
--     FBlockBit - one bit of an F-Block
--     AdderBit  - one bit of an adder (a full adder)
--     FBlock    - F-Block (logical operations)
--     Adder     - adder
--     Shifter   - shift and rotate and swap operations
--     ALU       - the actual ALU
--
--  Revision History:
--     25 Jan 21  Glen George       Initial revision.
--     27 Jan 21  Glen George       Changed left/right shift selection to a
--                                  constant.
--     27 Jan 21  Glen George       Changed F-Block to be on B input of adder.
--     27 Jan 21  Glen George       Updated comments.
--     29 Jan 21  Glen George       Fixed a number of wordsize bugs.
--     29 Jan 21  Glen George       Fixed overflow signal in adder.
--     11 Apr 25  Glen George       Removed Status Register.
--     11 May 25  Zack Huang        Removed Swap
--
----------------------------------------------------------------------------


--
--  Package containing the constants used by the ALU and all of its
--  sub-modules.
--

library ieee;
use ieee.std_logic_1164.all;

package  ALUConstants  is

--  Adder carry in select constants
--     may be freely changed

   constant CinCmd_ZERO   : std_logic_vector(1 downto 0) := "00";
   constant CinCmd_ONE    : std_logic_vector(1 downto 0) := "01";
   constant CinCmd_CIN    : std_logic_vector(1 downto 0) := "10";
   constant CinCmd_CINBAR : std_logic_vector(1 downto 0) := "11";


--  Shifter command constants
--     may be freely changed except a single bit pattern (currently high bit)
--     must distinguish between left shifts and rotates and right shifts and
--     rotates

   constant SCmd_LEFT  : std_logic_vector(2 downto 0) := "0--";
   constant SCmd_LSL   : std_logic_vector(2 downto 0) := "000";
   constant SCmd_ASL   : std_logic_vector(2 downto 0) := "001";
   constant SCmd_ROL   : std_logic_vector(2 downto 0) := "010";
   constant SCmd_RLC   : std_logic_vector(2 downto 0) := "011";
   constant SCmd_RIGHT : std_logic_vector(2 downto 0) := "1--";
   constant SCmd_LSR   : std_logic_vector(2 downto 0) := "100";
   constant SCmd_ASR   : std_logic_vector(2 downto 0) := "101";
   constant SCmd_ROR   : std_logic_vector(2 downto 0) := "110";
   constant SCmd_RRC   : std_logic_vector(2 downto 0) := "111";


--  ALU command constants
--     may be freely changed

   constant ALUCmd_FBLOCK  : std_logic_vector(1 downto 0) := "00";
   constant ALUCmd_ADDER   : std_logic_vector(1 downto 0) := "01";
   constant ALUCmd_SHIFT   : std_logic_vector(1 downto 0) := "10";


end package;



--
--  FBlockBit
--
--  This is a bit of the F-Block for doing logical operations in the ALU.  The
--  operations available are:
--     FCmd    Operation
--     0000    0
--     0001    A nor B
--     0010    not A and B
--     0011    not A
--     0100    A and not B
--     0101    not B
--     0110    A xor B
--     0111    A nand B
--     1000    A and B
--     1001    A xnor B
--     1010    B
--     1011    not A or B
--     1100    A
--     1101    A or not B
--     1110    A or B
--     1111    1
--
--  Inputs:
--    A    - first operand bit (bus A)
--    B    - second operand bit (bus B)
--    FCmd - operation to perform (4 bits)
--
--  Outputs:
--    F    - F-Block output (based on input busses and command)
--

library ieee;
use ieee.std_logic_1164.all;

entity  FBlockBit  is

    port(
        A    : in   std_logic;                      -- first operand
        B    : in   std_logic;                      -- second operand
        FCmd : in   std_logic_vector(3 downto 0);   -- operation to perform
        F    : out  std_logic                       -- result
    );

end  FBlockBit;


architecture  dataflow  of  FBlockBit  is
begin

    F  <=  FCmd(3)  when  ((A = '1') and (B = '1'))  else
           FCmd(2)  when  ((A = '1') and (B = '0'))  else
           FCmd(1)  when  ((A = '0') and (B = '1'))  else
           FCmd(0)  when  ((A = '0') and (B = '0'))  else
           'X';

end  dataflow;



--
--  AdderBit
--
--  This is a bit of the adder for doing addition in the ALU.
--
--  Inputs:
--    A  - first operand bit (bus A)
--    B  - second operand bit (bus B)
--    Ci - carry in (from previous bit)
--
--  Outputs:
--    S  - sum for this bit
--    Co - carry out for this bit
--

library ieee;
use ieee.std_logic_1164.all;

entity  AdderBit  is

    port(
        A  : in   std_logic;        -- first operand
        B  : in   std_logic;        -- second operand
        Ci : in   std_logic;        -- carry in from previous bit
        S  : out  std_logic;        -- sum (result)
        Co : out  std_logic         -- carry out to next bit
    );

end  AdderBit;


architecture  dataflow  of  AdderBit is
begin

    S  <=  A  xor  B  xor  Ci;
    Co <=  (A  and  B)  or  (A  and Ci)  or  (B  and  Ci);

end  dataflow;



--
--  FBlock
--
--  This is the F-Block for doing logical operations in the ALU.  The F-Block
--  operations are:
--     FCmd    Operation
--     0000    0
--     0001    FBOpA nor FBOpB
--     0010    not FBOpA and FBOpB
--     0011    not FBOpA
--     0100    FBOpA and not FBOpB
--     0101    not FBOpB
--     0110    FBOpA xor FBOpB
--     0111    FBOpA nand FBOpB
--     1000    FBOpA and FBOpB
--     1001    FBOpA xnor FBOpB
--     1010    FBOpB
--     1011    not FBOpA or FBOpB
--     1100    FBOpA
--     1101    FBOpA or not FBOpB
--     1110    FBOpA or FBOpB
--     1111    1
--
--  Generics:
--    wordsize - width of the F-Block in bits (default 8)
--
--  Inputs:
--    FBOpA   - first operand
--    FBOpB   - second operand
--    FCmd    - operation to perform (4 bits)
--
--  Outputs:
--    FResult - F-Block result (based on input busses and command)
--

library ieee;
use ieee.std_logic_1164.all;

entity  FBlock  is

    generic (
        wordsize : integer := 8      -- default width is 8-bits
    );

    port(
        FBOpA   : in   std_logic_vector(wordsize - 1 downto 0); -- first operand
        FBOpB   : in   std_logic_vector(wordsize - 1 downto 0); -- second operand
        FCmd    : in   std_logic_vector(3 downto 0);            -- operation to perform
        FResult : out  std_logic_vector(wordsize - 1 downto 0)  -- result
    );

end  FBlock;



architecture  structural  of  FBlock  is

    component  FBlockBit
        port(
            A    : in   std_logic;                      -- first operand
            B    : in   std_logic;                      -- second operand
            FCmd : in   std_logic_vector(3 downto 0);   -- operation to perform
            F    : out  std_logic                       -- result
        );
    end component;

begin

    F1:  for  i  in  FResult'Range  generate        -- make enough FBlockBits
    begin
        FBx: FBlockBit  port map  (FBOpA(i), FBOpB(i), FCmd, FResult(i));
    end generate;

end  structural;



--
--  Adder
--
--  This is the adder for doing addition in the ALU.
--
--  Generics:
--    wordsize - width of the adder in bits (default 8)
--
--  Inputs:
--    AddOpA - first operand
--    AddOpB - second operand
--    Cin    - carry in (from status register)
--    CinCmd - operation for carry in (2 bits)
--
--  Outputs:
--    AddResult - sum
--    Cout      - carry out for the addition
--    HalfCOut  - half carry out for the addition
--    Overflow  - signed overflow
--

library ieee;
use ieee.std_logic_1164.all;
use work.ALUConstants.all;

entity  Adder  is

    generic (
        wordsize : integer := 8      -- default width is 8-bits
    );

    port(
        AddOpA    : in   std_logic_vector(wordsize - 1 downto 0);   -- first operand
        AddOpB    : in   std_logic_vector(wordsize - 1 downto 0);   -- second operand
        Cin       : in   std_logic;                                 -- carry in
        CinCmd    : in   std_logic_vector(1 downto 0);              -- carry in operation
        AddResult : out  std_logic_vector(wordsize - 1 downto 0);   -- sum (result)
        Cout      : out  std_logic;                                 -- carry out
        HalfCout  : out  std_logic;                                 -- half carry out
        Overflow  : out  std_logic                                  -- signed overflow
    );

end  Adder;


architecture  structural  of  Adder  is

    component  AdderBit
        port(
            A  : in   std_logic;        -- first operand
            B  : in   std_logic;        -- second operand
            Ci : in   std_logic;        -- carry in from previous bit
            S  : out  std_logic;        -- sum (result)
            Co : out  std_logic         -- carry out to next bit
        );
    end component;

    signal  carry : std_logic_vector(wordsize downto 0);        -- intermediate carry results

begin

    -- get the carry in based on CinCmd
    carry(0)  <=  '0'      when  CinCmd = CinCmd_ZERO  else
                  '1'      when  CinCmd = CinCmd_ONE  else
                  Cin      when  CinCmd = CinCmd_CIN  else
                  not Cin  when  CinCmd = CinCmd_CINBAR  else
                  'X';

    A1:  for  i  in  AddResult'Range  generate      -- make enough AdderBits
    begin
        ABx: AdderBit  port map  (AddOpA(i), AddOpB(i), carry(i),
                                  AddResult(i), carry(i + 1));
    end generate;

    Cout     <= carry(wordsize);        -- compute carry out
    HalfCout <= carry(4);               -- half carry (carry into high nibble)
    -- overflow if carry into sign bit doesn't match the carry out
    Overflow <= carry(wordsize - 1) xor carry(wordsize);

end  structural;



--
--  Shifter
--
--  This is the shifter for doing shift/rotate operations in the ALU.  The
--  shift operations are defined by the constants SCmd_LEFT, SCmd_RIGHT,
--  SCmd_LSL, SCmd_ROL, SCmd_RLC, SCmd_LSR, SCmd_ASR, SCmd_ROR, and SCmd_RRC.
--
--  Generics:
--    wordsize - width of the shifter in bits (default 8)
--               must be an even number of bits
--
--  Inputs:
--    SOp     - operand
--    Cin     - carry in (from status register)
--    SCmd    - operation to perform (3 bits)
--
--  Outputs:
--    SResult - shift result
--    Cout    - carry out from the shift (link)
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.std_match;
use work.ALUConstants.all;

entity  Shifter  is

    generic (
        wordsize : integer := 8      -- default width is 8-bits
    );

    port(
        SOp     : in   std_logic_vector(wordsize - 1 downto 0); -- operand
        Cin     : in   std_logic;                               -- carry in
        SCmd    : in   std_logic_vector(2 downto 0);            -- shift operation
        SResult : out  std_logic_vector(wordsize - 1 downto 0); -- sum (result)
        Cout    : out  std_logic                                -- carry out
    );

end  Shifter;


architecture  dataflow  of  Shifter  is
begin

    -- middle bits get either bit to the left or bit to the right
    SResult(wordsize - 2 downto 1)  <= 

        -- right shift
        SOp(wordsize - 1 downto 2)                                                when  std_match(SCmd, SCmd_RIGHT)  else

        -- left shift
        SOp(wordsize - 3 downto 0)                                                when  std_match(SCmd, SCmd_LEFT)  else

        -- unknown command
        (others => 'X');


    -- high bit gets low bit, high bit, bit to the right, 0, or Cin depending
    --   on shift mode
    SResult(wordsize - 1)  <=
        SOp(wordsize - 2)    when  std_match(SCmd, SCmd_LEFT)  else  -- shift/rotate left
        SOp(0)               when  SCmd = SCmd_ROR   else  -- rotate right
        SOp(wordsize - 1)    when  SCmd = SCmd_ASR   else  -- arithmetic shift right
        '0'                  when  SCmd = SCmd_LSR   else  -- logical shift right
        Cin                  when  SCmd = SCmd_RRC   else  -- rotate right w/carry
        'X';                                               -- anything else is illegal


    -- low bit gets high bit, bit to the left, 0, or Cin depending on mode
    SResult(0)  <=
        SOp(1)            when  std_match(SCmd, SCmd_RIGHT)  else  -- shift/rotate right
        '0'               when  SCmd = SCmd_LSL or SCmd = SCmd_ASL   else  -- shift left
        SOp(wordsize - 1) when  SCmd = SCmd_ROL   else  -- rotate left
        Cin               when  SCmd = SCmd_RLC   else  -- rotate left w/carry
        'X';                                            -- anything else is illegal


    -- compute the carry out, it is low bit when shifting right and high bit
    --    when shifting left
    Cout  <=  SOp(0)             when  std_match(SCmd, SCmd_RIGHT)  else
              SOp(wordsize - 1)  when  std_match(SCmd, SCmd_LEFT)   else
              'X';

end  dataflow;



--
--  ALU
--
--  This is the actual ALU model for the CPU.  It includes the FBlock,
--  Adder, and Shifter modules.  It also outputs a number of status bits.
--
--  Generics:
--    wordsize - width of the ALU in bits (default 8)
--
--  Inputs:
--    ALUOpA   - first operand
--    ALUOpB   - second operand
--    Cin      - carry in (from status register)
--    FCmd     - F-Block operation to perform (4 bits)
--    CinCmd   - adder carry in operation for carry in (2 bits)
--    SCmd     - shift operation to perform (3 bits)
--    ALUCmd   - ALU operation to perform - selects result (2 bits)
--
--  Outputs:
--    Result   - ALU result
--    Cout     - carry out from the operation
--    HalfCOut - half carry out for addition
--    Overflow - signed overflow for addition
--    Zero     - zero result
--    Sign     - result sign (1 negative, 0 positive)
--

library ieee;
use ieee.std_logic_1164.all;
use work.ALUConstants.all;

entity  ALU  is

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

end  ALU;


architecture  structural  of  ALU  is

    component  FBlock
        generic(
            wordsize : integer
        );
        port(
            FBOpA   : in   std_logic_vector(wordsize - 1 downto 0);
            FBOpB   : in   std_logic_vector(wordsize - 1 downto 0);
            FCmd    : in   std_logic_vector(3 downto 0);
            FResult : out  std_logic_vector(wordsize - 1 downto 0)
        );
    end component;

    component  Adder
        generic(
            wordsize : integer
        );
        port(
            AddOpA    : in   std_logic_vector(wordsize - 1 downto 0);
            AddOpB    : in   std_logic_vector(wordsize - 1 downto 0);
            Cin       : in   std_logic;
            CinCmd    : in   std_logic_vector(1 downto 0);
            AddResult : out  std_logic_vector(wordsize - 1 downto 0);
            Cout      : out  std_logic;
            HalfCout  : out  std_logic;
            Overflow  : out  std_logic
        );
    end component;

    component  Shifter
        generic(
            wordsize : integer
        );
        port(
            SOp     : in   std_logic_vector(wordsize - 1 downto 0);
            Cin     : in   std_logic;
            SCmd    : in   std_logic_vector(2 downto 0);
            SResult : out  std_logic_vector(wordsize - 1 downto 0);
            Cout    : out  std_logic
        );
    end component;

    signal  FBRes   : std_logic_vector(wordsize - 1 downto 0);  -- F-Block result
    signal  AddRes  : std_logic_vector(wordsize - 1 downto 0);  -- adder result
    signal  ShRes   : std_logic_vector(wordsize - 1 downto 0);  -- shifter result

    signal  AddCout : std_logic;                                -- adder carry out
    signal  ShCout  : std_logic;                                -- shifter carry out

begin

    -- wire up the blocks
    FB1:   FBlock   generic map (wordsize)
                    port map  (ALUOpA, ALUOpB, FCmd, FBRes);
    Add1:  Adder    generic map (wordsize)
                    port map  (ALUOpA, FBRes, Cin, CinCmd,
                               AddRes, AddCout, HalfCout, Overflow);
    Sh1:   Shifter  generic map (wordsize)
                    port map  (ALUOpA, Cin, SCmd, ShRes, ShCout);

    -- figure out the result
    Result  <=  FBRes   when  ALUCmd = ALUCmd_FBLOCK  else  -- want F-Block
                AddRes  when  ALUCmd = ALUCmd_ADDER   else  -- want adder
                ShRes   when  ALUCmd = ALUCmd_SHIFT   else  -- want shifter
                (others => 'X');                            -- unknown command

    -- figure out the carry out
    COut  <=  '0'      when  ALUCmd = ALUCmd_FBLOCK  else   -- want F-Block
              AddCout  when  ALUCmd = ALUCmd_ADDER   else   -- want adder
              ShCout   when  ALUCmd = ALUCmd_SHIFT   else   -- want shifter
              'X';                                          -- unknown command

    -- zero flag is set when the result is 0
    Zero  <=  '1'  when  Result = (Result'range => '0')  else
              '0';

    -- compute the sign flag value
    Sign  <=  Result(wordsize - 1);

end  structural;
