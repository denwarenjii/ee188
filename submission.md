---
documentclass: extarticle
fontsize: 8pt
geometry:
- top=.5in
- bottom=.5in
- left=.3in
- right=.3in
linestretch: 1
mainfont: "DejaVu Serif"
monofont: "DejaVu Sans Mono"
---

# Hitachi SH-2 CPU Design

The Hitachi SH-2 is implemented in the `sh2_cpu.vhd` file, and tested in
`sh2_cpu_tb.vhd`.

## Control Unit and Instruction Register

The control unit and instruction register are implemented in `sh2_control.vhd`.
They are tested in `sh2_cpu_tb.vhd`.

## General-Purpose Registers

The register array is implemented in `sh2_reg.vhd` and tested in 
`sh2_reg_tb.vhd`.

## ALU

The ALU is implemented in `sh2_alu.vhd` and tested in `sh2_alu_tb.vhd`.

## Status Register

The status register is implemented in `sh2_cpu.vhd`, and tested along with the
entire CPU in `sh2_cpu_tb.vhd`.

## Data Memory Access Unit

The data memory access unit (DMAU) is implemented in `sh2_dmau.vhd`. Its
test bench is located in the `dmau_test/` directory. It can be tested by
invoking `make run` from within the `dmau_test/` directory.

## Program Memory Access Unit

The data memory access unit (PMAU) is implemented in `sh2_pmau.vhd`. Its
test bench is located in the `pmau_test/` directory. It can be tested by
invoking `make run` from within the `pmau_test/` directory.

## Design Testing

Each instruction is tested by assembling an assembly file that uses the
instruction under test and writes expected results to memory. The actual
memory contents are compared with the expected memory contents to check for
correctness. 

The assembly files are located in the `asm` and are postfixed
with `.asm`. Each assembly file has an associated `.expect` file which 
describes the expect memory contents, as well as a `.dump` file that shows
the memory contents after the tests are run.

The SH-2 CPU can be tested by invoking `make test` from the top level 
directory. This compiles any `.vhd` files that have changed and then enters
the `asm/` directory and compiles and `.asm` files that have changed. 
Assembling the assembly files generates binary `.bin` files which are read
by the test bench and then decoded and run by the CPU.

Testing requires `ghdl` with either the GCC or LLVM backend. The `AS` 
macro-assembler, linked below, is required for assembling the assembly files,
although it is possible to test the SH-2 CPU without this by invoking 
`make test-bin`, which uses the supplied binaries for each assembly file.


http://john.ccac.rwth-aachen.de:8000/as/download.html



## Extra Credit - Multi-bit shift Instructions (20 points)

## Extra Credit - Implementation (60 points)