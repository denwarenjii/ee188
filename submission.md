---
documentclass: extarticle
fontsize: 8pt
geometry:
- top=.5in
- bottom=.5in
- left=.5in
- right=.5in
linestretch: 1
mainfont: "DejaVu Serif"
monofont: "DejaVu Sans Mono"
---

**Name**: Christian Miranda and Zachary Huang

**Date:** 06/09/2025


# EE188 Assignment 3 - Hitachi SH-2 CPU Pipelined Design

A 5 stage pipeline with instruction fetch, instruction decode, execution, 
memory access, writeback was implemented. Major changes were made to `sh2_cpu.vhd`
and `sh2_control.vhd`.

The design can be tested in `ghdl` by invoking `make test`. All tests continue
to pass except the branch tests.

# Timing and Resource Usage

The pipelined and non-pipelined designs were implemented using Xilinx Vivado 
for the Xilinx Spartan 7 XC7S25-1CSGA225C.

The resource and timing reports for both designs are provided in the files titled
`report_pipelined.pdf` and `reports_unpipelined.pdf`.

The clock period for the non-pipelined design is determined by a maximum path
delay of $34.814 \text{ ns}$. This corresponds to a maximum clock speed of $28.72 \text{ MHz}$.

The clock period for the pipelined design is determined by a maximum path
delay of $41.784 \text{ ns}$. This corresponds to a maximum clock speed of $23.93 \text{ MHz}$.

Thus, we see that our maximum clock speed decreased by about $5 \text{ MHz}$, but because we are executing approximately 1 instruction per clock in the pipelined design versus 1 instruction per
3 clocks in the non-pipelined design, we have a net increase in instruction throughput (a little
less than 3 times as many instructions per clock).

The non-pipelined design produced the following resource utilization report:

```
+-------------------------+------+-------+------------+-----------+-------+
|        Site Type        | Used | Fixed | Prohibited | Available | Util% |
+-------------------------+------+-------+------------+-----------+-------+
| Slice LUTs              | 2150 |     0 |          0 |     14600 | 14.73 |
|   LUT as Logic          | 2150 |     0 |          0 |     14600 | 14.73 |
|   LUT as Memory         |    0 |     0 |          0 |      5000 |  0.00 |
| Slice Registers         |  975 |     0 |          0 |     29200 |  3.34 |
|   Register as Flip Flop |  787 |     0 |          0 |     29200 |  2.70 |
|   Register as Latch     |  188 |     0 |          0 |     29200 |  0.64 |
| F7 Muxes                |  257 |     0 |          0 |      7300 |  3.52 |
| F8 Muxes                |  128 |     0 |          0 |      3650 |  3.51 |
+-------------------------+------+-------+------------+-----------+-------+
```


The pipelined design produced the following resource utilization report:

```
+-------------------------+------+-------+------------+-----------+-------+
|        Site Type        | Used | Fixed | Prohibited | Available | Util% |
+-------------------------+------+-------+------------+-----------+-------+
| Slice LUTs              | 2329 |     0 |          0 |     14600 | 15.95 |
|   LUT as Logic          | 2329 |     0 |          0 |     14600 | 15.95 |
|   LUT as Memory         |    0 |     0 |          0 |      5000 |  0.00 |
| Slice Registers         | 1232 |     0 |          0 |     29200 |  4.22 |
|   Register as Flip Flop | 1067 |     0 |          0 |     29200 |  3.65 |
|   Register as Latch     |  165 |     0 |          0 |     29200 |  0.57 |
| F7 Muxes                |  257 |     0 |          0 |      7300 |  3.52 |
| F8 Muxes                |  128 |     0 |          0 |      3650 |  3.51 |
+-------------------------+------+-------+------------+-----------+-------+
```

Therefore, the increase in resource utilization is marginal (~1% increase) 
compared to the speedup.