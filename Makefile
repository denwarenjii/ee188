SOURCES = utils.vhd mau.vhd sh2_constants.vhd sh2_dmau.vhd sh2_pmau.vhd alu.vhd sh2_alu.vhd reg.vhd sh2_reg.vhd memory.vhd memory_interface.vhd sh2_control.vhd sh2_cpu.vhd
TESTBENCHES = sh2_alu_tb.vhd sh2_reg_tb.vhd sh2_cpu_tb.vhd memory_tb.vhd
WAVEFORM = waveform.ghw

BUILDFLAGS = --std=08 -fsynopsys
# BUILDFLAGS =
RUNFLAGS = --ieee-asserts=disable --wave=$(WAVEFORM)

all: $(basename $(TESTBENCHES))

build: $(SOURCES) $(TESTBENCHES)
	ghdl -a $(BUILDFLAGS) $(SOURCES)
	ghdl -a $(BUILDFLAGS) $(TESTBENCHES)

$(basename $(TESTBENCHES)): build $(TESTBENCHES)
	ghdl -e $(BUILDFLAGS) $@

view:
	gtkwave $(WAVEFORM)

clean:
	rm -rf *.cf

.PHONY: test clean build view
