SOURCES = utils.vhd alu.vhd sh2_alu.vhd reg.vhd sh2_reg.vhd
TESTBENCHES = sh2_alu_tb.vhd sh2_reg_tb.vhd
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
