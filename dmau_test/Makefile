SOURCES = utils.vhd sh2_constants.vhd mau.vhd sh2_dmau.vhd sh2_dmau_tb.vhd
TESTBENCHES= sh2_dmau_tb.vhd
WAVEFORM = sh2_dmau_tb.vcd

BUILDFLAGS = --std=08 -Wall -fsynopsys -frelaxed --workdir=work -P/home/chris/vhdl
RUNFLAGS = --vcd=work/$(WAVEFORM)

all: $(basename $(TESTBENCHES))

build: $(SOURCES) $(TESTBENCHES)
	ghdl -a $(BUILDFLAGS) $(SOURCES)
	ghdl -a $(BUILDFLAGS) $(TESTBENCHES)

$(basename $(TESTBENCHES)): build $(TESTBENCHES)
	ghdl -e $(BUILDFLAGS) $@

run: $(basename $(TESTBENCHES))
	ghdl -r $(basename $(TESTBENCHES)) $(RUNFLAGS)


view:
	gtkwave $(WAVEFORM)

clean:
	rm -rf *.cf

.PHONY: test clean build view

