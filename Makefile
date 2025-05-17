SOURCES = AnsiEscape.vhd utils.vhd mau.vhd sh2_constants.vhd logging.vhd sh2_dmau.vhd sh2_pmau.vhd alu.vhd sh2_alu.vhd reg.vhd sh2_reg.vhd memory.vhd memory_interface.vhd sh2_control.vhd sh2_cpu.vhd
TESTBENCHES = sh2_alu_tb.vhd sh2_reg_tb.vhd sh2_cpu_tb.vhd memory_tb.vhd
WAVEFORM = waveform.ghw


# Wuseless:   		Emit a warning on useless code (like conditions that are always
# 								false or true, assertions that cannot be triggered).
# Werror: 				Any warning is an error.
# Wruntime-error: 
BUILDFLAGS = --std=08 -Wuseless -Werror -Wruntime-error -Wnowrite -fsynopsys --workdir=work/
RUNFLAGS = --ieee-asserts=disable --wave=$(WAVEFORM)

all: $(basename $(TESTBENCHES))

build: $(SOURCES) $(TESTBENCHES)
	ghdl -a $(BUILDFLAGS) $(SOURCES)
	ghdl -a $(BUILDFLAGS) $(TESTBENCHES)

$(basename $(TESTBENCHES)): build $(TESTBENCHES)
	ghdl -e $(BUILDFLAGS) $@

run: $(basename $(TESTBENCHES))
	ghdl -r sh2_cpu_tb $(RUNFLAGS)

asm:
	cd asm && $(MAKE)

test: $(basename $(TESTBENCHES)) asm
	ghdl -r sh2_cpu_tb $(RUNFLAGS)

view:
	gtkwave $(WAVEFORM)

clean:
	ghdl --clean --workdir=work --std=08
	rm -rf *.cf *.o work/*

.PHONY: test clean build view asm run
