# Elaboration order:
#
# utils.vhd
# logging.vhd
# AnsiEscape.vhd 
# sh2_constants.vhd
# mau.vhd
# sh2_pmau.vhd
# memory_interface.vhd
# sh2_dmau.vhd
# sh2_alu.vhd
# sh2_control.vhd
# sh2_reg.vhd
# sh2_cpu.vhd
# memory.vhd
# sh2_cpu_tb.vhd
# reg.vhd
# alu.vhd

# Dependenices:

# sh2_pmau.o:  sh2_constants.o  mau.o
# memory_interface.o: utils.o logging.o
# sh2_dmau.o: mau.o sh2_constants.o
# sh2_control.o: memory_interface.o logging.o sh2_pmau.o sh2_dmau.o sh2_alu.o utils.o
# sh2_cpu.o: sh2_pmau.o memory_interface.o sh2_control.o logging.o sh2_constants.o sh2_dmau.o sh2_reg.o sh2_alu.o
# memory.o: logging.o utils.o
# sh2_cpu_tb.o: utils.o logging.o AnsiEscape.o sh2_cpu.o memory.o
# reg.o: logging.o

GHDL = ghdl

# Directory for object files.
WORKDIR = work/

WAVEFORM = sh2_cpu_tb.ghw

# BUILDFLAGS = --std=08 -Wuseless -Werror -Wruntime-error -Wnowrite -fsynopsys --workdir=$(WORKDIR)
BUILDFLAGS = --std=08 -O2 -Wuseless -Werror -Wruntime-error -Wnowrite -fsynopsys --workdir=$(WORKDIR)

RUNFLAGS = --ieee-asserts=disable --wave=$(WAVEFORM)

SOURCES = utils.vhd logging.vhd AnsiEscape.vhd sh2_constants.vhd mau.vhd sh2_pmau.vhd memory_interface.vhd sh2_dmau.vhd sh2_alu.vhd sh2_control.vhd sh2_reg.vhd sh2_cpu.vhd memory.vhd sh2_cpu_tb.vhd reg.vhd alu.vhd 

# SOURCES = utils.vhd logging.vhd AnsiEscape.vhd sh2_constants.vhd mau.vhd sh2_pmau.vhd memory_interface.vhd sh2_dmau.vhd sh2_alu.vhd sh2_control.vhd sh2_reg.vhd sh2_cpu.vhd memory.vhd sh2_cpu_tb.vhd reg.vhd alu.vhd 

OBJECTS = $(patsubst %.vhd,$(WORKDIR)%.o,$(SOURCES))
# OBJECTS = utils.o logging.o AnsiEscape.o sh2_constants.o mau.o sh2_pmau.o memory_interface.o sh2_dmau.o sh2_alu.o sh2_control.o sh2_reg.o sh2_cpu.o memory.o sh2_cpu_tb.o reg.o alu.o 

# The top level entity
TOPLEVEL = sh2_cpu_tb

# Note that $@ specifies the first target, which in this case is the top level entity
all: $(TOPLEVEL)

$(TOPLEVEL): $(OBJECTS)
	$(GHDL) -e $(BUILDFLAGS) $@

# Dependencies:
$(WORKDIR)sh2_pmau.o: $(WORKDIR)sh2_constants.o  $(WORKDIR)mau.o
$(WORKDIR)memory_interface.o: $(WORKDIR)utils.o $(WORKDIR)logging.o
$(WORKDIR)sh2_dmau.o: $(WORKDIR)mau.o $(WORKDIR)sh2_constants.o
$(WORKDIR)sh2_control.o: $(WORKDIR)memory_interface.o $(WORKDIR)logging.o  $(WORKDIR)sh2_pmau.o $(WORKDIR)sh2_dmau.o $(WORKDIR)sh2_alu.o  $(WORKDIR)utils.o
$(WORKDIR)sh2_cpu.o: $(WORKDIR)sh2_pmau.o $(WORKDIR)memory_interface.o $(WORKDIR)sh2_control.o $(WORKDIR)logging.o $(WORKDIR)sh2_constants.o $(WORKDIR)sh2_dmau.o $(WORKDIR)sh2_reg.o $(WORKDIR)sh2_alu.o
$(WORKDIR)memory.o:  $(WORKDIR)logging.o $(WORKDIR)utils.o
$(WORKDIR)sh2_cpu_tb.o: $(WORKDIR)utils.o  $(WORKDIR)logging.o $(WORKDIR)AnsiEscape.o $(WORKDIR)sh2_cpu.o $(WORKDIR)memory.o
$(WORKDIR)reg.o: $(WORKDIR)logging.o

# Note that $> is the first prereq (ie, the source file).
$(OBJECTS): $(WORKDIR)%.o: %.vhd
	$(GHDL) -a $(BUILDFLAGS) $<


asm:
	cd asm && $(MAKE)

test: $(TOPLEVEL) asm
	ghdl -r $(TOPLEVEL) $(RUNFLAGS)	

clean:
	ghdl --clean --workdir=$(WORKDIR) --std=08
	# rm -rf *.cf *.o $(WORKDIR)

view:
	gtkwave $(WAVEFORM) &

# $(WORKDIR)%.o: %.vhd
# 	$(GHDL) -a $(BUILDFLAGS) $<


#%.o: %.vhd
#	$(GHDL) -a $(BUILDFLAGS) $<

.PHONY: test clean build view asm run
