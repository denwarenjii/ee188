# Find all .asm files in the current directory
ASM_FILES := $(wildcard *.asm)

# Create list of corresponding .bin files
BIN_FILES := $(ASM_FILES:.asm=.bin)

# Default target: build all .bin files
all: $(BIN_FILES)

# Pattern rule to build a .bin file from an .asm file
%.bin: %.asm
	asl -cpu SH7600 $< && p2bin $*.p
	@rm -f $*.p  # Clean up intermediate .p file

# Clean target to remove generated files
clean:
	rm -f *.bin *.p

.PHONY: all clean

