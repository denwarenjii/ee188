# Generate the control signals for the SH-2's control unit based on the CSV file.
import csv
import re

def clean_newlines(line):
    return re.sub(r'\n+', ' ', line)

if __name__ == "__main__":
    with open("control_signals.csv", newline='') as infile

         reader = csv.DictReader(infile)

         keys = [
            'Instruction Mnemonic', 
            'Arguments', 
            'Function', 
            'Operation', 
            'Format Name', 
            'Format', 
            'Program Memory Addresing Mode', 
            'SH2PMAU : in RegIn',
            'SH2PMAU : in PRIn',
            'SH2PMAU : in PRWriteEN',
            'SH2PMAU : in Off8',
            'SH2PMAU : in Off12',
            'SH2PMAU : in PCAddrMode',
            'SH2PMAU : out PCOut',
            'SH2PMAU : out PROut',
            'Data Memory Addressing Mode',
            'SH2DMAU : in RegSrc',
            'SH2DMAU : in R0Src',
            'SH2DMAU : in PCSrc',
            'SH2DMAU : in GBRIn',
            'SH2DMAU : in GBRWriteEN',
            'SH2DMAU : in Off4',
            'SH2DMAU : in Off8',
            'SH2DMAU : in BaseSel',
            'SH2DMAU : in IndexSel',
            'SH2DMAU : in OffScalarSel',
            'SH2DMAU : in IncDecSel',
            'SH2DMAU : out Address',
            'SH2DMAU : out AddrSrcOut',
            'SH2DMAU : out GBROut',
            'Memory Access Type (Read, Write)',
            'Data Size',
            'Data Extension (Zero, Sign, None)',
            'Execution Clocks', 
            'Implemented?'
        ]

