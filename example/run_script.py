"""
Generate the SPI registers and corresponding verilog.
Tested with python 3.4 as well as 2.7.9

Copyright 2013-2016 Colin Weltin-Wu (colinww@gmail.com)
UC San Diego Integrated Signal Processing Group

Licensed under GNU General Public License 3.0 or later. 
Some rights reserved. See LICENSE.
"""

""" VARIABLES
SRC_MAP: points to the csv file which defines the registers and fields of the
SPI interface. See example file for syntax and special suffices.
SRC_REG: points to the file which defines the register test interface.
DEST_FIELDS_REGS: destination for the companion python classes.
DEST_VERILOG: destination for the RTL. spi_top.v is the top-level module.
SCRIPT_DIR: points to the scripts directory.
"""
SRC_MAP = "definitions/davinci_spi_map.csv"
SRC_REG = "definitions/davinci_reg_map.csv"
DEST_FIELDS_REGS = "results_python/"
DEST_VERILOG = "results_verilog/"
SCRIPT_DIR = "../scripts/"

# Load the tools
import sys
sys.path.insert(0, SCRIPT_DIR)
import make_spi

# The following was just used during testing, ignore
"""
import importlib
importlib.reload(make_spi)
"""

"""
mdata is a dict containing information on all the physical map addresses
mdata keys are the register addresses. Each mdata entry is a dict containing:
  -- 'Name': register name
  -- 'rmask': read mask of the byte: 1 indicates bit is readable
  -- 'wmask': write mask of the byte: 1 indicates bit is writable
fdata is a dict containing all the fields, i.e. the logical grouping of all
the bits in the corresponding addresses in mdata. Each fdata entry is
a dict, keyed by the field name. Each entry contains:
  -- 'Addr': the register address which contains this field. If the field
             spans multiple addresses, this is a list.
  -- 'Bit_Abs': a list [a,b] whose entries define the starting and ending
                position within the register of the field. So the field
                occupies bits [b:a] in the register location 'Addr'. If
                the field spans multiple addresses, this is a list of lists,
                each list corresponding to the location in the address.
  -- 'Bit_Rel': The relative bit position of the field corresponding to the
                absolute bit locations above.
  -- 'Rd_Wr': A two element boolean list, [rd, wr] indicating the actions
              allowed.
  -- 'Root': For reserved field names ('_step_clk', '_hold', etc) the name
             root chops off the suffix.
rdata is a dict containing all the information on the register test interface
addresses. Each entry is a dict (keyed by address) containing:
  -- 'Name': test register name
  -- 'Addr': Address of the register
  -- 'Size': Width of the register
  -- 'Rd_Wr': defined as above, indicates if register is readable/writable
"""
mdata,fdata = make_spi.ExtractMap(SRC_MAP)
rdata = make_spi.ExtractReg(SRC_REG)

"""
Generate the verilog RTL to implement the defined map and register interface.
"""
map_ro,map_wo,map_rw = make_spi.GenSpiMap(mdata,fdata, DEST_VERILOG + 'spi_map.v')
reg_rd,reg_wr = make_spi.GenSpiReg(rdata, 37, DEST_VERILOG + 'spi_reg.v')
make_spi.GenSpiTop(map_rw, map_ro, map_wo, reg_rd, reg_wr, DEST_VERILOG + 'spi_top.v')

"""
These functions map the fdata, mdata, and rdata to python classes for
interacting with the IC through the test motherboard.
"""
make_spi.GenMap(mdata, fdata, DEST_FIELDS_REGS + 'spi_map_regs_fields.py')
make_spi.GenReg(rdata, DEST_FIELDS_REGS + 'spi_test_regs.py')
