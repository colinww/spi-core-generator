"""
Generates the registers and fields of an SPI map, also generates the digital
registers which are exposed to the test interface.
"""

import csv
import os
from math import log10

# Make a note of the location of module
module_dir = os.path.dirname(os.path.abspath(__file__))

def PrintPorts(pdict, type, indent, max_char):
    spc = ' ' * indent
    ret_text = ''
    port_names = sorted(pdict.keys())
    maxlen = max(len(p['Name']) for i,p in pdict.items())
    maxsize = max(p['Size'] for i,p in pdict.items())
    maxdig = int(log10(maxsize) + 2)
    if "module" == type:
        curr_line = spc
        num_written = 0
        for name in port_names:
            p = pdict[name]
            if 0 == num_written:
                # If this is the first port in the line, just write it
                curr_line += p['Name'] + ','
                num_written = 1
            elif max_char-2 > len(curr_line) + len(p['Name']):
                # If this is not the first, only write it if it fits
                curr_line += ' ' + p['Name'] + ','
                num_written += 1
            else:
                # If the line would be too full, write it and start a new one
                ret_text += curr_line + '\n'
                num_written = 1
                curr_line = spc + p['Name'] + ','
        if 0 != num_written:
            # If number written is non-zero, need to flush the line
            ret_text += curr_line + '\n'
    elif "declaration" == type:
        for name in port_names:
            p = pdict[name]
            if 1 == p['Size']:
                curr_line = spc + '{typ:<{wid}}'.format(typ=p['Type'], wid=maxdig+16) \
                            + '{nm};\n'.format(nm=p['Name'])
            else:
                curr_line = spc + '{typ:<{wid}}'.format(typ=p['Type'], wid=11) \
                            + '[{ndig}:0]'.format(ndig=p['Size']-1).ljust(maxdig+5) \
                            + '{nm};\n'.format(nm=p['Name'])
            ret_text += curr_line
    elif "declaration_top" == type:
        # declaration_top converts all output reg types to just outputs
        for name in port_names:
            p = pdict[name]
            if 'output reg' == p['Type']:
                ptype = 'output'
            else:
                ptype = p['Type']
            if 1 == p['Size']:
                curr_line = spc + '{typ:<{wid}}'.format(typ=ptype, wid=maxdig+16) \
                            + '{nm};\n'.format(nm=p['Name'])
            else:
                curr_line = spc + '{typ:<{wid}}'.format(typ=ptype, wid=11) \
                            + '[{ndig}:0]'.format(ndig=p['Size']-1).ljust(maxdig+5) \
                            + '{nm};\n'.format(nm=p['Name'])
            ret_text += curr_line
    elif "instance" == type:
        for name in port_names:
            nom = pdict[name]['Name']
            curr_line = spc + '.{nm:<{wid}}'.format(nm=nom, wid=maxlen+1) \
                        + '({0}),\n'.format(nom)
            ret_text += curr_line
    return ret_text


def ExtractMap(map_file):
    fr = open(map_file, 'r')
    csv_file = csv.DictReader(fr)
    map_data = dict()
    field_data = dict()
    for row in csv_file:
        # Row number
        addr = int(row['Addr'])
        # Add read and write masks
        rmask = 0
        wmask = 0
        for bit_idx in range(0,8):
            bit_name = row['bit[' + str(bit_idx) + ']']
            if ( '-' == bit_name ):
                # Bit is empty, move on
                continue
            else:
                # Bit is not empty, determine its type
                name_num = bit_name.split('[')
            if ( 1 < len(name_num) ):
                # This is part of a multi-bit field
                bit_num = int(name_num[1].strip(']'))
            else:
                bit_num = 0
            bit_name = name_num[0]
            # Stip off ro, wo, or rw suffix
            rd = 0
            wr = 0
            name_pieces = bit_name.split('_')
            if ( 'ro' == name_pieces[-1] ):
                # Read-only register
                rd = 1
                rmask = rmask + 2**bit_idx
                name_root = '_'.join(name_pieces[0:-1])
                root = name_root
            elif ( 'wo' == name_pieces[-1] ):
                # Write-only register
                wr = 1
                wmask = wmask + 2**bit_idx
                name_root = '_'.join(name_pieces[0:-1])
                root = name_root
            elif ( 'rw' == name_pieces[-1] ):
                # Read-write register
                rd = 1
                wr = 1
                wmask = wmask + 2**bit_idx
                rmask = rmask + 2**bit_idx
                name_root = '_'.join(name_pieces[0:-1])
                root = name_root
            elif ( 'step_clk' == '_'.join(name_pieces[-2:]) ):
                # Digital register clock
                wr = 1
                wmask = wmask + 2**bit_idx
                rmask = rmask + 2**bit_idx
                name_root = '_'.join(name_pieces)
                root = '_'.join(name_pieces[0:-2])
            elif ( 'hold' == name_pieces[-1] ):
                # Digital register clock disable
                rd = 1
                wr = 1
                wmask = wmask + 2**bit_idx
                rmask = rmask + 2**bit_idx
                name_root = '_'.join(name_pieces)
                root = '_'.join(name_pieces[0:-1])
            else:
                print('_'.join(name_pieces) + " is an unknown register type!\n")
                continue
            # Fill in field information
            try:
                field_data[name_root]
                # This field has already appeared at least once
                if ( addr in field_data[name_root]['Addr'] ):
                    addr_idx = field_data[name_root]['Addr'].index(addr)
                    # Get the bounds of the field within this address
                    abs_bounds = field_data[name_root]['Bit_Abs'][addr_idx]
                    rel_bounds = field_data[name_root]['Bit_Rel'][addr_idx]
                    # Determine if this is larger than previously found
                    if ( bit_idx < abs_bounds[0] ):
                        # This bit index is smaller than previously found
                        abs_bounds[0] = bit_idx
                        rel_bounds[0] = bit_num
                    elif ( bit_idx > abs_bounds[1] ):
                        # Bit index is larger than previously found
                        abs_bounds[1] = bit_idx
                        rel_bounds[1] = bit_num
                    field_data[name_root]['Bit_Abs'][addr_idx] = abs_bounds
                    field_data[name_root]['Bit_Rel'][addr_idx] = rel_bounds
                else:
                    # This is a new address for an existing field
                    field_data[name_root]['Addr'].append(addr)
                    field_data[name_root]['Bit_Abs'].append([bit_idx, bit_idx])
                    field_data[name_root]['Bit_Rel'].append([bit_num, bit_num])
            except:
                # This is a new field
                field_data[name_root] = dict()
                field_data[name_root]['Addr'] = [addr]
                field_data[name_root]['Root'] = root
                field_data[name_root]['Bit_Abs'] = [[bit_idx, bit_idx]]
                field_data[name_root]['Bit_Rel'] = [[bit_num, bit_num]]
                field_data[name_root]['Rd_Wr'] = [rd, wr]
            # Do a range check
            bit_rel = field_data[name_root]['Bit_Rel'][0]
            bit_abs = field_data[name_root]['Bit_Abs'][0]
            if bit_abs[1] - bit_abs[0] != bit_rel[1] - bit_rel[0]:
                print("There is a problem with the indexing of field " \
                      + '\'' + name_root + '\'\n')
        # Create register data
        if addr in map_data:
            print("Non-unique map register addresses: registers " \
                  + row['Name'] + " and " + map_data[addr]['Name'] \
                  + " both exist at address " + str(addr) + "!")
            return
        else:
            map_data[addr] = dict()
            map_data[addr]['Name'] = row['Name']
            map_data[addr]['rmask'] = rmask
            map_data[addr]['wmask'] = wmask
    return map_data,field_data


def ExtractReg(reg_file):
    fr = open(reg_file, 'r')
    csv_file = csv.DictReader(fr)
    rdata = dict()
    # Registers should be addressed by name, since not necessarily sequential
    for row in csv_file:
        try:
            # This should not work since each name should appear once
            rdata[row['Name']]
            print("Register name: " + row['Name'] + " is not unique! Exiting\n")
            return
        except:
            # Generate the row information
            rdata[row['Name']] = dict()
            rdata[row['Name']]['Addr'] = int(row['Addr'])
            rdata[row['Name']]['Size'] = int(row['Size'])
            rdata[row['Name']]['Read'] = int(row['Read'])
            rdata[row['Name']]['Write'] = int(row['Write'])
    return rdata

def GenMap(mdata, fdata, out_file):
    fh = open(out_file, 'w')
    # Write header information
    cl = "\"\"\"\n"
    fh.write(cl)
    cl = "Automatically generated register map.\n"
    fh.write(cl)
    cl = "\"\"\"\n\n"
    fh.write(cl)
    cl = "from collections import namedtuple\n\n"
    fh.write(cl)
    cl = "register = namedtuple(\"register\", \"addr width value rmask wmask\")\n"
    fh.write(cl)
    cl = "reg_field = namedtuple(\"reg_field\", \"addr offset width rd wr\")\n\n"
    fh.write(cl)
    # Process the spi map file and print the register map
    cl = "# Registers\n"
    fh.write(cl)
    cl = "class Registers:\n"
    fh.write(cl)
    cl = "\tdef __init__(self):\n"
    fh.write(cl)
    # Loop through rows, create a dict of field names
    field_data = dict()
    for addr in range(0,len(mdata)):
        # Print register name to file
        cl = "\t\tself." + mdata[addr]['Name'] \
            + " = register(" + str(addr) \
            + ", 8, 0, " + str(mdata[addr]['rmask']) \
            + ", " +  str(mdata[addr]['wmask']) + ")\n"
        fh.write(cl)   
    # Print fields
    cl = "# Fields\n"
    fh.write(cl)
    cl = "class Fields:\n"
    fh.write(cl)
    cl = "\tdef __init__(self):\n"
    fh.write(cl)
    for fname,addrs in fdata.items():
        rd = addrs['Rd_Wr'][0]
        wr = addrs['Rd_Wr'][1]        
        if ( 1 == len(addrs['Addr']) ):
            # Only one address entry for this field
            # Calculate field offset and size
            reg_offset = addrs['Bit_Abs'][0][0]
            reg_size = addrs['Bit_Abs'][0][1] - addrs['Bit_Abs'][0][0] + 1
            cl = "\t\tself." + fname + " = reg_field(" + str(addrs['Addr'][0]) \
            + ', ' + str(reg_offset) + ", " +  str(reg_size) +  ", " \
            + str(rd) + ", " + str(wr) + ")\n"
            fh.write(cl)
        else:
            # Field spans multiple addresses, loop through each address
            for idx in range(0, len(addrs['Addr'])):
                fname_subset = fname + '_' + str(addrs['Bit_Rel'][idx][1]) \
                    + '_' + str(addrs['Bit_Rel'][idx][0])
                reg_offset = addrs['Bit_Abs'][idx][0]
                reg_size = addrs['Bit_Abs'][idx][1] - addrs['Bit_Abs'][idx][0] + 1
                cl = "\t\tself." + fname_subset + " = reg_field("  \
                    + str(addrs['Addr'][idx]) + ', ' + str(reg_offset) \
                    + ", " +  str(reg_size) + ", " + str(rd) + ", " \
                    + str(wr) + ")\n"
                fh.write(cl)
    
    # Close file
    fh.close()
    
def GenReg(rdata, out_file):
    fh = open(out_file, 'w')
    # Write header information
    cl = "\"\"\"\n"
    fh.write(cl)
    cl = "Automatically generated registers\n"
    fh.write(cl)
    cl = "\"\"\"\n\n"
    fh.write(cl)
    cl = "from collections import namedtuple\n\n"
    fh.write(cl)
    cl = "register = namedtuple(\"register\", \"addr width value rmask wmask\")\n\n"
    fh.write(cl)
    # Process the spi map file and print the register map
    cl = "# Test registers\n"
    fh.write(cl)
    cl = "class TestRegs:\n"
    fh.write(cl)
    cl = "\tdef __init__(self):\n"
    fh.write(cl)
    for rname in rdata:
        reg = rdata[rname]
        cl = "\t\tself." + rname + " = register(" + str(reg['Addr']) \
            + ", " + str(reg['Size']) + ", 0, " + str(reg['Read']) + ", " \
            +  str(reg['Write']) + ")\n"
        fh.write(cl)
    fh.close()


def GenWriteMemory(fdata, addr):
    ports = dict()
    reset_block = ''
    code_block = 'if (( ' + str(addr) + ' == rv_addr ) && r_write_map ) begin\n'
    # Iterate over fields, picking out any that are associated with addr
    for name,field in fdata.items():
        # Check if field has the right address and is type rw
        if ( addr in field['Addr'] ) and ( [1, 1] == field['Rd_Wr'] ):
            # Add port to dict as a registered output
            new_var = dict()
            new_var['Type'] = 'output reg'
            new_var['Size'] = max(map(max, zip(*field['Bit_Rel']))) + 1
            if 1 == new_var['Size']:
                nom = 'o_' + name
            else:
                nom = 'vo_' + name
            new_var['Name'] = nom
            ports[name] = new_var
            # Generate the reset string
            reset_block += nom + ' <= 0;\n'
            # Find index of address (for fields spanning multiple addresses)
            addr_idx = field['Addr'].index(addr)
            bit_abs = field['Bit_Abs'][addr_idx]
            bit_rel = field['Bit_Rel'][addr_idx]
            if bit_abs[0] == bit_abs[1]:
                # This is a single bit field
                rw_str = '  ' + nom + ' <= vi_data_rx[' + str(bit_abs[0]) + '];\n'
            elif 1 == len(field['Addr']):
                # The entire field is contained in a single register
                rw_str = '  ' + nom + ' <= vi_data_rx[' + str(bit_abs[1]) \
                         + ':' + str(bit_abs[0]) + '];\n'
            else:
                # This field spans multiple addresses
                rw_str = '  ' + nom + '[' + str(bit_rel[1]) + ':' + str(bit_rel[0]) \
                         + '] <= vi_data_rx[' + str(bit_abs[1]) \
                         + ':' + str(bit_abs[0]) + '];\n'
            code_block += rw_str
    code_block += 'end\n'
    return ports,reset_block,code_block


def GenReadMemory(fdata, addr):
    ports = dict()
    read_wires = ''
    code_block = ''
    # First iterate over all fields, picking out all associated with this address
    fields_at_addr = dict()
    for name,field in fdata.items():
        # Check if field has the right address and is readable
        if ( addr in field['Addr'] ) and ( 1 == field['Rd_Wr'][0] ):
            fields_at_addr[name] = field
    # If there are no readable fields at this address, return
    if 0 == len(fields_at_addr):
        return ports,read_wires,code_block
    # Starting at bit index 7, assemble the output field
    bit_idx = 7
    num_zeros = 0
    while 0 <= bit_idx:
        field_found = 0
        next_idx = bit_idx - 1
        for name,field in fields_at_addr.items():
            addr_idx = field['Addr'].index(addr)
            # Find the field which starts at this bit position
            if bit_idx == field['Bit_Abs'][addr_idx][1]:
                # Before writing the found field, dump the zeros
                if 0 != num_zeros:
                    if '' == code_block:
                        code_block = str(num_zeros) + '\'b' + '0'*num_zeros
                    else:
                        code_block += ',' + str(num_zeros) + '\'b' + '0'*num_zeros
                    num_zeros = 0
                new_var = dict()
                new_var['Size'] = max(map(max, zip(*field['Bit_Rel']))) + 1
                # If field is not writable, it is read-only, hence an input
                if field['Rd_Wr'][1]:
                    new_var['Type'] = 'output reg'
                    if 1 == new_var['Size']:
                        nom = 'o_' + name
                    else:
                        nom = 'vo_' + name
                else:
                    new_var['Type'] = 'input'
                    if 1 == new_var['Size']:
                        nom = 'i_' + name
                    else:
                        nom = 'vi_' + name
                new_var['Name'] = nom
                ports[name] = new_var
                # Determine if this is a multi-address field
                if 1 < len(field['Addr']):
                    fname = nom + '[' + str(field['Bit_Rel'][addr_idx][1]) \
                            + ':' + str(field['Bit_Rel'][addr_idx][0]) + ']'
                else:
                    fname = nom
                # Determine if this field is first in the concat list (no comma)
                if '' == code_block:
                    code_block = fname
                else:
                    code_block += ',' + fname
                field_found = 1
                next_idx = field['Bit_Abs'][addr_idx][0] - 1
                continue
        if not field_found:
            # If field was not found, increment the zero count
            num_zeros += 1
        # Jump down to the next vacant bit (toward 0 index)
        bit_idx = next_idx
    # Flush out any remaining zeros
    if 0 != num_zeros:
        if '' == code_block:
            code_block = str(num_zeros) + '\'b' + '0'*num_zeros
        else:
            code_block += ',' + str(num_zeros) + '\'b' + '0'*num_zeros    
    read_wires = 'wire read_addr_' + str(addr) + ' = ( ' \
               + str(addr) + ' == vi_data_rx ) && ( 1 == vi_byte_num );\n'
    code_block = 'if ( read_addr_' + str(addr) + ' )\n' \
                 '  r_data_to_send <= {' + code_block + '};\n'
    return ports,read_wires,code_block


def GenWriteOnly(fdata, addr):
    ports = dict()
    pulse_declaration = ''
    pulse_reset = ''
    pulse_reg = ''
    assigns = ''
    # Iterate over fields, picking out any that are associated with addr
    for name,field in fdata.items():
        # Check if field has the right address and is type wo
        if ( addr in field['Addr'] ) and ( [0, 1] == field['Rd_Wr'] ):
            # Write-only fields cannot span multiple register addresses
            if 1 < len(field['Addr']):
                print("Field " + name + " spans multiple addresses, illegal!\n")
                return
            # Add this field to the port list as combinational output
            new_var = dict()
            new_var['Type'] = 'output'
            new_var['Size'] = field['Bit_Rel'][0][1] + 1
            if 1 == new_var['Size']:
                nom = 'o_' + name
            else:
                nom = 'vo_' + name
            new_var['Name'] = nom
            ports[name] = new_var
            # Add to the assign statement block
            bit_abs = field['Bit_Abs'][0]
            bit_rel = field['Bit_Rel'][0]
            if bit_abs[0] == bit_abs[1]:
                # This is a single bit field
                rw_str = 'assign ' + nom + ' = vi_data_rx[' + str(bit_abs[0]) \
                         + '] && r_pulse_' + str(addr) + ';\n'
            else:
                # This is a multi-bit write-only output
                rw_str = 'assign ' + nom + ' <= vi_data_rx[' + str(bit_abs[1]) \
                         + ':' + str(bit_abs[0]) \
                         + '] & {' + str(bit_abs[1]-bit_abs[0]) \
                         + '{r_pulse_' + str(addr) + '}};\n'
            assigns += rw_str
    if '' != assigns:
        # If write-only registers are found, a pulse wire needs to be declared
        pulse_declaration = 'reg r_pulse_' + str(addr) + ';\n'
        pulse_reset = 'r_pulse_' + str(addr) + ' <= 0;\n'
        pulse_reg = 'if ( ' + str(addr) + ' == rv_addr )\n' \
                 '  r_pulse_' + str(addr) + ' <= 1;\n'
    return ports,pulse_declaration,pulse_reset,pulse_reg,assigns

    
def GenSpiMap(mdata, fdata, target_file):
    input_ports = dict()
    output_ports = dict()
    register_ports = dict()
    mem_resets = '' # All the fields are reset
    mem_writes = '' # Fields written
    mem_reads = '' # Fields and inputs read
    wire_read = '' # Read select signal
    declare_pulse = '' # Pulse declarations
    reset_pulse = '' # Reset the pulse signals
    reg_pulse = '' # Set the pulse signals
    wire_write = '' # Write-only fields
    # Find the bounds on the register addresses
    for addr in range(0,max(mdata.keys())+1):
        # Parse out the port specification
        def PortParse(pname, p):
            if 'input' == p['Type']:
                try:
                    input_ports[pname]
                    # Port already exists in list
                except:
                    input_ports[pname] = p
            if 'output' == p['Type']:
                try:
                    output_ports[pname]
                    # Port already exists
                except:
                    output_ports[pname] = p
            if 'output reg' == p['Type']:
                try:
                    register_ports[pname]
                    # Port already exists
                except:
                    register_ports[pname] = p
        # Parse out all the rw fields
        ports,reset_block,code_block = GenWriteMemory(fdata, addr)
        # Fill in ports
        for pname,p in ports.items():
            PortParse(pname, p)
        # Append code blocks
        if 0 != len(ports):
            mem_resets += reset_block
            mem_writes += code_block
        # Parse out all the wo
        ports,pulse_dec,pulse_reset,pulse_reg,assign = GenWriteOnly(fdata, addr)
        # Fill in ports
        for pname,p in ports.items():
            PortParse(pname, p)
        # Append code
        if 0 != len(ports):
            declare_pulse += pulse_dec
            reset_pulse += pulse_reset
            reg_pulse += pulse_reg
            wire_write += assign
        # Parse out all the readable fields
        ports,readwire,codeblock = GenReadMemory(fdata, addr)
        # Fill in ports
        for pname,p in ports.items():
            PortParse(pname, p)
        # Append code blocks
        if 0 != len(ports):
            wire_read += readwire
            mem_reads += codeblock
    # Open the destination file
    fh = open(target_file, 'w')
    # Open the source template file
    with open(module_dir + '/../templates/spi_map_template.v') as src_file:
        for cline in src_file:
            if "// INSERT_RO" == cline.strip():
                ctext = PrintPorts(input_ports, "module", 15, 80)
                fh.write(ctext)
            elif "// INSERT_WO" == cline.strip():
                ctext = PrintPorts(output_ports, "module", 15, 80)
                fh.write(ctext)
            elif "// INSERT_RW" == cline.strip():
                ctext = PrintPorts(register_ports, "module", 15, 80)
                fh.write(ctext)
            elif "// INSERT_RO_DECLARATION" == cline.strip():
                ctext = PrintPorts(input_ports, "declaration", 2, 80)
                fh.write(ctext)
            elif "// INSERT_WO_DECLARATION" == cline.strip():
                ctext = PrintPorts(output_ports, "declaration", 2, 80)
                fh.write(ctext)
            elif "// INSERT_RW_DECLARATION" == cline.strip():
                ctext = PrintPorts(register_ports, "declaration", 2, 80)
                fh.write(ctext)
            elif "// INSERT_RESET_REGS" == cline.strip():
                for cl in mem_resets.split('\n'):
                    if '' != cl:
                        fh.write(' '*6 + cl + '\n')
            elif "// INSERT_WRITE_REGS" == cline.strip():
                for cl in mem_writes.split('\n'):
                    if '' != cl:
                        fh.write(' '*6 + cl + '\n')
            elif "// INSERT_READ_REGS" == cline.strip():
                for cl in mem_reads.split('\n'):
                    if '' != cl:
                        fh.write(' '*6 + cl + '\n')
            elif "// INSERT_READ_WIRES" == cline.strip():
                for cl in wire_read.split('\n'):
                    if '' != cl:
                        fh.write(' '*2 + cl + '\n')
            elif "// INSERT_PULSE_DECLARATION" == cline.strip():
                for cl in declare_pulse.split('\n'):
                    if '' != cl:
                        fh.write(' '*2 + cl + '\n')
            elif "// INSERT_PULSE_RESET" == cline.strip():
                for cl in reset_pulse.split('\n'):
                    if '' != cl:
                        fh.write(' '*6 + cl + '\n')
            elif "// INSERT_PULSE_REG" == cline.strip():
                for cl in reg_pulse.split('\n'):
                    if '' != cl:
                        fh.write(' '*8 + cl + '\n')
            elif "// INSERT_WRITE_WIRES" == cline.strip():
                for cl in wire_write.split('\n'):
                    if '' != cl:
                        fh.write(' '*2 + cl + '\n')
            else:
                fh.write(cline)
    fh.close()
    return input_ports,output_ports,register_ports


def GenSpiReg(rdata, bus_width, target_file):
    read_ports = dict()
    write_ports = dict()
    num_regs = len(rdata)
    reg_list = [None] * num_regs
    for name,reg in rdata.items():
        # Rework the rdata dict into an ordered list of dicts sorted by address
        reg['Name'] = name
        try:
            reg_list[reg['Addr']] = reg
        except:
            print("There seem to be gaps in the register address: " \
                  + str(reg['Addr']) + " is out of range.")
            return
        # Generate the read ports
        if reg['Read']:
            new_port = dict()
            new_port['Name'] = 'o_' + name + '_rd'
            new_port['Size'] = 1
            new_port['Type'] = 'output'
            read_ports[name] = (new_port)
        # Generate the write ports
        if reg['Write']:
            new_port = dict()
            new_port['Name'] = 'o_' + name + '_wr'
            new_port['Size'] = 1
            new_port['Type'] = 'output'
            write_ports[name] = (new_port)
    # Generate the read and write wire mapping (in address order)
    rd_mask = ''
    rd_nbytes = ''
    rd_bus = ''
    wr_bus = ''
    for idx,reg in enumerate(reg_list):
        mask_str = ''
        byte_str = ''
        rd_str = ''
        wr_str = ''
        # Generate the read mask and number of bytes, and read wires
        if reg['Read']:
            if bus_width == reg['Size']:
                mask_str = ' '*6 + str(idx) + ' : rv_rd_mask = {' \
                           + str(bus_width) + '{1\'b1}};\n'
            else:
                mask_str = ' '*6 + str(idx) + ' : rv_rd_mask = {{' \
                           + str(bus_width-reg['Size']) + '{1\'b0}},{' \
                           + str(reg['Size']) + '{1\'b1}}};\n'
            byte_str = ' '*6 + str(idx) + ' : rv_num_rd_bytes = ' \
                       + str(int(reg['Size']/8+1)) + '; // ' \
                       + reg['Name'] + '\n'
            rd_str = ' '*2 + 'assign ' + read_ports[reg['Name']]['Name'] \
                     + ' = rv_rd_tbus[' + str(idx) + '];\n'
        else:
            mask_str = ' '*6 + str(idx) + ' : rv_rd_mask = 0;\n'
            byte_str = ' '*6 + str(idx) + ' : rv_num_rd_bytes = 0;\n'
        rd_mask += mask_str
        rd_nbytes += byte_str
        rd_bus += rd_str
        if reg['Write']:
            wr_str = ' '*2 + 'assign ' + write_ports[reg['Name']]['Name'] \
                     + ' = rv_wr_tbus[' + str(idx) + '];\n'
        wr_bus += wr_str
    # Open the destination file
    fh = open(target_file, 'w')
    # Open the source template file
    with open(module_dir + '/../templates/spi_reg_template.v') as src_file:
        for cline in src_file:
            if "// INSERT_RD" == cline.strip():
                ctext = PrintPorts(read_ports, "module", 16, 80)
                fh.write(ctext)
            elif "// INSERT_WR" == cline.strip():
                ctext = PrintPorts(write_ports, "module", 16, 80)
                fh.write(ctext)
            elif "// INSERT_RD_DECLARATION" == cline.strip():
                ctext = PrintPorts(read_ports, "declaration", 2, 80)
                fh.write(ctext)
            elif "// INSERT_WR_DECLARATION" == cline.strip():
                ctext = PrintPorts(write_ports, "declaration", 2, 80)
                fh.write(ctext)
            elif "// INSERT_NUM_REG" == cline.strip():
                ctext = '  localparam NUM_REGS = ' + str(num_regs) + ';\n'
                fh.write(ctext)
            elif "// INSERT_MASK" == cline.strip():
                fh.write(rd_mask)
            elif "// INSERT_NUM_BYTES" == cline.strip():
                fh.write(rd_nbytes)
            elif "// INSERT_RD_BUS" == cline.strip():
                fh.write(rd_bus)
            elif "// INSERT_WR_BUS" == cline.strip():
                fh.write(wr_bus)
            else:
                fh.write(cline)
    fh.close()
    return read_ports,write_ports


def GenSpiTop(map_rw, map_ro, map_wo, reg_rd, reg_wr, target_file):
    # Open the destination file
    fh = open(target_file, 'w')
    # Open the source template file
    with open(module_dir + '/../templates/spi_top_template.v') as src_file:
        for cline in src_file:
            if "// INSERT_RD" == cline.strip():
                ctext = PrintPorts(reg_rd, "module", 15, 80)
                fh.write(ctext)
            elif "// INSERT_WR" == cline.strip():
                ctext = PrintPorts(reg_wr, "module", 15, 80)
                fh.write(ctext)
            elif "// INSERT_RD_DECLARATION" == cline.strip():
                ctext = PrintPorts(reg_rd, "declaration_top", 2, 80)
                fh.write(ctext)
            elif "// INSERT_WR_DECLARATION" == cline.strip():
                ctext = PrintPorts(reg_wr, "declaration_top", 2, 80)
                fh.write(ctext)
            elif "// INSERT_RO" == cline.strip():
                ctext = PrintPorts(map_ro, "module", 15, 80)
                fh.write(ctext)
            elif "// INSERT_WO" == cline.strip():
                ctext = PrintPorts(map_wo, "module", 15, 80)
                fh.write(ctext)
            elif "// INSERT_RW" == cline.strip():
                ctext = PrintPorts(map_rw, "module", 15, 80)
                fh.write(ctext)
            elif "// INSERT_RO_DECLARATION" == cline.strip():
                ctext = PrintPorts(map_ro, "declaration_top", 2, 80)
                fh.write(ctext)
            elif "// INSERT_WO_DECLARATION" == cline.strip():
                ctext = PrintPorts(map_wo, "declaration_top", 2, 80)
                fh.write(ctext)
            elif "// INSERT_RW_DECLARATION" == cline.strip():
                ctext = PrintPorts(map_rw, "declaration_top", 2, 80)
                fh.write(ctext)
            elif "// INSERT_RD_INST" == cline.strip():
                ctext = PrintPorts(reg_rd, "instance", 22, 80)
                fh.write(ctext)
            elif "// INSERT_WR_INST" == cline.strip():
                ctext = PrintPorts(reg_wr, "instance", 22, 80)
                fh.write(ctext)
            elif "// INSERT_RO_INST" == cline.strip():
                ctext = PrintPorts(map_ro, "instance", 22, 80)
                fh.write(ctext)
            elif "// INSERT_WO_INST" == cline.strip():
                ctext = PrintPorts(map_wo, "instance", 22, 80)
                fh.write(ctext)
            elif "// INSERT_RW_INST" == cline.strip():
                ctext = PrintPorts(map_rw, "instance", 22, 80)
                fh.write(ctext)            
            else:
                fh.write(cline)
    fh.close()
