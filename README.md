## spi-core-generator

A python-based tool that loads user-provided definition files (defining registers and fields) and generates a synthesizable SPI interface. A set of python classes is also generated to aid testbench (verification and eval) development.

An example is given in example/run_script.py. cd to this directory and run

    python run_script.py

This has been tested on python 3.3 and 2.7.9. The script calls the functions defined in scripts/make_spi.py on the .csv files in definitions. The SPI interface has two use models, programming and testing.

### Part programming

In this mode, SPI is used to interface with a memory array within the IC. This is the most common use of the interface, where the memory array configures the IC functionality and behavior.

The file "davinci_reg_map.csv" is an example showing the format of this file. A distinction is made between "registers" and "fields". A register is an 8-bit wide physical location in memory, identified by its address. Currently the address is limited to 8 bits as well, but this can be extended fairly easily. A field, on the other hand, is a logical collection of locations in one or more registers. For example, a manual enable bit for a hardware block might be called "ena_foo", which would be the name of a field which occupied bit location X in register at address Y. This also extends to multi-bit fields, for example a 36-bit field "filter_coeff_a" could span registers Z (bits [7:0]), Z+1 (bits [15:8])... etc.

In the underlying physical SPI only the concept of a register is used by the communication protocol, however by using the accompanying python classes which define both registers and fields, it is simple to write abstractions so that reading and writing multi-register fields appear as atomic operations.

### Register testing

The generated core also contains a state machine which allows the user to interact with on-chip registers through the SPI, which are otherwise typically clocked at too-high a rate. For a register to be visible through the SPI, the following generic code:

	reg [N-1:0] v_reg_foo;
    wire [N-1:] v_wire_foo;
    always @( posedge clk or negedge rstb ) begin : reg_foo
    	if ( !rstb )
        	v_reg_foo <= 0;
        else
        	v_reg_foo <= v_wire_foo;
    end

should be replaced with the following instantiated module:

	reg_slice #(.DWIDTH(N))
      inst_reg_foo(
	     .vo_regs		(v_reg_foo),
	     .io_success	(io_success),
	     .vio_tbus		(vio_tbus[N-1:0]),
	     .i_clk			(i_clk),
	     .i_rstb		(i_rstb),
	     .i_ena			(ena_reg_foo),
	     .vi_regs		(v_wire_foo),
	     .i_step		(reg_foo_step_clk),
	     .i_rd			(reg_foo_rd),
	     .i_wr			(reg_foo_wr));

There are three special wires: <name>_step_clk, <name>_rd, and <name>_wr. These three signals are handled by the generated SPI core. Through these, the SPI can perform the following operations:
- Freezing the clock to the register
- Single-stepping the register clock
- Replacing the register contents with an external value
- Reading the register contents while clock is running, transparently with respect to register functionality

A state machine (which requires an external oscillator to run) controls the step_clk/rd/wr signals as well all synchronization with the register clock. The test data is passed along vio_tbus, which can be shared with several reg_slice instances.

