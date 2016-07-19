/*
 * Copyright 2013-2016 Colin Weltin-Wu (colinww@gmail.com)
 * UC San Diego Integrated Signal Processing Group
 *
 * Licensed under GNU General Public License 3.0 or later. 
 * Some rights reserved. See LICENSE.
 * 
 * spi_top.v
 * Top-level module which encompasses the SPI map containing all the
 * control registers, and the digital register test interface.
 */
module spi_top(
	       // Outputs
	       o_ena_spi_clk, o_sdo, o_drvb,
	       // Map read-only inputs
	       // INSERT_RO
	       // Map write-only outputs
	       // INSERT_WO
	       // Map registered outputs
	       // INSERT_RW
	       // Register read signals
	       // INSERT_RD
	       // Register write signals
	       // INSERT_WR
	       // Inouts
	       io_success, vio_tbus,
	       // Inputs
	       i_rstb, i_clk_ext_osc, i_cs, i_sck, i_sdi);
  // Clock and reset
  input        i_rstb;
  input        i_clk_ext_osc;
  output       o_ena_spi_clk;
  // SPI interface
  output       o_sdo;
  output       o_drvb;
  input        i_cs;
  input        i_sck;
  input        i_sdi;
  // SPI map controls
  // Read-only inputs
  // INSERT_RO_DECLARATION
  // Write-only inputs
  // INSERT_WO_DECLARATION
  // Registered outputs
  // INSERT_RW_DECLARATION
  // Digital register interface
  inout        io_success;
  inout [36:0] vio_tbus;
  // Register read signals
  // INSERT_RD_DECLARATION
  // Register write signals
  // INSERT_WR_DECLARATION
  /*
   * SPI core
   */
  wire 	       spi_active;
  wire 	       rx_valid;
  wire [2:0]   v_byte_num;
  wire [7:0]   v_data_rx;
  wire [7:0]   v_data_tx;
  assign o_ena_spi_clk = spi_active;
  spi_core #(.BITS_IN_BYTE(8), .NBIT_BIT_CTR(3), .NBIT_BYTE_CTR(3))
    spi_interface(
		  // Outputs
		  .o_sdo		(o_sdo),
		  .o_drvb		(o_drvb),
		  .o_spi_active		(spi_active),
		  .o_rx_valid		(rx_valid),
		  .vo_byte_num		(v_byte_num),
		  .vo_data_rx		(v_data_rx),
		  // Inputs
		  .i_rstb		(i_rstb),
		  .i_cs			(i_cs),
		  .i_sck		(i_sck),
		  .i_sdi		(i_sdi),
		  .i_epol		(1'b1),
		  .i_cpol		(1'b1),
		  .i_cpha		(1'b0),
		  .vi_data_tx		(v_data_tx));
  /*
   * SPI map
   */
  spi_map
    spi_map_interface(
		      // Read-only inputs
		      // INSERT_RO_INST
		      // Write-only inputs
		      // INSERT_WO_INST
		      // Registered outputs
		      // INSERT_RW_INST
		      // Inouts
		      .vio_data_spi	(v_data_tx),
		      // Inputs
		      .i_rstb		(i_rstb),
		      .i_spi_active	(spi_active),
		      .vi_data_rx	(v_data_rx),
		      .i_rx_valid	(rx_valid),
		      .vi_byte_num	(v_byte_num));
  /*
   * SPI reg
   */
  spi_reg
    spi_reg_interface(// Register read signals
		      // INSERT_RD_INST
		      // Register write signals
		      // INSERT_WR_INST
		      // Inouts
		      .vio_data_regs	(v_data_tx),
		      .io_success	(io_success),
		      .vio_tbus		(vio_tbus),
		      // Inputs
		      .i_rstb		(i_rstb),
		      .i_clk_ext_osc	(i_clk_ext_osc),
		      .i_spi_active	(spi_active),
		      .vi_data_rx	(v_data_rx),
		      .i_rx_valid	(rx_valid),
		      .vi_byte_num	(v_byte_num));
  
// Local Variables:
// verilog-library-directories:("/home/colinww/git/doormat_dig/scripts/spi_generation/" "/home/colinww/git/doormat_dig/source/spi_dig/")
// verilog-library-extensions:(".v" ".h")
// End:
endmodule // spi_top
