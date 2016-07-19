/*
 * Copyright 2013-2016 Colin Weltin-Wu (colinww@gmail.com)
 * UC San Diego Integrated Signal Processing Group
 *
 * Licensed under GNU General Public License 3.0 or later. 
 * Some rights reserved. See LICENSE.
 * 
 * spi_core.v
 * 
 * General SPI interface
 * Services the 4 SPI modes defined as (CPOL,CPHA)
 * See: http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
 * 
 * CPOL = 0: Base value of clock is 0
 * -- CPHA = 0: SDI sampled on rising edge, SDO changes on falling edge
 * -- CPHA = 1: SDO changes on rising edge, SDI sampled on falling edge
 * CPOL = 1: Base value of clock is 1
 * -- CPHA = 0: SDI sampled on falling edge, SDO changes on rising edge
 * -- CPHA = 1: SDO changes on falling edge, SDI sampled on rising edge
 * 
 * The signals i_epol, i_cpol, i_cpha configure the chip select polarity,
 * the clock polarity, and the clock phase respectively.
 * 
 * BITS_IN_BYTE determines how many bits form a single byte (serial to perallel
 * conversion)
 * 
 * NBIT_BIT_CTR needs to be ceil(log2(BITS_IN_BYTE)) e.g. the counter needs
 * enough bits to be able to reach a count of BITS_IN_BYTE maximum count.
 * 
 * NBIT_BYTE_CTR needs to be able to count up to the maximum number of bytes
 * expected in one CS period. For example, if we are expecting up to 56 bytes,
 * NBIT_BYTE_CTR needs to be at least 6.
 * 
 * The output byte_num indicates the number of successfully received bytes
 * on the SPI interface for the current chip select period.
 * 
 * The output o_rx_valid goes high as the LSB of the current byte is latched;
 * therefore it would make sense to delay rx_valid slightly before sampling
 * o_data.
 * 
 * The data on vi_data is latched into the SPI interface on the opposite
 * polarity clock edge, when the bit counter is equal to the MSB.
 * 
 */

module spi_core (/*AUTOARG*/
   // Outputs
   o_sdo, o_drvb, o_spi_active, o_rx_valid, vo_byte_num, vo_data_rx,
   // Inputs
   i_rstb, i_cs, i_sck, i_sdi, i_epol, i_cpol, i_cpha, vi_data_tx
   );
   parameter BITS_IN_BYTE = 8;
   parameter NBIT_BIT_CTR = 3;
   parameter NBIT_BYTE_CTR = 3;
   
   // Global reset
   input 		      i_rstb;
   // spi wires
   input 		      i_cs; // Chip select
   input 		      i_sck; // SPI clock
   input 		      i_sdi; // SPI data input
   output 		      o_sdo; // SPI data output
   output 		      o_drvb; // Pad tristate driver 		      
   // spi configuration
   input 		      i_epol; // 1 = inverted chip select
   input 		      i_cpol; // SPI mode configuration
   input 		      i_cpha;
   // parallel interface
   output 		      o_spi_active; // High with chip select
   output 		      o_rx_valid; // When high, indicates valid data
   output [NBIT_BYTE_CTR-1:0] vo_byte_num; // Indicates number of bytes rx'd
   input [BITS_IN_BYTE-1:0]   vi_data_tx; // Data to be transmitted
   output [BITS_IN_BYTE-1:0]  vo_data_rx; // Data received
   // Registered outputs
   reg 			      o_rx_valid;
   reg [NBIT_BYTE_CTR-1:0]    vo_byte_num;

   /*
    * Generate an active high chip select (regardless of configuration)
    * sck_core is the scan clock which always latches in data on the rising
    * edge, and cycles it out on the falling edge. When cpha=1, it is shifted
    * by half a period.
    */
   wire 		      chip_select; 
   wire 		      sck_core; 
   assign chip_select = i_rstb && (i_epol ^ i_cs);
   assign o_spi_active = chip_select;
   assign sck_core = i_cpha ^ i_cpol ^ i_sck;
   // Bring pad driver low when chip select is enabled
   assign o_drvb = !chip_select;
   /*
    * TX shift register and state machine. The inactive edge of the sck_core
    * cycles the data output, and also changes the bit and byte counters.
    * 
    * Data is only shifted out AFTER the first byte is received. As an SPI
    * slave, it is not expected to send anything out during the first byte
    * 
    * Byte counter is also incremented here to be aligned with the bit counter
    */
   reg [BITS_IN_BYTE-1:0]     rv_tx;
   reg [NBIT_BIT_CTR-1:0]     rv_tx_ptr;
   always @( negedge sck_core or negedge chip_select ) begin : tx_fsm
      if ( !chip_select ) begin
	 // Set data pointer to the MSB of the shift reg
	 rv_tx_ptr <= $unsigned(BITS_IN_BYTE - 1);
	 // Reset the byte counter
	 vo_byte_num <= 0;
	 // Reset the data in the shift register
	 rv_tx <= 0;
      end else begin
	 // If the bit counter has reached 0, load in a new byte
	 if ( 0 == rv_tx_ptr ) begin
	    rv_tx <= vi_data_tx;
	    rv_tx_ptr <= $unsigned(BITS_IN_BYTE - 1);
	    vo_byte_num <= vo_byte_num + 1;
	 end else begin
	    rv_tx_ptr <= rv_tx_ptr - 1;
	 end
      end // else: !if( !chip_select )
   end // block: tx_fsm
   // Point the output data to the correct bit in the tx_reg (not true shift)
   assign o_sdo = rv_tx[rv_tx_ptr];
   /*
    * RX shift register state machine
    * The shift register to receive data has one fewer bit than the actual
    * data length since the LSB points directly to the i_sdi pin.
    * 
    * The rx_valid flag is triggered on the last rising edge of sck_core
    * within each byte. It is assumed the data at vo_data_rx will be latched
    * by listening blocks on this edge.
    */
   reg [BITS_IN_BYTE-2:0]     rv_rx;
   always @( posedge sck_core or negedge chip_select ) begin : rx_fsm
      if ( !chip_select ) begin
	 // Clear RX register
	 rv_rx <= 0;
         // Clear valid bit
         o_rx_valid <= 0;
      end else begin
	 // If this is the last bit, do not shift rx data, raise valid flag
	 if ( 0 == rv_tx_ptr ) begin
	    o_rx_valid <= 1;   
	 end else begin
	    // RX is not valid
	    o_rx_valid <= 0;
            // Begin clocking in data, MSB first
	    rv_rx[BITS_IN_BYTE-2:1] <= rv_rx[BITS_IN_BYTE-3:0];
	    rv_rx[0] <= i_sdi;
	 end // else: !if( 0 == rv_bit_ctr )
      end // else: !if( !chip_select )
   end // block: rx_fsm
   assign vo_data_rx = {rv_rx,i_sdi};
endmodule
